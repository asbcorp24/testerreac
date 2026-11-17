#include "imu_receiver.h"
#include <WiFi.h>
#include <esp_now.h>
#include <math.h>

// Packet v1; shared with wearable/src/main.cpp. All timestamps are sensor-local.
struct __attribute__((packed)) ImuPacket {
    uint32_t magic;
    uint32_t seq;
    uint32_t tUs;
    int16_t ax, ay, az, gx, gy, gz;
};
static constexpr uint32_t MAGIC = 0x52494D55;
static constexpr uint32_t SYNC_MAGIC = 0x53594E43;
struct __attribute__((packed)) SyncPacket {uint32_t magic,nonce,t0,t1,t2;};
static uint8_t peerMac[6]={};
static bool peerKnown=false;
static uint32_t syncNonce=0,syncT0=0,nextSyncMs=0;
static int32_t clockOffsetUs=0; // sensor clock minus central clock
static uint32_t bestRttUs=UINT32_MAX,lastSyncMs=0;
static volatile bool syncReplyPending=false;
static SyncPacket syncReply={};
static constexpr uint32_t MAX_SYNC_RTT_US=20000;

static constexpr size_t RING = 400;
struct Sample { uint32_t t; float a,g,j; };
static Sample ring[RING], trial[RING];
static size_t head=0, used=0, trialN=0;
static uint32_t cueLocal=0, pressLocal=0, cueSensor=0, trialId=0;
static bool trialSyncValid=false;
static uint32_t trialSyncRtt=0;
static uint32_t lastPacketMs=0,lastSeq=0,lost=0, lastSensorUs=0;
static bool synced=false,haveSeq=false,havePrev=false;
static float gravity=9.80665f, prevA=0;
static uint32_t prevT=0;
static float onset=-1,peakA=0,peakG=0,peakJ=0;
static int brakes=0,corrections=0;
static uint32_t validTrials=0;
static double meanPeakA=0,m2PeakA=0;
static String quality="Ожидание измерений";
static portMUX_TYPE mux=portMUX_INITIALIZER_UNLOCKED;
static ImuPacket queue[64];
static volatile uint8_t wr=0,rd=0;
static volatile uint32_t dropped=0;

// ESP-NOW callback runs outside loop. Never do JSON/float work in callback.
static void receive(const uint8_t* mac,const uint8_t* data,int len) {
    if(len==sizeof(SyncPacket)){
        SyncPacket p;memcpy(&p,data,sizeof(p));
        if(p.magic==SYNC_MAGIC && p.nonce==syncNonce){
            portENTER_CRITICAL(&mux);
            if(!syncReplyPending){syncReply=p;syncReplyPending=true;}
            portEXIT_CRITICAL(&mux);
        }
        return;
    }
    if(len!=sizeof(ImuPacket))return;
    ImuPacket p;
    memcpy(&p,data,sizeof(p));
    if(p.magic!=MAGIC)return;
    portENTER_CRITICAL(&mux);
    memcpy(peerMac,mac,6);peerKnown=true;
    uint8_t next=static_cast<uint8_t>((wr+1)%64);
    if(next!=rd){queue[wr]=p;wr=next;}
    else dropped++;
    portEXIT_CRITICAL(&mux);
}
void imuReceiverBegin(){
    if(esp_now_init()!=ESP_OK){Serial.println("ESP-NOW init failed");return;}
    esp_now_register_recv_cb(receive);
    esp_now_peer_info_t peer={};
    memset(peer.peer_addr,0xFF,6);
    peer.channel=1;
    peer.encrypt=false;
    if(esp_now_add_peer(&peer)!=ESP_OK)Serial.println("ESP-NOW broadcast peer failed");
}
void imuMarkCue(uint32_t us){
    cueLocal=us;
    trialSyncValid=lastSyncMs && millis()-lastSyncMs<5000 && bestRttUs<MAX_SYNC_RTT_US;
    trialSyncRtt=bestRttUs;
    cueSensor=static_cast<uint32_t>(static_cast<int64_t>(us)+clockOffsetUs);
    synced=trialSyncValid && lastPacketMs && millis()-lastPacketMs<300;
    trialN=0;
}
void imuMarkPress(uint32_t us){
    pressLocal=us;
    trialN=0;onset=-1;peakA=peakG=peakJ=0;brakes=corrections=0;
    if(!synced || millis()-lastPacketMs>300){quality="Нет синхронизированных данных IMU";trialId++;return;}
    // Align sensor timestamps to central cue via midpoint RTT clock estimate.
    uint32_t duration=static_cast<uint32_t>(pressLocal-cueLocal);
    size_t start=(head+RING-used)%RING;
    bool moving=false;int lowCount=0;
    float previous=0;
    for(size_t i=0;i<used;i++){
        Sample s=ring[(start+i)%RING];
        int32_t dt=static_cast<int32_t>(s.t-cueSensor);
        if(dt<0 || static_cast<uint32_t>(dt)>duration+80000)continue;
        s.t=static_cast<uint32_t>(dt)/1000;
        if(trialN<RING)trial[trialN++]=s;
        peakA=fmaxf(peakA,s.a);peakG=fmaxf(peakG,s.g);peakJ=fmaxf(peakJ,s.j);
        bool active=s.a>1.2f || s.g>35.0f;
        if(active && onset<0)onset=s.t;
        if(onset>=0){
            if(s.a<previous-0.8f && previous>2.0f)brakes++;
            if(!active){lowCount++;}
            else if(lowCount>=3 && moving){corrections++;lowCount=0;}
            if(active)moving=true;
        }
        previous=s.a;
    }
    if(trialN>=10){
        ++validTrials;
        double delta=peakA-meanPeakA;
        meanPeakA+=delta/validTrials;
        m2PeakA+=delta*(peakA-meanPeakA);
    }
    quality=trialN<10?"Недостаточно отсчётов": "Синхронизация оценена по RTT; начало стимула и ориентация IMU не откалиброваны";
    trialId++;
}
void imuReceiverPoll(){
    // Process completed exchange in loop, not Wi-Fi callback.
    SyncPacket reply={};bool got=false;
    portENTER_CRITICAL(&mux);
    if(syncReplyPending){reply=syncReply;syncReplyPending=false;got=true;}
    portEXIT_CRITICAL(&mux);
    if(got && reply.nonce==syncNonce && reply.t0==syncT0){
        uint32_t t3=micros();
        uint32_t roundtrip=static_cast<uint32_t>(t3-reply.t0);
        uint32_t remoteProcessing=static_cast<uint32_t>(reply.t2-reply.t1);
        if(roundtrip>=remoteProcessing){
            uint32_t rtt=roundtrip-remoteProcessing;
            // Signed 32-bit deltas are valid for intervals shorter than ~35 min.
            int32_t offset=(static_cast<int32_t>(reply.t1-reply.t0)+
                            static_cast<int32_t>(reply.t2-t3))/2;
            if(rtt<MAX_SYNC_RTT_US){
                if(!lastSyncMs || millis()-lastSyncMs>5000 || rtt<=bestRttUs+1000){
                    clockOffsetUs=offset;
                    bestRttUs=rtt;
                }
                lastSyncMs=millis();
            }
        }
    }
    if(peerKnown && static_cast<int32_t>(millis()-nextSyncMs)>=0){
        nextSyncMs=millis()+500;
        SyncPacket req={};req.magic=SYNC_MAGIC;req.nonce=++syncNonce;
        req.t0=micros();syncT0=req.t0;
        // Broadcast works without registering an individual peer.
        const uint8_t broadcast[6]={255,255,255,255,255,255};
        esp_now_send(broadcast,reinterpret_cast<const uint8_t*>(&req),sizeof(req));
    }
    for(int n=0;n<40;n++){
        ImuPacket p;
        portENTER_CRITICAL(&mux);
        bool available=rd!=wr;
        if(available){p=queue[rd];rd=static_cast<uint8_t>((rd+1)%64);}
        portEXIT_CRITICAL(&mux);
        if(!available)break;
        if(haveSeq && p.seq>lastSeq+1)lost+=p.seq-lastSeq-1;
        haveSeq=true;lastSeq=p.seq;lastPacketMs=millis();lastSensorUs=p.tUs;
        float ax=p.ax/16384.0f*9.80665f,ay=p.ay/16384.0f*9.80665f,az=p.az/16384.0f*9.80665f;
        float magnitude=sqrtf(ax*ax+ay*ay+az*az);
        // Simple magnitude residual; not orientation-corrected linear acceleration.
        float a=fabsf(magnitude-gravity);
        float g=sqrtf(float(p.gx)*p.gx+float(p.gy)*p.gy+float(p.gz)*p.gz)/131.0f;
        float j=havePrev?fabsf(a-prevA)/(static_cast<uint32_t>(p.tUs-prevT)*1e-6f+1e-6f):0;
        if(j>500)j=500; // clip extreme packet jitter
        ring[head]={p.tUs,a,g,j};head=(head+1)%RING;if(used<RING)used++;
        prevA=a;prevT=p.tUs;havePrev=true;
    }
}
String imuStatusJson(){
    uint32_t age=lastPacketMs?millis()-lastPacketMs:999999;
    return "{\"connected\":"+String(age<1000?"true":"false")+
           ",\"age_ms\":"+String(age)+",\"lost\":"+String(lost+dropped)+
           ",\"hz\":200,\"sync_valid\":"+String(lastSyncMs && millis()-lastSyncMs<5000?"true":"false")+
           ",\"sync_rtt_us\":"+(lastSyncMs?String(bestRttUs):String("null"))+"}";
}
String imuTrialJson(){
    String s="{\"id\":"+String(trialId)+
             ",\"sync_valid\":"+String(trialSyncValid?"true":"false")+
             ",\"sync_rtt_us\":"+(trialSyncValid?String(trialSyncRtt):String("null"))+",\"quality\":\""+quality+
             "\",\"metrics\":{\"onset_ms\":"+(onset<0?String("null"):String(onset,1))+
             ",\"peak_a\":"+String(peakA,2)+",\"peak_g\":"+String(peakG,1)+
             ",\"peak_j\":"+String(peakJ,1)+",\"brakes\":"+String(brakes)+
             ",\"corrections\":"+String(corrections)+",\"stability_sd\":"+(validTrials>1?String(sqrt(m2PeakA/(validTrials-1)),2):String("null"))+",\"stability_n\":"+String(validTrials)+"},\"samples\":[";
    // Limit JSON payload and RAM use; plot uses up to 200 decimated samples.
    size_t step=trialN>200?(trialN+199)/200:1;
    for(size_t i=0;i<trialN;i+=step){
        if(i)s+=",";
        s+="{\"t\":"+String(trial[i].t)+",\"a\":"+String(trial[i].a,2)+
           ",\"g\":"+String(trial[i].g,1)+",\"j\":"+String(trial[i].j,1)+"}";
    }
    s+="]}";
    return s;
}
