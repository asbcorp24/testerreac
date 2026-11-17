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
static constexpr size_t RING = 400;
struct Sample { uint32_t t; float a,g,j; };
static Sample ring[RING], trial[RING];
static size_t head=0, used=0, trialN=0;
static uint32_t cueLocal=0, pressLocal=0, cueSensor=0, trialId=0;
static uint32_t lastPacketMs=0,lastSeq=0,lost=0, lastSensorUs=0;
static bool synced=false,haveSeq=false,havePrev=false;
static float gravity=9.80665f, prevA=0;
static uint32_t prevT=0;
static float onset=-1,peakA=0,peakG=0,peakJ=0;
static int brakes=0,corrections=0;
static String quality="Ожидание измерений";
static portMUX_TYPE mux=portMUX_INITIALIZER_UNLOCKED;
static ImuPacket queue[64];
static volatile uint8_t wr=0,rd=0;
static volatile uint32_t dropped=0;

// ESP-NOW callback runs outside loop. Never do JSON/float work in callback.
static void receive(const uint8_t*,const uint8_t* data,int len) {
    if(len!=sizeof(ImuPacket))return;
    ImuPacket p;
    memcpy(&p,data,sizeof(p));
    if(p.magic!=MAGIC)return;
    portENTER_CRITICAL(&mux);
    uint8_t next=static_cast<uint8_t>((wr+1)%64);
    if(next!=rd){queue[wr]=p;wr=next;}
    else dropped++;
    portEXIT_CRITICAL(&mux);
}
void imuReceiverBegin(){
    if(esp_now_init()!=ESP_OK){Serial.println("ESP-NOW init failed");return;}
    esp_now_register_recv_cb(receive);
}
void imuMarkCue(uint32_t us){
    cueLocal=us;
    cueSensor=lastSensorUs; // coarse alignment from latest received packet
    synced=lastPacketMs && millis()-lastPacketMs<300;
    trialN=0;
}
void imuMarkPress(uint32_t us){
    pressLocal=us;
    trialN=0;onset=-1;peakA=peakG=peakJ=0;brakes=corrections=0;
    if(!synced || millis()-lastPacketMs>300){quality="Нет синхронизированных данных IMU";trialId++;return;}
    // Time alignment is approximate: last received sample at cue, network delay uncalibrated.
    uint32_t duration=static_cast<uint32_t>(pressLocal-cueLocal);
    size_t start=(head+RING-used)%RING;
    bool moving=false;int lowCount=0;int peakCount=0;
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
    quality=trialN<10?"Недостаточно отсчётов": "Предварительная оценка: нет точной синхронизации и компенсации ориентации";
    trialId++;
}
void imuReceiverPoll(){
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
           ",\"hz\":200}";
}
String imuTrialJson(){
    String s="{\"id\":"+String(trialId)+",\"quality\":\""+quality+
             "\",\"metrics\":{\"onset_ms\":"+(onset<0?String("null"):String(onset,1))+
             ",\"peak_a\":"+String(peakA,2)+",\"peak_g\":"+String(peakG,1)+
             ",\"peak_j\":"+String(peakJ,1)+",\"brakes\":"+String(brakes)+
             ",\"corrections\":"+String(corrections)+"},\"samples\":[";
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
