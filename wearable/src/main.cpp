#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <Wire.h>

// ESP32-C3 + MPU6050 at I2C 0x68; SDA GPIO4, SCL GPIO5.
// Use 3.3 V for MPU6050 logic and power; check breakout voltage limits.
static constexpr int SDA_PIN=4,SCL_PIN=5;
static constexpr uint8_t MPU=0x68;
static constexpr uint32_t MAGIC=0x52494D55;
static constexpr uint32_t SYNC_MAGIC=0x53594E43;
struct __attribute__((packed)) SyncPacket {uint32_t magic,nonce,t0,t1,t2;};
static volatile bool syncWaiting=false;
static SyncPacket incomingSync={};
static portMUX_TYPE syncMux=portMUX_INITIALIZER_UNLOCKED;
static void onReceive(const uint8_t*,const uint8_t* data,int len){
    if(len!=sizeof(SyncPacket))return;
    SyncPacket req;memcpy(&req,data,sizeof(req));
    if(req.magic!=SYNC_MAGIC)return;
    req.t1=micros();
    portENTER_CRITICAL(&syncMux);
    incomingSync=req;syncWaiting=true;
    portEXIT_CRITICAL(&syncMux);
}
struct __attribute__((packed)) ImuPacket {
    uint32_t magic,seq,tUs;
    int16_t ax,ay,az,gx,gy,gz;
};
static uint32_t seq=0,nextUs=0;
static uint8_t broadcast[]={0xff,0xff,0xff,0xff,0xff,0xff};
bool writeReg(uint8_t reg,uint8_t value){
    Wire.beginTransmission(MPU);Wire.write(reg);Wire.write(value);
    return Wire.endTransmission()==0;
}
bool readImu(ImuPacket &p){
    Wire.beginTransmission(MPU);Wire.write(0x3B);
    if(Wire.endTransmission(false)!=0)return false;
    if(Wire.requestFrom(MPU,(uint8_t)14,(uint8_t)true)!=14)return false;
    auto word=[]()->int16_t{uint16_t hi=Wire.read(),lo=Wire.read();return (int16_t)((hi<<8)|lo);};
    p.ax=word();p.ay=word();p.az=word();word(); // skip temperature
    p.gx=word();p.gy=word();p.gz=word();
    return true;
}
void setup(){
    Serial.begin(115200);
    Wire.begin(SDA_PIN,SCL_PIN);Wire.setClock(400000);
    if(!writeReg(0x6B,0x01))Serial.println("MPU6050 not responding");
    writeReg(0x1C,0x00); // ±2 g: 16384 LSB/g
    writeReg(0x1B,0x00); // ±250 dps: 131 LSB/(°/s)
    writeReg(0x1A,0x03); // DLPF
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    // Central AP defaults to channel 1; both ESP-NOW peers must use same channel.
    esp_wifi_set_channel(1,WIFI_SECOND_CHAN_NONE);
    if(esp_now_init()!=ESP_OK){Serial.println("ESP-NOW failed");return;}
    esp_now_peer_info_t peer={};
    memcpy(peer.peer_addr,broadcast,6);peer.channel=1;peer.encrypt=false;
    esp_now_add_peer(&peer);
    esp_now_register_recv_cb(onReceive);
    nextUs=micros();
}
void loop(){
    SyncPacket reply={};bool pending=false;
    portENTER_CRITICAL(&syncMux);
    if(syncWaiting){reply=incomingSync;syncWaiting=false;pending=true;}
    portEXIT_CRITICAL(&syncMux);
    if(pending){
        reply.t2=micros();
        esp_now_send(broadcast,reinterpret_cast<const uint8_t*>(&reply),sizeof(reply));
    }
    if((int32_t)(micros()-nextUs)<0)return;
    nextUs+=5000; // 200 Hz
    ImuPacket p={};p.magic=MAGIC;p.seq=seq++;p.tUs=micros();
    if(readImu(p))esp_now_send(broadcast,(uint8_t*)&p,sizeof(p));
}
