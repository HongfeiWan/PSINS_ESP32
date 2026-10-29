#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <esp_timer.h>
#include <esp_system.h>
#include "attitude.h"
#include "navigation.h"
#include "clock_sync.h"
#include "logger.h"
#include "field_control.h"
#include <Preferences.h>
#include "web_assets.h"

constexpr char version[]="3.0.4";
constexpr int FIELD_BUTTON=0;
constexpr int LED=48,CS=10,IMU_SCK=12,IMU_MOSI=11,IMU_MISO=13,INT1=14;
constexpr int SD_CS=35,SD_MOSI=36,SD_SCK=37,SD_MISO=38,SD_DETECT=47;
constexpr int GNSS_RX=42,GNSS_TX=41,GNSS_RESET=39,GNSS_PPS=40;
constexpr uint32_t ringSize=1024, maxBatch=64;
SPIClass imuSpi(FSPI),sdSpi(HSPI);
HardwareSerial gnss(1);
WebServer server(80);
TaskHandle_t imuTask=nullptr;
portMUX_TYPE dataMux=portMUX_INITIALIZER_UNLOCKED,irqMux=portMUX_INITIALIZER_UNLOCKED;
volatile uint32_t interrupts=0,pps=0;
volatile int64_t irqTime=0;
bool imuReady=false,mathOk=false,mdnsStarted=false;
uint8_t who=0;
String deviceName,bootId;

struct Sample {uint32_t seq;uint64_t t;uint32_t dt,flags;float w[3],f[3];};
struct State {
  Sample last={};float q[4]={1,0,0,0},euler[3]={},bias[3]={};
  float period=1000000.0f/104,still=0;uint32_t missed=0,clockFallbacks=0,calibrationId=0,saturated=0;
  bool calibrated=false;
};
State state;
Sample ring[ringSize],batch[maxBatch];
bool calibrationRequest=false;
struct GnssState {GnssFix fix;ClockSync clock;uint32_t bytes=0,valid=0,bad=0,invalid=0,epochs=0;};
GnssState gnssState;NavOutput navState;QueueHandle_t navQueue;
bool simulation=false;volatile bool desiredSimulation=false;uint32_t navDrops=0,navMaxUs=0,navStackLeft=UINT32_MAX,navControl=1;
bool headingManual=false,headingTrack=false;double headingDegrees=0;
uint8_t savedHeadingMode=0;double savedHeadingDegrees=0;
FieldStopButton fieldButton;
volatile uint64_t ppsRing[16]={};volatile uint32_t ppsWrite=0;

LogContext getLogContext(){
  LogContext c;portENTER_CRITICAL(&dataMux);c.calibrated=state.calibrated;c.simulation=simulation;c.calibrationId=state.calibrationId;c.headingId=navControl;c.headingMode=headingManual?1:headingTrack?2:0;c.headingDegrees=headingDegrees;for(int i=0;i<3;i++)c.bias[i]=state.bias[i];portEXIT_CRITICAL(&dataMux);return c;
}
const char *headingName(uint8_t mode){return mode==1?"manual":mode==2?"forward":"unknown";}
const char *resetReasonName(){
  switch(esp_reset_reason()){
    case ESP_RST_POWERON:return "power_on";case ESP_RST_SW:return "software";
    case ESP_RST_PANIC:return "panic";case ESP_RST_TASK_WDT:return "task_watchdog";
    case ESP_RST_INT_WDT:return "interrupt_watchdog";case ESP_RST_BROWNOUT:return "brownout";
    case ESP_RST_EXT:return "external";default:return "other";
  }
}
void loadFieldHeading(){
  Preferences settings;if(!settings.begin("insmini",true))return;
  String value=settings.getString("field_heading","");settings.end();int split=value.indexOf(':');double degrees=0;
  if(split<0||!GnssParser::number(value.substring(split+1).c_str(),degrees)||degrees<0||degrees>=360)return;
  String mode=value.substring(0,split);savedHeadingMode=mode=="manual"?1:mode=="forward"?2:0;
  savedHeadingDegrees=savedHeadingMode?degrees:0;headingManual=savedHeadingMode==1;headingTrack=savedHeadingMode==2;headingDegrees=savedHeadingDegrees;
}
void publish(Sample sample,const float *q,const float *e,const float *bias,bool calibrated,float still,uint32_t calId,float period,uint32_t missed,uint32_t fallback,uint32_t saturated,float temperature){
  NavInput input;ClockSync clock;bool sim;
  portENTER_CRITICAL(&dataMux);
  sample.seq=state.last.seq+1;ring[sample.seq%ringSize]=sample;state.last=sample;state.period=period;state.missed=missed;state.clockFallbacks=fallback;state.saturated=saturated;
  state.calibrated=calibrated;state.still=still;state.calibrationId=calId;sim=simulation;clock=gnssState.clock;
  for(int i=0;i<4;i++)state.q[i]=input.q[i]=q[i];
  for(int i=0;i<3;i++){state.euler[i]=e[i];state.bias[i]=input.bias[i]=bias[i];input.w[i]=sample.w[i];input.f[i]=sample.f[i];}
  portEXIT_CRITICAL(&dataMux);
  input.seq=sample.seq;input.mono=sample.t;input.dtUs=sample.dt;input.flags=sample.flags;input.calibrated=calibrated;
  if(navQueue&&xQueueSend(navQueue,&input,0)!=pdTRUE){portENTER_CRITICAL(&dataMux);navDrops++;portEXIT_CRITICAL(&dataMux);}
  if(calibrated){LogRecord r;r.type=1;r.mode=sim;r.flags=(min(sample.dt,16383U)<<2)|sample.flags;r.seq=sample.seq;r.mono=sample.t;r.utc=clock.utc(sample.t,esp_timer_get_time());
    for(int i=0;i<3;i++){r.v[i]=sample.w[i];r.v[i+3]=sample.f[i];}r.v[6]=temperature;for(int i=0;i<4;i++)r.v[i+7]=q[i];loggerPush(r);}
}
void navigate(void*){
  NavInput input;uint32_t control=0;int64_t epoch=-1;
  for(;;){
    if(xQueueReceive(navQueue,&input,pdMS_TO_TICKS(100))!=pdTRUE)continue;
    GnssFix fix;uint32_t nextControl;bool manual,track;double degrees;
    portENTER_CRITICAL(&dataMux);fix=gnssState.fix;nextControl=navControl;manual=headingManual;track=headingTrack;degrees=headingDegrees;portEXIT_CRITICAL(&dataMux);
    if(control!=nextControl){navigationClearHeading();if(manual||track)navigationHeading(degrees,track);control=nextControl;epoch=-1;}
    uint64_t start=esp_timer_get_time();if(fix.valid&&fix.epochMono&&fix.utcUs!=epoch){navigationGnss(fix);epoch=fix.utcUs;}
    navigationUpdate(input);NavOutput n=navigationOutput();uint32_t used=esp_timer_get_time()-start;bool sim;
    portENTER_CRITICAL(&dataMux);navState=n;navMaxUs=max(navMaxUs,used);navStackLeft=min(navStackLeft,uint32_t(uxTaskGetStackHighWaterMark(nullptr)));sim=simulation;portEXIT_CRITICAL(&dataMux);
    if(n.updates&&input.seq%10==0){LogRecord r;r.type=3;r.mode=sim;r.flags=n.status;r.seq=input.seq;r.mono=n.mono;
      ClockSync c;portENTER_CRITICAL(&dataMux);c=gnssState.clock;portEXIT_CRITICAL(&dataMux);r.utc=c.utc(n.mono,esp_timer_get_time());
      double values[]={n.lat,n.lon,n.h,n.ve,n.vn,n.vu,n.pitch,n.roll,n.yaw,n.pSigma,n.vSigma};memcpy(r.v,values,sizeof(values));loggerPush(r);}
  }
}
void IRAM_ATTR imuIsr(){
  portENTER_CRITICAL_ISR(&irqMux);irqTime=esp_timer_get_time();interrupts++;portEXIT_CRITICAL_ISR(&irqMux);
  if(imuTask){BaseType_t wake=pdFALSE;vTaskNotifyGiveFromISR(imuTask,&wake);if(wake)portYIELD_FROM_ISR();}
}
void IRAM_ATTR ppsIsr(){portENTER_CRITICAL_ISR(&irqMux);ppsRing[ppsWrite%16]=esp_timer_get_time();ppsWrite++;pps++;portEXIT_CRITICAL_ISR(&irqMux);}
uint8_t readReg(uint8_t reg){
  imuSpi.beginTransaction(SPISettings(1000000,MSBFIRST,SPI_MODE3));digitalWrite(CS,LOW);
  imuSpi.transfer(reg|0x80);uint8_t v=imuSpi.transfer(0);digitalWrite(CS,HIGH);imuSpi.endTransaction();return v;
}
void writeReg(uint8_t reg,uint8_t value){
  imuSpi.beginTransaction(SPISettings(1000000,MSBFIRST,SPI_MODE3));digitalWrite(CS,LOW);
  imuSpi.transfer(reg&0x7f);imuSpi.transfer(value);digitalWrite(CS,HIGH);imuSpi.endTransaction();
}
bool startImu(){
  who=readReg(0x0f);if(who!=0x6a)return false;
  writeReg(0x12,1);uint32_t start=millis();
  while(readReg(0x12)&1){if(millis()-start>200)return false;delay(1);}
  writeReg(0x12,0x44);writeReg(0x10,0x48);writeReg(0x11,0x48);writeReg(0x0d,3);
  return readReg(0x12)==0x44 && readReg(0x10)==0x48 && readReg(0x11)==0x48;
}
bool mathSelftest(){
  Attitude a;a.setEuler(.31f,-.42f,.83f);float e[3];a.euler(e);
  if(fabsf(e[0]-.31f)>1e-5f || fabsf(e[1]+.42f)>1e-5f || fabsf(e[2]-.83f)>1e-5f)return false;
  float w[3]={1.3f*Attitude::rad,-4.4f*Attitude::rad,0},f[3]={0,0,Attitude::gravity};
  a.resetCalibration();for(int i=0;i<700;i++)a.update(w,f,.01f);
  if(!a.calibrated || fabsf(a.bias[1]-w[1])>1e-5f)return false;
  w[2]=Attitude::rad*90;for(int i=0;i<100;i++)a.update(w,f,.01f);
  a.euler(e);return fabsf(e[2]-1.57079632679f)<.002f;
}
void sampling(void*){
  Attitude a;uint64_t previous=0;uint32_t seq=0,missed=0,fallback=0,calId=0,saturated=0;uint8_t warmup=0;
  float period=1000000.0f/104;bool wasSimulation=false;float temperature=25;
  for(;;){
    ulTaskNotifyTake(pdTRUE,pdMS_TO_TICKS(20));
    if((readReg(0x1e)&3)!=3)continue;
    uint64_t t;portENTER_CRITICAL(&irqMux);t=irqTime;portEXIT_CRITICAL(&irqMux);
    if(t<=previous){t=esp_timer_get_time();fallback++;}
    uint8_t data[12];imuSpi.beginTransaction(SPISettings(1000000,MSBFIRST,SPI_MODE3));digitalWrite(CS,LOW);
    imuSpi.transfer(0xa2);for(auto &b:data)b=imuSpi.transfer(0);digitalWrite(CS,HIGH);imuSpi.endTransaction();
    bool sim;portENTER_CRITICAL(&dataMux);sim=simulation;portEXIT_CRITICAL(&dataMux);
    if(sim){wasSimulation=true;continue;}
    if(wasSimulation){a.resetCalibration();previous=t;warmup=0;wasSimulation=false;portENTER_CRITICAL(&dataMux);calId=state.calibrationId;portEXIT_CRITICAL(&dataMux);}
    // The first ready level can predate gyro startup; prime two fresh DRDY edges.
    if(warmup<2){previous=t;warmup++;continue;}
    Sample s={};s.seq=++seq;s.t=t;s.dt=uint32_t(t-previous);previous=t;
    if(s.dt>period*1.7f){s.flags=1;missed+=uint32_t(s.dt/period+.5f)-1;}
    else if(s.dt>4000 && s.dt<20000)period=.98f*period+.02f*s.dt;
    for(int i=0;i<3;i++){
      int16_t rawW=int16_t(uint16_t(data[2*i])|(uint16_t(data[2*i+1])<<8));
      int16_t rawF=int16_t(uint16_t(data[2*i+6])|(uint16_t(data[2*i+7])<<8));
      // ST sensitivities: +/-1000 dps -> 35 mdps/LSB, +/-4 g -> 0.122 mg/LSB.
      s.w[i]=rawW*.035f*Attitude::rad;s.f[i]=rawF*.000122f*Attitude::gravity;
      if(rawW>=32760 || rawW<=-32760 || rawF>=32760 || rawF<=-32760)s.flags|=2;
    }
    if(s.flags&2)saturated++;
    bool requested;portENTER_CRITICAL(&dataMux);requested=calibrationRequest;calibrationRequest=false;portEXIT_CRITICAL(&dataMux);
    if(requested)a.resetCalibration();
    bool before=a.calibrated;a.update(s.w,s.f,s.dt*1e-6f);if(!before && a.calibrated)calId++;
    float e[3];a.euler(e);
    if(s.seq%104==0){int16_t temp=int16_t(uint16_t(readReg(0x20))|(uint16_t(readReg(0x21))<<8));temperature=25+temp/256.0f;}
    publish(s,a.q,e,a.bias,a.calibrated,a.stillSeconds,calId,period,missed,fallback,saturated,temperature);
  }
}
void receiveGnss(void*){
  GnssParser parser;ClockSync clock;GnssFix lastFix;uint32_t bytes=0,readPps=0;char line[160];size_t length=0;uint64_t first=0;
  bool sim=false;uint64_t base=0;uint32_t index=0;int simSecond=-1;GnssFix truth;
  auto report=[&](){GnssState g;g.fix=lastFix;g.clock=clock;g.bytes=bytes;g.valid=parser.valid;g.bad=parser.bad;g.invalid=parser.invalid;g.epochs=parser.epochs;portENTER_CRITICAL(&dataMux);gnssState=g;portEXIT_CRITICAL(&dataMux);};
  auto pulse=[&](uint64_t t){clock.pps(t);LogRecord r;r.type=4;r.mode=sim;r.seq=clock.ppsCount;r.mono=t;r.utc=clock.utc(t,t);r.v[0]=clock.quality(t);r.v[1]=clock.driftPpm;loggerPush(r);};
  auto sentence=[&](const char *text,uint64_t start,uint64_t end){
    size_t n=strlen(text);for(size_t offset=0;offset<n;offset+=88){LogRecord r;r.type=5;r.mode=sim;r.seq=parser.valid+parser.bad;r.mono=start;r.flags=min(size_t(88),n-offset);if(offset+88<n)r.flags|=0x100;memcpy(r.v,text+offset,r.flags&255);loggerPush(r);}
    if(parser.parse(text,start,end)){
      GnssFix fix=parser.fix;clock.pair(fix.utcUs,fix.rxStart,fix.rxEnd,clock.lastPps);fix.epochMono=clock.quality(end)==2?clock.lastPps:0;lastFix=fix;
      LogRecord r;r.type=2;r.mode=sim;r.flags=(fix.altitude?1:0)|(fix.velocity?2:0);r.seq=parser.epochs;r.mono=end;r.utc=fix.utcUs;
      double values[]={fix.lat,fix.lon,fix.height,fix.ve,fix.vn,fix.hdop,double(fix.quality),double(fix.satellites),double(fix.rxStart),double(fix.rxEnd),double(fix.epochMono)};memcpy(r.v,values,sizeof(values));loggerPush(r);
    }else if(!parser.fix.valid){lastFix.valid=false;lastFix.epochMono=0;}
  };
  auto body=[&](const char *text,uint64_t start,uint64_t end,bool corrupt){unsigned sum=0;for(const char *p=text;*p;p++)sum^=(uint8_t)*p;char out[160];snprintf(out,sizeof(out),"$%s*%02X",text,sum^(corrupt?1:0));sentence(out,start,end);};
  for(;;){
    if(desiredSimulation!=sim){
      sim=desiredSimulation;parser=GnssParser();clock=ClockSync();lastFix=GnssFix();length=0;base=esp_timer_get_time()+20000;index=0;simSecond=-1;
      portENTER_CRITICAL(&irqMux);readPps=ppsWrite;portEXIT_CRITICAL(&irqMux);
      portENTER_CRITICAL(&dataMux);simulation=sim;state.calibrated=false;state.calibrationId++;navControl++;if(sim){headingManual=true;headingTrack=false;headingDegrees=0;}else{headingManual=savedHeadingMode==1;headingTrack=savedHeadingMode==2;headingDegrees=savedHeadingDegrees;}portEXIT_CRITICAL(&dataMux);
      report();Serial.printf("MODE %s\n",sim?"SIMULATION":"REAL");
    }
    uint64_t now=esp_timer_get_time();
    if(sim){
      while(gnss.available()){gnss.read();bytes++;}
      if(now>=base+uint64_t(index)*10000){
        double t=index*.01;int sec=index/100;uint64_t mono=base+uint64_t(index)*10000;NavInput input;navigationSynthetic(t,input,truth);
        bool outage=sec>=30&&sec<42;
        if(sec!=simSecond){simSecond=sec;if(!outage)pulse(base+uint64_t(sec)*1000000+(sec==70?100000:0));}
        if(index%100==22&&!outage){
          NavInput temp;GnssFix epoch;navigationSynthetic(sec,temp,epoch);if(sec==55)epoch.lon+=.001;
          double lat=fabs(epoch.lat/Attitude::rad),lon=fabs(epoch.lon/Attitude::rad);double la=int(lat)*100+(lat-int(lat))*60,lo=int(lon)*100+(lon-int(lon))*60;
          char b[140];uint64_t start=base+uint64_t(sec)*1000000+100000,end=base+uint64_t(sec)*1000000+220000;
          snprintf(b,sizeof(b),"GNRMC,12%02d%02d.000,A,%09.5f,N,%010.5f,E,%.4f,0.00,010126,,,A",sec/60,sec%60,la,lo,epoch.speed/.5144444444444444);body(b,start,end,sec==50);
          snprintf(b,sizeof(b),"GNGGA,12%02d%02d.000,%09.5f,N,%010.5f,E,1,10,0.8,100.0,M,25.0,M,,",sec/60,sec%60,la,lo);body(b,start,end,false);
        }
        Sample sample={};sample.t=mono;sample.dt=10000;memcpy(sample.w,input.w,sizeof(sample.w));memcpy(sample.f,input.f,sizeof(sample.f));float e[3]={},bias[3]={};
        uint32_t id;portENTER_CRITICAL(&dataMux);id=state.calibrationId;portEXIT_CRITICAL(&dataMux);report();publish(sample,input.q,e,bias,true,5,id,10000,0,0,0,25);
        if(index%100==0){LogRecord r;r.type=6;r.mode=1;r.seq=index/100;r.mono=mono;r.utc=1767268800000000LL+(index/100)*1000000LL;r.v[0]=truth.lat;r.v[1]=truth.lon;r.v[2]=truth.height;r.v[3]=truth.ve;r.v[4]=truth.vn;loggerPush(r);}
        index++;
      }
    }else{
      uint32_t written;portENTER_CRITICAL(&irqMux);written=ppsWrite;portEXIT_CRITICAL(&irqMux);
      if(written-readPps>16){readPps=written-16;clock.locked=false;clock.rejected++;}
      while(readPps<written){uint64_t t;portENTER_CRITICAL(&irqMux);t=ppsRing[(readPps++)%16];portEXIT_CRITICAL(&irqMux);pulse(t);}
      while(gnss.available()){
        char c=gnss.read();bytes++;uint64_t at=esp_timer_get_time();
        if(c=='$'){length=0;line[length++]=c;first=at;}
        else if(c=='\n'){if(length){line[length]=0;sentence(line,first,at);}length=0;}
        else if(c!='\r'&&length){if(length<sizeof(line)-1)line[length++]=c;else{length=0;parser.bad++;}}
      }
      report();
    }
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}
String quoteJson(const String &s){
  String r="\"";for(size_t i=0;i<s.length();i++){char c=s[i];if(c=='"' || c=='\\')r+='\\';if(uint8_t(c)<32)r+=' ';else r+=c;}return r+'"';
}
bool parseSince(uint32_t &since){
  if(!server.hasArg("since"))return false;String s=server.arg("since");if(!s.length() || s.length()>10)return false;
  uint64_t n=0;for(size_t i=0;i<s.length();i++){if(s[i]<'0' || s[i]>'9')return false;n=n*10+s[i]-'0';}
  if(n>UINT32_MAX)return false;since=uint32_t(n);return true;
}
void telemetry(){
  uint32_t since=0;bool hasSince=parseSince(since);
  if(server.hasArg("since") && !hasSince){server.send(400,"application/json","{\"error\":\"invalid since\"}");return;}
  State s;GnssState gs;NavOutput ns;bool sim;uint32_t nd,nm,nstack;uint32_t rows=0,dropped=0,oldest=0,next=0;
  portENTER_CRITICAL(&dataMux);s=state;gs=gnssState;ns=navState;sim=simulation;nd=navDrops;nm=navMaxUs;nstack=navStackLeft;
  if(s.last.seq){
    oldest=s.last.seq>=ringSize?s.last.seq-ringSize+1:1;
    uint32_t first=hasSince?since+1:s.last.seq;
    if(first<oldest){dropped=oldest-first;first=oldest;}
    if(first<=s.last.seq){rows=min(maxBatch,s.last.seq-first+1);for(uint32_t i=0;i<rows;i++)batch[i]=ring[(first+i)%ringSize];}
    next=rows?batch[rows-1].seq:s.last.seq;
  }
  portEXIT_CRITICAL(&dataMux);
  String out;out.reserve(1800+rows*150);char b[1600];uint64_t now=esp_timer_get_time();
  out="{\"protocol\":\"insmini/1\",\"firmware\":\"";out+=version;out+="\",\"device\":"+quoteJson(deviceName)+",\"boot\":"+quoteJson(bootId);
  snprintf(b,sizeof(b),",\"reset_reason\":%d,\"reset_reason_name\":\"%s\"",int(esp_reset_reason()),resetReasonName());out+=b;
  snprintf(b,sizeof(b),",\"uptime_ms\":%lu,\"heap\":%u,\"imu\":{\"ready\":%s,\"who\":%u,\"selftest\":%s,\"seq\":%lu,\"sample_hz\":%.3f,\"age_ms\":%.1f,\"missed\":%lu,\"clock_fallbacks\":%lu,\"irq\":%lu,\"q\":[%.8f,%.8f,%.8f,%.8f],\"att_deg\":[%.4f,%.4f,%.4f],\"gyro_rad_s\":[%.8g,%.8g,%.8g],\"force_m_s2\":[%.8g,%.8g,%.8g],\"calibrated\":%s,\"calibration_id\":%lu,\"still_s\":%.2f,\"bias_rad_s\":[%.9g,%.9g,%.9g]},",
    millis(),ESP.getFreeHeap(),imuReady?"true":"false",who,mathOk?"true":"false",s.last.seq,1e6f/s.period,s.last.t?(now-s.last.t)*.001f:-1.0f,s.missed,s.clockFallbacks,uint32_t(interrupts),
    s.q[0],s.q[1],s.q[2],s.q[3],s.euler[0]/Attitude::rad,s.euler[1]/Attitude::rad,s.euler[2]/Attitude::rad,
    s.last.w[0],s.last.w[1],s.last.w[2],s.last.f[0],s.last.f[1],s.last.f[2],s.calibrated?"true":"false",s.calibrationId,s.still,s.bias[0],s.bias[1],s.bias[2]);out+=b;
  bool gnssFresh=gs.fix.valid && now>=gs.fix.rxEnd && now-gs.fix.rxEnd<2500000;
  snprintf(b,sizeof(b),"\"data_mode\":\"%s\",\"gnss\":{\"bytes\":%lu,\"valid\":%lu,\"bad\":%lu,\"fix\":%d,\"rmc\":\"%c\",\"satellites\":%d,\"pps\":%lu,\"fresh\":%s,\"lat_deg\":%.9f,\"lon_deg\":%.9f,\"height_m\":%.3f,\"altitude_valid\":%s,\"hdop\":%.2f},\"sd_present\":%s,\"wifi\":{\"connected\":%s,\"rssi\":%d,\"ip\":",
    sim?"simulation":"real",gs.bytes,gs.valid,gs.bad,gnssFresh?gs.fix.quality:0,gnssFresh?'A':'V',gs.fix.satellites,gs.clock.ppsCount,gnssFresh?"true":"false",gs.fix.lat/Attitude::rad,gs.fix.lon/Attitude::rad,gs.fix.height,gs.fix.altitude?"true":"false",gs.fix.hdop,digitalRead(SD_DETECT)==LOW?"true":"false",WiFi.status()==WL_CONNECTED?"true":"false",WiFi.RSSI());out+=b;
  out+=quoteJson(WiFi.localIP().toString())+",\"ssid\":"+quoteJson(WiFi.SSID())+"},";
  LogState ls=loggerState();
  snprintf(b,sizeof(b),"\"clock\":{\"quality\":%d,\"utc_us\":%lld,\"pairs\":%lu,\"rejected\":%lu,\"drift_ppm\":%.3f,\"latency_ms\":%.3f},\"navigation\":{\"status\":%d,\"heading_known\":%s,\"updates\":%lu,\"accepted\":%lu,\"rejected\":%lu,\"resets\":%lu,\"queue_drops\":%lu,\"max_compute_us\":%lu,\"stack_left_bytes\":%lu,\"lat_deg\":%.9f,\"lon_deg\":%.9f,\"height_m\":%.3f,\"velocity_enu\":[%.5f,%.5f,%.5f],\"att_deg\":[%.4f,%.4f,%.4f],\"q\":[%.8f,%.8f,%.8f,%.8f],\"position_sigma_m\":%.3f,\"velocity_sigma_m_s\":%.3f},",
    gs.clock.quality(now),gs.clock.utc(s.last.t,now),gs.clock.paired,gs.clock.rejected,gs.clock.driftPpm,gs.clock.latencyMs,ns.status,ns.headingKnown?"true":"false",ns.updates,ns.gnssAccepted,ns.gnssRejected,ns.resets,nd,nm,nstack,ns.lat/Attitude::rad,ns.lon/Attitude::rad,ns.h,ns.ve,ns.vn,ns.vu,ns.pitch/Attitude::rad,ns.roll/Attitude::rad,ns.yaw/Attitude::rad,ns.q[0],ns.q[1],ns.q[2],ns.q[3],ns.pSigma,ns.vSigma);out+=b;
  snprintf(b,sizeof(b),"\"storage\":{\"mounted\":%s,\"active\":%s,\"enabled\":%s,\"records\":%lu,\"bytes\":%llu,\"drops\":%lu,\"errors\":%lu,\"queued\":%lu,\"queue_bytes\":%lu,\"queue_capacity\":65536,\"max_queue_bytes\":%lu,\"max_io_ms\":%.3f,\"session\":",ls.mounted?"true":"false",ls.active?"true":"false",ls.enabled?"true":"false",ls.records,ls.bytes,ls.drops,ls.errors,ls.queued,ls.queueBytes,ls.maxQueueBytes,ls.maxIoUs*.001);out+=b;out+=quoteJson(ls.session)+",\"error\":"+quoteJson(ls.error)+"},";
  uint8_t hm,sm;double hd,sd;portENTER_CRITICAL(&dataMux);hm=headingManual?1:headingTrack?2:0;sm=savedHeadingMode;hd=headingDegrees;sd=savedHeadingDegrees;portEXIT_CRITICAL(&dataMux);
  snprintf(b,sizeof(b),"\"initial_heading\":{\"mode\":\"%s\",\"degrees\":%.17g,\"saved_mode\":\"%s\",\"saved_degrees\":%.17g},\"logger_stack_left_bytes\":%lu,\"safe_to_power_off\":%s,\"imu_ranges\":{\"gyro_dps\":1000,\"accel_g\":4,\"saturated_samples\":%lu},\"next\":%lu,\"dropped\":%lu,\"oldest\":%lu,\"rows\":[",headingName(hm),hd,headingName(sm),sd,ls.stackLeftBytes,ls.safeToPowerOff()?"true":"false",s.saturated,next,dropped,oldest);out+=b;
  for(uint32_t i=0;i<rows;i++){
    const Sample &r=batch[i];
    snprintf(b,sizeof(b),"%s[%lu,%llu,%lu,%lu,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g]",i?",":"",r.seq,r.t,r.dt,r.flags,r.w[0],r.w[1],r.w[2],r.f[0],r.f[1],r.f[2]);out+=b;
  }
  out+="]}";server.sendHeader("Cache-Control","no-store");server.send(200,"application/json; charset=utf-8",out);
}
bool originOk(){const String origin=server.header("Origin");return !origin.length()||origin=="http://"+server.hostHeader();}
void setupServer(){
  const char *headers[]={"Origin"};server.collectHeaders(headers,1);
  server.on("/",HTTP_GET,[]{server.sendHeader("Cache-Control","no-cache");server.sendHeader("Content-Encoding","gzip");server.send_P(200,"text/html; charset=utf-8",(char*)index_html,sizeof(index_html));});
  server.on("/app.js",HTTP_GET,[]{server.sendHeader("Cache-Control","no-cache");server.sendHeader("Content-Encoding","gzip");server.send_P(200,"text/javascript; charset=utf-8",(char*)app_js,sizeof(app_js));});
  server.on("/style.css",HTTP_GET,[]{server.sendHeader("Cache-Control","no-cache");server.sendHeader("Content-Encoding","gzip");server.send_P(200,"text/css; charset=utf-8",(char*)style_css,sizeof(style_css));});
  server.on("/api/telemetry",HTTP_GET,telemetry);
  server.on("/api/calibrate",HTTP_POST,[]{
    const String origin=server.header("Origin");
    if(origin.length() && origin!="http://"+server.hostHeader()){server.send(403,"application/json","{\"error\":\"origin\"}");return;}
    if(simulation){server.send(409,"text/plain","Calibration is disabled during simulation");return;}
    if(!imuReady){server.send(503,"application/json","{\"error\":\"imu unavailable\"}");return;}
    portENTER_CRITICAL(&dataMux);calibrationRequest=true;portEXIT_CRITICAL(&dataMux);
    server.send(202,"application/json","{\"calibration_requested\":true}");
  });
  server.on("/api/storage",HTTP_GET,[]{server.send(200,"application/json","{\"sessions\":"+loggerList()+"}");});
  server.on("/api/record",HTTP_POST,[]{if(!originOk()){server.send(403,"text/plain","origin");return;}if(!server.hasArg("enabled")||(server.arg("enabled")!="0"&&server.arg("enabled")!="1")){server.send(400,"text/plain","enabled=0 or 1");return;}loggerEnable(server.arg("enabled")=="1");server.send(202,"application/json","{\"requested\":true}");});
  server.on("/api/log",HTTP_GET,[]{if(!loggerDownload(server,server.arg("session"),server.arg("file")))server.send(404,"text/plain","session file unavailable");});
  server.on("/api/navigation",HTTP_POST,[]{
    if(!originOk()){server.send(403,"text/plain","origin");return;}double degrees=0;String mode=server.arg("mode");
    if(mode!="manual"&&mode!="forward"&&mode!="clear"){server.send(400,"text/plain","invalid mode");return;}
    if(mode=="manual"&&(!GnssParser::number(server.arg("heading").c_str(),degrees)||degrees<0||degrees>=360)){server.send(400,"text/plain","heading 0..359.999");return;}
    if(server.hasArg("persist")&&server.arg("persist")!="0"&&server.arg("persist")!="1"){server.send(400,"text/plain","persist=0 or 1");return;}
    bool persist=server.arg("persist")=="1";
    LogState l=loggerState();
    if(simulation||desiredSimulation||l.enabled||l.active){server.send(409,"text/plain; charset=utf-8","请先停止 TF 记录，等待保存完成后再设置方向。模拟模式不可设置。");return;}
    if(persist){
      char headingText[40];snprintf(headingText,sizeof(headingText),"%.17g",degrees);
      Preferences settings;String value=mode+":"+headingText;
      if(!settings.begin("insmini",false)){server.send(500,"text/plain","Heading storage unavailable");return;}
      bool saved=settings.putString("field_heading",value)>0;settings.end();
      if(!saved){server.send(500,"text/plain","Heading save failed");return;}
    }
    portENTER_CRITICAL(&dataMux);headingManual=mode=="manual";headingTrack=mode=="forward";headingDegrees=degrees;if(persist){savedHeadingMode=mode=="manual"?1:mode=="forward"?2:0;savedHeadingDegrees=degrees;}navControl++;portEXIT_CRITICAL(&dataMux);
    server.send(202,"application/json","{\"requested\":true}");
  });
  server.onNotFound([]{server.send(404,"text/plain","Not found");});server.begin();
}
void serialStatus(){
  State s;GnssState g;NavOutput n;bool sim;portENTER_CRITICAL(&dataMux);s=state;g=gnssState;n=navState;sim=simulation;portEXIT_CRITICAL(&dataMux);LogState l=loggerState();
  Serial.printf("INSmini %s mode=%s IMU=%lu Hz=%.2f missed=%lu heap=%u cal=%d NAV=%d accepted=%lu rejected=%lu PPS=%lu sync=%d TF=%d records=%lu drops=%lu errors=%lu\n",version,sim?"SIMULATION":"REAL",s.last.seq,1e6f/s.period,s.missed,ESP.getFreeHeap(),s.calibrated,n.status,n.gnssAccepted,n.gnssRejected,g.clock.ppsCount,g.clock.quality(esp_timer_get_time()),l.active,l.records,l.drops,l.errors);
  if(WiFi.status()==WL_CONNECTED)Serial.printf("HTTP http://%s/\n",WiFi.localIP().toString().c_str());
}
void serialCommand(const char *text){
  if(!strcmp(text,"s")){serialStatus();return;}
  if(!strcmp(text,"WIFI off")){WiFi.setAutoReconnect(false);WiFi.disconnect(false,false);Serial.println("WIFI paused; TF recording continues");return;}
  if(!strcmp(text,"WIFI on")){WiFi.setAutoReconnect(true);WiFi.begin();return;}
  if(!strcmp(text,"SIM route")){desiredSimulation=true;return;}
  if(!strcmp(text,"SIM stop")){desiredSimulation=false;return;}
  if(!strcmp(text,"LOG stop")){loggerEnable(false);return;}
  if(!strcmp(text,"LOG start")){loggerEnable(true);return;}
  if(!strcmp(text,"LOG stall")){loggerPauseForTest();Serial.println("TF TEST: consumer paused for 5 seconds; IMU continues");return;}
  if(text[0]=='c'){
    char request[100];strlcpy(request,text+1,sizeof(request));char *password=strchr(request,'\t');
    if(password){*password++=0;if(strlen(request)&&strlen(request)<=32&&strlen(password)>=8&&strlen(password)<=63){WiFi.begin(request,password);Serial.println("WIFI connection requested");}}
    memset(request,0,sizeof(request));return;
  }
}
void setup(){
  pinMode(LED,OUTPUT);pinMode(FIELD_BUTTON,INPUT_PULLUP);fieldButton.update(digitalRead(FIELD_BUTTON)==LOW,millis());loadFieldHeading();pinMode(CS,OUTPUT);digitalWrite(CS,HIGH);pinMode(SD_CS,OUTPUT);digitalWrite(SD_CS,HIGH);pinMode(SD_DETECT,INPUT_PULLUP);
  pinMode(GNSS_RESET,INPUT);pinMode(GNSS_PPS,INPUT);pinMode(INT1,INPUT);
  attachInterrupt(digitalPinToInterrupt(INT1),imuIsr,RISING);attachInterrupt(digitalPinToInterrupt(GNSS_PPS),ppsIsr,RISING);
  imuSpi.begin(IMU_SCK,IMU_MISO,IMU_MOSI,CS);sdSpi.begin(SD_SCK,SD_MISO,SD_MOSI,SD_CS);
  gnss.setRxBufferSize(4096);gnss.begin(9600,SERIAL_8N1,GNSS_RX,GNSS_TX);
  Serial.begin(115200);Serial.setDebugOutput(true);Serial.setTimeout(2000);mathOk=mathSelftest();
  WiFi.persistent(true);WiFi.mode(WIFI_STA);String mac=WiFi.macAddress();deviceName="insmini-"+mac.substring(12);deviceName.replace(":","");deviceName.toLowerCase();
  WiFi.setHostname(deviceName.c_str());WiFi.setAutoReconnect(true);WiFi.begin();bootId=String(esp_random(),HEX);
  // Complete initial association/NVS activity before enabling timestamped sampling.
  uint32_t wifiStart=millis();
  while(WiFi.status()!=WL_CONNECTED && millis()-wifiStart<5000){delay(10);}
  delay(500);
  navQueue=xQueueCreate(48,sizeof(NavInput));
  loggerStart(sdSpi,bootId);
  if(navQueue)xTaskCreatePinnedToCore(navigate,"psins-nav",16384,nullptr,2,nullptr,1);
  xTaskCreatePinnedToCore(receiveGnss,"gnss-clock",8192,nullptr,3,nullptr,0);
  imuReady=startImu();
  if(imuReady && mathOk)imuReady=xTaskCreatePinnedToCore(sampling,"imu",4096,nullptr,3,&imuTask,1)==pdPASS;
  setupServer();Serial.printf("INSmini web %s mathSelftest=%d IMU=%d\n",version,mathOk,imuReady);
}
void loop(){
  server.handleClient();
  static char command[160];static size_t used=0;
  while(Serial.available()){char c=Serial.read();if(c=='\n'){command[used]=0;serialCommand(command);memset(command,0,sizeof(command));used=0;}else if(c!='\r'){if(used<sizeof(command)-1)command[used++]=c;else used=0;}}
  if(WiFi.status()==WL_CONNECTED && !mdnsStarted){mdnsStarted=MDNS.begin(deviceName.c_str());if(mdnsStarted)MDNS.addService("http","tcp",80);serialStatus();}
  static uint32_t reportTime=0;uint32_t now=millis();
  if(fieldButton.update(digitalRead(FIELD_BUTTON)==LOW,now)){loggerEnable(false);Serial.println("FIELD STOP requested; waiting for TF close");}
  LogState l=loggerState();bool safe=l.safeToPowerOff();
  bool fault=!imuReady||!mathOk||l.errors||!l.mounted;
  digitalWrite(LED,fault?((now/100)%2):safe?HIGH:l.active&&l.enabled?((now/500)%2):((now/150)%2));
  static bool wasSafe=false;if(safe&&!wasSafe)Serial.println("TF closed; safe to disconnect power");wasSafe=safe;
  if(now-reportTime>=2000){reportTime=now;serialStatus();}
  delay(1);
}
