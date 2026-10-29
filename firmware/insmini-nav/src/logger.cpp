#include "logger.h"
#include <SD.h>
#include <esp_timer.h>
#include <cerrno>
#include "log_queue.h"
#include "verified_json.h"
namespace {
LogQueue<65536> queue;bool ready=false;SemaphoreHandle_t diskMutex;portMUX_TYPE mux=portMUX_INITIALIZER_UNLOCKED;
uint32_t pauseForTest=0;
SPIClass *spi;String bootId;File stream;LogState status;LogContext current;volatile bool enabled=true;
uint8_t buffer[7200];size_t buffered=0;uint32_t flushAt=0,startDrops=0;
void metadataPause(){vTaskDelay(pdMS_TO_TICKS(1));}
size_t writeBuffer(size_t length){
  size_t done=0;
  while(done<length){size_t n=min(size_t(512),length-done);errno=0;size_t written=stream.write(buffer+done,n);
    if(written!=n){Serial.printf("TF WRITE requested=%u actual=%u errno=%d position=%u\n",unsigned(n),unsigned(written),errno,unsigned(stream.position()));return done+written;}done+=written;
  }
  return done;
}
void error(const char *message){portENTER_CRITICAL(&mux);status.errors++;strlcpy(status.error,message,sizeof(status.error));portEXIT_CRITICAL(&mux);}
void closeSession(const char *reason){
  if(!stream)return;
  bool ok=status.error[0]==0;if(buffered){ok=(writeBuffer(buffered)==buffered)&&ok;buffered=0;}
  stream.flush();uint64_t bytes=stream.size();stream.close();
  String path=String("/INSmini/")+status.session+"/closed.json";char text[256];
  int length=snprintf(text,sizeof(text),"{\"reason\":\"%s\",\"records\":%lu,\"bytes\":%llu,\"drops\":%lu,\"write_ok\":%s}",reason,status.records,bytes,status.drops-startDrops,ok?"true":"false");
  bool closed=length>0&&size_t(length)<sizeof(text)&&writeVerifiedJson(SD,path.c_str(),text,size_t(length),metadataPause);
  ok=closed&&ok;
  portENTER_CRITICAL(&mux);status.active=false;status.bytes=bytes;if(!ok){status.errors++;strlcpy(status.error,"close/write failed",sizeof(status.error));}portEXIT_CRITICAL(&mux);
}
bool openSession(const LogContext &c){
  portENTER_CRITICAL(&mux);startDrops=status.drops;portEXIT_CRITICAL(&mux);
  char id[64];snprintf(id,sizeof(id),"%s_%08lx_%s",bootId.c_str(),esp_random(),c.simulation?"SIM":"REAL");
  String dir=String("/INSmini/")+id;if(SD.exists(dir)||!SD.mkdir(dir))return false;
  char text[768];int length=snprintf(text,sizeof(text),"{\"format\":\"INL3\",\"record_bytes\":120,\"imu_interval\":\"flags>>2_us\",\"firmware\":\"3.0.4\",\"mode\":\"%s\",\"boot\":\"%s\",\"calibration_id\":%lu,\"heading_mode\":\"%s\",\"heading_deg\":%.17g,\"gyro_bias_rad_s\":[%.9g,%.9g,%.9g],\"axes\":\"sensor RFU\",\"height\":\"ellipsoid = GGA MSL + geoid separation\",\"clock\":\"UTC -1 until PPS/RMC lock; epoch phase requires outdoor verification\",\"start_mono_us\":%llu}",c.simulation?"simulation":"real",bootId.c_str(),c.calibrationId,c.headingMode==1?"manual":c.headingMode==2?"forward":"unknown",c.headingDegrees,c.bias[0],c.bias[1],c.bias[2],uint64_t(esp_timer_get_time()));
  if(length<=0||size_t(length)>=sizeof(text)||!writeVerifiedJson(SD,(dir+"/session.json").c_str(),text,size_t(length),metadataPause))return false;
  stream=SD.open(dir+"/stream.inl",FILE_WRITE);if(!stream)return false;
  if(!stream.setBufferSize(512)){stream.close();return false;}
  current=c;buffered=0;flushAt=millis();
  portENTER_CRITICAL(&mux);strlcpy(status.session,id,sizeof(status.session));status.records=0;status.bytes=0;status.active=true;status.error[0]=0;portEXIT_CRITICAL(&mux);return true;
}
void task(void*){
  uint32_t retryAt=0;
  for(;;){
    LogContext context=getLogContext();bool present=digitalRead(47)==LOW;
    if(xSemaphoreTake(diskMutex,pdMS_TO_TICKS(100))==pdTRUE){
      uint64_t ioStart=esp_timer_get_time();
      portENTER_CRITICAL(&mux);uint32_t pause=pauseForTest;pauseForTest=0;portEXIT_CRITICAL(&mux);
      if(pause)vTaskDelay(pdMS_TO_TICKS(pause));
      if(!present){
        if(status.active){stream.close();buffered=0;portENTER_CRITICAL(&mux);status.active=false;portEXIT_CRITICAL(&mux);error("card removed: session incomplete");}
        if(status.mounted){SD.end();portENTER_CRITICAL(&mux);status.mounted=false;portEXIT_CRITICAL(&mux);}
      }else if(!status.mounted&&millis()-retryAt>=1000){
        retryAt=millis();bool ok=SD.begin(35,*spi,1000000)&&SD.cardType()!=CARD_NONE;
        if(ok)ok=SD.exists("/INSmini")||SD.mkdir("/INSmini");
        portENTER_CRITICAL(&mux);status.mounted=ok;portEXIT_CRITICAL(&mux);if(!ok){SD.end();error("mount failed; card not formatted");}
      }
      if(status.active&&(!context.calibrated||context.simulation!=current.simulation||context.calibrationId!=current.calibrationId||context.headingId!=current.headingId))closeSession("calibration/mode/heading boundary");
      if(status.mounted&&!status.active&&enabled&&context.calibrated&&millis()-retryAt>=1000){retryAt=millis();if(!openSession(context))error("session open failed (full?)");}
      LogRecord r;int count=0;
      while(count++<96){
        portENTER_CRITICAL(&mux);bool have=queue.pop(r);portEXIT_CRITICAL(&mux);if(!have)break;
        r.crc=recordCrc(&r,offsetof(LogRecord,crc));
        if(!stream||r.mode!=uint8_t(current.simulation))continue;
        memcpy(buffer+buffered,&r,sizeof(r));buffered+=sizeof(r);
        portENTER_CRITICAL(&mux);status.records++;portEXIT_CRITICAL(&mux);
        if(buffered==sizeof(buffer)){
          size_t written=writeBuffer(buffered);buffered=0;
          portENTER_CRITICAL(&mux);status.bytes+=written;portEXIT_CRITICAL(&mux);
          if(written!=sizeof(buffer)){Serial.printf("TF SHORT requested=%u written=%u free=%llu\n",unsigned(sizeof(buffer)),unsigned(written),SD.totalBytes()-SD.usedBytes());error("short write / card full");closeSession("short write");enabled=false;break;}
        }
      }
      portENTER_CRITICAL(&mux);bool drained=queue.count==0;portEXIT_CRITICAL(&mux);
      if(status.active&&!enabled&&drained)closeSession("user stop, queue drained");
      if(stream&&millis()-flushAt>=1000){
        size_t n=buffered;if(n){size_t written=writeBuffer(n);buffered=0;portENTER_CRITICAL(&mux);status.bytes+=written;portEXIT_CRITICAL(&mux);if(written!=n){error("flush write failed");closeSession("flush error");enabled=false;}}
        if(stream)stream.flush();flushAt=millis();
        if(stream&&stream.size()>=64ULL*1024*1024)closeSession("64 MiB rotation");
      }
      uint32_t ioUs=uint32_t(esp_timer_get_time()-ioStart);portENTER_CRITICAL(&mux);status.maxIoUs=max(status.maxIoUs,ioUs);portEXIT_CRITICAL(&mux);
      xSemaphoreGive(diskMutex);
    }
    uint32_t stackLeft=uxTaskGetStackHighWaterMark(nullptr);
    portENTER_CRITICAL(&mux);status.enabled=enabled;status.queued=queue.count;status.queueBytes=queue.used();status.maxQueueBytes=queue.maxUsed;status.stackLeftBytes=min(status.stackLeftBytes,stackLeft);portEXIT_CRITICAL(&mux);
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}
}
void loggerStart(SPIClass &bus,const String &boot){
  spi=&bus;bootId=boot;diskMutex=xSemaphoreCreateMutex();
  ready=diskMutex&&xTaskCreatePinnedToCore(task,"tf-log",6144,nullptr,2,nullptr,0)==pdPASS;
  if(!ready){enabled=false;error("logger allocation failed");}
}
void loggerPush(LogRecord r){
  if(!ready||!enabled)return;
  portENTER_CRITICAL(&mux);if(!queue.push(r))status.drops++;portEXIT_CRITICAL(&mux);
}
void loggerEnable(bool v){enabled=v;}
void loggerPauseForTest(){portENTER_CRITICAL(&mux);pauseForTest=5000;portEXIT_CRITICAL(&mux);}
LogState loggerState(){portENTER_CRITICAL(&mux);LogState s=status;s.enabled=enabled;s.queued=queue.count;s.queueBytes=queue.used();s.maxQueueBytes=queue.maxUsed;portEXIT_CRITICAL(&mux);return s;}
String loggerList(){
  LogState s=loggerState();
  if(s.enabled||s.active)return s.session[0]?"[\""+String(s.session)+"\"]":"[]";
  if(!diskMutex||xSemaphoreTake(diskMutex,pdMS_TO_TICKS(250))!=pdTRUE)return "[]";
  String out="[";File root=SD.open("/INSmini");int n=0;
  if(root){File f=root.openNextFile();while(f&&n<128){if(f.isDirectory()){String name=f.name();int slash=name.lastIndexOf('/');if(slash>=0)name=name.substring(slash+1);if(n++)out+=',';out+='"'+name+'"';}f.close();f=root.openNextFile();}root.close();}
  xSemaphoreGive(diskMutex);return out+"]";
}
bool loggerDownload(WebServer &server,const String &id,const String &name){
  if(!id.length()||id.length()>63)return false;for(size_t i=0;i<id.length();i++)if(!isalnum((unsigned char)id[i])&&id[i]!='_')return false;
  if(name!="stream.inl"&&name!="session.json"&&name!="closed.json")return false;
  LogState s=loggerState();
  if(s.enabled||s.active){server.send(409,"text/plain","Stop TF recording and wait for it to close before downloading files");return true;}
  if(!diskMutex||xSemaphoreTake(diskMutex,pdMS_TO_TICKS(500))!=pdTRUE)return false;
  File f=SD.open("/INSmini/"+id+"/"+name,FILE_READ);size_t size=f?f.size():0;xSemaphoreGive(diskMutex);if(!f)return false;
  server.sendHeader("Content-Disposition","attachment; filename=\""+id+"_"+name+"\"");server.setContentLength(size);server.send(200,"application/octet-stream","");
  uint8_t chunk[2048];while(size&&server.client().connected()){
    if(xSemaphoreTake(diskMutex,pdMS_TO_TICKS(500))!=pdTRUE)break;
    size_t n=f.read(chunk,min(size,sizeof(chunk)));xSemaphoreGive(diskMutex);if(!n)break;
    size_t done=0;while(done<n&&server.client().connected()){size_t k=server.client().write(chunk+done,n-done);if(!k)break;done+=k;}if(done!=n)break;size-=n;delay(1);
  }
  if(xSemaphoreTake(diskMutex,pdMS_TO_TICKS(500))==pdTRUE){f.close();xSemaphoreGive(diskMutex);}return true;
}
