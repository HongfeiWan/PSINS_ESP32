#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include <vfs_api.h>
#include <esp_mac.h>
extern "C" {
#include <ff.h>
#include <diskio_impl.h>
}

// This isolated tool has no NVS clients. Prevent Arduino's exceptional NVS-erase fallback.
extern "C" esp_err_t __wrap_nvs_flash_init(){return ESP_OK;}

class FormatterSD : public fs::SDFS {
 public:
  FormatterSD():SDFS(fs::FSImplPtr(new VFSImpl())){}
  uint8_t drive()const{return _pdrv;}
};
FormatterSD card;
SPIClass bus(HSPI);
constexpr uint8_t expectedMac[6]={0x80,0x45,0x6b,0x44,0x07,0x68};
constexpr int TF_CS=35,TF_MOSI=36,TF_SCK=37,TF_MISO=38,TF_DETECT=47;
char macText[18]={},confirmation[120]={};
char volume[3]={'0',':',0};
FATFS *mountedFs=nullptr;
uint64_t previewBytes=0,previewTotal=0,previewUsed=0;
uint32_t previewEntries=0;
bool ready=false,attempted=false,destructiveStarted=false,cardRemoved=false;
uint32_t formatStarted=0;
static uint8_t mkfsWork[4096];

uint32_t crc32(uint32_t crc,const uint8_t *data,size_t count){
  while(count--){crc^=*data++;for(int i=0;i<8;i++)crc=(crc>>1)^((crc&1)?0xedb88320U:0);}
  return crc;
}
bool rootState(uint32_t &entries,FATFS *&filesystem){
  FF_DIR dir;FRESULT result=f_opendir(&dir,volume);
  if(result!=FR_OK)return false;
  filesystem=dir.obj.fs;entries=0;FILINFO info;
  for(;;){
    result=f_readdir(&dir,&info);
    if(result!=FR_OK||!info.fname[0])break;
    if(strcmp(info.fname,".")&&strcmp(info.fname,".."))entries++;
    if(entries>4096){result=FR_INT_ERR;break;}
    vTaskDelay(1);
  }
  FRESULT closed=f_closedir(&dir);
  return result==FR_OK&&closed==FR_OK&&filesystem;
}
bool uniqueTarget(){
  BYTE available=0xff;
  if(card.drive()!=0||ff_diskio_get_drive(&available)!=ESP_OK||available!=1)return false;
#if FF_MULTI_PARTITION
  if(VolToPart[0].pd!=0||VolToPart[0].pt!=0)return false;
#endif
  return mountedFs&&mountedFs->pdrv==0;
}
void preview(){
  Serial.printf("FORMAT_PREVIEW {\"ready\":%s,\"attempted\":%s,\"mac\":\"%s\",\"card_bytes\":%llu,\"total_bytes\":%llu,\"used_bytes\":%llu,\"pdrv\":%u,\"fat_type\":%u,\"root_entries\":%lu,\"cid_available\":false,\"confirm_command\":\"%s\"}\n",ready&&!cardRemoved?"true":"false",attempted?"true":"false",macText,previewBytes,previewTotal,previewUsed,card.drive(),mountedFs?mountedFs->fs_type:0,previewEntries,confirmation);
}
bool failed(const char *stage,int result){
  ready=false;
  Serial.printf("FORMAT_RESULT {\"status\":\"failed\",\"stage\":\"%s\",\"fatfs_result\":%d,\"destructive_started\":%s,\"elapsed_ms\":%lu}\n",stage,result,destructiveStarted?"true":"false",millis()-formatStarted);
  return false;
}
bool ioTest(uint32_t &writtenCrc,uint32_t &readCrc){
  const char *path="0:/FMTTEST.BIN";FIL file;UINT count=0;uint8_t block[512];
  FRESULT result=f_open(&file,path,FA_CREATE_NEW|FA_WRITE);
  if(result!=FR_OK)return failed("test_open_write",result);
  writtenCrc=0xffffffffU;
  for(unsigned part=0;part<8;part++){
    for(unsigned i=0;i<sizeof(block);i++)block[i]=uint8_t((part*512+i)*37+19);
    result=f_write(&file,block,sizeof(block),&count);
    if(result!=FR_OK||count!=sizeof(block)){f_close(&file);return failed("test_write",result==FR_OK?FR_DISK_ERR:result);}
    writtenCrc=crc32(writtenCrc,block,sizeof(block));vTaskDelay(1);
  }
  result=f_sync(&file);FRESULT closed=f_close(&file);
  if(result!=FR_OK||closed!=FR_OK)return failed("test_sync_close",result!=FR_OK?result:closed);
  writtenCrc^=0xffffffffU;readCrc=0xffffffffU;
  result=f_open(&file,path,FA_READ);
  if(result!=FR_OK)return failed("test_open_read",result);
  for(unsigned part=0;part<8;part++){
    result=f_read(&file,block,sizeof(block),&count);
    if(result!=FR_OK||count!=sizeof(block)){f_close(&file);return failed("test_read",result==FR_OK?FR_DISK_ERR:result);}
    for(unsigned i=0;i<sizeof(block);i++)if(block[i]!=uint8_t((part*512+i)*37+19)){f_close(&file);return failed("test_content",FR_INT_ERR);}
    readCrc=crc32(readCrc,block,sizeof(block));vTaskDelay(1);
  }
  result=f_read(&file,block,1,&count);closed=f_close(&file);readCrc^=0xffffffffU;
  if(result!=FR_OK||closed!=FR_OK||count||readCrc!=writtenCrc)return failed("test_crc_eof",FR_INT_ERR);
  result=f_unlink(path);
  if(result!=FR_OK)return failed("test_remove",result);
  return true;
}
void formatCard(){
  formatStarted=millis();
  if(!ready||cardRemoved||digitalRead(TF_DETECT)!=LOW||!uniqueTarget()||card.cardSize()!=previewBytes){failed("target_guard",FR_INVALID_DRIVE);return;}
  uint32_t entries=0;FATFS *original=nullptr;
  if(!rootState(entries,original)||original!=mountedFs){failed("before_unmount",FR_INVALID_OBJECT);return;}
  Serial.printf("FORMAT_STAGE {\"stage\":\"logical_unmount\",\"volume\":\"%s\",\"pdrv\":%u}\n",volume,card.drive());
  FRESULT result=f_mount(nullptr,volume,0);
  if(result!=FR_OK){failed("logical_unmount",result);return;}
  DWORD allocation=previewBytes>=4ULL*1024*1024*1024?32768:0;
  Serial.printf("FORMAT_STAGE {\"stage\":\"mkfs\",\"fat32\":true,\"allocation_unit_requested\":%lu}\n",allocation);
  destructiveStarted=true;
  result=f_mkfs(volume,FM_FAT32,allocation,mkfsWork,sizeof(mkfsWork));
  if(result!=FR_OK){failed("mkfs",result);return;}
  result=f_mount(original,volume,1);
  if(result!=FR_OK||original->fs_type!=FS_FAT32){failed("first_remount",result==FR_OK?FR_NO_FILESYSTEM:result);return;}
  // Only now unregister/reinitialize the card for an independent complete mount check.
  card.end();mountedFs=nullptr;delay(30);
  if(!card.begin(TF_CS,bus,1000000,"/sd",4,false)){failed("full_remount",FR_NOT_READY);return;}
  if(!rootState(entries,mountedFs)||!uniqueTarget()||mountedFs->fs_type!=FS_FAT32||entries){failed("empty_fat32_root",FR_INT_ERR);return;}
  if(card.cardSize()!=previewBytes){failed("capacity_changed",FR_INT_ERR);return;}
  Serial.println("FORMAT_STAGE {\"stage\":\"write_read_test\",\"bytes\":4096}");
  uint32_t writeCrc=0,readCrc=0;
  if(!ioTest(writeCrc,readCrc))return;
  card.end();mountedFs=nullptr;delay(30);
  if(!card.begin(TF_CS,bus,1000000,"/sd",4,false)){failed("post_test_remount",FR_NOT_READY);return;}
  if(!rootState(entries,mountedFs)||!uniqueTarget()||mountedFs->fs_type!=FS_FAT32||entries){failed("final_empty_root",FR_INT_ERR);return;}
  Serial.println("FORMAT_STAGE {\"stage\":\"final_capacity_scan\"}");
  uint64_t total=card.totalBytes(),used=card.usedBytes();
  Serial.println("FORMAT_STAGE {\"stage\":\"final_capacity_scan_complete\"}");
  if(!total||total>card.cardSize()){failed("final_capacity",FR_INT_ERR);return;}
  ready=false;
  Serial.printf("FORMAT_RESULT {\"status\":\"passed\",\"mac\":\"%s\",\"pdrv\":%u,\"fat32\":true,\"card_bytes\":%llu,\"total_bytes\":%llu,\"used_bytes\":%llu,\"allocation_unit_bytes\":%u,\"root_entries\":%lu,\"test_bytes\":4096,\"write_crc32\":\"%08lx\",\"read_crc32\":\"%08lx\",\"test_file_removed\":true,\"elapsed_ms\":%lu}\n",macText,card.drive(),card.cardSize(),total,used,unsigned(mountedFs->csize)*512,entries,writeCrc,readCrc,millis()-formatStarted);
}
void command(const char *line){
  if(!strcmp(line,"PREVIEW")){preview();return;}
  if(strncmp(line,"FORMAT",6)){Serial.println("FORMAT_REFUSED {\"reason\":\"unknown_command\"}");return;}
  if(attempted||!ready||cardRemoved||strcmp(line,confirmation)){Serial.println("FORMAT_REFUSED {\"reason\":\"guard_mismatch_or_already_attempted\"}");return;}
  attempted=true;formatCard();
}
void setup(){
  pinMode(TF_CS,OUTPUT);digitalWrite(TF_CS,HIGH);pinMode(10,OUTPUT);digitalWrite(10,HIGH);pinMode(TF_DETECT,INPUT_PULLUP);
  Serial.begin(115200);Serial.setTimeout(250);delay(1500);
  uint8_t mac[6]={};esp_err_t macResult=esp_read_mac(mac,ESP_MAC_WIFI_STA);
  snprintf(macText,sizeof(macText),"%02x:%02x:%02x:%02x:%02x:%02x",mac[0],mac[1],mac[2],mac[3],mac[4],mac[5]);
  Serial.printf("FORMATTER_BOOT {\"protocol\":\"insmini-tf-format/1\",\"mac\":\"%s\",\"automatic_format\":false,\"nvs_init_bypassed\":true}\n",macText);
  if(macResult!=ESP_OK||memcmp(mac,expectedMac,sizeof(mac))||digitalRead(TF_DETECT)!=LOW){Serial.println("FORMATTER_NOT_READY {\"reason\":\"mac_or_card_detect\"}");return;}
  bus.begin(TF_SCK,TF_MISO,TF_MOSI,TF_CS);
  Serial.println("FORMAT_STAGE {\"stage\":\"initial_mount\"}");
  if(!card.begin(TF_CS,bus,1000000,"/sd",4,false)){Serial.println("FORMATTER_NOT_READY {\"reason\":\"initial_mount_failed_no_autoformat\"}");return;}
  Serial.println("FORMAT_STAGE {\"stage\":\"initial_root_scan\"}");
  if(!rootState(previewEntries,mountedFs)||!uniqueTarget()){Serial.println("FORMATTER_NOT_READY {\"reason\":\"root_or_unique_drive\"}");return;}
  Serial.println("FORMAT_STAGE {\"stage\":\"initial_capacity_scan\"}");
  previewBytes=card.cardSize();previewTotal=card.totalBytes();previewUsed=card.usedBytes();
  Serial.println("FORMAT_STAGE {\"stage\":\"initial_capacity_scan_complete\"}");
  if(!previewBytes||!previewTotal||previewTotal>previewBytes){Serial.println("FORMATTER_NOT_READY {\"reason\":\"capacity_unavailable\"}");return;}
  snprintf(confirmation,sizeof(confirmation),"FORMAT %s %llu %08lx%08lx",macText,previewBytes,esp_random(),esp_random());
  ready=true;preview();
}
void loop(){
  if(digitalRead(TF_DETECT)!=LOW)cardRemoved=true;
  static char line[128];static size_t used=0;static bool overflow=false;
  while(Serial.available()){
    char c=Serial.read();
    if(c=='\n'){
      line[used]=0;
      if(overflow)Serial.println("FORMAT_REFUSED {\"reason\":\"line_too_long\"}");else command(line);
      used=0;overflow=false;
    }else if(c!='\r'){
      if(used<sizeof(line)-1&&!overflow)line[used++]=c;else overflow=true;
    }
  }
  delay(1);
}
