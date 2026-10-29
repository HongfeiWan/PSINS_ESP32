#pragma once
#include <cstddef>
#include <cstdint>
#include <cstring>

// The caller holds the filesystem mutex; partial files remain for diagnosis.
template<class FS>bool writeVerifiedJson(FS &disk,const char *path,const char *data,size_t length,void(*pause)()=nullptr){
  if(!path||(!data&&length))return false;
  auto rest=[pause](){if(pause)pause();};
  bool exists=disk.exists(path);rest();if(exists)return false;
  auto file=disk.open(path,"w");rest();
  if(!file)return false;
  bool buffered=file.setBufferSize(512);rest();
  if(!buffered){file.close();rest();return false;}
  for(size_t offset=0;offset<length;){
    size_t count=length-offset<512?length-offset:512;
    size_t written=file.write(reinterpret_cast<const uint8_t*>(data+offset),count);rest();
    if(written!=count){file.flush();rest();file.close();rest();return false;}
    offset+=count;
  }
  file.flush();rest();file.close();rest();
  auto check=disk.open(path,"r");rest();
  if(!check)return false;
  buffered=check.setBufferSize(512);rest();
  if(!buffered){check.close();rest();return false;}
  size_t stored=check.size();rest();
  if(stored!=length){check.close();rest();return false;}
  uint8_t buffer[512];
  for(size_t offset=0;offset<length;){
    size_t count=length-offset<sizeof(buffer)?length-offset:sizeof(buffer);
    size_t read=size_t(check.read(buffer,count));rest();
    if(read!=count||std::memcmp(buffer,data+offset,count)){check.close();rest();return false;}
    offset+=count;
  }
  check.close();rest();return true;
}
