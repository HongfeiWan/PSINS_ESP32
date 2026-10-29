#pragma once
#include "log_format.h"
#include <cstring>
#include <algorithm>

// Lossless RAM packing. The SD format remains INL3 with double payload and CRC.
// Callers serialize producers/consumer with their task critical section.
template<size_t Capacity>struct LogQueue {
  static_assert(Capacity && !(Capacity&(Capacity-1)),"power-of-two capacity");
  uint8_t data[Capacity]={};uint32_t read=0,write=0,count=0,maxUsed=0;
  uint32_t used()const{return write-read;}
  void put(uint32_t at,const void *p,size_t n){size_t first=std::min(n,Capacity-(at&(Capacity-1)));std::memcpy(data+(at&(Capacity-1)),p,first);std::memcpy(data,(const uint8_t*)p+first,n-first);}
  void get(uint32_t at,void *p,size_t n)const{size_t first=std::min(n,Capacity-(at&(Capacity-1)));std::memcpy(p,data+(at&(Capacity-1)),first);std::memcpy((uint8_t*)p+first,data,n-first);}
  bool push(const LogRecord &r){
    uint8_t packet[112];double values[11];std::memcpy(values,r.v,88);constexpr size_t header=24;std::memcpy(packet,&r.type,header);size_t payload=88;
    if(r.type==1){bool exact=true;for(double value:values)if(double(float(value))!=value)exact=false;
      if(exact){payload=44;for(int i=0;i<11;i++){float value=float(values[i]);std::memcpy(packet+header+4*i,&value,4);}}
    }else if(r.type==5){
      size_t n=r.flags&255;if(n>88)return false;bool zero=true;const uint8_t *raw=(const uint8_t*)r.v;
      for(size_t i=n;i<88;i++)if(raw[i])zero=false;if(zero)payload=n;
    }
    if(payload!=44 || r.type!=1)std::memcpy(packet+header,r.v,payload);
    uint8_t n=uint8_t(header+payload);if(used()+n+1>Capacity)return false;
    put(write,&n,1);put(write+1,packet,n);write+=n+1;count++;maxUsed=std::max(maxUsed,used());return true;
  }
  bool pop(LogRecord &r){
    if(!count)return false;uint8_t n;get(read,&n,1);if(n<24||n>112||used()<uint32_t(n)+1)return false;
    uint8_t packet[112];get(read+1,packet,n);r=LogRecord();std::memcpy(&r.type,packet,24);
    if(r.type==1&&n==68){double values[11];for(int i=0;i<11;i++){float value;std::memcpy(&value,packet+24+4*i,4);values[i]=value;}std::memcpy(r.v,values,88);}
    else std::memcpy(r.v,packet+24,n-24);
    read+=n+1;count--;return true;
  }
};
