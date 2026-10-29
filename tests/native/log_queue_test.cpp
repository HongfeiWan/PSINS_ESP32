#include "log_queue.h"
#include <cassert>
#include <deque>
#include <cstdio>
int main(){
  LogQueue<1024> q;std::deque<LogRecord> expected;uint32_t refused=0;
  auto take=[&](){LogRecord got;assert(q.pop(got));auto original=expected.front();expected.pop_front();assert(std::memcmp(&original,&got,offsetof(LogRecord,crc))==0);assert(q.count==expected.size());};
  for(int k=0;k<30000;k++){
    LogRecord r;r.type=1+k%6;r.mode=k%2;r.seq=k;r.mono=10000000ULL+k*9257;r.utc=1767268800000000LL+k*9257;
    r.flags=r.type==1?uint16_t(9257<<2):0;
    for(int i=0;i<11;i++)r.v[i]=r.type==1?double(float((k-i)*.001)):double(k+i)*.000000123456789;
    if(r.type==1&&k%101==0)r.v[0]=.123456789012345; // fallback preserves future double-precision inputs
    if(r.type==5){r.flags=(k%89)|0x100;std::memset(r.v,0,88);for(int i=0;i<(r.flags&255);i++)((uint8_t*)r.v)[i]=33+(i%80);}
    if(q.push(r))expected.push_back(r);else{refused++;assert(q.used()<=1024);}
    if(k%3==0&&!expected.empty())take();
  }
  while(!expected.empty())take();assert(q.count==0&&q.used()==0&&refused>0);
  LogRecord r;r.type=1;for(int i=0;i<11;i++)r.v[i]=float(i*.13f);assert(q.push(r)&&q.used()==69);LogRecord got;assert(q.pop(got));assert(std::memcmp(&r,&got,offsetof(LogRecord,crc))==0);
  std::puts("PASS lossless compact RAM queue: wrap, full, all record types, exact timestamps/doubles/floats, fallback and 69-byte IMU packing");
}
