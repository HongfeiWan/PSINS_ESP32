#pragma once
#include <cmath>
#include <cstdint>

struct ClockSync {
  uint64_t lastPps=0,anchorMono=0,prevPairMono=0;int64_t anchorUtc=-1,prevPairUtc=-1;
  uint32_t ppsCount=0,paired=0,rejected=0;int consecutive=0;double usPerSecond=1000000,driftPpm=0,residualUs=0,latencyMs=0;
  bool locked=false;
  bool pps(uint64_t t){
    if(lastPps && (t<=lastPps || t-lastPps<990000 || t-lastPps>1010000)){locked=false;consecutive=0;rejected++;lastPps=t;ppsCount++;return false;}
    lastPps=t;ppsCount++;return true;
  }
  bool pair(int64_t utc,uint64_t sentenceStart,uint64_t sentenceEnd,uint64_t ppsTime){
    if(utc<0 || utc%1000000!=0 || !ppsTime || ppsTime>sentenceStart || sentenceEnd<sentenceStart || sentenceEnd-ppsTime>900000){rejected++;return false;}
    if(prevPairUtc==utc)return false;
    if(prevPairUtc>=0){
      int64_t du=utc-prevPairUtc;uint64_t dm=ppsTime-prevPairMono;
      if(du!=1000000||ppsTime<=prevPairMono||dm<990000||dm>1010000){locked=false;consecutive=0;rejected++;}
      else{
        double measured=double(dm);residualUs=measured-usPerSecond;usPerSecond=consecutive>1?.85*usPerSecond+.15*measured:measured;
      }
    }
    prevPairUtc=anchorUtc=utc;prevPairMono=anchorMono=ppsTime;consecutive++;paired++;locked=consecutive>=3;
    driftPpm=(usPerSecond-1e6);latencyMs=(sentenceEnd-ppsTime)*.001;return true;
  }
  int quality(uint64_t now)const{
    if(!locked||!lastPps||!prevPairMono||now<lastPps||now<prevPairMono||now-lastPps>10000000||now-prevPairMono>10000000)return 0;
    return now-lastPps>2500000||now-prevPairMono>2500000?1:2;
  }
  int64_t utc(uint64_t mono,uint64_t now)const{
    if(!quality(now)||anchorUtc<0)return -1;
    return anchorUtc+int64_t(std::llround((double(mono)-double(anchorMono))*1e6/usPerSecond));
  }
};
