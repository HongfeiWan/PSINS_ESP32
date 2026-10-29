#pragma once
#include <cstdint>
#include "gnss.h"

struct NavInput {uint32_t seq=0,dtUs=0,flags=0;uint64_t mono=0;float w[3]={},f[3]={},q[4]={1,0,0,0},bias[3]={};bool calibrated=false;};
struct NavOutput {
  uint32_t updates=0,gnssAccepted=0,gnssRejected=0,resets=0;uint64_t mono=0,lastGnss=0;
  double lat=0,lon=0,h=0,ve=0,vn=0,vu=0,pitch=0,roll=0,yaw=0,pSigma=0,vSigma=0,bodyGyroBias[3]={},accBias[3]={};
  double q[4]={1,0,0,0};
  int status=0;bool headingKnown=false; // 0 waiting GNSS/heading, 1 fused, 2 coasting, 3 stale/error
};
// Accumulate measured intervals onto a 100 Hz grid for PSINS's AVP history.
struct ImuResampler {
  uint64_t cursor=0,next=0;double angle[3]={},velocity[3]={};
  void reset(){cursor=next=0;for(int i=0;i<3;i++)angle[i]=velocity[i]=0;}
  template<class Emit>void feed(const NavInput &s,Emit emit){
    if(s.flags || s.dtUs<4000 || s.dtUs>20000){reset();emit(s);return;}
    uint64_t begin=s.mono-s.dtUs;
    if(!next){cursor=begin;next=begin+10000;}
    if(begin!=cursor){reset();cursor=begin;next=begin+10000;}
    while(cursor<s.mono){
      uint64_t end=s.mono<next?s.mono:next;double dt=(end-cursor)*1e-6;
      for(int i=0;i<3;i++){angle[i]+=(s.w[i]-s.bias[i])*dt;velocity[i]+=s.f[i]*dt;}cursor=end;
      if(cursor==next){
        NavInput r=s;r.mono=next;r.dtUs=10000;
        for(int i=0;i<3;i++){r.w[i]=float(angle[i]*100+s.bias[i]);r.f[i]=float(velocity[i]*100);angle[i]=velocity[i]=0;}
        emit(r);next+=10000;
      }
    }
  }
};
void navigationReset();
void navigationHeading(double clockwiseNorthDeg,bool forwardAligned);
void navigationClearHeading();
void navigationGnss(const GnssFix &g);
void navigationUpdate(const NavInput &s);
NavOutput navigationOutput();
void navigationSynthetic(double seconds,NavInput &s,GnssFix &truth);
