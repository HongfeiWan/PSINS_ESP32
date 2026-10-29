#include "navigation.h"
#include <cmath>
#pragma pack(push,4)
#include "PSINS.h"
#pragma pack(pop)

namespace {
CSINSGNSS filter(15,6,.01,-1);NavOutput output;GnssFix pending,lastFix;
bool initialized=false,manualHeading=false,trackHeading=false,havePending=false;
double heading=0;uint64_t origin=0,lastUpdate=0;int trackCount=0;
ImuResampler resampler;
double wrap(double x){while(x>PI)x-=2*PI;while(x<-PI)x+=2*PI;return x;}
void initialize(const NavInput &s){
  CVect3 att=q2att(CQuat(s.q[0],s.q[1],s.q[2],s.q[3]));att.k=heading;
  CVect3 vel(lastFix.velocity?lastFix.ve:0,lastFix.velocity?lastFix.vn:0,0),pos(lastFix.lat,lastFix.lon,lastFix.height);
  origin=s.mono;filter.Init(CSINS(att,vel,pos,0),1);
  filter.SetMeasMask(073); // horizontal velocity + 3D position; no invented vertical speed
  filter.lvGNSS=O31;filter.SetMeasStop(0xffffffff,0);
  filter.Pk.SetDiag2(fDEG3(10),fXXX(2),fdPOS(15),fDPH3(3000),fMG3(30));
  filter.Qt.Set2(fDPSH3(.5),fUGPSHZ3(300),fOOO,fOO6);
  filter.Rt.Set2(fXXZ(.5,10),fdLLH(5,10));filter.Rt0=filter.Rt;
  filter.Rmax=filter.Rt*100;filter.Rmin=filter.Rt*.1;filter.Rb=.6;
  output.lastGnss=lastFix.epochMono;output.headingKnown=true;initialized=true;lastUpdate=s.mono;
}
}
void navigationReset(){initialized=false;havePending=false;lastFix=GnssFix();origin=lastUpdate=0;trackCount=0;output=NavOutput();resampler.reset();}
void navigationHeading(double degrees,bool forwardAligned){
  if(!std::isfinite(degrees)||degrees<-360||degrees>360)return;
  manualHeading=!forwardAligned;trackHeading=forwardAligned;heading=wrap(-degrees*DEG);navigationReset();
}
void navigationClearHeading(){manualHeading=trackHeading=false;navigationReset();}
void navigationGnss(const GnssFix &g){
  if(!g.valid||!g.altitude||!g.epochMono||g.hdop>5){output.gnssRejected++;return;}
  if(lastFix.utcUs==g.utcUs)return;
  lastFix=g;pending=g;havePending=true;
  if(trackHeading&&!initialized){
    if(g.velocity&&g.speed>=2){heading=wrap(-g.course);trackCount++;}else trackCount=0;
  }
}
static void updateUniform(const NavInput &s){
  if(!s.calibrated)return;
  if(s.flags || s.dtUs<4000 || s.dtUs>20000){output.resets++;initialized=false;output.status=3;return;}
  if(!initialized){
    if(!lastFix.valid||s.mono<lastFix.epochMono||s.mono-lastFix.epochMono>2000000||(!manualHeading&&!(trackHeading&&trackCount>=3)))return;
    initialize(s);output.mono=s.mono;return;
  }
  if(s.mono<=lastUpdate||s.mono-lastUpdate>25000){output.resets++;initialized=false;output.status=3;return;}
  double dt=s.dtUs*1e-6;
  CVect3 wm((s.w[0]-s.bias[0])*dt,(s.w[1]-s.bias[1])*dt,(s.w[2]-s.bias[2])*dt),vm(s.f[0]*dt,s.f[1]*dt,s.f[2]*dt);
  filter.Update(&wm,&vm,1,dt,5);lastUpdate=s.mono;
  if(havePending&&pending.epochMono<=s.mono){
    double delay=(int64_t(pending.epochMono)-int64_t(s.mono))*1e-6;
    bool interp=delay>=-.85&&filter.avpi.Interp(delay,0x6);
    double dh=(filter.avpi.pos.i-pending.lat)*RE,de=(filter.avpi.pos.j-pending.lon)*RE*std::cos(pending.lat),du=filter.avpi.pos.k-pending.height;
    double limit=std::fmax(30.,pending.hdop*18.);
    if(interp&&std::sqrt(dh*dh+de*de+du*du)<limit){
      filter.posGNSSdelay=filter.vnGNSSdelay=delay;
      double scale=std::fmax(1.,pending.hdop);
      filter.Rt.dd[3]=std::pow(3*scale/RE,2);filter.Rt.dd[4]=std::pow(3*scale/(RE*std::cos(pending.lat)),2);filter.Rt.dd[5]=std::pow(5*scale,2);
      filter.SetMeasGNSS(CVect3(pending.lat,pending.lon,pending.height));
      if(pending.velocity&&std::hypot(filter.avpi.vn.i-pending.ve,filter.avpi.vn.j-pending.vn)<10){
        filter.Zk.dd[0]=filter.avpi.vn.i-pending.ve;filter.Zk.dd[1]=filter.avpi.vn.j-pending.vn;filter.SetMeasFlag(003);
      }
      output.gnssAccepted++;output.lastGnss=pending.epochMono;
    }else output.gnssRejected++;
    havePending=false;
  }
  const CSINS &n=filter.sins;
  output.mono=s.mono;output.updates++;output.lat=n.pos.i;output.lon=n.pos.j;output.h=n.pos.k;
  output.ve=n.vn.i;output.vn=n.vn.j;output.vu=n.vn.k;output.pitch=n.att.i;output.roll=n.att.j;output.yaw=n.att.k;
  output.q[0]=n.qnb.q0;output.q[1]=n.qnb.q1;output.q[2]=n.qnb.q2;output.q[3]=n.qnb.q3;
  output.pSigma=std::sqrt(std::fabs(filter.Pk(6,6))+std::fabs(filter.Pk(7,7)))*RE;
  output.vSigma=std::sqrt(std::fabs(filter.Pk(3,3))+std::fabs(filter.Pk(4,4))+std::fabs(filter.Pk(5,5)));
  for(int i=0;i<3;i++){output.bodyGyroBias[i]=s.bias[i]+(&n.eb.i)[i];output.accBias[i]=(&n.db.i)[i];}
  if(!std::isfinite(output.lat)||!std::isfinite(output.lon)||!std::isfinite(output.h)||!std::isfinite(output.vSigma)||std::fabs(output.h)>20000||output.vSigma>1000){initialized=false;output.status=3;return;}
  uint64_t age=s.mono-output.lastGnss;output.status=age>10000000?3:age>2500000?2:1;
}
void navigationUpdate(const NavInput &s){resampler.feed(s,updateUniform);}
NavOutput navigationOutput(){return output;}
void navigationSynthetic(double t,NavInput &s,GnssFix &g){
  double distance=t<10?0:t<20?.25*(t-10)*(t-10):25+5*(t-20);
  double v=t<10?0:t<20?.5*(t-10):5,acc=t>=10&&t<20?.5:0;
  CEarth earth;double lat=30*DEG;earth.Update(CVect3(lat,114*DEG,125));lat+=distance/earth.RMh;
  earth.Update(CVect3(lat,114*DEG,125),CVect3(0,v,0));
  s=NavInput();s.dtUs=10000;s.calibrated=true;
  for(int i=0;i<3;i++){s.w[i]=(&earth.wnin.i)[i];s.f[i]=-(&earth.gcc.i)[i];}s.f[1]+=acc;
  g=GnssFix();g.valid=g.altitude=g.velocity=true;g.quality=1;g.satellites=10;g.hdop=.8;
  g.lat=lat;g.lon=114*DEG;g.height=125;g.msl=100;g.geoid=25;g.vn=g.speed=v;
}
