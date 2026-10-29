#include "gnss.h"
#include "clock_sync.h"
#include "navigation.h"
#include "PSINS.h"
#include <cassert>
#include <cstdio>
#include <string>
#include <vector>

std::string nmea(const char *body){unsigned c=0;for(const char *p=body;*p;p++)c^=(unsigned char)*p;char b[8];sprintf(b,"*%02X",c);return std::string("$")+body+b;}
int main(){
  GnssParser p;
  auto r=nmea("GNRMC,120000.000,A,3000.0000,N,11400.0000,E,0.00,0.00,010126,,,A");
  auto g=nmea("GNGGA,120000.000,3000.0000,N,11400.0000,E,1,10,0.8,100.0,M,25.0,M,,");
  assert(!p.parse(r.c_str(),10000000,10100000));assert(p.parse(g.c_str(),10150000,10220000));
  assert(p.fix.valid&&p.fix.altitude&&p.fix.height==125&&p.fix.utcUs==1767268800000000LL);
  assert(std::fabs(p.fix.lat-30*DEG)<1e-12);
  g.back()=g.back()=='1'?'2':'1';assert(!p.parse(g.c_str(),11000000,11200000));assert(p.bad==1);
  assert(GnssParser::dateSeconds("290224")>0&&GnssParser::dateSeconds("290223")==-1);
  double x;assert(!GnssParser::coordinate("3060.00","N",true,x));assert(!GnssParser::tod("235960.0",x));
  assert(!GnssParser::tod("1e300",x)&&!GnssParser::coordinate("1e300","N",true,x));
  ClockSync clock;
  for(int i=0;i<5;i++){uint64_t t=1000000ULL*(i+1)+uint64_t(25*i);clock.pps(t);assert(clock.pair(1767268800000000LL+1000000LL*i,t+100000,t+220000,t));}
  assert(clock.locked&&clock.quality(clock.lastPps+10000)==2&&clock.quality(clock.lastPps+3000000)==1&&clock.quality(clock.lastPps+11000000)==0);
  assert(std::abs(clock.utc(clock.anchorMono,clock.lastPps)-clock.anchorUtc)<=1);
  clock.pps(clock.lastPps+1200000);assert(!clock.locked);
  ImuResampler rs;double integrated=0;int samples=0;uint64_t mono=10000000;
  for(int i=0;i<1080;i++){NavInput s;s.dtUs=i%2?9250:9260;mono+=s.dtUs;s.mono=mono;s.w[0]=.1f;s.f[2]=9.8f;
    rs.feed(s,[&](const NavInput &r){assert(r.dtUs==10000);integrated+=r.w[0]*.01;samples++;});}
  assert(std::fabs(integrated-samples*.001)<1e-6&&samples>990);
  navigationReset();navigationHeading(0,false);
  double lat=30*DEG,lon=114*DEG,h=125,vel=0;int delivered=0;uint64_t time=10000000;
  std::vector<GnssFix> pending;
  double maxError=0;
  for(int k=0;k<12000;k++){
    double t=k*.01,acc=t>=10&&t<20?.5:0;vel+=acc*.01;
    CEarth earth;earth.Update(CVect3(lat,lon,h),CVect3(0,vel,0));lat+=vel/earth.RMh*.01;
    NavInput s;s.mono=time;s.dtUs=10000;s.seq=k+1;s.calibrated=true;s.q[0]=1;
    s.w[0]=earth.wnin.i;s.w[1]=earth.wnin.j;s.w[2]=earth.wnin.k;
    s.f[0]=-earth.gcc.i;s.f[1]=acc-earth.gcc.j;s.f[2]=-earth.gcc.k;
    if(k%100==0&&!(t>=30&&t<40)){
      GnssFix f;f.valid=f.altitude=f.velocity=true;f.quality=1;f.satellites=10;f.hdop=.8;f.lat=lat;f.lon=lon;f.height=h;f.ve=0;f.vn=vel;f.speed=vel;f.epochMono=time;f.utcUs=1767268800000000LL+k*10000LL;
      if(k==5500)f.lon+=.001;pending.push_back(f);
    }
    if(!pending.empty()&&time>=pending.front().epochMono+220000){navigationGnss(pending.front());pending.erase(pending.begin());delivered++;}
    navigationUpdate(s);auto v=navigationOutput();
    if(v.updates>0){double e=std::hypot((v.lat-lat)*RE,(v.lon-lon)*RE*std::cos(lat));maxError=std::fmax(e,maxError);assert(std::isfinite(e));}
    time+=10000;
  }
  auto v=navigationOutput();double finalError=std::hypot((v.lat-lat)*RE,(v.lon-lon)*RE*std::cos(lat));
  printf("NAV accepted=%u rejected=%u updates=%u status=%d maxError=%.4f finalError=%.4f sizeofFilter=%zu\n",v.gnssAccepted,v.gnssRejected,v.updates,v.status,maxError,finalError,sizeof(CSINSGNSS));
  assert(v.gnssAccepted>80&&v.gnssRejected>=1&&v.updates>11000&&v.status==1&&finalError<10&&maxError<30);
  navigationClearHeading();GnssFix fix;fix.valid=fix.altitude=fix.velocity=true;fix.lat=lat;fix.lon=lon;fix.height=h;fix.hdop=.8;fix.epochMono=time;fix.utcUs=1767268920000000LL;navigationGnss(fix);
  for(int i=0;i<100;i++){NavInput s;s.mono=time+i*10000;s.dtUs=10000;s.calibrated=true;s.f[2]=9.8;navigationUpdate(s);}assert(navigationOutput().updates==0);
  navigationHeading(0,false);fix.altitude=false;navigationGnss(fix);assert(navigationOutput().gnssRejected==1);
  navigationHeading(0,false);fix.altitude=true;fix.epochMono=time+2000000;fix.utcUs+=2000000;navigationGnss(fix);
  for(int i=0;i<1200;i++){NavInput s;navigationSynthetic(0,s,fix);s.mono=time+2000000+i*10000;s.dtUs=10000;navigationUpdate(s);}
  assert(navigationOutput().status==3);NavInput gap;gap.mono=time+15000000;gap.dtUs=10000;gap.calibrated=true;gap.flags=1;navigationUpdate(gap);assert(navigationOutput().resets>=1);
  puts("PASS: NMEA checksum/date/PVT, PPS drift/holdover/jump, measured-time resampling, original PSINS 15-state delayed fusion/outage/outlier/recovery");
}
