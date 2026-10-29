#pragma once
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cctype>

struct GnssFix {
  int64_t utcUs=-1;uint64_t rxStart=0,rxEnd=0,epochMono=0;
  double lat=0,lon=0,height=0,msl=0,geoid=0,ve=0,vn=0,speed=0,course=0,hdop=99;
  int quality=0,satellites=0;bool valid=false,altitude=false,velocity=false;
};
struct GnssParser {
  GnssFix fix;uint32_t valid=0,bad=0,invalid=0,epochs=0;
  double ggaTod=-1,rmcTod=-2;int64_t rmcUtc=-1,lastPublished=-1;
  bool rmcValid=false,ggaValid=false;uint64_t ggaRx=0,rmcRx=0;
  static constexpr double rad=0.017453292519943295,knots=.5144444444444444;
  static bool number(const char *s,double &v){if(!s || !*s)return false;char *end=nullptr;v=std::strtod(s,&end);return end && !*end && std::isfinite(v);}
  static bool tod(const char *s,double &seconds){
    double x;if(!number(s,x) || x<0||x>=240000)return false;int h=int(x/10000),m=int(x/100)%100;double z=x-h*10000-m*100;
    if(h>23||m>59||z<0||z>=60)return false;seconds=h*3600+m*60+z;return true;
  }
  static bool coordinate(const char *s,const char *hem,bool latitude,double &r){
    double x;if(!number(s,x)||x<0||x>(latitude?9000:18000)||!hem||std::strlen(hem)!=1)return false;
    int d=int(x/100);double m=x-d*100;if(m>=60 || d>(latitude?90:180) || (d==(latitude?90:180)&&m>0))return false;
    char p=*hem;if(latitude?(p!='N'&&p!='S'):(p!='E'&&p!='W'))return false;
    r=(d+m/60)*rad;if(p=='S'||p=='W')r=-r;return true;
  }
  static int64_t dateSeconds(const char *s){
    if(!s||std::strlen(s)!=6)return -1;for(int i=0;i<6;i++)if(s[i]<'0'||s[i]>'9')return -1;
    int d=(s[0]-'0')*10+s[1]-'0',m=(s[2]-'0')*10+s[3]-'0',y=2000+(s[4]-'0')*10+s[5]-'0';
    int md[]={31,28,31,30,31,30,31,31,30,31,30,31};bool leap=(y%4==0&&(y%100!=0||y%400==0));md[1]+=leap;
    if(m<1||m>12||d<1||d>md[m-1])return -1;
    int64_t days=0;for(int k=1970;k<y;k++)days+=365+(k%4==0&&(k%100!=0||k%400==0));for(int k=1;k<m;k++)days+=md[k-1];days+=d-1;
    return days*86400;
  }
  // Returns one coherent GGA+RMC epoch. A valid checksum alone is not a valid fix.
  bool parse(const char *input,uint64_t first,uint64_t last){
    size_t n=std::strlen(input);if(n<10||n>159||input[0]!='$'){bad++;return false;}
    const char *star=std::strchr(input,'*');if(!star||std::strlen(star)!=3){bad++;return false;}
    if(!std::isxdigit(static_cast<unsigned char>(star[1]))||!std::isxdigit(static_cast<unsigned char>(star[2]))){bad++;return false;}
    char *end=nullptr;unsigned check=std::strtoul(star+1,&end,16);if(end!=star+3){bad++;return false;}
    uint8_t sum=0;for(const char *p=input+1;p<star;p++)sum^=uint8_t(*p);if(check!=sum){bad++;return false;}valid++;
    char line[160];std::memcpy(line,input,n+1);line[star-input]=0;
    char *f[24]={};int count=1;f[0]=line;
    for(char *p=line;*p;p++)if(*p==','){*p=0;if(count<24)f[count++]=p+1;}
    if(std::strlen(f[0])!=6)return false;
    if(!std::strcmp(f[0]+3,"GGA")){
      ggaValid=false;double t,q,s,h,lat,lon,msl,sep;
      if(count<15 || !tod(f[1],t)||!number(f[6],q)||!number(f[7],s)||!number(f[8],h)){invalid++;fix.valid=false;return false;}
      if(q<0||q>8||q!=std::floor(q)||s<0||s>99||s!=std::floor(s)){invalid++;fix.valid=false;return false;}
      ggaTod=t;ggaRx=last;fix.quality=int(q);fix.satellites=int(s);fix.hdop=h;
      if(q<1||s<4||h<=0||h>10||!coordinate(f[2],f[3],true,lat)||!coordinate(f[4],f[5],false,lon)){invalid++;fix.valid=false;return false;}
      fix.lat=lat;fix.lon=lon;fix.altitude=number(f[9],msl)&&!std::strcmp(f[10],"M")&&number(f[11],sep)&&!std::strcmp(f[12],"M");
      if(fix.altitude){fix.altitude=msl>=-1000&&msl<=50000&&std::fabs(sep)<=1000;fix.msl=msl;fix.geoid=sep;fix.height=msl+sep;}ggaValid=true;
    }else if(!std::strcmp(f[0]+3,"RMC")){
      rmcValid=false;double t,lat,lon,speed,course=0;
      int64_t day=count>9?dateSeconds(f[9]):-1;
      if(count<10||day<0||!tod(f[1],t)||std::strcmp(f[2],"A")||!coordinate(f[3],f[4],true,lat)||!coordinate(f[5],f[6],false,lon)||!number(f[7],speed)||speed<0||speed>515){invalid++;fix.valid=false;return false;}
      bool courseOK=number(f[8],course)&&course>=0&&course<=360;
      rmcTod=t;rmcRx=last;rmcUtc=day*1000000+int64_t(std::llround(t*1e6));rmcValid=true;
      fix.speed=speed*knots;fix.course=course*rad;fix.velocity=courseOK||speed<.05;
      fix.ve=fix.speed*std::sin(fix.course);fix.vn=fix.speed*std::cos(fix.course);
    }else return false;
    if(!ggaValid||!rmcValid||std::fabs(ggaTod-rmcTod)>.001||last-ggaRx>1500000||last-rmcRx>1500000)return false;
    fix.valid=true;fix.utcUs=rmcUtc;fix.rxStart=first;fix.rxEnd=last;
    if(rmcUtc==lastPublished)return false;lastPublished=rmcUtc;epochs++;return true;
  }
};
