#pragma once
#include <cmath>
#include <cstdint>

// Scalar-first Hamilton quaternion, body -> local level, PSINS 3-1-2 Euler convention.
struct Attitude {
  float q[4]={1,0,0,0}, bias[3]={}, integral[3]={};
  float gyroSum[3]={}, accSum[3]={}, previousW[3]={}, previousF[3]={};
  float gyroM2[3]={},accM2[3]={};
  uint32_t count=0; float stillSeconds=0; bool calibrated=false, previous=false;
  static constexpr float gravity=9.80665f, rad=0.017453292519943295f;
  static float norm(const float *v){return std::sqrt(v[0]*v[0]+v[1]*v[1]+v[2]*v[2]);}
  static float clamp(float v){return v>1?1:v<-1?-1:v;}
  static void multiply(const float *a,const float *b,float *r){
    float t[4]={a[0]*b[0]-a[1]*b[1]-a[2]*b[2]-a[3]*b[3],
      a[0]*b[1]+a[1]*b[0]+a[2]*b[3]-a[3]*b[2],
      a[0]*b[2]-a[1]*b[3]+a[2]*b[0]+a[3]*b[1],
      a[0]*b[3]+a[1]*b[2]-a[2]*b[1]+a[3]*b[0]};
    for(int i=0;i<4;i++)r[i]=t[i];
  }
  void normalize(){float n=std::sqrt(q[0]*q[0]+q[1]*q[1]+q[2]*q[2]+q[3]*q[3]);for(auto &v:q)v/=n;}
  void setEuler(float pitch,float roll,float yaw){
    float a[4]={std::cos(yaw/2),0,0,std::sin(yaw/2)};
    float b[4]={std::cos(pitch/2),std::sin(pitch/2),0,0};
    float c[4]={std::cos(roll/2),0,std::sin(roll/2),0};
    multiply(a,b,q);multiply(q,c,q);normalize();
  }
  void tiltFromForce(const float *f){
    float n=norm(f);if(n<1)return;
    setEuler(std::asin(clamp(f[1]/n)),std::atan2(-f[0],f[2]),0);
  }
  void euler(float *a)const{
    const float w=q[0],x=q[1],y=q[2],z=q[3];
    const float c32=2*(y*z+w*x),c31=2*(x*z-w*y),c33=w*w-x*x-y*y+z*z;
    if(std::fabs(c32)>0.999999f){a[0]=std::copysign(1.57079632679f,c32);a[1]=std::atan2(2*(x*z+w*y),w*w+x*x-y*y-z*z);a[2]=0;}
    else{a[0]=std::asin(clamp(c32));a[1]=std::atan2(-c31,c33);a[2]=std::atan2(-2*(x*y-w*z),w*w-x*x+y*y-z*z);}
  }
  void resetCalibration(){
    calibrated=false;count=0;stillSeconds=0;previous=false;
    for(int i=0;i<3;i++){gyroSum[i]=accSum[i]=gyroM2[i]=accM2[i]=integral[i]=0;}
  }
  void update(const float *w,const float *f,float dt){
    if(dt<=0 || dt>0.05f)return;
    const float n=norm(f);
    if(!calibrated){
      bool still=n>gravity*.85f && n<gravity*1.15f && norm(w)<10*rad;
      if(previous)for(int i=0;i<3;i++)still=still && std::fabs(w[i]-previousW[i])<1.5f*rad && std::fabs(f[i]-previousF[i])<.04f*gravity;
      for(int i=0;i<3;i++){previousW[i]=w[i];previousF[i]=f[i];}previous=true;
      if(still){
        const float before=count?1.0f/count:0,after=1.0f/(count+1);
        for(int i=0;i<3;i++){
          const float dw=w[i]-gyroSum[i]*before,df=f[i]-accSum[i]*before;
          gyroSum[i]+=w[i];accSum[i]+=f[i];
          gyroM2[i]+=dw*(w[i]-gyroSum[i]*after);accM2[i]+=df*(f[i]-accSum[i]*after);
        }
        count++;stillSeconds+=dt;
      }else resetCalibration();
      if(stillSeconds>=5 && count>=400){
        // A five-second variance gate tolerates sensor noise and rejects slow changing motion.
        bool quiet=true;
        for(int i=0;i<3;i++)quiet=quiet && gyroM2[i]/(count-1)<(.35f*rad)*(.35f*rad) && accM2[i]/(count-1)<(.02f*gravity)*(.02f*gravity);
        if(quiet){
          for(int i=0;i<3;i++){bias[i]=gyroSum[i]/count;accSum[i]/=count;integral[i]=0;}
          tiltFromForce(accSum);calibrated=true;
        }else resetCalibration();
        if(!calibrated){tiltFromForce(f);return;}
      }else{tiltFromForce(f);return;}
    }
    float corrected[3];for(int i=0;i<3;i++)corrected[i]=w[i]-bias[i];
    if(n>gravity*.85f && n<gravity*1.15f){
      float g[3]={2*(q[1]*q[3]-q[0]*q[2]),2*(q[2]*q[3]+q[0]*q[1]),1-2*(q[1]*q[1]+q[2]*q[2])};
      float a[3]={f[0]/n,f[1]/n,f[2]/n};
      float e[3]={a[1]*g[2]-a[2]*g[1],a[2]*g[0]-a[0]*g[2],a[0]*g[1]-a[1]*g[0]};
      for(int i=0;i<3;i++){
        integral[i]+=.03f*e[i]*dt;
        integral[i]=std::fmax(-.05f,std::fmin(.05f,integral[i]));
        corrected[i]+=1.6f*e[i]+integral[i];
      }
    }
    float angle=norm(corrected)*dt,scale=angle<1e-6f?dt*.5f:std::sin(angle*.5f)*dt/angle;
    float dq[4]={std::cos(angle*.5f),corrected[0]*scale,corrected[1]*scale,corrected[2]*scale};
    multiply(q,dq,q);normalize();
  }
};
