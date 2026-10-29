#include <attitude.h>
#include <cassert>
#include <cstdio>
#include <fstream>
#include <cmath>
int main(){
  Attitude recorded;std::ifstream input("stationary.txt");assert(input.good());
  float w[3],f[3],dt;double elapsed=0;
  while(input>>dt>>w[0]>>w[1]>>w[2]>>f[0]>>f[1]>>f[2]){
    recorded.update(w,f,dt);elapsed+=dt;if(recorded.calibrated)break;
  }
  std::printf("recorded stationary calibration: completed=%d elapsed=%.3f\n",recorded.calibrated,elapsed);std::fflush(stdout);
  assert(recorded.calibrated&&elapsed<5.2);
  Attitude changingGyro,rocking;
  for(int k=0;k<1200;k++){
    const float t=k*.01f;
    float a[3]={0,0,Attitude::gravity};
    float g[3]={1.3f*Attitude::rad,(-4.4f+.8f*std::sin(3.14159265f*t))*Attitude::rad,0};
    changingGyro.update(g,a,.01f);assert(!changingGyro.calibrated);
    g[1]=-4.4f*Attitude::rad;a[1]=.05f*Attitude::gravity*std::sin(6.2831853f*t);
    rocking.update(g,a,.01f);assert(!rocking.calibrated);
  }
  changingGyro.resetCalibration();rocking.resetCalibration();
  for(int k=0;k<600;k++){
    float g[3]={1.3f*Attitude::rad,-4.4f*Attitude::rad,0},a[3]={0,0,Attitude::gravity};
    changingGyro.update(g,a,.01f);rocking.update(g,a,.01f);
  }
  assert(changingGyro.calibrated&&rocking.calibrated);
  assert(std::fabs(changingGyro.bias[1]+4.4f*Attitude::rad)<1e-5f);
  std::puts("PASS calibration: recorded stationary noise, changing gyro/force rejected, stationary recovery, bias units");
}
