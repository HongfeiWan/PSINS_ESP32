#pragma once
#include <cstdint>

// Ignore a BOOT held during startup; one hold stops once, without restarting a log.
struct FieldStopButton {
  bool initialized=false,raw=false,stable=false,armed=false,fired=false;
  uint32_t changed=0,pressed=0;
  bool update(bool down,uint32_t now){
    if(!initialized){initialized=true;raw=stable=down;changed=pressed=now;armed=!down;return false;}
    if(raw!=down){raw=down;changed=now;}
    if(raw!=stable&&uint32_t(now-changed)>=40){
      stable=raw;
      if(stable){pressed=now;fired=false;}
      else{armed=true;fired=false;}
    }
    if(stable&&raw&&armed&&!fired&&uint32_t(now-pressed)>=2000){fired=true;return true;}
    return false;
  }
};
