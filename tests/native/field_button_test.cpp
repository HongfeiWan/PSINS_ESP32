#include <cassert>
#include <cstdint>
#include <iostream>
#include "field_control.h"

int main(){
  FieldStopButton b;
  assert(!b.update(false,0));
  assert(!b.update(true,10));assert(!b.update(false,25));assert(!b.update(false,100));
  assert(!b.update(true,110));assert(!b.update(true,150));
  assert(!b.update(true,2149));assert(b.update(true,2150));
  assert(!b.update(true,8000));assert(!b.update(false,8010));assert(!b.update(false,8050));
  assert(!b.update(true,9000));assert(!b.update(true,9040));assert(b.update(true,11040));
  FieldStopButton startup;
  assert(!startup.update(true,0));assert(!startup.update(true,9000));
  assert(!startup.update(false,9010));assert(!startup.update(false,9050));
  assert(!startup.update(true,9100));assert(!startup.update(true,9140));assert(startup.update(true,11140));
  FieldStopButton wrap;
  assert(!wrap.update(false,0xfffff000U));assert(!wrap.update(true,0xffffff00U));
  assert(!wrap.update(true,0xffffff28U));assert(!wrap.update(true,0x000006f7U));
  assert(wrap.update(true,0x000006f8U));
  std::cout<<"PASS field BOOT hold: debounce, two-second threshold, no repeats, startup hold ignored, timer wrap\n";
}
