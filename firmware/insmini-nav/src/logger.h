#pragma once
#include <Arduino.h>
#include <SPI.h>
#include <WebServer.h>
#include "log_format.h"
struct LogContext {bool calibrated=false,simulation=false;uint32_t calibrationId=0,headingId=0;uint8_t headingMode=0;double headingDegrees=0;float bias[3]={};};
struct LogState {
  bool mounted=false,active=false,enabled=true;uint32_t records=0,drops=0,errors=0,queued=0,queueBytes=0,maxQueueBytes=0,maxIoUs=0,stackLeftBytes=UINT32_MAX;uint64_t bytes=0;
  char session[64]={},error[64]={};
  bool safeToPowerOff()const{return mounted&&!enabled&&!active&&!queued&&records&&bytes&&!errors&&!error[0];}
};
LogContext getLogContext();
void loggerStart(SPIClass &bus,const String &boot);
void loggerPush(LogRecord r);
void loggerEnable(bool enabled);
void loggerPauseForTest();
LogState loggerState();
String loggerList();
bool loggerDownload(WebServer &server,const String &session,const String &file);
