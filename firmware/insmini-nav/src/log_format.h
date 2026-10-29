#pragma once
#include <cstdint>
#include <cstddef>
#pragma pack(push,1)
struct LogRecord {
  uint32_t magic=0x334c4e49; // INL3, little endian
  uint8_t type=0,mode=0;uint16_t flags=0;uint32_t seq=0;
  uint64_t mono=0;int64_t utc=-1;double v[11]={};uint32_t crc=0;
};
#pragma pack(pop)
static_assert(sizeof(LogRecord)==120,"INL3 record layout");
inline uint32_t recordCrc(const void *data,size_t n){
  uint32_t c=0xffffffff;const uint8_t *p=(const uint8_t*)data;
  while(n--){c^=*p++;for(int k=0;k<8;k++)c=(c>>1)^(0xedb88320U&uint32_t(-int32_t(c&1)));}return ~c;
}
// 1 IMU: w[3],f[3],temperature,q[4]; flags bits 0/1 gap/saturation, bits 2..15 dt_us.
// 2 GNSS: lat,lon,h,VE,VN,HDOP,quality,sats,rxStart,rxEnd,epochMono.
// 3 NAV: lat,lon,h,VE,VN,VU,pitch,roll,yaw,posSigma,velSigma; flags=status.
// 4 PPS: v[0]=clock quality, v[1]=drift ppm; 5 NMEA: v bytes, flags=byte count.
// 6 truth: simulation reference lat,lon,h,VE,VN,VU. mode 0 real, 1 simulation.
