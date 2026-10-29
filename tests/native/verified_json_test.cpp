#include "verified_json.h"
#include <algorithm>
#include <cassert>
#include <cstdio>
#include <string>
#include <vector>

struct FakeFS {
  std::vector<uint8_t> bytes;
  bool present=false,failWriteOpen=false,failReadOpen=false,failWriteBuffer=false,failReadBuffer=false;
  bool shortWrite=false,shortRead=false,corruptRead=false,readError=false,wrongSize=false;
  size_t maxWrite=0,maxRead=0,writeOpens=0,readOpens=0,flushes=0,handles=0,maxHandles=0,buffers=0;
  mutable size_t operations=0;
  struct File {
    FakeFS *fs;bool writing;size_t offset=0;
    explicit File(FakeFS *disk=nullptr,bool write=false):fs(disk),writing(write){if(fs){fs->handles++;fs->maxHandles=std::max(fs->maxHandles,fs->handles);}}
    File(const File&)=delete;
    File(File &&other):fs(other.fs),writing(other.writing),offset(other.offset){other.fs=nullptr;}
    ~File(){close();}
    explicit operator bool()const{return fs!=nullptr;}
    bool setBufferSize(size_t size){assert(fs&&size==512);fs->operations++;fs->buffers++;return writing?!fs->failWriteBuffer:!fs->failReadBuffer;}
    size_t write(const uint8_t *data,size_t size){
      assert(fs&&writing);fs->operations++;fs->maxWrite=std::max(fs->maxWrite,size);size_t done=fs->shortWrite?size-1:size;
      fs->bytes.insert(fs->bytes.end(),data,data+done);offset+=done;return done;
    }
    int read(uint8_t *data,size_t size){
      assert(fs&&!writing);fs->operations++;fs->maxRead=std::max(fs->maxRead,size);if(fs->readError)return -1;
      size_t done=std::min(size,fs->bytes.size()-offset);if(fs->shortRead&&done)done--;
      std::memcpy(data,fs->bytes.data()+offset,done);offset+=done;if(fs->corruptRead&&done)data[0]^=1;return int(done);
    }
    size_t size()const{assert(fs);fs->operations++;return fs->bytes.size()+(fs->wrongSize?1:0);}
    void flush(){assert(fs);fs->operations++;fs->flushes++;}
    void close(){if(fs){assert(fs->handles);fs->operations++;fs->handles--;fs=nullptr;}}
  };
  bool exists(const char*)const{operations++;return present;}
  File open(const char*,const char *mode){
    operations++;
    bool write=std::strcmp(mode,"w")==0;assert(write||std::strcmp(mode,"r")==0);
    if(write){writeOpens++;if(failWriteOpen)return File();present=true;bytes.clear();}
    else{readOpens++;if(failReadOpen||!present)return File();}
    return File(this,write);
  }
};

FakeFS *pausedFS=nullptr;size_t pauses=0;
void countPause(){assert(pausedFS);pauses++;assert(pausedFS->operations==pauses);}

int main(){
  std::string text(1301,'x');for(size_t i=0;i<text.size();i++)text[i]=char(i%251);
  auto attempt=[&](FakeFS &fs){bool ok=writeVerifiedJson(fs,"/closed.json",text.data(),text.size());assert(fs.handles==0&&fs.maxHandles<=1&&fs.maxWrite<=512&&fs.maxRead<=512);return ok;};
  FakeFS good;assert(attempt(good));assert(good.bytes==std::vector<uint8_t>(text.begin(),text.end()));
  assert(good.maxWrite==512&&good.maxRead==512&&good.writeOpens==1&&good.readOpens==1&&good.buffers==2&&good.flushes==1);
  FakeFS existing;existing.present=true;existing.bytes={1,2,3};assert(!attempt(existing));assert(existing.bytes==std::vector<uint8_t>({1,2,3})&&existing.writeOpens==0&&existing.readOpens==0);
  FakeFS shortWrite;shortWrite.shortWrite=true;assert(!attempt(shortWrite));assert(shortWrite.present&&shortWrite.bytes.size()==511&&shortWrite.readOpens==0);
  FakeFS corrupt;corrupt.corruptRead=true;assert(!attempt(corrupt));assert(corrupt.present&&corrupt.bytes.size()==text.size());
  FakeFS writeOpen;writeOpen.failWriteOpen=true;assert(!attempt(writeOpen));assert(!writeOpen.present&&writeOpen.readOpens==0);
  FakeFS readOpen;readOpen.failReadOpen=true;assert(!attempt(readOpen));assert(readOpen.present&&readOpen.bytes.size()==text.size());
  FakeFS writeBuffer;writeBuffer.failWriteBuffer=true;assert(!attempt(writeBuffer));assert(writeBuffer.present&&writeBuffer.bytes.empty());
  FakeFS readBuffer;readBuffer.failReadBuffer=true;assert(!attempt(readBuffer));assert(readBuffer.buffers==2&&readBuffer.present);
  FakeFS size;size.wrongSize=true;assert(!attempt(size));assert(size.maxRead==0);
  FakeFS shortRead;shortRead.shortRead=true;assert(!attempt(shortRead));
  FakeFS readError;readError.readError=true;assert(!attempt(readError));
  for(int fault=0;fault<11;fault++){
    FakeFS fs;
    switch(fault){
      case 1:fs.present=true;fs.bytes={1,2,3};break;
      case 2:fs.shortWrite=true;break;
      case 3:fs.corruptRead=true;break;
      case 4:fs.failWriteOpen=true;break;
      case 5:fs.failReadOpen=true;break;
      case 6:fs.failWriteBuffer=true;break;
      case 7:fs.failReadBuffer=true;break;
      case 8:fs.wrongSize=true;break;
      case 9:fs.shortRead=true;break;
      case 10:fs.readError=true;break;
    }
    pausedFS=&fs;pauses=0;
    bool ok=writeVerifiedJson(fs,"/closed.json",text.data(),text.size(),countPause);
    assert(ok==(fault==0)&&pauses>0&&pauses==fs.operations&&fs.handles==0&&fs.maxHandles<=1&&fs.maxWrite<=512&&fs.maxRead<=512);
    if(fault==1)assert(fs.bytes==std::vector<uint8_t>({1,2,3}));
  }
  pausedFS=nullptr;
  std::puts("PASS verified JSON: exact readback, no overwrite, <=512-byte writes/reads, both buffers, short write/read, corrupt readback, size mismatch, open/buffer failures, closed handles and a pause after every FS operation");
}
