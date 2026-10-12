#pragma once
#include <array>
#include <cstring>
#include <string>
enum { LOCK_TYPE_MUTEX };
// Offline replacement: no OS shared memory or semaphore calls.
class SharedMemory {
    std::array<unsigned char,480> bytes{};
public:
    SharedMemory(const char*,int,int,int,std::string) {}
    bool sharedMemValid() { return true; }
    void* connect() { return bytes.data(); }
    template<class T> void read(T* p,int count,int offset=0) { std::memcpy(p,bytes.data()+offset,sizeof(T)*count); }
    template<class T> void write(T* p,int count,int offset=0) { std::memcpy(bytes.data()+offset,p,sizeof(T)*count); }
};
