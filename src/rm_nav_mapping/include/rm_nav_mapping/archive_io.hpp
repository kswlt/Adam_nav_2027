#pragma once
#include <filesystem>
#include <sstream>
#include <iomanip>
#include <fcntl.h>
#include <unistd.h>
#include <cerrno>
#include <stdexcept>

namespace rm_nav_mapping {
namespace fs=std::filesystem;
// Linux archive protocol: exclusive files, durable contents, then an atomic no-replace rename.
inline void durable_file(const fs::path & path,const void * data,std::size_t size)
{
  const int fd=::open(path.c_str(),O_WRONLY|O_CREAT|O_EXCL|O_CLOEXEC,0600);
  if(fd<0)throw std::runtime_error("Cannot exclusively create archive file");
  const auto * bytes=static_cast<const char *>(data);
  try {
    while(size) {
      const auto count=::write(fd,bytes,size);
      if(count<0 && errno==EINTR)continue;
      if(count<=0)throw std::runtime_error("Archive write failed");
      bytes+=count;size-=count;
    }
    if(::fsync(fd)!=0)throw std::runtime_error("Archive file fsync failed");
  }catch(...){::close(fd);throw;}
  if(::close(fd)!=0)throw std::runtime_error("Archive file close failed");
}
inline void durable_directory(const fs::path & path)
{
  const int fd=::open(path.c_str(),O_RDONLY|O_DIRECTORY|O_CLOEXEC);
  if(fd<0)throw std::runtime_error("Cannot open archive directory for fsync");
  const int result=::fsync(fd);::close(fd);
  if(result!=0)throw std::runtime_error("Archive directory fsync failed");
}
inline std::string json_string(const std::string & value)
{
  std::ostringstream out;out<<'"';
  for(unsigned char c:value) {
    if(c=='"' || c=='\\')out<<'\\'<<c;
    else if(c<32)out<<"\\u"<<std::hex<<std::setw(4)<<std::setfill('0')<<static_cast<int>(c)<<std::dec;
    else out<<c;
  }
  out<<'"';return out.str();
}

}
