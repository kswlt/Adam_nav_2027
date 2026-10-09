#include "rm_nav_sensors/mid360_quality.hpp"
#include <cassert>
#include <functional>

using namespace rm_nav_sensors;
sensor_msgs::msg::PointCloud2 fixture(bool planar=false) {
  sensor_msgs::msg::PointCloud2 c;
  c.header.frame_id="front_mid360"; c.header.stamp.sec=100;
  c.width=1000;c.height=1;c.point_step=24;c.row_step=24000;c.data.resize(24000);
  const std::vector<std::string> names{"x","y","z","tag","timestamp"};
  const std::vector<int> offsets{0,4,8,12,16},types{7,7,7,2,8};
  for (int j=0;j<5;++j) { sensor_msgs::msg::PointField f;f.name=names[j];f.offset=offsets[j];f.datatype=types[j];f.count=1;c.fields.push_back(f); }
  for (int i=0;i<1000;++i) {
    const float xyz[3]={1.f+(i%10)*.2f,1.f+((i/10)%10)*.2f,planar ? 1.f : 1.f+(i/100)*.2f};
    std::memcpy(c.data.data()+i*24,xyz,12);
    const double t=100000000000.+i*100000.;std::memcpy(c.data.data()+i*24+16,&t,8);
  }
  return c;
}
void rejected(const std::function<void()> & fn) { bool threw=false;try {fn();}catch(const std::invalid_argument &){threw=true;}assert(threw); }
int main() {
  auto c=fixture();const auto q=inspect_mid360(c);assert(q.valid_points==1000);assert(q.eigen_ratio>.9);
  auto original=c.data;inspect_mid360(c);assert(c.data==original);
  auto interleaved=c;double first=100000000000.;
  std::memcpy(interleaved.data.data()+999*24+16,&first,8);
  assert(inspect_mid360(interleaved).valid_points==1000);
  rejected([]{inspect_mid360(fixture(true));});
  auto zero=c;for(int i=0;i<1000;++i)std::memset(zero.data.data()+24*i,0,12);
  rejected([&]{inspect_mid360(zero);});
  auto tag=c;for(int i=0;i<1000;++i)tag.data[24*i+12]=16;
  rejected([&]{inspect_mid360(tag);});
  auto time=c;double relative=0;std::memcpy(time.data.data()+16,&relative,8);
  rejected([&]{inspect_mid360(time);});
  auto nan=c;float bad=std::numeric_limits<float>::quiet_NaN();std::memcpy(nan.data.data(),&bad,4);
  rejected([&]{inspect_mid360(nan);});
  auto duplicate=c;duplicate.fields.push_back(c.fields[3]);rejected([&]{inspect_mid360(duplicate);});
  auto short_data=c;short_data.data.pop_back();rejected([&]{inspect_mid360(short_data);});
}
