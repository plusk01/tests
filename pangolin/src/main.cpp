#include <vector>

#include <pangolin/pangolin.h>


int main() {
    // / view
  int width = 640;
  int height = 480;
  std::cout << "width:" << width << " , height:" << height << std::endl;
  // const int width = 1920, height = 1200;
  pangolin::CreateWindowAndBind("lidar2camera player", width, height);
}
