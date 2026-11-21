// 与上游 README 同口径的微基准：只测 YAstar 纯函数，不含任何 ROS。
// 编译：g++ -O3 -std=c++17 bench_yastar.cpp yastar.cpp -I<eigen> -o bench
#include <cctype>
#include <chrono>
#include <iterator>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <string>
#include <vector>

#include "yastar.hpp"

namespace
{
struct Pgm
{
  int w{0}, h{0};
  std::vector<unsigned char> data;
};

Pgm loadPgm(const std::string & path)
{
  std::ifstream f(path, std::ios::binary);
  std::string content((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());

  // PGM 头允许 '#' 注释行，逐 token 解析时跳过整行
  std::size_t pos = 0;
  auto nextToken = [&content, &pos]() -> std::string {
      while (pos < content.size()) {
        if (content[pos] == '#') {
          while (pos < content.size() && content[pos] != '\n') {++pos;}
          continue;
        }
        if (std::isspace(static_cast<unsigned char>(content[pos]))) {++pos; continue;}
        break;
      }
      const std::size_t begin = pos;
      while (pos < content.size() && !std::isspace(static_cast<unsigned char>(content[pos]))) {++pos;}
      return content.substr(begin, pos - begin);
    };

  Pgm p;
  const std::string magic = nextToken();
  p.w = std::atoi(nextToken().c_str());
  p.h = std::atoi(nextToken().c_str());
  const int maxval = std::atoi(nextToken().c_str());
  (void)magic; (void)maxval;
  ++pos;  // 头部结束的单个换行

  p.data.resize(static_cast<std::size_t>(p.w) * p.h);
  std::memcpy(p.data.data(), content.data() + pos, p.data.size());
  return p;
}
}  // namespace

int main(int argc, char ** argv)
{
  const std::string map_path = argc > 1 ? argv[1] : "/ws/map.pgm";
  const int iters = argc > 2 ? std::atoi(argv[2]) : 200;

  Pgm img = loadPgm(map_path);
  if (img.w == 0) {
    std::printf("读图失败\n");
    return 1;
  }
  std::printf("地图 %dx%d（%d 格）\n", img.w, img.h, img.w * img.h);

  // YAstar 6 参 setMap：0 = 可行、非 0 = 占用
  std::vector<int8_t> occ(img.data.size());
  for (std::size_t i = 0; i < img.data.size(); ++i) {
    occ[i] = img.data[i] < 65 ? 1 : 0;
  }

  // 起终点取自仿真实测：起点 (3.4,5.0)、终点 (12,3)，原点与分辨率同参数文件
  const float res = 0.05f, ox = -3.58f, oy = -9.44f;
  Eigen::Vector2f start((3.4f - ox) / res, (5.0f - oy) / res);
  Eigen::Vector2f goal((12.0f - ox) / res, (3.0f - oy) / res);
  (void)start; (void)goal;

  YAstar planner;
  planner.setMap(img.w, img.h, res, ox, oy, occ);
  planner.setMaskNumLayers(4);
  planner.setCostField(1.0f, [](float x) { return 0.5f / (0.1f + x); });

  const auto t0 = std::chrono::steady_clock::now();
  planner.initCostMap();
  const double init_ms =
    std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
  std::printf("initCostMap（代价图）: %.2f ms\n", init_ms);

  // 搜索：真实世界坐标
  double sum = 0.0, worst = 0.0;
  std::size_t last_size = 0;
  for (int i = 0; i < iters; ++i) {
    const auto t = std::chrono::steady_clock::now();
    auto path = planner.search(Eigen::Vector2f(3.4f, 5.0f), Eigen::Vector2f(12.0f, 3.0f));
    const double ms =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t).count();
    sum += ms;
    if (ms > worst) worst = ms;
    last_size = path.size();
  }
  std::printf(
    "search x%d：平均 %.2f ms，最大 %.2f ms，末次路径 %zu 点\n",
    iters, sum / iters, worst, last_size);
  return 0;
}
