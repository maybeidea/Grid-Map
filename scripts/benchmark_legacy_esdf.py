#!/usr/bin/env python3
"""Extract the old distance transform unchanged; benchmark with preclassified sources.
Usage: python3 benchmark_legacy_esdf.py OLD_GRID_MAP_CPP OUTPUT_DIRECTORY
This measures the ESDF kernel, not ROS, occupancy fusion or task-ROI classification.
"""
from pathlib import Path
import subprocess
import sys
out = Path(sys.argv[2])
out.mkdir(parents=True, exist_ok=True)
s=Path(sys.argv[1]).read_text()
a=s.index('template <typename F_get_val, typename F_set_val>\nvoid GridMap::fillESDF');b=s.index('\nvoid GridMap::publishESDF()',a)
(out / 'legacy_esdf.cpp').write_text('''#include <Eigen/Core>
#include <vector>
#include <limits>
#include <cmath>
#include <random>
#include <chrono>
#include <ctime>
#include <iostream>
#include <iomanip>
#include <omp.h>
struct MappingParameters { enum class EsdfUnknownMode { kOff, kTaskRoi }; };
class GridMap {
public:
 struct Params {
  Eigen::Vector3i map_voxel_num_;
  Eigen::Vector3d map_origin_ = Eigen::Vector3d::Zero();
  double resolution_ = .015, ground_height_ = 0., esdf_roi_z_max_ = .5;
  MappingParameters::EsdfUnknownMode esdf_unknown_mode_ = MappingParameters::EsdfUnknownMode::kOff;
 } mp_;
 struct Data {
  Eigen::Vector3i local_bound_min_, local_bound_max_;
  std::vector<double> tmp_buffer1_, tmp_buffer2_, distance_buffer_, distance_buffer_neg_, distance_buffer_all_;
  std::vector<char> occupancy_buffer_neg_;
 } md_;
 std::vector<char> occupied;
 int toAddress(int x,int y,int z) { return (x*mp_.map_voxel_num_[1]+y)*mp_.map_voxel_num_[2]+z; }
 void posToIndex(const Eigen::Vector3d &, Eigen::Vector3i &) {}
 void boundIndex(Eigen::Vector3i &) {}
 void fillEsdfUnknownXyMask(const Eigen::Vector3i &low,const Eigen::Vector3i &high,std::vector<char>* mask) {
  mask->assign((high.x()-low.x()+1)*(high.y()-low.y()+1),0);
 }
 bool isEsdfSourceOccupied(int x,int y,int z,const std::vector<char>&, const Eigen::Vector3i&,int,int,int) { return occupied[toAddress(x,y,z)]; }
 template<typename G,typename S> void fillESDF(G,S,int,int,int);
 void updateESDF3d();
 GridMap(int x,int y,int z) {
  mp_.map_voxel_num_={x,y,z};md_.local_bound_min_.setZero();md_.local_bound_max_={x-1,y-1,z-1};
  const int n=x*y*z;
  md_.tmp_buffer1_.resize(n);md_.tmp_buffer2_.resize(n);md_.distance_buffer_.resize(n);
  md_.distance_buffer_neg_.resize(n);md_.distance_buffer_all_.resize(n);md_.occupancy_buffer_neg_.resize(n);
  occupied.resize(n);std::mt19937 rng(42);for(auto &v:occupied)v=rng()%20==0;
 }
};
''' + s[a:b] + '''
int main(int argc,char**argv) {
 const int runs=argc>1?std::stoi(argv[1]):30;
 omp_set_dynamic(0);omp_set_num_threads(2);
 std::cout<<"case,iterations,wall_mean_ms,cpu_ms,cpu_percent\\n"<<std::fixed<<std::setprecision(4);
 for(const auto dims:std::vector<Eigen::Vector3i>{{50,50,50},{70,70,17},{200,200,34}}) {
  GridMap map(dims.x(),dims.y(),dims.z());for(int i=0;i<3;++i)map.updateESDF3d();
  auto t=std::chrono::steady_clock::now();auto c=std::clock();
  for(int i=0;i<runs;++i){map.occupied[0]=i%2;map.updateESDF3d();}
  double wall=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-t).count();
  double cpu=1000.*(std::clock()-c)/CLOCKS_PER_SEC;
  std::cout<<"legacy_esdf_"<<dims.prod()<<','<<runs<<','<<wall/runs<<','<<cpu/runs<<','<<100*cpu/wall<<'\\n';
 }
}
''')

subprocess.run(["g++", "-O3", "-DNDEBUG", "-std=c++14", "-fopenmp",
                "-I/usr/include/eigen3", str(out / "legacy_esdf.cpp"),
                "-o", str(out / "legacy_esdf")], check=True)
subprocess.run([str(out / "legacy_esdf"), "30"], check=True)
