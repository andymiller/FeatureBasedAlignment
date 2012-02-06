#include "util.h"
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <iostream>
using namespace std;
int main(int argc, char* argv[])
{
  if(argc < 2) {
    cout<<"usage: ./distance cloud.pcd"<<endl;
    return -1;
  }
  typedef pcl::PointXYZ PointXYZ;
  typedef pcl::PointCloud<PointXYZ> PointCloud;

  //load cloud
  int argI=1;
  PointCloud::Ptr cloud = util::loadCloud(argv[argI++]);
  cout<<"running distances on "<<*cloud<<endl;

  //put in kd tree
  pcl::KdTreeFLANN<PointXYZ> tree; 
  tree.setInputCloud(cloud);

  double minDist = 10e6, maxDist = -1.0;
  for(int i=0; i<cloud->size(); ++i) {
    int k = 2;
    vector<int> indices(k);
    vector<float> sqrDist(k);
    tree.nearestKSearch(i, k, indices, sqrDist);
    if(sqrDist[1] < minDist)
      minDist = sqrDist[1];

  }


  //find min/max distance
//  double minDist = 10e6, maxDist = -1.0;
//  for(int i=0; i<cloud->size(); ++i) {
//    PointXYZ pta = cloud->at(i);
//    for(int j=0; j<cloud->size(); ++j) {
//      if(j > i) 
//        continue;
//      PointXYZ ptb = cloud->at(j);
//      double dist2 = util::distance2(pta, ptb);
//      if(dist2 > maxDist)
//        maxDist = dist2;
//      if(dist2 < minDist)
//        minDist = dist2;
//    }
//  }
  minDist = sqrt(minDist);
  maxDist = sqrt(maxDist);

  //report the distances
  cout<<"Min distance between points "<<minDist<<endl;
  cout<<"Max distance between points "<<maxDist<<endl;
  return 0;
}
