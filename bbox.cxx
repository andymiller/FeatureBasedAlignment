#include "util.h"
#include <pcl/common/common.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <iostream>
#include <algorithm>
using namespace std;
int main(int argc, char* argv[])
{
  if(argc < 2) {
    cout<<"usage: ./bbox cloud.pcd"<<endl;
    return -1;
  }

  //visualize with correspondences
  pcl::PointCloud<pcl::PointXYZ>::Ptr irCloud = util::loadCloud(argv[1]);
  pcl::PointXYZ minPt, maxPt; 
  pcl::getMinMax3D(*irCloud, minPt, maxPt);   

  cout<<"Bounding box min,max: "<<minPt<<","<<maxPt<<endl;
	return 0;
}
