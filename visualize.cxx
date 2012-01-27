#include "util.h"
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <iostream>
#include <algorithm>
using namespace std;
int main(int argc, char* argv[])
{
  if(argc < 3) {
    cout<<"usage: ./show_clouds cloud1.pcd cloud2.pcd"<<endl;
    return -1;
  }

  //load corrs if applicable
  pcl::CorrespondencesPtr corrs; 
  if(argc >= 4) { 
    corrs = util::loadCorrespondences(argv[3]);
    sort(corrs->begin(), corrs->end(), pcl::isBetterCorrespondence);
    reverse(corrs->begin(), corrs->end());
  }
  //load num to show if applicable
  int numShow = 0;
  if(argc >= 5)
    numShow = util::fromString<int>(argv[4]); 

  //visualize with correspondences
  pcl::PointCloud<pcl::PointXYZ>::Ptr irCloud = util::loadCloud(argv[1]);
  pcl::PointCloud<pcl::PointXYZ>::Ptr eoCloud = util::loadCloud(argv[2]);
  cout<<"red Cloud: "<<argv[1]<<'\n'<<*irCloud<<endl;
  cout<<"blue Cloud: "<<argv[2]<<'\n'<<*eoCloud<<endl;
  util::visualizeClouds(irCloud, eoCloud, corrs, numShow);
	return 0;
}
