#include "util.h"
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <iostream>
using namespace std;
int main(int argc, char* argv[])
{
  if(argc < 7) {
    cout<<"usage: ./crop cloud.pcd centerX centerY centerZ radius cloud_out.pcd"<<endl;
    return -1;
  }
  typedef pcl::PointXYZ PointXYZ;
  typedef pcl::PointCloud<PointXYZ> PointCloud;

  //--- grab center argument ---
  //create center point
  PointXYZ center(util::fromString<float>(argv[2]),
                  util::fromString<float>(argv[3]),
                  util::fromString<float>(argv[4]));
  PointXYZ centerZ(center.x, center.y, 0.0);

  //radius
  double radius = util::fromString<double>(argv[5]); 
  double rad2 = radius*radius;
  cout<<"Cropping cloud around point: "<<center<<" with radius "<<radius<<endl;

  //load cloud from file
  PointCloud::Ptr cloud = util::loadCloud(argv[1]);

  //create cropped cloud - just crop along xy plane
  PointCloud::Ptr out(new PointCloud());
  for(int i=0; i<cloud->size(); ++i) {
    PointXYZ pt = cloud->at(i);
    PointXYZ pt2d(pt.x, pt.y, 0.0);
    if(util::distance2(pt2d, centerZ) < rad2)
      out->push_back(pt);
  }

  //save cloud
  cout<<"Saving cloud of size: "<<out->size()<<endl;
  util::saveCloud(argv[6], *out);

  return 0;
}
