#ifndef util_h_
#define util_h_
#include <pcl/common/common.h>
#include <pcl/point_types.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/visualization/cloud_viewer.h>
#include <sstream>
using namespace std;

//--- Utility functions -----
namespace util {

  //xyz point and xyz cloud
  typedef pcl::PointXYZ             PointXYZ;
  typedef pcl::PointXYZRGB          PointXYZRGB;
  typedef pcl::PointCloud<PointXYZ> PointCloud;
  typedef pcl::PointCloud<PointXYZRGB> ColorCloud;

  //trim clouds down to just the correspondences given
  void trimClouds(PointCloud::Ptr A, 
                  PointCloud::Ptr B, 
                  pcl::CorrespondencesPtr corrs, 
                  PointCloud::Ptr Atrim, 
                  PointCloud::Ptr Btrim);

  //cloud IO wrappers.  Visualizer wrapper
  PointCloud::Ptr loadCloud(const string& name);
  void            saveCloud(const string& name, const PointCloud& cloud);
  void            visualizeClouds(PointCloud::Ptr A, 
                                  PointCloud::Ptr B, 
                                  pcl::CorrespondencesPtr corrs=pcl::CorrespondencesPtr(),
                                  int numShow = 5);

  //correspondences IO wrapper
  pcl::CorrespondencesPtr loadCorrespondences(const string& name);
  void                    saveCorrespondences(const string& name, pcl::CorrespondencesPtr);

  //computer RMSD of cloud
  template<typename PointT>
  double computeRMSD(const pcl::PointCloud<PointT>& cloud, const PointT& pt);
  template<typename PointT>
  double computeRMSD(const pcl::PointCloud<PointT>& cloud, const vector<int>& indices, const PointT& pt);
  double computeRMSD(const Eigen::MatrixXf& cloud, const Eigen::Vector4f& pt);

  //pcl point distances - these definitely exist somewhere
  inline double distance2(const PointXYZ& a, const PointXYZ& b) {
    return (a.x-b.x)*(a.x-b.x)+(a.y-b.y)*(a.y-b.y)+(a.z-b.z)*(a.z-b.z); 
  } 
  inline double distance(const PointXYZ& a, const PointXYZ& b) { return sqrt(distance2(a,b)); }

  //---- String methods that I probably should not have had to implement ----
  //convert type from string
  template<class T>
  T fromString(const string& s);

  //split string into vector of strings
  vector<string>  split(const string& s, char delim=' ');
  vector<string>& split(const string &s, char delim, vector<string> &elems);
};

template<typename PointT>
double util::computeRMSD(const pcl::PointCloud<PointT>& cloud, const PointT& pt)
{
  double distSqrSum = 0.0; 
  for(int i=0; i<cloud.size(); ++i) 
    distSqrSum += distance2(pt, cloud[i]);
  return sqrt(distSqrSum / cloud.size()); 
}

template<typename PointT>
double util::computeRMSD(const pcl::PointCloud<PointT>& cloud, const vector<int>& indices, const PointT& pt)
{
  double distSqrSum = 0.0; 
  for(int i=0; i<indices.size(); ++i) {
    int idx = indices[i];
    distSqrSum += distance2(pt, cloud[idx]); 
  }
  return sqrt(distSqrSum / indices.size()); 
}


template<class T>
T util::fromString(const std::string& s)
{
  try {
    istringstream stream (s);
    T t;
    stream >> t;
    return t;
  }
  catch(int e){
    cout<<"Cannot convert string "<<s<<" to numeric value"<<endl;
    return 0;
  }
}

#endif //util_h_
