#include "util.h"
#include "pcl/visualization/pcl_visualizer.h"
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <iostream>
#include <fstream>


//compute RMSD for eigen matrix represented point clouds
double util::computeRMSD(const Eigen::MatrixXf& cloud, const Eigen::Vector4f& pt)
{
  size_t npts = cloud.cols(); 
  if(cloud.rows() != 4) {
    cout<<"Cloud must be 4xNumPoints!"<<endl;
    return 0.0; 
  }  

  double distSqrSum = 0.0;
  for (size_t i = 0; i < npts; ++i) {
    Eigen::Vector4f cpt = cloud.block<4,1>(0,i);
    double distSqr = (pt[0] - cpt[0])*(pt[0] - cpt[0]) + 
                     (pt[1] - cpt[1])*(pt[1] - cpt[1]) + 
                     (pt[2] - cpt[2])*(pt[2] - cpt[2]); 
    distSqrSum+=distSqr; 
  }
  return sqrt(distSqrSum/npts);
}

//Trim clouds down to the correspondences
void util::trimClouds(PointCloud::Ptr A, PointCloud::Ptr B, pcl::CorrespondencesPtr corrs, PointCloud::Ptr Atrim, PointCloud::Ptr Btrim)
{
  for(int i=0; i<corrs->size(); ++i) {
    PointXYZ& ptA = A->at( corrs->at(i).index_query );
    PointXYZ& ptB = B->at( corrs->at(i).index_match );
    Atrim->push_back(ptA);
    Btrim->push_back(ptB);
  } 
}


//load cloud into pointer - does PCD and PLY files
util::PointCloud::Ptr util::loadCloud(const string& name)
{
  string ext = name.substr(name.find_last_of(".")+1);
  PointCloud::Ptr cloud(new PointCloud());
  if(ext == "ply") {
    pcl::PLYReader reader;
    reader.read(name, *cloud);
  }
  else if(ext=="pcd"){
    pcl::io::loadPCDFile(name, *cloud);
  }
  return cloud;
}

//save ply or pcd files
void util::saveCloud(const string& name, const util::PointCloud& cloud)
{
  string ext = name.substr(name.find_last_of(".")+1);
  if(ext == "ply")
    pcl::io::savePLYFile(name, cloud);
  else if(ext=="pcd")
    pcl::io::savePCDFile(name, cloud);
}

//show two clouds
void util::visualizeClouds(PointCloud::Ptr A, PointCloud::Ptr B, pcl::CorrespondencesPtr corrs, int numShow)
{
  //--- visualize transformed cloud.
  ColorCloud::Ptr Acolor(new ColorCloud());
  PointCloud::const_iterator aIter = A->begin();
  for(aIter = A->begin(); aIter != A->end(); ++aIter){
    PointXYZRGB pt;
    pt.x = aIter->x;
    pt.y = aIter->y;
    pt.z = aIter->z;
    uint8_t r = 255, g = 0, b = 0;    // Example: Red color
    uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
    pt.rgb = *reinterpret_cast<float*>(&rgb);    
    Acolor->push_back(pt);
  }

  //b cloud
  ColorCloud::Ptr Bcolor(new ColorCloud());
  PointCloud::const_iterator bIter = B->begin();
  for(bIter = B->begin(); bIter != B->end(); ++bIter){
    PointXYZRGB pt;
    pt.x = bIter->x;
    pt.y = bIter->y;
    pt.z = bIter->z;
    uint8_t r =0, g = 0, b = 255;    // Example:Blue color
    uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
    pt.rgb = *reinterpret_cast<float*>(&rgb);    
    Bcolor->push_back(pt);
  }

  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  pcl::visualization::PCLVisualizer viewer("3D Viewer");
  viewer.setBackgroundColor (0, 0, 0);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(Acolor), rgb2(Bcolor);
  viewer.addPointCloud<pcl::PointXYZRGB> (Acolor, rgb, "sample cloud");
  viewer.addPointCloud<pcl::PointXYZRGB> (Bcolor, rgb2, "cloud 2");
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud 2");
  viewer.addCoordinateSystem (1.0);
  viewer.initCameraParameters ();

  //------------------------------------
  //-----Add shapes at cloud points-----
  //------------------------------------
   //paint the first few corrs
  if(corrs.get() != NULL && numShow > 0) {
    numShow = min((int) corrs->size(), numShow); 
    int count=0;
    for(int i=0; i<corrs->size(); ++i) {
      pcl::Correspondence c = (*corrs)[i];
      if(c.distance < .01)
        continue;

      uint8_t r = 0, g = 255, b = 0;   
      uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
      PointXYZRGB& ptA = Acolor->at(c.index_query);
      PointXYZRGB& ptB = Bcolor->at(c.index_match);
      ptA.rgb = *reinterpret_cast<float*>(&rgb);
      ptB.rgb = *reinterpret_cast<float*>(&rgb);
      viewer.addLine<pcl::PointXYZRGB> (ptA, ptB, "line"+count);

      count++;
      if(count > numShow) break;
    }
  }

  //pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");
  //viewer.showCloud(Acolor, "cloud A");
  //viewer.showCloud(Bcolor, "cloud B");
  //--------------------
  // -----Main loop-----
  //--------------------
  while (!viewer.wasStopped ())
  {
    viewer.spinOnce (100);
    //boost::this_thread::sleep (boost::posix_time::microseconds (100000));  
  }
}



//String split helper methods....
vector<string>& util::split(const string &s, char delim, vector<string> &elems) {
    stringstream ss(s);
    string item;
    while(getline(ss, item, delim)) {
        elems.push_back(item);
    }
    return elems;
}
vector<string> util::split(const string &s, char delim) {
    vector<string> elems;
    return split(s, delim, elems);
}

//load correspondences from file - comma separated
pcl::CorrespondencesPtr util::loadCorrespondences(const string& name)
{
  pcl::CorrespondencesPtr corrs(new pcl::Correspondences);

  //load from file
  ifstream file(name.c_str()); 
  if (file.is_open())
  {
    string line; 
    while ( file.good() ) {
      getline (file,line);
      vector<string> strNums = util::split(line); 
      if(strNums.size()<3)
        continue;
      int index_query = util::fromString<int>(strNums[0]); 
      int index_match = util::fromString<int>(strNums[1]);
      float distance  = util::fromString<float>(strNums[2]);
      pcl::Correspondence corr(index_query, index_match, distance);
      corrs->push_back(corr);
    }
    file.close();
  }
  else {
    cout << "Unable to open correspondence file "<<name<<endl; 
  }
  return corrs;
}


//save correspondences to text file
void util::saveCorrespondences(const string& name, pcl::CorrespondencesPtr corrs )
{
  ofstream file;
  file.open (name.c_str());
  for(int i=0; i<corrs->size(); ++i) {
    file << corrs->at(i) << '\n';
  }
  file.close();
}
