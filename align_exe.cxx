#include "util.h"
#include "alignment.h"

//-------------------------
//--- Alignment Main ------
//-------------------------
int main(int argc, char* argv[])
{
  if(argc < 3) {
    cout<<"usage: ./icp cloud0.ply cloud1.ply [corrs.txt]"<<endl;
    return -1;
  }
  PointCloud::Ptr irCloud = util::loadCloud(argv[1]);
  PointCloud::Ptr eoCloud = util::loadCloud(argv[2]);
  cout<<"IR Cloud: "<<*irCloud<<endl;
  cout<<"EO Cloud: "<<*eoCloud<<endl;

  //transform EO cloud - rotation/translation
  PointXYZ min_pt, max_pt;
  pcl::getMinMax3D(*irCloud, min_pt, max_pt);
  double width = max_pt.x - min_pt.x;
  
  //initial transformation - scale, rotation and translation
  double scale = 2.0;
  Eigen::Matrix3f R;
  R<<0, -1, 0,
     1,  0, 0,
     0,  0, 1; 
  Eigen::Vector4f T(width, 0, 0, 1);  

  Eigen::Matrix4f trans0 = Eigen::Matrix4f::Zero();
  trans0.block<3,3>(0,0) = scale*R; 
  trans0.block<4,1>(0,3) = T;
  cout<<"Initial Transformation:\n"<<trans0<<endl;

  //if corr file is passed in, then use that....
  pcl::CorrespondencesPtr corrs; 
  if(argc == 4) {
    string corrFile(argv[3]); 
    corrs = util::loadCorrespondences(corrFile);
    cout<<"Corrs size: "<<corrs->size()<<endl;
  }
  else {
    //transform the point cloud
    pcl::transformPointCloud( *eoCloud, *eoCloud, trans0 ); 
    util::saveCloud("irOrigTransformed.pcd", *irCloud);
    util::saveCloud("eoOrigTransformed.pcd", *eoCloud);
  }

  //run rough alignment 
  RoughFeatureAlignment rfa; 
  Eigen::Matrix4f rough = rfa.align(irCloud, eoCloud, corrs);
  cout<<"True Transform: \n"<<trans0<<endl;
  cout<<"Rough alignment: \n"<<rough<<endl;

  //------- TRANFORM irCloud into eo Coordinates ------
  pcl::transformPointCloud( *irCloud, *irCloud, rough ); 
  util::saveCloud("irRough.pcd", *irCloud);
  util::saveCloud("eoRough.pcd", *eoCloud);

  //---- Trim the number of points in both clouds ----------
  pcl::CorrespondencesPtr newCorrs = rfa.getTrimmedCorrespondences();
  PointCloud::Ptr Atrim(new PointCloud()), Btrim(new PointCloud());
  util::trimClouds(irCloud, eoCloud, newCorrs, Atrim, Btrim);
  cout<<"Trimmed ir: "<<*Atrim<<endl;
  cout<<"Trimmed eo: "<<*Btrim<<endl;
  util::saveCloud("irTrimmed.pcd", *irCloud);
  util::saveCloud("eoTrimmed.pcd", *eoCloud);
  
  //---- RUN ICP ------guess transformation:
  Eigen::Matrix4f finalTrans;
  finalTrans << 1, 0, 0, 0,
                0, 1, 0,  0.0,
                0, 0, 1,  0.0,
                0, 0, 0,  1; 
  cout<<"Running ICP alignment"<<endl; 
  rfa.runICP<PointXYZ>(Atrim, Btrim, finalTrans);     
  cout<<"ICP Transform:\n"<<finalTrans<<endl;

  cout<<"Final Transorm:\n"<< finalTrans*rough << endl;
  pcl::transformPointCloud( *irCloud, *irCloud, finalTrans ); 
  util::saveCloud("irFinal.pcd", *irCloud);
  util::saveCloud("eoFinal.pcd", *eoCloud);
  return 0;
}

