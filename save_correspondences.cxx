#include "util.h"
#include "alignment.h"
using namespace std;
//------------------------------------
//--- Save Correspondences Main ------
//------------------------------------
int main(int argc, char* argv[])
{
  if(argc < 4) {
    cout<<"usage: ./icp cloud0.ply cloud1.ply corrFile.txt"<<endl;
    return -1;
  }
  PointCloud::Ptr srcCloud = util::loadCloud(argv[1]);
  PointCloud::Ptr tgtCloud = util::loadCloud(argv[2]);
  cout<<"IR Cloud: "<<*srcCloud<<endl;
  cout<<"EO Cloud: "<<*tgtCloud<<endl;

  RoughFeatureAlignment rfa; 
  NormalCloud::Ptr srcNormals = rfa.getPointNormals(srcCloud);
  NormalCloud::Ptr tgtNormals = rfa.getPointNormals(tgtCloud);
  PFHCloud::Ptr srcPFH = rfa.getPFHFeatures(srcCloud, srcNormals);
  PFHCloud::Ptr tgtPFH = rfa.getPFHFeatures(tgtCloud, tgtNormals);

  //save correspondences
  pcl::CorrespondencesPtr corrs = rfa.estimateCorrespondances<pcl::FPFHSignature33>(srcPFH, tgtPFH);
  cout<<"Saving "<<corrs->size()<<" correspondences"<<endl;
  string outFile(argv[3]);
  util::saveCorrespondences(outFile, corrs);
  return 0;
}
