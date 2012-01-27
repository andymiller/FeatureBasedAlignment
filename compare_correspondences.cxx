#include "util.h"
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <iostream>
using namespace std;
int main(int argc, char* argv[])
{
  if(argc < 3) {
    cout<<"usage: ./compare corr1.txt corr2.txt"<<endl;
    return -1;
  }

  //load corrs if applicable
  pcl::CorrespondencesPtr A = util::loadCorrespondences(argv[1]);
  pcl::CorrespondencesPtr B = util::loadCorrespondences(argv[2]);
	
  double ssd = 0.0; 
  int badMatches = 0, goodMatches = 0;
  double meanDist = 0.0;

  //map for quicker access
  map<int, pcl::Correspondence> corrsA; 
  for(int i=0; i<A->size(); ++i){
    pcl::Correspondence corrA = A->at(i);
    corrsA[corrA.index_query] = corrA;
  }

  //zip through B for comparison
  for(int i=0; i<B->size(); ++i) {
    pcl::Correspondence corrB = B->at(i);
    map<int, pcl::Correspondence>::iterator iter = corrsA.find(corrB.index_query);
    if(iter == corrsA.end()) 
      continue;

    pcl::Correspondence corrA = iter->second; 
    if(corrA.index_match == corrB.index_match) {
      ssd = (corrA.distance - corrB.distance) * (corrA.distance - corrB.distance);
      goodMatches++;
      meanDist += (corrA.distance + corrB.distance)/2.0;
    }
    else {
      badMatches++;
    }
  }

  cout<<"Number of good matches: "<<goodMatches<<'\n'
      <<"Number of bad matches:  "<<badMatches<<'\n'
      <<"SSD between good matches: "<<ssd/goodMatches<<'\n'
      <<"Mean distance for good matches: "<<meanDist/goodMatches<<endl;
  return 0;
}
