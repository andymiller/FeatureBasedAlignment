#ifndef alignment_h_
#define alignment_h_

#include "util.h"
#include <pcl/common/common.h>
#include <pcl/registration/icp.h>
#include <pcl/point_types.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/normal_3d.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <iostream>
using namespace std;
  //point types
  typedef pcl::PointXYZ 			 		  		PointXYZ;
  typedef pcl::PointXYZRGB              PointXYZRGB;
  typedef pcl::Normal							  		Normal;
  typedef pcl::PointNormal							PointNormal;
  typedef pcl::FPFHSignature33					PointPFH;

  //cloud types 
  typedef pcl::PointCloud<PointXYZ> 		PointCloud;
  typedef pcl::PointCloud<PointXYZRGB>  ColorCloud;
  typedef pcl::PointCloud<Normal>   		NormalCloud;
  typedef pcl::PointCloud<PointNormal>	PointNormalCloud;
  typedef pcl::PointCloud<PointPFH>		  PFHCloud;


class RoughFeatureAlignment {
  public: 
    //---------- rough alignment step (non intersecting clouds) ------
    Eigen::Matrix4f align(PointCloud::Ptr A, 
                          PointCloud::Ptr B, 
                          pcl::CorrespondencesPtr corrs = pcl::CorrespondencesPtr());

    //get and return correspondences
    pcl::CorrespondencesPtr getCorrespondences() { return corrs_; }
    pcl::CorrespondencesPtr getTrimmedCorrespondences() { return trimmedCorrs_; }
 
    //----- Takes in an xyz point cloud and normal cloud and spits out PFH Feature cloud --------
    PFHCloud::Ptr getPFHFeatures(PointCloud::Ptr cloud, NormalCloud::Ptr normals, int K=60);

    //---------Estimates point normals using points within radius------------
    NormalCloud::Ptr getPointNormals(PointCloud::Ptr cloud, int K=45);

    //--------- estimate correspondances ----------------
    template <typename T>
    pcl::CorrespondencesPtr  estimateCorrespondances(typename pcl::PointCloud<T>::Ptr& sourcePts,
                                                     typename pcl::PointCloud<T>::Ptr& targetPts,
                                                     const unsigned K=10000); 
    
    //--------- More exact feature alignment ---------------
    template <typename T >
    typename pcl::PointCloud<T>::Ptr runICP(typename pcl::PointCloud<T>::Ptr irCloud, 
                                  typename pcl::PointCloud<T>::Ptr eoCloud, 
                                            Eigen::Matrix4f& transform) 
    {
      pcl::IterativeClosestPoint<T, T> icp;
      // Set the input source and target
      icp.setInputCloud(irCloud);
      icp.setInputTarget(eoCloud);

      // Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
      icp.setMaxCorrespondenceDistance (200);
      // Set the maximum number of iterations (criterion 1)
      icp.setMaximumIterations (300);
      // Set the transformation epsilon (criterion 2)
      icp.setTransformationEpsilon (1e-16);
      // Set the euclidean distance difference epsilon (criterion 3)
      icp.setEuclideanFitnessEpsilon (.5);

      // Perform the alignment
      PointCloud::Ptr registered(new pcl::PointCloud<T>());
      icp.align (*registered, transform);
      cout<<"ICP has converged:"<<icp.hasConverged()<<" score: "<<icp.getFitnessScore()<<endl;
      
      // Obtain the transformation that aligned cloud_source to cloud_source_registered
      //Eigen::Matrix4f transformation = icp.getFinalTransformation ();
      transform = icp.getFinalTransformation ();

      //report transform and error
      return registered;
    }
    

  private:
    pcl::CorrespondencesPtr  corrs_, trimmedCorrs_;

};

template <typename T>
pcl::CorrespondencesPtr 
RoughFeatureAlignment::estimateCorrespondances(typename pcl::PointCloud<T>::Ptr& sourcePts,
                                               typename pcl::PointCloud<T>::Ptr& targetPts,
                                               const unsigned K) 
{
  //---- grab correspondences for each point in the source cloud -----
  cout<<"Getting correspondences"<<endl; 
  cout<<"Src cloud: "<<*sourcePts<<endl;
  cout<<"Tar cloud: "<<*targetPts<<endl;
  pcl::registration::CorrespondenceEstimation<T,T> ce;
  ce.setInputTarget(targetPts);
  ce.setInputCloud(sourcePts);
  pcl::CorrespondencesPtr corr(new pcl::Correspondences());
  ce.determineCorrespondences(*corr);

  sort(corr->begin(), corr->end(), pcl::isBetterCorrespondence);
  //reverse(corr->begin(), corr->end());
  corrs_ = corr; 

  //corrs
  //vector<pcl::Correspondence> corrs; 
  //for(int i=0; i<corr->size(); ++i)
  //  corrs.push_back( (*corr)[i]) ;

  //find range of correspondences 
  //sort(corrs.begin(), corrs.end(), pcl::isBetterCorrespondence);
  //reverse(corrs.begin(), corrs.end());

  //get trimmed
  //reject manually....
  //pcl::CorrespondencesPtr trimmed(new pcl::Correspondences());
  //for(int i=0; i<corrs.size(); ++i) {
  //  trimmed->push_back(corrs[ corrs.size() - i - 1 ]);
  //}
  //trimmedCorrs_ = trimmed;
  return corrs_; 
}

#endif //alignment_h_


