#include "alignment.h"
#include "util.h"
#include <pcl/common/common.h>
#include <pcl/registration/icp.h>
#include <pcl/point_types.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/normal_3d.h>
#include <pcl/registration/ia_ransac.h>
#include "correspondence_procrustes.h"
#include <iostream>
#include <algorithm>

//--- Main Public Method.  Computes rough alignment between two point clouds -----
Eigen::Matrix4f RoughFeatureAlignment::align(PointCloud::Ptr irCloud, 
                                             PointCloud::Ptr eoCloud,
                                             pcl::CorrespondencesPtr corrs)
{
  if(corrs.get() == NULL) {
    NormalCloud::Ptr irNormals = getPointNormals(irCloud);
    NormalCloud::Ptr eoNormals = getPointNormals(eoCloud);

    //----PFH Features -----
    PFHCloud::Ptr irPFH = getPFHFeatures(irCloud, irNormals);
    PFHCloud::Ptr eoPFH = getPFHFeatures(eoCloud, eoNormals);

    //---- Get PFH Corrs ----
    cout<<"Getting PFH Correspondences"<<endl;
    corrs = estimateCorrespondances<PointPFH>(irPFH, eoPFH); 
    cout<<"Number of correspondences found: "<<corrs->size()<<endl;
  }

  //----- Do rough scale alignment using top features ------
  sort(corrs->begin(), corrs->end(), pcl::isBetterCorrespondence);
  reverse(corrs->begin(), corrs->end());
  int numFeats = (int) (.1 * corrs->size());
  vector<int> indices_src(numFeats), indices_tgt(numFeats);
  for(int i=0; i<numFeats; ++i) {
    indices_src[i] = corrs->at(i).index_query;
    indices_tgt[i] = corrs->at(i).index_match;
  }
  
  int topHalf = (int) (.15 * corrs->size());
  pcl::CorrespondencesPtr topCorrs(new pcl::Correspondences());
  for(int i=0; i<topHalf; ++i)
    topCorrs->push_back( corrs->at(i) );
  corrs = topCorrs;

  // Estimate the centroids of source, target
  Eigen::Vector4f centroid_src, centroid_tgt;
  compute3DCentroid (*irCloud, indices_src, centroid_src);
  compute3DCentroid (*eoCloud, indices_tgt, centroid_tgt);

  // Compute RMSD for each centroid for top features
  PointXYZ srcCent(centroid_src[0], centroid_src[1], centroid_src[2]);
  double scale_src = util::computeRMSD(*irCloud, indices_src, srcCent);
  PointXYZ tgtCent(centroid_tgt[0], centroid_tgt[1], centroid_tgt[2]);
  double scale_tgt = util::computeRMSD(*eoCloud, indices_tgt, tgtCent); 
  
  //calculate rough target scaling
  double roughScale = scale_tgt / scale_src; 
  cout<<"Rough initial scaling: "<<roughScale<<endl;  
  Eigen::Matrix4f scale = roughScale*Eigen::Matrix4f::Identity();
  scale(3,3) = 1.0;
  PointCloud::Ptr scaledIR(new PointCloud());
  pcl::transformPointCloud(*irCloud, *scaledIR, scale);


  //----- Correspondence rejection ---------
  cout<<"Rejecting correspondences"<<endl;
  pcl::registration::CorrespondenceRejectorProcrustes<PointXYZ> sac;
  sac.setInputCloud(scaledIR);
  sac.setTargetCloud(eoCloud);
  sac.setMaxIterations(1000);
  sac.setInlierThreshold(20);//5.0 for geo downtown, .5 for hemenways....
  sac.setInputCorrespondences(corrs);
  pcl::CorrespondencesPtr newCorrs(new pcl::Correspondences());
  sac.getCorrespondences(*newCorrs);
  Eigen::Matrix4f rSacTrans = sac.getBestTransformation();
  trimmedCorrs_= newCorrs; 
  cout<<"Number of correspondences left: "<<trimmedCorrs_->size()<<endl;
  cout<<"Ransac transformation:\n"<<rSacTrans;
  
  //Estimate the transformation from these corrs - maybe even use SAC 
  Eigen::Matrix4f transform;
  pcl::registration::TransformationEstimationSVD<PointXYZ,PointXYZ> svd;
  svd.estimateRigidTransformation(*scaledIR, *eoCloud, *newCorrs, transform);
  Eigen::Matrix3f R = roughScale*transform.block<3,3>(0,0);
  transform.block<3,3>(0,0) = R; 
  return  transform; 
#if 0
  pcl::registration::TransformationEstimationLM<PointXYZ,PointXYZ> lev;
  lev.estimateRigidTransformation(*irCloud, *eoCloud, *corrs, transform);
  cout<<"Levenberg Marcquat est: \n"<<transform<<endl;
  //----Run initial alignment ----
	cout<<"Running SAC alignment"<<endl;
	Eigen::Matrix4f trans = rSacTrans;
	runSAC<PointXYZ, PointPFH>( irCloud, irPFH, eoCloud, eoPFH, trans ); 
	cout<<"SAC Transform:\n"<<trans<<endl;
#endif
}



//----- Takes in an xyz point cloud and normal cloud and spits out PFH Feature cloud --------
PFHCloud::Ptr 
RoughFeatureAlignment::getPFHFeatures(PointCloud::Ptr cloud, NormalCloud::Ptr normals, int K)
{
  // Create the PFH estimation class, and pass the input dataset+normals to it
  pcl::FPFHEstimation<PointXYZ, Normal, PointPFH> pfh;
  pfh.setInputCloud (cloud);
  pfh.setInputNormals (normals);
  // alternatively, if cloud is of tpe PointNormal, do pfh.setInputNormals (cloud);

  // Create an empty kdtree representation, and pass it to the PFH estimation object.
  // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
  pcl::search::KdTree<PointXYZ>::Ptr tree (new pcl::search::KdTree<PointXYZ> ());
  pfh.setSearchMethod (tree);

  // Output datasets
  PFHCloud::Ptr pfhs (new PFHCloud());

  // Use all neighbors in a sphere of radius 5cm
  // IMPORTANT: the radius used here has to be larger than the radius used to estimate the surface normals!!!
  //pfh.setRadiusSearch (radius);
  pfh.setKSearch(K);

  // Compute the features
  pfh.compute (*pfhs);

  // pfhs->points.size () should have the same size as the input cloud->points.size ()*
	return pfhs;
}


//---------Estimates point normals using points within radius------------
NormalCloud::Ptr 
RoughFeatureAlignment::getPointNormals(PointCloud::Ptr cloud, int K)
{
  // Create the normal estimation class, and pass the input dataset to it
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  ne.setInputCloud (cloud);

  // Create an empty kdtree representation, and pass it to the normal estimation object.
  // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
  ne.setSearchMethod (tree);

#if 0
  //test search method
  tree->setInputCloud(cloud);
  int index = 10000;
  double radius = 1.25;
  vector<int> k_indices;
  vector<float> k_sqr_distances;
  tree->radiusSearch(index, radius, k_indices, k_sqr_distances);
  cout<<"Tree found "<<k_indices.size()<<" neighbors in radius "<<radius<<endl;
#endif

  // Output datasets
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>());

  // Use all neighbors in a sphere of radius 3cm
  //ne.setRadiusSearch (radius);
  ne.setKSearch(K);

  // Compute the features
  ne.compute (*cloud_normals);

	//output cloud norms
	return cloud_normals; 
}

