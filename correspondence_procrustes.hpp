#ifndef correspondence_procrustes_hpp_
#define correspondence_procrustes_hpp_
#include <boost/unordered_map.hpp>

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void 
pcl::registration::CorrespondenceRejectorProcrustes<PointT>::getRemainingCorrespondences (
    const pcl::Correspondences& original_correspondences, 
    pcl::Correspondences& remaining_correspondences)
{
  int nr_correspondences = (int)original_correspondences.size ();
  std::vector<int> source_indices (nr_correspondences);
  std::vector<int> target_indices (nr_correspondences);

  // Copy the query-match indices
  for (size_t i = 0; i < original_correspondences.size (); ++i)
  {
    source_indices[i] = original_correspondences[i].index_query;
    target_indices[i] = original_correspondences[i].index_match;
  }

   // from pcl/registration/icp.hpp:
   std::vector<int> source_indices_good;
   std::vector<int> target_indices_good;
   {
     // From the set of correspondences found, attempt to remove outliers
     // Create the registration model
     typedef typename pcl::SampleConsensusModelNonRigid<PointT>::Ptr SampleConsensusModelNonRigidPtr;
     SampleConsensusModelNonRigidPtr model;
     model.reset (new pcl::SampleConsensusModelNonRigid<PointT> (input_, source_indices));
     // Pass the target_indices
     model->setInputTarget (target_, target_indices);
     // Create a RANSAC model
     pcl::RandomSampleConsensus<PointT> sac (model, inlier_threshold_);
     sac.setMaxIterations (max_iterations_);

     // Compute the set of inliers
     if (!sac.computeModel ())
     {
       remaining_correspondences = original_correspondences;
       best_transformation_.setIdentity ();
       return;
     }
     else
     {
       std::vector<int> inliers;
       sac.getInliers (inliers);

       if (inliers.size () < 3)
       {
         remaining_correspondences = original_correspondences;
         best_transformation_.setIdentity ();
         return;
       }
       boost::unordered_map<int, int> index_to_correspondence;
       for (int i = 0; i < nr_correspondences; ++i)
         index_to_correspondence[original_correspondences[i].index_query] = i;

       remaining_correspondences.resize (inliers.size ());
       for (size_t i = 0; i < inliers.size (); ++i)
         remaining_correspondences[i] = original_correspondences[index_to_correspondence[inliers[i]]];

       // get best transformation
       Eigen::VectorXf model_coefficients;
       sac.getModelCoefficients (model_coefficients);
       best_transformation_.row (0) = model_coefficients.segment<4>(0);
       best_transformation_.row (1) = model_coefficients.segment<4>(4);
       best_transformation_.row (2) = model_coefficients.segment<4>(8);
       best_transformation_.row (3) = model_coefficients.segment<4>(12);
     }
   }
}

#endif //correspondence_procrustes_hpp_

