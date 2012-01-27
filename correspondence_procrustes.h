#ifndef correspondence_procrustes_h_
#define correspondence_procrustes_h_

#include <pcl/registration/correspondence_rejection.h>

#include <pcl/sample_consensus/ransac.h>
#include "sac_non_rigid.h"
//#include <pcl/sample_consensus/sac_model_registration.h>

namespace pcl
{
  namespace registration
  {
    /** \brief CorrespondenceRejectorProcrustes implements a correspondence rejection
      * using Random Sample Consensus to identify inliers (and reject outliers), using
      * procrustes to estimate goodness of fit
      * \author Andy Miller
      * \ingroup registration
      */
    template <typename PointT>
    class CorrespondenceRejectorProcrustes: public CorrespondenceRejector
    {
      using CorrespondenceRejector::input_correspondences_;
      using CorrespondenceRejector::rejection_name_;
      using CorrespondenceRejector::getClassName;

      typedef pcl::PointCloud<PointT> PointCloud;
      typedef typename PointCloud::Ptr PointCloudPtr;
      typedef typename PointCloud::ConstPtr PointCloudConstPtr;

      public:

        /** \brief Empty constructor. */
        CorrespondenceRejectorProcrustes ()
        {
          rejection_name_ = "CorrespondenceRejectorProcrustes";
          inlier_threshold_ = 0.05;
        }

        /** \brief Get a list of valid correspondences after rejection from the original set of correspondences.
          * \param[in] original_correspondences the set of initial correspondences given
          * \param[out] remaining_correspondences the resultant filtered set of remaining correspondences
          */
        inline void 
        getRemainingCorrespondences (const pcl::Correspondences& original_correspondences, 
                                     pcl::Correspondences& remaining_correspondences);

        /** \brief Provide a source point cloud dataset (must contain XYZ data!)
          * \param[in] cloud a cloud containing XYZ data
          */
        virtual inline void 
        setInputCloud (const PointCloudConstPtr &cloud) { input_ = cloud; }

        /** \brief Provide a target point cloud dataset (must contain XYZ data!)
          * \param[in] cloud a cloud containing XYZ data
          */
        virtual inline void 
        setTargetCloud (const PointCloudConstPtr &cloud) { target_ = cloud; }

        /** \brief Set the maximum distance between corresponding points.
          * Correspondences with distances below the threshold are considered as inliers.
          * \param[in] threshold Distance threshold in the same dimension as source and target data sets.
          */
        inline void 
        setInlierThreshold (double threshold) { inlier_threshold_ = threshold; };

        /** \brief Get the maximum distance between corresponding points.
          * \return Distance threshold in the same dimension as source and target data sets.
          */
        inline double 
        getInlierThreshold() { return inlier_threshold_; };

        /** \brief Set the maximum number of iterations.
          * \param[in] max_iterations Maximum number if iterations to run
          */
        inline void 
        setMaxIterations (int max_iterations) {max_iterations_ = std::max(max_iterations, 0); };

        /** \brief Get the maximum number of iterations.
          * \return max_iterations Maximum number if iterations to run
          */
        inline int 
        getMaxIterations () { return max_iterations_; };

        /** \brief Get the best transformation after RANSAC rejection.
          * \return The homogeneous 4x4 transformation yielding the largest number of inliers.
          */
        inline Eigen::Matrix4f 
        getBestTransformation () { return best_transformation_; };

      protected:

        /** \brief Apply the rejection algorithm.
          * \param[out] correspondences the set of resultant correspondences.
          */
        inline void 
        applyRejection (pcl::Correspondences &correspondences)
        {
          getRemainingCorrespondences (*input_correspondences_, correspondences);
        }

        double inlier_threshold_;

        int max_iterations_;

        PointCloudConstPtr input_;
        PointCloudConstPtr target_;

        Eigen::Matrix4f best_transformation_;
    };
  }
}

#include "correspondence_procrustes.hpp"

#endif /* correspondence_procrustes_ */


