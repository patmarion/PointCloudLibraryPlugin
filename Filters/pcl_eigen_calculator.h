
#ifndef __pcl_eigen_calculator_h
#define __pcl_eigen_calculator_h

#include <pcl/features/normal_3d.h>

namespace pcl {

  template <typename PointInT, typename PointOutT>
  class EigenCalculator: public Feature<PointInT, PointOutT>
  {
    public:
      using Feature<PointInT, PointOutT>::feature_name_;
      using Feature<PointInT, PointOutT>::getClassName;
      using Feature<PointInT, PointOutT>::indices_;
      using Feature<PointInT, PointOutT>::input_;
      using Feature<PointInT, PointOutT>::surface_;
      using Feature<PointInT, PointOutT>::k_;
      using Feature<PointInT, PointOutT>::search_radius_;
      using Feature<PointInT, PointOutT>::search_parameter_;

      typedef typename Feature<PointInT, PointOutT>::PointCloudOut PointCloudOut;

      EigenCalculator () 
      {
        feature_name_ = "EigenCalculator";
      };


      inline void 
      computeEigenValues (const pcl::PointCloud<PointInT> &cloud, const std::vector<int> &indices, Eigen::Vector3f &eigen_values, Eigen::Matrix3f &eigen_vectors)
      {
        if (indices.empty ())
        {
          PCL_WARN("[computeEigenValues] indices is empty!\n");
          return;
        }

        // Estimate the XYZ centroid
        compute3DCentroid (cloud, indices, xyz_centroid_);

        // Compute the 3x3 covariance matrix
        computeCovarianceMatrix (cloud, indices, xyz_centroid_, covariance_matrix_);

        // Avoid getting hung on Eigen's optimizers
        for (int i = 0; i < 3; ++i)
          for (int j = 0; j < 3; ++j)
            if (!pcl_isfinite (covariance_matrix_ (i, j)))
            {
              PCL_WARN ("[computeEigenValues] Covariance matrix has NaN/Inf values!\n");
              return;
            }

        pcl::eigen33 (covariance_matrix_, eigen_vectors, eigen_values);
      }


    vtkFloatArray* eigen_vector_0;
    vtkFloatArray* eigen_vector_1;
    vtkFloatArray* eigen_vector_2;
    vtkFloatArray* eigen_values;

    protected:

      void computeFeature (PointCloudOut &output);
      void computeFeatureEigen (PointCloud<Eigen::MatrixXf> &output);

    private:

      EIGEN_ALIGN16 Eigen::Matrix3f covariance_matrix_;
      Eigen::Vector4f xyz_centroid_;
  };


}


template <typename PointInT, typename PointOutT> void
pcl::EigenCalculator<PointInT, PointOutT>::computeFeature (PointCloudOut &output)
{
  std::vector<int> nn_indices (k_);
  std::vector<float> nn_dists (k_);

  EIGEN_ALIGN16 Eigen::Vector3f eigen_values;
  EIGEN_ALIGN16 Eigen::Matrix3f eigen_vectors;

  for (size_t idx = 0; idx < indices_->size (); ++idx)
  {
    if (!this->searchForNeighbors ((*indices_)[idx], search_parameter_, nn_indices, nn_dists))
    {
      output.points[idx].normal[0] = output.points[idx].normal[1] = output.points[idx].normal[2] = output.points[idx].curvature = std::numeric_limits<float>::quiet_NaN ();
      continue;
    }

    computeEigenValues(*surface_, nn_indices, eigen_values, eigen_vectors);

    float eigenVec[9] = {
      eigen_vectors(0, 0), eigen_vectors(1, 0), eigen_vectors(2, 0),
      eigen_vectors(0, 1), eigen_vectors(1, 1), eigen_vectors(2, 1),
      eigen_vectors(0, 2), eigen_vectors(1, 2), eigen_vectors(2, 2),
    };

    this->eigen_vector_0->SetTuple(idx, eigenVec);
    this->eigen_vector_1->SetTuple(idx, eigenVec+3);
    this->eigen_vector_2->SetTuple(idx, eigenVec+6);
    this->eigen_values->SetTuple(idx, eigen_values.data());
  }
}

template <typename PointInT, typename PointOutT>
void
pcl::EigenCalculator<PointInT, PointOutT>::
computeFeatureEigen (PointCloud<Eigen::MatrixXf> &output)
{
  assert(false);
}

#endif
