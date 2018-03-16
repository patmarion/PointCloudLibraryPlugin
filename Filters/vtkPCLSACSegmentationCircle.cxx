/*=========================================================================

  Program:   Visualization Toolkit
  Module:    vtkPCLSACSegmentationCircle.cxx

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
#include "vtkPCLSACSegmentationCircle.h"
#include "vtkPCLConversions.h"

#include "vtkPolyData.h"
#include "vtkPointData.h"
#include "vtkInformation.h"
#include "vtkInformationVector.h"
#include "vtkObjectFactory.h"
#include "vtkSmartPointer.h"
#include "vtkStreamingDemandDrivenPipeline.h"
#include "vtkPlane.h"
#include "vtkNew.h"

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

//----------------------------------------------------------------------------
namespace {

void ComputeSACSegmentationCircle(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,
                                  double distanceThreshold,
                                  int maxIterations,
                                  const Eigen::Vector2d& radiusLimit,
                                  bool radiusConstraintEnabled,
                                  const Eigen::Vector3d& vtkNotUsed(parallelLine),
                                  bool vtkNotUsed(parallelLineEnabled),
                                  pcl::ModelCoefficients::Ptr &modelCoefficients,
                                  pcl::PointIndices::Ptr &inliers)
{
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  inliers = pcl::PointIndices::Ptr(new pcl::PointIndices);
  modelCoefficients = pcl::ModelCoefficients::Ptr(new pcl::ModelCoefficients);

  seg.setOptimizeCoefficients (true);
  seg.setModelType(pcl::SACMODEL_CIRCLE3D);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(maxIterations);
  seg.setDistanceThreshold(distanceThreshold);

  if (radiusConstraintEnabled)
    {
    seg.setRadiusLimits(radiusLimit[0], radiusLimit[1]);
    }

  seg.setInputCloud(cloud);
  seg.segment(*inliers, *modelCoefficients);
}


void ComputeSACSegmentationPerpendicularPlane(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,
                                  double distanceThreshold,
                                  const Eigen::Vector3f& ParallelLine,
                                  double angleEpsilon,
                                  int maxIterations,
                                  pcl::ModelCoefficients::Ptr &modelCoefficients,
                                  pcl::PointIndices::Ptr &inliers)
{

  pcl::SACSegmentation<pcl::PointXYZ> seg;
  inliers = pcl::PointIndices::Ptr(new pcl::PointIndices);
  modelCoefficients = pcl::ModelCoefficients::Ptr(new pcl::ModelCoefficients);

  seg.setOptimizeCoefficients (true);
  seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(maxIterations);
  seg.setDistanceThreshold(distanceThreshold);
  seg.setAxis(ParallelLine);
  seg.setEpsAngle(angleEpsilon);

  seg.setInputCloud(cloud);
  seg.segment(*inliers, *modelCoefficients);
}

}


//----------------------------------------------------------------------------
vtkStandardNewMacro(vtkPCLSACSegmentationCircle);

//----------------------------------------------------------------------------
vtkPCLSACSegmentationCircle::vtkPCLSACSegmentationCircle()
{
  this->DistanceThreshold = 0.05;
  this->MaxIterations = 200;

  this->CircleRadius = 0.0;
  for (size_t i = 0; i < 3; ++i)
    {
    this->CircleOrigin[i] = 0.0;
    this->CircleNormal[i] = 0.0;
    }

  this->ParallelLineConstraintEnabled = false;
  this->RadiusConstraintEnabled = false;
  this->AngleEpsilon = 0.2;
  this->ParallelLine[0] = 1.0;
  this->ParallelLine[1] = 0.0;
  this->ParallelLine[2] = 0.0;
  this->RadiusLimit[0] = 0.0;
  this->RadiusLimit[1] = 0.0;

  this->SetNumberOfInputPorts(1);
  this->SetNumberOfOutputPorts(1);
}

//----------------------------------------------------------------------------
vtkPCLSACSegmentationCircle::~vtkPCLSACSegmentationCircle()
{
}

//----------------------------------------------------------------------------
int vtkPCLSACSegmentationCircle::RequestData(
  vtkInformation* vtkNotUsed(request),
  vtkInformationVector **inputVector,
  vtkInformationVector *outputVector)
{
  // get input and output data objects
  vtkInformation *inInfo = inputVector[0]->GetInformationObject(0);
  vtkPolyData *input = vtkPolyData::SafeDownCast(inInfo->Get(vtkDataObject::DATA_OBJECT()));
  vtkInformation *outInfo = outputVector->GetInformationObject(0);
  vtkPolyData *output = vtkPolyData::SafeDownCast(outInfo->Get(vtkDataObject::DATA_OBJECT()));

  // perform plane model fit
  pcl::PointIndices::Ptr inlierIndices;
  pcl::ModelCoefficients::Ptr modelCoefficients;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = vtkPCLConversions::PointCloudFromPolyData(input);


  ComputeSACSegmentationCircle(cloud,
                             this->DistanceThreshold,
                             this->MaxIterations,
                             Eigen::Vector2d(this->RadiusLimit),
                             this->RadiusConstraintEnabled,
                             Eigen::Vector3d(this->ParallelLine),
                             this->ParallelLineConstraintEnabled,
                             modelCoefficients,
                             inlierIndices);

  if (modelCoefficients->values.size() != 7)
    {
    vtkErrorMacro("Error segmenting circle3d with RANSAC.");
    return 0;
    }

  // store circle coefficients
  for (size_t i = 0; i < 3; ++i)
    {
    this->CircleOrigin[i] = modelCoefficients->values[i];
    this->CircleNormal[i] = modelCoefficients->values[i+4];
    }
  this->CircleRadius = modelCoefficients->values[3];

  // pass thru input add labels
  vtkSmartPointer<vtkIntArray> labels = vtkPCLConversions::NewLabelsArray(inlierIndices, input->GetNumberOfPoints());
  labels->SetName("ransac_labels");
  output->ShallowCopy(input);
  output->GetPointData()->AddArray(labels);

  return 1;
}

//----------------------------------------------------------------------------
void vtkPCLSACSegmentationCircle::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os,indent);
}
