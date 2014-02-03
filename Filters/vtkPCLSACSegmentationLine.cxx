/*=========================================================================

  Program:   Visualization Toolkit
  Module:    vtkPCLSACSegmentationLine.cxx

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
#include "vtkPCLSACSegmentationLine.h"
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

void ComputeSACSegmentationLine(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,
                                  double distanceThreshold,
                                  int maxIterations,
                                  pcl::ModelCoefficients::Ptr &modelCoefficients,
                                  pcl::PointIndices::Ptr &inliers)
{
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  inliers = pcl::PointIndices::Ptr(new pcl::PointIndices);
  modelCoefficients = pcl::ModelCoefficients::Ptr(new pcl::ModelCoefficients);

  seg.setOptimizeCoefficients (true);
  seg.setModelType(pcl::SACMODEL_LINE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(maxIterations);
  seg.setDistanceThreshold(distanceThreshold);

  seg.setInputCloud(cloud);
  seg.segment(*inliers, *modelCoefficients);
}

void ComputeSACSegmentationParallelLine(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,
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
  seg.setModelType(pcl::SACMODEL_PARALLEL_LINE);
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
vtkStandardNewMacro(vtkPCLSACSegmentationLine);

//----------------------------------------------------------------------------
vtkPCLSACSegmentationLine::vtkPCLSACSegmentationLine()
{
  this->DistanceThreshold = 0.05;
  this->MaxIterations = 200;

  for (size_t i = 0; i < 3; ++i)
    {
    this->LineOrigin[i] = 0.0;
    this->LineDirection[i] = 0.0;
    }

  this->ParallelLineConstraintEnabled = false;
  this->AngleEpsilon = 0.2;
  this->ParallelLine[0] = 1.0;
  this->ParallelLine[1] = 0.0;
  this->ParallelLine[2] = 0.0;

  this->SetNumberOfInputPorts(1);
  this->SetNumberOfOutputPorts(1);
}

//----------------------------------------------------------------------------
vtkPCLSACSegmentationLine::~vtkPCLSACSegmentationLine()
{
}

//----------------------------------------------------------------------------
int vtkPCLSACSegmentationLine::RequestData(
  vtkInformation* vtkNotUsed(request),
  vtkInformationVector **inputVector,
  vtkInformationVector *outputVector)
{
  // get input and output data objects
  vtkInformation *inInfo = inputVector[0]->GetInformationObject(0);
  vtkPolyData *input = vtkPolyData::SafeDownCast(inInfo->Get(vtkDataObject::DATA_OBJECT()));
  vtkInformation *outInfo = outputVector->GetInformationObject(0);
  vtkPolyData *output = vtkPolyData::SafeDownCast(outInfo->Get(vtkDataObject::DATA_OBJECT()));

  // perform line model fit
  pcl::PointIndices::Ptr inlierIndices;
  pcl::ModelCoefficients::Ptr modelCoefficients;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = vtkPCLConversions::PointCloudFromPolyData(input);

  if (this->ParallelLineConstraintEnabled)
    {

    ComputeSACSegmentationParallelLine(cloud,
                               this->DistanceThreshold,
                               Eigen::Vector3f(this->ParallelLine[0],
                                               this->ParallelLine[1],
                                               this->ParallelLine[2]),
                               this->AngleEpsilon,
                               this->MaxIterations,
                               modelCoefficients,
                               inlierIndices);
    }
  else
    {
    ComputeSACSegmentationLine(cloud,
                               this->DistanceThreshold,
                               this->MaxIterations,
                               modelCoefficients,
                               inlierIndices);
    }

  if (modelCoefficients->values.size() != 6)
    {
    vtkErrorMacro("Error segmenting line with RANSAC.");
    return 0;
    }

  // store line coefficients
  for (size_t i = 0; i < 3; ++i)
    {
    this->LineOrigin[i] = modelCoefficients->values[i];
    this->LineDirection[i] = modelCoefficients->values[i+3];
    }


  // pass thru input add labels
  vtkSmartPointer<vtkIntArray> labels = vtkPCLConversions::NewLabelsArray(inlierIndices, input->GetNumberOfPoints());
  labels->SetName("ransac_labels");
  output->ShallowCopy(input);
  output->GetPointData()->AddArray(labels);

  return 1;
}

//----------------------------------------------------------------------------
void vtkPCLSACSegmentationLine::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os,indent);
}
