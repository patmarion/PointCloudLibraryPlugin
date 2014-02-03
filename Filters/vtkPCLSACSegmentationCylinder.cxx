/*=========================================================================

  Program:   Visualization Toolkit
  Module:    vtkPCLSACSegmentationCylinder.cxx

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
#include "vtkPCLSACSegmentationCylinder.h"
#include "vtkPCLConversions.h"

#include "vtkPolyData.h"
#include "vtkPointData.h"
#include "vtkInformation.h"
#include "vtkInformationVector.h"
#include "vtkObjectFactory.h"
#include "vtkStreamingDemandDrivenPipeline.h"
#include "vtkSmartPointer.h"
#include "vtkNew.h"
#include "vtkAlgorithmOutput.h"
#include "vtkIdTypeArray.h"

#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

//----------------------------------------------------------------------------
namespace {


void ComputeSACSegmentationCylinder(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,
                                  double distanceThreshold,
                                  double radiusLimit,
                                  double searchRadius,
                                  double normalDistanceWeight,                              
                                  int maxIterations,
                                  pcl::ModelCoefficients::Ptr &modelCoefficients,
                                  pcl::PointIndices::Ptr &inliers)
  {

  // Data structs for surface normals, search tree, etc.
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
 
  // Compute surface normals
  ne.setSearchMethod (tree);
  ne.setInputCloud (cloud);
  ne.setRadiusSearch (searchRadius);
  ne.compute (*cloud_normals);

  // Cylinder RANSAC fitting only works with the SACSegmentation from normal
  pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg; 
  inliers = pcl::PointIndices::Ptr(new pcl::PointIndices);
  modelCoefficients = pcl::ModelCoefficients::Ptr(new pcl::ModelCoefficients);

  Eigen::Vector3f axis(0,0,1.0);

  // Perform RANSAC fitting
  seg.setOptimizeCoefficients (true);
  seg.setModelType(pcl::SACMODEL_CYLINDER);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(maxIterations);
  seg.setNormalDistanceWeight(normalDistanceWeight);
  seg.setRadiusLimits(0,radiusLimit);
  seg.setDistanceThreshold(distanceThreshold);
  //seg.setAxis(axis);
  //seg.setEpsAngle(0.2);

  seg.setInputCloud(cloud);
  seg.setInputNormals(cloud_normals);
  seg.segment(*inliers, *modelCoefficients);
  }

}

//----------------------------------------------------------------------------
vtkStandardNewMacro(vtkPCLSACSegmentationCylinder);

//----------------------------------------------------------------------------
vtkPCLSACSegmentationCylinder::vtkPCLSACSegmentationCylinder()
{
  this->RadiusLimit = 0.1;
  this->DistanceThreshold = 0.05;
  this->MaxIterations = 200;

  this->CylinderRadius = 0.0;
  for (size_t i = 0; i < 3; ++i)
    {
    this->CylinderOrigin[i] = 0.0;
    this->CylinderNormal[i] = 0.0;
    }

  this->SetNumberOfInputPorts(1);
  this->SetNumberOfOutputPorts(1);
}

//----------------------------------------------------------------------------
vtkPCLSACSegmentationCylinder::~vtkPCLSACSegmentationCylinder()
{
}

//----------------------------------------------------------------------------
int vtkPCLSACSegmentationCylinder::RequestData(
  vtkInformation* vtkNotUsed(request),
  vtkInformationVector **inputVector,
  vtkInformationVector *outputVector)
{
  // get input and output data objects
  vtkInformation *inInfo = inputVector[0]->GetInformationObject(0);
  vtkPolyData *input = vtkPolyData::SafeDownCast(inInfo->Get(vtkDataObject::DATA_OBJECT()));
  vtkInformation *outInfo = outputVector->GetInformationObject(0);
  vtkPolyData *output = vtkPolyData::SafeDownCast(outInfo->Get(vtkDataObject::DATA_OBJECT()));

  // perform cylinder model fit
  pcl::PointIndices::Ptr inlierIndices;
  pcl::ModelCoefficients::Ptr modelCoefficients;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = vtkPCLConversions::PointCloudFromPolyData(input);

  ComputeSACSegmentationCylinder(cloud,
    this->DistanceThreshold, 
    this->RadiusLimit,
    this->SearchRadius, 
    this->NormalDistanceWeight,
    this->MaxIterations,
    modelCoefficients, 
    inlierIndices);


  if (modelCoefficients->values.size() != 7)
    {
    vtkErrorMacro("Error segmenting cylinder with RANSAC.");
    return 0;
    }

  // store cylinder coefficients
  for (size_t i = 0; i < 3; ++i)
    {
    this->CylinderOrigin[i] = modelCoefficients->values[i];
    this->CylinderNormal[i] = modelCoefficients->values[i+3];
    }
  this->CylinderRadius = modelCoefficients->values[6];

  // pass thru input add labels
  vtkSmartPointer<vtkIntArray> labels = vtkPCLConversions::NewLabelsArray(inlierIndices, input->GetNumberOfPoints());
  labels->SetName("ransac_labels");
  output->ShallowCopy(input);
  output->GetPointData()->AddArray(labels);

  return 1;
}

//----------------------------------------------------------------------------
void vtkPCLSACSegmentationCylinder::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os,indent);
}
