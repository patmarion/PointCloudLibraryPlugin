/*=========================================================================

  Program:   Visualization Toolkit
  Module:    vtkPCLEigenCalculator.cxx

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
#include "vtkPCLEigenCalculator.h"
#include "vtkPCLConversions.h"

#include "vtkPolyData.h"
#include "vtkInformation.h"
#include "vtkInformationVector.h"
#include "vtkObjectFactory.h"
#include "vtkStreamingDemandDrivenPipeline.h"
#include "vtkSmartPointer.h"
#include "vtkNew.h"
#include "vtkFloatArray.h"
#include "vtkPointData.h"

#include "pcl_eigen_calculator.h"

#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>

//----------------------------------------------------------------------------
namespace {

pcl::PointCloud<pcl::Normal>::Ptr ComputeNormalEstimation(
      pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,
      pcl::PointCloud<pcl::PointXYZ>::ConstPtr searchCloud,
      double searchRadius)
{
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ> ());

  ne.setSearchMethod(tree);
  ne.setInputCloud(cloud);
  if (searchCloud)
    {
    ne.setSearchSurface(searchCloud);
    }

  ne.setRadiusSearch(searchRadius);
  ne.compute(*cloud_normals);
  return cloud_normals;
}


void ComputeEigenValues(vtkPolyData* polyData, vtkPolyData* searchCloud, double neighborhoodRadius)
{
  const vtkIdType nPoints = polyData->GetNumberOfPoints();

  pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud = vtkPCLConversions::PointCloudFromPolyData(polyData);

  pcl::EigenCalculator<pcl::PointXYZ, pcl::Normal> ec;
  ec.setInputCloud(pointCloud);


  if (searchCloud)
    {
    ec.setSearchSurface(vtkPCLConversions::PointCloudFromPolyData(searchCloud));
    }


  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
  ec.setSearchMethod(tree);

  pcl::PointCloud<pcl::Normal>::Ptr cloudNormals(new pcl::PointCloud<pcl::Normal>);

  vtkNew<vtkFloatArray> eigenVec0;
  vtkNew<vtkFloatArray> eigenVec1;
  vtkNew<vtkFloatArray> eigenVec2;
  vtkNew<vtkFloatArray> eigenValues;

  eigenVec0->SetName("eigen_vector_0");
  eigenVec0->SetNumberOfComponents(3);
  eigenVec0->SetNumberOfTuples(nPoints);
  polyData->GetPointData()->AddArray(eigenVec0.GetPointer());
  ec.eigen_vector_0 = eigenVec0.GetPointer();

  eigenVec1->SetName("eigen_vector_1");
  eigenVec1->SetNumberOfComponents(3);
  eigenVec1->SetNumberOfTuples(nPoints);
  polyData->GetPointData()->AddArray(eigenVec1.GetPointer());
  ec.eigen_vector_1 = eigenVec1.GetPointer();

  eigenVec2->SetName("eigen_vector_2");
  eigenVec2->SetNumberOfComponents(3);
  eigenVec2->SetNumberOfTuples(nPoints);
  polyData->GetPointData()->AddArray(eigenVec2.GetPointer());
  ec.eigen_vector_2 = eigenVec2.GetPointer();

  eigenValues->SetName("eigen_values");
  eigenValues->SetNumberOfComponents(3);
  eigenValues->SetNumberOfTuples(nPoints);
  polyData->GetPointData()->AddArray(eigenValues.GetPointer());
  ec.eigen_values = eigenValues.GetPointer();


  ec.setRadiusSearch(neighborhoodRadius);
  ec.compute(*cloudNormals);
}

}

//----------------------------------------------------------------------------
vtkStandardNewMacro(vtkPCLEigenCalculator);

//----------------------------------------------------------------------------
vtkPCLEigenCalculator::vtkPCLEigenCalculator()
{
  this->SearchRadius = 0.1;
  this->SetNumberOfInputPorts(2);
  this->SetNumberOfOutputPorts(1);
}

//----------------------------------------------------------------------------
vtkPCLEigenCalculator::~vtkPCLEigenCalculator()
{
}

//----------------------------------------------------------------------------
int vtkPCLEigenCalculator::FillInputPortInformation(int port, vtkInformation* info)
{
  if (port == 1)
    {
    info->Set(vtkAlgorithm::INPUT_IS_OPTIONAL(), 1);
    }

  return this->Superclass::FillInputPortInformation(port, info);
}

//----------------------------------------------------------------------------
int vtkPCLEigenCalculator::RequestData(
  vtkInformation* vtkNotUsed(request),
  vtkInformationVector **inputVector,
  vtkInformationVector *outputVector)
{

  vtkInformation *inInfo = inputVector[0]->GetInformationObject(0);
  vtkPolyData *input = vtkPolyData::SafeDownCast(inInfo->Get(vtkDataObject::DATA_OBJECT()));
  vtkInformation *outInfo = outputVector->GetInformationObject(0);
  vtkPolyData *output = vtkPolyData::SafeDownCast(outInfo->Get(vtkDataObject::DATA_OBJECT()));


  vtkInformation *searchInfo = inputVector[1]->GetInformationObject(0);
  vtkPolyData *searchCloudPolyData = 0;
  if (searchInfo)
    {
    searchCloudPolyData = vtkPolyData::SafeDownCast(searchInfo->Get(vtkDataObject::DATA_OBJECT()));
    }

  // early exit if input data has no points
  if (!input->GetNumberOfPoints())
    {
    return 1;
    }

  // pass input thru to output
  output->ShallowCopy(input);

  ComputeEigenValues(output, searchCloudPolyData, this->SearchRadius);

  return 1;
}

//----------------------------------------------------------------------------
void vtkPCLEigenCalculator::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os,indent);
}
