/*=========================================================================

  Program:   Visualization Toolkit
  Module:    vtkPCLSACSegmentationCylinder.h

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
// .NAME vtkPCLSACSegmentationCylinder -
// .SECTION Description
//

#ifndef __vtkPCLSACSegmentationCylinder_h
#define __vtkPCLSACSegmentationCylinder_h

#include <vtkPolyDataAlgorithm.h>
#include <vtkPCLFiltersModule.h>

class VTKPCLFILTERS_EXPORT vtkPCLSACSegmentationCylinder : public vtkPolyDataAlgorithm
{
public:
  vtkTypeMacro(vtkPCLSACSegmentationCylinder, vtkPolyDataAlgorithm);
  void PrintSelf(ostream& os, vtkIndent indent) VTKPCLFILTERS_OVERRIDE;

  static vtkPCLSACSegmentationCylinder *New();

  vtkSetMacro(DistanceThreshold, double);
  vtkGetMacro(DistanceThreshold, double);

  vtkSetMacro(MaxIterations, int);
  vtkGetMacro(MaxIterations, int);

  vtkSetMacro(RadiusLimit, double);
  vtkGetMacro(RadiusLimit, double);

  vtkSetMacro(SearchRadius, double);
  vtkGetMacro(SearchRadius, double);

  vtkSetMacro(NormalDistanceWeight, double);
  vtkGetMacro(NormalDistanceWeight, double);

  vtkGetVector3Macro(CylinderOrigin, double);
  vtkGetVector3Macro(CylinderNormal, double);
  vtkGetMacro(CylinderRadius, double);

protected:

  double NormalDistanceWeight;
  double DistanceThreshold;
  double RadiusLimit;
  double SearchRadius;
  int MaxIterations;

  double CylinderRadius;
  double CylinderOrigin[3];
  double CylinderNormal[3];

  virtual int RequestData(vtkInformation *request,
                          vtkInformationVector **inputVector,
                          vtkInformationVector *outputVector) VTKPCLFILTERS_OVERRIDE;
  vtkPCLSACSegmentationCylinder();
  virtual ~vtkPCLSACSegmentationCylinder() VTKPCLFILTERS_OVERRIDE;

private:
  vtkPCLSACSegmentationCylinder(const vtkPCLSACSegmentationCylinder&)
      VTKPCLFILTERS_DELETE_FUNCTION;
  void operator=(const vtkPCLSACSegmentationCylinder&)
      VTKPCLFILTERS_DELETE_FUNCTION;
};

#endif


