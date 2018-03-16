/*=========================================================================

  Program:   Visualization Toolkit
  Module:    vtkPCLSACSegmentationCircle.h

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
// .NAME vtkPCLSACSegmentationCircle -
// .SECTION Description
//

#ifndef __vtkPCLSACSegmentationCircle_h
#define __vtkPCLSACSegmentationCircle_h

#include <vtkPolyDataAlgorithm.h>
#include <vtkPCLFiltersModule.h>

class VTKPCLFILTERS_EXPORT vtkPCLSACSegmentationCircle : public vtkPolyDataAlgorithm
{
public:
  vtkTypeMacro(vtkPCLSACSegmentationCircle, vtkPolyDataAlgorithm);
  void PrintSelf(ostream& os, vtkIndent indent);

  static vtkPCLSACSegmentationCircle *New();

  vtkSetMacro(DistanceThreshold, double);
  vtkGetMacro(DistanceThreshold, double);

  vtkSetMacro(MaxIterations, int);
  vtkGetMacro(MaxIterations, int);

  vtkGetVector3Macro(CircleOrigin, double);
  vtkGetVector3Macro(CircleNormal, double);
  vtkGetMacro(CircleRadius, double);

  vtkSetMacro(ParallelLineConstraintEnabled, bool);
  vtkGetMacro(ParallelLineConstraintEnabled, bool);

  vtkSetMacro(RadiusConstraintEnabled, bool);
  vtkGetMacro(RadiusConstraintEnabled, bool);

  vtkSetMacro(AngleEpsilon, double);
  vtkGetMacro(AngleEpsilon, double);

  vtkGetVector3Macro(ParallelLine, double);
  vtkSetVector3Macro(ParallelLine, double);

  vtkGetVector2Macro(RadiusLimit, double);
  vtkSetVector2Macro(RadiusLimit, double);

protected:

  double DistanceThreshold;
  int MaxIterations;

  bool ParallelLineConstraintEnabled;
  bool RadiusConstraintEnabled;
  double RadiusLimit[2];
  double ParallelLine[3];
  double AngleEpsilon;

  double CircleRadius;
  double CircleOrigin[3];
  double CircleNormal[3];

  virtual int RequestData(vtkInformation *request,
                          vtkInformationVector **inputVector,
                          vtkInformationVector *outputVector);


  vtkPCLSACSegmentationCircle();
  virtual ~vtkPCLSACSegmentationCircle();

private:
  vtkPCLSACSegmentationCircle(const vtkPCLSACSegmentationCircle&)
      VTKPCLFILTERS_DELETE_FUNCTION;
  void operator=(const vtkPCLSACSegmentationCircle&)
      VTKPCLFILTERS_DELETE_FUNCTION;
};

#endif


