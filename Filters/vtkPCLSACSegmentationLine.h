/*=========================================================================

  Program:   Visualization Toolkit
  Module:    vtkPCLSACSegmentationLine.h

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
// .NAME vtkPCLSACSegmentationLine -
// .SECTION Description
//

#ifndef __vtkPCLSACSegmentationLine_h
#define __vtkPCLSACSegmentationLine_h

#include <vtkPolyDataAlgorithm.h>
#include <vtkPCLFiltersModule.h>

class VTKPCLFILTERS_EXPORT vtkPCLSACSegmentationLine : public vtkPolyDataAlgorithm
{
public:
  vtkTypeMacro(vtkPCLSACSegmentationLine, vtkPolyDataAlgorithm);
  void PrintSelf(ostream& os, vtkIndent indent) VTKPCLFILTERS_OVERRIDE;

  static vtkPCLSACSegmentationLine *New();

  vtkSetMacro(DistanceThreshold, double);
  vtkGetMacro(DistanceThreshold, double);

  vtkSetMacro(MaxIterations, int);
  vtkGetMacro(MaxIterations, int);

  vtkGetVector3Macro(LineOrigin, double);
  vtkGetVector3Macro(LineDirection, double);

  vtkSetMacro(ParallelLineConstraintEnabled, bool);
  vtkGetMacro(ParallelLineConstraintEnabled, bool);

  vtkSetMacro(AngleEpsilon, double);
  vtkGetMacro(AngleEpsilon, double);

  vtkGetVector3Macro(ParallelLine, double);
  vtkSetVector3Macro(ParallelLine, double);

protected:

  double DistanceThreshold;
  int MaxIterations;

  bool ParallelLineConstraintEnabled;
  double ParallelLine[3];
  double AngleEpsilon;

  double LineOrigin[3];
  double LineDirection[3];

  virtual int RequestData(vtkInformation *request,
                          vtkInformationVector **inputVector,
                          vtkInformationVector *outputVector) VTKPCLFILTERS_OVERRIDE;


  vtkPCLSACSegmentationLine();
  virtual ~vtkPCLSACSegmentationLine() VTKPCLFILTERS_OVERRIDE;

private:
  vtkPCLSACSegmentationLine(const vtkPCLSACSegmentationLine&)
      VTKPCLFILTERS_DELETE_FUNCTION;
  void operator=(const vtkPCLSACSegmentationLine&)
      VTKPCLFILTERS_DELETE_FUNCTION;
};

#endif


