/*=========================================================================

  Program:   Visualization Toolkit
  Module:    vtkPCLEigenCalculator.h

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
// .NAME vtkPCLEigenCalculator -
// .SECTION Description
//

#ifndef __vtkPCLEigenCalculator_h
#define __vtkPCLEigenCalculator_h

#include <vtkPolyDataAlgorithm.h>
#include <vtkPCLFiltersModule.h>

class VTKPCLFILTERS_EXPORT vtkPCLEigenCalculator : public vtkPolyDataAlgorithm
{
public:
  vtkTypeMacro(vtkPCLEigenCalculator, vtkPolyDataAlgorithm);
  void PrintSelf(ostream& os, vtkIndent indent);

  static vtkPCLEigenCalculator *New();

  vtkSetMacro(SearchRadius, double);
  vtkGetMacro(SearchRadius, double);


protected:

  double SearchRadius;

  virtual int RequestData(vtkInformation *request,
                          vtkInformationVector **inputVector,
                          vtkInformationVector *outputVector);

  vtkPCLEigenCalculator();
  virtual ~vtkPCLEigenCalculator();


  virtual int FillInputPortInformation(int port, vtkInformation* info);

private:
  vtkPCLEigenCalculator(const vtkPCLEigenCalculator&);  // Not implemented.
  void operator=(const vtkPCLEigenCalculator&);  // Not implemented.
};

#endif
