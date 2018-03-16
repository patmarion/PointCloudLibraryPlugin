/*=========================================================================

  Program:   Visualization Toolkit
  Module:    vtkPCLRadiusOutlierRemoval.h

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
// .NAME vtkPCLRadiusOutlierRemoval -
// .SECTION Description
//

#ifndef __vtkPCLRadiusOutlierRemoval_h
#define __vtkPCLRadiusOutlierRemoval_h

#include <vtkPolyDataAlgorithm.h>
#include <vtkPCLFiltersModule.h>

class VTKPCLFILTERS_EXPORT vtkPCLRadiusOutlierRemoval : public vtkPolyDataAlgorithm
{
public:
  vtkTypeMacro(vtkPCLRadiusOutlierRemoval, vtkPolyDataAlgorithm);
  void PrintSelf(ostream& os, vtkIndent indent) VTKPCLFILTERS_OVERRIDE;

  static vtkPCLRadiusOutlierRemoval *New();

  vtkSetMacro(SearchRadius, double);
  vtkGetMacro(SearchRadius, double);

  vtkSetMacro(NeighborsInSearchRadius, int);
  vtkGetMacro(NeighborsInSearchRadius, int);

protected:

  double SearchRadius;
  int NeighborsInSearchRadius;

  virtual int RequestData(vtkInformation *request,
                          vtkInformationVector **inputVector,
                          vtkInformationVector *outputVector) VTKPCLFILTERS_OVERRIDE;

  vtkPCLRadiusOutlierRemoval();
  virtual ~vtkPCLRadiusOutlierRemoval() VTKPCLFILTERS_OVERRIDE;

private:
  vtkPCLRadiusOutlierRemoval(const vtkPCLRadiusOutlierRemoval&)
      VTKPCLFILTERS_DELETE_FUNCTION;
  void operator=(const vtkPCLRadiusOutlierRemoval&)
      VTKPCLFILTERS_DELETE_FUNCTION;
};

#endif


