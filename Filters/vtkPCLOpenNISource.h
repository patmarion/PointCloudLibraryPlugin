/*=========================================================================

  Program:   Visualization Toolkit
  Module:    vtkPCLOpenNISource.h

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
// .NAME vtkPCLOpenNISource -
// .SECTION Description
//

#ifndef __vtkPCLOpenNISource_h
#define __vtkPCLOpenNISource_h

#include <vtkPolyDataAlgorithm.h>
#include <vtkPCLFiltersModule.h>

class VTKPCLFILTERS_EXPORT vtkPCLOpenNISource : public vtkPolyDataAlgorithm
{
public:
  vtkTypeMacro(vtkPCLOpenNISource, vtkPolyDataAlgorithm);
  void PrintSelf(ostream& os, vtkIndent indent) VTKPCLFILTERS_OVERRIDE;

  static vtkPCLOpenNISource *New();

  bool HasNewData();

  void Poll();

  void StartGrabber();
  void StopGrabber();

protected:

  virtual int RequestData(vtkInformation *request,
                          vtkInformationVector **inputVector,
                          vtkInformationVector *outputVector) VTKPCLFILTERS_OVERRIDE;

  vtkPCLOpenNISource();
  virtual ~vtkPCLOpenNISource() VTKPCLFILTERS_OVERRIDE;

private:
  vtkPCLOpenNISource(const vtkPCLOpenNISource&) VTKPCLFILTERS_DELETE_FUNCTION;
  void operator=(const vtkPCLOpenNISource&) VTKPCLFILTERS_DELETE_FUNCTION;

  class vtkInternal;
  vtkInternal * Internal;
};

#endif


