/*=========================================================================

  Program:   Visualization Toolkit
  Module:    vtkPCDReader.h

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
// .NAME vtkPCDReader -
// .SECTION Description
//

#ifndef __vtkPCDReader_h
#define __vtkPCDReader_h

#include <vtkPolyDataAlgorithm.h>
#include <vtkPCLFiltersModule.h>

class VTKPCLFILTERS_EXPORT vtkPCDReader : public vtkPolyDataAlgorithm
{
public:
  vtkTypeMacro(vtkPCDReader, vtkPolyDataAlgorithm);
  void PrintSelf(ostream& os, vtkIndent indent) VTKPCLFILTERS_OVERRIDE;

  static vtkPCDReader *New();

  vtkSetStringMacro(FileName);
  vtkGetStringMacro(FileName);

protected:

  char* FileName;

  virtual int RequestData(vtkInformation *request,
                          vtkInformationVector **inputVector,
                          vtkInformationVector *outputVector) VTKPCLFILTERS_OVERRIDE;


  vtkPCDReader();
  virtual ~vtkPCDReader() VTKPCLFILTERS_OVERRIDE;

private:
  vtkPCDReader(const vtkPCDReader&) VTKPCLFILTERS_DELETE_FUNCTION;
  void operator=(const vtkPCDReader&) VTKPCLFILTERS_DELETE_FUNCTION;
};

#endif


