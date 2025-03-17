// opencl.h

#ifndef OPENCL_H
#define OPENCL_H

#include <stdio.h>
#include <CL/cl.h>

// When an OpenCL function fails, an error code is returned in the return value or in the argument 
// that stores the error, so I need a map to check this easily.It is very convenient when there is a black.
//
#define checkError(openclFunction)	        \
	if (cl_int err = (openclFunction))	    \
	{                                       \
		printf("error : %d\n", err);        \
	}

char* g_sourceString;
int g_set_gs = 0;

namespace OpenCL
{
	void initialize(int platformIdx, int deviceIdx, int plat, int dev);
	void detectLine(unsigned char* result, unsigned char* origin);
	void release();
	cl_program compileProgram(char* fileName);
	cl_kernel createKernel(cl_program program, char* kernelName);
}

#endif