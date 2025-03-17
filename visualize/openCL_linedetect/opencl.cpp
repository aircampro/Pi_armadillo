#include <stdio.h>
#include <malloc.h>
#include <CL/cl.h>
#include "opencl.h"

cl_device_id gDevice;
cl_command_queue gCommandQueue;
cl_context gContext;

cl_program gProgram;
cl_kernel gKernel;

cl_mem gResult;
cl_mem gOrigin;

/*
  OpenCL The first thing you need to do to use OpenCL is to specify the device that executes the kernel function, 
  and to specify the context and kernel for memory retrieval.Is to get the command queue for execution.
  The platform and device on which the kernel functions are executed are specified by the index value.
*/
void OpenCL::initialize(int platformIdx, int deviceIdx, int plat, int dev)
{
	// platform
	cl_uint platformNumber = plat;
	cl_platform_id platformIds[8];
	checkError(clGetPlatformIDs(8, platformIds, &platformNumber));
	cl_platform_id platform = platformIds[platformIdx];

	// device
	cl_uint deviceNumber = dev;
	cl_device_id deviceIds[8];
	checkError(clGetDeviceIDs(platform, CL_DEVICE_TYPE_ALL, 8, deviceIds, &deviceNumber));
	gDevice = deviceIds[deviceIdx];

	// Create context (used for memory allocation, etc.) and command queue (used for kernel execution, etc.)
	gContext = clCreateContext(NULL, 1, &gDevice, NULL, NULL, NULL);
	gCommandQueue = clCreateCommandQueue(gContext, gDevice, 0, NULL);

	// Once you have the necessary environment to run the kernel, you will need to compile the kernel program
	gProgram = compileProgram("kernel.cl");
	gKernel = createKernel(gProgram, "detectLine");

	// Allocate memory objects for use by the device
	// Since the address space is different between the CPU side and the device side (physical memory may be different),
    // it is possible to use the data reserved by the CPU side as it is
	gResult = clCreateBuffer(gContext, CL_MEM_READ_WRITE, sizeof(char) * 640 * 480, NULL, NULL);
	gOrigin = clCreateBuffer(gContext, CL_MEM_READ_WRITE, sizeof(char) * 640 * 480 * 3, NULL, NULL);
}

/*
OpenCL Calls the kernel to perform the calculation
It then writes and reads data to the memory object obtained by clCreateBuffer
*/
void OpenCL::detectLine(unsigned char* result, unsigned char* origin)
{
	cl_int2 size = { 640, 480 };
	// Host to device memory transfer
	// write and read data to the memory object obtained by clCreateBuffer
	checkError(clEnqueueWriteBuffer(gCommandQueue, gOrigin, CL_TRUE, 0, sizeof(char) * 640 * 480 * 3, origin, 0, NULL, NULL));
	// Set memory objects as arguments to kernel functions
	// Set the kernel object argument to a memory object
    // Set kernel execution size
    // Kernel execution
	checkError(clSetKernelArg(gKernel, 0, sizeof(cl_mem), &gResult));
	checkError(clSetKernelArg(gKernel, 1, sizeof(cl_mem), &gOrigin));
	checkError(clSetKernelArg(gKernel, 2, sizeof(cl_int2), &size));
	// Set the number of kernel parallel executions
	size_t workSize[2] = { 640, 480 };
	// Calling the kernel
	checkError(clEnqueueNDRangeKernel(gCommandQueue, gKernel, 2, NULL, workSize, NULL, 0, NULL, NULL));
	// Memory transfer from device to host
	checkError(clEnqueueReadBuffer(gCommandQueue, gResult, CL_TRUE, 0, sizeof(char) * 640 * 480, result, 0, NULL, NULL));
}


void OpenCL::release()
{
	clFinish(gCommandQueue);
	// free the memory objects.
	clReleaseMemObject(gResult);
	clReleaseMemObject(gOrigin);
	clReleaseCommandQueue(gCommandQueue);
	clReleaseContext(gContext);
}

/*
   OpenCL Functions for quick compilation and retrieval of kernel function objects.
*/
cl_program OpenCL::compileProgram(char* fileName)
{
	// read the kernel file
	FILE* fp;
	fp = fopen(fileName, "r");
	if (fp == NULL)
	{
		printf("%s load failed\n", fileName);
		return NULL;
	}

	fseek(fp, 0, SEEK_END);
	const int filesize = ftell(fp);

	fseek(fp, 0, 0);
	g_sourceString = (char*)malloc(filesize);
	g_set_gs = 1;
	size_t sourceSize = fread(g_sourceString, sizeof(char), filesize, fp);
	fclose(fp);

	// Compile the program
	cl_program program = clCreateProgramWithSource(gContext, 1, (const char**)&g_sourceString, (const size_t*)&sourceSize, NULL);
	cl_int err = clBuildProgram(program, 1, &gDevice, NULL, NULL, NULL);
	// Display error message if compilation fails
	if (err != CL_SUCCESS)
	{
		size_t logSize;
		clGetProgramBuildInfo(program, gDevice, CL_PROGRAM_BUILD_LOG, 0, NULL, &logSize);
		char* buildLog = (char*)malloc((logSize + 1));
		clGetProgramBuildInfo(program, gDevice, CL_PROGRAM_BUILD_LOG, logSize, buildLog, NULL);
		printf("%s", buildLog);
		free(buildLog);
	}
	free(g_sourceString);
    g_set_gs = 0;
	return program;
}

/*
   Create a kernel object from a program object.
*/
cl_kernel OpenCL::createKernel(cl_program program, char* kernelName)
{
	return clCreateKernel(program, kernelName, NULL);
}