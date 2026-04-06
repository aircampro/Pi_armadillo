//-----------------------------------------------------------------------------
// name: calories_def.h
// desc: defines for calories
//
// author: Ge Wang (ge@ccrma.stanford.edu)
//   date: 2008.1.15
//-----------------------------------------------------------------------------
#ifndef __CALORIES_DEF_H__
#define __CALORIES_DEF_H__

#include <stdlib.h>
#include <memory.h>
#include <assert.h>

// bool
#ifndef TRUE
#define TRUE                        1
#define FALSE                       0
#endif

#define ONE_PI (3.14159265358979323846)
#define TWO_PI (2.0 * ONE_PI)
#define SQRT2  (1.41421356237309504880)

#ifndef SAFE_DELETE
#define SAFE_DELETE(x)              do { if(x){ delete x; x = NULL; } } while(0)
#define SAFE_DELETE_ARRAY(x)        do { if(x){ delete [] x; x = NULL; } } while(0)
#endif

#ifdef __MACOSX_CORE__
#define __PLATFORM_MACOSX__
#endif

#if defined(__LINUX_ALSA__) || defined(__LINUX_JACK__) || defined(__LINUX_OSS__) 
#define __PLATFORM_LINUX__
#endif

#ifdef __PLATFORM_WIN32__
#include <process.h>
  #ifndef usleep
  #define usleep(x) Sleep( (x / 1000 <= 0 ? 1 : x / 1000) )
  #endif
#pragma warning (disable : 4996)  // stdio deprecation
#pragma warning (disable : 4786)  // stl debug info
#pragma warning (disable : 4312)  // type casts from void*
#pragma warning (disable : 4311)  // type casts to void*
#pragma warning (disable : 4244)  // truncation
#pragma warning (disable : 4068)  // unknown pragma
#else
#include <unistd.h>
#endif


#endif