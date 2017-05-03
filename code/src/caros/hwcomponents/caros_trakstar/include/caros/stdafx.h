// stdafx.h : include file for standard system include files,
//  or project specific include files that are used frequently, but
//      are changed infrequently
//
#ifndef CAROS_STDAFX_H
#define CAROS_STDAFX_H

#if !defined(AFX_STDAFX_H__4446E194_24C7_420A_85E4_E0E52DC5FF93__INCLUDED_)
#define AFX_STDAFX_H__4446E194_24C7_420A_85E4_E0E52DC5FF93__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif  // _MSC_VER > 1000

#ifdef _WIN32
#include <windows.h>
#include <stdio.h>   // printf
#include <string.h>  // string handling
#include <stdlib.h>  // exit() function
#include <time.h>    // needed for time functions
#endif

#ifdef LINUX
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <memory.h>
#include <time.h>
#include <sys/types.h>
#endif

#ifdef MAC
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <memory.h>
#include <math.h>
#include <time.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <pthread.h>
#include <errno.h>
#endif

//  {{AFX_INSERT_LOCATION}}
//  Microsoft Visual C++ will insert additional declarations immediately before the previous line.

#endif  // !defined(AFX_STDAFX_H__4446E194_24C7_420A_85E4_E0E52DC5FF93__INCLUDED_)

#endif  // CAROS_STDAFX_H
