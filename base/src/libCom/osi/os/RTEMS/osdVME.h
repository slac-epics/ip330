/*************************************************************************\
* Copyright (c) 2002 The University of Chicago, as Operator of Argonne
*     National Laboratory.
* Copyright (c) 2002 The Regents of the University of California, as
*     Operator of Los Alamos National Laboratory.
* EPICS BASE Versions 3.13.7
* and higher are distributed subject to a Software License Agreement found
* in file LICENSE that is included with this distribution. 
\*************************************************************************/
/* osdVME.h,v 1.1.2.3 2006/09/19 16:18:26 lange Exp */

/*
 * OS-dependent VME support
 */
#ifndef __i386__
#ifndef __mc68000
#ifndef __arm__
#include <bsp/VME.h>
#endif
#endif
#endif