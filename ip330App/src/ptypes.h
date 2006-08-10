#ifndef _INCLUDE_PTYPES_H_
#define _INCLUDE_PTYPES_H_

/****************************************************************/
/* $Id: ptypes.h,v 1.3 2006/08/10 16:18:26 pengs Exp $          */
/* Common definition for size sensetive variable                */
/* Author: Sheng Peng, pengs@slac.stanford.edu, 650-926-3847    */
/****************************************************************/

#ifndef VXWORKS

typedef unsigned int UINT32;
typedef unsigned short UINT16;
typedef signed short SINT16;
typedef unsigned char UINT8;

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

#endif

#endif
