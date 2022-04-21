#ifndef     __COMDEF_H__
#define     __COMDEF_H__
// ****** Common definitions and typedefs **********************************
typedef unsigned char       u8;
typedef unsigned short      u16;
typedef unsigned int       u32;
typedef unsigned int       ui32;

#if defined(WIN32)
typedef unsigned __int64     u64;
#elif defined(_ARMV7_)
typedef unsigned long long  u64;
#else
typedef unsigned long long  u64;
#endif  //WIN32


typedef signed char         s8;
typedef short               s16;
typedef int                 s32;


#if defined(WIN32)
typedef _int64              s64;
#elif defined(_ARMV7_)
typedef long long           s64;
#else
typedef long long           s64;
#endif  //WIN32

typedef u8 *                pu8;
typedef u16 *               pu16;
typedef u32 *               pu32;
typedef u64 *               pu64;

typedef s8 *                ps8;
typedef s16 *               ps16;
typedef s32 *               ps32;
typedef s64 *               ps64;

#if !defined(__cplusplus) 
typedef unsigned int bool;
#endif
typedef unsigned int        aubool;


// ========== types.h ===========
#ifndef true
#define true (0 == 0)
#endif
#ifndef false
#define false (!true)
#endif

#ifndef TRUE
#define TRUE (0 == 0)
#endif
#ifndef FALSE
#define FALSE (!TRUE)
#endif



#ifndef NULL
#define NULL        0
#endif //

#endif      // __COMDEF_H__
