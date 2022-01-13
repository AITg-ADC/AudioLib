#ifndef _NTK_BASIC_OP_
#define _NTK_BASIC_OP_

#define NTK_INTRINSICS_VERSION     "1.0 [20160908]"

//Integer
typedef signed char          NTK_SINT8;
typedef signed short         NTK_SINT16;
typedef signed int           NTK_SINT32;
typedef signed long long     NTK_SINT64;
typedef unsigned char        NTK_UINT8;
typedef unsigned short       NTK_UINT16;
typedef unsigned int         NTK_UINT32;
typedef unsigned long long   NTK_UINT64;

//Fraction
typedef short       NTK_FRACT16;            //fraction [1.15]
typedef int         NTK_FRACT32;            //fraction [1.31]
typedef int         NTK_FRACT24;            //fraction [9.23]
typedef int         NTK_ACCU32;
typedef long long   NTK_ACCU64;

//Internal defination
#define _NTK_SFRACT_MANTBITS 15
#define _NTK_DSP24_MANTBITS  23
#define _NTK_LFRACT_MANTBITS 31

#define _NTK_FRACT_1(mantbits)  ((1u<<(mantbits)) & (~0u))
#define _NTK_FRACT_MAX(mantbits) (NTK_SINT32)(_NTK_FRACT_1((mantbits)) - 1)
#define _NTK_FRACT_MIN(mantbits) (-_NTK_FRACT_MAX((mantbits)) - 1)

#define _NTK_mul64(a,b,n) (((NTK_SINT64)a*b)>>n)
#define _NTK_add(a,b) (a+b)
#define _NTK_sub(a,b) (a-b)
#if defined (_WIN32)
#define ntk_inline __inline
#elif defined (__arm) && defined (__ARMCC_VERSION)
#define ntk_inline __inline
#elif defined(__GNUC__) && defined(__arm__)
#define ntk_inline __inline
#elif defined(__XTENSA__)
#define ntk_inline inline
#elif defined (ARMV8)
#define ntk_inline __inline
#elif defined(_MIPS_)
#define ntk_inline __inline
#else
#define ntk_inline __inline
//#error Unsupported platform!!!
#endif



//Max&Min
#define NTK_FRACT16_MAX (NTK_FRACT16)(_NTK_FRACT_MAX(15))
#define NTK_FRACT16_MIN (NTK_FRACT16)(_NTK_FRACT_MIN(15))

#define NTK_FRACT24_MAX (NTK_FRACT24)(_NTK_FRACT_MAX(23))
#define NTK_FRACT24_MIN (NTK_FRACT24)(_NTK_FRACT_MIN(23))

#define NTK_FRACT32_MAX (NTK_FRACT32)(_NTK_FRACT_MAX(31))
#define NTK_FRACT32_MIN (NTK_FRACT32)(_NTK_FRACT_MIN(31))

#define NTK_FRACT_MAX(mant) (NTK_FRACT32)(_NTK_FRACT_MAX(mant))
#define NTK_FRACT_MIN(mant) (NTK_FRACT32)(_NTK_FRACT_MIN(mant))


/******************************************************************************
*                             Function Declaration
******************************************************************************/
//Abs,Negation
static ntk_inline NTK_FRACT32 NTK_SatAbs32(NTK_FRACT32 a);  //saturate the result to 32 bits
static ntk_inline NTK_FRACT32 NTK_SatNeg32(NTK_FRACT32 a);  //saturate the result to 32 bits
static ntk_inline NTK_FRACT24 NTK_SatAbs24(NTK_FRACT24 a);  //saturate the result to 24 bits
static ntk_inline NTK_FRACT24 NTK_SatNeg24(NTK_FRACT24 a);  //saturate the result to 24 bits
static ntk_inline NTK_FRACT16 NTK_SatAbs16(NTK_FRACT16 a);  //saturate the result to 16 bits
static ntk_inline NTK_FRACT16 NTK_SatNeg16(NTK_FRACT16 a);  //saturate the result to 16 bits

//Saturate,Shift,Normalize
static ntk_inline NTK_SINT32  NTK_Norm32(NTK_FRACT32 a);
static ntk_inline NTK_SINT32  NTK_Norm24(NTK_FRACT32 a);
static ntk_inline NTK_SINT32  NTK_Norm16(NTK_FRACT16 a);
static ntk_inline NTK_SINT32  NTK_RoundShiftR32(NTK_SINT32 a,NTK_SINT32 b);
static ntk_inline NTK_SINT16  NTK_SatRoundShiftR32to16(NTK_SINT32 a,NTK_SINT32 b);
static ntk_inline NTK_SINT32  NTK_SatShiftL32(NTK_SINT32 a,NTK_SINT32 b);
static ntk_inline NTK_SINT16  NTK_SatShiftL16(NTK_SINT16 a,NTK_SINT32 b);
static ntk_inline NTK_SINT32  NTK_Sat32(NTK_SINT64 a);
static ntk_inline NTK_SINT32  NTK_Sat24(NTK_SINT32 a);
static ntk_inline NTK_SINT16  NTK_Sat16(NTK_SINT32 a);
static ntk_inline NTK_SINT32  NTK_Min(NTK_SINT32 a,NTK_SINT32 b);
static ntk_inline NTK_SINT32  NTK_Max(NTK_SINT32 a,NTK_SINT32 b);

//Add,Subtract
static ntk_inline NTK_FRACT32 NTK_SatAdd32(NTK_FRACT32 a,NTK_FRACT32 b);
static ntk_inline NTK_FRACT32 NTK_SatSub32(NTK_FRACT32 a,NTK_FRACT32 b);
static ntk_inline NTK_FRACT24 NTK_SatAdd24(NTK_FRACT24 a,NTK_FRACT24 b);
static ntk_inline NTK_FRACT24 NTK_SatSub24(NTK_FRACT24 a,NTK_FRACT24 b);
static ntk_inline NTK_FRACT16 NTK_SatAdd16(NTK_FRACT16 a,NTK_FRACT16 b);
static ntk_inline NTK_FRACT16 NTK_SatSub16(NTK_FRACT16 a,NTK_FRACT16 b);

//Multiply
static ntk_inline NTK_FRACT32 NTK_Mul32X32_RShiftN(NTK_FRACT32 a,NTK_FRACT32 b,NTK_SINT32 n);
static ntk_inline NTK_FRACT32 NTK_Mul32X32_RoundShiftN(NTK_FRACT32 a,NTK_FRACT32 b,NTK_SINT32 n);//n>16 for xtensa
static ntk_inline NTK_FRACT32 NTK_FpMul32X32(NTK_FRACT32 a,NTK_FRACT32 b);
static ntk_inline NTK_FRACT32 NTK_Mul32X32_RShift32(NTK_FRACT32 a,NTK_FRACT32 b);
static ntk_inline NTK_ACCU32  NTK_FpMac32X32(NTK_ACCU32 accu,NTK_FRACT32 b,NTK_FRACT32 c);
static ntk_inline NTK_ACCU32  NTK_FpMas32X32(NTK_ACCU32 accu,NTK_FRACT32 b,NTK_FRACT32 c);
static ntk_inline NTK_FRACT32 NTK_FpMul32X16(NTK_FRACT32 a,NTK_FRACT16 b);
static ntk_inline NTK_FRACT32 NTK_Mul32X16_RShift16(NTK_FRACT32 a,NTK_FRACT16 b);
static ntk_inline NTK_ACCU32  NTK_FpMac32X16(NTK_ACCU32 accu,NTK_FRACT32 b,NTK_FRACT16 c);
static ntk_inline NTK_ACCU32  NTK_FpMas32X16(NTK_ACCU32 accu,NTK_FRACT32 b,NTK_FRACT16 c);
static ntk_inline NTK_FRACT32 NTK_FpMul32X16RoundShift16(NTK_FRACT32 a, NTK_FRACT16 b);
static ntk_inline NTK_FRACT32 NTK_FpMul32X16RoundShift15(NTK_FRACT32 a, NTK_FRACT16 b);
static ntk_inline NTK_FRACT24 NTK_FpMul24X24(NTK_FRACT24 a,NTK_FRACT24 b);
static ntk_inline NTK_ACCU32  NTK_FpMac24X24(NTK_ACCU32 accu,NTK_FRACT24 b,NTK_FRACT24 c);
static ntk_inline NTK_ACCU32  NTK_FpMas24X24(NTK_ACCU32 accu,NTK_FRACT24 b,NTK_FRACT24 c);
static ntk_inline NTK_FRACT24 NTK_SatFpMul24X24(NTK_FRACT24 a,NTK_FRACT24 b);
static ntk_inline NTK_ACCU32  NTK_SatFpMac24X24(NTK_ACCU32 accu,NTK_FRACT24 b,NTK_FRACT24 c);
static ntk_inline NTK_ACCU32  NTK_SatFpMas24X24(NTK_ACCU32 accu,NTK_FRACT24 b,NTK_FRACT24 c);
static ntk_inline NTK_FRACT16 NTK_FpMul16X16(NTK_FRACT16 a,NTK_FRACT16 b);
static ntk_inline NTK_ACCU32  NTK_FpMac16X16(NTK_ACCU32 accu,NTK_FRACT16 b,NTK_FRACT16 c);
static ntk_inline NTK_ACCU32  NTK_FpMas16X16(NTK_ACCU32 accu,NTK_FRACT16 b,NTK_FRACT16 c);
static ntk_inline NTK_FRACT32 NTK_Mul16X16_D(NTK_FRACT16 a,NTK_FRACT16 b);
static ntk_inline NTK_ACCU32 NTK_Mac16X16_D(NTK_ACCU32 accu,NTK_FRACT16 b,NTK_FRACT16 c);
static ntk_inline NTK_ACCU32 NTK_Mas16X16_D(NTK_ACCU32 accu,NTK_FRACT16 b,NTK_FRACT16 c);
/******************************************************************************
*                             End of Function Declaration
******************************************************************************/

#if defined (_WIN32) 
#include "backend_genreral.h"
#elif defined (ARMV8)
#include "backend_genreral.h"
#elif defined (__arm) && defined (__ARMCC_VERSION)
#include "backend_armcc.h"
#elif defined(__GNUC__) && defined(__arm__)
#include "backend_armgnu.h"
#elif defined(__XTENSA__)
#include "backend_xtensa.h"
#elif defined(_MIPS_)
#include "backend_genreral.h"
#else
//#error Unsupported platform!!!
#include "backend_armgnu.h"
#endif


#endif  //#ifndef _NTK_BASIC_OP_
