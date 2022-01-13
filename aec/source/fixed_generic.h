/* Copyright (C) 2003 Jean-Marc Valin */
/**
@file fixed_generic.h
@brief Generic fixed-point operations
*/
/*
Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions
are met:

- Redistributions of source code must retain the above copyright
notice, this list of conditions and the following disclaimer.

- Redistributions in binary form must reproduce the above copyright
notice, this list of conditions and the following disclaimer in the
documentation and/or other materials provided with the distribution.

- Neither the name of the Xiph.org Foundation nor the names of its
contributors may be used to endorse or promote products derived from
this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE FOUNDATION OR
CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef FIXED_GENERIC_H
#define FIXED_GENERIC_H

#include "ntk_opt_switch.h"

#define QCONST16(x,bits) ((spx_word16_t)(.5+(x)*(((spx_word32_t)1)<<(bits))))
#define QCONST32(x,bits) ((spx_word32_t)(.5+(x)*(((spx_word32_t)1)<<(bits))))

#define NEG16(x) (-(x))
#define NEG32(x) (-(x))
//#define NTK_EXTRACT16(x) ((spx_word16_t)(x))
#define EXTEND32(x) ((spx_word32_t)(x))
#define SHR16(a,shift) ((a) >> (shift))
#define SHR32(a,shift) ((a) >> (shift))

static inline s16 EXTRACT16(int x) 
{
    if(x>>15 != x>>31)
    {
        if(x>>31)
            return 0x8000;
        else
            return 0x7FFF;
    }
    return (s16)x;
}
static inline s16 EXTRACT16_FLAG(int x, s16 *flag) 
{
    if(x>>15 != x>>31)
    {
    	*flag = 1;
        if(x>>31)
            return 0x8000;
        else
            return 0x7FFF;
    }
    return (s16)x;
}

#ifdef _MIPS_
#define SHL16(a,shift) SHLLV_S_PH(a,shift)
#define SHL32(a,shift) SHLLV_S_W(a,shift)
#else
//#define NTK_SHL32(a,shift) ((a) << (shift))
static inline int SHL32(int y, int shift) 
{
    int sign = (y) >> 31; 
    if (sign != (y) >> (31-shift)) 
    {
        //		(y) = sign ^ ((1 << (31)) - 1);
        (y) = sign ^ 0x7FFFFFFF;
    }
    else
    {
        y <<= shift;
    }
    return(y);
}
//#define NTK_SHL16(a,shift) ((a) << (shift))
static inline short int SHL16(short int y, int shift) 
{
    int sign = (y) >> 15; 
    if (sign != (y) >> (15-shift)) 
    {
        (y) = sign ^ ((1 << (15)) - 1);
    }
    else
    {
        y <<= shift;
    }
    return((short int) y);
}
#endif

#define PSHR16(a,shift) (SHR16((a)+((1<<((shift))>>1)),shift))
#define VSHR32(a, shift) (((shift)>0) ? SHR32(a, shift) : NTK_SHL32(a, -(shift)))
#define SATURATE16(x,a) (((x)>(a) ? (a) : (x)<-(a) ? -(a) : (x)))
#define SATURATE32(x,a) (((x)>(a) ? (a) : (x)<-(a) ? -(a) : (x)))

#define SHR(a,shift) ((a) >> (shift))
#define SHL(a,shift) ((spx_word32_t)(a) << (shift))
#define PSHR(a,shift) (SHR((a)+((EXTEND32(1)<<((shift))>>1)),shift))
#define SATURATE(x,a) (((x)>(a) ? (a) : (x)<-(a) ? -(a) : (x)))

#ifdef _ADD_SUB_SATURATE
#define ADD16(a,b)	NTK_add16(a,b)
#define ADD32(a,b)	NTK_add32(a,b)
#define SUB16(a,b)	NTK_sub16(a,b)
#define SUB32(a,b)	NTK_sub32(a,b)
#else
#define ADD16(a,b) ((spx_word16_t)((spx_word16_t)(a)+(spx_word16_t)(b)))
#define SUB16(a,b) ((spx_word16_t)(a)-(spx_word16_t)(b))
#define ADD32(a,b) ((spx_word32_t)(a)+(spx_word32_t)(b))
#define SUB32(a,b) ((spx_word32_t)(a)-(spx_word32_t)(b))
#endif

/* result fits in 16 bits */
#define MULT16_16_16(a,b)     ((((spx_word16_t)(a))*((spx_word16_t)(b))))

/* (spx_word32_t)(spx_word16_t) gives TI compiler a hint that it's 16x16->32 multiply */
#define MULT16_16(a,b)     (((spx_word32_t)(spx_word16_t)(a))*((spx_word32_t)(spx_word16_t)(b)))
#define MAC16_16(c,a,b) (ADD32((c),MULT16_16((a),(b))))
#define MULT16_32_Q12(a,b) ADD32(MULT16_16((a),SHR((b),12)), SHR(MULT16_16((a),((b)&0x00000fff)),12))
#define MULT16_32_Q13(a,b) ADD32(MULT16_16((a),SHR((b),13)), SHR(MULT16_16((a),((b)&0x00001fff)),13))
#define MULT16_32_Q14(a,b) ADD32(MULT16_16((a),SHR((b),14)), SHR(MULT16_16((a),((b)&0x00003fff)),14))

#define MULT16_32_Q11(a,b) ADD32(MULT16_16((a),SHR((b),11)), SHR(MULT16_16((a),((b)&0x000007ff)),11))
#define MAC16_32_Q11(c,a,b) ADD32(c,ADD32(MULT16_16((a),SHR((b),11)), SHR(MULT16_16((a),((b)&0x000007ff)),11)))

#if defined(WIN32) || defined(_ARMV7_)
#define PSHR32(a,shift) (SHR32((a)+((EXTEND32(1)<<((shift))>>1)),shift))
//#define MULT16_32_P15(a,b) ADD32(MULT16_16((a),SHR((b),15)), PSHR(MULT16_16((a),((b)&0x00007fff)),15))
static inline int MULT16_32_P15(short int a, int b) 
{
    int tmp = SHR(b,15);
    int res = a * tmp;
    if(a==0 || tmp==0)
        return SHR(MULT16_16((a),((b)&0x00007fff)),15);
    else if(((a^tmp)>=0) && ((res>>31)!=0))
        res = 0x7FFFFFFF;
    else if(((a^tmp)<0) && ((res>>31)==0))
        res = 0x80000000;

    return ADD32(res,PSHR(MULT16_16((a),((b)&0x00007fff)),15));
}
#else
#define PSHR32 MIPS_PSHR32 
#endif

#ifdef _MIPS_MULT16_32_Q15_OPT_NOT_Exactly
#define MULT16_32_Q15 MULT16_32_P15
#else
//#define NTK_MULT16_32_Q15(a,b) ADD32(MULT16_16((a),SHR((b),15)), SHR(MULT16_16((a),((b)&0x00007fff)),15))
static inline int MULT16_32_Q15(short int a, int b) 
{
int tmp = SHR(b,15);
int res = a * tmp;
if(a==0 || tmp==0)
return SHR(MULT16_16((a),((b)&0x00007fff)),15);
else if(((a^tmp)>=0) && ((res>>31)!=0))
res = 0x7FFFFFFF;
else if(((a^tmp)<0) && ((res>>31)==0))
res = 0x80000000;

return ADD32(res,SHR(MULT16_16((a),((b)&0x00007fff)),15));
}
#endif


//#define NTK_MAC16_32_Q15(c,a,b) ADD32(c,ADD32(MULT16_16((a),SHR((b),15)), SHR(MULT16_16((a),((b)&0x00007fff)),15)))
static inline int MAC16_32_Q15(int c, short int a, int b) 
{
    int tmp = SHR(b,15);
    int res = a * tmp;
    if(a==0 || tmp==0)
        return ADD32(c,SHR(MULT16_16((a),((b)&0x00007fff)),15));
    else if(((a^tmp)>=0) && ((res>>31)!=0))
        res = 0x7FFFFFFF;
    else if(((a^tmp)<0) && ((res>>31)==0))
        res = 0x80000000;

    return ADD32(c, ADD32(res,SHR(MULT16_16((a),((b)&0x00007fff)),15)));
}

#define MAC16_16_Q11(c,a,b)     (ADD32((c),SHR(MULT16_16((a),(b)),11)))
#define MAC16_16_Q13(c,a,b)     (ADD32((c),SHR(MULT16_16((a),(b)),13)))
#define MAC16_16_P13(c,a,b)     (ADD32((c),SHR(ADD32(4096,MULT16_16((a),(b))),13)))

#define MULT16_16_Q11_32(a,b)	(SHR(MULT16_16((a),(b)),11))
#define MULT16_16_Q13(a,b)		(SHR(MULT16_16((a),(b)),13))
#define MULT16_16_Q14(a,b) 		(SHR(MULT16_16((a),(b)),14))

#ifdef _MIPS_MULT16_16_Q15_OPT_NOT_Exactly
#define MULT16_16_Q15(a,b)		MULT16_16_P15(a,b)
#else
#define MULT16_16_Q15(a,b) 		NTK_EXTRACT16(SHR(MULT16_16((a),(b)),15))
#endif

#define MULT16_16_P13(a,b) 		(SHR(ADD32(4096,MULT16_16((a),(b))),13))
#define MULT16_16_P14(a,b) 		(SHR(ADD32(8192,MULT16_16((a),(b))),14))
#if defined(WIN32) || defined(_ARMV7_)
#define MULT16_16_P15(a,b) NTK_EXTRACT16(SHR(ADD32(16384,MULT16_16((a),(b))),15))
#endif

#define MUL_16_32_R15(a,bh,bl) ADD32(MULT16_16((a),(bh)), SHR(MULT16_16((a),(bl)),15))

#define DIV32_16(a,b) ((spx_word16_t)(((spx_word32_t)(a))/((spx_word16_t)(b))))
#define PDIV32_16(a,b) ((spx_word16_t)(((spx_word32_t)(a)+((spx_word16_t)(b)>>1))/((spx_word16_t)(b))))
#define DIV32(a,b) (((spx_word32_t)(a))/((spx_word32_t)(b)))
#define PDIV32(a,b) (((spx_word32_t)(a)+((spx_word16_t)(b)>>1))/((spx_word32_t)(b)))

#endif
