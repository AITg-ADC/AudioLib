#ifndef __BASIC_OP_H
#define __BASIC_OP_H
#include "comdef_nvt.h"
//#include "kiss_fft.h"
//#include <stdio.h>

#ifdef _MIPS_
#include "mips_asm.h"

#define NMAX16	0x7FFF
#define NMIN16	0x8000

static __inline int Shift6_Round_AC0(void);
static __inline int Shift15_Round_AC0 (void);
static __inline int Shift17_Round_AC0(void);
static __inline int Shift11_Round_AC0 (void);
static __inline int Shift11_Round_AC1 (void);
static __inline s16 ABS16(s16 x);
static __inline s32 ABS32(s32 x);
static __inline s16 MULT16_16_P15 (int x, int y);	

#define C_MUL(m,a,b)	\
{  \
    register short int ar = (a).r, br = (b).r, ai=(a).i, bi=(b).i;	\
    MULT_AC0(ar, br);											\
    MSUB_AC0(ai, bi);											\
    (m).r = (s16)Shift15_Round_AC0();							\
    MULT_AC0(ar, bi);											\
    MADD_AC0(ai, br);											\
    (m).i = (s16)Shift15_Round_AC0();							\
}

#define C_MUL4(m,a,b)	\
{  \
    register short int ar = (a).r, br = (b).r, ai=(a).i, bi=(b).i;	\
    MULT_AC0(ar, br);											\
    MSUB_AC0(ai, bi);											\
    (m).r = (s16)Shift17_Round_AC0();							\
    MULT_AC0(ar, bi);											\
    MADD_AC0(ai, br);											\
    (m).i = (s16)Shift17_Round_AC0();							\
}


#define MIPS_PSHR16_2(m) \
{   \
    register int *s32Val = (int *)m;    \
    asm volatile ("SHRA_R.PH %[res], %[x], 2" : [res] "=&r" (*s32Val) : [x] "r" (*s32Val)); \
}

#define MIPS_PSHR32_15(rtn, m) \
{   \
    asm volatile ("SHRA_R.W %[res], %[x], 15" : [res] "=&r" (rtn) : [x] "r" (m)); \
}

#define MIPS_PSHR32_16(rtn, m) \
{   \
    asm volatile ("SHRA_R.W %[res], %[x], 16" : [res] "=&r" (rtn) : [x] "r" (m)); \
}

static __inline s32 MIPS_PSHR32(s32 x, s32 shift)
{
    int res;
    __asm__ __volatile__("SHRAV_R.W %[res], %[x], %[y]   \n\t"
        : [res] "=&r" (res)
        : [x] "r" (x), [y] "r" (shift)
        );
    return((s32)res);
}


#define MIPS_SHL16_PH_3(x1, x2) {\
    asm volatile("SHLL_S.PH %[res], %[x], 3" : [res] "=&r" (x1) : [x] "r" (x2)); \
}

#define MIPS_SUBQ_PH(y1, x1, x2) {\
    asm volatile("SUBQ.PH %[res], %[x], %[y]" : [res] "=&r" (y1) : [x] "r" (x1), [y] "r" (x2));     \
}

#define MIPS_ADDQ_S_PH(y1, x1, x2) {\
    asm volatile("ADDQ_S.PH %[res], %[x], %[y]" : [res] "=&r" (y1) : [x] "r" (x1), [y] "r" (x2));     \
}

static __inline s16 MULT16_16_P15(int x, int y)
{
    register int res;
    __asm__ __volatile__("MULQ_RS.PH %[res], %[x], %[y]       \n\t"
        : [res] "=&r" (res)
        : [x] "r" (x), [y] "r" (y)
        );
    return((s16)res);	  
}

static __inline int MULT16_16_P15_PH(int x, int y)
{
    register int res;
    __asm__ __volatile__("MULQ_RS.PH %[res], %[x], %[y]       \n\t"
        : [res] "=&r" (res)
        : [x] "r" (x), [y] "r" (y)
        );
    return(res);	  
}

static __inline int MULT16_32_P15(int x, int y)
{
    int ret;
    asm volatile ("mult $ac0, %0, %1" : : "r" (x), "r" (y));
    __asm__ __volatile__("EXTR_RS.W %[res], $ac0, 15 \n\t"
        : [res] "=&r" (ret)
        : 
    ); 
    return ret;
}


static __inline int Shift6_Round_AC0 (void)
{
    int x;
    __asm__ __volatile__("EXTR_RS.W %[res], $ac0, 6 \n\t"
        : [res] "=&r" (x)
        : 
    ); 
    return x;
}
static __inline int Shift15_Round_AC0 (void)
{
    int x;
    __asm__ __volatile__("EXTR_RS.W %[res], $ac0, 15 \n\t"
        : [res] "=&r" (x)
        : 
    ); 
    return x;
}
static __inline int Shift17_Round_AC0 (void)
{
    int x;
    __asm__ __volatile__("EXTR_RS.W %[res], $ac0, 17 \n\t"
        : [res] "=&r" (x)
        : 
    ); 
    return x;
}
static __inline int Shift11_Round_AC0 (void)
{
    int x;
    __asm__ __volatile__("EXTR_RS.W %[res], $ac0, 11 \n\t"
        : [res] "=&r" (x)
        : 
    ); 
    return x;
}
static __inline int Shift11_Round_AC1 (void)
{
    int x;
    __asm__ __volatile__("EXTR_RS.W %[res], $ac1, 11 \n\t"
        : [res] "=&r" (x)
        : 
    ); 
    return x;
}



#define ABS	ABS32
static __inline s16 ABS16(s16 x)
{
    int res;
    __asm__ __volatile__("ABSQ_S.PH %[res], %[x]       \n\t"
        : [res] "=&r" (res)
        : [x] "r" (x)
        );
    return((s16)res);
}

static __inline s32 ABS16_P(s32 x)
{
    int res;
    __asm__ __volatile__("ABSQ_S.PH %[res], %[x]       \n\t"
        : [res] "=&r" (res)
        : [x] "r" (x)
        );
    return(res);
}
static __inline s32 ABS32(s32 x)
{
    int res;
    __asm__ __volatile__("ABSQ_S.W %[res], %[x]       \n\t"
        : [res] "=&r" (res)
        : [x] "r" (x)
        );
    return((s32)res);
}

static __inline s32 SHLLV_PH(s32 x, s32 shift)
{
    int res;
    __asm__ __volatile__("SHLLV.PH %[res], %[x], %[y]   \n\t"
        : [res] "=&r" (res)
        : [x] "r" (x), [y] "r" (shift)
        );
    return((s32)res);
}

static __inline s32 SHRAV_R_PH(s32 x, s32 shift)
{
    int res;
    __asm__ __volatile__("SHRAV_R.PH %[res], %[x], %[y]   \n\t"
        : [res] "=&r" (res)
        : [x] "r" (x), [y] "r" (shift)
        );
    return((s32)res);
}
static __inline s32 SHLLV_S_PH(s32 x, s32 shift)
{
    int res;
    __asm__ __volatile__("SHLLV_S.PH %[res], %[x], %[y]   \n\t"
        : [res] "=&r" (res)
        : [x] "r" (x), [y] "r" (shift)
        );
    return((s32)res);
}
static __inline s32 SHLLV_S_W(s32 x, s32 shift)
{
    int res;
    __asm__ __volatile__("SHLLV_S.W %[res], %[x], %[y]   \n\t"
        : [res] "=&r" (res)
        : [x] "r" (x), [y] "r" (shift)
        );
    return((s32)res);
}

static __inline s32 EXTRV_RS_W(s32 shift)
{
    int res;
    __asm__ __volatile__("EXTRV_RS.W %[res], $ac0, %[x]   \n\t"
        : [res] "=&r" (res)
        : [x] "r" (shift)
        );
    return((s32)res);
}

static __inline void SHILOV(int x)
{
    // x>0 -> rithgt shift
    __asm__ __volatile__("SHILOV $ac0, %[x]   \n\t" : : [x] "r" (x));
}

static __inline s32 CLZ_32(s32 op)
{

    //int x = op ^ (op>>31);
    int res;

    __asm__ ("clz %[a],%[b]"
        : [a] "=r" (res) : [b] "r" (op));

    return res;
}

static __inline int DIV32_E (int var1, int var2)
{
    int quotient;
    /*if (var2 == 0)
    {
    return(0x7fffffff);
    }
    if (var1 == 0)
    {
    return(0);
    }
    else
    {
    if (var1 == var2)
    {
    return(0x7fffffff);
    }
    else*/
    {
        //printf("[V1]%d [V2]%d ",var1, var2);
        __asm__ __volatile__("DIV %[a],%[b]   \n\t"
            : "=&l" (quotient)
            : [a] "r" (var1), [b] "r" (var2));
        //printf("[Q]%d\n",quotient);
        return(quotient);
    }
    //}
}
#endif

static __inline s16 add16 (s16 x, s16 y);
static __inline int add32 (int x, int y);
static __inline s16 sub16 (s16 x, s16 y);
static __inline int sub32 (int x, int y);
//--------------------------------------------------------
static __inline s16 add16 (s16 x, s16 y)
{
#ifdef _MIPS_
    int res;
    __asm__ __volatile__("ADDQ_S.PH %[res], %[x], %[y]       \n\t"
        : [res] "=&r" (res)
        : [x] "r" (x), [y] "r" (y)
        );
    return((s16)res);
#else /*_MIPS_*/
    s16 result = x + y;
    /* saturation */
    if (((x ^ y) >= 0) && ((x ^ result) < 0)) {
        /* Same sign of both operands, but different sign of result */
        if (x < 0)
            result =  ((int)0x80000000) >> 16;
        else
            result = (0x7fffffff) >> 16;
    }
    return result;
#endif /*_MIPS_*/
}
static __inline int add32 (int x, int y)
{
#ifdef _MIPS_
    int res;
    __asm__ __volatile__("ADDQ_S.W %[res], %[x], %[y]       \n\t"
        : [res] "=&r" (res)
        : [x] "r" (x), [y] "r" (y)
        );
    return(res);
#else /*_MIPS_*/
    int result = x + y;
    /* saturation */
    if (((x ^ y) >= 0) && ((x ^ result) < 0)) {
        /* Same sign of both operands, but different sign of result */
        if (x < 0)
            result =  ((int)0x80000000);
        else
            result = (0x7fffffff);
        //printf("0x%08x 0x%08x 0x%08x\n",x,y,result);
    }
    return result;
#endif /*_MIPS_*/
}

//--------------------------------------------------------
static __inline s16 sub16 (s16 x, s16 y)
{
#ifdef _MIPS_
    int res;
    __asm__ __volatile__("SUBQ_S.PH %[res], %[x], %[y]		 \n\t"
        : [res] "=&r" (res)
        : [x] "r" (x), [y] "r" (y)
        );
    return((s16)res);
#else /*_MIPS_*/
    s16 result = x - y;
    /* saturation */
    if (((x ^ y) < 0) && ((x ^ result) < 0)) {
        /* Different sign of both operands, wrong sign of result */
        if (x < 0)
            result = (0x80000000) >> 16;
        else
            result = (0x7fffffff) >> 16;
    }
    return result;

#endif /*_MIPS_*/
}
static __inline int sub32 (int x, int y)
{
#ifdef _MIPS_
    int res;
    __asm__ __volatile__("SUBQ_S.W %[res], %[x], %[y]		 \n\t"
        : [res] "=&r" (res)
        : [x] "r" (x), [y] "r" (y)
        );
    return(res);
#else /*_MIPS_*/
    int result = x - y;
    /* saturation */
    if (((x ^ y) < 0) && ((x ^ result) < 0)) {
        /* Different sign of both operands, wrong sign of result */
        if (x < 0)
            result = (0x80000000);
        else
            result = (0x7fffffff);
    }
    return result;

#endif /*_MIPS_*/
}

#endif

