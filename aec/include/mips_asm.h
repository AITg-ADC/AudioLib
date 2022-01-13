#ifndef __MIPS_ASM_H__
#define __MIPS_ASM_H__

#if defined(WIN32) || defined(_ARMV7_) 
#else   //WIN32

#include <mips/mips32.h>
#include <math.h>
#ifdef BIG_ENDIAN
    #define HI 0
    #define LO 1
#else
    #define HI 1
    #define LO 0
#endif

//---------------------------------------------------
// AC0
//---------------------------------------------------
static inline void SHILO_AC0(void)
{
    asm volatile ("shilo $ac0, 16" : :);
}

static inline void SHILO1_AC0(void)
{
    asm volatile ("shilo $ac0, 1" : : );
}

static inline void SHILO6_AC0(void)
{
    asm volatile ("shilo $ac0, 6" : : );
}

static inline void SHILO8_AC0(void)
{
    asm volatile ("shilo $ac0, 8" : : );
}

static inline void SHILO13_AC0(void)
{
    asm volatile ("shilo $ac0, 13" : : );
}
static inline void SHILO15_AC0(void)
{
    asm volatile ("shilo $ac0, 15" : : );
}
static inline void SHILO17_AC0(void)
{
    asm volatile ("shilo $ac0, 17" : : );
}
static inline void SHILO23_AC0(void)
{
    asm volatile ("shilo $ac0, 23" : : );
}

static inline void SHILO24_AC0(void)
{
    asm volatile ("shilo $ac0, 24" : : );
}

static inline void SHILO27_AC0(void)
{
    asm volatile ("shilo $ac0, 27" : : );
}

static inline void SHILO28_AC0(void)
{
    asm volatile ("shilo $ac0, 28" : : );
}

static inline void SHILO30_AC0(void)
{
    asm volatile ("shilo $ac0, 30" : : );
}

static inline int SHILO16_SAT_AC0(void)
{
	int res;
	asm volatile ("mflo %0, $ac0" : "=r" (res) :);
    asm volatile ("shll_s.w %0, %1, 16" : "=r" (res) : "r" (res));
    asm volatile ("sra %0, %1, 16" : "=r" (res) : "r" (res));
	return res;
}

static inline void MULT_AC0(int x, int y)
{
    asm volatile ("mult $ac0, %0, %1" : : "r" (x), "r" (y));
}

static inline void MADD_AC0(int x, int y)
{
    asm volatile ("madd $ac0, %0, %1" : : "r" (x), "r" (y));
}

static inline void MSUB_AC0(int x, int y)
{
    asm volatile ("msub $ac0, %0, %1" : : "r" (x), "r" (y));
}

static inline int MFHI_AC0(void)
{
    int res;
    asm volatile ("mfhi %0, $ac0" : "=r" (res) :);
    return res;
}

static inline int MFLO_AC0(void)
{
    int res;
    asm volatile ("mflo %0, $ac0" : "=r" (res) :);
    return res;
}

static inline void MFHILO_AC0(int *hi, int *lo)
{
    asm volatile ("mfhi %0, $ac0 ; mflo %1, $ac0" : "=r" (*hi), "=r" (*lo) :);
}

static inline void MTHI_AC0(int x)
{
    asm volatile ("mthi %0, $ac0" : : "r" (x));
}

static inline void MTLO_AC0(int x)
{
    asm volatile ("mtlo %0, $ac0" : : "r" (x));
}

static inline void MTHILO_AC0(int hi, int lo)
{
    asm volatile ("mthi %0, $ac0 ; mtlo %1, $ac0" : : "r" (hi), "r" (lo));
}

//---------------------------------------------------
// AC1
//---------------------------------------------------
static inline void SHILO_AC1(void)
{
    asm volatile ("shilo $ac1, 16" : :);
}

static inline void MULT_AC1(int x, int y)
{
    asm volatile ("mult $ac1, %0, %1" : : "r" (x), "r" (y));
}

static inline void MADD_AC1(int x, int y)
{
    asm volatile ("madd $ac1, %0, %1" : : "r" (x), "r" (y));
}

static inline void MSUB_AC1(int x, int y)
{
    asm volatile ("msub $ac1, %0, %1" : : "r" (x), "r" (y));
}

static inline int MFHI_AC1(void)
{
    int res;
    asm volatile ("mfhi %0, $ac1" : "=r" (res) :);
    return res;
}

static inline int MFLO_AC1(void)
{
    int res;
    asm volatile ("mflo %0, $ac1" : "=r" (res) :);
    return res;
}

static inline void MFHILO_AC1(int *hi, int *lo)
{
    asm volatile ("mfhi %0, $ac1 ; mflo %1, $ac1" : "=r" (*hi), "=r" (*lo) :);
}

static inline void MTHI_AC1(int x)
{
    asm volatile ("mthi %0, $ac1" : : "r" (x));
}

static inline void MTLO_AC1(int x)
{
    asm volatile ("mtlo %0, $ac1" : : "r" (x));
}

static inline void MTHILO_AC1(int hi, int lo)
{
    asm volatile ("mthi %0, $ac1 ; mtlo %1, $ac1" : : "r" (hi), "r" (lo));
}

//---------------------------------------------------
// AC2
//---------------------------------------------------
static inline void SHILO_AC2(void)
{
    asm volatile ("shilo $ac2, 16" : :);
}

static inline void SHILO8_AC2(void)
{
    asm volatile ("shilo $ac2, 8" : :);
}

static inline void SHILO24_AC2(void)
{
    asm volatile ("shilo $ac2, 24" : :);
}

static inline void SHILO27_AC2(void)
{
    asm volatile ("shilo $ac2, 27" : :);
}

static inline void SHILO28_AC2(void)
{
    asm volatile ("shilo $ac2, 28" : :);
}

static inline int SHILO16_SAT_AC2(void)
{
	int res;
	asm volatile ("mflo %0, $ac2" : "=r" (res) :);
    asm volatile ("shll_s.w %0, %1, 16" : "=r" (res) : "r" (res));
    asm volatile ("sra %0, %1, 16" : "=r" (res) : "r" (res));
	return res;
}

static inline void MULT_AC2(int x, int y)
{
    asm volatile ("mult $ac2, %0, %1" : : "r" (x), "r" (y));
}

static inline void MADD_AC2(int x, int y)
{
    asm volatile ("madd $ac2, %0, %1" : : "r" (x), "r" (y));
}

static inline void MSUB_AC2(int x, int y)
{
    asm volatile ("msub $ac2, %0, %1" : : "r" (x), "r" (y));
}

static inline int MFHI_AC2(void)
{
    int res;
    asm volatile ("mfhi %0, $ac2" : "=r" (res) :);
    return res;
}

static inline int MFLO_AC2(void)
{
    int res;
    asm volatile ("mflo %0, $ac2" : "=r" (res) :);
    return res;
}

static inline void MFHILO_AC2(int *hi, int *lo)
{
    asm volatile ("mfhi %0, $ac2 ; mflo %1, $ac2" : "=r" (*hi), "=r" (*lo) :);
}

static inline void MTHI_AC2(int x)
{
    asm volatile ("mthi %0, $ac2" : : "r" (x));
}

static inline void MTLO_AC2(int x)
{
    asm volatile ("mtlo %0, $ac2" : : "r" (x));
}

static inline void MTHILO_AC2(int hi, int lo)
{
    asm volatile ("mthi %0, $ac2 ; mtlo %1, $ac2" : : "r" (hi), "r" (lo));
}

//---------------------------------------------------
// AC3
//---------------------------------------------------
static inline void SHILO_AC3(void)
{
    asm volatile ("shilo $ac3, 16" : :);
}

static inline void MULT_AC3(int x, int y)
{
    asm volatile ("mult $ac3, %0, %1" : : "r" (x), "r" (y));
}

static inline void MADD_AC3(int x, int y)
{
    asm volatile ("madd $ac3, %0, %1" : : "r" (x), "r" (y));
}

static inline void MSUB_AC3(int x, int y)
{
    asm volatile ("msub $ac3, %0, %1" : : "r" (x), "r" (y));
}

static inline int MFHI_AC3(void)
{
    int res;
    asm volatile ("mfhi %0, $ac3" : "=r" (res) :);
    return res;
}

static inline int MFLO_AC3(void)
{
    int res;
    asm volatile ("mflo %0, $ac3" : "=r" (res) :);
    return res;
}

static inline void MFHILO_AC3(int *hi, int *lo)
{
    asm volatile ("mfhi %0, $ac3 ; mflo %1, $ac3" : "=r" (*hi), "=r" (*lo) :);
}

static inline void MTHI_AC3(int x)
{
    asm volatile ("mthi %0, $ac3" : : "r" (x));
}

static inline void MTLO_AC3(int x)
{
    asm volatile ("mtlo %0, $ac3" : : "r" (x));
}

static inline void MTHILO_AC3(int hi, int lo)
{
    asm volatile ("mthi %0, $ac3 ; mtlo %1, $ac3" : : "r" (hi), "r" (lo));
}

//---------------------------------------------------


static inline int SMUL32_1(int x, int y)
{
    int result;
    asm volatile ("mult $ac1, %1, %2; mfhi %0, $ac1" : "=r" (result): "r" (y), "r" (x));
    return result;
}


static inline int SMUL32_2(int x, int y)
{
    int result;
    asm volatile ("mult $ac2, %1, %2; mfhi %0, $ac2" : "=r" (result): "r" (y), "r" (x));
    return result;
}

static inline int SMUL32_3(int x, int y)
{
    int result;
    asm volatile ("mult $ac3, %1, %2; mfhi %0, $ac3" : "=r" (result): "r" (y), "r" (x));
    return result;
}

#endif  //WIN32
#endif  //__MIPS_ASM_H__
