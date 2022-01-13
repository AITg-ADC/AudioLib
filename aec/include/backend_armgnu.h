#ifndef _NTK_BACKEND_ARMCC_H_
#define _NTK_BACKEND_ARMCC_H_

//#define ARM_GNU_NOT_BIT_EXACT

//Add,Subtract
static ntk_inline NTK_FRACT32 NTK_SatAdd32(NTK_FRACT32 a,NTK_FRACT32 b)
{
    NTK_FRACT32 nAcc;
    __asm__ volatile ("qadd %0,%1,%2" : "=r" (nAcc) : "r" (a), "r" (b): "cc");
    return nAcc;
}

static ntk_inline NTK_FRACT32 NTK_SatSub32(NTK_FRACT32 a,NTK_FRACT32 b)
{
    NTK_FRACT32 nAcc;
    __asm__ volatile ("qsub %0,%1,%2" : "=r" (nAcc) : "r" (a), "r" (b): "cc");
    return nAcc;
}

static ntk_inline NTK_FRACT24 NTK_SatAdd24(NTK_FRACT24 a,NTK_FRACT24 b)
{
    NTK_FRACT24 nAcc;
    __asm__ volatile (	"add %0,%2,%3\n\t"
    					"ssat %0,#24,%1" 
    					: "=r" (nAcc)
    					: "0" (nAcc), "r" (a), "r" (b)
    					: "cc");
    return nAcc;

}

static ntk_inline NTK_FRACT24 NTK_SatSub24(NTK_FRACT24 a,NTK_FRACT24 b)
{
    NTK_FRACT24 nAcc;
    __asm__ volatile (	"sub %0,%2,%3\n\t"
    					"ssat %0,#24,%1" 
    					: "=r" (nAcc)
    					: "0" (nAcc), "r" (a), "r" (b)
    					: "cc");
    return nAcc;

}

static ntk_inline NTK_FRACT16 NTK_SatAdd16(NTK_FRACT16 a,NTK_FRACT16 b)
{
	NTK_FRACT32 nAcc;
    __asm__ volatile ("qadd16 %0,%1,%2" : "=r" (nAcc) : "r" (a), "r" (b): "cc");
    return (NTK_FRACT16)nAcc;
}

static ntk_inline NTK_FRACT16 NTK_SatSub16(NTK_FRACT16 a,NTK_FRACT16 b)
{
	NTK_FRACT32 nAcc;
    __asm__ volatile ("qsub16 %0,%1,%2" : "=r" (nAcc) : "r" (a), "r" (b): "cc");
    return (NTK_FRACT16)nAcc;
}

//Abs,Negation
static ntk_inline NTK_FRACT32 NTK_SatAbs32(NTK_FRACT32 a)
{
	NTK_FRACT32 result;
    __asm__ volatile (	"eors %0,%3,%3,asr #31\n\t"
    					"mov %1,%3,asr #31\n\t"
					    "qsub %0,%2,%3" 
					    :"=r" (result), "=r" (a)
					    :"0" (result), "1" (a)
					    : "cc");
	return result;
}

static ntk_inline NTK_FRACT32 NTK_SatNeg32(NTK_FRACT32 a)
{
    NTK_FRACT32 result;
    NTK_FRACT32 z = 0;
    __asm__ volatile ("qsub %0,%1,%2" : "=r" (result) : "r" (z), "r" (a): "cc");
    return result;
}

static ntk_inline NTK_FRACT16 NTK_SatAbs16(NTK_FRACT16 a)
{
	NTK_FRACT32 result;
	NTK_SINT32 a_32 = (NTK_SINT32)a;
    __asm__ volatile (	"eors %0,%3,%3,asr #15\n\t"
    					"mov %1,%3,asr #15\n\t"
					    "qsub16 %0,%2,%3" 
					    :"=r" (result), "=r" (a_32)
					    :"0" (result), "1" (a_32)
					    : "cc");
	return (NTK_FRACT16)result;
}

static ntk_inline NTK_FRACT16 NTK_SatNeg16(NTK_FRACT16 a)
{
    NTK_FRACT16 result;
    NTK_FRACT32 z = 0;
    __asm__ volatile ("qsub16 %0,%1,%2" : "=r" (result) : "r" (z), "r" (a): "cc");
    return result;
}

static ntk_inline NTK_FRACT24 NTK_SatAbs24(NTK_FRACT24 a)
{
	NTK_FRACT32 result;
    NTK_FRACT32 a_32 = a<<8;
    __asm__ volatile (	"eors %0,%4,%3,asr #23\n\t"
    					"mov %1,%3,asr #23\n\t"
					    "qsub %0,%2,%3" 
					    :"=r" (result), "=r" (a)
					    :"0" (result), "1" (a), "r" (a_32)
					    : "cc");
	return result>>8;
}

static ntk_inline NTK_FRACT24 NTK_SatNeg24(NTK_FRACT24 a)
{
    NTK_FRACT24 result;
    __asm__ volatile (	"rsb %1,%3,#0\n\t"
    					"ssat %0,#24,%3"
    					: "=r" (result), "=r" (a)
    					: "0" (result), "1" (a) 
    					: "cc");
    return result;
}

//Saturate,Shift,Normalize
static ntk_inline NTK_SINT32  NTK_Norm32(NTK_FRACT32 a)
{
    NTK_SINT32 nShift;
    __asm__ volatile (	"eor %0,%2,%2,asr #31\n\t"
					    "clz %1,%2\n\t"
					    "sub %1,%3,#1" 
					    :"=r" (a),"=r" (nShift)
					    :"0" (a),"1" (nShift)
					    : "cc");
    return nShift;
}

static ntk_inline NTK_SINT32  NTK_Norm24(NTK_FRACT32 a)
{
    NTK_SINT32 nShift;
    __asm__ volatile (	"eor %0,%2,%2,asr #23\n\t"
					    "clz %1,%2\n\t"
					    "sub %1,%3,#9" 
					    :"=r" (a),"=r" (nShift)
					    :"0" (a),"1" (nShift)
					    : "cc");
	return nShift;
}

static ntk_inline NTK_SINT32  NTK_Norm16(NTK_FRACT16 a)
{
    NTK_SINT32 nShift;
    NTK_SINT32 a_32 = (NTK_SINT32)a;
    __asm__ volatile (	"eor %0,%2,%2,asr #15\n\t"
					    "clz %1,%2\n\t"
					    "sub %1,%3,#17" 
					    :"=r" (a_32),"=r" (nShift)
					    :"0" (a_32),"1" (nShift)
					    : "cc");
	return nShift;
}

static NTK_SINT32 RoundShiftR_Table[32] = {0,1,1<<1,1<<2,1<<3,1<<4,1<<5,1<<6,1<<7,1<<8,1<<9,1<<10,1<<11,1<<12,1<<13,1<<14,1<<15,1<<16,1<<17,1<<18,1<<19,1<<20,1<<21,1<<22,1<<23,1<<24,1<<25,1<<26,1<<27,1<<28,1<<29,1<<30};
static ntk_inline NTK_SINT32 NTK_RoundShiftR32(NTK_SINT32 a,NTK_SINT32 b)
{
    NTK_SINT32 result;
    __asm__ volatile (	"qadd %0,%1,%3\n\t"
    					"mov %0,%4,asr %2"
						: "=r" (result)
						: "r" (a), "r" (b), "r" (RoundShiftR_Table[b]), "0" (result)
						:"cc");
    return result;
}

static ntk_inline NTK_SINT16  NTK_SatRoundShiftR32to16(NTK_SINT32 a,NTK_SINT32 b)
{
    NTK_SINT32 result;
    __asm__ volatile (	"qadd %0,%1,%3\n\t"
    					"mov %0,%4,asr %2\n\t"
    					"ssat %0,#16,%4"
						: "=r" (result)
						: "r" (a), "r" (b), "r" (RoundShiftR_Table[b]), "0" (result)
						:"cc");
    return (NTK_SINT16)result;
}

static ntk_inline NTK_SINT32  NTK_SatShiftL32(NTK_SINT32 a,NTK_SINT32 b)
{
    NTK_SINT32 result;
    NTK_SINT32 m=0x7fffffff;
    __asm__ volatile (	"mov %0,%1,lsl %2\n\t"
						"teq %1,%4,asr %2\n\t"
						"eorne %0,%3,%1,asr #31"
						: "=r" (result)
						: "r" (a), "r" (b), "r" (m), "0"(result)
						:"cc");
    return result;
}

static ntk_inline NTK_SINT16  NTK_SatShiftL16(NTK_SINT16 a,NTK_SINT32 b)
{
    NTK_SINT16 result;
    NTK_SINT32 m=0x7fff0000;
    NTK_SINT32 a_32=(NTK_SINT32)a<<16;
    NTK_SINT32 r;
    __asm__ volatile (	"mov %0,%1,lsl %2\n\t"
						"teq %1,%4,asr %2\n\t"
						"eorne %0,%3,%1,asr #15"
						: "=r" (r)
						: "r" (a_32), "r" (b), "r" (m), "0"(r)
						:"cc");
    result = (NTK_SINT16)(r>>16);
    return result;
}

static ntk_inline NTK_SINT32  NTK_Sat32(NTK_SINT64 a)
{
    NTK_SINT64 out = a;
    if(a>NTK_FRACT32_MAX)
        out = NTK_FRACT32_MAX;
    else if(a<NTK_FRACT32_MIN)
        out = NTK_FRACT32_MIN;
    return (NTK_SINT32)out;
}
static ntk_inline NTK_SINT32  NTK_Sat24(NTK_SINT32 a)
{
    NTK_SINT32 out = a;
    __asm__ volatile ("ssat %0,#24,%1" : "=r" (out): "r" (out) : "cc");
    return out;
}

static ntk_inline NTK_SINT16  NTK_Sat16(NTK_SINT32 a)
{
    NTK_SINT32 out = a;
    __asm__ volatile ("ssat %0,#16,%1" : "=r" (out): "r" (out) : "cc");
    return (NTK_SINT16)out;
}

static ntk_inline NTK_SINT32  NTK_Min(NTK_SINT32 a,NTK_SINT32 b)
{
	NTK_SINT32 out = a;
	__asm__ volatile(	"cmp %1,%2\n\t"
						"movgt %0,%2"
						:"=r" (out)
						:"r" (a), "r" (b)
						:"cc");
    return out;
}

static ntk_inline NTK_SINT32  NTK_Max(NTK_SINT32 a,NTK_SINT32 b)
{
	NTK_SINT32 out = a;
	__asm__ volatile(	"cmp %1,%2\n\t"
						"movlt %0,%2"
						:"=r" (out)
						:"r" (a), "r" (b)
						:"cc");
    return out;
}

//Multiply
static ntk_inline NTK_FRACT32 NTK_Mul32X32_RShiftN(NTK_FRACT32 a,NTK_FRACT32 b,NTK_SINT32 n)
{
    NTK_FRACT32 result;
    NTK_FRACT32 r_H;
    NTK_UINT32 r_L;
    NTK_SINT64 r_64;
    __asm__ volatile ("smull %0,%1,%2,%3" : "=&r" (r_L), "=r" (r_H) : "r" (a), "1" (b) : "cc");
    r_64 = (NTK_SINT64)r_H <<32;
	r_64 = r_64|r_L;
	result = (NTK_FRACT32)NTK_Sat32(r_64>>n);
    return result;
}

static ntk_inline NTK_FRACT32 NTK_Mul32X32_RoundShiftN(NTK_FRACT32 a,NTK_FRACT32 b,NTK_SINT32 n)
{
    NTK_FRACT32 result;
    NTK_FRACT32 r_H;
    NTK_UINT32 r_L;
    NTK_SINT64 r_64;
    __asm__ volatile ("smull %0,%1,%2,%3" : "=&r" (r_L), "=r" (r_H) : "r" (a), "1" (b) : "cc");
    r_64 = (NTK_SINT64)r_H <<32;
	r_64 = r_64|r_L;
	r_64 = r_64 + (1<<(n-1));
	result = (NTK_FRACT32)NTK_Sat32(r_64>>n);
    return result;
}

#ifdef ARM_GNU_NOT_BIT_EXACT
static ntk_inline NTK_FRACT32 NTK_FpMul32X32(NTK_FRACT32 a,NTK_FRACT32 b)
{
    NTK_FRACT32 result;
    __asm__ volatile (	"smmul %0,%2,%3\n\t"
	    				"qadd %0,%1,%1"
	    				: "=r" (result) 
	    				: "0" (result), "r" (a), "r" (b) 
	    				: "cc");
    return result;
}
#else
static ntk_inline NTK_FRACT32 NTK_FpMul32X32(NTK_FRACT32 a,NTK_FRACT32 b)
{
    NTK_FRACT32 result,r_H;
    NTK_UINT32 r_L;
    NTK_SINT32 n = _NTK_LFRACT_MANTBITS;
    NTK_SINT32 m = 32-_NTK_LFRACT_MANTBITS;
    __asm__ volatile (  "smull %0,%1,%3,%4\n\t"
			    		"mov %0,%7,lsr %5\n\t"
			    		"orr %2,%7,%8,lsl %6"
    					: "=&r" (r_L), "=r" (r_H), "=r" (result)
    					: "r" (a), "r" (b), "r" (n), "r" (m), "0" (r_L), "1" (r_H)
    					: "cc");
    return result;
}
#endif

#ifdef ARM_GNU_NOT_BIT_EXACT
static ntk_inline NTK_ACCU32 NTK_FpMac32X32(NTK_ACCU32 accu,NTK_FRACT32 b,NTK_FRACT32 c)
{
    NTK_ACCU32 result;
    __asm__ volatile (	"smmul %0,%3,%4\n\t"
	    				"qdadd %0,%2,%1"
	    				: "=r" (result) 
	    				: "0" (result), "r" (accu), "r" (b), "r" (c) 
	    				: "cc");
    return result;
}
#else
static ntk_inline NTK_ACCU32 NTK_FpMac32X32(NTK_ACCU32 accu,NTK_FRACT32 b,NTK_FRACT32 c)
{
    NTK_ACCU32 result;
    NTK_FRACT32 r_H;
    NTK_UINT32 r_L;
    NTK_SINT32 n = _NTK_LFRACT_MANTBITS;
    NTK_SINT32 m = 32-_NTK_LFRACT_MANTBITS;
    __asm__ volatile (  "smull %0,%1,%3,%4\n\t"
			    		"mov %0,%7,lsr %5\n\t"
			    		"orr %1,%7,%8,lsl %6\n\t"
    					"qadd %2,%9,%8"
    					: "=&r" (r_L), "=r" (r_H), "=r" (result)
    					: "r" (b), "r" (c), "r" (n), "r" (m), "0" (r_L), "1" (r_H), "r" (accu)
    					: "cc");
    return result;
}
#endif

#ifdef ARM_GNU_NOT_BIT_EXACT
static ntk_inline NTK_ACCU32 NTK_FpMas32X32(NTK_ACCU32 accu,NTK_FRACT32 b,NTK_FRACT32 c)
{
    NTK_ACCU32 result;
    __asm__ volatile (	"smmul %0,%3,%4\n\t"
	    				"qdsub %0,%2,%1"
	    				: "=r" (result) 
	    				: "0" (result), "r" (accu), "r" (b), "r" (c) 
	    				: "cc");
    return result;
}
#else
static ntk_inline NTK_ACCU32 NTK_FpMas32X32(NTK_ACCU32 accu,NTK_FRACT32 b,NTK_FRACT32 c)
{
    NTK_ACCU32 result;
    NTK_FRACT32 r_H;
    NTK_UINT32 r_L;
    NTK_SINT32 n = _NTK_LFRACT_MANTBITS;
    NTK_SINT32 m = 32-_NTK_LFRACT_MANTBITS;
    __asm__ volatile (  "smull %0,%1,%3,%4\n\t"
			    		"mov %0,%7,lsr %5\n\t"
			    		"orr %1,%7,%8,lsl %6\n\t"
    					"qsub %2,%9,%8"
    					: "=&r" (r_L), "=r" (r_H), "=r" (result)
    					: "r" (b), "r" (c), "r" (n), "r" (m), "0" (r_L), "1" (r_H), "r" (accu)
    					: "cc");
    return result;
}
#endif

static ntk_inline NTK_FRACT32 NTK_Mul32X32_RShift32(NTK_FRACT32 a,NTK_FRACT32 b)
{
    NTK_FRACT32 result;
    __asm__ volatile ("smull %0,%1,%2,%3" : "=&r" (b), "=r" (result) : "r" (a), "1" (b) : "cc");
    return result;
}

#ifdef ARM_GNU_NOT_BIT_EXACT
static ntk_inline NTK_FRACT32 NTK_FpMul32X16(NTK_FRACT32 a,NTK_FRACT16 b)
{
    NTK_FRACT32 result;
    NTK_SINT32 b_32 = (NTK_FRACT32)b;
    __asm__ volatile (	"smulwb %0,%2,%3\n\t"
	    				"qadd %0,%1,%1"
	    				: "=r" (result) 
	    				: "0" (result), "r" (a), "r" (b_32) 
	    				: "cc");
    return result;
}
#else
static ntk_inline NTK_FRACT32 NTK_FpMul32X16(NTK_FRACT32 a,NTK_FRACT16 b)
{
    NTK_FRACT32 result;
    NTK_FRACT32 r_H;
    NTK_UINT32 r_L;
    NTK_SINT32 n = _NTK_SFRACT_MANTBITS;
    NTK_SINT32 m = 32-_NTK_SFRACT_MANTBITS;
    NTK_SINT32 b_32 = (NTK_FRACT32)b;
    __asm__ volatile (  "smull %0,%1,%3,%4\n\t"
			    		"mov %0,%7,lsr %5\n\t"
			    		"orr %2,%7,%8,lsl %6"
    					: "=&r" (r_L), "=r" (r_H), "=r" (result)
    					: "r" (a), "r" (b_32), "r" (n), "r" (m), "0" (r_L), "1" (r_H)
    					: "cc");
    return result;
}
#endif

static ntk_inline NTK_FRACT32 NTK_FpMul32X16RoundShift16(NTK_FRACT32 a, NTK_FRACT16 b)
{
    NTK_FRACT32 result,r_H;
    NTK_UINT32 r_L, max_low = 0xFFFF8000;
    NTK_SINT32 n = _NTK_SFRACT_MANTBITS+1;
    NTK_SINT32 m = 32-(_NTK_SFRACT_MANTBITS+1);
    NTK_SINT32 b_32 = (NTK_FRACT32)b;
    __asm__ volatile (  "smull %0,%1,%3,%4\n\t"
    					"cmp %7,%9\n\t"
    					"add %0,%7,#0x8000\n\t"
    					"addcs %1,%8,#1\n\t"
			    		"mov %0,%7,lsr %5\n\t"
			    		"orr %2,%7,%8,lsl %6"
    					: "=&r" (r_L), "=r" (r_H), "=r" (result)
    					: "r" (a), "r" (b_32), "r" (n), "r" (m), "0" (r_L), "1" (r_H), "r" (max_low)
    					: "cc");
    return result;
}

static ntk_inline NTK_FRACT32 NTK_FpMul32X16RoundShift15(NTK_FRACT32 a, NTK_FRACT16 b)
{
    NTK_FRACT32 result,r_H;
    NTK_UINT32 r_L, max_low = 0xFFFFC000;
    NTK_SINT32 n = _NTK_SFRACT_MANTBITS;
    NTK_SINT32 m = 32-_NTK_SFRACT_MANTBITS;
    NTK_SINT32 b_32 = (NTK_FRACT32)b;
    __asm__ volatile (  "smull %0,%1,%3,%4\n\t"
    					"cmp %7,%9\n\t"
    					"add %0,%7,#0x4000\n\t"
    					"addcs %1,%8,#1\n\t"
			    		"mov %0,%7,lsr %5\n\t"
			    		"orr %2,%7,%8,lsl %6"
    					: "=&r" (r_L), "=r" (r_H), "=r" (result)
    					: "r" (a), "r" (b_32), "r" (n), "r" (m), "0" (r_L), "1" (r_H), "r" (max_low)
    					: "cc");
    return result;
}

#ifdef ARM_GNU_NOT_BIT_EXACT
static ntk_inline NTK_ACCU32 NTK_FpMac32X16(NTK_ACCU32 accu,NTK_FRACT32 b,NTK_FRACT16 c)
{
    NTK_ACCU32 result;
    NTK_SINT32 c_32 = (NTK_FRACT32)c;
    __asm__ volatile (	"smulwb %0,%3,%4\n\t"
	    				"qdadd %0,%2,%1"
	    				: "=r" (result) 
	    				: "0" (result), "r" (accu), "r" (b), "r" (c_32) 
	    				: "cc");
    return result;
}
#else
static ntk_inline NTK_ACCU32 NTK_FpMac32X16(NTK_ACCU32 accu,NTK_FRACT32 b,NTK_FRACT16 c)
{
    NTK_ACCU32 result;
    NTK_FRACT32 r_H;
    NTK_UINT32 r_L;
    NTK_SINT32 n = _NTK_SFRACT_MANTBITS;
    NTK_SINT32 m = 32-_NTK_SFRACT_MANTBITS;
    NTK_SINT32 c_32 = (NTK_FRACT32)c;
    __asm__ volatile (  "smull %0,%1,%3,%4\n\t"
			    		"mov %0,%7,lsr %5\n\t"
			    		"orr %1,%7,%8,lsl %6\n\t"
    					"qadd %2,%9,%8"
    					: "=&r" (r_L), "=r" (r_H), "=r" (result)
    					: "r" (b), "r" (c_32), "r" (n), "r" (m), "0" (r_L), "1" (r_H), "r" (accu)
    					: "cc");
    return result;
}
#endif

#ifdef ARM_GNU_NOT_BIT_EXACT
static ntk_inline NTK_ACCU32 NTK_FpMas32X16(NTK_ACCU32 accu,NTK_FRACT32 b,NTK_FRACT16 c)
{
    NTK_ACCU32 result;
    NTK_SINT32 c_32 = (NTK_FRACT32)c;
    __asm__ volatile (	"smulwb %0,%3,%4\n\t"
	    				"qdsub %0,%2,%1"
	    				: "=r" (result) 
	    				: "0" (result), "r" (accu), "r" (b), "r" (c_32) 
	    				: "cc");
    return result;
}
#else
static ntk_inline NTK_ACCU32 NTK_FpMas32X16(NTK_ACCU32 accu,NTK_FRACT32 b,NTK_FRACT16 c)
{
    NTK_ACCU32 result;
    NTK_FRACT32 r_H;
    NTK_UINT32 r_L;
    NTK_SINT32 n = _NTK_SFRACT_MANTBITS;
    NTK_SINT32 m = 32-_NTK_SFRACT_MANTBITS;
    NTK_SINT32 c_32 = (NTK_FRACT32)c;
    __asm__ volatile (  "smull %0,%1,%3,%4\n\t"
			    		"mov %0,%7,lsr %5\n\t"
			    		"orr %1,%7,%8,lsl %6\n\t"
    					"qsub %2,%9,%8"
    					: "=&r" (r_L), "=r" (r_H), "=r" (result)
    					: "r" (b), "r" (c_32), "r" (n), "r" (m), "0" (r_L), "1" (r_H), "r" (accu)
    					: "cc");
    return result;
}
#endif

static ntk_inline NTK_FRACT32 NTK_Mul32X16_RShift16(NTK_FRACT32 a,NTK_FRACT16 b)
{
    NTK_FRACT32 result;
    __asm__ volatile ("smulwb %0,%1,%2" : "=r" (result) : "r" (a), "r" (b) : "cc");
    return result;
}

static ntk_inline NTK_FRACT24 NTK_FpMul24X24(NTK_FRACT24 a,NTK_FRACT24 b)
{
    NTK_FRACT24 result;
    NTK_FRACT32 r_H;
    NTK_UINT32 r_L;
    NTK_SINT32 n = _NTK_DSP24_MANTBITS;
    NTK_SINT32 m = 32-_NTK_DSP24_MANTBITS;
    __asm__ volatile (  "smull %0,%1,%3,%4\n\t"
			    		"mov %0,%7,lsr %5\n\t"
			    		"orr %2,%7,%8,lsl %6"
    					: "=&r" (r_L), "=r" (r_H), "=r" (result)
    					: "r" (a), "r" (b), "r" (n), "r" (m), "0" (r_L), "1" (r_H)
    					: "cc");
    return result;
}

static ntk_inline NTK_ACCU32 NTK_FpMac24X24(NTK_ACCU32 accu,NTK_FRACT24 b,NTK_FRACT24 c)
{
    NTK_ACCU32 result;
    NTK_FRACT32 r_H;
    NTK_UINT32 r_L;
    NTK_SINT32 n = _NTK_DSP24_MANTBITS;
    NTK_SINT32 m = 32-_NTK_DSP24_MANTBITS;
    __asm__ volatile (  "smull %0,%1,%3,%4\n\t"
			    		"mov %0,%7,lsr %5\n\t"
			    		"orr %1,%7,%8,lsl %6\n\t"
    					"qadd %2,%9,%8"
    					: "=&r" (r_L), "=r" (r_H), "=r" (result)
    					: "r" (b), "r" (c), "r" (n), "r" (m), "0" (r_L), "1" (r_H), "r" (accu)
    					: "cc");
    return result;
}

static ntk_inline NTK_ACCU32 NTK_FpMas24X24(NTK_ACCU32 accu,NTK_FRACT24 b,NTK_FRACT24 c)
{
    NTK_ACCU32 result;
    NTK_FRACT32 r_H;
    NTK_UINT32 r_L;
    NTK_SINT32 n = _NTK_DSP24_MANTBITS;
    NTK_SINT32 m = 32-_NTK_DSP24_MANTBITS;
    __asm__ volatile (  "smull %0,%1,%3,%4\n\t"
			    		"mov %0,%7,lsr %5\n\t"
			    		"orr %1,%7,%8,lsl %6\n\t"
    					"qsub %2,%9,%8"
    					: "=&r" (r_L), "=r" (r_H), "=r" (result)
    					: "r" (b), "r" (c), "r" (n), "r" (m), "0" (r_L), "1" (r_H), "r" (accu)
    					: "cc");
    return result;
}

static ntk_inline NTK_FRACT24 NTK_SatFpMul24X24(NTK_FRACT24 a,NTK_FRACT24 b)
{
    NTK_FRACT24 result;
    NTK_FRACT32 r_H;
    NTK_UINT32 r_L;
    NTK_SINT32 n = _NTK_DSP24_MANTBITS;
    NTK_SINT32 m = 32-_NTK_DSP24_MANTBITS;
    __asm__ volatile (  "smull %0,%1,%3,%4\n\t"
			    		"mov %0,%7,lsr %5\n\t"
			    		"orr %2,%7,%8,lsl %6\n\t"
    					"ssat %2,#24,%9"
    					: "=&r" (r_L), "=r" (r_H), "=r" (result)
    					: "r" (a), "r" (b), "r" (n), "r" (m), "0" (r_L), "1" (r_H), "2" (result)
    					: "cc");
    return result;
}

static ntk_inline NTK_ACCU32 NTK_SatFpMac24X24(NTK_ACCU32 accu,NTK_FRACT24 b,NTK_FRACT24 c)
{
    NTK_ACCU32 result;
    NTK_FRACT32 r_H;
    NTK_UINT32 r_L;
    NTK_SINT32 n = _NTK_DSP24_MANTBITS;
    NTK_SINT32 m = 32-_NTK_DSP24_MANTBITS;
    __asm__ volatile (  "smull %0,%1,%3,%4\n\t"
			    		"mov %0,%7,lsr %5\n\t"
			    		"orr %1,%7,%8,lsl %6\n\t"
    					"add %1,%9,%8\n\t"
    					"ssat %2,#24,%1"
    					: "=&r" (r_L), "=r" (r_H), "=r" (result)
    					: "r" (b), "r" (c), "r" (n), "r" (m), "0" (r_L), "1" (r_H), "r" (accu)
    					: "cc");
    return result;
}

static ntk_inline NTK_ACCU32 NTK_SatFpMas24X24(NTK_ACCU32 accu,NTK_FRACT24 b,NTK_FRACT24 c)
{
    NTK_ACCU32 result;
    NTK_FRACT32 r_H;
    NTK_UINT32 r_L;
    NTK_SINT32 n = _NTK_DSP24_MANTBITS;
    NTK_SINT32 m = 32-_NTK_DSP24_MANTBITS;
    __asm__ volatile (  "smull %0,%1,%3,%4\n\t"
			    		"mov %0,%7,lsr %5\n\t"
			    		"orr %1,%7,%8,lsl %6\n\t"
    					"sub %1,%9,%8\n\t"
    					"ssat %2,#24,%1"
    					: "=&r" (r_L), "=r" (r_H), "=r" (result)
    					: "r" (b), "r" (c), "r" (n), "r" (m), "0" (r_L), "1" (r_H), "r" (accu)
    					: "cc");
    return result;
}

static ntk_inline NTK_FRACT16 NTK_FpMul16X16(NTK_FRACT16 a,NTK_FRACT16 b)
{
	NTK_SINT32 result;
    NTK_SINT32 a_32 = (NTK_SINT32)a;
    NTK_SINT32 b_32 = (NTK_SINT32)b;
    __asm__ volatile (	"smulbb %0,%1,%2\n\t"
    					"mov %0,%3,lsr #15"
    					: "=r" (result) 
    					: "r" (a_32), "r" (b_32), "0" (result) 
    					: "cc");
    return (NTK_FRACT16)result;
}

static ntk_inline NTK_ACCU32 NTK_FpMac16X16(NTK_ACCU32 accu,NTK_FRACT16 b,NTK_FRACT16 c)
{
    NTK_ACCU32 result;
    NTK_SINT32 b_32 = (NTK_SINT32)b;
    NTK_SINT32 c_32 = (NTK_SINT32)c;
    __asm__ volatile (	"smulbb %0,%1,%2\n\t" 
    				  	"add %0,%3,%4,asr #15"
    					: "=r" (result)
    					: "r" (b_32), "r" (c_32), "r" (accu), "0" (result)
    					: "cc");
    return result;
}

static ntk_inline NTK_ACCU32 NTK_FpMas16X16(NTK_ACCU32 accu,NTK_FRACT16 b,NTK_FRACT16 c)
{
    NTK_ACCU32 result;
    NTK_SINT32 b_32 = (NTK_SINT32)b;
    NTK_SINT32 c_32 = (NTK_SINT32)c;
    __asm__ volatile (	"smulbb %0,%1,%2\n\t" 
    				  	"sub %0,%3,%4,asr #15"
    					: "=r" (result)
    					: "r" (b_32), "r" (c_32), "r" (accu), "0" (result)
    					: "cc");
    return result;
}

static ntk_inline NTK_FRACT32 NTK_Mul16X16_D(NTK_FRACT16 a,NTK_FRACT16 b) 
{
	NTK_ACCU32 result;
    NTK_SINT32 a_32 = (NTK_SINT32)a;
    NTK_SINT32 b_32 = (NTK_SINT32)b;
    __asm__ volatile (	"smulbb %0,%2,%3\n\t"
					    "qadd %0,%1,%1" 
					    :"=r" (result)
					    :"0" (result),"r" (a_32),"r" (b_32)
					    : "cc");
	return result;
}

static ntk_inline NTK_ACCU32 NTK_Mac16X16_D(NTK_ACCU32 accu,NTK_FRACT16 b,NTK_FRACT16 c)
{
	NTK_ACCU32 result;
    NTK_SINT32 b_32 = (NTK_SINT32)b;
    NTK_SINT32 c_32 = (NTK_SINT32)c;
    __asm__ volatile (	"smulbb %0,%2,%3\n\t"
					    "qdadd %0,%4,%1" 
					    :"=r" (result)
					    :"0" (result),"r" (b_32),"r" (c_32),"r"(accu)
					    : "cc");
	return result;
}

static ntk_inline NTK_ACCU32 NTK_Mas16X16_D(NTK_ACCU32 accu,NTK_FRACT16 b,NTK_FRACT16 c)
{
	NTK_ACCU32 result;
    NTK_SINT32 b_32 = (NTK_SINT32)b;
    NTK_SINT32 c_32 = (NTK_SINT32)c;
    __asm__ volatile (	"smulbb %0,%2,%3\n\t"
					    "qdsub %0,%4,%1" 
					    :"=r" (result)
					    :"0" (result),"r" (b_32),"r" (c_32),"r"(accu)
					    : "cc");
	return result;
}
#endif
