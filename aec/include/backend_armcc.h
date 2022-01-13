#ifndef _NTK_BACKEND_ARMCC_H_
#define _NTK_BACKEND_ARMCC_H_

//#define ARM_CC_NOT_BIT_EXACT

//Add,Subtract
static ntk_inline NTK_FRACT32 NTK_SatAdd32(NTK_FRACT32 a,NTK_FRACT32 b)
{
    NTK_FRACT32 nAcc;
    __asm{
    	qadd nAcc,a,b
    }
    return nAcc;
}

static ntk_inline NTK_FRACT32 NTK_SatSub32(NTK_FRACT32 a,NTK_FRACT32 b)
{
    NTK_FRACT32 nAcc;
    __asm{
    	qsub nAcc,a,b
    }
    return nAcc;
}

static ntk_inline NTK_FRACT24 NTK_SatAdd24(NTK_FRACT24 a,NTK_FRACT24 b)
{
    NTK_FRACT24 nAcc;
    __asm{
    	add nAcc,a,b
    	ssat nAcc,#24,nAcc
    }
    return nAcc;

}

static ntk_inline NTK_FRACT24 NTK_SatSub24(NTK_FRACT24 a,NTK_FRACT24 b)
{
    NTK_FRACT24 nAcc;
    __asm{
    	sub nAcc,a,b
     	ssat nAcc,#24,nAcc
    }
    return nAcc;

}

static ntk_inline NTK_FRACT16 NTK_SatAdd16(NTK_FRACT16 a,NTK_FRACT16 b)
{
	NTK_FRACT32 nAcc;
    __asm{
     	qadd16 nAcc,a,b
    }
    return (NTK_FRACT16)nAcc;
}

static ntk_inline NTK_FRACT16 NTK_SatSub16(NTK_FRACT16 a,NTK_FRACT16 b)
{
	NTK_FRACT32 nAcc;
    __asm{
     	qsub16 nAcc,a,b
    }
    return (NTK_FRACT16)nAcc;
}

//Abs,Negation
static ntk_inline NTK_FRACT32 NTK_SatAbs32(NTK_FRACT32 a)
{  
	NTK_FRACT32 result;
	__asm { 	
	   eors result,a,a,asr #31
	   mov a,a,asr #31
	   qsub result,result,a
	}
	return result;
}

static ntk_inline NTK_FRACT32 NTK_SatNeg32(NTK_FRACT32 a)
{
    NTK_FRACT32 result;
    NTK_FRACT32 z = 0;
    __asm{
    	qsub result,z,a
    }
    return result;
}

static ntk_inline NTK_FRACT16 NTK_SatAbs16(NTK_FRACT16 a)
{
	NTK_FRACT32 result;
	__asm {
	   eors result,a,a,asr #15
	   mov a,a,asr #15
	   qsub16 result,result,a
	}
	return (NTK_FRACT16)result;
}

static ntk_inline NTK_FRACT16 NTK_SatNeg16(NTK_FRACT16 a)
{
	NTK_FRACT32 result;
    NTK_FRACT32 z = 0;
    __asm{
    	qsub16 result,z,a
    }
    return (NTK_FRACT16)result;
}

static ntk_inline NTK_FRACT24 NTK_SatAbs24(NTK_FRACT24 a)
{
	NTK_FRACT32 result;
    NTK_FRACT32 a_32 = a<<8;
	__asm { 	
	   eors result,a_32,a,asr #23
	   mov a,a,asr #23
	   qsub result,result,a
	}
	return result>>8;
}

static ntk_inline NTK_FRACT24 NTK_SatNeg24(NTK_FRACT24 a)
{
    NTK_FRACT24 result;
    __asm{
    	rsb a,a,#0
     	ssat result,#24,a
    }
    return result;
}

//Saturate,Shift,Normalize
static ntk_inline NTK_SINT32  NTK_Norm32(NTK_FRACT32 a)
{
    NTK_SINT32 nShift;
    __asm{
    	eor a,a,a,asr #31
    	clz nShift,a	
    	sub nShift,nShift,#1
    }
	return nShift;
}

static ntk_inline NTK_SINT32  NTK_Norm24(NTK_FRACT32 a)
{
    NTK_SINT32 nShift;
    __asm{
    	eor a,a,a,asr #23
    	clz nShift,a	
    	sub nShift,nShift,#9
    }
	return nShift;
}

static ntk_inline NTK_SINT32  NTK_Norm16(NTK_FRACT16 a)
{
    NTK_SINT32 nShift;
    __asm{
    	eor a,a,a,asr #15
    	clz nShift,a	
    	sub nShift,nShift,#17
    }
	return nShift;
}

static NTK_SINT32 RoundShiftR_Table[32] = {0,1,1<<1,1<<2,1<<3,1<<4,1<<5,1<<6,1<<7,1<<8,1<<9,1<<10,1<<11,1<<12,1<<13,1<<14,1<<15,1<<16,1<<17,1<<18,1<<19,1<<20,1<<21,1<<22,1<<23,1<<24,1<<25,1<<26,1<<27,1<<28,1<<29,1<<30};
static ntk_inline NTK_SINT32 NTK_RoundShiftR32(NTK_SINT32 a,NTK_SINT32 b)
{
    NTK_SINT32 result;
    __asm{
    	qadd result,a,RoundShiftR_Table[b]
    	mov result,result,asr b
    }
    return result;
}

static ntk_inline NTK_SINT16  NTK_SatRoundShiftR32to16(NTK_SINT32 a,NTK_SINT32 b)
{
    NTK_SINT32 result;
    __asm{
    	qadd result,a,RoundShiftR_Table[b]
    	mov result,result,asr b
     	ssat result,#16,result
    }
    return (NTK_SINT16)result;
}

static ntk_inline NTK_SINT32  NTK_SatShiftL32(NTK_SINT32 a,NTK_SINT32 b)
{
    NTK_SINT32 result;
    NTK_SINT32 m=0x7fffffff;
    __asm{
    	mov result,a,lsl b
    	teq a,result,asr b
    	eorne result,m,a,asr #31
    }
    return result;
}

static ntk_inline NTK_SINT16  NTK_SatShiftL16(NTK_SINT16 a,NTK_SINT32 b)
{
    NTK_SINT16 result;
    NTK_SINT32 m=0x7fff0000;
    NTK_SINT32 a_32=(NTK_SINT32)a<<16;
    NTK_SINT32 r;
    __asm{
    	mov r,a_32,lsl b
    	teq a_32,r,asr b
    	eorne r,m,a_32,asr #15
    }
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
    __asm{
     	ssat out,#24,out
    }
    return out;
}

static ntk_inline NTK_SINT16  NTK_Sat16(NTK_SINT32 a)
{
    NTK_SINT32 out = a;
    __asm{
     	ssat out,#16,out
    }
    return (NTK_SINT16)out;
}

static ntk_inline NTK_SINT32  NTK_Min(NTK_SINT32 a,NTK_SINT32 b)
{
	NTK_SINT32 out = a;
	__asm{
		cmp a,b
		movgt out,b
	}
    return out;
}

static ntk_inline NTK_SINT32  NTK_Max(NTK_SINT32 a,NTK_SINT32 b)
{
	NTK_SINT32 out = a;
	__asm{
		cmp a,b
		movlt out,b
	}
    return out;
}

//Multiply
static ntk_inline NTK_FRACT32 NTK_Mul32X32_RShiftN(NTK_FRACT32 a,NTK_FRACT32 b,NTK_SINT32 n)
{
    NTK_FRACT32 result;
    NTK_FRACT32 r_H;
    NTK_UINT32 r_L;
    NTK_SINT64 r_64;
    __asm{
    	smull r_L,r_H,a,b
    }
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
    __asm{
    	smull r_L,r_H,a,b
    }
    r_64 = (NTK_SINT64)r_H <<32;
	r_64 = r_64|r_L;
	r_64 = r_64 + (1<<(n-1));
	result = (NTK_FRACT32)NTK_Sat32(r_64>>n);
    return result;
}

#ifdef ARM_CC_NOT_BIT_EXACT
static ntk_inline NTK_FRACT32 NTK_FpMul32X32(NTK_FRACT32 a,NTK_FRACT32 b)
{
    NTK_FRACT32 result;
    __asm{
    	smmul result,a,b
    	qadd result,result,result
    }
    return result;
}
#else
static ntk_inline NTK_FRACT32 NTK_FpMul32X32(NTK_FRACT32 a,NTK_FRACT32 b)
{
    NTK_FRACT32 result,r_H;
    NTK_UINT32 r_L;
    NTK_SINT32 n = _NTK_LFRACT_MANTBITS;
    NTK_SINT32 m = 32-_NTK_LFRACT_MANTBITS;
    __asm{
    	smull r_L,r_H,a,b
    	mov r_L,r_L,lsr n
    	orr result,r_L,r_H,lsl m
    }
    return result;
}
#endif

#ifdef ARM_CC_NOT_BIT_EXACT
static ntk_inline NTK_ACCU32 NTK_FpMac32X32(NTK_ACCU32 accu,NTK_FRACT32 b,NTK_FRACT32 c)
{
    NTK_ACCU32 result;
    __asm{
    	smmul result,b,c
    	qdadd result,accu,result
    }
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
    __asm{
    	smull r_L,r_H,b,c
    	mov r_L,r_L,lsr n
    	orr result,r_L,r_H,lsl m
    	qadd result,accu,result
    }
    return result;
}
#endif

#ifdef ARM_CC_NOT_BIT_EXACT
static ntk_inline NTK_ACCU32 NTK_FpMas32X32(NTK_ACCU32 accu,NTK_FRACT32 b,NTK_FRACT32 c)
{
    NTK_ACCU32 result;
    __asm{
    	smmul result,b,c
    	qdsub result,accu,result
    }
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
    __asm{
    	smull r_L,r_H,b,c
    	mov r_L,r_L,lsr n
    	orr result,r_L,r_H,lsl m
    	qsub result,accu,result
    }
    return result;
}
#endif

static ntk_inline NTK_FRACT32 NTK_Mul32X32_RShift32(NTK_FRACT32 a,NTK_FRACT32 b)
{
    NTK_FRACT32 result;
    __asm{
    	smull b,result,a,b
    }
    return result;
}

#ifdef ARM_CC_NOT_BIT_EXACT
static ntk_inline NTK_FRACT32 NTK_FpMul32X16(NTK_FRACT32 a,NTK_FRACT16 b)
{
    NTK_FRACT32 result;
    __asm{
    	smulwb result,a,b
    	qadd result,result,result
    }
    return result;
}
#else
static ntk_inline NTK_FRACT32 NTK_FpMul32X16(NTK_FRACT32 a,NTK_FRACT16 b)
{
    NTK_FRACT32 result,r_H;
    NTK_UINT32 r_L;
    NTK_SINT32 n = _NTK_SFRACT_MANTBITS;
    NTK_SINT32 m = 32-_NTK_SFRACT_MANTBITS;
    __asm{
    	smull r_L,r_H,a,b
    	mov r_L,r_L,lsr n
    	orr result,r_L,r_H,lsl m
    }
    return result;
}
#endif

static ntk_inline NTK_FRACT32 NTK_FpMul32X16RoundShift16(NTK_FRACT32 a, NTK_FRACT16 b)
{
    NTK_FRACT32 result,r_H;
    NTK_UINT32 r_L, max_low = 0xFFFF8000;
    NTK_SINT32 n = _NTK_SFRACT_MANTBITS+1;
    NTK_SINT32 m = 32-(_NTK_SFRACT_MANTBITS+1);
    __asm{
    	smull r_L,r_H,a,b
    	cmp r_L,max_low
    	add r_L,r_L,RoundShiftR_Table[16]
    	addcs r_H,r_H,#1                              
    	mov r_L,r_L,lsr n
    	orr result,r_L,r_H,lsl m
    }
    return result;
}

static ntk_inline NTK_FRACT32 NTK_FpMul32X16RoundShift15(NTK_FRACT32 a, NTK_FRACT16 b)
{
    NTK_FRACT32 result,r_H;
    NTK_UINT32 r_L, max_low = 0xFFFFC000;
    NTK_SINT32 n = _NTK_SFRACT_MANTBITS;
    NTK_SINT32 m = 32-_NTK_SFRACT_MANTBITS;
    __asm{
    	smull r_L,r_H,a,b
    	cmp r_L,max_low
    	add r_L,r_L,RoundShiftR_Table[15]
    	addcs r_H,r_H,#1                              
    	mov r_L,r_L,lsr n
    	orr result,r_L,r_H,lsl m
    }
    return result;
}

#ifdef ARM_CC_NOT_BIT_EXACT
static ntk_inline NTK_ACCU32 NTK_FpMac32X16(NTK_ACCU32 accu,NTK_FRACT32 b,NTK_FRACT16 c)
{
    NTK_ACCU32 result;
    __asm{
    	smulwb result,b,c
    	qdadd result,accu,result
    }
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
    __asm{
    	smull r_L,r_H,b,c
    	mov r_L,r_L,lsr n
    	orr result,r_L,r_H,lsl m
    	qadd result,accu,result
    }
    return result;
}
#endif

#ifdef ARM_CC_NOT_BIT_EXACT
static ntk_inline NTK_ACCU32 NTK_FpMas32X16(NTK_ACCU32 accu,NTK_FRACT32 b,NTK_FRACT16 c)
{
    NTK_ACCU32 result;
    __asm{
    	smulwb result,b,c
    	qdsub result,accu,result
    }
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
    __asm{
    	smull r_L,r_H,b,c
    	mov r_L,r_L,lsr n
    	orr result,r_L,r_H,lsl m
    	qsub result,accu,result
    }
    return result;
}
#endif

static ntk_inline NTK_FRACT32 NTK_Mul32X16_RShift16(NTK_FRACT32 a,NTK_FRACT16 b)
{
    NTK_FRACT32 result;
    __asm{
    	smulwb result,a,b
    }
    return result;
}

static ntk_inline NTK_FRACT24 NTK_FpMul24X24(NTK_FRACT24 a,NTK_FRACT24 b)
{
    NTK_FRACT24 result;
    NTK_FRACT32 r_H;
    NTK_UINT32 r_L;
    NTK_SINT32 n = _NTK_DSP24_MANTBITS;
    NTK_SINT32 m = 32-_NTK_DSP24_MANTBITS;
    __asm{
    	smull r_L,r_H,a,b
    	mov r_L,r_L,lsr n
    	orr result,r_L,r_H,lsl m
    }
    return result;
}

static ntk_inline NTK_ACCU32 NTK_FpMac24X24(NTK_ACCU32 accu,NTK_FRACT24 b,NTK_FRACT24 c)
{
    NTK_ACCU32 result;
    NTK_FRACT32 r_H;
    NTK_UINT32 r_L;
    NTK_SINT32 n = _NTK_DSP24_MANTBITS;
    NTK_SINT32 m = 32-_NTK_DSP24_MANTBITS;
    __asm{
    	smull r_L,r_H,b,c
    	mov r_L,r_L,lsr n
    	orr result,r_L,r_H,lsl m
    	qadd result,accu,result
    }
    return result;
}

static ntk_inline NTK_ACCU32 NTK_FpMas24X24(NTK_ACCU32 accu,NTK_FRACT24 b,NTK_FRACT24 c)
{
    NTK_ACCU32 result;
    NTK_FRACT32 r_H;
    NTK_UINT32 r_L;
    NTK_SINT32 n = _NTK_DSP24_MANTBITS;
    NTK_SINT32 m = 32-_NTK_DSP24_MANTBITS;
    __asm{
    	smull r_L,r_H,b,c
    	mov r_L,r_L,lsr n
    	orr result,r_L,r_H,lsl m
    	qsub result,accu,result
    }
    return result;
}

static ntk_inline NTK_FRACT24 NTK_SatFpMul24X24(NTK_FRACT24 a,NTK_FRACT24 b)
{
    NTK_FRACT24 result;
    NTK_FRACT32 r_H;
    NTK_UINT32 r_L;
    NTK_SINT32 n = _NTK_DSP24_MANTBITS;
    NTK_SINT32 m = 32-_NTK_DSP24_MANTBITS;
    __asm{
    	smull r_L,r_H,a,b
    	mov r_L,r_L,lsr n
    	orr result,r_L,r_H,lsl m
    	ssat result,#24,result
    }
    return result;
}

static ntk_inline NTK_ACCU32 NTK_SatFpMac24X24(NTK_ACCU32 accu,NTK_FRACT24 b,NTK_FRACT24 c)
{
    NTK_ACCU32 result;
    NTK_FRACT32 r_H;
    NTK_UINT32 r_L;
    NTK_SINT32 n = _NTK_DSP24_MANTBITS;
    NTK_SINT32 m = 32-_NTK_DSP24_MANTBITS;
    __asm{
    	smull r_L,r_H,b,c
    	mov r_L,r_L,lsr n
    	orr result,r_L,r_H,lsl m
    	add result,accu,result
    	ssat result,#24,result
    }
    return result;
}

static ntk_inline NTK_ACCU32 NTK_SatFpMas24X24(NTK_ACCU32 accu,NTK_FRACT24 b,NTK_FRACT24 c)
{
    NTK_ACCU32 result;
    NTK_FRACT32 r_H;
    NTK_UINT32 r_L;
    NTK_SINT32 n = _NTK_DSP24_MANTBITS;
    NTK_SINT32 m = 32-_NTK_DSP24_MANTBITS;
    __asm{
    	smull r_L,r_H,b,c
    	mov r_L,r_L,lsr n
    	orr result,r_L,r_H,lsl m
    	sub result,accu,result
    	ssat result,#24,result
    }
    return result;
}

static ntk_inline NTK_FRACT16 NTK_FpMul16X16(NTK_FRACT16 a,NTK_FRACT16 b)
{
	NTK_SINT32 result;
    __asm{
    	smulbb result,a,b
    	mov result,result,lsr #15
    }
    return (NTK_FRACT16)result;
}

static ntk_inline NTK_ACCU32 NTK_FpMac16X16(NTK_ACCU32 accu,NTK_FRACT16 b,NTK_FRACT16 c)
{
    NTK_ACCU32 result;
    __asm{
    	smulbb result,b,c
    	add result,accu,result,asr #15
    }
    return result;
}
static ntk_inline NTK_ACCU32 NTK_FpMas16X16(NTK_ACCU32 accu,NTK_FRACT16 b,NTK_FRACT16 c)
{
    NTK_ACCU32 result;
    __asm{
    	smulbb result,b,c
    	sub result,accu,result,asr #15
    }
    return result;
}

static ntk_inline NTK_FRACT32 NTK_Mul16X16_D(NTK_FRACT16 a,NTK_FRACT16 b) 
{
	NTK_ACCU32 result;
    __asm{
    	smulbb result,a,b
    	qadd result,result,result
    }
	return result;
}

static ntk_inline NTK_ACCU32 NTK_Mac16X16_D(NTK_ACCU32 accu,NTK_FRACT16 b,NTK_FRACT16 c)
{
	NTK_ACCU32 result;
    __asm{
    	smulbb result,b,c
    	qdadd result,accu,result
    }
	return result;
}

static ntk_inline NTK_ACCU32 NTK_Mas16X16_D(NTK_ACCU32 accu,NTK_FRACT16 b,NTK_FRACT16 c)
{
	NTK_ACCU32 result;
    __asm{
    	smulbb result,b,c
    	qdsub result,accu,result
    }
	return result;
}
#endif
