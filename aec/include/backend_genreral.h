#ifndef _NTK_BACKEND_GENERAL_H_
#define _NTK_BACKEND_GENERAL_H_

//Add,Subtract
static ntk_inline NTK_FRACT32 NTK_SatAdd32(NTK_FRACT32 a,NTK_FRACT32 b)
{
    NTK_FRACT32 nAcc;
    nAcc = a + b;
    if( a>0 && b>0 )
    {
        if(nAcc<=0)
            nAcc = NTK_FRACT32_MAX;
    }
    else if( a<0 && b<0 )
    {
        if(nAcc>=0)
            nAcc = NTK_FRACT32_MIN;
    }
    return nAcc;
}

static ntk_inline NTK_FRACT32 NTK_SatSub32(NTK_FRACT32 a,NTK_FRACT32 b)
{
    NTK_FRACT32 nAcc;
    nAcc = a - b;
    if( a>0 && b<0 )
    {
        if(nAcc<=0)
            nAcc = NTK_FRACT32_MAX;
    }
    else if( a<0 && b>0 )
    {
        if(nAcc>=0)
            nAcc = NTK_FRACT32_MIN;
    }
    return nAcc;
}

static ntk_inline NTK_FRACT24 NTK_SatAdd24(NTK_FRACT24 a,NTK_FRACT24 b)
{
    NTK_FRACT24 nAcc;
    nAcc = a + b;
    nAcc = NTK_Sat24(nAcc);
    return nAcc;

}

static ntk_inline NTK_FRACT24 NTK_SatSub24(NTK_FRACT24 a,NTK_FRACT24 b)
{
    NTK_FRACT24 nAcc;
    nAcc = a - b;
    nAcc = NTK_Sat24(nAcc);
    return nAcc;

}

static ntk_inline NTK_FRACT16 NTK_SatAdd16(NTK_FRACT16 a,NTK_FRACT16 b)
{
    NTK_FRACT16 nAcc;
    nAcc = a + b;
    if( a>0 && b>0 )
    {
        if(nAcc<=0)
            nAcc = NTK_FRACT16_MAX;
    }
    else if( a<0 && b<0 )
    {
        if(nAcc>=0)
            nAcc = NTK_FRACT16_MIN;
    }
    return nAcc;
}

static ntk_inline NTK_FRACT16 NTK_SatSub16(NTK_FRACT16 a,NTK_FRACT16 b)
{
	NTK_FRACT16 nAcc;
	nAcc = a - b;
	if( a>=0 && b<0 )
	{
		if(nAcc<=0)
			nAcc = NTK_FRACT16_MAX;
	}
	else if( a<0 && b>0 )
	{
		if(nAcc>=0)
			nAcc = NTK_FRACT16_MIN;
	}
	return nAcc;
}


//Abs,Negation
static ntk_inline NTK_FRACT32 NTK_SatAbs32(NTK_FRACT32 a)
{
    if (a == NTK_FRACT32_MIN)
    {
        a = NTK_FRACT32_MAX;
    }
    return a > 0 ? a : -a;
}

static ntk_inline NTK_FRACT32 NTK_SatNeg32(NTK_FRACT32 a)
{
    NTK_FRACT32 result;
    if(a == NTK_FRACT32_MIN)
        result = NTK_FRACT32_MAX;
    else
        result = -a;
    return result;
}

static ntk_inline NTK_FRACT16 NTK_SatAbs16(NTK_FRACT16 a)
{
    if (a == NTK_FRACT16_MIN)
    {
        a = NTK_FRACT16_MAX;
    }
    return a > 0 ? a : -a;
}

static ntk_inline NTK_FRACT16 NTK_SatNeg16(NTK_FRACT16 a)
{
    NTK_FRACT16 result;
    if(a == NTK_FRACT16_MIN)
        result = NTK_FRACT16_MAX;
    else
        result = -a;
    return result;
}

static ntk_inline NTK_FRACT24 NTK_SatAbs24(NTK_FRACT24 a)
{
    NTK_FRACT24 result = a > 0 ? a : -a;
    return NTK_Sat24(result);
}

static ntk_inline NTK_FRACT24 NTK_SatNeg24(NTK_FRACT24 a)
{
    NTK_FRACT24 result = -a;
    return NTK_Sat24(result);
}

//Saturate,Shift,Normalize
static ntk_inline NTK_SINT32  NTK_Norm32(NTK_FRACT32 a)
{
    NTK_SINT32 nShift;
	NTK_SINT32 nValue, nMask;
	int i;

	nValue = a;
	if(nValue<0)
		nValue = ~nValue;

	nShift = 0;
	nMask = 0x40000000;
	for(i=0; i<31; i++)
	{
		if(nMask & nValue)
			break;
		nMask >>= 1;
		nShift++;
	}

	return nShift;
}

static ntk_inline NTK_SINT32  NTK_Norm24(NTK_FRACT32 a)
{
	return (NTK_Norm32(a)-8);
}

static ntk_inline NTK_SINT32  NTK_Norm16(NTK_FRACT16 a)
{
    return (NTK_Norm32((NTK_FRACT32)a)-16);
}

static ntk_inline NTK_SINT32 NTK_RoundShiftR32(NTK_SINT32 a,NTK_SINT32 b)
{
    NTK_SINT32 result;
    if(b>0)
    {
    	result = NTK_SatAdd32(a,(1<<(b-1)))>>b;
    }
    else
    {
    	result = a;
    }
    return result;
}

static ntk_inline NTK_SINT16  NTK_SatRoundShiftR32to16(NTK_SINT32 a,NTK_SINT32 b)
{
    NTK_SINT32 result;
    if(b>0)
    {
    	result = NTK_SatAdd32(a,(1<<(b-1)))>>b;
    }
    else
    {
    	result = a;
    }
    result = NTK_Sat16(result);
    return (NTK_SINT16)result;
}

static ntk_inline NTK_SINT32  NTK_SatShiftL32(NTK_SINT32 a,NTK_SINT32 b)
{
    NTK_SINT32 result;
    NTK_SINT32 m=0x7fffffff;
    result = a<<b;
    if(a!=(result>>b))
        result = m^(a>>31);
    return result;
}

static ntk_inline NTK_SINT16  NTK_SatShiftL16(NTK_SINT16 a,NTK_SINT32 b)
{
    NTK_SINT16 result;
    NTK_SINT16 m=0x7fff;
    result = a<<b;
    if(a!=(result>>b))
        result = m^(a>>15);
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
    if(a>NTK_FRACT24_MAX)
        out = NTK_FRACT24_MAX;
    else if(a<NTK_FRACT24_MIN)
        out = NTK_FRACT24_MIN;
    return out;
}

static ntk_inline NTK_SINT16  NTK_Sat16(NTK_SINT32 a)
{
    NTK_SINT32 out = a;
    if(a>NTK_FRACT16_MAX)
        out = NTK_FRACT16_MAX;
    else if(a<NTK_FRACT16_MIN)
        out = NTK_FRACT16_MIN;
    return (NTK_SINT16)out;
}

static ntk_inline NTK_SINT32  NTK_Min(NTK_SINT32 a,NTK_SINT32 b)
{
    return (a>b?b:a);
}

static ntk_inline NTK_SINT32  NTK_Max(NTK_SINT32 a,NTK_SINT32 b)
{
    return (a<b?b:a);
}

//Multiply
static ntk_inline NTK_FRACT32 NTK_Mul32X32_RShiftN(NTK_FRACT32 a,NTK_FRACT32 b,NTK_SINT32 n)
{
    NTK_FRACT32 result;
    result = (NTK_FRACT32)NTK_Sat32(_NTK_mul64(a,b,n));
    return result;
}

static ntk_inline NTK_FRACT32 NTK_Mul32X32_RoundShiftN(NTK_FRACT32 a,NTK_FRACT32 b,NTK_SINT32 n)
{
	NTK_SINT64 result;
	if(n>0)
	{
		result = (NTK_SINT64)a*b+(1<<(n-1));
		result = (NTK_FRACT32)NTK_Sat32(result>>n);
	}
	else
	{
		result = (NTK_FRACT32)NTK_Sat32((NTK_SINT64)a*b);
	}
	return result;
}

static ntk_inline NTK_FRACT32 NTK_FpMul32X32(NTK_FRACT32 a,NTK_FRACT32 b)
{
    NTK_FRACT32 result;
    result = (NTK_FRACT32)_NTK_mul64(a,b,_NTK_LFRACT_MANTBITS);
    return result;
}

static ntk_inline NTK_FRACT32 NTK_Mul32X32_RShift32(NTK_FRACT32 a,NTK_FRACT32 b)
{
    NTK_FRACT32 result;
    result = (NTK_FRACT32)_NTK_mul64(a,b,32);
    return result;
}

static ntk_inline NTK_ACCU32 NTK_FpMac32X32(NTK_ACCU32 accu,NTK_FRACT32 b,NTK_FRACT32 c)
{
    NTK_ACCU32 result;

    result = NTK_SatAdd32(accu,(NTK_FRACT32)_NTK_mul64(b,c,_NTK_LFRACT_MANTBITS));
    return result;
}

static ntk_inline NTK_ACCU32 NTK_FpMas32X32(NTK_ACCU32 accu,NTK_FRACT32 b,NTK_FRACT32 c)
{
    NTK_ACCU32 result;

    result = NTK_SatSub32(accu,(NTK_FRACT32)_NTK_mul64(b,c,_NTK_LFRACT_MANTBITS));
    return result;
}

static ntk_inline NTK_FRACT32 NTK_FpMul32X16(NTK_FRACT32 a,NTK_FRACT16 b)
{
    NTK_FRACT32 result;
    result = (NTK_FRACT32)_NTK_mul64(a,b,_NTK_SFRACT_MANTBITS);
    return result;
}

static ntk_inline NTK_FRACT32 NTK_FpMul32X16RoundShift16(NTK_FRACT32 a, NTK_FRACT16 b)
{
  NTK_ACCU64 acc;
  acc = (NTK_FRACT32)(NTK_UINT16)(a) * (NTK_FRACT32)(b);
  acc += 0x8000;
  acc >>= 16;
  acc += ((a)>>16) * (NTK_FRACT32)(b);

  return (NTK_FRACT32)acc;
}

static ntk_inline NTK_FRACT32 NTK_FpMul32X16RoundShift15(NTK_FRACT32 a, NTK_FRACT16 b)
{
  NTK_ACCU64 acc;
  acc = (NTK_ACCU64)(a) * b;
  acc += 0x4000;
  acc >>= 15;

  return (NTK_FRACT32)acc;
}

static ntk_inline NTK_FRACT32 NTK_Mul32X16_RShift16(NTK_FRACT32 a,NTK_FRACT16 b)
{
	NTK_FRACT32 result;
    result = (NTK_FRACT32)_NTK_mul64(a,b,16);
    return result;
}

static ntk_inline NTK_ACCU32 NTK_FpMac32X16(NTK_ACCU32 accu,NTK_FRACT32 b,NTK_FRACT16 c)
{
    NTK_ACCU32 result;

    result = NTK_SatAdd32(accu,(NTK_FRACT32)_NTK_mul64(b,c,_NTK_SFRACT_MANTBITS));
    return result;
}

static ntk_inline NTK_ACCU32 NTK_FpMas32X16(NTK_ACCU32 accu,NTK_FRACT32 b,NTK_FRACT16 c)
{
    NTK_ACCU32 result;

    result = NTK_SatSub32(accu,(NTK_FRACT32)_NTK_mul64(b,c,_NTK_SFRACT_MANTBITS));
    return result;
}

static ntk_inline NTK_FRACT24 NTK_FpMul24X24(NTK_FRACT24 a,NTK_FRACT24 b)
{
    NTK_FRACT24 result;
    result = (NTK_FRACT32)_NTK_mul64(a,b,_NTK_DSP24_MANTBITS);

    return result;
}

static ntk_inline NTK_ACCU32 NTK_FpMac24X24(NTK_ACCU32 accu,NTK_FRACT24 b,NTK_FRACT24 c)
{
    NTK_ACCU32 result;

    result = NTK_SatAdd32(accu,(NTK_FRACT24)_NTK_mul64(b,c,_NTK_DSP24_MANTBITS));
    return result;
}

static ntk_inline NTK_ACCU32 NTK_FpMas24X24(NTK_ACCU32 accu,NTK_FRACT24 b,NTK_FRACT24 c)
{
    NTK_ACCU32 result;

    result = NTK_SatSub32(accu,(NTK_FRACT24)_NTK_mul64(b,c,_NTK_DSP24_MANTBITS));
    return result;
}

static ntk_inline NTK_FRACT24 NTK_SatFpMul24X24(NTK_FRACT24 a,NTK_FRACT24 b)
{
    NTK_FRACT24 result;
    result = (NTK_FRACT32)_NTK_mul64(a,b,_NTK_DSP24_MANTBITS);

    return NTK_Sat24(result);
}

static ntk_inline NTK_ACCU32 NTK_SatFpMac24X24(NTK_ACCU32 accu,NTK_FRACT24 b,NTK_FRACT24 c)
{
    NTK_ACCU32 result;

    result = NTK_SatAdd24(accu,(NTK_FRACT24)_NTK_mul64(b,c,_NTK_DSP24_MANTBITS));
    return result;
}

static ntk_inline NTK_ACCU32 NTK_SatFpMas24X24(NTK_ACCU32 accu,NTK_FRACT24 b,NTK_FRACT24 c)
{
    NTK_ACCU32 result;

    result = NTK_SatSub24(accu,(NTK_FRACT24)_NTK_mul64(b,c,_NTK_DSP24_MANTBITS));
    return result;
}

static ntk_inline NTK_FRACT16 NTK_FpMul16X16(NTK_FRACT16 a,NTK_FRACT16 b)
{
    NTK_FRACT16 result;
    result = (NTK_FRACT16)_NTK_mul64(a,b,_NTK_SFRACT_MANTBITS);
    return result;
}

static ntk_inline NTK_ACCU32 NTK_FpMac16X16(NTK_ACCU32 accu,NTK_FRACT16 b,NTK_FRACT16 c)
{
    NTK_ACCU32 result;

    result = NTK_SatAdd32(accu,(NTK_FRACT32)_NTK_mul64(b,c,_NTK_SFRACT_MANTBITS));
    return result;
}
static ntk_inline NTK_ACCU32 NTK_FpMas16X16(NTK_ACCU32 accu,NTK_FRACT16 b,NTK_FRACT16 c)
{
    NTK_ACCU32 result;

    result = NTK_SatSub32(accu,(NTK_FRACT32)_NTK_mul64(b,c,_NTK_SFRACT_MANTBITS));
    return result;
}

static ntk_inline NTK_FRACT32 NTK_Mul16X16_D(NTK_FRACT16 a,NTK_FRACT16 b) 
{
	NTK_ACCU32 result;
	result = NTK_Sat32(((NTK_SINT64)a*b)<<1);
	return result;
}

static ntk_inline NTK_ACCU32 NTK_Mac16X16_D(NTK_ACCU32 accu,NTK_FRACT16 b,NTK_FRACT16 c)
{
	NTK_ACCU32 result;
	result = NTK_SatAdd32(accu,NTK_Mul16X16_D(b,c));
	return result;
}

static ntk_inline NTK_ACCU32 NTK_Mas16X16_D(NTK_ACCU32 accu,NTK_FRACT16 b,NTK_FRACT16 c)
{
	NTK_ACCU32 result;
	result = NTK_SatSub32(accu,NTK_Mul16X16_D(b,c));
	return result;
}
#endif
