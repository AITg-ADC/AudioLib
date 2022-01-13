#ifndef _NTK_OPT_SWITCH_H_
#define _NTK_OPT_SWITCH_H_

#if defined(__GNUC__) && defined(__arm__)

#define NTK_sub16(a, b) NTK_SatSub16(a, b)
#define NTK_add16(a, b) NTK_SatAdd16(a, b)
#define NTK_sub32(a, b) NTK_SatSub32(a, b)
#define NTK_add32(a, b) NTK_SatAdd32(a, b)
#define NTK_SHL16(a, b) NTK_SatShiftL16(a, b)
#define NTK_SHL32(a, b) NTK_SatShiftL32(a, b)
#define NTK_MULT16_32_Q15(a, b) NTK_FpMul32X16(b, a)
#define NTK_MULT16_32_P15(a, b) NTK_FpMul32X16RoundShift15(b, a)
#define NTK_MAC16_32_Q15(c, a, b) NTK_FpMac32X16(c, b, a)
#define NTK_PSHR16(a, b) NTK_RoundShiftR32(a, b)
#define NTK_PSHR32(a, b) NTK_RoundShiftR32(a, b)
#define NTK_EXTRACT16(a) NTK_Sat16(a)

#elif defined (__arm) && defined (__ARMCC_VERSION)

#define NTK_sub16(a, b) NTK_SatSub16(a, b)
#define NTK_add16(a, b) NTK_SatAdd16(a, b)
#define NTK_sub32(a, b) NTK_SatSub32(a, b)
#define NTK_add32(a, b) NTK_SatAdd32(a, b)
#define NTK_SHL16(a, b) NTK_SatShiftL16(a, b)
#define NTK_SHL32(a, b) NTK_SatShiftL32(a, b)
#define NTK_MULT16_32_Q15(a, b) NTK_FpMul32X16(b, a)
//#define NTK_MULT16_32_P15(a, b) NTK_FpMul32X16RoundShift15(b, a) // DJ's rightshift rounding is different from AEC's, so a one difference exists. More slower
#define NTK_MULT16_32_P15(a, b) MULT16_32_P15(a, b)
#define NTK_MAC16_32_Q15(c, a, b) NTK_FpMac32X16(c, b, a)
#define NTK_PSHR16(a, b) NTK_RoundShiftR32(a, b)
#define NTK_PSHR32(a, b) NTK_RoundShiftR32(a, b)
#define NTK_EXTRACT16(a) NTK_Sat16(a)

#else

#define NTK_add16(a, b) add16(a, b)
#define NTK_sub16(a, b) sub16(a, b)
#define NTK_add32(a, b) add32(a, b)
#define NTK_sub32(a, b) sub32(a, b)
#define NTK_SHL16(a, b) SHL16(a, b)
#define NTK_SHL32(a, b) SHL32(a, b)
#define NTK_MULT16_32_Q15(a, b) MULT16_32_Q15(a, b)
#define NTK_MULT16_32_P15(a, b) MULT16_32_P15(a, b)
#define NTK_MAC16_32_Q15(c, a, b) MAC16_32_Q15(c, a, b)
#define NTK_PSHR16(a, b) PSHR16(a, b)
#define NTK_PSHR32(a, b) PSHR32(a, b)
#define NTK_EXTRACT16(a) EXTRACT16(a)

#endif
#endif
