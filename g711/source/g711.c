/**
    @file       G711.c
    @ingroup    mIAVG711

    @brief      G.711 u-law, A-law Encode/Decode Library
    @version    V1.00.000
    @date       2015/08/17

*/

#include "audlib_g711.h"





/*
 * G711.c
 *
 * u-law, A-law and linear PCM conversions.
 */
#define SIGN_BIT    (0x80)      /* Sign bit for a A-law byte. */
#define QUANT_MASK  (0xf)       /* Quantization field mask. */
#define NSEGS       (8)         /* Number of A-law segments. */
#define SEG_SHIFT   (4)         /* Left shift for segment number. */
#define SEG_MASK    (0x70)      /* Segment field mask. */
#define BIAS        (0x84)      /* Bias for linear code. */

#define SWAP2(a)    (((a<<8)&0xff00)|((a>>8)&0xff))

#ifndef E_OK
#define E_OK	(0)
#endif


static INT16 seg_end[8] = {0xFF, 0x1FF, 0x3FF, 0x7FF,
						   0xFFF, 0x1FFF, 0x3FFF, 0x7FFF
						  };
/*
// copy from CCITT G.711 specifications
UINT8 _u2a[128] = {     // u- to A-law conversions
  1,  1,  2,  2,  3,  3,  4,  4,
  5,  5,  6,  6,  7,  7,  8,  8,
  9,  10, 11, 12, 13, 14, 15, 16,
  17, 18, 19, 20, 21, 22, 23, 24,
  25, 27, 29, 31, 33, 34, 35, 36,
  37, 38, 39, 40, 41, 42, 43, 44,
  46, 48, 49, 50, 51, 52, 53, 54,
  55, 56, 57, 58, 59, 60, 61, 62,
  64, 65, 66, 67, 68, 69, 70, 71,
  72, 73, 74, 75, 76, 77, 78, 79,
  81, 82, 83, 84, 85, 86, 87, 88,
  89, 90, 91, 92, 93, 94, 95, 96,
  97, 98, 99, 100,  101,  102,  103,  104,
  105,  106,  107,  108,  109,  110,  111,  112,
  113,  114,  115,  116,  117,  118,  119,  120,
  121,  122,  123,  124,  125,  126,  127,  128};

UINT8 _a2u[128] = {     // A- to u-law conversions
  1,  3,  5,  7,  9,  11, 13, 15,
  16, 17, 18, 19, 20, 21, 22, 23,
  24, 25, 26, 27, 28, 29, 30, 31,
  32, 32, 33, 33, 34, 34, 35, 35,
  36, 37, 38, 39, 40, 41, 42, 43,
  44, 45, 46, 47, 48, 48, 49, 49,
  50, 51, 52, 53, 54, 55, 56, 57,
  58, 59, 60, 61, 62, 63, 64, 64,
  65, 66, 67, 68, 69, 70, 71, 72,
  73, 74, 75, 76, 77, 78, 79, 79,
  80, 81, 82, 83, 84, 85, 86, 87,
  88, 89, 90, 91, 92, 93, 94, 95,
  96, 97, 98, 99, 100,  101,  102,  103,
  104,  105,  106,  107,  108,  109,  110,  111,
  112,  113,  114,  115,  116,  117,  118,  119,
  120,  121,  122,  123,  124,  125,  126,  127};
*/

static INT32 search(INT32 val, INT16 *table, INT32 size)
{
	INT32 i;

	for (i = 0; i < size; i++) {
		if (val <= *table++) {
			return i;
		}
	}

	return size;
}

ER g711_ulaw_encode(INT16 *p_data_in, UINT8 *p_data_out, UINT32 sample_count, BOOL input_swap)
{
	UINT32  i;
	INT32   mask, seg, pcm_val;
	UINT8   uval;

	for (i = 0; i < sample_count; i++) {
		if (input_swap) {
			pcm_val = (INT32)SWAP2(*(p_data_in + i));
		} else {
			pcm_val = (INT32) * (p_data_in + i);
		}

		/* Get the sign and the magnitude of the value. */
		if (pcm_val < 0) {
			pcm_val = BIAS - pcm_val;
			mask = 0x7F;
		} else {
			pcm_val += BIAS;
			mask = 0xFF;
		}

		/* Convert the scaled magnitude to segment number. */
		seg = search(pcm_val, seg_end, 8);

		/*
		 * Combine the sign, segment, quantization bits;
		 * and complement the code word.
		 */
		if (seg >= 8) {   /* out of range, return maximum value. */
			*(p_data_out + i) = (UINT8)(0x7F ^ mask);
		} else {
			uval = (seg << 4) | ((pcm_val >> (seg + 3)) & 0xF);
			*(p_data_out + i) = (UINT8)(uval ^ mask);
		}
	}

	return E_OK;
}

ER g711_ulaw_decode(UINT8 *p_data_in, INT16 *p_data_out, UINT32 sample_count, BOOL duplicate_channel, BOOL output_swap)
{
	UINT32  i;
	INT32   t;
	UINT8   u_val;

	for (i = 0; i < sample_count; i++) {
		/* Complement to obtain normal u-law value. */
		u_val = *(p_data_in + i);
		u_val = ~u_val;

		/*
		 * Extract and bias the quantization bits. Then
		 * shift up by the segment number and subtract out the bias.
		 */
		t = ((u_val & QUANT_MASK) << 3) + BIAS;
		t <<= ((unsigned)u_val & SEG_MASK) >> SEG_SHIFT;

		if (duplicate_channel) {
			*(p_data_out + (i << 1)) = (INT16)((u_val & SIGN_BIT) ? (BIAS - t) : (t - BIAS));
			if (output_swap) {
				*(p_data_out + (i << 1)) = (INT16)SWAP2(*(p_data_out + (i << 1)));
			}
			*(p_data_out + (i << 1) + 1) = *(p_data_out + (i << 1));
		} else {
			*(p_data_out + i) = (INT16)((u_val & SIGN_BIT) ? (BIAS - t) : (t - BIAS));
			if (output_swap) {
				*(p_data_out + i) = (INT16)SWAP2(*(p_data_out + i));
			}
		}
	}

	return E_OK;
}

ER g711_alaw_encode(INT16 *p_data_in, UINT8 *p_data_out, UINT32 sample_count, BOOL input_swap)
{
	INT32   mask, seg, pcm_val;
	UINT8   aval;
	UINT32  i;

	for (i = 0; i < sample_count; i++) {
		if (input_swap) {
			pcm_val = (INT32)SWAP2(*(p_data_in + i));
		} else {
			pcm_val = (INT32) * (p_data_in + i);
		}

		if (pcm_val >= 0) {
			mask = 0xD5;    /* sign (7th) bit = 1 */
		} else {
			mask = 0x55;    /* sign bit = 0 */
			pcm_val = -pcm_val - 1;
		}

		/* Convert the scaled magnitude to segment number. */
		seg = search(pcm_val, seg_end, 8);

		/* Combine the sign, segment, and quantization bits. */

		if (seg >= 8) {   /* out of range, return maximum value. */
			*(p_data_out + i) = (UINT8)(0x7F ^ mask);
		} else {
			aval = seg << SEG_SHIFT;
			if (seg < 2) {
				aval |= (pcm_val >> 4) & QUANT_MASK;
			} else {
				aval |= (pcm_val >> (seg + 3)) & QUANT_MASK;
			}

			*(p_data_out + i) = (UINT8)(aval ^ mask);
		}
	}

	return E_OK;
}

ER g711_alaw_decode(UINT8 *p_data_in, INT16 *p_data_out, UINT32 sample_count, BOOL duplicate_channel, BOOL output_swap)
{
	UINT32  i;
	INT32   t, seg;
	UINT8   a_val;

	for (i = 0; i < sample_count; i++) {
		a_val = *(p_data_in + i);
		a_val ^= 0x55;

		t = (a_val & QUANT_MASK) << 4;
		seg = ((unsigned)a_val & SEG_MASK) >> SEG_SHIFT;
		switch (seg) {
		case 0:
			t += 8;
			break;
		case 1:
			t += 0x108;
			break;
		default:
			t += 0x108;
			t <<= seg - 1;
		}
		if (duplicate_channel) {
			*(p_data_out + (i << 1)) = (INT16)((a_val & SIGN_BIT) ? t : -t);
			if (output_swap) {
				*(p_data_out + (i << 1)) = (INT16)SWAP2(*(p_data_out + (i << 1)));
			}
			*(p_data_out + (i << 1) + 1) = *(p_data_out + (i << 1)) ;
		} else {
			*(p_data_out + i) = (INT16)((a_val & SIGN_BIT) ? t : -t);
			if (output_swap) {
				*(p_data_out + i) = (INT16)SWAP2(*(p_data_out + i));
			}
		}
	}

	return E_OK;
}

/*
// A-law to u-law conversion
UINT8 alaw2ulaw(UINT8 aval)
{
  aval &= 0xff;
  return ((aval & 0x80) ? (0xFF ^ _a2u[aval ^ 0xD5]) :
         (0x7F ^ _a2u[aval ^ 0x55]));
}

// u-law to A-law conversion
UINT8 ulaw2alaw(UINT8 uval)
{
  uval &= 0xff;
  return ((uval & 0x80) ? (0xD5 ^ (_u2a[0xFF ^ uval] - 1)) :
         (0x55 ^ (_u2a[0x7F ^ uval] - 1)));
}
*/

#ifdef __KERNEL__
EXPORT_SYMBOL(g711_ulaw_encode);
EXPORT_SYMBOL(g711_ulaw_decode);
EXPORT_SYMBOL(g711_alaw_encode);
EXPORT_SYMBOL(g711_alaw_decode);
#endif
