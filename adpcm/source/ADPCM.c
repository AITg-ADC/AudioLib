/*
    ADPCM Encode/Decode Library

    This file is the IMA ADPCM Encode/Decode Library.

    @file       ADPCM.c
    @ingroup    mIAVADPCM
    @note       Nothing.

*/

#include <stdio.h>
#include "audlib_adpcm.h"

#define ADPCM_LIB_VERSION     "1.00.00"


// ADPCM step variation table
static INT32    iIndexTable[16] = {
	-1, -1, -1, -1, 2, 4, 6, 8,
	-1, -1, -1, -1, 2, 4, 6, 8,
};

static INT32    iStepSizeTable[89] = {
	7, 8, 9, 10, 11, 12, 13, 14, 16, 17,
	19, 21, 23, 25, 28, 31, 34, 37, 41, 45,
	50, 55, 60, 66, 73, 80, 88, 97, 107, 118,
	130, 143, 157, 173, 190, 209, 230, 253, 279, 307,
	337, 371, 408, 449, 494, 544, 598, 658, 724, 796,
	876, 963, 1060, 1166, 1282, 1411, 1552, 1707, 1878, 2066,
	2272, 2499, 2749, 3024, 3327, 3660, 4026, 4428, 4871, 5358,
	5894, 6484, 7132, 7845, 8630, 9493, 10442, 11487, 12635, 13899,
	15289, 16818, 18500, 20350, 22385, 24623, 27086, 29794, 32767
};

/**
    Encode 16bits mono PCM data to IMA ADPCM data.

    This function encode 16bits mono PCM data to IMA ADPCM data.
    You have to handle packet header by yourself.

    @param[in] p_data_in          Memory address of 16bits mono PCM data
    @param[in] p_data_out         Memory address of mono IMA ADPCM data
    @param[in] sample_count    Sample count of PCM data
    @param[in] adpcm_state           Pointer of previous value & index data
    @return The encoded data length
*/
UINT32 audlib_adpcm_encode_mono(INT16 *p_data_in, INT8 *p_data_out, UINT32 sample_count, PADPCM_STATE adpcm_state)
{
	INT16   *pLIn;              // Input buffer pointer
	INT8    *pLOut;             // Output buffer pointer
	INT32   iLVal;              // Current input sample value
	INT32   iLSign;             // Current adpcm sign bit
	INT32   iLDelta;            // Current adpcm output value
	INT32   iLDiff;             // Difference between Val and ValPrev
	INT32   iLStep;             // Stepsize
	INT32   iLValPred;          // Predicted output value
	INT32   iLVPDiff;           // Current change to ValPred
	INT32   iLIndex;            // Current Step change Index
	INT32   iLOutputBuffer;     // Place to keep previous 4-bit value
	INT32   iLBufferStep;       // Toggle between OutputBuffer/output
	INT32   iLByteCount;        // Output byte count

	iLOutputBuffer  = 0;

	pLOut           = p_data_out;
	pLIn            = p_data_in;

	iLValPred       = adpcm_state->l_val_prev;
	iLIndex         = adpcm_state->l_index;
	iLStep          = iStepSizeTable[iLIndex];

	iLBufferStep    = 1;
	iLByteCount     = 0;

	for (; sample_count > 0; sample_count--) {
		iLVal = *pLIn++;

		// Step 1 - compute difference with previous value
		iLDiff = iLVal - iLValPred;
		iLSign = (iLDiff < 0) ? 8 : 0;
		if (iLSign) {
			iLDiff = (-iLDiff);
		}

		// Step 2 - Divide and clamp
		iLDelta = 0;
		iLVPDiff = (iLStep >> 3);

		if (iLDiff >= iLStep) {
			iLDelta   = 4;
			iLDiff   -= iLStep;
			iLVPDiff += iLStep;
		}

		iLStep >>= 1;
		if (iLDiff >= iLStep) {
			iLDelta  |= 2;
			iLDiff   -= iLStep;
			iLVPDiff += iLStep;
		}

		iLStep >>= 1;
		if (iLDiff >= iLStep) {
			iLDelta  |= 1;
			iLVPDiff += iLStep;
		}

		// Step 3 - Update previous value
		if (iLSign) {
			iLValPred -= iLVPDiff;
		} else {
			iLValPred += iLVPDiff;
		}

		// Step 4 - Clamp previous value to 16 bits
		if (iLValPred > 32767) {
			iLValPred = 32767;
		} else if (iLValPred < -32768) {
			iLValPred = -32768;
		}

		// Step 5 - Assemble value, update index and step values
		iLDelta |= iLSign;

		iLIndex += iIndexTable[iLDelta];
		if (iLIndex < 0) {
			iLIndex = 0;
		} else if (iLIndex > 88) {
			iLIndex = 88;
		}
		iLStep = iStepSizeTable[iLIndex];

		// Step 6 - Output value
		if (iLBufferStep) {
			iLOutputBuffer = iLDelta & 0x0F;
		} else {
			*pLOut++    = (INT8)(((iLDelta << 4) & 0xF0) | iLOutputBuffer);
			iLByteCount = (iLByteCount + 1) & 0x03;
		}
		iLBufferStep = !iLBufferStep;
	}

	// Output last step, if needed
	if (!iLBufferStep) {
		*pLOut++    = (INT8)iLOutputBuffer;
		iLByteCount = (iLByteCount + 1) & 0x03;
	}

	// padding zero
	if (iLByteCount != 0) {
		for (iLByteCount = 4 - iLByteCount; iLByteCount > 0; iLByteCount--) {
			*pLOut++ = 0;
		}
	}

	// Save Value and Index for next encode
	adpcm_state->l_val_prev    = (INT16)iLValPred;
	adpcm_state->l_index      = (INT8)iLIndex;

	return ((UINT32)pLOut - (UINT32)p_data_out);
}

/**
    Encode 16bits stereo PCM data to IMA ADPCM data.

    This function encode 16bits stereo PCM data to IMA ADPCM data.
    You have to handle packet header by yourself.

    @param[in] p_data_in          Memory address of 16bits stereo PCM data
    @param[in] p_data_out         Memory address of stereo IMA ADPCM data
    @param[in] sample_count    Sample count of PCM data
    @param[in] adpcm_state           Pointer of previous value & index data
    @return The encoded data length
*/
UINT32 audlib_adpcm_encode_stereo(INT16 *p_data_in, INT8 *p_data_out, UINT32 sample_count, PADPCM_STATE adpcm_state)
{
	INT16   *pLIn, *pRIn;                   // Input buffer pointer
	INT8    *pLOut, *pROut;                 // Output buffer pointer
	INT32   iLVal, iRVal;                   // Current input sample value
	INT32   iLSign, iRSign;                 // Current adpcm sign bit
	INT32   iLDelta, iRDelta;               // Current adpcm output value
	INT32   iLDiff, iRDiff;                 // Difference between Val and ValPrev
	INT32   iLStep, iRStep;                 // Stepsize
	INT32   iLValPred, iRValPred;           // Predicted output value
	INT32   iLVPDiff, iRVPDiff;             // Current change to ValPred
	INT32   iLIndex, iRIndex;               // Current Step change Index
	INT32   iLOutputBuffer, iROutputBuffer; // Place to keep previous 4-bit value
	INT32   iLBufferStep, iRBufferStep;     // Toggle between OutputBuffer/output
	INT32   iLByteCount, iRByteCount;       // Output byte count

	iLOutputBuffer  = 0;
	iROutputBuffer  = 0;

	pLOut           = p_data_out;
	pLIn            = p_data_in;

	pROut           = p_data_out + 4;
	pRIn            = p_data_in + 1;

	iLValPred       = adpcm_state->l_val_prev;
	iLIndex         = adpcm_state->l_index;
	iLStep          = iStepSizeTable[iLIndex];

	iRValPred       = adpcm_state->r_val_prev;
	iRIndex         = adpcm_state->r_index;
	iRStep          = iStepSizeTable[iRIndex];

	iLBufferStep    = 1;
	iRBufferStep    = 1;

	iLByteCount     = 0;
	iRByteCount     = 0;

	for (; sample_count > 0; sample_count--) {
		// Left channel
		iLVal = *pLIn;
		pLIn += 2;

		// Step 1 - compute difference with previous value
		iLDiff = iLVal - iLValPred;
		iLSign = (iLDiff < 0) ? 8 : 0;
		if (iLSign) {
			iLDiff = (-iLDiff);
		}

		// Step 2 - Divide and clamp
		iLDelta = 0;
		iLVPDiff = (iLStep >> 3);

		if (iLDiff >= iLStep) {
			iLDelta   = 4;
			iLDiff   -= iLStep;
			iLVPDiff += iLStep;
		}

		iLStep >>= 1;
		if (iLDiff >= iLStep) {
			iLDelta  |= 2;
			iLDiff   -= iLStep;
			iLVPDiff += iLStep;
		}

		iLStep >>= 1;
		if (iLDiff >= iLStep) {
			iLDelta  |= 1;
			iLVPDiff += iLStep;
		}

		// Step 3 - Update previous value
		if (iLSign) {
			iLValPred -= iLVPDiff;
		} else {
			iLValPred += iLVPDiff;
		}

		// Step 4 - Clamp previous value to 16 bits
		if (iLValPred > 32767) {
			iLValPred = 32767;
		} else if (iLValPred < -32768) {
			iLValPred = -32768;
		}

		// Step 5 - Assemble value, update index and step values
		iLDelta |= iLSign;

		iLIndex += iIndexTable[iLDelta];
		if (iLIndex < 0) {
			iLIndex = 0;
		} else if (iLIndex > 88) {
			iLIndex = 88;
		}
		iLStep = iStepSizeTable[iLIndex];

		// Step 6 - Output value
		if (iLBufferStep) {
			iLOutputBuffer = iLDelta & 0x0F;
		} else {
			*pLOut++ = (INT8)(((iLDelta << 4) & 0xF0) | iLOutputBuffer);
			iLByteCount = (iLByteCount + 1) & 0x03;
			if (iLByteCount == 0) {
				pLOut += 4;
			}
		}
		iLBufferStep = !iLBufferStep;

		// Right channel
		iRVal = *pRIn;
		pRIn += 2;

		// Step 1 - compute difference with previous value
		iRDiff = iRVal - iRValPred;
		iRSign = (iRDiff < 0) ? 8 : 0;
		if (iRSign) {
			iRDiff = (-iRDiff);
		}

		// Step 2 - Divide and clamp
		iRDelta  = 0;
		iRVPDiff = (iRStep >> 3);

		if (iRDiff >= iRStep) {
			iRDelta   = 4;
			iRDiff   -= iRStep;
			iRVPDiff += iRStep;
		}

		iRStep >>= 1;
		if (iRDiff >= iRStep) {
			iRDelta  |= 2;
			iRDiff   -= iRStep;
			iRVPDiff += iRStep;
		}

		iRStep >>= 1;
		if (iRDiff >= iRStep) {
			iRDelta  |= 1;
			iRVPDiff += iRStep;
		}

		// Step 3 - Update previous value
		if (iRSign) {
			iRValPred -= iRVPDiff;
		} else {
			iRValPred += iRVPDiff;
		}

		// Step 4 - Clamp previous value to 16 bits
		if (iRValPred > 32767) {
			iRValPred = 32767;
		} else if (iRValPred < -32768) {
			iRValPred = -32768;
		}

		// Step 5 - Assemble value, update index and step values
		iRDelta |= iRSign;

		iRIndex += iIndexTable[iRDelta];
		if (iRIndex < 0) {
			iRIndex = 0;
		} else if (iRIndex > 88) {
			iRIndex = 88;
		}
		iRStep = iStepSizeTable[iRIndex];

		// Step 6 - Output value
		if (iRBufferStep) {
			iROutputBuffer = iRDelta & 0x0F;
		} else {
			*pROut++ = (INT8)(((iRDelta << 4) & 0xF0) | iROutputBuffer);
			iRByteCount = (iRByteCount + 1) & 0x03;
			if (iRByteCount == 0) {
				pROut += 4;
			}
		}
		iRBufferStep = !iRBufferStep;
	}

	// Output last step, if needed
	if (!iLBufferStep) {
		*pLOut++    = (INT8)iLOutputBuffer;
		iLByteCount = (iLByteCount + 1) & 0x03;
	}
	if (!iRBufferStep) {
		*pROut++    = (INT8)iROutputBuffer;
		iRByteCount = (iRByteCount + 1) & 0x03;
	}

	// padding zero
	if (iLByteCount != 0) {
		for (iLByteCount = 4 - iLByteCount; iLByteCount > 0; iLByteCount--) {
			*pLOut++ = 0;
		}
		// For calculating output stream length
		pLOut += 4;
	}
	if (iRByteCount != 0) {
		for (iRByteCount = 4 - iRByteCount; iRByteCount > 0; iRByteCount--) {
			*pROut++ = 0;
		}
	}

	// Save Value and Index for next encode
	adpcm_state->l_val_prev    = (INT16)iLValPred;
	adpcm_state->l_index      = (INT8)iLIndex;
	adpcm_state->r_val_prev    = (INT16)iRValPred;
	adpcm_state->r_index      = (INT8)iRIndex;

	return ((UINT32)pLOut - (UINT32)p_data_out);
}

/**
    Decode mono IMA ADPCM data to 16bits mono PCM data.

    This function decode mono IMA ADPCM data to 16bits mono PCM data.
    You have to handle packet header by yourself.

    @param[in] p_data_in          Memory address of mono IMA ADPCM data
    @param[in] p_data_out         Memory address of 16bits mono PCM data
    @param[in] sample_count    Sample count of IMA ADPCM data
    @param[in] adpcm_state           Pointer of previous value & index data
    @return The PCM data length
*/
UINT32 audlib_adpcm_decode_mono(INT8 *p_data_in, INT16 *p_data_out, UINT32 sample_count, PADPCM_STATE adpcm_state)
{
	INT8    *pLIn;                  // Input buffer pointer
	INT16   *pLOut;                 // output buffer pointer
	INT32   iLSign;                 // Current adpcm sign bit
	INT32   iLDelta;                // Current adpcm output value
	INT32   iLStep;                 // Stepsize
	INT32   iLValPred;              // Predicted value
	INT32   iLVPDiff;               // Current change to ValPred
	INT32   iLIndex;                // Current Step change Index
	INT32   iLInputBuffer;          // Place to keep next 4-bit value
	INT32   iLBufferStep;           // Toggle between InputBuffer/input

	iLInputBuffer   = 0;

	pLOut           = p_data_out;
	pLIn            = p_data_in;

	iLValPred       = adpcm_state->l_val_prev;
	iLIndex         = adpcm_state->l_index;
	iLStep          = iStepSizeTable[iLIndex];

	iLBufferStep    = 0;

	for (; sample_count > 0; sample_count--) {
		// Step 1 - get the Delta value
		if (iLBufferStep) {
			iLDelta = (iLInputBuffer >> 4) & 0x0F;
		} else {
			iLInputBuffer   = *pLIn++;
			iLDelta         = iLInputBuffer & 0x0F;
		}
		iLBufferStep = !iLBufferStep;

		// Step 2 - Find new index value (for later)
		iLIndex += iIndexTable[iLDelta];
		if (iLIndex < 0) {
			iLIndex = 0;
		} else if (iLIndex > 88) {
			iLIndex = 88;
		}

		// Step 3 - Separate sign and magnitude
		iLSign  = iLDelta & 8;
		iLDelta = iLDelta & 7;

		// Step 4 - Compute difference and new predicted value
		iLVPDiff = iLStep >> 3;
		if (iLDelta & 4) {
			iLVPDiff += iLStep;
		}
		if (iLDelta & 2) {
			iLVPDiff += iLStep >> 1;
		}
		if (iLDelta & 1) {
			iLVPDiff += iLStep >> 2;
		}

		if (iLSign) {
			iLValPred -= iLVPDiff;
		} else {
			iLValPred += iLVPDiff;
		}

		// Step 5 - clamp output value
		if (iLValPred > 32767) {
			iLValPred = 32767;
		} else if (iLValPred < -32768) {
			iLValPred = -32768;
		}

		// Step 6 - Update step value
		iLStep = iStepSizeTable[iLIndex];

		// Step 7 - Output value
		*pLOut++ = (INT16)iLValPred;
	}

	// Save Value and Index for next decode
	adpcm_state->l_val_prev    = (INT16)iLValPred;
	adpcm_state->l_index      = (INT8)iLIndex;

	return ((UINT32)pLOut - (UINT32)p_data_out);
}

/**
    Decode stereo IMA ADPCM data to 16bits stereo PCM data.

    This function decode stereo IMA ADPCM data to 16bits stereo PCM data.
    You have to handle packet header by yourself.

    @param[in] p_data_in          Memory address of stereo IMA ADPCM data
    @param[in] p_data_out         Memory address of 16bits stereo PCM data
    @param[in] sample_count    Sample count of IMA ADPCM data
    @param[in] adpcm_state           Pointer of previous value & index data
    @return The PCM data length
*/
UINT32 audlib_adpcm_decode_stereo(INT8 *p_data_in, INT16 *p_data_out, UINT32 sample_count, PADPCM_STATE adpcm_state)
{
	INT8    *pLIn, *pRIn;                   // Input buffer pointer
	INT16   *pLOut, *pROut;                 // output buffer pointer
	INT32   iLSign, iRSign;                 // Current adpcm sign bit
	INT32   iLDelta, iRDelta;               // Current adpcm output value
	INT32   iLStep, iRStep;                 // Stepsize
	INT32   iLValPred, iRValPred;           // Predicted value
	INT32   iLVPDiff, iRVPDiff;             // Current change to ValPred
	INT32   iLIndex, iRIndex;               // Current Step change Index
	INT32   iLInputBuffer, iRInputBuffer;   // Place to keep next 4-bit value
	INT32   iLBufferStep, iRBufferStep;     // Toggle between InputBuffer/input
	INT32   iLByteCount, iRByteCount;       // Input byte count

	iLInputBuffer   = 0;
	iRInputBuffer   = 0;

	pLOut           = p_data_out;
	pLIn            = p_data_in;

	pROut           = p_data_out + 1;
	pRIn            = p_data_in + 4;

	iLValPred       = adpcm_state->l_val_prev;
	iLIndex         = adpcm_state->l_index;
	iLStep          = iStepSizeTable[iLIndex];

	iRValPred       = adpcm_state->r_val_prev;
	iRIndex         = adpcm_state->r_index;
	iRStep          = iStepSizeTable[iRIndex];

	iLBufferStep    = 0;
	iRBufferStep    = 0;

	iLByteCount     = 0;
	iRByteCount     = 0;

	for (; sample_count > 0; sample_count--) {
		// Left channel
		// Step 1 - get the Delta value
		if (iLBufferStep) {
			iLDelta = (iLInputBuffer >> 4) & 0x0F;
		} else {
			iLInputBuffer   = *pLIn++;
			iLDelta         = iLInputBuffer & 0x0F;
			iLByteCount     = (iLByteCount + 1) & 0x03;
			if (iLByteCount == 0) {
				pLIn += 4;
			}
		}
		iLBufferStep = !iLBufferStep;

		// Step 2 - Find new index value (for later)
		iLIndex += iIndexTable[iLDelta];
		if (iLIndex < 0) {
			iLIndex = 0;
		} else if (iLIndex > 88) {
			iLIndex = 88;
		}

		// Step 3 - Separate sign and magnitude
		iLSign  = iLDelta & 8;
		iLDelta = iLDelta & 7;

		// Step 4 - Compute difference and new predicted value
		iLVPDiff = iLStep >> 3;
		if (iLDelta & 4) {
			iLVPDiff += iLStep;
		}
		if (iLDelta & 2) {
			iLVPDiff += iLStep >> 1;
		}
		if (iLDelta & 1) {
			iLVPDiff += iLStep >> 2;
		}

		if (iLSign) {
			iLValPred -= iLVPDiff;
		} else {
			iLValPred += iLVPDiff;
		}

		// Step 5 - clamp output value
		if (iLValPred > 32767) {
			iLValPred = 32767;
		} else if (iLValPred < -32768) {
			iLValPred = -32768;
		}

		// Step 6 - Update step value
		iLStep = iStepSizeTable[iLIndex];

		// Step 7 - Output value
		*pLOut = (INT16)iLValPred;
		pLOut += 2;

		// Right channel
		// Step 1 - get the Delta value
		if (iRBufferStep) {
			iRDelta = (iRInputBuffer >> 4) & 0x0F;
		} else {
			iRInputBuffer   = *pRIn++;
			iRDelta         = iRInputBuffer & 0x0F;
			iRByteCount     = (iRByteCount + 1) & 0x03;
			if (iRByteCount == 0) {
				pRIn += 4;
			}
		}
		iRBufferStep = !iRBufferStep;

		// Step 2 - Find new index value (for later)
		iRIndex += iIndexTable[iRDelta];
		if (iRIndex < 0) {
			iRIndex = 0;
		} else if (iRIndex > 88) {
			iRIndex = 88;
		}

		// Step 3 - Separate sign and magnitude
		iRSign  = iRDelta & 8;
		iRDelta = iRDelta & 7;

		// Step 4 - Compute difference and new predicted value
		iRVPDiff = iRStep >> 3;
		if (iRDelta & 4) {
			iRVPDiff += iRStep;
		}
		if (iRDelta & 2) {
			iRVPDiff += iRStep >> 1;
		}
		if (iRDelta & 1) {
			iRVPDiff += iRStep >> 2;
		}

		if (iRSign) {
			iRValPred -= iRVPDiff;
		} else {
			iRValPred += iRVPDiff;
		}

		// Step 5 - clamp output value
		if (iRValPred > 32767) {
			iRValPred = 32767;
		} else if (iRValPred < -32768) {
			iRValPred = -32768;
		}

		// Step 6 - Update step value
		iRStep = iStepSizeTable[iRIndex];

		// Step 7 - Output value
		*pROut = (INT16)iRValPred;
		pROut += 2;
	}

	// Save Value and Index for next decode
	adpcm_state->l_val_prev    = (INT16)iLValPred;
	adpcm_state->l_index      = (INT8)iLIndex;
	adpcm_state->r_val_prev    = (INT16)iRValPred;
	adpcm_state->r_index      = (INT8)iRIndex;

	return ((UINT32)pLOut - (UINT32)p_data_out);
}

/**
    Encode 16bits mono PCM data to IMA ADPCM packet.

    This function encode 16bits mono PCM data to IMA ADPCM packet.
    For the first packet, adpcm_state->l_index must be 0.

    The sample_count must >= 1 and <= packet sample count (please
    refer to ADPCM.h, the packet sample count is sampling rate
    relative, ex: ADPCM_PACKET_SAMPLES_8K)

    @param[in] p_data_in          Memory address of 16bits mono PCM data
    @param[in] p_data_out         Memory address of mono IMA ADPCM packet
    @param[in] sample_count    Sample count
    @param[in] adpcm_state           Pointer of previous value & index data
    @return The encoded packet length
*/
UINT32 audlib_adpcm_encode_packet_mono(INT16 *p_data_in, INT8 *p_data_out, UINT32 sample_count, PADPCM_STATE adpcm_state)
{
	UINT32  *puiTemp, uiOutLen;

	if (sample_count < 1) {
		printf("Invalid sample count\r\n");
		return 0;
	}

	adpcm_state->l_val_prev    = *p_data_in;

	puiTemp             = (UINT32 *)p_data_out;
	*puiTemp            = ((UINT32)(adpcm_state->l_index & 0xFF) << 16) | (adpcm_state->l_val_prev & 0xFFFF);

	uiOutLen = audlib_adpcm_encode_mono(p_data_in + 1, p_data_out + 4, sample_count - 1, adpcm_state);
	return (uiOutLen + 4);
}

/**
    Encode 16bits stereo PCM data to IMA ADPCM packet.

    This function encode 16bits stereo PCM data to IMA ADPCM packet.
    For the first packet, adpcm_state->l_index and adpcm_state->r_index must be 0.

    The sample_count must >= 1 and <= packet sample count (please
    refer to ADPCM.h, the packet sample count is sampling rate
    relative, ex: ADPCM_PACKET_SAMPLES_8K)

    @param[in] p_data_in          Memory address of 16bits stereo PCM data
    @param[in] p_data_out         Memory address of stereo IMA ADPCM packet
    @param[in] sample_count    Sample count
    @param[in] adpcm_state           Pointer of previous value & index data
    @return The encoded packet length
*/
UINT32 audlib_adpcm_encode_packet_stereo(INT16 *p_data_in, INT8 *p_data_out, UINT32 sample_count, PADPCM_STATE adpcm_state)
{
	UINT32  *puiTemp, uiOutLen;

	if (sample_count < 1) {
		printf("Invalid sample count\r\n");
		return 0;
	}

	adpcm_state->l_val_prev    = *p_data_in;
	adpcm_state->r_val_prev    = *(p_data_in + 1);

	puiTemp             = (UINT32 *)p_data_out;
	*puiTemp            = ((UINT32)(adpcm_state->l_index & 0xFF) << 16) | (adpcm_state->l_val_prev & 0xFFFF);
	*(puiTemp + 1)      = ((UINT32)(adpcm_state->r_index & 0xFF) << 16) | (adpcm_state->r_val_prev & 0xFFFF);

	uiOutLen = audlib_adpcm_encode_stereo(p_data_in + 2, p_data_out + 8, sample_count - 1, adpcm_state);
	return (uiOutLen + 8);
}

/**
    Decode mono IMA ADPCM packet to 16bits mono PCM data.

    This function decode mono IMA ADPCM packet to 16bits mono PCM data.
    For the full IMA ADPCM packet, the sample counts should be one of the
    following:
        - @b ADPCM_PACKET_SAMPLES_8K    (Sampling Rate = 8     KHz)
        - @b ADPCM_PACKET_SAMPLES_11K   (Sampling Rate = 11.025KHz)
        - @b ADPCM_PACKET_SAMPLES_22K   (Sampling Rate = 22.05 KHz)
        - @b ADPCM_PACKET_SAMPLES_44K   (Sampling Rate = 44.1  KHz)

    @param[in] p_data_in          Memory address of mono IMA ADPCM packet
    @param[in] p_data_out         Memory address of 16bits mono PCM data
    @param[in] sample_count    Total sample counts
    @return The PCM data length
*/
UINT32 audlib_adpcm_decode_packet_mono(INT8 *p_data_in, INT16 *p_data_out, UINT32 sample_count)
{
	ADPCM_STATE AdpcmState;
	UINT32      *puiTemp, uiOutLen;

	if (sample_count < 1) {
		printf("Invalid sample count\r\n");
		return 0;
	}

	puiTemp                 = (UINT32 *)p_data_in;
	AdpcmState.l_val_prev     = *puiTemp & 0xFFFF;
	AdpcmState.l_index       = (*puiTemp >> 16) & 0xFF;

	*p_data_out               = AdpcmState.l_val_prev;

	uiOutLen = audlib_adpcm_decode_mono(p_data_in + 4, p_data_out + 1, sample_count - 1, &AdpcmState);
	return (uiOutLen + 2);
}

/**
    Decode stereo IMA ADPCM packet to 16bits stereo PCM data.

    This function decode stereo IMA ADPCM packet to 16bits stereo PCM data.
    For the full IMA ADPCM packet, the sample counts should be one of the
    following:
        - @b ADPCM_PACKET_SAMPLES_8K    (Sampling Rate = 8     KHz)
        - @b ADPCM_PACKET_SAMPLES_11K   (Sampling Rate = 11.025KHz)
        - @b ADPCM_PACKET_SAMPLES_22K   (Sampling Rate = 22.05 KHz)
        - @b ADPCM_PACKET_SAMPLES_44K   (Sampling Rate = 44.1  KHz)

    @param[in] p_data_in          Memory address of stereo IMA ADPCM packet
    @param[in] p_data_out         Memory address of 16bits stereo PCM data
    @param[in] sample_count    Total sample counts
    @return The PCM data length
*/
UINT32 audlib_adpcm_decode_packet_stereo(INT8 *p_data_in, INT16 *p_data_out, UINT32 sample_count)
{
	ADPCM_STATE AdpcmState;
	UINT32      *puiTemp, uiOutLen;

	if (sample_count < 1) {
		printf("Invalid sample count\r\n");
		return 0;
	}

	puiTemp                 = (UINT32 *)p_data_in;
	AdpcmState.l_val_prev     = *puiTemp & 0xFFFF;
	AdpcmState.l_index       = (*puiTemp >> 16) & 0xFF;
	AdpcmState.r_val_prev     = *(puiTemp + 1) & 0xFFFF;
	AdpcmState.r_index       = (*(puiTemp + 1) >> 16) & 0xFF;

	*p_data_out               = AdpcmState.l_val_prev;
	*(p_data_out + 1)         = AdpcmState.r_val_prev;

	uiOutLen = audlib_adpcm_decode_stereo(p_data_in + 8, p_data_out + 2, sample_count - 1, &AdpcmState);
	return (uiOutLen + 4);
}

#ifdef __KERNEL__
EXPORT_SYMBOL(audlib_adpcm_encode_mono);
EXPORT_SYMBOL(audlib_adpcm_encode_stereo);
EXPORT_SYMBOL(audlib_adpcm_decode_mono);
EXPORT_SYMBOL(audlib_adpcm_decode_stereo);
EXPORT_SYMBOL(audlib_adpcm_encode_packet_mono);
EXPORT_SYMBOL(audlib_adpcm_encode_packet_stereo);
EXPORT_SYMBOL(audlib_adpcm_decode_packet_mono);
EXPORT_SYMBOL(audlib_adpcm_decode_packet_stereo);
#endif

//@}
