#ifndef _AEC_H_
#define _AEC_H_

#include "aud_aec_api.h"
#include "comdef_nvt.h" 

EN_AUD_AEC_ERR _AUD_AEC_Init(void *pInternalBuf, int u32BufSize, PST_AUD_AEC_PRELOAD pstAecPreload);
EN_AUD_AEC_ERR _AUD_AEC_Uninit(void);
void _AUD_AEC_Run(short *ps16MicBuf, short *ps16SpeakerBuf, short *ps16OutBuf, short s16DisNoiseSuppr, PST_AUD_AEC_PRELOAD pstAecPreload);
EN_AUD_AEC_ERR _AUD_AEC_SetParam(EN_AUD_AEC_PARAMS enParamsCMD, void *pParamsValue);

#endif
