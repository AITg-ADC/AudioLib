#ifndef _AGC_H_
#define _AGC_H_

#include "aud_agc_api.h"
#include "comdef_nvt.h" 

EN_AUD_AGC_ERR _AUD_AGC_Init(void *pInternalBuf, int u32BufSize);
EN_AUD_AGC_ERR _AUD_AGC_Uninit(void);
void _AUD_AGC_Run(short *ps16InBuf, short *ps16OutBuf);
EN_AUD_AGC_ERR _AUD_AGC_SetParam(EN_AUD_AGC_PARAMS enParamsCMD, void *pParamsValue);

#endif
