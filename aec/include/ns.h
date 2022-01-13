#ifndef _NS_H_
#define _NS_H_

#include "aud_ns_api.h"
#include "comdef_nvt.h" 

EN_AUD_NS_ERR _AUD_NS_Init(void *pInternalBuf, int u32BufSize);
void _AUD_NS_Run(short *ps16InBuf, short *ps16OutBuf);
EN_AUD_NS_ERR _AUD_NS_SetParam(EN_AUD_NS_PARAMS enParamsCMD, void *pParamsValue);

#endif
