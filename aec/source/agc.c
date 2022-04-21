#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
//#include <sys/types.h>
//#include <sys/stat.h>
//#include <fcntl.h>
#ifdef _MIPS_
#include <mips/mips32.h>
#include <math.h>
#endif

#include "agc.h"
#include "speex_echo.h"
#include "speex_preprocess.h"

/*-----------------------------------------------------------------------------*/
/* Extern Global Variables                                                     */
/*-----------------------------------------------------------------------------*/
extern u32 _Agc_GetInternalBufSize(PST_AUD_AGC_INFO pstAgcInfo, u32 u32NumPreProc);
extern u32 _Agc_SetPreProcParams(void **ppstPreProcState, EN_AUD_AGC_PARAMS enParamsCMD, void *pParamsValue, s32 s32ChannelNum);

extern void AUD_Malloc_Init(void *ptr, u32 u32TotalLen);
extern void *(*AUD_calloc)(s32 s32num, s32 s32size);

/*-----------------------------------------------------------------------------*/
/* Local Global Variables                                                      */
/*-----------------------------------------------------------------------------*/
static void **_ppstPreProcState;
static ST_AUD_AGC_INFO _stAgcInfo;

/*-------------------------------------------------------------------------------
** Input    : pstAgcInfo
** Output   : pstAgcRtn
**--------------------------------------------------------------------------------*/
void AUD_AGC_PreInit(PST_AUD_AGC_INFO pstAgcInfo, PST_AUD_AGC_RTN pstAgcRtn)
{
    PST_AUD_AGC_INFO pstAgcInfoSave = &_stAgcInfo;
    u32 u32BytePerSample            = 2;

    memcpy(pstAgcInfoSave, pstAgcInfo, sizeof(ST_AUD_AGC_INFO));

    pstAgcRtn->u32InBufSize       = pstAgcInfoSave->s32ChannelNum * pstAgcInfoSave->s32FrameSize * u32BytePerSample;
    pstAgcRtn->u32OutBufSize      = pstAgcRtn->u32InBufSize;
    pstAgcRtn->u32InternalBufSize = _Agc_GetInternalBufSize(pstAgcInfoSave, pstAgcInfoSave->s32ChannelNum);
#ifndef FIXED_POINT
    pstAgcRtn->u32InternalBufSize *= 2;
#endif
}

/*-------------------------------------------------------------------------------
** Input    : pInternalBuf, u32BufSize
** Output   : EN_AUD_AGC_ERR
**--------------------------------------------------------------------------------*/
EN_AUD_AGC_ERR _AUD_AGC_Init(void *pInternalBuf, int u32BufSize)
{
    PST_AUD_AGC_INFO pstAgcInfo = &_stAgcInfo;
    u32 u32FrameSize           = (u32)pstAgcInfo->s32FrameSize;
    u32 u32ChannelNum          = (u32)pstAgcInfo->s32ChannelNum;
    u32 u32SamplingRate        = (u32)pstAgcInfo->s32SamplingRate;
    u32 i;

#ifdef _MIPS_
    {
        int temp;
        asm volatile("mfc0 %0,$12"
                     : "=r"(temp));
        (volatile int)temp |= (volatile int)(1 << 24);
        asm volatile("mtc0 %0,$12"
                     :
                     : "r"(temp));
    }
#endif /*_MIPS_*/

    if (pInternalBuf == 0)
        return EN_AUD_AGC_EINITFAIL;

    AUD_Malloc_Init(pInternalBuf, u32BufSize * 2);
    _ppstPreProcState = (void **)AUD_calloc(u32ChannelNum, sizeof(void *));

    for (i = 0; i < u32ChannelNum; i++) {
        _ppstPreProcState[i] = speex_preprocess_state_init(u32FrameSize, u32SamplingRate);
        if (!_ppstPreProcState[i])
            return EN_AUD_AGC_EINITFAIL;
    }

    return EN_AUD_AGC_ENOERR;
}

/*-------------------------------------------------------------------------------
** Input        : ps16InBuf
** Output   : pu16OutBuf
**--------------------------------------------------------------------------------*/
void _AUD_AGC_Run(short *ps16InBuf, short *ps16OutBuf)
{
    s32 s32NumCH = _stAgcInfo.s32ChannelNum;
    s32 i;

    for (i = 0; i < s32NumCH; i++)
        speex_preprocess_run(_ppstPreProcState[i], ps16InBuf);

    if (((uintptr_t)ps16InBuf) != ((uintptr_t)ps16OutBuf))
        memcpy(ps16OutBuf, ps16InBuf, s32NumCH * _stAgcInfo.s32FrameSize * sizeof(short));
}

/*-------------------------------------------------------------------------------
** Input    : enParamsCMD,  u32ParamsValue
** Output   : EN_AUD_AGC_ERR
**--------------------------------------------------------------------------------*/
EN_AUD_AGC_ERR _AUD_AGC_SetParam(EN_AUD_AGC_PARAMS enParamsCMD, void *pParamsValue)
{
    s32 s32NumCH = _stAgcInfo.s32ChannelNum;
    u32 ret;

    ret = _Agc_SetPreProcParams(_ppstPreProcState, enParamsCMD, pParamsValue, s32NumCH);

    if (ret != 0)
        return EN_AUD_AGC_EINVALCMD;
    return EN_AUD_AGC_ENOERR;
}

/*-------------------------------------------------------------------------------
** Input        :
** Output   : version
**--------------------------------------------------------------------------------*/
/*int AUD_NS_GetVersion(void)
{
    return NS_CODE_VERSION;
}*/
