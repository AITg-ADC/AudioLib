#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
//#include <sys/types.h>
//#include <sys/stat.h>
//#include <fcntl.h>
#ifdef _MIPS_
#include <mips/mips32.h>
#include <math.h>
#endif

#include "aec.h"
#include "speex_echo.h"
#include "speex_preprocess.h"
//#include "perf.h"

// Release AACE Version
#define AEC_CODE_VERSION_INT 5
#define AEC_CODE_VERSION_FRA 2
#define AEC_CODE_VERSION ((AEC_CODE_VERSION_INT << 8) | AEC_CODE_VERSION_FRA)

/*-----------------------------------------------------------------------------*/
/* Extern Global Variables                                                     */
/*-----------------------------------------------------------------------------*/
extern u32 _Aec_GetInternalBufSize(PST_AUD_AEC_INFO pstAecInfo, u32 u32NumPreProc);
extern u32 _Aec_SetEchoParams(void *pstEchoState, EN_AUD_AEC_PARAMS enParamsCMD, void *pParamsValue);
extern u32 _Aec_SetPreProcParams(void **ppstPreProcState, EN_AUD_AEC_PARAMS enParamsCMD, void *pParamsValue);

extern void AUD_Malloc_Init(void *ptr, u32 u32TotalLen);
extern int AUD_Malloc_Uninit(void);
extern void *(*AUD_calloc)(s32 s32num, s32 s32size);

/*-----------------------------------------------------------------------------*/
/* Local Global Variables                                                      */
/*-----------------------------------------------------------------------------*/
static void *_pstEchoState;
static void **_ppstPreProcState;
static void **_ppstEchoState;

static ST_AUD_AEC_INFO _stAecInfo;

static s16 *_ps16AecOutBuf;
short _g_tmp_buf[2048];

/*-------------------------------------------------------------------------------
** Input        : pstAecInfo
** Output   : pstAecRtn
**--------------------------------------------------------------------------------*/
void AUD_AEC_PreInit(PST_AUD_AEC_INFO pstAecInfo, PST_AUD_AEC_RTN pstAecRtn)
{
    PST_AUD_AEC_INFO pstAecInfoSave = &_stAecInfo;
    u32 u32BytePerSample            = 2;

    memcpy(pstAecInfoSave, pstAecInfo, sizeof(ST_AUD_AEC_INFO));

    pstAecRtn->u32MicBufSize      = pstAecInfoSave->u32NumMic * pstAecInfoSave->u32FrameSize * u32BytePerSample;
    pstAecRtn->u32EchoBufSize     = pstAecInfoSave->u32NumSpeaker * pstAecInfoSave->u32FrameSize * u32BytePerSample;
    pstAecRtn->u32OutBufSize      = pstAecInfoSave->u32NumMic * pstAecInfoSave->u32FrameSize * u32BytePerSample;
    pstAecRtn->u32InternalBufSize = _Aec_GetInternalBufSize(pstAecInfoSave, pstAecInfoSave->u32NumMic);
#ifndef FIXED_POINT
    pstAecRtn->u32InternalBufSize *= 2;
#endif
    if (pstAecInfo->u32SpkrMixIn)
        pstAecRtn->u32EchoBufSize <<= 1;
}

/*-------------------------------------------------------------------------------
** Input        : pInternalBuf, u32BufSize
** Output   : EN_AUD_AEC_ERR
**--------------------------------------------------------------------------------*/
EN_AUD_AEC_ERR _AUD_AEC_Init(void *pInternalBuf, int u32BufSize, PST_AUD_AEC_PRELOAD pstAecPreload)
{
    PST_AUD_AEC_INFO pstAecInfo = &_stAecInfo;
    u32 u32FrameSize            = pstAecInfo->u32FrameSize;
    u32 u32FilterLen            = pstAecInfo->u32FilterLen;
    u32 u32NumMic               = pstAecInfo->u32NumMic;
    u32 u32NumSpeaker           = pstAecInfo->u32NumSpeaker;
    u32 u32SamplingRate         = pstAecInfo->u32SamplingRate;
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
        return EN_AUD_AEC_EINITFAIL;

    AUD_Malloc_Init(pInternalBuf, u32BufSize);
    _ppstPreProcState = (void **)AUD_calloc(u32NumMic, sizeof(void *));
    _ps16AecOutBuf    = (s16 *)AUD_calloc((u32NumMic * u32FrameSize), sizeof(s16));
    _ppstEchoState    = (void **)AUD_calloc(u32NumMic, sizeof(void *));

    if (_stAecInfo.u32SpkrDualMono) {
        for (i = 0; i < u32NumMic; i++) {
            _ppstEchoState[i] = speex_echo_state_init(u32FrameSize, u32FilterLen);
            speex_echo_ctl(_ppstEchoState[i], SPEEX_ECHO_SET_SAMPLING_RATE, &u32SamplingRate);
        }

        for (i = 0; i < u32NumMic; i++) {
            _ppstPreProcState[i] = speex_preprocess_state_init(u32FrameSize, u32SamplingRate);
            speex_preprocess_ctl(_ppstPreProcState[i], SPEEX_PREPROCESS_SET_ECHO_STATE, _ppstEchoState[i]);
        }

        // Noah@20220118, _ps16AecOutBuf -> _ppstEchoState
        _pstEchoState = _ppstEchoState[0];
        if (_ppstEchoState[i - 1] == 0 || _ppstPreProcState[i - 1] == 0) {
            return EN_AUD_AEC_EINITFAIL;
        }
    } else {
        _pstEchoState = _ppstEchoState[0] = speex_echo_state_init_mc(u32FrameSize, u32FilterLen, u32NumMic, u32NumSpeaker);
        speex_echo_ctl(_pstEchoState, SPEEX_ECHO_SET_SAMPLING_RATE, &u32SamplingRate);
        for (i = 0; i < u32NumMic; i++) {
            _ppstPreProcState[i] = speex_preprocess_state_init(u32FrameSize, u32SamplingRate);
            speex_preprocess_ctl(_ppstPreProcState[i], SPEEX_PREPROCESS_SET_ECHO_STATE, _pstEchoState);
        }
        if (_pstEchoState == 0 || _ppstPreProcState[i - 1] == 0)
            return EN_AUD_AEC_EINITFAIL;
    }

    return EN_AUD_AEC_ENOERR;
}
/*-------------------------------------------------------------------------------
** Input    : pu16MicBuf, pu16EchoBuf
** Output   : pu16OutBuf
**--------------------------------------------------------------------------------*/
void _AUD_AEC_Run(short *ps16MicBuf, short *ps16SpeakerBuf, short *ps16OutBuf, short s16DisNoiseSuppr, PST_AUD_AEC_PRELOAD pstAecPreload)
{
    s32 s32NumMic    = _stAecInfo.u32NumMic;
    s32 s32FrameSize = _stAecInfo.u32FrameSize;
    s32 i;

    if (_stAecInfo.u32SpkrMixIn) {
        s16 *s16Src = ps16SpeakerBuf, *s16Des = ps16SpeakerBuf;
        for (i = 0; i < (s32FrameSize >> 1); i++) {
            *s16Des = ((*s16Src) >> 1) + ((*(s16Src + 1)) >> 1);
            s16Src += 2;
            s16Des++;
            *s16Des = ((*s16Src) >> 1) + ((*(s16Src + 1)) >> 1);
            s16Src += 2;
            s16Des++;
        }
    }

    if ((_stAecInfo.u32SpkrDualMono) && (_stAecInfo.u32NumSpeaker == 2)) {
        memcpy(_g_tmp_buf, ps16MicBuf, s32FrameSize * 4);
        for (i = 0; i < (s32FrameSize); i++) {
            ps16MicBuf[i]                = _g_tmp_buf[i * 2];
            ps16MicBuf[s32FrameSize + i] = _g_tmp_buf[i * 2 + 1];
        }
        memcpy(_g_tmp_buf, ps16SpeakerBuf, s32FrameSize * 4);
        for (i = 0; i < (s32FrameSize); i++) {
            ps16SpeakerBuf[i]                = _g_tmp_buf[i * 2];
            ps16SpeakerBuf[s32FrameSize + i] = _g_tmp_buf[i * 2 + 1];
        }
        for (i = 0; i < 2; i++) {
            speex_echo_cancellation(_ppstEchoState[i], &ps16MicBuf[i * s32FrameSize], &ps16SpeakerBuf[i * s32FrameSize], &_ps16AecOutBuf[i * s32FrameSize]);
        }
        memcpy(_g_tmp_buf, _ps16AecOutBuf, s32FrameSize * 4);

        for (i = 0; i < (s32FrameSize); i++) {
            _ps16AecOutBuf[2 * i]     = _g_tmp_buf[i];
            _ps16AecOutBuf[i * 2 + 1] = _g_tmp_buf[s32FrameSize + i];
        }

    } else {
        speex_echo_cancellation(_pstEchoState, ps16MicBuf, ps16SpeakerBuf, _ps16AecOutBuf);
    }
    if (s16DisNoiseSuppr == 0) {
        for (i = 0; i < s32NumMic; i++)
            speex_preprocess_run(_ppstPreProcState[i], _ps16AecOutBuf);
    }

    memcpy(ps16OutBuf, _ps16AecOutBuf, (s32NumMic * s32FrameSize) << 1);
}

/*-------------------------------------------------------------------------------
** Input    : enParamsCMD,  u32ParamsValue
** Output   : EN_AUD_AEC_ERR
**--------------------------------------------------------------------------------*/
EN_AUD_AEC_ERR _AUD_AEC_SetParam(EN_AUD_AEC_PARAMS enParamsCMD, void *pParamsValue)
{
    u32 ret                     = 0;
    PST_AUD_AEC_INFO pstAecInfo = &_stAecInfo;
    u32 u32NumMic               = pstAecInfo->u32NumMic;
    int i;
    if ((s32)enParamsCMD < (s32)EN_AUD_AEC_ECHO_END) {
        if (_stAecInfo.u32SpkrDualMono) {
            for (i = 0; i < u32NumMic; i++) {
                ret = _Aec_SetEchoParams(_ppstEchoState[i], enParamsCMD, pParamsValue);
            }
        } else {
            ret = _Aec_SetEchoParams(_pstEchoState, enParamsCMD, pParamsValue);
        }
    } else
        ret = _Aec_SetPreProcParams(_ppstPreProcState, enParamsCMD, pParamsValue);

    if (ret != 0)
        return EN_AUD_AEC_EINVALCMD;
    return EN_AUD_AEC_ENOERR;
}

/*-------------------------------------------------------------------------------
** Input    :
** Output   : version
**--------------------------------------------------------------------------------*/
int AUD_AEC_GetVersion(void)
{
    return AEC_CODE_VERSION;
}

/*-------------------------------------------------------------------------------
** Input    :
** Output   : version
**--------------------------------------------------------------------------------*/
EN_AUD_AEC_ERR _AUD_AEC_Uninit(void)
{
    return AUD_Malloc_Uninit();
}