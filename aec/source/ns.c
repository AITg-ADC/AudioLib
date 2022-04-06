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

#include "ns.h"
//#include "perf.h"


//Release NS Version
//#define NS_CODE_VERSION_INT 1
//#define NS_CODE_VERSION_FRA 0
//#define NS_CODE_VERSION ((NS_CODE_VERSION_INT<<8)|NS_CODE_VERSION_FRA)


/*-----------------------------------------------------------------------------*/
/* Extern Global Variables                                                     */
/*-----------------------------------------------------------------------------*/
extern u32 _Ns_GetInternalBufSize(PST_AUD_NS_INFO pstNsInfo, u32 u32NumPreProc);
extern u32 _Ns_SetPreProcParams(void **ppstPreProcState, EN_AUD_NS_PARAMS enParamsCMD, void *pParamsValue, s32 s32ChannelNum);

extern void AUD_Malloc_Init(void *ptr, u32 u32TotalLen);
extern void *(*AUD_calloc)(s32 s32num, s32 s32size);

extern void * speex_preprocess_state_init(int frame_size, int sampling_rate, void *pstEchoState);
#ifndef ADD_NLP
extern int speex_preprocess_run(void *state, s16 *x, int num_chan, int chan_index);
#else
extern int speex_preprocess_run(void *state, s16 *near, s16 *far, s16 *x, int num_chan, int chan_index);
#endif
extern void set_kiss_fft_stride(int frame_size);


/*-----------------------------------------------------------------------------*/
/* Local Global Variables                                                      */
/*-----------------------------------------------------------------------------*/
static void **_ppstPreProcState;
static ST_AUD_NS_INFO _stNsInfo;


/*-------------------------------------------------------------------------------
** Input        : pstNsInfo
** Output   : pstNsRtn
**--------------------------------------------------------------------------------*/
void AUD_NS_PreInit(PST_AUD_NS_INFO pstNsInfo, PST_AUD_NS_RTN pstNsRtn)
{
    PST_AUD_NS_INFO pstNsInfoSave = &_stNsInfo; 
    u32 u32BytePerSample = 2;

    memcpy(pstNsInfoSave,pstNsInfo,sizeof(ST_AUD_NS_INFO));

    pstNsRtn->u32InBufSize = pstNsInfoSave->s32ChannelNum * pstNsInfoSave->s32FrameSize * u32BytePerSample;
    pstNsRtn->u32OutBufSize = pstNsRtn->u32InBufSize;
    pstNsRtn->u32InternalBufSize = _Ns_GetInternalBufSize(pstNsInfoSave, pstNsInfoSave->s32ChannelNum);
}


/*-------------------------------------------------------------------------------
** Input        : pInternalBuf, u32BufSize
** Output   : EN_AUD_NS_ERR
**--------------------------------------------------------------------------------*/
EN_AUD_NS_ERR _AUD_NS_Init(void *pInternalBuf, int u32BufSize)
{
    PST_AUD_NS_INFO pstNsInfo = &_stNsInfo; 
    u32 u32FrameSize = (u32)pstNsInfo->s32FrameSize;
    u32 u32ChannelNum = (u32)pstNsInfo->s32ChannelNum;
    u32 u32SamplingRate = (u32)pstNsInfo->s32SamplingRate;
    u32 i;
    
#ifdef _MIPS_
    {
        int temp;
        asm volatile("mfc0 %0,$12"
            : "=r" (temp));
        (volatile int)temp |= (volatile int)(1<<24);
        asm volatile("mtc0 %0,$12"
            : : "r" (temp));
    }
#endif /*_MIPS_*/

    if(pInternalBuf == 0)
        return EN_AUD_NS_EINITFAIL;
    
    AUD_Malloc_Init(pInternalBuf, u32BufSize);
    _ppstPreProcState = (void**)AUD_calloc(u32ChannelNum,sizeof(void *));

    set_kiss_fft_stride(u32FrameSize);

    for(i=0; i<u32ChannelNum; i++)
    {
        _ppstPreProcState[i] = speex_preprocess_state_init(u32FrameSize, u32SamplingRate, 0);       
        if(!_ppstPreProcState[i])
            return EN_AUD_NS_EINITFAIL;
    }
    
    return EN_AUD_NS_ENOERR;
}

/*-------------------------------------------------------------------------------
** Input        : ps16InBuf
** Output   : pu16OutBuf
**--------------------------------------------------------------------------------*/
void _AUD_NS_Run(short *ps16InBuf, short *ps16OutBuf)
{
    s32 s32NumCH = _stNsInfo.s32ChannelNum;
    s32 i;

    for(i=0; i<s32NumCH; i++)
#ifndef ADD_NLP
        speex_preprocess_run(_ppstPreProcState[i], ps16InBuf, s32NumCH, i);
#else
        speex_preprocess_run(_ppstPreProcState[i], 0, 0, ps16InBuf, s32NumCH, i);
#endif
    if(((uintptr_t)ps16InBuf) != ((uintptr_t)ps16OutBuf))
        memcpy(ps16OutBuf, ps16InBuf, s32NumCH*_stNsInfo.s32FrameSize*sizeof(short));

}

/*-------------------------------------------------------------------------------
** Input        : enParamsCMD,  u32ParamsValue
** Output   : EN_AUD_NS_ERR
**--------------------------------------------------------------------------------*/
EN_AUD_NS_ERR _AUD_NS_SetParam(EN_AUD_NS_PARAMS enParamsCMD, void *pParamsValue)
{
    s32 s32NumCH = _stNsInfo.s32ChannelNum;
    u32 ret;

    ret = _Ns_SetPreProcParams(_ppstPreProcState, enParamsCMD, pParamsValue, s32NumCH);

    if(ret != 0)
        return EN_AUD_NS_EINVALCMD;
    return EN_AUD_NS_ENOERR;
}


/*-------------------------------------------------------------------------------
** Input        : 
** Output   : version
**--------------------------------------------------------------------------------*/
/*int AUD_NS_GetVersion(void)
{
    return NS_CODE_VERSION;
}*/

