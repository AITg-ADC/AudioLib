
/*-----------------------------------------------------------------------------*/
/* Include Header Files                                                        */
/*-----------------------------------------------------------------------------*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
//#include <sys/types.h>
//#include <sys/stat.h>
//#include <fcntl.h>

#include "aud_aec_api.h"

/*-----------------------------------------------------------------------------*/
/* Definition                                                      */
/*-----------------------------------------------------------------------------*/

#define TRUE 1
#define FALSE 0

EN_AUD_AEC_ERR _AUD_AEC_Init(void *pInternalBuf, int u32BufSize, PST_AUD_AEC_PRELOAD pstAecPreload);
EN_AUD_AEC_ERR _AUD_AEC_Uninit(void);
void _AUD_AEC_Run(short *ps16MicBuf, short *ps16SpeakerBuf, short *ps16OutBuf, short s16DisNoiseSuppr, PST_AUD_AEC_PRELOAD pstAecPreload);
EN_AUD_AEC_ERR _AUD_AEC_SetParam(EN_AUD_AEC_PARAMS enParamsCMD, void *pParamsValue);

/*-----------------------------------------------------------------------------*/
/* Interface Functions                                                         */
/*-----------------------------------------------------------------------------*/

/*-------------------------------------------------------------------------------
** Input    : pInternalBuf, u32BufSize
** Output   : err
**--------------------------------------------------------------------------------*/
int AUD_AEC_Init(void *pInternalBuf, int u32BufSize, PST_AUD_AEC_PRELOAD pstAecPreload)
{
    EN_AUD_AEC_ERR err;
    err = _AUD_AEC_Init(pInternalBuf, u32BufSize, pstAecPreload);
    if (err != EN_AUD_AEC_ENOERR) {
        printf("AUD_AEC_Init fail(%d)...\n", err);
        return FALSE;
    } else
        return TRUE;
}

/*-------------------------------------------------------------------------------
** Input    : pu16MicBuf, pu16EchoBuf
** Output   : pu16OutBuf
**--------------------------------------------------------------------------------*/
void AUD_AEC_Run(short *ps16MicBuf, short *ps16SpeakerBuf, short *ps16OutBuf, short s16DisNoiseSuppr, PST_AUD_AEC_PRELOAD pstAecPreload)
{
    _AUD_AEC_Run(ps16MicBuf, ps16SpeakerBuf, ps16OutBuf, s16DisNoiseSuppr, pstAecPreload);
}

/*-------------------------------------------------------------------------------
** Input    : enParamsCMD,  u32ParamsValue
** Output   : err
**--------------------------------------------------------------------------------*/
int AUD_AEC_SetParam(EN_AUD_AEC_PARAMS enParamsCMD, void *pParamsValue)
{
    int err;
    err = _AUD_AEC_SetParam(enParamsCMD, pParamsValue);
    if (err != EN_AUD_AEC_ENOERR)
        return FALSE;
    else
        return TRUE;
}

/*-------------------------------------------------------------------------------
** Output   : err
**--------------------------------------------------------------------------------*/
int AUD_AEC_Uninit(void)
{
    return _AUD_AEC_Uninit();
}