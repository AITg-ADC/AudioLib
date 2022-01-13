
/*-----------------------------------------------------------------------------*/
/* Include Header Files                                                        */
/*-----------------------------------------------------------------------------*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
//#include <sys/types.h>
//#include <sys/stat.h>
//#include <fcntl.h>

#include "aud_ns_api.h"

/*-----------------------------------------------------------------------------*/
/* Definition                                                      */
/*-----------------------------------------------------------------------------*/

#define TRUE    1
#define FALSE   0


EN_AUD_NS_ERR _AUD_NS_Init(void *pInternalBuf, int u32BufSize);
void _AUD_NS_Run(short *ps16InBuf, short *ps16OutBuf);
EN_AUD_NS_ERR _AUD_NS_SetParam(EN_AUD_NS_PARAMS enParamsCMD, void *pParamsValue);


/*-----------------------------------------------------------------------------*/
/* Interface Functions                                                         */
/*-----------------------------------------------------------------------------*/

/*-------------------------------------------------------------------------------
** Input        : pInternalBuf, u32BufSize
** Output   : err
**--------------------------------------------------------------------------------*/
int AUD_NS_Init(void *pInternalBuf, int u32BufSize)
{
    EN_AUD_NS_ERR err;
    err = _AUD_NS_Init(pInternalBuf, u32BufSize);
    if(err != EN_AUD_NS_ENOERR)
        return FALSE;
    else
        return TRUE;
}

/*-------------------------------------------------------------------------------
** Input        : ps16InBuf
** Output   : ps16OutBuf
**--------------------------------------------------------------------------------*/
void AUD_NS_Run(short *ps16InBuf, short *ps16OutBuf)
{
    _AUD_NS_Run(ps16InBuf, ps16OutBuf);
}

/*-------------------------------------------------------------------------------
** Input        : enParamsCMD,  pParamsValue
** Output   : err
**--------------------------------------------------------------------------------*/
int AUD_NS_SetParam(EN_AUD_NS_PARAMS enParamsCMD, void *pParamsValue)
{
    int err;
    err = _AUD_NS_SetParam(enParamsCMD, pParamsValue);
    if(err != EN_AUD_NS_ENOERR)
        return FALSE;
    else
        return TRUE;
}



