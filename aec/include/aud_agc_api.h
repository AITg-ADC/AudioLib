#ifndef _AUD_AGC_API_H_
#define _AUD_AGC_API_H_

typedef struct _ST_AUD_AGC_INFO {
    int s32FrameSize;     // Number of samples in a frame
    int s32ChannelNum;    // Number of channels
    int s32SamplingRate;  // Sampling rate of processed signal.
} ST_AUD_AGC_INFO, *PST_AUD_AGC_INFO;

typedef struct _ST_AUD_AGC_RTN {
    int u32InBufSize;        // Return Input buffer size
    int u32OutBufSize;       // Return Output buffer size
    int u32InternalBufSize;  // Return NS internal buffer size
} ST_AUD_AGC_RTN, *PST_AUD_AGC_RTN;

typedef enum _EN_AUD_AGC_PARAMS {
    EN_AUD_AGC_AGC,             //Automatic Gain Control state
    EN_AUD_AGC_AGC_LEVEL,       //Automatic Gain Control level (float), default: 8000
    EN_AUD_AGC_AGC_INCREMENT,   //maximal gain increase in dB/second (int32), default: 
    EN_AUD_AGC_AGC_DECREMENT,   //maximal gain decrease in dB/second (int32)
    EN_AUD_AGC_AGC_MAX_GAIN,    //maximal gain in dB (int32), default: 30
    EN_AUD_AGC_NOISE_SUPPRESS,  //Noise suppression level (defualt=-15)
    EN_AUD_AGC_DENOISE,         //denoiser state
    EN_AUD_AGC_PARAMS_TOTAL
} EN_AUD_AGC_PARAMS;

typedef enum _EN_AUD_AGC_ERR {
    EN_AUD_AGC_ENOERR,     // No error.
    EN_AUD_AGC_EINITFAIL,  // AUD_AEC_Init fail.
    EN_AUD_AGC_EINVALCMD,  // AUD_AEC_SetParam fail due to invalid command.
    EN_AUD_AGC_ERR_TOTAL
} EN_AUD_AGC_ERR;

void AUD_AGC_PreInit(PST_AUD_AGC_INFO pstNsInfo, PST_AUD_AGC_RTN pstNsRtn);
int AUD_AGC_Init(void *pInternalBuf, int u32BufSize);
void AUD_AGC_Run(short *ps16InBuf, short *ps16OutBuf);
int AUD_AGC_SetParam(EN_AUD_AGC_PARAMS enParamsCMD, void *pParamsValue);

#endif  //#ifndef _AUD_AGC_API_H_
