
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
//#include <sys/types.h>
//#include <sys/stat.h>
//#include <fcntl.h>
#include <stdint.h>
#include "aud_aec_api.h"
#include "aud_ns_api.h"
#include "perf.h"
#include "comdef_nvt.h"

#ifdef _ARMV7_
#include <sys/time.h>
#include <unistd.h>
#endif

//#define WRITE_2CHAN_PCM

#define SAMPLING_RATE 8000

#if SAMPLING_RATE == 48000
#define NN 1024
#define TAIL 1024
#elif SAMPLING_RATE == 32000
#define NN 512
#define TAIL 1024
#else
#define NN 256
#define TAIL 512
#endif

#define NUM_MIC 1
#define NUM_SPEAKER 1

#define CH 1

/*----------------------------------*/
/* debug  */
/*----------------------------------*/

/*----------------------------------*/
/* static  */
/*----------------------------------*/
static int _u32FrameCount;
static int _u32SampleCount;
// static void _dynamicParamsTest(void);

int main_AEC(void);
int main_NS(void);

AUD_TEST_PERF_DEFINE_VARIABLES

#if defined(__GNUC__) && defined(__arm__)
struct timeval tv_start, tv_end;
// struct timezone tz;
float total_time, dtmp;
#endif

/*----------------------------------*/
/* main  */
/*----------------------------------*/
int main(int argc, char **argv)
{
#if defined(__GNUC__) && defined(__arm__)
    total_time = 0;
#endif
    main_AEC();
    main_NS();

    return 0;
}

int main_AEC(void)
{
    FILE *echo_fd, *ref_fd, *e_fd;
    FILE *FilterW_fd, *FilterR_fd;
    short echo_buf[NUM_SPEAKER * NN * 2], ref_buf[NUM_MIC * NN];  //, e_buf[NUM_MIC*NN];
    short *e_buf = ref_buf;
    int speakerSize = NUM_SPEAKER * NN;
    int micSize = NUM_MIC * NN;

    char mic_pcm[32] = "aec_mic.pcm";
    char speaker_pcm[32] = "aec_speaker.pcm";
    char out_pcm[64] = "aec_output.pcm";
    char FilterW_bin[64] = "preload.bin";

    printf("   mic pcm : %s\n", mic_pcm);
    printf("speaker pcm: %s\n", speaker_pcm);
    printf(" output pcm: %s\n\n", out_pcm);
    printf("sample rate: %d, num_mic: %d, num_speaker: %d, channel: %d\n",
           SAMPLING_RATE, NUM_MIC, NUM_SPEAKER, CH);

    ST_AUD_AEC_RTN stAecRtn;
    ST_AUD_AEC_INFO stAecInfoPre;
    ST_AUD_AEC_PRELOAD stAecPreload;
    void *pInternalBuf;
    int ps32Params[5];
    int s32RealCount = 0;

    memset(&stAecPreload, 0, sizeof(ST_AUD_AEC_PRELOAD));

    _u32FrameCount = 0;

    // Open file
    if ((echo_fd = fopen(speaker_pcm, "rb")) == NULL) {
        printf("Open %s failed\n", speaker_pcm);
        return 0;
    }
    if ((ref_fd = fopen(mic_pcm, "rb")) == NULL) {
        printf("Open %s failed\n", mic_pcm);
        return 0;
    }
    if ((e_fd = fopen(out_pcm, "wb")) == NULL) {
        printf("Open %s failed\n", out_pcm);
        return 0;
    }
    if ((FilterW_fd = fopen(FilterW_bin, "wb")) == NULL) {
        printf("Open %s failed\n", FilterW_bin);
        return 0;
    }
    AUD_TEST_PERF_RESET_COUNTER;

    // API
    stAecInfoPre.u32FilterLen = TAIL;
    stAecInfoPre.u32FrameSize = NN;
    stAecInfoPre.u32NumMic = NUM_MIC;
    stAecInfoPre.u32NumSpeaker = NUM_SPEAKER;
    stAecInfoPre.u32SamplingRate = SAMPLING_RATE;
    stAecInfoPre.u32SpkrMixIn = 0;
    stAecInfoPre.u32SpkrDualMono = 1;

    AUD_AEC_PreInit(&stAecInfoPre, &stAecRtn);
    pInternalBuf = malloc(stAecRtn.u32InternalBufSize);

    stAecPreload.u32PreloadEnable = 0;
    if (stAecPreload.u32PreloadEnable) {
        char FilterR_bin[64] = "preload_r.bin";
        if ((FilterR_fd = fopen(FilterR_bin, "rb")) == NULL) {
            printf("Open %s failed\n", FilterR_bin);
            return 0;
        }
        fread(&stAecPreload.u32ForegroundSize, sizeof(u32), 1, FilterR_fd);
        stAecPreload.ps16Foreground = malloc(stAecPreload.u32ForegroundSize);
        fread(stAecPreload.ps16Foreground, 1, stAecPreload.u32ForegroundSize, FilterR_fd);
        fread(&stAecPreload.u32BackgroundSize, sizeof(u32), 1, FilterR_fd);
        stAecPreload.ps32Background = malloc(stAecPreload.u32BackgroundSize);
        fread(stAecPreload.ps32Background, 1, stAecPreload.u32BackgroundSize, FilterR_fd);
        fclose(FilterR_fd);
        printf("----------Preload----------\n");
        printf("[Foreground] %p: 0x%x\n", stAecPreload.ps16Foreground, stAecPreload.u32ForegroundSize);
        printf("[Background] %p: 0x%x\n", stAecPreload.ps32Background, stAecPreload.u32BackgroundSize);
    }
    AUD_AEC_Init(pInternalBuf, stAecRtn.u32InternalBufSize, &stAecPreload);

    memset(e_buf, 0, micSize);
    ps32Params[0] = 1;  // 0:disable 1:enable   	//default=1
    AUD_AEC_SetParam(EN_AUD_AEC_ENABLE_AR, (void *)ps32Params);
    ps32Params[0] = 1;  //  for 1st channel	//1 or 2	//default = 1
    ps32Params[1] = 1;  //  for 2nd channel
    AUD_AEC_SetParam(EN_AUD_AEC_AMP_RATE, (void *)ps32Params);
    ps32Params[0] = (int)AEC_QCONST16(.9, 15);  // 0.75~0.9	//default=0.9
    AUD_AEC_SetParam(EN_AUD_AEC_PREEMPH, (void *)ps32Params);
    ps32Params[0] = (int)AEC_QCONST16(.992, 15);  // 0.9~0.992	//default=0.992
    AUD_AEC_SetParam(EN_AUD_AEC_NOTCH_RADIUS, (void *)ps32Params);
    ps32Params[0] = (int)AEC_QCONST16(0.99, 15);  // 0.1~0.5		//default=0.25
    AUD_AEC_SetParam(EN_AUD_AEC_LEAK_ESTIMATE, (void *)ps32Params);
    ps32Params[0] = 1;  // 1:disable 0:enable   	//default=0
    AUD_AEC_SetParam(EN_AUD_AEC_DISABLE_LEAK_ESTIMTAE, (void *)ps32Params);
    ps32Params[0] = 0;  // 1:disable 0:enable   	//default=0
    AUD_AEC_SetParam(EN_AUD_AEC_DISABLE_DC_FILTER, (void *)ps32Params);
    ps32Params[0] = 0;  // 1:disable 0:enable   	//default=0
    AUD_AEC_SetParam(EN_AUD_AEC_DISABLE_ECHO_SMOOTH, (void *)ps32Params);

    ps32Params[0] = 0;  // default = 1 (linear)
    AUD_AEC_SetParam(EN_AUD_AEC_BANK_SCALE, (void *)ps32Params);
    ps32Params[0] = -20;  // default = -15dB
    AUD_AEC_SetParam(EN_AUD_AEC_NOISE_SUPPRESS, (void *)ps32Params);
    ps32Params[0] = -50;  // default = -40dB
    AUD_AEC_SetParam(EN_AUD_AEC_ECHO_SUPPRESS, (void *)ps32Params);
    ps32Params[0] = -50;  // default = -15dB
    AUD_AEC_SetParam(EN_AUD_AEC_ECHO_SUPPRESS_ACTIVE, (void *)ps32Params);
    ps32Params[0] = (int)AEC_QCONST16(0.8f, 15);  // 0.6~0.8		//default=0.8
    AUD_AEC_SetParam(EN_AUD_AEC_ECHO_NOISE_RATIO, (void *)ps32Params);
    ps32Params[0] = 1;  // 0:disable 1:enable  //default=1
    AUD_AEC_SetParam(EN_AUD_AEC_NLP_ENABLE, (void *)ps32Params);

    if (stAecInfoPre.u32SpkrMixIn)
        speakerSize <<= 1;

    while (!feof(ref_fd) && !feof(echo_fd)) {
        _u32FrameCount++;

        fread(ref_buf, sizeof(short), micSize, ref_fd);
        fread(echo_buf, sizeof(short), speakerSize, echo_fd);

        // if(_u32FrameCount < 2000)
        //	continue;

        AUD_TEST_PERF_START_COUNTER(0);
        /* Test dynamic params */
        //_dynamicParamsTest();
#ifdef _ARMV7_
        // gettimeofday (&tv_start, &tz);      //start time evaluation
        gettimeofday(&tv_start, NULL);  // start time evaluation
#endif

        /* Run */
        AUD_AEC_Run(ref_buf, echo_buf, e_buf, 0, &stAecPreload);

#ifdef _ARMV7_
        // gettimeofday (&tv_end, &tz);        //end of time evaluation
        gettimeofday(&tv_end, NULL);  // end of time evaluation
        total_time = total_time + (tv_end.tv_sec - tv_start.tv_sec) + (tv_end.tv_usec - tv_start.tv_usec) * 0.000001;
#endif

        s32RealCount++;

        AUD_TEST_PERF_STOP_COUNTER(0);

        fwrite(e_buf, sizeof(short), micSize, e_fd);

        printf("[frames]%d\r", _u32FrameCount);

        if (s32RealCount == 500) {
            AUD_TEST_PERF_RESULT(SAMPLING_RATE, NN * s32RealCount);
            AUD_TEST_PERF_RESET_COUNTER;
            s32RealCount = 0;
        }

        // if(_u32FrameCount >= 2600)
        //	break;
    }

    _u32SampleCount = NN * s32RealCount;
    AUD_TEST_PERF_RESULT(SAMPLING_RATE, _u32SampleCount);
#ifdef _ARMV7_
    printf("decode time = %f seconds\n", total_time);
    total_time = 0;
#endif
    fwrite(&stAecPreload.u32ForegroundSize, sizeof(u32), 1, FilterW_fd);
    fwrite(stAecPreload.ps16Foreground, 1, stAecPreload.u32ForegroundSize, FilterW_fd);
    fwrite(&stAecPreload.u32BackgroundSize, sizeof(u32), 1, FilterW_fd);
    fwrite(stAecPreload.ps32Background, 1, stAecPreload.u32BackgroundSize, FilterW_fd);

    fclose(e_fd);
    fclose(echo_fd);
    fclose(ref_fd);
    fclose(FilterW_fd);

    return 0;
}

int main_NS(void)
{
    FILE *in_fd, *out_fd;
    short in_buf[CH * NN];
    short extra_buf[CH * NN];
    short *out_buf = extra_buf;  // in_buf;
    int Size = CH * NN;
#if SAMPLING_RATE == 8000
#if CH == 2
    char in_pcm[32] = "output2_8k.pcm";
    char out_pcm[32] = "ns_out_8k.pcm";
#else
    char in_pcm[32] = "aec_mic.pcm";
    char out_pcm[32] = "ns_output.pcm";
#endif
#else
    char in_pcm[32] = "output_test.pcm";
    char out_pcm[32] = "ns_output_test.pcm";
#endif

    ST_AUD_NS_RTN stNsRtn;
    ST_AUD_NS_INFO stNsInfoPre;
    void *pInternalBuf;
    int ps32Params[5];
    int s32RealCount = 0;

    _u32FrameCount = 0;
    memset(in_buf, 0, Size * sizeof(short));
    memset(extra_buf, 0, Size * sizeof(short));

    // Open file
    if ((in_fd = fopen(in_pcm, "rb")) == NULL) {
        printf("Open %s failed\n", in_pcm);
        return 0;
    }
    if ((out_fd = fopen(out_pcm, "wb")) == NULL) {
        printf("Open %s failed\n", out_pcm);
        return 0;
    }
    AUD_TEST_PERF_RESET_COUNTER;

    // API
    stNsInfoPre.s32FrameSize = NN;
    stNsInfoPre.s32ChannelNum = CH;
    stNsInfoPre.s32SamplingRate = SAMPLING_RATE;
    AUD_NS_PreInit(&stNsInfoPre, &stNsRtn);
    pInternalBuf = malloc(stNsRtn.u32InternalBufSize);

    AUD_NS_Init(pInternalBuf, stNsRtn.u32InternalBufSize);

    ps32Params[0] = 0;  // default = 1 (linear)
    AUD_NS_SetParam(EN_AUD_NS_BANK_SCALE, (void *)ps32Params);
    ps32Params[0] = -40;  // default = -40dB
    AUD_NS_SetParam(EN_AUD_NS_NOISE_SUPPRESS, (void *)ps32Params);

    while (!feof(in_fd) && !feof(out_fd)) {
        _u32FrameCount++;

        fread(in_buf, sizeof(short), Size, in_fd);
        // if(_u32FrameCount < 100)
        //	continue;

        AUD_TEST_PERF_START_COUNTER(0);
        /* Test dynamic params */
        //_dynamicParamsTest();
#ifdef _ARMV7_
        // gettimeofday (&tv_start, &tz);      //start time evaluation
        gettimeofday(&tv_start, NULL);  // start time evaluation
#endif
        /* Run */
        AUD_NS_Run(in_buf, out_buf);
#ifdef _ARMV7_
        // gettimeofday (&tv_end, &tz);        //end of time evaluation
        gettimeofday(&tv_end, NULL);  // end of time evaluation
        total_time = total_time + (tv_end.tv_sec - tv_start.tv_sec) + (tv_end.tv_usec - tv_start.tv_usec) * 0.000001;
#endif

        s32RealCount++;

        AUD_TEST_PERF_STOP_COUNTER(0);

        fwrite(out_buf, sizeof(short), Size, out_fd);

        printf("[frames]%d\r", _u32FrameCount);

        /*if(s32RealCount == 500)
        {
            AUD_TEST_PERF_RESULT(SAMPLING_RATE, NN*s32RealCount);
            AUD_TEST_PERF_RESET_COUNTER;
            s32RealCount = 0;
        }*/

        // if(_u32FrameCount >= 1000)
        //	break;
    }

#ifdef _ARMV7_
    printf("decode time = %f seconds\n", total_time);
#endif

    _u32SampleCount = NN * s32RealCount;
    AUD_TEST_PERF_RESULT(SAMPLING_RATE, _u32SampleCount);

    fclose(in_fd);
    fclose(out_fd);

    return 0;
}

#if 0
static void _dynamicParamsTest(void)
{
//	int s32ParamsValue, ps32AmpRate[NUM_MIC];

	if(_u32FrameCount == 1000)
	{
  		int s32ParamsValue = -30;
		AUD_AEC_SetParam(EN_AUD_AEC_NOISE_SUPPRESS, &s32ParamsValue);
	}
	else if(_u32FrameCount == 2000)	  	
	{
  		int s32ParamsValue = -15;
		AUD_AEC_SetParam(EN_AUD_AEC_NOISE_SUPPRESS, &s32ParamsValue);
	}
}
#endif
