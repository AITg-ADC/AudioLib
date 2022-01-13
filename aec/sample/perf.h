#ifndef __PERF_H__
#define __PERF_H__

#define AUD_TEST_PERF_LEN 7

extern unsigned int _u32TotalCount[AUD_TEST_PERF_LEN], _u32StartCount[AUD_TEST_PERF_LEN];

#if defined(WIN32) || defined(_ARMV7_)

#define AUD_TEST_PERF_DEFINE_VARIABLES
#define AUD_TEST_PERF_RESET_COUNTER 
#define AUD_TEST_PERF_START_COUNTER(x) 
#define AUD_TEST_PERF_STOP_COUNTER(x)
#define AUD_TEST_PERF_RESULT(SamplingRateIndex, TotalSamples)

#else //WIN32
#define AUD_TEST_PERF_DEFINE_VARIABLES unsigned int _u32TotalCount[AUD_TEST_PERF_LEN], _u32StartCount[AUD_TEST_PERF_LEN];

#define AUD_TEST_PERF_RESET_COUNTER \
{                               \
    int i;                      \
    for(i=0; i<AUD_TEST_PERF_LEN; i++)           \
    _u32TotalCount[i] = 0;  \
}

//#if 1
#ifdef _ARMV7_
#define AUD_TEST_PERF_START_COUNTER(x) { u32 u32StartCounter;                       \
    _u32StartCount[x] = u32StartCounter; }

#define AUD_TEST_PERF_STOP_COUNTER(x)  { u32 u32StopCounter;                        \
    _u32TotalCount[x] += u32StopCounter - _u32StartCount[x]; }
#else
#define AUD_TEST_PERF_START_COUNTER(x) { u32 u32StartCounter;                       \
    asm volatile("mfc0 %0,$9 " : "=r" (u32StartCounter));                           \
    _u32StartCount[x] = u32StartCounter; }

#define AUD_TEST_PERF_STOP_COUNTER(x)  { u32 u32StopCounter;                        \
    asm volatile("mfc0 %0,$9 " : "=r" (u32StopCounter));                            \
    _u32TotalCount[x] += u32StopCounter - _u32StartCount[x]; }
#endif

#define AUD_TEST_PERF_RESULT(SamplingRate, TotalSamples)  {                         \
    double real_time;                                                           \
    int i;                                                                      \
    real_time = (double)TotalSamples / SamplingRate;                            \
    \
    printf("\n  PERFORMANCE INFORMATION\n");                                    \
    printf("  -----------------------------------\n");                          \
    printf("   samples processed: %d\n", TotalSamples);                         \
    printf("       Sampling Rate: %d seconds\n", SamplingRate);                 \
    printf("           real-time: %f seconds\n", real_time);                    \
    for(i=0; i < AUD_TEST_PERF_LEN; i++)                                        \
    printf("   cycle count: 0x%08x\n",_u32TotalCount[i]);                   \
    for(i=0; i < AUD_TEST_PERF_LEN; i++)                                        \
    printf("   CPU bandwidth[%d]: %3.2f MHz\n", i, ((double)((_u32TotalCount[i]/real_time)/1000000.0)*2));  \
}


#endif //WIN32

#endif // __PERF_H__
