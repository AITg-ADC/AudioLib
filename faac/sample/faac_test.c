#include <stdio.h>
#include <stdlib.h>
#include "faac.h"

#ifndef BYTE
typedef unsigned char BYTE;
#endif

int main(int argc, char *argv[])
{
    unsigned long nSampleRate     = 8000;
    unsigned int nChannels        = 1;
    unsigned int nPCMBitSize      = 16;
    unsigned long nInputSamples   = 1024;
    unsigned long nMaxOutputBytes = 0;
    size_t nRet                   = 0;
    faacEncHandle hEncoder        = {0};

    FILE *fpIn  = NULL;
    FILE *fpOut = NULL;

    fpIn  = fopen("faac_in.PCM", "rb");
    fpOut = fopen("faac_out_8khz_16bit.aac", "wb");

    if (fpIn == NULL || fpOut == NULL) {
        printf("open file fail!\n");
        return -1;
    }

    hEncoder = faacEncOpen(nSampleRate, nChannels, &nInputSamples, &nMaxOutputBytes);
    if (hEncoder == NULL) {
        printf("faacEncOpen failed!\n");
        return -1;
    }

    int nPCMBufferSize = nInputSamples * nPCMBitSize / 8;
    BYTE *pbPCMBuffer  = (BYTE *)malloc(nPCMBufferSize);
    BYTE *pbAACBuffer  = (BYTE *)malloc(nMaxOutputBytes);

    faacEncConfigurationPtr pConfiguration = {0};

    pConfiguration = faacEncGetCurrentConfiguration(hEncoder);

    printf("pConfiguration->bitRate:%d, pConfiguration->bandWidth:%d\n", pConfiguration->bitRate, pConfiguration->bandWidth);
    /*
    PCM Sample Input Format
    0   FAAC_INPUT_NULL         invalid, signifies a misconfigured config
    1   FAAC_INPUT_16BIT        native endian 16bit
    2   FAAC_INPUT_24BIT        native endian 24bit in 24 bits      (not implemented)
    3   FAAC_INPUT_32BIT        native endian 24bit in 32 bits      (DEFAULT)
    4   FAAC_INPUT_FLOAT        32bit floating point
    */
    pConfiguration->inputFormat = FAAC_INPUT_16BIT;

    //#define MAIN 1
    //#define LOW  2
    //#define SSR  3
    //#define LTP  4
    pConfiguration->aacObjectType = LOW;
    pConfiguration->bitRate       = 48000;
    pConfiguration->bandWidth     = 32000;
    pConfiguration->allowMidside  = 1;
    pConfiguration->useLfe        = 0;
    pConfiguration->useTns        = 0;
    pConfiguration->quantqual     = 100;

    // outputformat 0 = Raw; 1 = ADTS
    pConfiguration->outputFormat = 1;
    pConfiguration->shortctl     = SHORTCTL_NORMAL;

    nRet = faacEncSetConfiguration(hEncoder, pConfiguration);
    if (nRet < 0) {
        printf("----------[ERROR] Failed to call faacEncSetConfiguration()\n");
    }

    while ((nRet = fread(pbPCMBuffer, 1, nPCMBufferSize, fpIn)) > 0) {
        nInputSamples = nRet / (nPCMBitSize / 8);
        nRet          = faacEncEncode(hEncoder, (int *)pbPCMBuffer, nInputSamples, pbAACBuffer, nMaxOutputBytes);

        fwrite(pbAACBuffer, 1, nRet, fpOut);
    }

    faacEncClose(hEncoder);
    fclose(fpOut);
    fclose(fpIn);

    free(pbAACBuffer);
    free(pbPCMBuffer);

    printf("faac complete aac encoder finish!!\n");
    return 0;
}
