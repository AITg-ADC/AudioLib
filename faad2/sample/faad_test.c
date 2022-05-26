#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "faad.h"

#ifndef BYTE
typedef unsigned char BYTE;
#endif

#define STEREO_AUDIO 0

#define FRAME_MAX_LEN 1024 * 5
#define BUFFER_MAX_LEN 1024 * 1024

int get_one_ADTS_frame(unsigned char *buffer, size_t buf_size, unsigned char *data, size_t *data_size)
{
    size_t size = 0;

    if (!buffer || !data || !data_size)
    {
        return -1;
    }

    while (1)
    {
        if (buf_size < 7)
        {
            return -1;
        }

        if ((buffer[0] == 0xff) && ((buffer[1] & 0xf0) == 0xf0))
        {
            size |= ((buffer[3] & 0x03) << 11); // high 2 bit
            size |= buffer[4] << 3;             // middle 8 bit
            size |= ((buffer[5] & 0xe0) >> 5);  // low 3bit
            break;
        }
        --buf_size;
        ++buffer;
    }

    if (buf_size < size)
    {
        return -1;
    }

    memcpy(data, buffer, size);
    *data_size = size;

    return 0;
}

int main(int argc, char *argv[])
{
    NeAACDecConfigurationPtr conf;
    static unsigned char frame[FRAME_MAX_LEN] = {0};
    static unsigned char buffer[BUFFER_MAX_LEN] = {0};

    FILE *ifile = NULL;
    FILE *ofile = NULL;

    unsigned long samplerate = 0;
    unsigned char channels = 0;
    NeAACDecHandle decoder = 0;

    size_t data_size = 0;
    size_t size = 0;

    NeAACDecFrameInfo frame_info;
    unsigned char *input_data = buffer;
    unsigned char *pcm_data = NULL, pcm_mono_data[2048];

    ifile = fopen("faad_in_8khz_16bit.aac", "rb");
    ofile = fopen("faad_out.PCM", "wb");
    if (!ifile || !ofile)
    {
        printf("source or destination file open failed");
        return -1;
    }

    data_size = fread(buffer, 1, BUFFER_MAX_LEN, ifile);

    // open decoder
    decoder = NeAACDecOpen();
    if (get_one_ADTS_frame(buffer, data_size, frame, &size) < 0)
    {
        return -1;
    }

    conf = NeAACDecGetCurrentConfiguration(decoder);
    conf->defObjectType = LC;
    conf->defSampleRate = 8000;          // real samplerate/2
    conf->outputFormat = FAAD_FMT_16BIT; //
    conf->dontUpSampleImplicitSBR = 1;
    NeAACDecSetConfiguration(decoder, conf);

    // initialize decoder
    NeAACDecInit(decoder, frame, size, &samplerate, &channels);

    while (get_one_ADTS_frame(input_data, data_size, frame, &size) == 0)
    {
        // decode ADTS frame
        pcm_data = (unsigned char *)NeAACDecDecode(decoder, &frame_info, frame, size);

        if (frame_info.error > 0)
        {
            printf("%s\n", NeAACDecGetErrorMessage(frame_info.error));
        }
        else if (pcm_data && frame_info.samples > 0)
        {
            printf("frame info: bytesconsumed %d, channels %d, header_type %d\
                object_type %d, samples %d, samplerate %d\n",
                   frame_info.bytesconsumed,
                   frame_info.channels, frame_info.header_type,
                   frame_info.object_type, frame_info.samples,
                   frame_info.samplerate);

#if STEREO_AUDIO
            fwrite(pcm_data, 1, frame_info.samples * frame_info.channels, ofile); // 2個通道
            fflush(ofile);
#else
            if (frame_info.channels == 2)
            {
                int i, j;
                for (i = 0, j = 0; i < 4096 && j < 2048; i += 4, j += 2)
                {
                    pcm_mono_data[j] = ((char *)pcm_data)[i];
                    pcm_mono_data[j + 1] = ((char *)pcm_data)[i + 1];
                }

                frame_info.channels = 1;
                fwrite(pcm_mono_data, 1, frame_info.samples * frame_info.channels, ofile); // 2個通道
                fflush(ofile);
            }
#endif
        }
        data_size -= size;
        input_data += size;
    }

    NeAACDecClose(decoder);

    fclose(ifile);
    fclose(ofile);

    return 0;
}
