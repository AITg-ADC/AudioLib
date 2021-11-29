// ConsoleApplication1.cpp : Defines the entry point for the console application.
//
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "audlib_adpcm.h"


#define test_file_size 0x15FC00
ADPCM_STATE            adpcm_state;


static UINT32 uiAudAecPlyBufAdr,uiAudAecRecBufAdr,uiAudAecOutBufAdr;

int testcode_read_file(CHAR* file_name, CHAR* addr)
{
	FILE *fp;
	char *buf;


	// load dtb to memory
	fp = fopen(file_name, "rb");
	if (fp == NULL) {
		printf("failed to open %s\n", file_name);
		return -1;
	}
	fseek(fp, 0, SEEK_SET);
	buf = addr;

	fread(buf, test_file_size, 1, fp);
	fclose(fp);

    return test_file_size;
}

void testcode_write_file(CHAR* file_name, CHAR* addr,UINT32 file_size)
{
	FILE *fp;
	char *buf;
	buf = addr;

	fp = fopen(file_name, "wb");
	if (fp == NULL) {
		printf("failed to open %s\n", file_name);
		return;
	}

	printf("write addr =  %x\n", buf);

	fwrite(buf, file_size, 1, fp);
	fclose(fp);
	return;
}

void _run_adpcm_test(void)
{

    int                     file_size;
    CHAR                    adpcm_input[30]    = "/mnt/sd/adpcm_in.PCM";
    CHAR                    adpcm_output[30]   = "/mnt/sd/adpcm_out.PCM";
    INT16                   *buf_in,*buf_out;
    INT8                   *buf_bsout;
    UINT32                  pcm_sample_count = ADPCM_PACKET_SAMPLES_8K,pcm_sample_count_total;
    UINT32                  i=0;
    PADPCM_STATE            p_adpcm_state = &adpcm_state;


    printf("In run adpcm test\r\n");
    p_adpcm_state->l_index   = 0;
    p_adpcm_state->l_val_prev = 0;
    p_adpcm_state->r_index   = 0;
    p_adpcm_state->r_val_prev = 0;

	uiAudAecRecBufAdr = (UINT32)malloc(test_file_size * 5);
	uiAudAecPlyBufAdr = uiAudAecRecBufAdr + test_file_size;
	uiAudAecOutBufAdr = uiAudAecRecBufAdr + test_file_size*2;	



    file_size = testcode_read_file(adpcm_input, (CHAR *) uiAudAecRecBufAdr);

    pcm_sample_count_total = (file_size>>1);

    printf("adpcm pattern size = [%d] sample count = [%d] \r\n",file_size,pcm_sample_count_total);

    buf_in   = (INT16 *)uiAudAecRecBufAdr;
    buf_out  = (INT16 *)uiAudAecOutBufAdr;
    buf_bsout= (INT8 *)uiAudAecPlyBufAdr;

/*
adpcm encode
*/
    do{
        if(pcm_sample_count_total - i < pcm_sample_count)
            pcm_sample_count = pcm_sample_count_total - i;
        audlib_adpcm_encode_packet_mono(buf_in+i, buf_bsout+i, pcm_sample_count, p_adpcm_state);
        i += pcm_sample_count;
    }while(i<pcm_sample_count_total);




    printf("adpcm encode done\r\n");


/*
adpcm decode
*/

    pcm_sample_count = ADPCM_PACKET_SAMPLES_8K;
    i = 0;
    do{
        if(pcm_sample_count_total - i < pcm_sample_count)
            pcm_sample_count = pcm_sample_count_total - i;
        audlib_adpcm_decode_packet_mono(buf_bsout+i, buf_out+i, pcm_sample_count);
        i += pcm_sample_count;
    }while(i<pcm_sample_count_total);

    printf("adpcm decode done\r\n");

    testcode_write_file(adpcm_output, (CHAR *) buf_out, file_size);


    printf("Out run_adpcm_test \r\n");


}


int main(void)
{	
	printf("Hello, in g711 user space test program ~\r\n");

	_run_adpcm_test();
	
	return 0;
}