// ConsoleApplication1.cpp : Defines the entry point for the console application.
//
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "audlib_g711.h"


#define test_file_size 0x15FC00


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

void _run_g711_test(void)
{

    int                     file_size;
    CHAR                    g711_input[20]         = "/mnt/sd/g711_in.PCM";
    CHAR                    g711_output_alaw[30]   = "/mnt/sd/g711_out_alaw.PCM";
    CHAR                    g711_output_ulaw[30]   = "/mnt/sd/g711_out_ulaw.PCM";
    CHAR                    g711_alaw_bitstream[25]= "/mnt/sd/g711_bs_alaw.PCM";
    CHAR                    g711_ulaw_bitstream[25]= "/mnt/sd/g711_bs_ulaw.PCM";
    INT16                   *buf_in,*buf_out;
    UINT8                   *buf_bsout;
    UINT32                  pcm_sample_count;


    printf("In run_g711_test \r\n");


	uiAudAecRecBufAdr = (UINT32)malloc(test_file_size * 5);
	uiAudAecPlyBufAdr = uiAudAecRecBufAdr + test_file_size;
	uiAudAecOutBufAdr = uiAudAecRecBufAdr + test_file_size*2;

    file_size = testcode_read_file(g711_input, (CHAR *) uiAudAecRecBufAdr);

    pcm_sample_count = (file_size>>1);

    buf_in   = (INT16 *)uiAudAecRecBufAdr;
    buf_out  = (INT16 *)uiAudAecOutBufAdr;
    buf_bsout= (UINT8 *)uiAudAecPlyBufAdr;

    printf("g711 pattern sample = [%d] \r\n",pcm_sample_count);
	printf("g711 buf_in    addr = [%x] \r\n",buf_in);
	printf("g711 buf_out   addr = [%x] \r\n",buf_out);
	printf("g711 buf_bsout addr = [%x] \r\n",buf_bsout);

/*
alaw encode
*/

    g711_alaw_encode(buf_in, buf_bsout, pcm_sample_count, 0);

    testcode_write_file(g711_alaw_bitstream,    (CHAR *) buf_bsout, file_size>>1);

    printf("a-law encode done\r\n");
/*
alaw decode
*/

    g711_alaw_decode(buf_bsout, buf_out, pcm_sample_count, 0, 0);

    testcode_write_file(g711_output_alaw, (CHAR *) buf_out, file_size);

    memset((void *)buf_out, 0x00, file_size);
    memset((void *)buf_bsout, 0x00, file_size>>1);

    printf("a-law decode done\r\n");



/*
ulaw encode
*/
    file_size = testcode_read_file(g711_input, (CHAR *) uiAudAecRecBufAdr);
    pcm_sample_count = (file_size>>1);

    g711_ulaw_encode(buf_in, buf_bsout, pcm_sample_count, 0);

    testcode_write_file(g711_ulaw_bitstream,    (CHAR *) buf_bsout, file_size>>1);

    printf("u-law encode done\r\n");
/*
ulaw decode
*/

    g711_ulaw_decode(buf_bsout, buf_out, pcm_sample_count, 0, 0);

    testcode_write_file(g711_output_ulaw, (CHAR *) buf_out, file_size);

    memset((void *)buf_out, 0x00, file_size);
    memset((void *)buf_bsout, 0x00, file_size>>1);

    printf("u-law decode done\r\n");


}


int main(void)
{	
	printf("Hello, in g711 user space test program ~\r\n");

	_run_g711_test();
	
	return 0;
}