/*
Copyright (c) 2003-2004, Mark Borgerding

All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
* Neither the author nor the names of any contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "os_support.h"
#include "kiss_fftr.h"
#include "_kiss_fft_guts.h"
#include "basic_op.h"
#include "ntk_basic_operation.h"
#include "ntk_opt_switch.h"


struct kiss_fftr_state{
    kiss_fft_cfg substate;
    kiss_fft_cpx * tmpbuf;
    kiss_fft_cpx * super_twiddles;
#ifdef USE_SIMD    
    long pad;
#endif    
};


#ifdef _MIPS_kiss_fftr_alloc_OPT
kiss_fftr_cfg kiss_fftr_alloc(int nfft,int inverse_fft,void * mem,size_t * lenmem)
{
    int i;
    kiss_fftr_cfg st = NULL;
    size_t subsize, memneeded;

    if (nfft & 1) {
        speex_warning("Real FFT optimization must be even.\n");
        return NULL;
    }
    nfft >>= 1;

    kiss_fft_alloc (nfft, inverse_fft, NULL, &subsize);
    memneeded = sizeof(struct kiss_fftr_state) + subsize + (nfft << 3);

    if (lenmem == NULL) {
        st = (kiss_fftr_cfg) KISS_FFT_MALLOC (memneeded);
    } else {
        if (*lenmem >= memneeded)
            st = (kiss_fftr_cfg) mem;
        *lenmem = memneeded;
    }
    if (!st)
        return NULL;

    st->substate = (kiss_fft_cfg) (st + 1); /*just beyond kiss_fftr_state struct */
    st->tmpbuf = (kiss_fft_cpx *) (((char *) st->substate) + subsize);
    st->super_twiddles = st->tmpbuf + nfft;
    kiss_fft_alloc(nfft, inverse_fft, st->substate, &subsize);

#ifdef FIXED_POINT
    {
        spx_word32_t init = (nfft>>1);
        spx_word32_t phase = init;
        spx_word16_t delta = 1;
        if(!inverse_fft)
        {
            phase = -init;
            delta = -1;
        }
        for (i=0;i<nfft;++i) {
            kf_cexp2(st->super_twiddles+i, DIV32(NTK_SHL32(phase,16),nfft));
            phase += delta;
        }
    }
#else
    for (i=0;i<nfft;++i) {
        const double pi=3.14159265358979323846264338327;
        double phase = pi*(((double)i) /nfft + .5);
        if (!inverse_fft)
            phase = -phase;
        kf_cexp(st->super_twiddles+i, phase );
    }
#endif
    return st;
}
#else
kiss_fftr_cfg kiss_fftr_alloc(int nfft,int inverse_fft,void * mem,size_t * lenmem)
{
    int i;
    kiss_fftr_cfg st = NULL;
    size_t subsize, memneeded;

    if (nfft & 1) {
        speex_warning("Real FFT optimization must be even.\n");
        return NULL;
    }
    nfft >>= 1;

    kiss_fft_alloc (nfft, inverse_fft, NULL, &subsize);
    memneeded = sizeof(struct kiss_fftr_state) + subsize + sizeof(kiss_fft_cpx) * ( nfft * 2);

    if (lenmem == NULL) {
        st = (kiss_fftr_cfg) KISS_FFT_MALLOC (memneeded);
    } else {
        if (*lenmem >= memneeded)
            st = (kiss_fftr_cfg) mem;
        *lenmem = memneeded;
    }
    if (!st)
        return NULL;

    st->substate = (kiss_fft_cfg) (st + 1); /*just beyond kiss_fftr_state struct */
    st->tmpbuf = (kiss_fft_cpx *) (((char *) st->substate) + subsize);
    st->super_twiddles = st->tmpbuf + nfft;
    kiss_fft_alloc(nfft, inverse_fft, st->substate, &subsize);

#ifdef FIXED_POINT
    for (i=0;i<nfft;++i) {
        spx_word32_t phase = i+(nfft>>1);
        if (!inverse_fft)
            phase = -phase;
        kf_cexp2(st->super_twiddles+i, DIV32(NTK_SHL32(phase,16),nfft));
    }
#else
    for (i=0;i<nfft;++i) {
        const double pi=3.14159265358979323846264338327;
        double phase = pi*(((double)i) /nfft + .5);
        if (!inverse_fft)
            phase = -phase;
        kf_cexp(st->super_twiddles+i, phase );
    }
#endif
    return st;
}
#endif

#if 0
void kiss_fftr(kiss_fftr_cfg st,const kiss_fft_scalar *timedata,kiss_fft_cpx *freqdata)
{
    /* input buffer timedata is stored row-wise */
    int k,ncfft;
    kiss_fft_cpx fpnk,fpk,f1k,f2k,tw,tdc;

    if ( st->substate->inverse) {
        speex_fatal("kiss fft usage error: improper alloc\n");
    }

    ncfft = st->substate->nfft;

    /*perform the parallel fft of two real signals packed in real,imag*/
    kiss_fft( st->substate , (const kiss_fft_cpx*)timedata, st->tmpbuf );
    /* The real part of the DC element of the frequency spectrum in st->tmpbuf
    * contains the sum of the even-numbered elements of the input time sequence
    * The imag part is the sum of the odd-numbered elements
    *
    * The sum of tdc.r and tdc.i is the sum of the input time sequence. 
    *      yielding DC of input time sequence
    * The difference of tdc.r - tdc.i is the sum of the input (dot product) [1,-1,1,-1... 
    *      yielding Nyquist bin of input time sequence
    */

    tdc.r = st->tmpbuf[0].r;
    tdc.i = st->tmpbuf[0].i;
    C_FIXDIV(tdc,2);
    CHECK_OVERFLOW_OP(tdc.r ,+, tdc.i);
    CHECK_OVERFLOW_OP(tdc.r ,-, tdc.i);
    freqdata[0].r = tdc.r + tdc.i;
    freqdata[ncfft].r = tdc.r - tdc.i;
#ifdef USE_SIMD    
    freqdata[ncfft].i = freqdata[0].i = _mm_set1_ps(0);
#else
    freqdata[ncfft].i = freqdata[0].i = 0;
#endif

    for ( k=1;k <= ncfft/2 ; ++k ) {
        fpk    = st->tmpbuf[k]; 
        fpnk.r =   st->tmpbuf[ncfft-k].r;
        fpnk.i = - st->tmpbuf[ncfft-k].i;
        C_FIXDIV(fpk,2);
        C_FIXDIV(fpnk,2);

        C_ADD( f1k, fpk , fpnk );
        C_SUB( f2k, fpk , fpnk );
        C_MUL( tw , f2k , st->super_twiddles[k]);

        freqdata[k].r = HALF_OF(f1k.r + tw.r);
        freqdata[k].i = HALF_OF(f1k.i + tw.i);
        freqdata[ncfft-k].r = HALF_OF(f1k.r - tw.r);
        freqdata[ncfft-k].i = HALF_OF(tw.i - f1k.i);
    }
}

void kiss_fftri(kiss_fftr_cfg st,const kiss_fft_cpx *freqdata, kiss_fft_scalar *timedata)
{
    /* input buffer timedata is stored row-wise */
    int k, ncfft;

    if (st->substate->inverse == 0) {
        speex_fatal("kiss fft usage error: improper alloc\n");
    }

    ncfft = st->substate->nfft;

    st->tmpbuf[0].r = freqdata[0].r + freqdata[ncfft].r;
    st->tmpbuf[0].i = freqdata[0].r - freqdata[ncfft].r;
    /*C_FIXDIV(st->tmpbuf[0],2);*/

    for (k = 1; k <= ncfft / 2; ++k) {
        kiss_fft_cpx fk, fnkc, fek, fok, tmp;
        fk = freqdata[k];
        fnkc.r = freqdata[ncfft - k].r;
        fnkc.i = -freqdata[ncfft - k].i;
        /*C_FIXDIV( fk , 2 );
        C_FIXDIV( fnkc , 2 );*/

        C_ADD (fek, fk, fnkc);
        C_SUB (tmp, fk, fnkc);
        C_MUL (fok, tmp, st->super_twiddles[k]);
        C_ADD (st->tmpbuf[k],     fek, fok);
        C_SUB (st->tmpbuf[ncfft - k], fek, fok);
#ifdef USE_SIMD        
        st->tmpbuf[ncfft - k].i *= _mm_set1_ps(-1.0);
#else
        st->tmpbuf[ncfft - k].i *= -1;
#endif
    }
    kiss_fft (st->substate, st->tmpbuf, (kiss_fft_cpx *) timedata);
}
#endif
#ifdef _MIPS_kiss_fftr2_OPT
void kiss_fftr2(kiss_fftr_cfg st,const kiss_fft_scalar *timedata,kiss_fft_scalar *freqdata)
{
    /* input buffer timedata is stored row-wise */
    int k, ncfft=st->substate->nfft, tmp;
    kiss_fft_cpx f2k,tdc;
    spx_word32_t f1kr, f1ki, twr, twi;
    kiss_fft_cpx *tmpbuf = st->tmpbuf;
    kiss_fft_cpx *tmpbuf2 = st->tmpbuf + ncfft;
    kiss_fft_cpx *super_twiddles = st->super_twiddles;
    kiss_fft_scalar *freqdata_tmp = freqdata + 1;
    kiss_fft_scalar *freqdata_tmp2 = freqdata + ((ncfft<<1)-2);


    if ( st->substate->inverse) {
        speex_fatal("kiss fft usage error: improper alloc\n");
    }

    /*perform the parallel fft of two real signals packed in real,imag*/
    kiss_fft( st->substate , (const kiss_fft_cpx*)timedata, st->tmpbuf );
    /* The real part of the DC element of the frequency spectrum in st->tmpbuf
    * contains the sum of the even-numbered elements of the input time sequence
    * The imag part is the sum of the odd-numbered elements
    *
    * The sum of tdc.r and tdc.i is the sum of the input time sequence. 
    *      yielding DC of input time sequence
    * The difference of tdc.r - tdc.i is the sum of the input (dot product) [1,-1,1,-1... 
    *      yielding Nyquist bin of input time sequence
    */


    tdc.r = tmpbuf->r;
    tdc.i = tmpbuf->i;
    //C_FIXDIV(tdc,2);  
    tdc.r = (short)NTK_PSHR16((int)tdc.r,1);
    tdc.i = (short)NTK_PSHR16((int)tdc.i,1);
    CHECK_OVERFLOW_OP(tdc.r ,+, tdc.i);
    CHECK_OVERFLOW_OP(tdc.r ,-, tdc.i);
    freqdata[0] = ADD16(tdc.r , tdc.i);
    freqdata[(ncfft<<1)-1] = SUB16(tdc.r , tdc.i);

    tmp = ncfft>>2;
    for ( k=1;k <= tmp ; ++k )
    {
        tmpbuf++;
        tmpbuf2--;
        super_twiddles++;
        f2k.r = SHR32((EXTEND32(tmpbuf->r) - EXTEND32(tmpbuf2->r)),1);
        f2k.i = NTK_PSHR32((EXTEND32(tmpbuf->i) + EXTEND32(tmpbuf2->i)),1);
        f1kr = NTK_SHL32((EXTEND32(tmpbuf->r) + EXTEND32(tmpbuf2->r)),13);
        f1ki = NTK_SHL32((EXTEND32(tmpbuf->i) - EXTEND32(tmpbuf2->i)),13);

        twr = SHR32(SUB32(MULT16_16(f2k.r,super_twiddles->r),MULT16_16(f2k.i,super_twiddles->i)), 1);
        twi = SHR32(ADD32(MULT16_16(f2k.i,super_twiddles->r),MULT16_16(f2k.r,super_twiddles->i)), 1); 

        MIPS_PSHR32_15(*freqdata_tmp, f1kr + twr);  freqdata_tmp++;
        MIPS_PSHR32_15(*freqdata_tmp, f1ki + twi);  freqdata_tmp++;
        MIPS_PSHR32_15(*freqdata_tmp2, twi - f1ki); freqdata_tmp2--;
        MIPS_PSHR32_15(*freqdata_tmp2, f1kr - twr); freqdata_tmp2--;  

        /*
        *freqdata_tmp = NTK_PSHR32(f1kr + twr, 15); freqdata_tmp++;
        *freqdata_tmp = NTK_PSHR32(f1ki + twi, 15); freqdata_tmp++;
        *freqdata_tmp2 = NTK_PSHR32(twi - f1ki, 15);    freqdata_tmp2--;
        *freqdata_tmp2 = NTK_PSHR32(f1kr - twr, 15);    freqdata_tmp2--;  
        */

        tmpbuf++;
        tmpbuf2--;
        super_twiddles++;
        f2k.r = SHR32((EXTEND32(tmpbuf->r) - EXTEND32(tmpbuf2->r)),1);
        f2k.i = NTK_PSHR32((EXTEND32(tmpbuf->i) + EXTEND32(tmpbuf2->i)),1);
        f1kr = NTK_SHL32((EXTEND32(tmpbuf->r) + EXTEND32(tmpbuf2->r)),13);
        f1ki = NTK_SHL32((EXTEND32(tmpbuf->i) - EXTEND32(tmpbuf2->i)),13);
#if 0
        MULT_AC0(f2k.r,super_twiddles->r);
        MSUB_AC0(f2k.i,super_twiddles->i);
        SHILO1_AC0();
        twr = MFLO_AC0();

        MULT_AC0(f2k.i,super_twiddles->r);
        MADD_AC0(f2k.r,super_twiddles->i);
        SHILO1_AC0();
        twi = MFLO_AC0(); 
#endif    
        twr = SHR32(SUB32(MULT16_16(f2k.r,super_twiddles->r),MULT16_16(f2k.i,super_twiddles->i)), 1);
        twi = SHR32(ADD32(MULT16_16(f2k.i,super_twiddles->r),MULT16_16(f2k.r,super_twiddles->i)), 1);

        MIPS_PSHR32_15(*freqdata_tmp, f1kr + twr);  freqdata_tmp++;
        MIPS_PSHR32_15(*freqdata_tmp, f1ki + twi);  freqdata_tmp++;
        MIPS_PSHR32_15(*freqdata_tmp2, twi - f1ki); freqdata_tmp2--;
        MIPS_PSHR32_15(*freqdata_tmp2, f1kr - twr); freqdata_tmp2--;  

        /*      
        *freqdata_tmp = NTK_PSHR32(f1kr + twr, 15); freqdata_tmp++;
        *freqdata_tmp = NTK_PSHR32(f1ki + twi, 15); freqdata_tmp++;
        *freqdata_tmp2 = NTK_PSHR32(twi - f1ki, 15);    freqdata_tmp2--;
        *freqdata_tmp2 = NTK_PSHR32(f1kr - twr, 15);    freqdata_tmp2--;
        */    

    }

}
#else
void kiss_fftr2(kiss_fftr_cfg st,const kiss_fft_scalar *timedata,kiss_fft_scalar *freqdata)
{
    /* input buffer timedata is stored row-wise */
    int k,ncfft;
    kiss_fft_cpx f2k,tdc;
    spx_word32_t f1kr, f1ki, twr, twi;

    if ( st->substate->inverse) {
        speex_fatal("kiss fft usage error: improper alloc\n");
    }

    ncfft = st->substate->nfft;

    /*perform the parallel fft of two real signals packed in real,imag*/
    kiss_fft( st->substate , (const kiss_fft_cpx*)timedata, st->tmpbuf );
    /* The real part of the DC element of the frequency spectrum in st->tmpbuf
    * contains the sum of the even-numbered elements of the input time sequence
    * The imag part is the sum of the odd-numbered elements
    *
    * The sum of tdc.r and tdc.i is the sum of the input time sequence. 
    *      yielding DC of input time sequence
    * The difference of tdc.r - tdc.i is the sum of the input (dot product) [1,-1,1,-1... 
    *      yielding Nyquist bin of input time sequence
    */

    tdc.r = st->tmpbuf[0].r;
    tdc.i = st->tmpbuf[0].i;
    //C_FIXDIV(tdc,2);
    tdc.r = (short)NTK_PSHR16((int)tdc.r,1);
    tdc.i = (short)NTK_PSHR16((int)tdc.i,1);
    CHECK_OVERFLOW_OP(tdc.r ,+, tdc.i);
    CHECK_OVERFLOW_OP(tdc.r ,-, tdc.i);
    freqdata[0] = ADD16(tdc.r , tdc.i);
    freqdata[2*ncfft-1] = SUB16(tdc.r , tdc.i);

    for ( k=1;k <= ncfft/2 ; ++k )
    {
        /*fpk    = st->tmpbuf[k]; 
        fpnk.r =   st->tmpbuf[ncfft-k].r;
        fpnk.i = - st->tmpbuf[ncfft-k].i;
        C_FIXDIV(fpk,2);
        C_FIXDIV(fpnk,2);

        C_ADD( f1k, fpk , fpnk );
        C_SUB( f2k, fpk , fpnk );

        C_MUL( tw , f2k , st->super_twiddles[k]);

        freqdata[2*k-1] = HALF_OF(f1k.r + tw.r);
        freqdata[2*k] = HALF_OF(f1k.i + tw.i);
        freqdata[2*(ncfft-k)-1] = HALF_OF(f1k.r - tw.r);
        freqdata[2*(ncfft-k)] = HALF_OF(tw.i - f1k.i);
        */

        /*f1k.r = NTK_PSHR32(ADD32(EXTEND32(st->tmpbuf[k].r), EXTEND32(st->tmpbuf[ncfft-k].r)),1);
        f1k.i = NTK_PSHR32(SUB32(EXTEND32(st->tmpbuf[k].i), EXTEND32(st->tmpbuf[ncfft-k].i)),1);
        f2k.r = NTK_PSHR32(SUB32(EXTEND32(st->tmpbuf[k].r), EXTEND32(st->tmpbuf[ncfft-k].r)),1);
        f2k.i = SHR32(ADD32(EXTEND32(st->tmpbuf[k].i), EXTEND32(st->tmpbuf[ncfft-k].i)),1);

        C_MUL( tw , f2k , st->super_twiddles[k]);

        freqdata[2*k-1] = HALF_OF(f1k.r + tw.r);
        freqdata[2*k] = HALF_OF(f1k.i + tw.i);
        freqdata[2*(ncfft-k)-1] = HALF_OF(f1k.r - tw.r);
        freqdata[2*(ncfft-k)] = HALF_OF(tw.i - f1k.i);
        */
        f2k.r = SHR32((EXTEND32(st->tmpbuf[k].r) - EXTEND32(st->tmpbuf[ncfft-k].r)),1);
        f2k.i = NTK_PSHR32((EXTEND32(st->tmpbuf[k].i) + EXTEND32(st->tmpbuf[ncfft-k].i)),1);

        f1kr = NTK_SHL32((EXTEND32(st->tmpbuf[k].r) + EXTEND32(st->tmpbuf[ncfft-k].r)),13);
        f1ki = NTK_SHL32((EXTEND32(st->tmpbuf[k].i) - EXTEND32(st->tmpbuf[ncfft-k].i)),13);

        twr = SHR32(SUB32(MULT16_16(f2k.r,st->super_twiddles[k].r),MULT16_16(f2k.i,st->super_twiddles[k].i)), 1);
        twi = SHR32(ADD32(MULT16_16(f2k.i,st->super_twiddles[k].r),MULT16_16(f2k.r,st->super_twiddles[k].i)), 1);

#ifdef FIXED_POINT
        freqdata[2*k-1] = NTK_EXTRACT16(NTK_PSHR32(f1kr + twr, 15));
        freqdata[2*k] = NTK_EXTRACT16(NTK_PSHR32(f1ki + twi, 15));
        freqdata[2*(ncfft-k)-1] = NTK_EXTRACT16(NTK_PSHR32(f1kr - twr, 15));
        freqdata[2*(ncfft-k)] = NTK_EXTRACT16(NTK_PSHR32(twi - f1ki, 15));
#else
        freqdata[2*k-1] = .5f*(f1kr + twr);
        freqdata[2*k] = .5f*(f1ki + twi);
        freqdata[2*(ncfft-k)-1] = .5f*(f1kr - twr);
        freqdata[2*(ncfft-k)] = .5f*(twi - f1ki);

#endif
    }
}
#endif

#ifdef  _MIPS_kiss_fftri2_OPT
void kiss_fftri2(kiss_fftr_cfg st, kiss_fft_scalar *freqdata, kiss_fft_scalar *timedata)
{
    /* input buffer timedata is stored row-wise */
    int k, ncfft=st->substate->nfft, temp;
    kiss_fft_cpx *tmpbuf = st->tmpbuf;
    kiss_fft_cpx *tmpbuf2 = st->tmpbuf + ncfft;
    kiss_fft_cpx *super_twiddles = st->super_twiddles;
    kiss_fft_scalar *freqdata_tmp = freqdata + 1;
    kiss_fft_scalar *freqdata_tmp2 = freqdata + ((ncfft<<1)-2);

    if (st->substate->inverse == 0) {
        speex_fatal ("kiss fft usage error: improper alloc\n");
    }

    temp = (ncfft<<1) - 1;
    tmpbuf->r = ADD16(freqdata[0] , freqdata[temp]);
    tmpbuf->i = SUB16(freqdata[0] , freqdata[temp]);
    /*C_FIXDIV(st->tmpbuf[0],2);*/

    temp = ncfft>>2;
    for (k = 1; k <= temp; ++k) {
        kiss_fft_cpx fk, fnkc, fek, fok, tmp;
        fk.r = *freqdata_tmp;           freqdata_tmp++;
        fk.i = *freqdata_tmp;           freqdata_tmp++;
        fnkc.i = -(*freqdata_tmp2); freqdata_tmp2--;
        fnkc.r = *freqdata_tmp2;        freqdata_tmp2--;
        /*C_FIXDIV( fk , 2 );
        C_FIXDIV( fnkc , 2 );*/

        super_twiddles++;
        tmpbuf++;
        tmpbuf2--;

        C_ADD (fek, fk, fnkc);
        C_SUB (tmp, fk, fnkc);
        C_MUL (fok, tmp, *super_twiddles);
        C_ADD (*tmpbuf,     fek, fok);
        C_SUB (*tmpbuf2, fek, fok);
#ifdef USE_SIMD        
        tmpbuf2->i *= _mm_set1_ps(-1.0);
#else
        tmpbuf2->i = -tmpbuf2->i;
#endif      

        fk.r = *freqdata_tmp;           freqdata_tmp++;
        fk.i = *freqdata_tmp;           freqdata_tmp++;
        fnkc.i = -(*freqdata_tmp2); freqdata_tmp2--;
        fnkc.r = *freqdata_tmp2;        freqdata_tmp2--;
        /*C_FIXDIV( fk , 2 );
        C_FIXDIV( fnkc , 2 );*/

        super_twiddles++;
        tmpbuf++;
        tmpbuf2--;

        C_ADD (fek, fk, fnkc);
        C_SUB (tmp, fk, fnkc);
        C_MUL (fok, tmp, *super_twiddles);
        C_ADD (*tmpbuf,     fek, fok);
        C_SUB (*tmpbuf2, fek, fok);
#ifdef USE_SIMD        
        tmpbuf2->i *= _mm_set1_ps(-1.0);
#else
        tmpbuf2->i = -tmpbuf2->i;
#endif
    }
    kiss_fft (st->substate, st->tmpbuf, (kiss_fft_cpx *) timedata);
}
#else
void kiss_fftri2(kiss_fftr_cfg st, kiss_fft_scalar *freqdata, kiss_fft_scalar *timedata)
{
    /* input buffer timedata is stored row-wise */
    int k, ncfft;

    if (st->substate->inverse == 0) {
        speex_fatal ("kiss fft usage error: improper alloc\n");
    }

    ncfft = st->substate->nfft;

    st->tmpbuf[0].r = ADD16(freqdata[0] , freqdata[2*ncfft-1]);
    st->tmpbuf[0].i = SUB16(freqdata[0] , freqdata[2*ncfft-1]);
    /*C_FIXDIV(st->tmpbuf[0],2);*/

    for (k = 1; k <= ncfft / 2; ++k) {
        kiss_fft_cpx fk, fnkc, fek, fok, tmp;
        fk.r = freqdata[2*k-1];
        fk.i = freqdata[2*k];
        fnkc.r = freqdata[2*(ncfft - k)-1];
        fnkc.i = -freqdata[2*(ncfft - k)];
        /*C_FIXDIV( fk , 2 );
        C_FIXDIV( fnkc , 2 );*/

        C_ADD (fek, fk, fnkc);
        C_SUB (tmp, fk, fnkc);
        C_MUL (fok, tmp, st->super_twiddles[k]);
        C_ADD (st->tmpbuf[k],     fek, fok);
        C_SUB (st->tmpbuf[ncfft - k], fek, fok);
#ifdef USE_SIMD        
        st->tmpbuf[ncfft - k].i *= _mm_set1_ps(-1.0);
#else
        st->tmpbuf[ncfft - k].i = -st->tmpbuf[ncfft - k].i;
#endif
    }
    kiss_fft (st->substate, st->tmpbuf, (kiss_fft_cpx *) timedata);
}
#endif
