/*
Copyright (c) 2003-2004, Mark Borgerding
Copyright (c) 2005-2007, Jean-Marc Valin

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

#include "_kiss_fft_guts.h"
#include "arch.h"
#include "os_support.h"
#include "basic_op.h"
#include "ntk_basic_operation.h"
#include "ntk_opt_switch.h"

#ifdef _ARMV7_
#ifdef _ARM_NEON_
#include <arm_neon.h>
#endif
#endif

#define KISS_FFT_OPT

void (*kiss_fft_stride)(kiss_fft_cfg st,const kiss_fft_cpx *fin,kiss_fft_cpx *fout,int in_stride);
#ifdef _ONLY_FOR_1024_SAMPLES_
unsigned char tbl_kf_shuffle_1024[256] = {
0x0,  0x40, 0x80, 0xc0, 0x10, 0x50, 0x90, 0xd0, 0x20, 0x60, 0xa0, 0xe0, 0x30, 0x70, 0xb0, 0xf0,
0x4,  0x44, 0x84, 0xc4, 0x14, 0x54, 0x94, 0xd4,0x24, 0x64, 0xa4, 0xe4, 0x34, 0x74, 0xb4, 0xf4,
0x8,  0x48, 0x88, 0xc8, 0x18, 0x58, 0x98, 0xd8,0x28, 0x68, 0xa8, 0xe8, 0x38, 0x78, 0xb8, 0xf8,
0xc,  0x4c, 0x8c, 0xcc, 0x1c, 0x5c, 0x9c, 0xdc,0x2c, 0x6c, 0xac, 0xec, 0x3c, 0x7c, 0xbc, 0xfc,
0x1,  0x41, 0x81, 0xc1, 0x11, 0x51, 0x91, 0xd1,0x21, 0x61, 0xa1, 0xe1, 0x31, 0x71, 0xb1, 0xf1,
0x5,  0x45, 0x85, 0xc5, 0x15, 0x55, 0x95, 0xd5,0x25, 0x65, 0xa5, 0xe5, 0x35, 0x75, 0xb5, 0xf5,
0x9,  0x49, 0x89, 0xc9, 0x19, 0x59, 0x99, 0xd9,0x29, 0x69, 0xa9, 0xe9, 0x39, 0x79, 0xb9, 0xf9,
0xd,  0x4d, 0x8d, 0xcd, 0x1d, 0x5d, 0x9d, 0xdd,0x2d, 0x6d, 0xad, 0xed, 0x3d, 0x7d, 0xbd, 0xfd,
0x2,  0x42, 0x82, 0xc2, 0x12, 0x52, 0x92, 0xd2,0x22, 0x62, 0xa2, 0xe2, 0x32, 0x72, 0xb2, 0xf2,
0x6,  0x46, 0x86, 0xc6, 0x16, 0x56, 0x96, 0xd6,0x26, 0x66, 0xa6, 0xe6, 0x36, 0x76, 0xb6, 0xf6,
0xa,  0x4a, 0x8a, 0xca, 0x1a, 0x5a, 0x9a, 0xda,0x2a, 0x6a, 0xaa, 0xea, 0x3a, 0x7a, 0xba, 0xfa,
0xe,  0x4e, 0x8e, 0xce, 0x1e, 0x5e, 0x9e, 0xde,0x2e, 0x6e, 0xae, 0xee, 0x3e, 0x7e, 0xbe, 0xfe,
0x3,  0x43, 0x83, 0xc3, 0x13, 0x53, 0x93, 0xd3,0x23, 0x63, 0xa3, 0xe3, 0x33, 0x73, 0xb3, 0xf3,
0x7,  0x47, 0x87, 0xc7, 0x17, 0x57, 0x97, 0xd7,0x27, 0x67, 0xa7, 0xe7, 0x37, 0x77, 0xb7, 0xf7,
0xb,  0x4b, 0x8b, 0xcb, 0x1b, 0x5b, 0x9b, 0xdb,0x2b, 0x6b, 0xab, 0xeb, 0x3b, 0x7b, 0xbb, 0xfb,
0xf,  0x4f, 0x8f, 0xcf, 0x1f, 0x5f, 0x9f, 0xdf,0x2f, 0x6f, 0xaf, 0xef, 0x3f, 0x7f, 0xbf, 0xff
};
#endif

//------------------------------------------------------------
/* The guts header contains all the multiplication and addition macros that are defined for
fixed or floating point complex numbers.  It also delares the kf_ internal functions.
*/
#ifdef _MIPS_kf_bfly2_OPT
static void kf_bfly2(
    kiss_fft_cpx * Fout,
    const size_t fstride,
    const kiss_fft_cfg st,
    int m,
    int N,
    int mm
    )
{
    kiss_fft_cpx * Fout2;
    kiss_fft_cpx * tw1;
    kiss_fft_cpx t;
    if (!st->inverse) {
        int i,j;
        kiss_fft_cpx * Fout_beg = Fout;
        for (i=0;i<N;i++)
        {
            Fout = Fout_beg;
            //Fout = Fout_beg + i*mm;
            Fout2 = Fout + m;
            tw1 = st->twiddles;
            for(j=0;j<m;j++)
            {
                /* Almost the same as the code path below, except that we divide the input by two
                (while keeping the best accuracy possible) */
                spx_word32_t tr, ti;
                tr = SHR32(SUB32(MULT16_16(Fout2->r , tw1->r),MULT16_16(Fout2->i , tw1->i)), 1);
                ti = SHR32(ADD32(MULT16_16(Fout2->i , tw1->r),MULT16_16(Fout2->r , tw1->i)), 1);
                tw1 += fstride;
#if 1               
                Fout2->r = PSHR32(((EXTEND32(Fout->r) << 14) - tr), 15);      
                Fout2->i = PSHR32(((EXTEND32(Fout->i) << 14) - ti), 15);  
                Fout->r = PSHR32(((EXTEND32(Fout->r) << 14) + tr), 15);  
                Fout->i = PSHR32(((EXTEND32(Fout->i) << 14) + ti), 15);  
#else               

                Fout2->r = PSHR32(SUB32(NTK_SHL32(EXTEND32(Fout->r), 14), tr), 15);
                Fout2->i = PSHR32(SUB32(NTK_SHL32(EXTEND32(Fout->i), 14), ti), 15);
                Fout->r = PSHR32(ADD32(NTK_SHL32(EXTEND32(Fout->r), 14), tr), 15);
                Fout->i = PSHR32(ADD32(NTK_SHL32(EXTEND32(Fout->i), 14), ti), 15);
#endif              
                ++Fout2;
                ++Fout;
            }
            Fout_beg += mm;
        }
    } else {
        int i,j;
        kiss_fft_cpx * Fout_beg = Fout;
        for (i=0;i<N;i++)
        {
            Fout = Fout_beg;
            //Fout = Fout_beg + i*mm;
            Fout2 = Fout + m;
            tw1 = st->twiddles;
            for(j=0;j<m;j++)
            {
                C_MUL (t,  *Fout2 , *tw1);
                tw1 += fstride;
                C_SUB( *Fout2 ,  *Fout , t );
                C_ADDTO( *Fout ,  t );
                ++Fout2;
                ++Fout;
            }
            Fout_beg += mm;
        }
    }
}
#else
static void kf_bfly2(
    kiss_fft_cpx * Fout,
    const size_t fstride,
    const kiss_fft_cfg st,
    int m,
    int N,
    int mm
    )
{
    kiss_fft_cpx * Fout2;
    kiss_fft_cpx * tw1;
    kiss_fft_cpx t;
    if (!st->inverse) {
        int i,j;
        kiss_fft_cpx * Fout_beg = Fout;
        for (i=0;i<N;i++)
        {
          Fout = Fout_beg + i*mm;
            Fout2 = Fout + m;
            tw1 = st->twiddles;
            for(j=0;j<m;j++)
            {
                /* Almost the same as the code path below, except that we divide the input by two
                (while keeping the best accuracy possible) */
                spx_word32_t tr, ti;
                tr = SHR32(SUB32(MULT16_16(Fout2->r , tw1->r),MULT16_16(Fout2->i , tw1->i)), 1);
                ti = SHR32(ADD32(MULT16_16(Fout2->i , tw1->r),MULT16_16(Fout2->r , tw1->i)), 1);
                tw1 += fstride;
                Fout2->r = PSHR32(SUB32(NTK_SHL32(EXTEND32(Fout->r), 14), tr), 15);
                Fout2->i = PSHR32(SUB32(NTK_SHL32(EXTEND32(Fout->i), 14), ti), 15);
                Fout->r = PSHR32(ADD32(NTK_SHL32(EXTEND32(Fout->r), 14), tr), 15);
                Fout->i = PSHR32(ADD32(NTK_SHL32(EXTEND32(Fout->i), 14), ti), 15);
                ++Fout2;
                ++Fout;
            }
        }
    } else {
        int i,j;
        kiss_fft_cpx * Fout_beg = Fout;
        for (i=0;i<N;i++)
        {
          Fout = Fout_beg + i*mm;
            Fout2 = Fout + m;
            tw1 = st->twiddles;
            for(j=0;j<m;j++)
            {
                C_MUL (t,  *Fout2 , *tw1);
                tw1 += fstride;
                C_SUB( *Fout2 ,  *Fout , t );
                C_ADDTO( *Fout ,  t );
                ++Fout2;
                ++Fout;
            }
        }
    }
}
#endif

//------------------------------------------------------------
#ifdef _MIPS_kf_bfly4_OPT
static void kf_bfly4(
    kiss_fft_cpx * Fout,
    const size_t fstride,
    const kiss_fft_cfg st,
    int m,
    int N,
    int mm
    )
{
    kiss_fft_cpx *tw1,*tw2,*tw3;
    kiss_fft_cpx scratch0, scratch1, scratch2, scratch3, scratch4, scratch5;    
    kiss_fft_cpx * Fout1, *Fout2, *Fout3;
    int i, j;
    int tmp2,tmp3;
    tmp2 = fstride<<1;
    tmp3 = tmp2 + fstride;

    if (st->inverse)
    {
        kiss_fft_cpx * Fout_beg = Fout;
        for (i=0;i<N;i++)
        {
            Fout = Fout_beg;
            Fout1 = Fout + m;
            Fout2 = Fout1 + m;
            Fout3 = Fout2 + m;
            tw3 = tw2 = tw1 = st->twiddles;
            for (j=0;j<m;j++)
            {
                C_MUL(scratch0, *Fout1, *tw1);
                C_MUL(scratch1, *Fout2, *tw2);
                C_MUL(scratch2, *Fout3, *tw3);

                C_SUB( scratch5 , *Fout, scratch1 );
                C_ADDTO(*Fout, scratch1);
                C_ADD( scratch3 , scratch0 , scratch2 );
                C_SUB( scratch4 , scratch0 , scratch2 );
                C_SUB( *Fout2, *Fout, scratch3 );
                tw1 += fstride;
                tw2 += tmp2;
                tw3 += tmp3;
                C_ADDTO( *Fout , scratch3 );

                Fout1->r = SUB16(scratch5.r , scratch4.i);
                Fout1->i = ADD16(scratch5.i , scratch4.r);
                Fout3->r = ADD16(scratch5.r , scratch4.i);
                Fout3->i = SUB16(scratch5.i , scratch4.r);
                ++Fout; ++Fout1; ++Fout2; ++Fout3;
            }
            Fout_beg += mm;
        }
    } else
    {
        kiss_fft_cpx * Fout_beg = Fout;
        for (i=0;i<N;i++)
        {
            Fout = Fout_beg;
            Fout1 = Fout + m;
            Fout2 = Fout1 + m;
            Fout3 = Fout2 + m;
            tw3 = tw2 = tw1 = st->twiddles;
            for (j=0;j<m;j++)
            {       
                C_MUL4(scratch0, *Fout1 , *tw1 );
                C_MUL4(scratch1, *Fout2 , *tw2 );
                C_MUL4(scratch2, *Fout3 , *tw3 );

                //Fout->r = NTK_PSHR16(Fout->r, 2);
                //Fout->i = NTK_PSHR16(Fout->i, 2);
                MIPS_PSHR16_2(Fout);

                C_SUB( scratch5 , *Fout, scratch1 );
                C_ADDTO(*Fout, scratch1);
                C_ADD( scratch3 , scratch0 , scratch2 );
                C_SUB( scratch4 , scratch0 , scratch2 );

                //Fout2->r = NTK_PSHR16(Fout2->r, 2);
                //Fout2->i = NTK_PSHR16(Fout2->i, 2);
                MIPS_PSHR16_2(Fout2);

                C_SUB( *Fout2, *Fout, scratch3 );
                tw1 += fstride;
                tw2 += tmp2;
                tw3 += tmp3;
                C_ADDTO( *Fout , scratch3 );

                Fout1->r = ADD16(scratch5.r , scratch4.i);
                Fout1->i = SUB16(scratch5.i , scratch4.r);
                Fout3->r = SUB16(scratch5.r , scratch4.i);
                Fout3->i = ADD16(scratch5.i , scratch4.r);
                ++Fout; ++Fout1; ++Fout2; ++Fout3;
            }
            Fout_beg += mm;
        }
    }
}

#else
#ifdef _kf_bfly4_OPT_NEON
static void kf_bfly4(
    kiss_fft_cpx * Fout,
    const size_t fstride,
    const kiss_fft_cfg st,
    int m,
    int N,
    int mm
    )
{
    kiss_fft_cpx *tw1,*tw2,*tw3;
    kiss_fft_cpx scratch[6];

    const size_t m2=2*m;
    const size_t m3=3*m;
    int i, j;
#ifdef _ARMV7_ 
    int16x4_t ctmp0010_16x4, ctmp0m1m_16x4, ctmp0m21m2_16x4, ctmp0m31m3_16x4;
    int16x4_t ctmptw1_16x4, ctmptw2_16x4, ctmptw3_16x4;
    int16x4_t ctmp0_16x4, ctmp1_16x4, ctmp2_16x4, ctmp3_16x4, ctmp4_16x4, ctmp5_16x4;
    int32x4_t ctmp0_32x4, ctmp1_32x4, ctmp2_32x4, ctmp3_32x4, ctmp_32x4;
    uint32x4_t mask_32x4;

    mask_32x4[0] = 0xffffffff;
    mask_32x4[1] = 0x00000000;
    mask_32x4[2] = 0xffffffff;
    mask_32x4[3] = 0x00000000;
#endif

    if (st->inverse)
    {
        kiss_fft_cpx * Fout_beg = Fout;
        for (i=0;i<N;i++)
        {
            Fout = Fout_beg + i*mm;
            tw3 = tw2 = tw1 = st->twiddles;
            if((m%2) != 0)
            {
                for (j=0;j<m;j++)
                {              
                    C_MUL(scratch[0],Fout[m] , *tw1 );
                    C_MUL(scratch[1],Fout[m2] , *tw2 );
                    C_MUL(scratch[2],Fout[m3] , *tw3 );

                    C_SUB( scratch[5] , *Fout, scratch[1] );
                    C_ADDTO(*Fout, scratch[1]);
                    C_ADD( scratch[3] , scratch[0] , scratch[2] );
                    C_SUB( scratch[4] , scratch[0] , scratch[2] );
                    C_SUB( Fout[m2], *Fout, scratch[3] );
                    tw1 += fstride;
                    tw2 += fstride*2;
                    tw3 += fstride*3;
                    C_ADDTO( *Fout , scratch[3] );

                    Fout[m].r = SUB16(scratch[5].r , scratch[4].i);
                    Fout[m].i = ADD16(scratch[5].i , scratch[4].r);
                    Fout[m3].r = ADD16(scratch[5].r , scratch[4].i);
                    Fout[m3].i = SUB16(scratch[5].i , scratch[4].r);
                    ++Fout;
                }
            }
            else
            {
                for (j=0;j<m;j+=2)
                {           
                    ctmp0010_16x4 = vld1_s16((const int16_t *)Fout); //Fout[0].r | Fout[0].i | Fout[1].r | Fout[1].i
                    ctmp0m1m_16x4 = vld1_s16((const int16_t *)&Fout[m]); //Fout[m].r | Fout[m].i | Fout[m+1].r | Fout[m+1].i
                    ctmp0m21m2_16x4 = vld1_s16((const int16_t *)&Fout[m2]); //Fout[m2].r | Fout[m2].i | Fout[m2+1].r | Fout[m2+1].i
                    ctmp0m31m3_16x4 = vld1_s16((const int16_t *)&Fout[m3]); //Fout[m3].r | Fout[m3].i | Fout[m3+1].r | Fout[m3+1].i
                    ctmptw1_16x4[0] = (*tw1).r;
                    ctmptw1_16x4[1] = (*tw1).i;
                    ctmptw1_16x4[2] = (*(tw1+fstride)).r;
                    ctmptw1_16x4[3] = (*(tw1+fstride)).i;

                    ctmptw2_16x4[0] = (*tw2).r;
                    ctmptw2_16x4[1] = (*tw2).i;
                    ctmptw2_16x4[2] = (*(tw2+fstride*2)).r;
                    ctmptw2_16x4[3] = (*(tw2+fstride*2)).i;

                    ctmptw3_16x4[0] = (*tw3).r;
                    ctmptw3_16x4[1] = (*tw3).i;
                    ctmptw3_16x4[2] = (*(tw3+fstride*3)).r;
                    ctmptw3_16x4[3] = (*(tw3+fstride*3)).i;

                    
                    //C_MUL(scratch[0],Fout[m] , *tw1 );
                    ctmp0_32x4 = vmull_s16(ctmp0m1m_16x4, ctmptw1_16x4); // Fout[m].r*tw1.r | Fout[m].i*tw1.i | Fout[m+1].r*(tw1+fstride).r | Fout[m+1].i*(tw1+fstride).i
                    ctmp2_32x4 = vrev64q_s32(ctmp0_32x4);                // Fout[m].i*tw1.i | Fout[m].r*tw1.r | Fout[m+1].i*(tw1+fstride).i | Fout[m+1].r*(tw1+fstride).r                    
                    ctmp0_32x4 = vqsubq_s32(ctmp0_32x4, ctmp2_32x4);      // Fout[m].r*tw1.r - Fout[m].i*tw1.i | Fout[m].i*tw1.i - Fout[m].r*tw1.r | Fout[m+1].r*(tw1+fstride).r - Fout[m+1].i*(tw1+fstride).i | Fout[m+1].i*(tw1+fstride).i - Fout[m+1].r*(tw1+fstride).r
                    
                    ctmp0_16x4 = vrev32_s16(ctmptw1_16x4);               // tw1.i                 | tw1.r                | (tw1+fstride).i                     | (tw1+fstride).r
                    ctmp1_32x4 = vmull_s16(ctmp0m1m_16x4, ctmp0_16x4);   // Fout[m].r*tw1.i | Fout[m].i*tw1.r | Fout[m+1].r*(tw1+fstride).i | Fout[m+1].i*(tw1+fstride).r
                    ctmp2_32x4 = vrev64q_s32(ctmp1_32x4);                // Fout[m].i*tw1.r | Fout[m].r*tw1.i | Fout[m+1].i*(tw1+fstride).r | Fout[m+1].r*(tw1+fstride).i 
                    ctmp1_32x4 = vqaddq_s32(ctmp1_32x4, ctmp2_32x4);      // Fout[m].r*tw1.i + Fout[m].i*tw1.r | Fout[m].i*tw1.r + Fout[m].r*tw1.i | Fout[m+1].r*(tw1+fstride).i + Fout[m+1].i*(tw1+fstride).r | Fout[m+1].i*(tw1+fstride).r + Fout[m+1].r*(tw1+fstride).i 

                    ctmp0_32x4 = vbslq_s32 (mask_32x4, ctmp0_32x4, ctmp1_32x4);                   
                    ctmp0_32x4 = vrshrq_n_s32(ctmp0_32x4, 15);
                    ctmp0_16x4 = vqmovn_s32(ctmp0_32x4);                    
                    //ctmp0_16x4 = vqrshrn_n_s32 (ctmp0_32x4, 15);                // Fout[m].r*tw1.r - Fout[m].i*tw1.i | Fout[m].r*tw1.i + Fout[m].i*tw1.r | Fout[m+1].r*(tw1+fstride).r - Fout[m+1].i*(tw1+fstride).i | Fout[m+1].r*(tw1+fstride).i + Fout[m+1].i*(tw1+fstride).r
                    
                    
                    //C_MUL(scratch[1],Fout[m2] , *tw2 );
                    ctmp1_32x4 = vmull_s16(ctmp0m21m2_16x4, ctmptw2_16x4); // Fout[m2].r*tw2.r | Fout[m2].i*tw2.i | Fout[m2+1].r*(tw2+fstride).r | Fout[m2+1].i*(tw2+fstride).i
                    ctmp2_32x4 = vrev64q_s32(ctmp1_32x4);                  // Fout[m2].i*tw2.i | Fout[m2].r*tw2.r | Fout[m2+1].i*(tw2+fstride).i | Fout[m2+1].r*(tw2+fstride).r                    
                    ctmp1_32x4 = vqsubq_s32(ctmp1_32x4, ctmp2_32x4);        // Fout[m2].r*tw2.r - Fout[m2].i*tw2.i | Fout[m2].i*tw2.i - Fout[m2].r*tw2.r | Fout[m2+1].r*(tw2+fstride).r - Fout[m2+1].i*(tw2+fstride).i | Fout[m2+1].i*(tw2+fstride).i - Fout[m2+1].r*(tw2+fstride).r
                    
                    ctmp1_16x4 = vrev32_s16(ctmptw2_16x4);                 // tw2.i                   | tw2.r                  | (tw2+fstride).i                       | (tw2+fstride).r
                    ctmp2_32x4 = vmull_s16(ctmp0m21m2_16x4, ctmp1_16x4);   // Fout[m2].r*tw2.i | Fout[m2].i*tw2.r | Fout[m2+1].r*(tw2+fstride).i | Fout[m2+1].i*(tw2+fstride).r
                    ctmp3_32x4 = vrev64q_s32(ctmp2_32x4);                  // Fout[m2].i*tw2.r | Fout[m2].r*tw2.i | Fout[m2+1].i*(tw2+fstride).r | Fout[m2+1].r*(tw2+fstride).i 
                    ctmp2_32x4 = vqaddq_s32(ctmp2_32x4, ctmp3_32x4);        // Fout[m2].r*tw2.i + Fout[m2].i*tw2.r | Fout[m2].i*tw2.r + Fout[m2].r*tw2.i | Fout[m2+1].r*(tw2+fstride).i + Fout[m2+1].i*(tw2+fstride).r | Fout[m2+1].i*(tw2+fstride).r + Fout[m2+1].r*(tw2+fstride).i 

                    ctmp1_32x4 = vbslq_s32 (mask_32x4, ctmp1_32x4, ctmp2_32x4);
                    ctmp1_32x4 = vrshrq_n_s32(ctmp1_32x4, 15);
                    ctmp1_16x4 = vqmovn_s32(ctmp1_32x4); 
                    //ctmp1_16x4 = vqrshrn_n_s32(ctmp1_32x4, 15);                  // Fout[m2].r*tw2.r - Fout[m2].i*tw2.i | Fout[m2].r*tw2.i + Fout[m2].i*tw2.r | Fout[m2+1].r*(tw2+fstride).r - Fout[m2+1].i*(tw2+fstride).i | Fout[m2+1].r*(tw2+fstride).i + Fout[m2+1].i*(tw2+fstride).r
                    
                    //C_MUL(scratch[2],Fout[m3] , *tw3 );
                    ctmp1_32x4 = vmull_s16(ctmp0m31m3_16x4, ctmptw3_16x4); // Fout[m3].r*tw3.r | Fout[m3].i*tw3.i | Fout[m3+1].r*(tw3+fstride).r | Fout[m3+1].i*(tw3+fstride).i
                    ctmp2_32x4 = vrev64q_s32(ctmp1_32x4);                  // Fout[m3].i*tw3.i | Fout[m3].r*tw3.r | Fout[m3+1].i*(tw3+fstride).i | Fout[m3+1].r*(tw3+fstride).r                    
                    ctmp1_32x4 = vqsubq_s32(ctmp1_32x4, ctmp2_32x4);        // Fout[m3].r*tw3.r - Fout[m3].i*tw3.i | Fout[m3].i*tw3.i - Fout[m3].r*tw3.r | Fout[m3+1].r*(tw3+fstride).r - Fout[m2+1].i*(tw2+fstride).i | Fout[m2+1].i*(tw2+fstride).i - Fout[m2+1].r*(tw2+fstride).r
                    
                    ctmp3_16x4 = vrev32_s16(ctmptw3_16x4);                 // tw3.i                   | tw3.r                  | (tw3+fstride).i                       | (tw3+fstride).r
                    ctmp2_32x4 = vmull_s16(ctmp0m31m3_16x4, ctmp3_16x4);   // Fout[m3].r*tw3.i | Fout[m3].i*tw3.r | Fout[m3+1].r*(tw3+fstride).i | Fout[m3+1].i*(tw3+fstride).r
                    ctmp3_32x4 = vrev64q_s32(ctmp2_32x4);                  // Fout[m3].i*tw3.r | Fout[m3].r*tw3.i | Fout[m3+1].i*(tw3+fstride).r | Fout[m3+1].r*(tw3+fstride).i 
                    ctmp2_32x4 = vqaddq_s32(ctmp2_32x4, ctmp3_32x4);        // Fout[m3].r*tw3.i + Fout[m3].i*tw3.r | Fout[m3].i*tw3.r + Fout[m3].r*tw3.i | Fout[m3+1].r*(tw3+fstride).i + Fout[m3+1].i*(tw3+fstride).r | Fout[m3+1].i*(tw3+fstride).r + Fout[m3+1].r*(tw3+fstride).i 

                    ctmp1_32x4 = vbslq_s32 (mask_32x4, ctmp1_32x4, ctmp2_32x4);
                    ctmp1_32x4 = vrshrq_n_s32(ctmp1_32x4, 15);
                    ctmp2_16x4 = vqmovn_s32(ctmp1_32x4); 
                    //ctmp2_16x4 = vqrshrn_n_s32(ctmp1_32x4, 15);                  // Fout[m2].r*tw2.r - Fout[m2].i*tw2.i | Fout[m2].r*tw2.i + Fout[m2].i*tw2.r | Fout[m2+1].r*(tw2+fstride).r - Fout[m2+1].i*(tw2+fstride).i | Fout[m2+1].r*(tw2+fstride).i + Fout[m2+1].i*(tw2+fstride).r

                    //C_SUB( scratch[5] , *Fout, scratch[1] );
                    //C_ADDTO(*Fout, scratch[1]);
                    //C_ADD( scratch[3] , scratch[0] , scratch[2] );
                    //C_SUB( scratch[4] , scratch[0] , scratch[2] );
                    //C_SUB( Fout[m2], *Fout, scratch[3] );                   
                    ctmp5_16x4 = vqsub_s16(ctmp0010_16x4, ctmp1_16x4);
                    ctmp0010_16x4 = vqadd_s16(ctmp0010_16x4, ctmp1_16x4);
                    ctmp3_16x4 = vqadd_s16(ctmp0_16x4, ctmp2_16x4);
                    ctmp4_16x4 = vqsub_s16(ctmp0_16x4, ctmp2_16x4);
                    ctmp0m21m2_16x4 = vqsub_s16(ctmp0010_16x4, ctmp3_16x4);
                    //C_ADDTO( *Fout , scratch[3] );
                    ctmp0010_16x4 = vqadd_s16(ctmp0010_16x4, ctmp3_16x4);


                    vst1_s16(&(Fout[0].r), ctmp0010_16x4);
                    vst1_s16(&(Fout[m2].r), ctmp0m21m2_16x4);
                    Fout[m].r = SUB16(ctmp5_16x4[0] , ctmp4_16x4[1]);
                    Fout[m+1].r = SUB16(ctmp5_16x4[2] , ctmp4_16x4[3]);
                    Fout[m].i = ADD16(ctmp5_16x4[1] , ctmp4_16x4[0]);
                    Fout[m+1].i = ADD16(ctmp5_16x4[3] , ctmp4_16x4[2]);

                    Fout[m3].r = ADD16(ctmp5_16x4[0] , ctmp4_16x4[1]);
                    Fout[m3+1].r = ADD16(ctmp5_16x4[2] , ctmp4_16x4[3]);
                    Fout[m3].i = SUB16(ctmp5_16x4[1] , ctmp4_16x4[0]);
                    Fout[m3+1].i = SUB16(ctmp5_16x4[3] , ctmp4_16x4[2]);
                    
                    tw1 += fstride*2;
                    tw2 += fstride*2*2;
                    tw3 += fstride*3*2;
                    Fout+=2;                             
                }
            }
        }
    } else
    {
        kiss_fft_cpx * Fout_beg = Fout;
        for (i=0;i<N;i++)
        {
            Fout = Fout_beg + i*mm;
            tw3 = tw2 = tw1 = st->twiddles;
            if((m%2) != 0)
            {
                for (j=0;j<m;j++)
                {
                    C_MUL4(scratch[0],Fout[m] , *tw1 );
                    C_MUL4(scratch[1],Fout[m2] , *tw2 );
                    C_MUL4(scratch[2],Fout[m3] , *tw3 );

                    Fout->r = (short)NTK_PSHR16((int)Fout->r, 2);
                    Fout->i = (short)NTK_PSHR16((int)Fout->i, 2);
                    C_SUB( scratch[5] , *Fout, scratch[1] );
                    C_ADDTO(*Fout, scratch[1]);
                    C_ADD( scratch[3] , scratch[0] , scratch[2] );
                    C_SUB( scratch[4] , scratch[0] , scratch[2] );
                    Fout[m2].r = (short)NTK_PSHR16((int)Fout[m2].r, 2);
                    Fout[m2].i = (short)NTK_PSHR16((int)Fout[m2].i, 2);
                    C_SUB( Fout[m2], *Fout, scratch[3] );
                    tw1 += fstride;
                    tw2 += fstride*2;
                    tw3 += fstride*3;
                    C_ADDTO( *Fout , scratch[3] );

                    Fout[m].r = ADD16(scratch[5].r , scratch[4].i);
                    Fout[m].i = SUB16(scratch[5].i , scratch[4].r);
                    Fout[m3].r = SUB16(scratch[5].r , scratch[4].i);
                    Fout[m3].i = ADD16(scratch[5].i , scratch[4].r);
                    ++Fout;
                }
            }
            else
            {
                for (j=0;j<m;j+=2)
                {
                    ctmp0010_16x4 = vld1_s16((const int16_t *)Fout); //Fout[0].r | Fout[0].i | Fout[1].r | Fout[1].i
                    ctmp0m1m_16x4 = vld1_s16((const int16_t *)&Fout[m]); //Fout[m].r | Fout[m].i | Fout[m+1].r | Fout[m+1].i
                    ctmp0m21m2_16x4 = vld1_s16((const int16_t *)&Fout[m2]); //Fout[m2].r | Fout[m2].i | Fout[m2+1].r | Fout[m2+1].i
                    ctmp0m31m3_16x4 = vld1_s16((const int16_t *)&Fout[m3]); //Fout[m3].r | Fout[m3].i | Fout[m3+1].r | Fout[m3+1].i
                    ctmptw1_16x4[0] = (*tw1).r;
                    ctmptw1_16x4[1] = (*tw1).i;
                    ctmptw1_16x4[2] = (*(tw1+fstride)).r;
                    ctmptw1_16x4[3] = (*(tw1+fstride)).i;

                    ctmptw2_16x4[0] = (*tw2).r;
                    ctmptw2_16x4[1] = (*tw2).i;
                    ctmptw2_16x4[2] = (*(tw2+fstride*2)).r;
                    ctmptw2_16x4[3] = (*(tw2+fstride*2)).i;

                    ctmptw3_16x4[0] = (*tw3).r;
                    ctmptw3_16x4[1] = (*tw3).i;
                    ctmptw3_16x4[2] = (*(tw3+fstride*3)).r;
                    ctmptw3_16x4[3] = (*(tw3+fstride*3)).i;

                    
                    //C_MUL(scratch[0],Fout[m] , *tw1 );
                    ctmp0_32x4 = vmull_s16(ctmp0m1m_16x4, ctmptw1_16x4); // Fout[m].r*tw1.r | Fout[m].i*tw1.i | Fout[m+1].r*(tw1+fstride).r | Fout[m+1].i*(tw1+fstride).i
                    ctmp2_32x4 = vrev64q_s32(ctmp0_32x4);                // Fout[m].i*tw1.i | Fout[m].r*tw1.r | Fout[m+1].i*(tw1+fstride).i | Fout[m+1].r*(tw1+fstride).r                    
                    ctmp0_32x4 = vqsubq_s32(ctmp0_32x4, ctmp2_32x4);      // Fout[m].r*tw1.r - Fout[m].i*tw1.i | Fout[m].i*tw1.i - Fout[m].r*tw1.r | Fout[m+1].r*(tw1+fstride).r - Fout[m+1].i*(tw1+fstride).i | Fout[m+1].i*(tw1+fstride).i - Fout[m+1].r*(tw1+fstride).r
                    
                    ctmp0_16x4 = vrev32_s16(ctmptw1_16x4);               // tw1.i                 | tw1.r                | (tw1+fstride).i                     | (tw1+fstride).r
                    ctmp1_32x4 = vmull_s16(ctmp0m1m_16x4, ctmp0_16x4);   // Fout[m].r*tw1.i | Fout[m].i*tw1.r | Fout[m+1].r*(tw1+fstride).i | Fout[m+1].i*(tw1+fstride).r
                    ctmp2_32x4 = vrev64q_s32(ctmp1_32x4);                // Fout[m].i*tw1.r | Fout[m].r*tw1.i | Fout[m+1].i*(tw1+fstride).r | Fout[m+1].r*(tw1+fstride).i 
                    ctmp1_32x4 = vqaddq_s32(ctmp1_32x4, ctmp2_32x4);      // Fout[m].r*tw1.i + Fout[m].i*tw1.r | Fout[m].i*tw1.r + Fout[m].r*tw1.i | Fout[m+1].r*(tw1+fstride).i + Fout[m+1].i*(tw1+fstride).r | Fout[m+1].i*(tw1+fstride).r + Fout[m+1].r*(tw1+fstride).i 

                    ctmp0_32x4 = vbslq_s32 (mask_32x4, ctmp0_32x4, ctmp1_32x4); 
                    ctmp0_32x4 = vrshrq_n_s32(ctmp0_32x4, 17);
                    ctmp0_16x4 = vqmovn_s32(ctmp0_32x4);                    
                    //ctmp0_16x4 = vqrshrn_n_s32 (ctmp0_32x4, 15);                // Fout[m].r*tw1.r - Fout[m].i*tw1.i | Fout[m].r*tw1.i + Fout[m].i*tw1.r | Fout[m+1].r*(tw1+fstride).r - Fout[m+1].i*(tw1+fstride).i | Fout[m+1].r*(tw1+fstride).i + Fout[m+1].i*(tw1+fstride).r
                    
                    
                    //C_MUL(scratch[1],Fout[m2] , *tw2 );
                    ctmp1_32x4 = vmull_s16(ctmp0m21m2_16x4, ctmptw2_16x4); // Fout[m2].r*tw2.r | Fout[m2].i*tw2.i | Fout[m2+1].r*(tw2+fstride).r | Fout[m2+1].i*(tw2+fstride).i
                    ctmp2_32x4 = vrev64q_s32(ctmp1_32x4);                  // Fout[m2].i*tw2.i | Fout[m2].r*tw2.r | Fout[m2+1].i*(tw2+fstride).i | Fout[m2+1].r*(tw2+fstride).r                    
                    ctmp1_32x4 = vqsubq_s32(ctmp1_32x4, ctmp2_32x4);        // Fout[m2].r*tw2.r - Fout[m2].i*tw2.i | Fout[m2].i*tw2.i - Fout[m2].r*tw2.r | Fout[m2+1].r*(tw2+fstride).r - Fout[m2+1].i*(tw2+fstride).i | Fout[m2+1].i*(tw2+fstride).i - Fout[m2+1].r*(tw2+fstride).r
                    
                    ctmp1_16x4 = vrev32_s16(ctmptw2_16x4);                 // tw2.i                   | tw2.r                  | (tw2+fstride).i                       | (tw2+fstride).r
                    ctmp2_32x4 = vmull_s16(ctmp0m21m2_16x4, ctmp1_16x4);   // Fout[m2].r*tw2.i | Fout[m2].i*tw2.r | Fout[m2+1].r*(tw2+fstride).i | Fout[m2+1].i*(tw2+fstride).r
                    ctmp3_32x4 = vrev64q_s32(ctmp2_32x4);                  // Fout[m2].i*tw2.r | Fout[m2].r*tw2.i | Fout[m2+1].i*(tw2+fstride).r | Fout[m2+1].r*(tw2+fstride).i 
                    ctmp2_32x4 = vqaddq_s32(ctmp2_32x4, ctmp3_32x4);        // Fout[m2].r*tw2.i + Fout[m2].i*tw2.r | Fout[m2].i*tw2.r + Fout[m2].r*tw2.i | Fout[m2+1].r*(tw2+fstride).i + Fout[m2+1].i*(tw2+fstride).r | Fout[m2+1].i*(tw2+fstride).r + Fout[m2+1].r*(tw2+fstride).i 

                    ctmp1_32x4 = vbslq_s32 (mask_32x4, ctmp1_32x4, ctmp2_32x4);
                    ctmp1_32x4 = vrshrq_n_s32(ctmp1_32x4, 17);
                    ctmp1_16x4 = vqmovn_s32(ctmp1_32x4); 
                    //ctmp1_16x4 = vqrshrn_n_s32(ctmp1_32x4, 15);                  // Fout[m2].r*tw2.r - Fout[m2].i*tw2.i | Fout[m2].r*tw2.i + Fout[m2].i*tw2.r | Fout[m2+1].r*(tw2+fstride).r - Fout[m2+1].i*(tw2+fstride).i | Fout[m2+1].r*(tw2+fstride).i + Fout[m2+1].i*(tw2+fstride).r
                    
                    //C_MUL(scratch[2],Fout[m3] , *tw3 );
                    ctmp1_32x4 = vmull_s16(ctmp0m31m3_16x4, ctmptw3_16x4); // Fout[m3].r*tw3.r | Fout[m3].i*tw3.i | Fout[m3+1].r*(tw3+fstride).r | Fout[m3+1].i*(tw3+fstride).i
                    ctmp2_32x4 = vrev64q_s32(ctmp1_32x4);                  // Fout[m3].i*tw3.i | Fout[m3].r*tw3.r | Fout[m3+1].i*(tw3+fstride).i | Fout[m3+1].r*(tw3+fstride).r                    
                    ctmp1_32x4 = vqsubq_s32(ctmp1_32x4, ctmp2_32x4);        // Fout[m3].r*tw3.r - Fout[m3].i*tw3.i | Fout[m3].i*tw3.i - Fout[m3].r*tw3.r | Fout[m3+1].r*(tw3+fstride).r - Fout[m2+1].i*(tw2+fstride).i | Fout[m2+1].i*(tw2+fstride).i - Fout[m2+1].r*(tw2+fstride).r
                    
                    ctmp3_16x4 = vrev32_s16(ctmptw3_16x4);                 // tw3.i                   | tw3.r                  | (tw3+fstride).i                       | (tw3+fstride).r
                    ctmp2_32x4 = vmull_s16(ctmp0m31m3_16x4, ctmp3_16x4);   // Fout[m3].r*tw3.i | Fout[m3].i*tw3.r | Fout[m3+1].r*(tw3+fstride).i | Fout[m3+1].i*(tw3+fstride).r
                    ctmp3_32x4 = vrev64q_s32(ctmp2_32x4);                  // Fout[m3].i*tw3.r | Fout[m3].r*tw3.i | Fout[m3+1].i*(tw3+fstride).r | Fout[m3+1].r*(tw3+fstride).i 
                    ctmp2_32x4 = vqaddq_s32(ctmp2_32x4, ctmp3_32x4);        // Fout[m3].r*tw3.i + Fout[m3].i*tw3.r | Fout[m3].i*tw3.r + Fout[m3].r*tw3.i | Fout[m3+1].r*(tw3+fstride).i + Fout[m3+1].i*(tw3+fstride).r | Fout[m3+1].i*(tw3+fstride).r + Fout[m3+1].r*(tw3+fstride).i 

                    ctmp1_32x4 = vbslq_s32 (mask_32x4, ctmp1_32x4, ctmp2_32x4);
                    ctmp1_32x4 = vrshrq_n_s32(ctmp1_32x4, 17);
                    ctmp2_16x4 = vqmovn_s32(ctmp1_32x4); 
                    //ctmp2_16x4 = vqrshrn_n_s32(ctmp1_32x4, 15);                  // Fout[m2].r*tw2.r - Fout[m2].i*tw2.i | Fout[m2].r*tw2.i + Fout[m2].i*tw2.r | Fout[m2+1].r*(tw2+fstride).r - Fout[m2+1].i*(tw2+fstride).i | Fout[m2+1].r*(tw2+fstride).i + Fout[m2+1].i*(tw2+fstride).r

                    //Fout->r = (short)NTK_PSHR16((int)Fout->r, 2);
                    //Fout->i = (short)NTK_PSHR16((int)Fout->i, 2);
                    ctmp0010_16x4 = vrshr_n_s16(ctmp0010_16x4, 2);

                    //C_SUB( scratch[5] , *Fout, scratch[1] );
                    //C_ADDTO(*Fout, scratch[1]);
                    //C_ADD( scratch[3] , scratch[0] , scratch[2] );
                    //C_SUB( scratch[4] , scratch[0] , scratch[2] );
                    //Fout[m2].r = (short)NTK_PSHR16((int)Fout[m2].r, 2);
                    //Fout[m2].i = (short)NTK_PSHR16((int)Fout[m2].i, 2);
                    //C_SUB( Fout[m2], *Fout, scratch[3] );                   
                    ctmp5_16x4 = vqsub_s16(ctmp0010_16x4, ctmp1_16x4);
                    ctmp0010_16x4 = vqadd_s16(ctmp0010_16x4, ctmp1_16x4);
                    ctmp3_16x4 = vqadd_s16(ctmp0_16x4, ctmp2_16x4);
                    ctmp4_16x4 = vqsub_s16(ctmp0_16x4, ctmp2_16x4);
                    ctmp0m21m2_16x4 = vrshr_n_s16(ctmp0m21m2_16x4, 2);
                    ctmp0m21m2_16x4 = vqsub_s16(ctmp0010_16x4, ctmp3_16x4);
                    //C_ADDTO( *Fout , scratch[3] );
                    ctmp0010_16x4 = vqadd_s16(ctmp0010_16x4, ctmp3_16x4);


                    vst1_s16(&(Fout[0].r), ctmp0010_16x4);
                    vst1_s16(&(Fout[m2].r), ctmp0m21m2_16x4);
                    Fout[m].r = ADD16(ctmp5_16x4[0] , ctmp4_16x4[1]);
                    Fout[m+1].r = ADD16(ctmp5_16x4[2] , ctmp4_16x4[3]);
                    Fout[m].i = SUB16(ctmp5_16x4[1] , ctmp4_16x4[0]);
                    Fout[m+1].i = SUB16(ctmp5_16x4[3] , ctmp4_16x4[2]);

                    Fout[m3].r = SUB16(ctmp5_16x4[0] , ctmp4_16x4[1]);
                    Fout[m3+1].r = SUB16(ctmp5_16x4[2] , ctmp4_16x4[3]);
                    Fout[m3].i = ADD16(ctmp5_16x4[1] , ctmp4_16x4[0]);
                    Fout[m3+1].i = ADD16(ctmp5_16x4[3] , ctmp4_16x4[2]);
                    
                    tw1 += fstride*2;
                    tw2 += fstride*2*2;
                    tw3 += fstride*3*2;
                    Fout+=2;                             
                }
            }
        }
    }
}
#else
static void kf_bfly4(
    kiss_fft_cpx * Fout,
    const size_t fstride,
    const kiss_fft_cfg st,
    int m,
    int N,
    int mm
    )
{
    kiss_fft_cpx *tw1,*tw2,*tw3;
    kiss_fft_cpx scratch[6];
    const size_t m2=2*m;
    const size_t m3=3*m;
    int i, j;

    if (st->inverse)
    {
        kiss_fft_cpx * Fout_beg = Fout;
        for (i=0;i<N;i++)
        {
            Fout = Fout_beg + i*mm;
            tw3 = tw2 = tw1 = st->twiddles;
            for (j=0;j<m;j++)
            {
                C_MUL(scratch[0],Fout[m] , *tw1 );
                C_MUL(scratch[1],Fout[m2] , *tw2 );
                C_MUL(scratch[2],Fout[m3] , *tw3 );

                C_SUB( scratch[5] , *Fout, scratch[1] );
                C_ADDTO(*Fout, scratch[1]);
                C_ADD( scratch[3] , scratch[0] , scratch[2] );
                C_SUB( scratch[4] , scratch[0] , scratch[2] );
                C_SUB( Fout[m2], *Fout, scratch[3] );
                tw1 += fstride;
                tw2 += fstride*2;
                tw3 += fstride*3;
                C_ADDTO( *Fout , scratch[3] );

                Fout[m].r = SUB16(scratch[5].r , scratch[4].i);
                Fout[m].i = ADD16(scratch[5].i , scratch[4].r);
                Fout[m3].r = ADD16(scratch[5].r , scratch[4].i);
                Fout[m3].i = SUB16(scratch[5].i , scratch[4].r);
                ++Fout;
            }
        }
    } else
    {
        kiss_fft_cpx * Fout_beg = Fout;
        for (i=0;i<N;i++)
        {
            Fout = Fout_beg + i*mm;
            tw3 = tw2 = tw1 = st->twiddles;
            for (j=0;j<m;j++)
            {
                C_MUL4(scratch[0],Fout[m] , *tw1 );
                C_MUL4(scratch[1],Fout[m2] , *tw2 );
                C_MUL4(scratch[2],Fout[m3] , *tw3 );

                Fout->r = (short)NTK_PSHR16((int)Fout->r, 2);
                Fout->i = (short)NTK_PSHR16((int)Fout->i, 2);
                C_SUB( scratch[5] , *Fout, scratch[1] );
                C_ADDTO(*Fout, scratch[1]);
                C_ADD( scratch[3] , scratch[0] , scratch[2] );
                C_SUB( scratch[4] , scratch[0] , scratch[2] );
                Fout[m2].r = (short)NTK_PSHR16((int)Fout[m2].r, 2);
                Fout[m2].i = (short)NTK_PSHR16((int)Fout[m2].i, 2);
                C_SUB( Fout[m2], *Fout, scratch[3] );
                tw1 += fstride;
                tw2 += fstride*2;
                tw3 += fstride*3;
                C_ADDTO( *Fout , scratch[3] );

                Fout[m].r = ADD16(scratch[5].r , scratch[4].i);
                Fout[m].i = SUB16(scratch[5].i , scratch[4].r);
                Fout[m3].r = SUB16(scratch[5].r , scratch[4].i);
                Fout[m3].i = ADD16(scratch[5].i , scratch[4].r);
                ++Fout;
            }
        }
    }
}


#endif
#endif

//------------------------------------------------------------
#ifdef KISS_FFT_OPT
static void kf_bfly3(
    kiss_fft_cpx * Fout,
    const size_t fstride,
    const kiss_fft_cfg st,
    size_t m
    )
{
    size_t k=m;
    kiss_fft_cpx *Fout1, *Fout2;
    kiss_fft_cpx *tw1,*tw2;
    kiss_fft_cpx scratch0, scratch1, scratch2, scratch3;
    kiss_fft_cpx epi3;
    int tmp2 = fstride<<1;
    epi3 = st->twiddles[fstride*m];
    Fout1 = Fout + m;
    Fout2 = Fout1 + m;

    tw1=tw2=st->twiddles;

    do{
        if (!st->inverse) {
            C_FIXDIV(*Fout,3); C_FIXDIV(*Fout1,3); C_FIXDIV(*Fout2,3);
        }

        C_MUL(scratch1,*Fout1 , *tw1);
        C_MUL(scratch2,*Fout2 , *tw2);

        C_ADD(scratch3,scratch1,scratch2);
        C_SUB(scratch0,scratch1,scratch2);
        tw1 += fstride;
        tw2 += tmp2;

        Fout1->r = Fout->r - HALF_OF(scratch3.r);
        Fout1->i = Fout->i - HALF_OF(scratch3.i);

        C_MULBYSCALAR( scratch0 , epi3.i );

        C_ADDTO(*Fout,scratch3);

        Fout2->r = Fout1->r + scratch0.i;
        Fout2->i = Fout1->i - scratch0.r;

        Fout1->r -= scratch0.i;
        Fout1->i += scratch0.r;

        ++Fout; ++Fout1; ++Fout2;
    }while(--k);
}

#else
static void kf_bfly3(
    kiss_fft_cpx * Fout,
    const size_t fstride,
    const kiss_fft_cfg st,
    size_t m
    )
{
    size_t k=m;
    const size_t m2 = 2*m;
    kiss_fft_cpx *tw1,*tw2;
    kiss_fft_cpx scratch[5];
    kiss_fft_cpx epi3;
    epi3 = st->twiddles[fstride*m];

    tw1=tw2=st->twiddles;

    do{
        if (!st->inverse) {
            C_FIXDIV(*Fout,3); C_FIXDIV(Fout[m],3); C_FIXDIV(Fout[m2],3);
        }

        C_MUL(scratch[1],Fout[m] , *tw1);
        C_MUL(scratch[2],Fout[m2] , *tw2);

        C_ADD(scratch[3],scratch[1],scratch[2]);
        C_SUB(scratch[0],scratch[1],scratch[2]);
        tw1 += fstride;
        tw2 += fstride*2;

        Fout[m].r = Fout->r - HALF_OF(scratch[3].r);
        Fout[m].i = Fout->i - HALF_OF(scratch[3].i);

        C_MULBYSCALAR( scratch[0] , epi3.i );

        C_ADDTO(*Fout,scratch[3]);

        Fout[m2].r = Fout[m].r + scratch[0].i;
        Fout[m2].i = Fout[m].i - scratch[0].r;

        Fout[m].r -= scratch[0].i;
        Fout[m].i += scratch[0].r;

        ++Fout;
    }while(--k);
}
#endif

//------------------------------------------------------------
#ifdef KISS_FFT_OPT
static void kf_bfly5(
    kiss_fft_cpx * Fout,
    const size_t fstride,
    const kiss_fft_cfg st,
    int m
    )
{
    kiss_fft_cpx *Fout0,*Fout1,*Fout2,*Fout3,*Fout4;
    int u;
    kiss_fft_cpx scratch0, scratch1, scratch2, scratch3, scratch4, scratch5, scratch6;
    kiss_fft_cpx * twiddles = st->twiddles;
    kiss_fft_cpx *tw1, *tw2, *tw3, *tw4;
    kiss_fft_cpx ya,yb;
    int tmp = fstride*m, tmp2, tmp3, tmp4;
    ya = twiddles[tmp];
    yb = twiddles[tmp<<1];

    Fout0=Fout;
    Fout1=Fout0+m;
    Fout2=Fout1+m;
    Fout3=Fout2+m;
    Fout4=Fout3+m;

    tmp = fstride;
    tmp2 = fstride<<1;
    tmp3 = tmp + tmp2;
    tmp4 = tmp2<<1;
    tw1 = tw2 = tw3 = tw4 = st->twiddles;
    for ( u=0; u<m; ++u ) 
    {
        if (!st->inverse) 
        {
            C_FIXDIV( *Fout0,5); C_FIXDIV( *Fout1,5); C_FIXDIV( *Fout2,5); C_FIXDIV( *Fout3,5); C_FIXDIV( *Fout4,5);
        }
        scratch0 = *Fout0;

        C_MUL(scratch1 ,*Fout1, *tw1);  tw1+=tmp;           //u*fstride
        C_MUL(scratch2 ,*Fout2, *tw2);  tw2+=tmp2;          //2*u*fstride
        C_MUL(scratch3 ,*Fout3, *tw3);  tw3+=tmp3;
        C_MUL(scratch4 ,*Fout4, *tw4);  tw4+=tmp4;

        C_ADD( scratch1, scratch1, scratch4);   
        C_SUB( scratch4, scratch1, scratch4);       
        C_ADD( scratch2, scratch2, scratch3);       
        C_SUB( scratch3, scratch2, scratch3);       

        Fout0->r += scratch1.r + scratch2.r;
        Fout0->i += scratch1.i + scratch2.i;

        scratch5.r = scratch0.r + S_MUL(scratch1.r,ya.r) + S_MUL(scratch2.r,yb.r);
        scratch5.i = scratch0.i + S_MUL(scratch1.i,ya.r) + S_MUL(scratch2.i,yb.r);

        scratch6.r =  S_MUL(scratch4.i,ya.i) + S_MUL(scratch3.i,yb.i);
        scratch6.i = -S_MUL(scratch4.r,ya.i) - S_MUL(scratch3.r,yb.i);

        C_SUB(*Fout1,scratch5,scratch6);
        C_ADD(*Fout4,scratch5,scratch6);

        scratch5.r = scratch0.r + S_MUL(scratch1.r,yb.r) + S_MUL(scratch2.r,ya.r);  
        scratch5.i = scratch0.i + S_MUL(scratch1.i,yb.r) + S_MUL(scratch2.i,ya.r);      
        scratch6.r = - S_MUL(scratch4.i,yb.i) + S_MUL(scratch3.i,ya.i);             
        scratch6.i = S_MUL(scratch4.r,yb.i) - S_MUL(scratch3.r,ya.i);                   

        C_ADD(*Fout2,scratch5,scratch6);
        C_SUB(*Fout3,scratch5,scratch6);

        ++Fout0;++Fout1;++Fout2;++Fout3;++Fout4;
    }
}
#else
static void kf_bfly5(
    kiss_fft_cpx * Fout,
    const size_t fstride,
    const kiss_fft_cfg st,
    int m
    )
{
    kiss_fft_cpx *Fout0,*Fout1,*Fout2,*Fout3,*Fout4;
    int u;
    kiss_fft_cpx scratch[13];
    kiss_fft_cpx * twiddles = st->twiddles;
    kiss_fft_cpx *tw;
    kiss_fft_cpx ya,yb;
    ya = twiddles[fstride*m];
    yb = twiddles[fstride*2*m];

    Fout0=Fout;
    Fout1=Fout0+m;
    Fout2=Fout0+2*m;
    Fout3=Fout0+3*m;
    Fout4=Fout0+4*m;

    tw=st->twiddles;
    for ( u=0; u<m; ++u ) {
        if (!st->inverse) {
            C_FIXDIV( *Fout0,5); C_FIXDIV( *Fout1,5); C_FIXDIV( *Fout2,5); C_FIXDIV( *Fout3,5); C_FIXDIV( *Fout4,5);
        }
        scratch[0] = *Fout0;

        C_MUL(scratch[1] ,*Fout1, tw[u*fstride]);
        C_MUL(scratch[2] ,*Fout2, tw[2*u*fstride]);
        C_MUL(scratch[3] ,*Fout3, tw[3*u*fstride]);
        C_MUL(scratch[4] ,*Fout4, tw[4*u*fstride]);

        C_ADD( scratch[7],scratch[1],scratch[4]);
        C_SUB( scratch[10],scratch[1],scratch[4]);
        C_ADD( scratch[8],scratch[2],scratch[3]);
        C_SUB( scratch[9],scratch[2],scratch[3]);

        Fout0->r += scratch[7].r + scratch[8].r;
        Fout0->i += scratch[7].i + scratch[8].i;

        scratch[5].r = scratch[0].r + S_MUL(scratch[7].r,ya.r) + S_MUL(scratch[8].r,yb.r);
        scratch[5].i = scratch[0].i + S_MUL(scratch[7].i,ya.r) + S_MUL(scratch[8].i,yb.r);

        scratch[6].r =  S_MUL(scratch[10].i,ya.i) + S_MUL(scratch[9].i,yb.i);
        scratch[6].i = -S_MUL(scratch[10].r,ya.i) - S_MUL(scratch[9].r,yb.i);

        C_SUB(*Fout1,scratch[5],scratch[6]);
        C_ADD(*Fout4,scratch[5],scratch[6]);

        scratch[11].r = scratch[0].r + S_MUL(scratch[7].r,yb.r) + S_MUL(scratch[8].r,ya.r);
        scratch[11].i = scratch[0].i + S_MUL(scratch[7].i,yb.r) + S_MUL(scratch[8].i,ya.r);
        scratch[12].r = - S_MUL(scratch[10].i,yb.i) + S_MUL(scratch[9].i,ya.i);
        scratch[12].i = S_MUL(scratch[10].r,yb.i) - S_MUL(scratch[9].r,ya.i);

        C_ADD(*Fout2,scratch[11],scratch[12]);
        C_SUB(*Fout3,scratch[11],scratch[12]);

        ++Fout0;++Fout1;++Fout2;++Fout3;++Fout4;
    }
}
#endif
//------------------------------------------------------------
#ifdef USE_KF_BFLY_GENERIC
#ifdef KISS_FFT_OPT
static void kf_bfly_generic(
    kiss_fft_cpx * Fout,
    const size_t fstride,
    const kiss_fft_cfg st,
    int m,
    int p
    )
{
    int u,k,q1,q,tmp0=fstride*m;
    kiss_fft_cpx * twiddles = st->twiddles;
    kiss_fft_cpx t;
    kiss_fft_cpx scratchbuf[17];
    int Norig = st->nfft;
    kiss_fft_cpx * Ftmp = Fout;
    int delta, init=0;

    /*CHECKBUF(scratchbuf,nscratchbuf,p);*/
    if (p>17)
        speex_fatal("KissFFT: max radix supported is 17");

    for ( u=0; u<m; ++u ) {
        Ftmp += u;
        for ( q1=0 ; q1<p ; ++q1 ) {
            scratchbuf[q1] = *Ftmp;
            if (!st->inverse) {
                C_FIXDIV(scratchbuf[q1],p);
            }
            Ftmp += m;
        }

        k=u;
        delta = init;
        for ( q1=0 ; q1<p ; ++q1 ) {
            int twidx=0;
            Fout[ k ] = scratchbuf[0];
            for (q=1;q<p;++q ) {
                twidx += delta;
                if (twidx>=Norig) twidx-=Norig;
                C_MUL(t,scratchbuf[q] , twiddles[twidx] );
                C_ADDTO( Fout[ k ] ,t);
            }
            k += m;
            delta += tmp0;
        }
        init += fstride;
    }
}

#else
/* perform the butterfly for one stage of a mixed radix FFT */
static void kf_bfly_generic(
    kiss_fft_cpx * Fout,
    const size_t fstride,
    const kiss_fft_cfg st,
    int m,
    int p
    )
{
    int u,k,q1,q;
    kiss_fft_cpx * twiddles = st->twiddles;
    kiss_fft_cpx t;
    kiss_fft_cpx scratchbuf[17];
    int Norig = st->nfft;

    /*CHECKBUF(scratchbuf,nscratchbuf,p);*/
    if (p>17)
        speex_fatal("KissFFT: max radix supported is 17");

    for ( u=0; u<m; ++u ) {
        k=u;
        for ( q1=0 ; q1<p ; ++q1 ) {
            scratchbuf[q1] = Fout[ k  ];
            if (!st->inverse) {
                C_FIXDIV(scratchbuf[q1],p);
            }
            k += m;
        }

        k=u;
        for ( q1=0 ; q1<p ; ++q1 ) {
            int twidx=0;
            Fout[ k ] = scratchbuf[0];
            for (q=1;q<p;++q ) {
                twidx += fstride * k;
                if (twidx>=Norig) twidx-=Norig;
                C_MUL(t,scratchbuf[q] , twiddles[twidx] );
                C_ADDTO( Fout[ k ] ,t);
            }
            k += m;
        }
    }
}
#endif
#endif  //USE_KF_BFLY_GENERIC
//------------------------------------------------------------

#ifdef KISS_FFT_OPT
static void kf_shuffle(
    kiss_fft_cpx * Fout,
    const kiss_fft_cpx * f,
    const size_t fstride,
    int in_stride,
    int * factors,
    const kiss_fft_cfg st
    )
{
    const int p=*factors++; /* the radix  */
    const int m=*factors++; /* stage's fft length/p */
    int tmp = fstride*in_stride;
    size_t fstride_next = fstride*p;
    kiss_fft_cpx * Ftmp = Fout;

    /*printf ("fft %d %d %d %d %d %d\n", p*m, m, p, s2, fstride*in_stride, N);*/
    if (m==1)
    {
        int j;
        for (j=0;j<p;j++)
        {
            *Ftmp = *f;
            f += tmp;
            Ftmp++;
        }
    } else {
        int j;
        for (j=0;j<p;j++)
        {
            kf_shuffle( Fout , f, fstride_next, in_stride, factors,st);
            f += tmp;
            Fout += m;
        }
    }
}
#else
static
    void kf_shuffle(
    kiss_fft_cpx * Fout,
    const kiss_fft_cpx * f,
    const size_t fstride,
    int in_stride,
    int * factors,
    const kiss_fft_cfg st
    )
{
    const int p=*factors++; /* the radix  */
    const int m=*factors++; /* stage's fft length/p */

    /*printf ("fft %d %d %d %d %d %d\n", p*m, m, p, s2, fstride*in_stride, N);*/
    if (m==1)
    {
        int j;
        for (j=0;j<p;j++)
        {
            Fout[j] = *f;
            f += fstride*in_stride;
        }
    } else {
        int j;
        for (j=0;j<p;j++)
        {
            kf_shuffle( Fout , f, fstride*p, in_stride, factors,st);
            f += fstride*in_stride;
            Fout += m;
        }
    }
}
#endif

//------------------------------------------------------------
static void kf_work(
    kiss_fft_cpx * Fout,
    const kiss_fft_cpx * f,
    const size_t fstride,
    int in_stride,
    int * factors,
    const kiss_fft_cfg st,
    int N,
    int s2,
    int m2
    )
{
    int i; 
    kiss_fft_cpx * Fout_beg=Fout;
    const int p=*factors++; /* the radix  */
    const int m=*factors++; /* stage's fft length/p */

#if 0
    /*printf ("fft %d %d %d %d %d %d\n", p*m, m, p, s2, fstride*in_stride, N);*/
    if (m==1)
    {
        /*   int j;
        for (j=0;j<p;j++)
        {
        Fout[j] = *f;
        f += fstride*in_stride;
        }*/
    } else {
        int j;
        for (j=0;j<p;j++)
        {
            kf_work( Fout , f, fstride*p, in_stride, factors,st, N*p, fstride*in_stride, m);
            f += fstride*in_stride;
            Fout += m;
        }
    }

    Fout=Fout_beg;

    switch (p) {
    case 2: kf_bfly2(Fout,fstride,st,m); break;
    case 3: kf_bfly3(Fout,fstride,st,m); break; 
    case 4: kf_bfly4(Fout,fstride,st,m); break;
    case 5: kf_bfly5(Fout,fstride,st,m); break; 
    default: kf_bfly_generic(Fout,fstride,st,m,p); break;
    }
#else
    /*printf ("fft %d %d %d %d %d %d %d\n", p*m, m, p, s2, fstride*in_stride, N, m2);*/
    /*if (m==1) 
    {
    for (i=0;i<N;i++)
    {
    int j;
    Fout = Fout_beg+i*m2;
    const kiss_fft_cpx * f2 = f+i*s2;
    for (j=0;j<p;j++)
    {
    *Fout++ = *f2;
    f2 += fstride*in_stride;
    }
    }
    }else{*/
    if(m!=1){
        kf_work( Fout , f, fstride*p, in_stride, factors,st, N*p, s2/*fstride*in_stride*/, m);       
    }


    switch (p) {
    case 2: 
        {
            kf_bfly2(Fout,fstride,st,m, N, m2); 
        }
        break;
    case 3: 
        {
            for (i=0;i<N;i++)
            {    
                Fout = Fout_beg;
                //Fout=Fout_beg+i*m2;            
                kf_bfly3(Fout,fstride,st,m);
                Fout_beg += m2;
            } 
            break; 
        }
    case 4: 
        {
            kf_bfly4(Fout,fstride,st,m, N, m2); 
        }
        break;
    case 5: 
        {
            for (i=0;i<N;i++)
            {
                Fout = Fout_beg;
                //Fout=Fout_beg+i*m2; 
                kf_bfly5(Fout,fstride,st,m);
                Fout_beg += m2;
            } 
            break; 
        }
    default: 
        {
#ifdef USE_KF_BFLY_GENERIC            
            for (i=0;i<N;i++)
            {
                Fout = Fout_beg;
                //Fout=Fout_beg+i*m2; 
                kf_bfly_generic(Fout,fstride,st,m,p);
                Fout_beg += m2;
            } 
#endif
            break;
        }
    }    
#endif //0       
}

//----------------------------------------
/*  facbuf is populated by p1,m1,p2,m2, ...
where 
p[i] * m[i] = m[i-1]
m0 = n                  */
static void kf_factor(int n,int * facbuf)
{
    int p=4;

    /*factor out powers of 4, powers of 2, then any remaining primes */
    do {
        while (n % p) {
            switch (p) {
            case 4: p = 2; break;
            case 2: p = 3; break;
            default: p += 2; break;
            }
            if (p>32000 || (spx_int32_t)p*(spx_int32_t)p > n)
                p = n;          /* no more factors, skip to end */
        }
        n /= p;
        *facbuf++ = p;
        *facbuf++ = n;
    } while (n > 1);
}

//-----------------------------------------------------------
/*
*
* User-callable function to allocate all necessary storage space for the fft.
*
* The return value is a contiguous block of memory, allocated with malloc.  As such,
* It can be freed with free(), rather than a kiss_fft-specific function.
* */
kiss_fft_cfg kiss_fft_alloc(int nfft,int inverse_fft,void * mem,size_t * lenmem )
{
    kiss_fft_cfg st=NULL;
    size_t memneeded = sizeof(struct kiss_fft_state)
        + sizeof(kiss_fft_cpx)*(nfft-1); /* twiddle factors*/

    if ( lenmem==NULL ) {
        st = ( kiss_fft_cfg)KISS_FFT_MALLOC( memneeded );
    }else{
        if (mem != NULL && *lenmem >= memneeded)
            st = (kiss_fft_cfg)mem;
        *lenmem = memneeded;
    }
    if (st) {
        int i;
        st->nfft=nfft;
        st->inverse = inverse_fft;
#ifdef FIXED_POINT
        for (i=0;i<nfft;++i) {
            spx_word32_t phase = i;
            if (!st->inverse)
                phase = -phase;
            kf_cexp2(st->twiddles+i, DIV32(NTK_SHL32(phase,17),nfft));
        }
#else
        for (i=0;i<nfft;++i) {
            const double pi=3.14159265358979323846264338327;
            double phase = ( -2*pi /nfft ) * i;
            if (st->inverse)
                phase *= -1;
            kf_cexp(st->twiddles+i, phase );
        }
#endif
        kf_factor(nfft,st->factors);
    }
    return st;
}

//---------------------------------------------------
void kiss_fft_stride_org(kiss_fft_cfg st,const kiss_fft_cpx *fin,kiss_fft_cpx *fout,int in_stride)
{
    if (fin == fout) 
    {
        speex_fatal("In-place FFT not supported");
        /*CHECKBUF(tmpbuf,ntmpbuf,st->nfft);
        kf_work(tmpbuf,fin,1,in_stride, st->factors,st);
        SPEEX_MOVE(fout,tmpbuf,st->nfft);*/
    } else {
        kf_shuffle( fout, fin, 1,in_stride, st->factors,st); 
        kf_work( fout, fin, 1,in_stride, st->factors,st, 1, in_stride, 1);
    }
}

void kiss_fft(kiss_fft_cfg cfg,const kiss_fft_cpx *fin,kiss_fft_cpx *fout)
{
    kiss_fft_stride(cfg,fin,fout,1);
}

#ifdef _ONLY_FOR_1024_SAMPLES_
//-----------------------------------------------------------------------
// FFT 2048
//-----------------------------------------------------------------------
void kf_shuffle_2048(kiss_fft_cpx * Fout, const kiss_fft_cpx * f)
{
    int i; 
    kiss_fft_cpx *dst = Fout;
    unsigned char *coef = tbl_kf_shuffle_1024;

    for(i=0; i<(256>>1); i++)
    {
        unsigned int index;
        index = *coef;  coef++;
        *dst = f[index];        dst++;
        *dst = f[index+0x400];  dst++;
        *dst = f[index+0x100];  dst++;
        *dst = f[index+0x500];  dst++;
        *dst = f[index+0x200];  dst++;
        *dst = f[index+0x600];  dst++;
        *dst = f[index+0x300];  dst++;
        *dst = f[index+0x700];  dst++;

        index = *coef;  coef++;
        *dst = f[index];        dst++;
        *dst = f[index+0x400];  dst++;
        *dst = f[index+0x100];  dst++;
        *dst = f[index+0x500];  dst++;
        *dst = f[index+0x200];  dst++;
        *dst = f[index+0x600];  dst++;
        *dst = f[index+0x300];  dst++;
        *dst = f[index+0x700];  dst++;

    }
}

static void kf_work_2048(kiss_fft_cpx * Fout, const kiss_fft_cfg st)
{
    //2048
    kf_bfly2(Fout, 1024, st,    1, 1024,    2);
    kf_bfly4(Fout,  256, st,    2,  256,    8);
    kf_bfly4(Fout,   64, st,    8,   64,   32);
    kf_bfly4(Fout,   16, st,   32,   16,  128);
    kf_bfly4(Fout,    4, st,  128,    4,  512);
    kf_bfly4(Fout,    1, st,  512,    1,    1);
}

void kiss_fft_stride_2048(kiss_fft_cfg st,const kiss_fft_cpx *fin,kiss_fft_cpx *fout,int in_stride)
{
    kf_shuffle_2048(fout, fin);
    kf_work_2048(fout, st);
}

//-----------------------------------------------------------------------
// FFT 1024
//-----------------------------------------------------------------------
void kf_shuffle_1024(kiss_fft_cpx * Fout, const kiss_fft_cpx * f)
{
    int i; 
    kiss_fft_cpx *dst = Fout;
    unsigned char *coef = tbl_kf_shuffle_1024;

    for(i=0; i<(256>>2); i++)
    {
        unsigned int index;
        index = *coef;  coef++;
        *dst = f[index];        dst++;
        *dst = f[index+0x100];  dst++;
        *dst = f[index+0x200];  dst++;
        *dst = f[index+0x300];  dst++;

        index = *coef;  coef++;
        *dst = f[index];        dst++;
        *dst = f[index+0x100];  dst++;
        *dst = f[index+0x200];  dst++;
        *dst = f[index+0x300];  dst++;

        index = *coef;  coef++;
        *dst = f[index];        dst++;
        *dst = f[index+0x100];  dst++;
        *dst = f[index+0x200];  dst++;
        *dst = f[index+0x300];  dst++;

        index = *coef;  coef++;
        *dst = f[index];        dst++;
        *dst = f[index+0x100];  dst++;
        *dst = f[index+0x200];  dst++;
        *dst = f[index+0x300];  dst++;
    }
}

static void kf_work_1024(kiss_fft_cpx * Fout, const kiss_fft_cfg st)
{
    //1024
    kf_bfly4(Fout, 256, st,  1, 256,   4); 
    kf_bfly4(Fout,  64, st,  4,  64,  16); 
    kf_bfly4(Fout,  16, st, 16,  16,  64); 
    kf_bfly4(Fout,   4, st, 64,   4, 256); 
    kf_bfly4(Fout,   1, st, 256,  1,   1); 
}

void kiss_fft_stride_1024(kiss_fft_cfg st,const kiss_fft_cpx *fin,kiss_fft_cpx *fout,int in_stride)
{
    kf_shuffle_1024(fout, fin);
    kf_work_1024(fout, st);
}

//-----------------------------------------------------------------------
// FFT 512
//-----------------------------------------------------------------------
void kf_shuffle_512(kiss_fft_cpx * Fout, const kiss_fft_cpx * f)
{
    int i; 
    kiss_fft_cpx *dst = Fout;
    unsigned char *coef = tbl_kf_shuffle_1024;

    for(i=0; i<(256>>2); i++)
    {
        unsigned int index;
        index = *coef;  coef++;
        *dst = f[index];        dst++;
        *dst = f[index+0x100];  dst++;

        index = *coef;  coef++;
        *dst = f[index];        dst++;
        *dst = f[index+0x100];  dst++;

        index = *coef;  coef++;
        *dst = f[index];        dst++;
        *dst = f[index+0x100];  dst++;

        index = *coef;  coef++;
        *dst = f[index];        dst++;
        *dst = f[index+0x100];  dst++;
    }
}

static void kf_work_512(kiss_fft_cpx * Fout, const kiss_fft_cfg st)
{
    //512
    kf_bfly2(Fout,  256, st,    1,  256,    2);
    kf_bfly4(Fout,   64, st,    2,   64,    8);
    kf_bfly4(Fout,   16, st,    8,   16,   32);
    kf_bfly4(Fout,    4, st,   32,    4,  128);
    kf_bfly4(Fout,    1, st,  128,    1,    1);
}

void kiss_fft_stride_512(kiss_fft_cfg st,const kiss_fft_cpx *fin,kiss_fft_cpx *fout,int in_stride)
{
    kf_shuffle_512(fout, fin);
    kf_work_512(fout, st);
}

//-----------------------------------------------------------------------
// FFT 256
//-----------------------------------------------------------------------
void kf_shuffle_256(kiss_fft_cpx * Fout, const kiss_fft_cpx * f)
{
    int i; 
    kiss_fft_cpx *dst = Fout;
    unsigned char *coef = tbl_kf_shuffle_1024;

    for(i=0; i<(256>>2); i++)
    {
        *dst = f[*coef]; dst++; coef++;
        *dst = f[*coef]; dst++; coef++;
        *dst = f[*coef]; dst++; coef++;
        *dst = f[*coef]; dst++; coef++;
    }
}

static void kf_work_256(kiss_fft_cpx * Fout, const kiss_fft_cfg st)
{
    //256
    kf_bfly4(Fout,   64, st,    1,   64,    4);
    kf_bfly4(Fout,   16, st,    4,   16,   16);
    kf_bfly4(Fout,    4, st,   16,    4,   64);
    kf_bfly4(Fout,    1, st,   64,    1,    1);
}

void kiss_fft_stride_256(kiss_fft_cfg st,const kiss_fft_cpx *fin,kiss_fft_cpx *fout,int in_stride)
{
    kf_shuffle_256(fout, fin);
    kf_work_256(fout, st);
}

//-----------------------------------------------------------------------
// FFT 128
//-----------------------------------------------------------------------
void kf_shuffle_128(kiss_fft_cpx * Fout, const kiss_fft_cpx * f)
{
    int i; 
    kiss_fft_cpx *dst = Fout;
    unsigned char *coef = tbl_kf_shuffle_1024;

    for(i=0; i<(128>>2); i++)
    {
        *dst = f[*coef]; dst++; coef++;
        *dst = f[*coef]; dst++; coef+=3;
        *dst = f[*coef]; dst++; coef++;
        *dst = f[*coef]; dst++; coef+=3;
    }
}

static void kf_work_128(kiss_fft_cpx * Fout, const kiss_fft_cfg st)
{
    //128
    kf_bfly2(Fout,   64, st,    1,   64,    2);
    kf_bfly4(Fout,   16, st,    2,   16,    8);
    kf_bfly4(Fout,    4, st,    8,    4,   32);
    kf_bfly4(Fout,    1, st,   32,    1,    1);
}

void kiss_fft_stride_128(kiss_fft_cfg st,const kiss_fft_cpx *fin,kiss_fft_cpx *fout,int in_stride)
{
    kf_shuffle_128(fout, fin);
    kf_work_128(fout, st);
}
#endif //#ifdef _ONLY_FOR_1024_SAMPLES_

//--------------------------------------------
void set_kiss_fft_stride(int frame_size)
{
#ifdef _ONLY_FOR_1024_SAMPLES_
    switch(frame_size)
    {
        case 2048:
            kiss_fft_stride = kiss_fft_stride_2048;
            break;
        case 1024:
            kiss_fft_stride = kiss_fft_stride_1024;
            break;
        case 512:
            kiss_fft_stride = kiss_fft_stride_512;
            break;
        case 256:
            kiss_fft_stride = kiss_fft_stride_256;
            break;
        case 128:
            kiss_fft_stride = kiss_fft_stride_128;
            break;
        default:
            kiss_fft_stride = kiss_fft_stride_org;
            break;
    }
#else
    kiss_fft_stride = kiss_fft_stride_org;
#endif
}
