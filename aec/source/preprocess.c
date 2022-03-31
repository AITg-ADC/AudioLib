/* Copyright (C) 2003 Epic Games (written by Jean-Marc Valin)
Copyright (C) 2004-2006 Epic Games

File: preprocess.c
Preprocessor with denoising based on the algorithm by Ephraim and Malah

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are
met:

1. Redistributions of source code must retain the above copyright notice,
this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright
notice, this list of conditions and the following disclaimer in the
documentation and/or other materials provided with the distribution.

3. The name of the author may not be used to endorse or promote products
derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT,
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
*/

/*
Recommended papers:

Y. Ephraim and D. Malah, "Speech enhancement using minimum mean-square error
short-time spectral amplitude estimator". IEEE Transactions on Acoustics,
Speech and Signal Processing, vol. ASSP-32, no. 6, pp. 1109-1121, 1984.

Y. Ephraim and D. Malah, "Speech enhancement using minimum mean-square error
log-spectral amplitude estimator". IEEE Transactions on Acoustics, Speech and
Signal Processing, vol. ASSP-33, no. 2, pp. 443-445, 1985.

I. Cohen and B. Berdugo, "Speech enhancement for non-stationary noise environments".
Signal Processing, vol. 81, no. 2, pp. 2403-2418, 2001.

Stefan Gustafsson, Rainer Martin, Peter Jax, and Peter Vary. "A psychoacoustic
approach to combined acoustic echo cancellation and noise reduction". IEEE
Transactions on Speech and Audio Processing, 2002.

J.-M. Valin, J. Rouat, and F. Michaud, "Microphone array post-filter for separation
of simultaneous non-stationary sources". In Proceedings IEEE International
Conference on Acoustics, Speech, and Signal Processing, 2004.
*/

#include "speex_preprocess.h"
#include "aud_aec_api.h"
#include "aud_ns_api.h"
#include "basic_op.h"
#include "ntk_basic_operation.h"
#include "ntk_opt_switch.h"

#ifdef _ARMV7_
#ifdef _ARM_NEON_
#include <arm_neon.h>
#endif
#endif

//#include "perf.h"

#ifndef M_PI
#define M_PI 3.14159263
#endif

#define LOUDNESS_EXP 5.f
#define AMP_SCALE .001f
#define AMP_SCALE_1 1000.f

#define SPEECH_PROB_START_DEFAULT QCONST16(0.35f, 15)
#define SPEECH_PROB_CONTINUE_DEFAULT QCONST16(0.20f, 15)
#define NOISE_SUPPRESS_DEFAULT -15
#define ECHO_SUPPRESS_DEFAULT -40
#define ECHO_SUPPRESS_ACTIVE_DEFAULT -15

#ifndef NULL
#define NULL 0
#endif

#define SQR(x) ((x) * (x))
#define SQR16(x) (MULT16_16((x), (x)))
#define SQR16_Q15(x) (MULT16_16_Q15((x), (x)))
#define ALLIGN_4BYTE(x) (((x) + 3) & 0xfffffffc)

// Noah@20220113
#define SIZE_OF_KISS_CONFIG 24
#define MIST_NUMBER 70+16

#ifdef FIXED_POINT

#ifdef DIV32_16_Q8_ARM_OPT_BIT_EXACT
static inline spx_word16_t DIV32_16_Q8(spx_word32_t a, spx_word32_t b)
{
    if (SHR32(a, 7) >= b) {
        return 32767;
    } else {
        if (b >= QCONST32(1, 23)) {
            __asm {
                mov a,a,asr #8
                mov b,b,asr #8
            }
        }
        if (b >= QCONST32(1, 19)) {
            __asm {
                mov a,a,asr #4
                mov b,b,asr #4
            }
        }
        if (b >= QCONST32(1, 15)) {
            __asm {
                mov a,a,asr #4
                mov b,b,asr #4
            }
        }
        __asm {
            mov a, a, lsl #8
        }
        return PDIV32_16(a, b);
    }
}

#else
static inline spx_word16_t DIV32_16_Q8(spx_word32_t a, spx_word32_t b)
{
    if (SHR32(a, 7) >= b) {
        return 32767;
    } else {
        if (b >= QCONST32(1, 23)) {
            a = SHR32(a, 8);
            b = SHR32(b, 8);
        }
        if (b >= QCONST32(1, 19)) {
            a = SHR32(a, 4);
            b = SHR32(b, 4);
        }
        if (b >= QCONST32(1, 15)) {
            a = SHR32(a, 4);
            b = SHR32(b, 4);
        }
        a = NTK_SHL32(a, 8);
        return PDIV32_16(a, b);
    }
}
#endif

#ifdef DIV32_16_Q15_ARM_OPT_BIT_EXACT
static inline spx_word16_t DIV32_16_Q15(spx_word32_t a, spx_word32_t b)
{
    if (SHR32(a, 15) >= b) {
        return 32767;
    } else {
        if (b >= QCONST32(1, 23)) {
            __asm {
                mov a,a,asr #8
                mov b,b,asr #8
            }
        }
        if (b >= QCONST32(1, 19)) {
            __asm {
                mov a,a,asr #4
                mov b,b,asr #4
            }
        }
        if (b >= QCONST32(1, 15)) {
            __asm {
                mov a,a,asr #4
                mov b,b,asr #4
            }
        }
        __asm {
            rsb a, a, a, lsl #15
        }
        return DIV32_16(a, b);
    }
}

#else
static inline spx_word16_t DIV32_16_Q15(spx_word32_t a, spx_word32_t b)
{
    if (SHR32(a, 15) >= b) {
        return 32767;
    } else {
        if (b >= QCONST32(1, 23)) {
            a = SHR32(a, 8);
            b = SHR32(b, 8);
        }
        if (b >= QCONST32(1, 19)) {
            a = SHR32(a, 4);
            b = SHR32(b, 4);
        }
        if (b >= QCONST32(1, 15)) {
            a = SHR32(a, 4);
            b = SHR32(b, 4);
        }
        a = NTK_SHL32(a, 15) - a;
        return DIV32_16(a, b);
    }
}
#endif

#define SNR_SCALING 256.f
#define SNR_SCALING_1 0.0039062f
#define SNR_SHIFT 8

#define FRAC_SCALING 32767.f
#define FRAC_SCALING_1 3.0518e-05
#define FRAC_SHIFT 1

#define EXPIN_SCALING 2048.f
#define EXPIN_SCALING_1 0.00048828f
#define EXPIN_SHIFT 11
#define EXPOUT_SCALING_1 1.5259e-05

#define NOISE_SHIFT 7

#else

#define DIV32_16_Q8(a, b) ((a) / (b))
#define DIV32_16_Q15(a, b) ((a) / (b))
#define SNR_SCALING 1.f
#define SNR_SCALING_1 1.f
#define SNR_SHIFT 0
#define FRAC_SCALING 1.f
#define FRAC_SCALING_1 1.f
#define FRAC_SHIFT 0
#define NOISE_SHIFT 0

#define EXPIN_SCALING 1.f
#define EXPIN_SCALING_1 1.f
#define EXPOUT_SCALING_1 1.f

#endif

#ifdef ADD_NLP
#define FFT_SHIFT (0)  //(1)
#define NLP_MAX(A, B) (A > B ? A : B)

typedef int complex_t32[2];
typedef spx_float_t complex_pf[2];

typedef struct _NlpState {
    short NlpEnable;
    // short NlpEffect;
    int frame_size;

    short *dBuf16;  //[PART_LEN2];  // nearend
    short *fBuf16;  //[PART_LEN2];  // nearend
    short *eBuf16;  //[PART_LEN2];  // error

    void *fft_table;
    short *fft_out;  //[PART_LEN2 + 2];
    short *fft_in;   //[PART_LEN2];
    short *window;

    short *dfw16;  //[2][PART_LEN1]
    short *xfw16;  //[2][PART_LEN1]
    short *efw16;  //[2][PART_LEN1]

    int *sd32;           //[PART_LEN1];
    int *sx32;           //[PART_LEN1];
    int *se32;           //[PART_LEN1];
    complex_t32 *sde32;  //[PART_LEN1];
    complex_t32 *sdx32;  //[PART_LEN1];

    unsigned short *cohde16;  //[PART_LEN1];
    unsigned short *cohdx16;  //[PART_LEN1];

    int isNearState;
    unsigned short *nlp_gain;
    unsigned short gainAvgMin16;

    short overDrive_Q10;
    short overDriveSm_Q10;

    short *overDriveCurve;  //[PART_LEN1];

} NlpState;

extern short powf_approach(short base0, short exp);  // base0: Q12, exp: Q8

static NlpState stNlp;
static int isNlpInitDone = 0;

const short WebRtcAec_overDriveCurve_Q13[65] = {
    0x2000, 0x2400, 0x25a8, 0x26ed, 0x2800, 0x28f1, 0x29cc, 0x2a95,
    0x2b50, 0x2c00, 0x2ca6, 0x2d44, 0x2ddb, 0x2e6c, 0x2ef7, 0x2f7d,
    0x3000, 0x307e, 0x30f8, 0x316f, 0x31e3, 0x3254, 0x32c2, 0x332f,
    0x3398, 0x3400, 0x3465, 0x34c8, 0x352a, 0x358a, 0x35e9, 0x3645,
    0x36a0, 0x36fa, 0x3753, 0x37a9, 0x3800, 0x3854, 0x38a8, 0x38fa,
    0x394c, 0x399c, 0x39ec, 0x3a3a, 0x3a88, 0x3ad4, 0x3b21, 0x3b6c,
    0x3bb6, 0x3c00, 0x3c48, 0x3c90, 0x3cd8, 0x3d1e, 0x3d65, 0x3da9,
    0x3dee, 0x3e32, 0x3e76, 0x3eb9, 0x3efb, 0x3f3d, 0x3f7f, 0x3fc0,
    0x4000};

static int fxlog(int x)
{
    int t, y;

    y = 0xa65af;
    if (x < 0x00008000) x <<= 16, y -= 0xb1721;
    if (x < 0x00800000) x <<= 8, y -= 0x58b91;
    if (x < 0x08000000) x <<= 4, y -= 0x2c5c8;
    if (x < 0x20000000) x <<= 2, y -= 0x162e4;
    if (x < 0x40000000) x <<= 1, y -= 0x0b172;
    t = x + (x >> 1);
    if ((t & 0x80000000) == 0) x = t, y -= 0x067cd;
    t = x + (x >> 2);
    if ((t & 0x80000000) == 0) x = t, y -= 0x03920;
    t = x + (x >> 3);
    if ((t & 0x80000000) == 0) x = t, y -= 0x01e27;
    t = x + (x >> 4);
    if ((t & 0x80000000) == 0) x = t, y -= 0x00f85;
    t = x + (x >> 5);
    if ((t & 0x80000000) == 0) x = t, y -= 0x007e1;
    t = x + (x >> 6);
    if ((t & 0x80000000) == 0) x = t, y -= 0x003f8;
    t = x + (x >> 7);
    if ((t & 0x80000000) == 0) x = t, y -= 0x001fe;
    x = 0x80000000 - x;
    y -= x >> 15;
    return y;
}

#ifdef _MIPS_nlp_fft_OPT
static void nlp_fft(void *state, short *Tbuf, short *Fbuf)
{
    NlpState *nlp = (NlpState *)state;
    int frame_size = nlp->frame_size;
    int i;
    {
        s32 *ps32fftin = (ps32)nlp->fft_in;
        s32 *ps32Time = (ps32)Tbuf;
        s32 *ps32Window = (ps32)nlp->window;
        for (i = 0; i < frame_size; i++) {
            // nlp->fft_in[i] = (Tbuf[i] * nlp->window[i])>>15;
            *ps32fftin = MULT16_16_P15_PH(*ps32Time, *ps32Window);
            ps32Time++;
            ps32Window++;
            ps32fftin++;
        }
    }
    spx_fft(nlp->fft_table, nlp->fft_in, nlp->fft_out);
    memcpy(Fbuf, nlp->fft_out, ((frame_size + 1) << 2));
    Fbuf[(frame_size << 1) + 1] = 0;
}
#else
static void nlp_fft(void *state, short *Tbuf, short *Fbuf)
{
    NlpState *nlp = (NlpState *)state;
    int frame_size = nlp->frame_size;
    int i;

    for (i = 0; i < (frame_size << 1); i++) {
        nlp->fft_in[i] = (Tbuf[i] * nlp->window[i]) >> 15;
    }
    spx_fft(nlp->fft_table, nlp->fft_in, nlp->fft_out);
    for (i = 0; i < ((frame_size + 1) << 1) - 1; i++) {
        Fbuf[i] = nlp->fft_out[i];
    }
    Fbuf[i] = 0;
}
#endif

int InitNLP(void *state, SpeexEchoState *echo_state)
{
    NlpState *nlp = (NlpState *)state;
    int frame_size = echo_state->frame_size;
    int window_size = frame_size << 1;
    short i, j, ilog2_uprate, step16 = 0;

    nlp->NlpEnable = 1;
    // nlp->NlpEffect = AEC_QCONST16(0.4f, 15);;
    nlp->frame_size = frame_size;

    nlp->dBuf16 = (short *)speex_alloc(window_size << 1);
    nlp->fBuf16 = (short *)speex_alloc(window_size << 1);
    nlp->eBuf16 = (short *)speex_alloc(window_size << 1);

    nlp->fft_table = spx_fft_init(window_size);
    nlp->fft_out = (short *)speex_alloc((window_size + 2) << 1);
    nlp->fft_in = (short *)speex_alloc(window_size << 1);
    nlp->window = echo_state->window;

    nlp->dfw16 = (short *)speex_alloc((frame_size + 1) << 2);
    nlp->xfw16 = (short *)speex_alloc((frame_size + 1) << 2);
    nlp->efw16 = (short *)speex_alloc((frame_size + 1) << 2);

    nlp->sd32 = (int *)speex_alloc((frame_size + 1) << 2);
    nlp->sx32 = (int *)speex_alloc((frame_size + 1) << 2);
    nlp->se32 = (int *)speex_alloc((frame_size + 1) << 2);
    for (i = 0; i < frame_size + 1; i++) {
        nlp->sd32[i] = 1;
        nlp->sx32[i] = 1;
        nlp->se32[i] = 1;
    }

    nlp->sde32 = (complex_t32 *)speex_alloc((frame_size + 1) * sizeof(complex_t32));
    nlp->sdx32 = (complex_t32 *)speex_alloc((frame_size + 1) * sizeof(complex_t32));

    nlp->cohde16 = (unsigned short *)speex_alloc((frame_size + 1) << 1);
    nlp->cohdx16 = (unsigned short *)speex_alloc((frame_size + 1) << 1);

    nlp->isNearState = 0;
    nlp->nlp_gain = NULL;
    nlp->gainAvgMin16 = 32768;

    nlp->overDrive_Q10 = 2048;
    nlp->overDriveSm_Q10 = 2048;
    // Generate overDriveCurve
    nlp->overDriveCurve = (short *)speex_alloc((frame_size + 1) << 1);

    ilog2_uprate = spx_ilog2(frame_size / 64);
    for (i = 0, j = 0; i < frame_size; i++) {
        if ((short)(i << (16 - ilog2_uprate)) == 0) {
            nlp->overDriveCurve[i] = WebRtcAec_overDriveCurve_Q13[j];
            step16 = (WebRtcAec_overDriveCurve_Q13[j + 1] - WebRtcAec_overDriveCurve_Q13[j]) >> ilog2_uprate;
            j++;
        } else
            nlp->overDriveCurve[i] = nlp->overDriveCurve[i - 1] + step16;
    }
    nlp->overDriveCurve[i] = WebRtcAec_overDriveCurve_Q13[j];
    return 0;
}

#ifdef _MIPS_nlp_preprocess_OPT_NOT_Exactly
int nlp_preprocess(void *state, spx_int16_t *near, spx_int16_t *far, spx_int16_t *err)
{
    NlpState *nlp = (NlpState *)state;
    int frame_size = nlp->frame_size;
    s16 *dfw16 = nlp->dfw16;
    s16 *xfw16 = nlp->xfw16;
    s16 *efw16 = nlp->efw16;
    s32 *psd32 = nlp->sd32;
    s32 *psx32 = nlp->sx32;
    s32 *pse32 = nlp->se32;
    s32 *psde32 = (s32 *)nlp->sde32;
    s32 *psdx32 = (s32 *)nlp->sdx32;
    s16 *pcohde16 = nlp->cohde16;
    s16 *pcohdx16 = nlp->cohdx16;

    const spx_float_t pd_Q15 = {0x4000, 0x0001};  // PSEUDOFLOAT(32768);
    spx_float_t tmp_pf0, tmp_pf1;
    complex_pf tmp_cpf;
    int i;

    memcpy(nlp->dBuf16 + frame_size, near, frame_size << 1);
    memcpy(nlp->fBuf16 + frame_size, far, frame_size << 1);
    memcpy(nlp->eBuf16 + frame_size, err, frame_size << 1);

    // near FFT
    nlp_fft(state, nlp->dBuf16, dfw16);
    nlp_fft(state, nlp->fBuf16, xfw16);
    nlp_fft(state, nlp->eBuf16, efw16);

// Spectrum density
#define GCoh (29491)
#define GCoh_1 (3277)

    for (i = 0; i < (frame_size + 1); i++) {
        s32 s32tmp;

        // power(auto) spectrum density = |X(f)|^2 = X(f)X'(f)  = X(f).real^2 + X(f).imag^2
        // tmp64 = (u64)MULT16_16(dfw16[2*i],dfw16[2*i]) + (u64)MULT16_16(dfw16[2*i+1],dfw16[2*i+1]);
        // nlp->sd32[i] = (int)PSHR((u64)GCoh * (u64)nlp->sd32[i] + (u64)GCoh_1 * (u64)tmp64, 15);
        MULT_AC0(*dfw16, *dfw16);
        MADD_AC0(*(dfw16 + 1), *(dfw16 + 1));
        s32tmp = EXTRV_RS_W(0);
        MULT_AC0(GCoh, *psd32);
        MADD_AC0(GCoh_1, s32tmp);
        *psd32 = EXTRV_RS_W(15);

        // tmp64 = (u64)MULT16_16(efw16[2*i],efw16[2*i]) + (u64)MULT16_16(efw16[2*i+1],efw16[2*i+1]);
        // nlp->se32[i] = (int)PSHR((u64)GCoh * (u64)nlp->se32[i] + (u64)GCoh_1 * (u64)tmp64, 15);
        MULT_AC0(*efw16, *efw16);
        MADD_AC0(*(efw16 + 1), *(efw16 + 1));
        s32tmp = EXTRV_RS_W(0);
        MULT_AC0(GCoh, *pse32);
        MADD_AC0(GCoh_1, s32tmp);
        *pse32 = EXTRV_RS_W(15);

        // tmp64 = NLP_MAX((u64)MULT16_16(xfw16[2*i],xfw16[2*i]) + (u64)MULT16_16(xfw16[2*i+1],xfw16[2*i+1]), 15);
        // nlp->sx32[i] = (int)PSHR((u64)GCoh * (u64)nlp->sx32[i] + (u64)GCoh_1 * (u64)tmp64, 15);
        MULT_AC0(*xfw16, *xfw16);
        MADD_AC0(*(xfw16 + 1), *(xfw16 + 1));
        s32tmp = EXTRV_RS_W(0);
        MULT_AC0(GCoh, *psx32);
        MADD_AC0(GCoh_1, s32tmp);
        *psx32 = EXTRV_RS_W(15);

        // cross spectrum density = X(f)Y'(f)  = ( X(f).real*Y(f).real + X(f).imag*Y(f).imag ) + i (X(f).imag*Y(f).real - X(f).real*Y(f).imag)   //note: X(f) = dfw16[0] + i dfw16[1]
        // tmp64 = (u64)MULT16_16(dfw16[2*i],efw16[2*i]) + (u64)MULT16_16(dfw16[2*i+1],efw16[2*i+1]);
        // nlp->sde32[i][0] = (int)PSHR((u64)GCoh * (u64)nlp->sde32[i][0] + (u64)GCoh_1 * (u64)tmp64, 15);
        MULT_AC0(*dfw16, *efw16);
        MADD_AC0(*(dfw16 + 1), *(efw16 + 1));
        s32tmp = EXTRV_RS_W(0);
        MULT_AC0(GCoh, *psde32);
        MADD_AC0(GCoh_1, s32tmp);
        *psde32 = EXTRV_RS_W(15);

        // tmp64 = (u64)MULT16_16(dfw16[2*i+1],efw16[2*i]) - (u64)MULT16_16(dfw16[2*i],efw16[2*i+1]);
        // nlp->sde32[i][1] = (int)PSHR((u64)GCoh * (u64)nlp->sde32[i][1] + (u64)GCoh_1 * (u64)tmp64, 15);
        MULT_AC0(*(dfw16 + 1), *efw16);
        MSUB_AC0(*dfw16, *(efw16 + 1));
        s32tmp = EXTRV_RS_W(0);
        MULT_AC0(GCoh, *(psde32 + 1));
        MADD_AC0(GCoh_1, s32tmp);
        *(psde32 + 1) = EXTRV_RS_W(15);

        // tmp64 = (u64)MULT16_16(dfw16[2*i],xfw16[2*i]) + (u64)MULT16_16(dfw16[2*i+1],xfw16[2*i+1]);
        // nlp->sdx32[i][0] = (int)PSHR((u64)GCoh * (u64)nlp->sdx32[i][0] + (u64)GCoh_1 * (u64)tmp64, 15);
        MULT_AC0(*dfw16, *xfw16);
        MADD_AC0(*(dfw16 + 1), *(xfw16 + 1));
        s32tmp = EXTRV_RS_W(0);
        MULT_AC0(GCoh, *psdx32);
        MADD_AC0(GCoh_1, s32tmp);
        *psdx32 = EXTRV_RS_W(15);

        // tmp64 = (u64)MULT16_16(dfw16[2*i+1],xfw16[2*i]) - (u64)MULT16_16(dfw16[2*i],xfw16[2*i+1]);
        // nlp->sdx32[i][1] = (int)PSHR((u64)GCoh * (u64)nlp->sdx32[i][1] + (u64)GCoh_1 * (u64)tmp64, 15);
        MULT_AC0(*(dfw16 + 1), *xfw16);
        MSUB_AC0(*dfw16, *(xfw16 + 1));
        s32tmp = EXTRV_RS_W(0);
        MULT_AC0(GCoh, *(psdx32 + 1));
        MADD_AC0(GCoh_1, s32tmp);
        *(psdx32 + 1) = EXTRV_RS_W(15);

        // spectrum coherence of DE = |cross specturm density of DE|^2/(psd(D)*psd(E)) ::  [[cohde = |sde|^2  / (sd * se)]]
        tmp_pf0 = PSEUDOFLOAT(*psd32);
        tmp_pf1 = PSEUDOFLOAT(*pse32);
        tmp_cpf[0] = PSEUDOFLOAT(*psde32);
        tmp_cpf[1] = PSEUDOFLOAT(*(psde32 + 1));
        tmp_pf1 = FLOAT_MULT(tmp_pf0, tmp_pf1);
        tmp_pf0 = FLOAT_MULT(FLOAT_ADD(FLOAT_MULT(tmp_cpf[0], tmp_cpf[0]), FLOAT_MULT(tmp_cpf[1], tmp_cpf[1])), pd_Q15);
        *pcohde16 = EXTRACT16(FLOAT_EXTRACT32(FLOAT_DIVU(tmp_pf0, tmp_pf1)));

        tmp_pf0 = PSEUDOFLOAT(*psd32);
        tmp_pf1 = PSEUDOFLOAT(*psx32);
        tmp_cpf[0] = PSEUDOFLOAT(*psdx32);
        tmp_cpf[1] = PSEUDOFLOAT(*(psdx32 + 1));
        tmp_pf1 = FLOAT_MULT(tmp_pf0, tmp_pf1);
        tmp_pf0 = FLOAT_MULT(FLOAT_ADD(FLOAT_MULT(tmp_cpf[0], tmp_cpf[0]), FLOAT_MULT(tmp_cpf[1], tmp_cpf[1])), pd_Q15);
        *pcohdx16 = SUB16(0x7FFF, EXTRACT16(FLOAT_EXTRACT32(FLOAT_DIVU(tmp_pf0, tmp_pf1))));

        dfw16 += 2;
        efw16 += 2;
        xfw16 += 2;
        psd32++;
        psx32++;
        pse32++;
        psde32 += 2;
        psdx32 += 2;
        pcohde16++;
        pcohdx16++;
    }

    {
        unsigned short cohdeAvg16, cohdxAvg16, gainAvg16;

        const short p60 = 19661;
        const short p01 = 328;
        const short p99 = 32440;
        const short p10 = 3277;
        const short p90 = 29491;
        const short p98 = 32113;
        const short p95 = 31130;
        const short p80 = 26214;

        // Compute average DE coherence
        {
            int tmp32 = 0, tmp32_1 = 0;
            s16 *ps16tmp0 = &nlp->cohde16[4];
            s16 *ps16tmp1 = &nlp->cohdx16[4];
            for (i = 4; i < 28; i++)  // definition for 8kHz sampling rate
            {
                tmp32 = ADD32(tmp32, (int)*ps16tmp0);
                ps16tmp0++;
                tmp32_1 = ADD32(tmp32_1, (int)*ps16tmp1);
                ps16tmp1++;
            }
            cohdeAvg16 = DIV32_E(tmp32, 24);
            cohdxAvg16 = DIV32_E(tmp32_1, 24);
        }

        // Determine b8NearState and nlp_gain
        if (cohdeAvg16 > p98 && cohdxAvg16 > p90) {
            nlp->isNearState = 1;
        } else if (cohdeAvg16 < p95 || cohdxAvg16 < p80) {
            nlp->isNearState = 0;
        }

        if (nlp->isNearState) {
            nlp->nlp_gain = nlp->cohde16;
            gainAvg16 = cohdeAvg16;
        } else {
            nlp->nlp_gain = nlp->cohdx16;
            gainAvg16 = cohdxAvg16;
        }

        // Update overDrive and overDriveSm
        if (gainAvg16 < p60 && gainAvg16 < nlp->gainAvgMin16) {
            nlp->gainAvgMin16 = gainAvg16;
            nlp->overDrive_Q10 = NLP_MAX(((int)0xad60c800 / (fxlog((int)gainAvg16 << 15) - 0x0009b43d)), 5120);  //  Q10 format
        }

        if (nlp->overDrive_Q10 < nlp->overDriveSm_Q10)
            nlp->overDriveSm_Q10 = ADD16(MULT16_16_P15(p99, nlp->overDriveSm_Q10), MULT16_16_P15(p01, nlp->overDrive_Q10));
        else
            nlp->overDriveSm_Q10 = ADD16(MULT16_16_P15(p90, nlp->overDriveSm_Q10), MULT16_16_P15(p10, nlp->overDrive_Q10));
    }

    // Compute overDrive gain
    {
        s32 *ps32Gain = (s32 *)nlp->nlp_gain;
        s32 *ps32overDrCur = (s32 *)nlp->overDriveCurve;
        s32 s32overDriveSm_Q10 = ((nlp->overDriveSm_Q10 << 16) | nlp->overDriveSm_Q10);
        s32 tmp32;
        s16 tmp16, tmp16_1;
        for (i = 0; i < (frame_size >> 1); i++) {
            tmp32 = MULT16_16_P15_PH(s32overDriveSm_Q10, *ps32overDrCur);
            ps32overDrCur++;                                                          // Q8 Q8 foramt
            *ps32Gain = SHRAV_R_PH(*ps32Gain, 3);                                     // Q12 Q12 format
            tmp16 = powf_approach((short)*ps32Gain, (short)tmp32);                    // Q12 format (base0: Q12, exp: Q8)
            tmp16_1 = powf_approach((short)(*ps32Gain >> 16), (short)(tmp32 >> 16));  // Q12 format (base0: Q12, exp: Q8)
            *ps32Gain = ((tmp16_1 << 16) | tmp16);
            *ps32Gain = SHLLV_S_PH(*ps32Gain, 3);  // Q15 format
            ps32Gain++;
        }
        tmp16 = MULT16_16_P15(nlp->overDriveSm_Q10, nlp->overDriveCurve[frame_size]);
        nlp->nlp_gain[frame_size] = SHLLV_S_PH(powf_approach((s16)SHRAV_R_PH(*ps32Gain, 3), tmp16), 3);
    }

    // Done
    memcpy(nlp->dBuf16, near, frame_size << 1);
    memcpy(nlp->fBuf16, far, frame_size << 1);
    memcpy(nlp->eBuf16, err, frame_size << 1);

    return 0;
}
#else
int nlp_preprocess(void *state, spx_int16_t *near, spx_int16_t *far, spx_int16_t *err)
{
    NlpState *nlp = (NlpState *)state;
    int frame_size = nlp->frame_size;
    int frame_size_1 = frame_size + 1;
    unsigned short cohdeAvg16, cohdxAvg16, gainAvg16;
    s16 *dfw16, *xfw16, *efw16;

    spx_float_t pd_Q15 = {0x4000, 0x0001};  // PSEUDOFLOAT(32768);
    spx_float_t tmp_pf0, tmp_pf1;
    complex_pf tmp_cpf;
    int i;
    u64 tmp64;
    const short p60 = 19661;
    const short p01 = 328;
    const short p99 = 32440;
    const short p10 = 3277;
    const short p90 = 29491;
    const short p98 = 32113;
    const short p95 = 31130;
    const short p80 = 26214;

    dfw16 = nlp->dfw16;
    xfw16 = nlp->xfw16;
    efw16 = nlp->efw16;

    memcpy(nlp->dBuf16 + frame_size, near, frame_size << 1);
    memcpy(nlp->fBuf16 + frame_size, far, frame_size << 1);
    memcpy(nlp->eBuf16 + frame_size, err, frame_size << 1);

    // near FFT
    nlp_fft(state, nlp->dBuf16, dfw16);
    nlp_fft(state, nlp->fBuf16, xfw16);
    nlp_fft(state, nlp->eBuf16, efw16);

// Spectrum density
#define GCoh (29491)
#define GCoh_1 (3277)
    for (i = 0; i < frame_size_1; i++) {
        // power(auto) spectrum density = |X(f)|^2 = X(f)X'(f)  = X(f).real^2 + X(f).imag^2
        tmp64 = (u64)MULT16_16(dfw16[2 * i], dfw16[2 * i]) + (u64)MULT16_16(dfw16[2 * i + 1], dfw16[2 * i + 1]);
        nlp->sd32[i] = (int)PSHR((u64)GCoh * (u64)nlp->sd32[i] + (u64)GCoh_1 * (u64)tmp64, 15);

        tmp64 = (u64)MULT16_16(efw16[2 * i], efw16[2 * i]) + (u64)MULT16_16(efw16[2 * i + 1], efw16[2 * i + 1]);
        nlp->se32[i] = (int)PSHR((u64)GCoh * (u64)nlp->se32[i] + (u64)GCoh_1 * (u64)tmp64, 15);

        tmp64 = NLP_MAX((u64)MULT16_16(xfw16[2 * i], xfw16[2 * i]) + (u64)MULT16_16(xfw16[2 * i + 1], xfw16[2 * i + 1]), 15);
        nlp->sx32[i] = (int)PSHR((u64)GCoh * (u64)nlp->sx32[i] + (u64)GCoh_1 * (u64)tmp64, 15);

        // cross spectrum density = X(f)Y'(f)  = ( X(f).real*Y(f).real + X(f).imag*Y(f).imag ) + i (X(f).imag*Y(f).real - X(f).real*Y(f).imag)   //note: X(f) = dfw16[0] + i dfw16[1]
        tmp64 = (u64)MULT16_16(dfw16[2 * i], efw16[2 * i]) + (u64)MULT16_16(dfw16[2 * i + 1], efw16[2 * i + 1]);
        nlp->sde32[i][0] = (int)PSHR((u64)GCoh * (u64)nlp->sde32[i][0] + (u64)GCoh_1 * (u64)tmp64, 15);
        tmp64 = (u64)MULT16_16(dfw16[2 * i + 1], efw16[2 * i]) - (u64)MULT16_16(dfw16[2 * i], efw16[2 * i + 1]);
        nlp->sde32[i][1] = (int)PSHR((u64)GCoh * (u64)nlp->sde32[i][1] + (u64)GCoh_1 * (u64)tmp64, 15);

        tmp64 = (u64)MULT16_16(dfw16[2 * i], xfw16[2 * i]) + (u64)MULT16_16(dfw16[2 * i + 1], xfw16[2 * i + 1]);
        nlp->sdx32[i][0] = (int)PSHR((u64)GCoh * (u64)nlp->sdx32[i][0] + (u64)GCoh_1 * (u64)tmp64, 15);
        tmp64 = (u64)MULT16_16(dfw16[2 * i + 1], xfw16[2 * i]) - (u64)MULT16_16(dfw16[2 * i], xfw16[2 * i + 1]);
        nlp->sdx32[i][1] = (int)PSHR((u64)GCoh * (u64)nlp->sdx32[i][1] + (u64)GCoh_1 * (u64)tmp64, 15);

        // spectrum coherence of DE = |cross specturm density of DE|^2/(psd(D)*psd(E)) ::  [[cohde = |sde|^2  / (sd * se)]]
        tmp_pf0 = PSEUDOFLOAT(nlp->sd32[i]);
        tmp_pf1 = PSEUDOFLOAT(nlp->se32[i]);
        tmp_cpf[0] = PSEUDOFLOAT(nlp->sde32[i][0]);
        tmp_cpf[1] = PSEUDOFLOAT(nlp->sde32[i][1]);
        tmp_pf1 = FLOAT_MULT(tmp_pf0, tmp_pf1);
        tmp_pf0 = FLOAT_MULT(FLOAT_ADD(FLOAT_MULT(tmp_cpf[0], tmp_cpf[0]), FLOAT_MULT(tmp_cpf[1], tmp_cpf[1])), pd_Q15);
        nlp->cohde16[i] = EXTRACT16(FLOAT_EXTRACT32(FLOAT_DIVU(tmp_pf0, tmp_pf1)));

        tmp_pf0 = PSEUDOFLOAT(nlp->sd32[i]);
        tmp_pf1 = PSEUDOFLOAT(nlp->sx32[i]);
        tmp_cpf[0] = PSEUDOFLOAT(nlp->sdx32[i][0]);
        tmp_cpf[1] = PSEUDOFLOAT(nlp->sdx32[i][1]);
        tmp_pf1 = FLOAT_MULT(tmp_pf0, tmp_pf1);
        tmp_pf0 = FLOAT_MULT(FLOAT_ADD(FLOAT_MULT(tmp_cpf[0], tmp_cpf[0]), FLOAT_MULT(tmp_cpf[1], tmp_cpf[1])), pd_Q15);
        nlp->cohdx16[i] = SUB16(0x7FFF, EXTRACT16(FLOAT_EXTRACT32(FLOAT_DIVU(tmp_pf0, tmp_pf1))));
    }

    // Compute average DE coherence
    {
        int tmp32 = 0, tmp32_1 = 0;
        for (i = 4; i < 28; i++)  // definition for 8kHz sampling rate
        {
            tmp32 += nlp->cohde16[i];
            tmp32_1 += nlp->cohdx16[i];
        }
        cohdeAvg16 = tmp32 / 24;
        cohdxAvg16 = tmp32_1 / 24;
    }

    // Determine b8NearState and nlp_gain
    if (cohdeAvg16 > p98 && cohdxAvg16 > p90) {
        nlp->isNearState = 1;
    } else if (cohdeAvg16 < p95 || cohdxAvg16 < p80) {
        nlp->isNearState = 0;
    }

    if (nlp->isNearState) {
        nlp->nlp_gain = nlp->cohde16;
        gainAvg16 = cohdeAvg16;
    } else {
        nlp->nlp_gain = nlp->cohdx16;
        gainAvg16 = cohdxAvg16;
    }

    // Update overDrive and overDriveSm
    if (gainAvg16 < p60 && gainAvg16 < nlp->gainAvgMin16) {
        nlp->gainAvgMin16 = gainAvg16;
        nlp->overDrive_Q10 = NLP_MAX(((int)0xad60c800 / (fxlog((int)gainAvg16 << 15) - 0x0009b43d)), 5120);  //  Q10 format
    }

    if (nlp->overDrive_Q10 < nlp->overDriveSm_Q10)
        nlp->overDriveSm_Q10 = ADD16(MULT16_16_P15(p99, nlp->overDriveSm_Q10), MULT16_16_P15(p01, nlp->overDrive_Q10));
    else
        nlp->overDriveSm_Q10 = ADD16(MULT16_16_P15(p90, nlp->overDriveSm_Q10), MULT16_16_P15(p10, nlp->overDrive_Q10));

    // Compute overDrive gain
    {
        for (i = 0; i < frame_size_1; i++) {
            short tmp16;
            tmp16 = MULT16_16_Q15(nlp->overDriveSm_Q10, nlp->overDriveCurve[i]);  // Q8 foramt
            nlp->nlp_gain[i] = PSHR16(nlp->nlp_gain[i], 3);                       // Q12 format
            nlp->nlp_gain[i] = powf_approach(nlp->nlp_gain[i], tmp16);            // Q12 format (base0: Q12, exp: Q8)
            nlp->nlp_gain[i] = SHL16(nlp->nlp_gain[i], 3);                        // Q15 format
        }
    }

    // Done
    memcpy(nlp->dBuf16, near, frame_size << 1);
    memcpy(nlp->fBuf16, far, frame_size << 1);
    memcpy(nlp->eBuf16, err, frame_size << 1);

    return 0;
}
#endif
#endif

static void conj_window(spx_word16_t *w, int len)
{
    int i;
    for (i = 0; i < len; i++) {
        spx_word16_t tmp;
#ifdef FIXED_POINT
        spx_word16_t x = DIV32_16(MULT16_16(32767, i), len);
#else
        spx_word16_t x = DIV32_16(MULT16_16(QCONST16(4.f, 13), i), len);
#endif
        int inv = 0;
        if (x < QCONST16(1.f, 13)) {
        } else if (x < QCONST16(2.f, 13)) {
            x = QCONST16(2.f, 13) - x;
            inv = 1;
        } else if (x < QCONST16(3.f, 13)) {
            x = x - QCONST16(2.f, 13);
            inv = 1;
        } else {
            x = QCONST16(2.f, 13) - x + QCONST16(2.f, 13); /* 4 - x */
        }
        x = MULT16_16_Q14(QCONST16(1.271903f, 14), x);
        tmp = SQR16_Q15(QCONST16(.5f, 15) - MULT16_16_P15(QCONST16(.5f, 15), spx_cos_norm(NTK_SHL32(EXTEND32(x), 2))));
        if (inv)
            tmp = SUB16(Q15_ONE, tmp);
        w[i] = spx_sqrt(NTK_SHL32(EXTEND32(tmp), 15));
    }
}

#ifdef FIXED_POINT
/* This function approximates the gain function
y = gamma(1.25)^2 * M(-.25;1;-x) / sqrt(x)
which multiplied by xi/(1+xi) is the optimal gain
in the loudness domain ( sqrt[amplitude] )
Input in Q11 format, output in Q15
*/
static inline spx_word32_t hypergeom_gain(spx_word32_t xx)
{
    int ind;
    spx_word16_t frac;
    /* Q13 table */
    static const spx_word16_t table[21] = {
        6730, 8357, 9868, 11267, 12563, 13770, 14898,
        15959, 16961, 17911, 18816, 19682, 20512, 21311,
        22082, 22827, 23549, 24250, 24931, 25594, 26241};
    ind = SHR32(xx, 10);
    if (ind < 0)
        return Q15_ONE;
    if (ind > 19)
        return ADD32(EXTEND32(Q15_ONE), EXTEND32(DIV32_16(QCONST32(.1296, 23), EXTRACT16(SHR32(xx, EXPIN_SHIFT - SNR_SHIFT)))));
    frac = NTK_SHL32(xx - NTK_SHL32(ind, 10), 5);
    return NTK_SHL32(DIV32_16(PSHR32(MULT16_16(Q15_ONE - frac, table[ind]) + MULT16_16(frac, table[ind + 1]), 7), (spx_sqrt(NTK_SHL32(xx, 15) + 6711))), 7);
}

static inline spx_word16_t qcurve(spx_word16_t x)
{
    x = MAX16(x, 1);
    return DIV32_16(NTK_SHL32(EXTEND32(32767), 9), ADD16(512, MULT16_16_Q15(QCONST16(.60f, 15), DIV32_16(32767, x))));
}

/* Compute the gain floor based on different floors for the background noise and residual echo */
// This one is for no AEC case
#ifdef _compute_gain_floor_NS_OPT
static void compute_gain_floor_NS(int noise_suppress, int effective_echo_suppress, spx_word32_t *noise, spx_word32_t *echo, spx_word16_t *gain_floor, int len)
{
    int i;
    spx_word16_t noise_gain, *pgainf = gain_floor;
    noise_gain = EXTRACT16(MIN32(Q15_ONE, SHR32(spx_exp(MULT16_16(QCONST16(.2302585f, 11), noise_suppress)), 1)));
    // noise_gain = MULT16_16_Q15(noise_gain,noise_gain);
    for (i = 0; i < len; i++)
        *pgainf++ = noise_gain;
    // spx_sqrt(NTK_SHL32(EXTEND32(DIV32_16_Q15(NTK_MULT16_32_Q15(noise_gain,PSHR32(noise[i],NOISE_SHIFT)),
    //(1+PSHR32(noise[i],NOISE_SHIFT)) )),15));
}
#else
static void compute_gain_floor_NS(int noise_suppress, int effective_echo_suppress, spx_word32_t *noise, spx_word32_t *echo, spx_word16_t *gain_floor, int len)
{
    int i;
    spx_word16_t noise_gain;
    noise_gain = EXTRACT16(MIN32(Q15_ONE, SHR32(spx_exp(MULT16_16(QCONST16(.2302585f, 11), noise_suppress)), 1)));
    // noise_gain = MULT16_16_Q15(noise_gain,noise_gain);
    for (i = 0; i < len; i++)
        gain_floor[i] = noise_gain;
    // spx_sqrt(NTK_SHL32(EXTEND32(DIV32_16_Q15(NTK_MULT16_32_Q15(noise_gain,PSHR32(noise[i],NOISE_SHIFT)),
    //(1+PSHR32(noise[i],NOISE_SHIFT)) )),15));
}
#endif

/* Compute the gain floor based on different floors for the background noise and residual echo */
#ifdef _compute_gain_floor_OPT
static void compute_gain_floor(int noise_suppress, int effective_echo_suppress, spx_word32_t *noise, spx_word32_t *echo, spx_word16_t *gain_floor, int len)
{
    int i;

    if (noise_suppress > effective_echo_suppress) {
        spx_word16_t noise_gain, gain_ratio, *ptr_gainfloor = gain_floor;
        int *p_echo = echo, *p_noise = noise;
        noise_gain = EXTRACT16(SHR32(spx_exp(MULT16_16(QCONST16(0.11513, 11), noise_suppress)), 1));
        gain_ratio = EXTRACT16(SHR32(spx_exp(MULT16_16(QCONST16(.2302585f, 11), effective_echo_suppress - noise_suppress)), 1));

        /* gain_floor = sqrt [ (noise*noise_floor + echo*echo_floor) / (noise+echo) ] */
        for (i = 0; i < len; i++) {
            int noise_shift = PSHR32(*p_noise, NOISE_SHIFT);
            *ptr_gainfloor = MULT16_16_Q15(noise_gain,
                                           spx_sqrt(NTK_SHL32(EXTEND32(DIV32_16_Q15(noise_shift + NTK_MULT16_32_Q15(gain_ratio, *p_echo), (1 + noise_shift + (*p_echo)))), 15)));
            ptr_gainfloor++;
            p_echo++;
            p_noise++;
        }
    } else {
        spx_word16_t echo_gain, gain_ratio, *ptr_gainfloor = gain_floor;
        int *p_echo = echo, *p_noise = noise;
        echo_gain = EXTRACT16(SHR32(spx_exp(MULT16_16(QCONST16(0.11513, 11), effective_echo_suppress)), 1));
        gain_ratio = EXTRACT16(SHR32(spx_exp(MULT16_16(QCONST16(.2302585f, 11), noise_suppress - effective_echo_suppress)), 1));

        /* gain_floor = sqrt [ (noise*noise_floor + echo*echo_floor) / (noise+echo) ] */
        for (i = 0; i < len; i++) {
            int noise_shift = PSHR32(*p_noise, NOISE_SHIFT);
            gain_floor[i] = MULT16_16_Q15(echo_gain,
                                          spx_sqrt(NTK_SHL32(EXTEND32(DIV32_16_Q15(NTK_MULT16_32_Q15(gain_ratio, noise_shift) + (*p_echo), (1 + noise_shift + (*p_echo)))), 15)));
            ptr_gainfloor++;
            p_echo++;
            p_noise++;
        }
    }
}
#else
static void compute_gain_floor(int noise_suppress, int effective_echo_suppress, spx_word32_t *noise, spx_word32_t *echo, spx_word16_t *gain_floor, int len)
{
    int i;

    if (noise_suppress > effective_echo_suppress) {
        spx_word16_t noise_gain, gain_ratio;
        noise_gain = EXTRACT16(MIN32(Q15_ONE, SHR32(spx_exp(MULT16_16(QCONST16(0.11513, 11), noise_suppress)), 1)));
        gain_ratio = EXTRACT16(MIN32(Q15_ONE, SHR32(spx_exp(MULT16_16(QCONST16(.2302585f, 11), effective_echo_suppress - noise_suppress)), 1)));

        /* gain_floor = sqrt [ (noise*noise_floor + echo*echo_floor) / (noise+echo) ] */
        for (i = 0; i < len; i++)
            gain_floor[i] = MULT16_16_Q15(noise_gain,
                                          spx_sqrt(NTK_SHL32(EXTEND32(DIV32_16_Q15(PSHR32(noise[i], NOISE_SHIFT) + NTK_MULT16_32_Q15(gain_ratio, echo[i]),
                                                                                   (1 + PSHR32(noise[i], NOISE_SHIFT) + echo[i]))),
                                                             15)));
    } else {
        spx_word16_t echo_gain, gain_ratio;
        echo_gain = EXTRACT16(MIN32(Q15_ONE, SHR32(spx_exp(MULT16_16(QCONST16(0.11513, 11), effective_echo_suppress)), 1)));
        gain_ratio = EXTRACT16(MIN32(Q15_ONE, SHR32(spx_exp(MULT16_16(QCONST16(.2302585f, 11), noise_suppress - effective_echo_suppress)), 1)));

        /* gain_floor = sqrt [ (noise*noise_floor + echo*echo_floor) / (noise+echo) ] */
        for (i = 0; i < len; i++)
            gain_floor[i] = MULT16_16_Q15(echo_gain,
                                          spx_sqrt(NTK_SHL32(EXTEND32(DIV32_16_Q15(NTK_MULT16_32_Q15(gain_ratio, PSHR32(noise[i], NOISE_SHIFT)) + echo[i],
                                                                                   (1 + PSHR32(noise[i], NOISE_SHIFT) + echo[i]))),
                                                             15)));
    }
}
#endif
#else
/* This function approximates the gain function
y = gamma(1.25)^2 * M(-.25;1;-x) / sqrt(x)
which multiplied by xi/(1+xi) is the optimal gain
in the loudness domain ( sqrt[amplitude] )
*/
static inline spx_word32_t hypergeom_gain(spx_word32_t xx)
{
    int ind;
    float integer, frac;
    float x;
    static const float table[21] = {
        0.82157f, 1.02017f, 1.20461f, 1.37534f, 1.53363f, 1.68092f, 1.81865f,
        1.94811f, 2.07038f, 2.18638f, 2.29688f, 2.40255f, 2.50391f, 2.60144f,
        2.69551f, 2.78647f, 2.87458f, 2.96015f, 3.04333f, 3.12431f, 3.20326f};
    x = EXPIN_SCALING_1 * xx;
    integer = floor(2 * x);
    ind = (int)integer;
    if (ind < 0)
        return FRAC_SCALING;
    if (ind > 19)
        return FRAC_SCALING * (1 + .1296 / x);
    frac = 2 * x - integer;
    return FRAC_SCALING * ((1 - frac) * table[ind] + frac * table[ind + 1]) / sqrt(x + .0001f);
}

static inline spx_word16_t qcurve(spx_word16_t x)
{
    return 1.f / (1.f + .15f / (SNR_SCALING_1 * x));
}

static void compute_gain_floor(int noise_suppress, int effective_echo_suppress, spx_word32_t *noise, spx_word32_t *echo, spx_word16_t *gain_floor, int len)
{
    int i;
    float echo_floor;
    float noise_floor;

    noise_floor = exp(.2302585f * noise_suppress);
    echo_floor = exp(.2302585f * effective_echo_suppress);

    /* Compute the gain floor based on different floors for the background noise and residual echo */
    for (i = 0; i < len; i++)
        gain_floor[i] = FRAC_SCALING * sqrt(noise_floor * PSHR32(noise[i], NOISE_SHIFT) + echo_floor * echo[i]) / sqrt(1 + PSHR32(noise[i], NOISE_SHIFT) + echo[i]);
}

#endif
EXPORT SpeexPreprocessState *speex_preprocess_state_init(int frame_size, int sampling_rate, void *pstEchoState)
{
    int i, tmp, N, M;
    // int N3, N4;

    SpeexPreprocessState *st = (SpeexPreprocessState *)speex_alloc(sizeof(SpeexPreprocessState));
    if (!st) return 0;

    st->frame_size = frame_size;

    /* Round ps_size down to the nearest power of two */
#if 0
    i=1;
    st->ps_size = st->frame_size;
    while(1)
    {
        if (st->ps_size & ~i)
        {
            st->ps_size &= ~i;
            i<<=1;
        } else {
            break;
        }
    }


    if (st->ps_size < 3*st->frame_size/4)
        st->ps_size = st->ps_size * 3 / 2;
#else
    st->ps_size = st->frame_size;
#endif

    N = st->ps_size;
    // N3 = (N<<1) - st->frame_size;
    // N4 = st->frame_size - N3;

    st->sampling_rate = sampling_rate;
#ifdef USE_THOSE_PARAMS
    st->denoise_enabled = 1;
    st->vad_enabled = 0;
    st->dereverb_enabled = 0;
    st->reverb_decay = 0;
    st->reverb_level = 0;
#endif
    st->noise_suppress = NOISE_SUPPRESS_DEFAULT;
    st->echo_suppress = ECHO_SUPPRESS_DEFAULT;
    st->echo_suppress_active = ECHO_SUPPRESS_ACTIVE_DEFAULT;

#ifdef USE_THOSE_PARAMS
    st->speech_prob_start = SPEECH_PROB_START_DEFAULT;
    st->speech_prob_continue = SPEECH_PROB_CONTINUE_DEFAULT;
#endif
    st->echo_state = (SpeexEchoState *)pstEchoState;

    st->echo_noise_ratio = QCONST16(.8f, 15);

    st->nbands = NB_BANDS;
    M = st->nbands;
    st->bank = filterbank_new(M, sampling_rate, N, 1);

    tmp = N << 2;  // 2*N*sizeof(spx_word16_t)
    st->frame = (spx_word16_t *)speex_alloc(tmp);
    st->window = (spx_word16_t *)speex_alloc(tmp);
    st->ft = (spx_word16_t *)speex_alloc(tmp);

    tmp = (N + M) << 2;  //(N+M)*sizeof(spx_word32_t)
    st->ps = (spx_word32_t *)speex_alloc(tmp);
    st->noise = (spx_word32_t *)speex_alloc(tmp);
    st->echo_noise = (spx_word32_t *)speex_alloc(tmp);
    st->residual_echo = (spx_word32_t *)speex_alloc(tmp);

#ifdef USE_THOSE_PARAMS
    st->reverb_estimate = (spx_word32_t *)speex_alloc(tmp);
#endif
    st->old_ps = (spx_word32_t *)speex_alloc(tmp);

    tmp = tmp >> 1;  //(N+M)*sizeof(spx_word16_t)
    st->prior = (spx_word16_t *)speex_alloc(tmp);
    st->post = (spx_word16_t *)speex_alloc(tmp);
    st->gain = (spx_word16_t *)speex_alloc(tmp);
    st->gain2 = (spx_word16_t *)speex_alloc(tmp);
    st->gain_floor = (spx_word16_t *)speex_alloc(tmp);
    st->zeta = (spx_word16_t *)speex_alloc(tmp);

    tmp = N << 2;  // N*sizeof(spx_word32_t)
    st->S = (spx_word32_t *)speex_alloc(tmp);
    st->Smin = (spx_word32_t *)speex_alloc(tmp);
    st->Stmp = (spx_word32_t *)speex_alloc(tmp);
    st->update_prob = (int *)speex_alloc(tmp);

    tmp = tmp >> 1;  // N3*sizeof(spx_word16_t)
    st->inbuf = (spx_word16_t *)speex_alloc(tmp);
    st->outbuf = (spx_word16_t *)speex_alloc(tmp);
    if (!st->outbuf) return 0;

    conj_window(st->window, N << 1);

    /*for (i=2*N3;i<2*st->ps_size;i++)
    st->window[i]=Q15_ONE;

    if (N4>0)
    {
    for (i=N3-1;i>=0;i--)
    {
    st->window[i+N3+N4]=st->window[i+N3];
    st->window[i+N3]=1;
    }
    }*/

    tmp = (N + M) << 2;
    // memset(st->noise, QCONST32(1.f,NOISE_SHIFT), tmp);
#ifdef USE_THOSE_PARAMS
    memset(st->reverb_estimate, 0, tmp);
#endif
    memset(st->old_ps, 1, tmp);
    tmp = tmp >> 1;
    memset(st->gain, 32767, tmp);
    // memset(st->post, NTK_SHL16(1, SNR_SHIFT), tmp);
    // memset(st->prior, NTK_SHL16(1, SNR_SHIFT), tmp);
    {
        int default_noise = QCONST32(1.f, NOISE_SHIFT);
        int default_prop = NTK_SHL16(1, SNR_SHIFT);
        for (i = 0; i < N + M; i++) {
            st->noise[i] = default_noise;
            // st->reverb_estimate[i]=0;
            // st->old_ps[i]=1;
            // st->gain[i]=Q15_ONE;
            st->post[i] = default_prop;
            st->prior[i] = default_prop;
        }
    }

    memset(st->update_prob, 1, N << 2);
    tmp = N >> 1;
    memset(st->inbuf, 0, tmp);
    memset(st->outbuf, 0, tmp);
    /*for (i=0;i<N;i++)
    st->update_prob[i] = 1;
    for (i=0;i<N3;i++)
    {
    st->inbuf[i]=0;
    st->outbuf[i]=0;
    }*/

#ifndef FIXED_POINT
    st->agc_enabled = 0;
    st->agc_level = 8000;
    st->loudness_weight = (float *)speex_alloc(N * sizeof(float));
    if (!st->loudness_weight) return 0;

    for (i = 0; i < N; i++) {
        float ff = ((float)i) * .5 * sampling_rate / ((float)N);
        /*st->loudness_weight[i] = .5f*(1.f/(1.f+ff/8000.f))+1.f*exp(-.5f*(ff-3800.f)*(ff-3800.f)/9e5f);*/
        st->loudness_weight[i] = .35f - .35f * ff / 16000.f + .73f * exp(-.5f * (ff - 3800) * (ff - 3800) / 9e5f);
        if (st->loudness_weight[i] < .01f)
            st->loudness_weight[i] = .01f;
        st->loudness_weight[i] *= st->loudness_weight[i];
    }
    /*st->loudness = pow(AMP_SCALE*st->agc_level,LOUDNESS_EXP);*/
    st->loudness = 1e-15;
    st->agc_gain = 1;
    st->max_gain = 30;
    st->max_increase_step = exp(0.11513f * 12. * st->frame_size / st->sampling_rate);
    st->max_decrease_step = exp(-0.11513f * 40. * st->frame_size / st->sampling_rate);
    st->prev_loudness = 1;
    st->init_max = 1;
#endif
#ifdef USE_THOSE_PARAMS
    st->was_speech = 0;
#endif

    st->fft_lookup = spx_fft_init(N << 1);
    if (!st->fft_lookup) return 0;

    st->nb_adapt = 0;
    st->min_count = 0;
    st->bank_scale = 1;  // linear scale

#ifdef ADD_NLP
    if (st->echo_state) {
        if ((st->echo_state->C == 1) && (st->echo_state->K == 1) && (sampling_rate == 8000)) {
            if (!isNlpInitDone) {
                InitNLP(&stNlp, st->echo_state);
                isNlpInitDone = 1;
            }
        } else {
            stNlp.NlpEnable = 0;
        }
    }
#endif
    return st;
}
#if 0
EXPORT void speex_preprocess_state_destroy(void *state)
{
    SpeexPreprocessState *st = (SpeexPreprocessState *)state;
    speex_free(st->frame);
    speex_free(st->ft);
    speex_free(st->ps);
    speex_free(st->gain2);
    speex_free(st->gain_floor);
    speex_free(st->window);
    speex_free(st->noise);
    speex_free(st->reverb_estimate);
    speex_free(st->old_ps);
    speex_free(st->gain);
    speex_free(st->prior);
    speex_free(st->post);
#ifndef FIXED_POINT
    speex_free(st->loudness_weight);
#endif
    speex_free(st->echo_noise);
    speex_free(st->residual_echo);

    speex_free(st->S);
    speex_free(st->Smin);
    speex_free(st->Stmp);
    speex_free(st->update_prob);
    speex_free(st->zeta);

    speex_free(st->inbuf);
    speex_free(st->outbuf);

    spx_fft_destroy(st->fft_lookup);
    filterbank_destroy(st->bank);
    speex_free(st);
}
#endif
/* FIXME: The AGC doesn't work yet with fixed-point*/
#ifndef FIXED_POINT
static void speex_compute_agc(SpeexPreprocessState *st, spx_word16_t Pframe, spx_word16_t *ft)
{
    int i;
    int N = st->ps_size;
    float target_gain;
    float loudness = 1.f;
    float rate;

    for (i = 2; i < N; i++) {
        loudness += 2.f * N * st->ps[i] * st->loudness_weight[i];
    }
    loudness = sqrt(loudness);
    /*if (loudness < 2*pow(st->loudness, 1.0/LOUDNESS_EXP) &&
    loudness*2 > pow(st->loudness, 1.0/LOUDNESS_EXP))*/
    if (Pframe > .3f) {
        /*rate=2.0f*Pframe*Pframe/(1+st->nb_loudness_adapt);*/
        rate = .03 * Pframe * Pframe;
        st->loudness = (1 - rate) * st->loudness + (rate)*pow(AMP_SCALE * loudness, LOUDNESS_EXP);
        st->loudness_accum = (1 - rate) * st->loudness_accum + rate;
        if (st->init_max < st->max_gain && st->nb_adapt > 20)
            st->init_max *= 1.f + .1f * Pframe * Pframe;
    }
    /*printf ("%f %f %f %f\n", Pframe, loudness, pow(st->loudness, 1.0f/LOUDNESS_EXP), st->loudness2);*/

    target_gain = AMP_SCALE * st->agc_level * pow(st->loudness / (1e-4 + st->loudness_accum), -1.0f / LOUDNESS_EXP);

    if ((Pframe > .5 && st->nb_adapt > 20) || target_gain < st->agc_gain) {
        if (target_gain > st->max_increase_step * st->agc_gain)
            target_gain = st->max_increase_step * st->agc_gain;
        if (target_gain < st->max_decrease_step * st->agc_gain && loudness < 10 * st->prev_loudness)
            target_gain = st->max_decrease_step * st->agc_gain;
        if (target_gain > st->max_gain)
            target_gain = st->max_gain;
        if (target_gain > st->init_max)
            target_gain = st->init_max;

        st->agc_gain = target_gain;
    }
    /*fprintf (stderr, "%f %f %f\n", loudness, (float)AMP_SCALE_1*pow(st->loudness, 1.0f/LOUDNESS_EXP), st->agc_gain);*/

    for (i = 0; i < 2 * N; i++)
        ft[i] *= st->agc_gain;
    st->prev_loudness = loudness;
}
#endif

#ifdef _MIPS_preprocess_analysis_OPT
static void preprocess_analysis(SpeexPreprocessState *st, spx_int16_t *x, int num_chan, int chan_index)
{
    int i;
    int N = st->ps_size;  // frame_size
    // int N2 = N<<1;
    // int N3 = N;//N<<1 - st->frame_size;   //frame_size
    // int N4 = 0;//st->frame_size - N3; //0
    spx_word32_t *pPS = st->ps;
    spx_word16_t max_val = 0;
    s16 *pFrame = st->frame, *pInbuf = st->inbuf, *px = x + chan_index, *pWindow = st->window;
    s16 *pFrame2 = (pFrame + N), *pWindow2 = (st->window + N);
    int shift;

    /* 'Build' input frame */
    for (i = 0; i < N; i++) {
        *pFrame = MULT16_16_Q15(*pInbuf, *pWindow);
        max_val |= ABS16(*pFrame);

        *pInbuf++ = *px;  // Update inbuf
        *pFrame2 = MULT16_16_Q15(*px, *pWindow2);
        max_val |= ABS16(*pFrame2);

        pFrame++;
        pFrame2++;
        pWindow++;
        pWindow2++;
        px += num_chan;
    }

#ifdef FIXED_POINT
    {
        int s32max_val = max_val;
        int *ps32Frame = (int *)st->frame;

        st->frame_shift = CLZ_32(s32max_val) - 17;
        shift = st->frame_shift;

        for (i = 0; i < (N >> 2); i++) {
            *ps32Frame = SHLLV_PH(*ps32Frame, shift);
            ps32Frame++;
            *ps32Frame = SHLLV_PH(*ps32Frame, shift);
            ps32Frame++;
            *ps32Frame = SHLLV_PH(*ps32Frame, shift);
            ps32Frame++;
            *ps32Frame = SHLLV_PH(*ps32Frame, shift);
            ps32Frame++;
        }
    }
#endif

    /* Perform FFT */
    spx_fft(st->fft_lookup, st->frame, st->ft);

    /* Power spectrum */
    shift = shift << 1;
    pWindow = st->ft;
    *pPS = MULT16_16(*pWindow, *pWindow);
    pWindow++;
    *pPS = PSHR32(*pPS, shift);
    pPS++;
    for (i = 1; i < N; i++) {
        //*pPS = MULT16_16(st->ft[2*i-1],st->ft[2*i-1]) + MULT16_16(st->ft[2*i],st->ft[2*i]);
        MULT_AC0(*pWindow, *pWindow);
        pWindow++;
        MADD_AC0(*pWindow, *pWindow);
        pWindow++;
        *pPS = EXTRV_RS_W(shift);
        pPS++;
    }

    filterbank_compute_bank32(st->bank, st->ps, st->ps + N);
}
#else

#ifdef _preprocess_analysis_OPT_NEON
static void preprocess_analysis(SpeexPreprocessState *st, spx_int16_t *x, int num_chan, int chan_index)
{
#ifdef _ARMV7_
    int16x4_t frame_16x4, window_16x4, tmp_16x4;
    int32x4_t tmp_32x4;
    spx_word16_t *p_frame = st->frame, *p_window = st->window, *p_ft = NULL, *p_ps = NULL;
#endif

    int i;
    int N = st->ps_size;              // frame_size
    int N3 = 2 * N - st->frame_size;  // frame_size
    int N4 = st->frame_size - N3;     // 0
    spx_word32_t *ps = st->ps;

#ifdef _ARMV7_
    int32x4_t shift_32x4;

    shift_32x4[0] = -15;
    shift_32x4[1] = -15;
    shift_32x4[2] = -15;
    shift_32x4[3] = -15;
#endif

    /* 'Build' input frame */
    for (i = 0; i < N3; i++)
        st->frame[i] = st->inbuf[i];
    for (i = 0; i < st->frame_size; i++)
        st->frame[N3 + i] = x[num_chan * i + chan_index];

    /* Update inbuf */
    for (i = 0; i < N3; i++)
        st->inbuf[i] = x[num_chan * (N4 + i) + chan_index];

        /* Windowing */
#if 0
    for (i=0;i<2*N;i++)
        st->frame[i] = MULT16_16_Q15(st->frame[i], st->window[i]);
#else
    for (i = 0; i < (N >> 2); i++) {
        frame_16x4 = vld1_s16((const int16_t *)p_frame);
        window_16x4 = vld1_s16((const int16_t *)p_window);

        tmp_32x4 = vmull_s16(frame_16x4, window_16x4);
        tmp_32x4 = vshlq_s32(tmp_32x4, shift_32x4);
        tmp_16x4 = vqmovn_s32(tmp_32x4);
        vst1_s16(p_frame, tmp_16x4);
        p_frame += 4;
        p_window += 4;

        frame_16x4 = vld1_s16((const int16_t *)p_frame);
        window_16x4 = vld1_s16((const int16_t *)p_window);

        tmp_32x4 = vmull_s16(frame_16x4, window_16x4);
        tmp_32x4 = vshlq_s32(tmp_32x4, shift_32x4);
        tmp_16x4 = vqmovn_s32(tmp_32x4);
        vst1_s16(p_frame, tmp_16x4);
        p_frame += 4;
        p_window += 4;
    }

#endif

#ifdef FIXED_POINT
    {
        spx_word16_t max_val = 0;
        for (i = 0; i < 2 * N; i++)
            max_val = MAX16(max_val, ABS16(st->frame[i]));
        st->frame_shift = 14 - spx_ilog2(EXTEND32(max_val));
        for (i = 0; i < 2 * N; i++)
            st->frame[i] = NTK_SHL16(st->frame[i], st->frame_shift);
    }
#endif

    /* Perform FFT */
    spx_fft(st->fft_lookup, st->frame, st->ft);

    /* Power spectrum */
    ps[0] = MULT16_16(st->ft[0], st->ft[0]);

#if 1
    for (i = 1; i < N; i++)
        ps[i] = ADD32(MULT16_16(st->ft[2 * i - 1], st->ft[2 * i - 1]), MULT16_16(st->ft[2 * i], st->ft[2 * i]));
#else
    p_ft = &(st->ft[1]);
    p_ps = &ps[1];

    for (i = 1; i < (N >> 2); i++) {
        frame_16x4 = vld1_s16((const int16_t *)p_ft);
        tmp_32x4 = vmull_s16(frame_16x4, frame_16x4);
        p_ft += 4;

        ps[4 * i - 3] = tmp_32x4[0] + tmp_32x4[1];
        ps[4 * i - 2] = tmp_32x4[2] + tmp_32x4[3];

        frame_16x4 = vld1_s16((const int16_t *)p_ft);
        tmp_32x4 = vmull_s16(frame_16x4, frame_16x4);
        p_ft += 4;

        ps[4 * i - 1] = tmp_32x4[0] + tmp_32x4[1];
        ps[4 * i] = tmp_32x4[2] + tmp_32x4[3];
    }
    ps[N - 3] = ADD32(MULT16_16(st->ft[2 * (N - 3) - 1], st->ft[2 * (N - 3) - 1]), MULT16_16(st->ft[2 * (N - 3)], st->ft[2 * (N - 3)]));
    ps[N - 2] = ADD32(MULT16_16(st->ft[2 * (N - 2) - 1], st->ft[2 * (N - 2) - 1]), MULT16_16(st->ft[2 * (N - 2)], st->ft[2 * (N - 2)]));
    ps[N - 1] = ADD32(MULT16_16(st->ft[2 * (N - 1) - 1], st->ft[2 * (N - 1) - 1]), MULT16_16(st->ft[2 * (N - 1)], st->ft[2 * (N - 1)]));
#endif

    for (i = 0; i < N; i++)
        st->ps[i] = PSHR32(st->ps[i], 2 * st->frame_shift);

    filterbank_compute_bank32(st->bank, ps, ps + N);
}
#else
static void preprocess_analysis(SpeexPreprocessState *st, spx_int16_t *x, int num_chan, int chan_index)
{
    int i;
    int N = st->ps_size;              // frame_size
    int N3 = 2 * N - st->frame_size;  // frame_size
    int N4 = st->frame_size - N3;     // 0
    spx_word32_t *ps = st->ps;

    /* 'Build' input frame */
    for (i = 0; i < N3; i++)
        st->frame[i] = st->inbuf[i];
    for (i = 0; i < st->frame_size; i++)
        st->frame[N3 + i] = x[num_chan * i + chan_index];

    /* Update inbuf */
    for (i = 0; i < N3; i++)
        st->inbuf[i] = x[num_chan * (N4 + i) + chan_index];

    /* Windowing */
    for (i = 0; i < 2 * N; i++)
        st->frame[i] = MULT16_16_Q15(st->frame[i], st->window[i]);

#ifdef FIXED_POINT
    {
        spx_word16_t max_val = 0;
        for (i = 0; i < 2 * N; i++)
            max_val = MAX16(max_val, ABS16(st->frame[i]));
        st->frame_shift = 14 - spx_ilog2(EXTEND32(max_val));
        for (i = 0; i < 2 * N; i++)
            st->frame[i] = NTK_SHL16(st->frame[i], st->frame_shift);
    }
#endif

    /* Perform FFT */
    spx_fft(st->fft_lookup, st->frame, st->ft);

    /* Power spectrum */
    ps[0] = MULT16_16(st->ft[0], st->ft[0]);
    for (i = 1; i < N; i++)
        ps[i] = ADD32(MULT16_16(st->ft[2 * i - 1], st->ft[2 * i - 1]), MULT16_16(st->ft[2 * i], st->ft[2 * i]));
    for (i = 0; i < N; i++)
        st->ps[i] = PSHR32(st->ps[i], 2 * st->frame_shift);

    filterbank_compute_bank32(st->bank, ps, ps + N);
}

#endif
#endif

#if (defined(_MIPS_update_noise_prob_OPT_NOT_Exactly) || defined(_MIPS_update_noise_prob_OPT))
static void update_noise_prob(SpeexPreprocessState *st)
{
    int i, const08, const005, const01, const02, const04;
    int min_range;
    int N = st->ps_size;
    int *ptr_S, *ptr_ps, *ptr_Smin, *ptr_prob;
    const005 = QCONST16(.05f, 15);
    const01 = QCONST16(.1f, 15);
    const02 = QCONST16(.2f, 15);
    const08 = QCONST16(.8f, 15);
    const04 = QCONST16(.4f, 15);
    ptr_S = st->S + 1;
    ptr_ps = st->ps;
    ptr_Smin = st->Smin;
    ptr_prob = st->update_prob;

    for (i = 1; i < N - 1; i++) {
#ifdef _MIPS_update_noise_prob_OPT_NOT_Exactly
        MULT_AC0(const08, *ptr_S);
        MADD_AC0(const005, ptr_ps[0]);
        MADD_AC0(const01, ptr_ps[1]);
        MADD_AC0(const005, ptr_ps[2]);
        *ptr_S++ = Shift15_Round_AC0();
        ptr_ps++;
#else
        *ptr_S = NTK_MULT16_32_Q15(const08, *ptr_S) + NTK_MULT16_32_Q15(const005, ptr_ps[0]) + NTK_MULT16_32_Q15(const01, ptr_ps[1]) + NTK_MULT16_32_Q15(const005, ptr_ps[2]);
        ptr_S++;
        ptr_ps++;
#endif
    }
    st->S[0] = (NTK_MULT16_32_Q15(const08, st->S[0]) + NTK_MULT16_32_Q15(const02, st->ps[0]));
    st->S[N - 1] = (NTK_MULT16_32_Q15(const08, st->S[N - 1]) + NTK_MULT16_32_Q15(const02, st->ps[N - 1]));

    if (st->nb_adapt == 1) {
        memset(st->Smin, 0, N << 2);
        memset(st->Stmp, 0, N << 2);
    }

    if (st->nb_adapt < 100)
        min_range = 15;
    else if (st->nb_adapt < 1000)
        min_range = 50;
    else if (st->nb_adapt < 10000)
        min_range = 150;
    else
        min_range = 300;

    ptr_S = st->S;
    ptr_ps = st->Stmp;
    memset(ptr_prob, 0, N << 2);
    if (st->min_count > min_range) {
        st->min_count = 0;
        for (i = 0; i < N; i++) {
            *ptr_Smin = MIN32(*ptr_ps, *ptr_S);
            *ptr_ps = *ptr_S;
            if (NTK_MULT16_32_Q15(const04, *ptr_S) > *ptr_Smin)
                *ptr_prob = 1;
            ptr_Smin++;
            ptr_ps++;
            ptr_S++;
            ptr_prob++;
        }
    } else {
        for (i = 0; i < N; i++) {
            *ptr_Smin = MIN32(*ptr_Smin, *ptr_S);
            *ptr_ps = MIN32(*ptr_ps, *ptr_S);
            if (NTK_MULT16_32_Q15(const04, *ptr_S) > *ptr_Smin)
                *ptr_prob = 1;
            ptr_Smin++;
            ptr_ps++;
            ptr_S++;
            ptr_prob++;
        }
    }
}
#else
static void update_noise_prob(SpeexPreprocessState *st)
{
    int i;
    int min_range;
    int N = st->ps_size;

    for (i = 1; i < N - 1; i++)
        st->S[i] = ADD32(ADD32(NTK_MULT16_32_Q15(QCONST16(.8f, 15), st->S[i]), NTK_MULT16_32_Q15(QCONST16(.05f, 15), st->ps[i - 1])), ADD32(NTK_MULT16_32_Q15(QCONST16(.1f, 15), st->ps[i]), NTK_MULT16_32_Q15(QCONST16(.05f, 15), st->ps[i + 1])));
    st->S[0] = ADD32(NTK_MULT16_32_Q15(QCONST16(.8f, 15), st->S[0]), NTK_MULT16_32_Q15(QCONST16(.2f, 15), st->ps[0]));
    st->S[N - 1] = ADD32(NTK_MULT16_32_Q15(QCONST16(.8f, 15), st->S[N - 1]), NTK_MULT16_32_Q15(QCONST16(.2f, 15), st->ps[N - 1]));

    if (st->nb_adapt == 1) {
        for (i = 0; i < N; i++)
            st->Smin[i] = st->Stmp[i] = 0;
    }

    if (st->nb_adapt < 100)
        min_range = 15;
    else if (st->nb_adapt < 1000)
        min_range = 50;
    else if (st->nb_adapt < 10000)
        min_range = 150;
    else
        min_range = 300;
    if (st->min_count > min_range) {
        st->min_count = 0;
        for (i = 0; i < N; i++) {
            st->Smin[i] = MIN32(st->Stmp[i], st->S[i]);
            st->Stmp[i] = st->S[i];
        }
    } else {
        for (i = 0; i < N; i++) {
            st->Smin[i] = MIN32(st->Smin[i], st->S[i]);
            st->Stmp[i] = MIN32(st->Stmp[i], st->S[i]);
        }
    }
    for (i = 0; i < N; i++) {
        if (NTK_MULT16_32_Q15(QCONST16(.4f, 15), st->S[i]) > st->Smin[i])
            st->update_prob[i] = 1;
        else
            st->update_prob[i] = 0;
        /*fprintf (stderr, "%f ", st->S[i]/st->Smin[i]);*/
        /*fprintf (stderr, "%f ", st->update_prob[i]);*/
    }
}
#endif

#define NOISE_OVERCOMPENS 1.

void speex_echo_get_residual(SpeexEchoState *st, spx_word32_t *Yout, int len, int chan_index);

#if 0
EXPORT int speex_preprocess(SpeexPreprocessState *st, spx_int16_t *x, spx_int32_t *echo)
{
    return speex_preprocess_run(st, x);
}
#endif
#ifdef _MIPS_speex_preprocess_run_OPT
#ifndef ADD_NLP
EXPORT int speex_preprocess_run(void *state, spx_int16_t *x, int num_chan, int chan_index)
#else
EXPORT int speex_preprocess_run(void *state, spx_int16_t *near, spx_int16_t *far, spx_int16_t *x, int num_chan, int chan_index)
#endif
{
    SpeexPreprocessState *st = (SpeexPreprocessState *)state;
    int i;
    int M;
    int N = st->ps_size;  // = st->frame_size;
    // int N3 = 2*N - st->frame_size;    // = st->frame_size;
    // int N4 = st->frame_size - N3; // = 0;
    spx_word32_t *ps = st->ps;
    spx_word32_t Zframe;
    spx_word16_t Pframe;
    spx_word16_t effective_echo_suppress;

    if (st->nb_adapt < 20000)
        st->nb_adapt++;
    st->min_count++;

    M = st->nbands;
    /* Deal with residual echo if provided */
    if (st->echo_state) {
        int *ptrA, *ptrB;
        s16 echo_noise_ratio = st->echo_noise_ratio;
        speex_echo_get_residual(st->echo_state, st->residual_echo, N, chan_index);
#ifndef FIXED_POINT
        /* If there are NaNs or ridiculous values, it'll show up in the DC and we just reset everything to zero */
        if (!(st->residual_echo[0] >= 0 && st->residual_echo[0] < N * 1e9f)) {
            for (i = 0; i < N; i++)
                st->residual_echo[i] = 0;
        }
#endif

        ptrA = st->echo_noise;
        ptrB = st->residual_echo;
#ifdef _MIPS_speex_preprocess_run_OPT_NOT_Exactly
        for (i = 0; i < (N >> 1); i++) {
            MULT_AC0(echo_noise_ratio, *ptrA);
            *ptrA++ = MAX32(Shift15_Round_AC0(), *ptrB);
            ptrB++;
            MULT_AC0(echo_noise_ratio, *ptrA);
            *ptrA++ = MAX32(Shift15_Round_AC0(), *ptrB);
            ptrB++;
        }
#else
        for (i = 0; i < (N >> 1); i++) {
            MULT_AC0(echo_noise_ratio, *ptrA);
            SHILO15_AC0();
            *ptrA++ = MAX32(MFLO_AC0(), *ptrB);
            ptrB++;
            MULT_AC0(echo_noise_ratio, *ptrA);
            SHILO15_AC0();
            *ptrA++ = MAX32(MFLO_AC0(), *ptrB);
            ptrB++;
            // st->echo_noise[i] = MAX32(NTK_MULT16_32_Q15(st->echo_noise_ratio,st->echo_noise[i]), st->residual_echo[i]);
        }
#endif
        filterbank_compute_bank32(st->bank, st->echo_noise, st->echo_noise + N);
    } else {
        memset(st->echo_noise, 0, (N + M) << 2);
    }

    preprocess_analysis(st, x, num_chan, chan_index);

    update_noise_prob(st);

    /* Update the noise estimate for the frequencies where it can be */
    {
        spx_word16_t beta, beta_1;
        int *ptr_upprob = st->update_prob, *pps = st->ps, *ptr_n = st->noise;
        beta = MAX16(QCONST16(.03, 15), DIV32_16(Q15_ONE, st->nb_adapt));
        beta_1 = Q15_ONE - beta;
        for (i = 0; i < N; i++) {
            if (!(*ptr_upprob) || (*pps) < PSHR32(*ptr_n, NOISE_SHIFT)) {
#ifdef _MIPS_speex_preprocess_run_OPT_NOT_Exactly
                MULT_AC0(beta_1, *ptr_n);
                MADD_AC0(beta, NTK_SHL32(*pps, NOISE_SHIFT));
                *ptr_n = MAX32(EXTEND32(0), Shift15_Round_AC0());
#else
                *ptr_n = MAX32(EXTEND32(0), ADD32(NTK_MULT16_32_Q15(beta_1, *ptr_n), NTK_MULT16_32_Q15(beta, NTK_SHL32(*pps, NOISE_SHIFT))));
#endif
            }
            ptr_upprob++;
            pps++;
            ptr_n++;
        }
    }
    filterbank_compute_bank32(st->bank, st->noise, st->noise + N);

    /* Special case for first frame */
    if (st->nb_adapt == 1)
        memcpy(st->old_ps, ps, (N + M) << 2);

    {
        s16 const1 = QCONST16(1.f, SNR_SHIFT);
        s16 const100 = QCONST16(100.f, SNR_SHIFT);
        s16 const089 = QCONST16(.89f, 15);
        s16 s16const01 = QCONST16(.1f, 15);
        s16 *ptr_post = st->post, *ptr_prio = st->prior;
        s32 *ptr_ops = st->old_ps;
        s32 *ptr_n = st->noise;
        s32 *ptr_en = st->echo_noise;
        s32 *pps = st->ps;
        /* Compute a posteriori SNR */
        for (i = 0; i < N + M; i++) {
            spx_word16_t gamma;

            /* Total noise estimate including residual echo and reverberation */
            spx_word32_t tot_noise = ADD32((EXTEND32(1) + PSHR32(*ptr_n, NOISE_SHIFT)), *ptr_en);
            ptr_n++;
            ptr_en++;

            /* A posteriori SNR = ps/noise - 1*/
            *ptr_post = DIV32_16_Q8(*pps, tot_noise) - const1;
            pps++;
            *ptr_post = MIN16(*ptr_post, const100);

            /* Computing update gamma = .1 + .9*(old/(old+noise))^2 */
            gamma = s16const01 + MULT16_16_Q15(const089, SQR16_Q15(DIV32_16_Q15(*ptr_ops, ADD32(*ptr_ops, tot_noise))));

            /* A priori SNR update = gamma*max(0,post) + (1-gamma)*old/noise */
#if 0
            MULT_AC0(gamma,MAX16(0,*ptr_post));
            MADD_AC0(Q15_ONE-gamma,DIV32_16_Q8(*ptr_ops,tot_noise));
            *ptr_prio = (s16)Shift15_Round_AC0();
#else
            *ptr_prio = EXTRACT16(PSHR32((MULT16_16(gamma, MAX16(0, *ptr_post)) + MULT16_16(Q15_ONE - gamma, DIV32_16_Q8(*ptr_ops, tot_noise))), 15));
#endif
            *ptr_prio = MIN16(*ptr_prio, const100);
            ptr_post++;
            ptr_prio++;
            ptr_ops++;
        }
    }

#ifdef _DISABLE_PRIOR_SMOOTH
    {
        s32 *ps32zeta = (s32 *)st->zeta;
        s32 *ps32prior = (s32 *)st->prior;
        for (i = 0; i < ((N + M) >> 1); i++) {
            MIPS_ADDQ_S_PH(*ps32zeta, SHRAV_R_PH(*ps32zeta, 1), SHRAV_R_PH(*ps32zeta, 2));
            MIPS_ADDQ_S_PH(*ps32zeta, *ps32zeta, SHRAV_R_PH(*ps32prior, 2));
            ps32zeta++;
            ps32prior++;
        }
    }
    // memcpy(st->zeta, st->prior, (N+M)<<1);
#else
    {
        s16 const07 = QCONST16(.7f, 15);
        s16 const03 = QCONST16(.3f, 15);
        s16 const015 = QCONST16(.15f, 15);
        s16 const0075 = QCONST16(.075f, 15);
        s16 *pZeta = st->zeta, *pPrior = st->prior;
        /* Recursive average of the a priori SNR. A bit smoothed for the psd components */
        MULT_AC0(const07, *pZeta);
        MADD_AC0(const03, *pPrior);
        *pZeta++ = Shift15_Round_AC0();
        //*pZeta = PSHR32(ADD32(MULT16_16(const07,*pZeta), MULT16_16(const03,*pPrior)),15);
        for (i = 1; i < N - 1; i++) {
            MULT_AC0(const07, *pZeta);
            MADD_AC0(const015, pPrior[1]);
            MADD_AC0(const0075, pPrior[0]);
            MADD_AC0(const0075, pPrior[2]);
            *pZeta++ = Shift15_Round_AC0();
            // *pZeta++ = PSHR32(ADD32(ADD32(ADD32(MULT16_16(const07,*pZeta), MULT16_16(const015,pPrior[1])),
            //                      MULT16_16(const0075,pPrior[0])), MULT16_16(const0075,pPrior[2])),15);
            pPrior++;
        }
        pPrior++;
        for (i = N - 1; i < N + M; i++) {
            MULT_AC0(const07, *pZeta);
            MADD_AC0(const03, *pPrior);
            *pZeta++ = Shift15_Round_AC0();
            //*pZeta++ = PSHR32(ADD32(MULT16_16(const07,*pZeta), MULT16_16(const03,*pPrior)),15);
            pPrior++;
        }
    }
#endif

    /* Speech probability of presence for the entire frame is based on the average filterbank a priori SNR */
    {
        s16 *p_zeta = st->zeta + N;
        Zframe = 0;
        for (i = 0; i < M; i++) {
            Zframe = ADD32(Zframe, EXTEND32(*p_zeta));
            p_zeta++;
        }
    }
    Pframe = QCONST16(.1f, 15) + MULT16_16_Q15(QCONST16(.899f, 15), qcurve(DIV32_16(Zframe, M)));

    // effective_echo_suppress = EXTRACT16(PSHR32(ADD32(MULT16_16(SUB16(Q15_ONE,Pframe), st->echo_suppress), MULT16_16(Pframe, st->echo_suppress_active)),15));
    effective_echo_suppress = MULT16_16_P15(Q15_ONE - Pframe, (s16)st->echo_suppress) + MULT16_16_P15(Pframe, (s16)st->echo_suppress_active);

    if (st->echo_state)
        compute_gain_floor(st->noise_suppress, effective_echo_suppress, st->noise + N, st->echo_noise + N, st->gain_floor + N, M);
    else
        compute_gain_floor_NS(st->noise_suppress, 0 /*effective_echo_suppress*/, st->noise + N, 0 /*st->echo_noise+N*/, st->gain_floor + N, M);

    {
        s16 *ptr_prio = st->prior + N, *ptr_post = st->post + N, *ptr_gain = st->gain + N, *ptr_gain2 = st->gain2 + N, *ptr_zeta = st->zeta + N;
        s32 *ptr_ops = st->old_ps + N, *pps = st->ps + N;
        s16 const0119 = QCONST16(.199f, 15);
        s16 const3 = QCONST16(3., SNR_SHIFT);
        s16 s16const08 = QCONST16(.8f, 15);
        s16 s16const02 = QCONST16(.2f, 15);
        s32 Shift = EXPIN_SHIFT - SNR_SHIFT;
        /* Compute Ephraim & Malah gain speech probability of presence for each critical band (Bark scale)
        Technically this is actually wrong because the EM gaim assumes a slightly different probability
        distribution */
        for (i = 0; i < M; i++) {
            /* See EM and Cohen papers*/
            spx_word32_t theta;
            /* Gain from hypergeometric function */
            spx_word32_t MM;
            /* Weiner filter gain */
            spx_word16_t prior_ratio;
            /* a priority probability of speech presence based on Bark sub-band alone */
            spx_word16_t P1;
            /* Speech absence a priori probability (considering sub-band and frame) */
            spx_word16_t q;
#ifdef FIXED_POINT
            spx_word16_t tmp;
#endif

            prior_ratio = PDIV32_16(NTK_SHL32(EXTEND32(*ptr_prio), 15), ADD16(*ptr_prio, NTK_SHL16(1, SNR_SHIFT)));
            theta = NTK_MULT16_32_P15(prior_ratio, QCONST32(1.f, EXPIN_SHIFT) + NTK_SHL32(EXTEND32(*ptr_post), Shift));

            MM = hypergeom_gain(theta);
            /* Gain with bound */
            *ptr_gain = EXTRACT16(NTK_MULT16_32_Q15(prior_ratio, MM));

            /* Save old Bark power spectrum */
#ifdef _MIPS_speex_preprocess_run_OPT_NOT_Exactly
            MULT_AC0(s16const02, *ptr_ops);
            MADD_AC0(MULT16_16_P15(s16const08, SQR16_Q15(*ptr_gain)), *pps);
            *ptr_ops = Shift15_Round_AC0();
#else
            *ptr_ops = NTK_MULT16_32_P15(s16const02, *ptr_ops) + NTK_MULT16_32_P15(MULT16_16_P15(s16const08, SQR16_Q15(*ptr_gain)), *pps);
#endif

            P1 = const0119 + MULT16_16_Q15(s16const08, qcurve(*ptr_zeta));
            q = MULT16_16_Q15(Pframe, P1);
#ifdef FIXED_POINT
            // theta = MIN32(theta, EXTEND32(32767));
            /*Q8*/ tmp = MULT16_16_Q15((NTK_SHL16(1, SNR_SHIFT) + (*ptr_prio)), EXTRACT16(SHR32(spx_exp(-EXTRACT16(theta)), 1)));
            tmp = MIN16(const3, tmp); /* Prevent overflows in the next line*/
            /*Q8*/ tmp = EXTRACT16(PSHR32(MULT16_16(PDIV32_16(NTK_SHL32(EXTEND32(Q15_ONE - q), 8), q), tmp), 8));
            *ptr_gain2 = DIV32_16(NTK_SHL32(EXTEND32(32767), SNR_SHIFT), ADD16(256, tmp));
#else
            *ptr_gain2 = 1 / (1.f + (q / (1.f - q)) * (1 + (*ptr_prio)) * exp(-theta));
#endif
            ptr_prio++;
            ptr_post++;
            ptr_gain++;
            ptr_gain2++;
            ptr_ops++;
            pps++;
            ptr_zeta++;
        }
    }

    /* Use 1 for linear gain resolution (best) or 0 for Bark gain resolution (faster) */
    if (st->bank_scale) {
        s16 *pGain = st->gain, *pGain2 = st->gain2, *p_prio = st->prior, *p_post = st->post, *p_gainfloor = st->gain_floor;
        s32 *p_ops = st->old_ps, *pps = st->ps;
        s16 s16const0333 = QCONST16(.333f, 15);
        s16 s16const08 = QCONST16(.8f, 15);
        s16 s16const02 = QCONST16(.2f, 15);
        int Shift = EXPIN_SHIFT - SNR_SHIFT;

        /* Convert the EM gains and speech prob to linear frequency */
        filterbank_compute_psd16(st->bank, st->gain2 + N, st->gain2);
        filterbank_compute_psd16(st->bank, st->gain + N, st->gain);
        filterbank_compute_psd16(st->bank, p_gainfloor + N, p_gainfloor);

        /* Compute gain according to the Ephraim-Malah algorithm -- linear frequency */
        for (i = 0; i < N; i++) {
            spx_word32_t MM;
            spx_word32_t theta;
            spx_word16_t prior_ratio;
            spx_word16_t tmp;
            spx_word16_t p;
            spx_word16_t g;

            /* Wiener filter gain */
            prior_ratio = PDIV32_16(NTK_SHL32(EXTEND32(*p_prio), 15), ADD16(*p_prio, NTK_SHL16(1, SNR_SHIFT)));
            theta = NTK_MULT16_32_P15(prior_ratio, QCONST32(1.f, EXPIN_SHIFT) + NTK_SHL32(EXTEND32(*p_post), Shift));

            /* Optimal estimator for loudness domain */
            MM = hypergeom_gain(theta);
            /* EM gain with bound */
            g = EXTRACT16(NTK_MULT16_32_Q15(prior_ratio, MM));
            /* Interpolated speech probability of presence */
            p = *pGain2;

            /* Constrain the gain to be close to the Bark scale gain */
            if (MULT16_16_Q15(s16const0333, g) > *pGain)
                g = MULT16_16(3, *pGain);
            *pGain = g;

            /* Save old power spectrum */
#ifdef _MIPS_speex_preprocess_run_OPT_NOT_Exactly
            MULT_AC0(s16const02, *p_ops);
            MADD_AC0(MULT16_16_P15(s16const08, SQR16_Q15(*pGain)), *pps);
            *p_ops = Shift15_Round_AC0();
#else
            *p_ops = NTK_MULT16_32_P15(s16const02, *p_ops) + NTK_MULT16_32_P15(MULT16_16_P15(s16const08, SQR16_Q15(*pGain)), *pps);
#endif

            /* Apply gain floor */
            if (*pGain < *p_gainfloor)
                *pGain = *p_gainfloor;

                /* Exponential decay model for reverberation (unused) */
                /*st->reverb_estimate[i] = st->reverb_decay*st->reverb_estimate[i] + st->reverb_decay*st->reverb_level*st->gain[i]*st->gain[i]*st->ps[i];*/

                /* Take into account speech probability of presence (loudness domain MMSE estimator) */
                /* gain2 = [p*sqrt(gain)+(1-p)*sqrt(gain _floor) ]^2 */
#ifdef _MIPS_speex_preprocess_run_OPT_NOT_Exactly
            MULT_AC0(p, spx_sqrt(NTK_SHL32(EXTEND32(*pGain), 15)));
            MADD_AC0(Q15_ONE - p, spx_sqrt(NTK_SHL32(EXTEND32(*p_gainfloor), 15)));
            tmp = Shift15_Round_AC0();
#else
            tmp = MULT16_16_P15(p, spx_sqrt(NTK_SHL32(EXTEND32(*pGain), 15))) + MULT16_16_P15(Q15_ONE - p, spx_sqrt(NTK_SHL32(EXTEND32(*p_gainfloor), 15)));
#endif
            *pGain2 = SQR16_Q15(tmp);

            /* Use this if you want a log-domain MMSE estimator instead */
            /*st->gain2[i] = pow(st->gain[i], p) * pow(st->gain_floor[i],1.f-p);*/

            pGain++;
            pGain2++;
            p_prio++;
            p_post++;
            p_ops++;
            pps++;
            p_gainfloor++;
        }
    } else {
        s16 *p_gain = st->gain + N;
        s16 *p_gain2 = st->gain2 + N;
        s16 *p_gainf = st->gain_floor + N;
        for (i = 0; i < M; i++)  //(i=N;i<N+M;i++)
        {
            spx_word16_t tmp;
            spx_word16_t p = *p_gain2;
#ifdef _Bark_scale_OPT
            if (*p_gainf < *p_gain) {
                tmp = MULT16_16_P15(p, spx_sqrt(NTK_SHL32(EXTEND32(*p_gain), 15))) + MULT16_16_P15(Q15_ONE - p, spx_sqrt(NTK_SHL32(EXTEND32(*p_gainf), 15)));
            } else
                tmp = spx_sqrt(NTK_SHL32(EXTEND32(*p_gainf), 15));
#else
            *p_gain = MAX16(*p_gain, *p_gainf);
            tmp = MULT16_16_P15(p, spx_sqrt(NTK_SHL32(EXTEND32(*p_gain), 15))) + MULT16_16_P15(Q15_ONE - p, spx_sqrt(NTK_SHL32(EXTEND32(*p_gainf), 15)));
#endif
            *p_gain2 = SQR16_Q15(tmp);
            p_gain++;
            p_gain2++;
            p_gainf++;
        }
        filterbank_compute_psd16(st->bank, st->gain2 + N, st->gain2);
    }

#ifdef ADD_NLP
    if (stNlp.NlpEnable && near && far) {
        // s16 s16NLPRatio;
        // s16 s16NLPR_INV = SUB16(Q15_ONE, stNlp.NlpEffect);
        s16 *ps16Gain = st->gain2;
        u16 *pu16NlpGain;
        nlp_preprocess(&stNlp, near, far, x);
        pu16NlpGain = stNlp.nlp_gain;
#if 1  // new method
        for (i = 0; i < N; i++) {
            if (*ps16Gain > *pu16NlpGain) {
                *ps16Gain = ADD16(MULT16_16_P15(SUB16(Q15_ONE, *ps16Gain), *pu16NlpGain), MULT16_16_P15(*ps16Gain, *ps16Gain));
            }
            ps16Gain++;
            pu16NlpGain++;
        }
#else
        for (i = 0; i < N; i++) {
            MULT_AC0(s16NLPR_INV, *ps16Gain);
            s16NLPRatio = (s16)EXTRV_RS_W(15);
            MADD_AC0(SUB16(Q15_ONE, s16NLPRatio), *pu16NlpGain);
            *ps16Gain = MULT16_16_P15(*ps16Gain, EXTRV_RS_W(15));
            ps16Gain++;
            pu16NlpGain++;
        }
#endif
    }
#endif

    /* If noise suppression is off, don't apply the gain (but then why call this in the first place!) */
    // if (!st->denoise_enabled)
    //     memset(st->gain2, Q15_ONE, (N+M)<<1);

    {
        /* Apply computed gain */
        s16 *pFT = st->ft, *pG2 = st->gain2;
        *pFT = MULT16_16_P15(*pG2, *pFT);
        pFT++;
        pG2++;
        for (i = 1; i < N; i++) {
            *pFT = MULT16_16_P15(*pG2, *pFT);
            pFT++;
            *pFT = MULT16_16_P15(*pG2, *pFT);
            pFT++;
            pG2++;
        }
        pG2--;
        *pFT = MULT16_16_P15(*pG2, *pFT);
    }

    /*FIXME: This *will* not work for fixed-point */
#ifndef FIXED_POINT
    if (st->agc_enabled)
        speex_compute_agc(st, Pframe, st->ft);
#endif

    /* Inverse FFT with 1/N scaling */
    spx_ifft(st->fft_lookup, st->ft, st->frame);
    /* Scale back to original (lower) amplitude */
    {
        s32 *ps32Frame = (s32 *)st->frame;
        for (i = 0; i < N; i++) {
            // st->frame[i] = NTK_PSHR16(st->frame[i], st->frame_shift);
            *ps32Frame = SHRAV_R_PH(*ps32Frame, st->frame_shift);
            ps32Frame++;
        }
    }

    /*FIXME: This *will* not work for fixed-point */
#ifndef FIXED_POINT
    if (st->agc_enabled) {
        float max_sample = 0;
        for (i = 0; i < 2 * N; i++)
            if (fabs(st->frame[i]) > max_sample)
                max_sample = fabs(st->frame[i]);
        if (max_sample > 28000.f) {
            float damp = 28000.f / max_sample;
            for (i = 0; i < 2 * N; i++)
                st->frame[i] *= damp;
        }
    }
#endif

#ifdef _MIPS_speex_preprocess_run_OPT_NOT_Exactly
    {
        s32 *ps32Frame = (s32 *)st->frame, *ps32Window = (s32 *)st->window;
        /* Synthesis window (for WOLA) */
        for (i = 0; i < N; i++) {
            *ps32Frame = MULT16_16_P15_PH(*ps32Frame, *ps32Window);
            ps32Frame++;
            ps32Window++;
            // st->frame[i] = MULT16_16_Q15(st->frame[i], st->window[i]);
        }
    }
#else
    for (i = 0; i < (N << 1); i++)
        st->frame[i] = MULT16_16_Q15(st->frame[i], st->window[i]);
#endif

    {
        s16 *px = x + chan_index;
        s16 *pOutbuf = st->outbuf;
        s16 *pFrame = st->frame;
        // s16 *pFrame2 = st->frame + st->frame_size;
        /* Perform overlap and add */
        for (i = 0; i < (N >> 1); i++) {
            *px = ADD16(*pOutbuf, *pFrame);
            //*pOutbuf = *pFrame2;  pFrame2++;
            px += num_chan;
            pFrame++;
            pOutbuf++;

            *px = ADD16(*pOutbuf, *pFrame);
            //*pOutbuf = *pFrame2;  pFrame2++;
            px += num_chan;
            pFrame++;
            pOutbuf++;
        }
        memcpy(st->outbuf, st->frame + st->frame_size, N << 1);
    }

#if 0  // useless
    /* FIXME: This VAD is a kludge */
    st->speech_prob = Pframe;
    if (st->vad_enabled)
    {
        if (st->speech_prob > st->speech_prob_start || (st->was_speech && st->speech_prob > st->speech_prob_continue))
        {
            st->was_speech=1;
            return 1;
        } else
        {
            st->was_speech=0;
            return 0;
        }
    } else {
        return 1;
    }
#endif
    return 0;
}

#else
#ifndef ADD_NLP
EXPORT int speex_preprocess_run(void *state, spx_int16_t *x, int num_chan, int chan_index)
#else
EXPORT int speex_preprocess_run(void *state, spx_int16_t *near, spx_int16_t *far, spx_int16_t *x, int num_chan, int chan_index)
#endif
{
    SpeexPreprocessState *st = (SpeexPreprocessState *)state;
    int i;
    int M;
    int N = st->ps_size;
    int N3 = 2 * N - st->frame_size;
    int N4 = st->frame_size - N3;
    spx_word32_t *ps = st->ps;
    spx_word32_t Zframe;
    spx_word16_t Pframe;
    spx_word16_t beta, beta_1;
    spx_word16_t effective_echo_suppress;

    st->nb_adapt++;
    if (st->nb_adapt > 20000)
        st->nb_adapt = 20000;
    st->min_count++;

    beta = MAX16(QCONST16(.03, 15), DIV32_16(Q15_ONE, st->nb_adapt));
    beta_1 = Q15_ONE - beta;
    M = st->nbands;
    /* Deal with residual echo if provided */
    if (st->echo_state) {
        speex_echo_get_residual(st->echo_state, st->residual_echo, N, chan_index);
#ifndef FIXED_POINT
        /* If there are NaNs or ridiculous values, it'll show up in the DC and we just reset everything to zero */
        if (!(st->residual_echo[0] >= 0 && st->residual_echo[0] < N * 1e9f)) {
            for (i = 0; i < N; i++)
                st->residual_echo[i] = 0;
        }
#endif
        for (i = 0; i < N; i++)
            st->echo_noise[i] = MAX32(NTK_MULT16_32_Q15(st->echo_noise_ratio, st->echo_noise[i]), st->residual_echo[i]);
        filterbank_compute_bank32(st->bank, st->echo_noise, st->echo_noise + N);
    } else {
        for (i = 0; i < N + M; i++)
            st->echo_noise[i] = 0;
    }
    preprocess_analysis(st, x, num_chan, chan_index);

    update_noise_prob(st);

    /* Noise estimation always updated for the 10 first frames */
    /*if (st->nb_adapt<10)
    {
    for (i=1;i<N-1;i++)
    st->update_prob[i] = 0;
    }
    */

    /* Update the noise estimate for the frequencies where it can be */
    for (i = 0; i < N; i++) {
        if (!st->update_prob[i] || st->ps[i] < PSHR32(st->noise[i], NOISE_SHIFT))
            st->noise[i] = MAX32(EXTEND32(0), ADD32(NTK_MULT16_32_Q15(beta_1, st->noise[i]), NTK_MULT16_32_Q15(beta, NTK_SHL32(st->ps[i], NOISE_SHIFT))));
    }
    filterbank_compute_bank32(st->bank, st->noise, st->noise + N);

    /* Special case for first frame */
    if (st->nb_adapt == 1)
        for (i = 0; i < N + M; i++)
            st->old_ps[i] = ps[i];

    /* Compute a posteriori SNR */
    for (i = 0; i < N + M; i++) {
        spx_word16_t gamma;

        /* Total noise estimate including residual echo and reverberation */
        spx_word32_t tot_noise = ADD32(ADD32(EXTEND32(1), PSHR32(st->noise[i], NOISE_SHIFT)), st->echo_noise[i]);

        /* A posteriori SNR = ps/noise - 1*/
        st->post[i] = SUB16(DIV32_16_Q8(ps[i], tot_noise), QCONST16(1.f, SNR_SHIFT));
        st->post[i] = MIN16(st->post[i], QCONST16(100.f, SNR_SHIFT));

        /* Computing update gamma = .1 + .9*(old/(old+noise))^2 */
        gamma = QCONST16(.1f, 15) + MULT16_16_Q15(QCONST16(.89f, 15), SQR16_Q15(DIV32_16_Q15(st->old_ps[i], ADD32(st->old_ps[i], tot_noise))));

        /* A priori SNR update = gamma*max(0,post) + (1-gamma)*old/noise */
        st->prior[i] = EXTRACT16(PSHR32(ADD32(MULT16_16(gamma, MAX16(0, st->post[i])), MULT16_16(Q15_ONE - gamma, DIV32_16_Q8(st->old_ps[i], tot_noise))), 15));
        st->prior[i] = MIN16(st->prior[i], QCONST16(100.f, SNR_SHIFT));
    }

    /*print_vec(st->post, N+M, "");*/

#ifdef _DISABLE_PRIOR_SMOOTH
    {
        s16 *ps32zeta = st->zeta;
        s16 *ps32prior = st->prior;
        for (i = 0; i < (N + M); i++) {
            *ps32zeta = (*ps32zeta >> 1) + (*ps32zeta >> 2) + (*ps32prior >> 2);
            ps32zeta++;
            ps32prior++;
        }
    }
    // memcpy(st->zeta, st->prior, (N+M)<<1);
#else
    /* Recursive average of the a priori SNR. A bit smoothed for the psd components */
    st->zeta[0] = PSHR32(ADD32(MULT16_16(QCONST16(.7f, 15), st->zeta[0]), MULT16_16(QCONST16(.3f, 15), st->prior[0])), 15);
    for (i = 1; i < N - 1; i++)
        st->zeta[i] = PSHR32(ADD32(ADD32(ADD32(MULT16_16(QCONST16(.7f, 15), st->zeta[i]), MULT16_16(QCONST16(.15f, 15), st->prior[i])),
                                         MULT16_16(QCONST16(.075f, 15), st->prior[i - 1])),
                                   MULT16_16(QCONST16(.075f, 15), st->prior[i + 1])),
                             15);
    for (i = N - 1; i < N + M; i++)
        st->zeta[i] = PSHR32(ADD32(MULT16_16(QCONST16(.7f, 15), st->zeta[i]), MULT16_16(QCONST16(.3f, 15), st->prior[i])), 15);
#endif

    /* Speech probability of presence for the entire frame is based on the average filterbank a priori SNR */
    Zframe = 0;
    for (i = N; i < N + M; i++)
        Zframe = ADD32(Zframe, EXTEND32(st->zeta[i]));
    Pframe = QCONST16(.1f, 15) + MULT16_16_Q15(QCONST16(.899f, 15), qcurve(DIV32_16(Zframe, st->nbands)));

    // effective_echo_suppress = EXTRACT16(PSHR32(ADD32(MULT16_16(SUB16(Q15_ONE,Pframe), st->echo_suppress), MULT16_16(Pframe, st->echo_suppress_active)),15));
    effective_echo_suppress = ADD16(MULT16_16_P15(SUB16(Q15_ONE, Pframe), EXTRACT16(st->echo_suppress)), MULT16_16_P15(Pframe, EXTRACT16(st->echo_suppress_active)));

    if (st->echo_state)
        compute_gain_floor(st->noise_suppress, effective_echo_suppress, st->noise + N, st->echo_noise + N, st->gain_floor + N, M);
    else
        compute_gain_floor_NS(st->noise_suppress, 0 /*effective_echo_suppress*/, st->noise + N, 0 /*st->echo_noise+N*/, st->gain_floor + N, M);

    /* Compute Ephraim & Malah gain speech probability of presence for each critical band (Bark scale)
    Technically this is actually wrong because the EM gaim assumes a slightly different probability
    distribution */
    for (i = N; i < N + M; i++) {
        /* See EM and Cohen papers*/
        spx_word32_t theta;
        /* Gain from hypergeometric function */
        spx_word32_t MM;
        /* Weiner filter gain */
        spx_word16_t prior_ratio;
        /* a priority probability of speech presence based on Bark sub-band alone */
        spx_word16_t P1;
        /* Speech absence a priori probability (considering sub-band and frame) */
        spx_word16_t q;
#ifdef FIXED_POINT
        spx_word16_t tmp;
#endif

        prior_ratio = PDIV32_16(NTK_SHL32(EXTEND32(st->prior[i]), 15), ADD16(st->prior[i], NTK_SHL16(1, SNR_SHIFT)));
        theta = NTK_MULT16_32_P15(prior_ratio, QCONST32(1.f, EXPIN_SHIFT) + NTK_SHL32(EXTEND32(st->post[i]), EXPIN_SHIFT - SNR_SHIFT));

        MM = hypergeom_gain(theta);
        /* Gain with bound */
        st->gain[i] = EXTRACT16(NTK_MULT16_32_Q15(prior_ratio, MM));
        /* Save old Bark power spectrum */
        st->old_ps[i] = NTK_MULT16_32_P15(QCONST16(.2f, 15), st->old_ps[i]) + NTK_MULT16_32_P15(MULT16_16_P15(QCONST16(.8f, 15), SQR16_Q15(st->gain[i])), ps[i]);

        P1 = QCONST16(.199f, 15) + MULT16_16_Q15(QCONST16(.8f, 15), qcurve(st->zeta[i]));
        q = MULT16_16_Q15(Pframe, P1);
#ifdef FIXED_POINT
        // theta = MIN32(theta, EXTEND32(32767));
        /*Q8*/ tmp = MULT16_16_Q15((NTK_SHL32(1, SNR_SHIFT) + st->prior[i]), EXTRACT16(MIN32(Q15ONE, SHR32(spx_exp(-EXTRACT16(theta)), 1))));
        tmp = MIN16(QCONST16(3., SNR_SHIFT), tmp); /* Prevent overflows in the next line*/
        /*Q8*/ tmp = EXTRACT16(PSHR32(MULT16_16(PDIV32_16(NTK_SHL32(EXTEND32(Q15_ONE - q), 8), q), tmp), 8));
        st->gain2[i] = DIV32_16(NTK_SHL32(EXTEND32(32767), SNR_SHIFT), ADD16(256, tmp));
#else
        st->gain2[i] = 1 / (1.f + (q / (1.f - q)) * (1 + st->prior[i]) * exp(-theta));
#endif
    }

    /* Use 1 for linear gain resolution (best) or 0 for Bark gain resolution (faster) */
    if (st->bank_scale) {
        /* Convert the EM gains and speech prob to linear frequency */
        filterbank_compute_psd16(st->bank, st->gain2 + N, st->gain2);
        filterbank_compute_psd16(st->bank, st->gain + N, st->gain);
        filterbank_compute_psd16(st->bank, st->gain_floor + N, st->gain_floor);

        /* Compute gain according to the Ephraim-Malah algorithm -- linear frequency */
        for (i = 0; i < N; i++) {
            spx_word32_t MM;
            spx_word32_t theta;
            spx_word16_t prior_ratio;
            spx_word16_t tmp;
            spx_word16_t p;
            spx_word16_t g;

            /* Wiener filter gain */
            prior_ratio = PDIV32_16(NTK_SHL32(EXTEND32(st->prior[i]), 15), ADD16(st->prior[i], NTK_SHL16(1, SNR_SHIFT)));
            theta = NTK_MULT16_32_P15(prior_ratio, QCONST32(1.f, EXPIN_SHIFT) + NTK_SHL32(EXTEND32(st->post[i]), EXPIN_SHIFT - SNR_SHIFT));

            /* Optimal estimator for loudness domain */
            MM = hypergeom_gain(theta);
            /* EM gain with bound */
            g = EXTRACT16(NTK_MULT16_32_Q15(prior_ratio, MM));
            /* Interpolated speech probability of presence */
            p = st->gain2[i];

            /* Constrain the gain to be close to the Bark scale gain */
            if (MULT16_16_Q15(QCONST16(.333f, 15), g) > st->gain[i])
                g = MULT16_16(3, st->gain[i]);
            st->gain[i] = g;

            /* Save old power spectrum */
            st->old_ps[i] = NTK_MULT16_32_P15(QCONST16(.2f, 15), st->old_ps[i]) + NTK_MULT16_32_P15(MULT16_16_P15(QCONST16(.8f, 15), SQR16_Q15(st->gain[i])), ps[i]);

            /* Apply gain floor */
            if (st->gain[i] < st->gain_floor[i])
                st->gain[i] = st->gain_floor[i];

            /* Exponential decay model for reverberation (unused) */
            /*st->reverb_estimate[i] = st->reverb_decay*st->reverb_estimate[i] + st->reverb_decay*st->reverb_level*st->gain[i]*st->gain[i]*st->ps[i];*/

            /* Take into account speech probability of presence (loudness domain MMSE estimator) */
            /* gain2 = [p*sqrt(gain)+(1-p)*sqrt(gain _floor) ]^2 */
            tmp = MULT16_16_P15(p, spx_sqrt(NTK_SHL32(EXTEND32(st->gain[i]), 15))) + MULT16_16_P15(SUB16(Q15_ONE, p), spx_sqrt(NTK_SHL32(EXTEND32(st->gain_floor[i]), 15)));
            st->gain2[i] = SQR16_Q15(tmp);

            /* Use this if you want a log-domain MMSE estimator instead */
            /*st->gain2[i] = pow(st->gain[i], p) * pow(st->gain_floor[i],1.f-p);*/
        }
    } else {
        for (i = N; i < N + M; i++) {
            spx_word16_t tmp;
            spx_word16_t p = st->gain2[i];
#ifdef _Bark_scale_OPT
            if (st->gain_floor[i] < st->gain[i])
                tmp = MULT16_16_P15(p, spx_sqrt(NTK_SHL32(EXTEND32(st->gain[i]), 15))) + MULT16_16_P15(Q15_ONE - p, spx_sqrt(NTK_SHL32(EXTEND32(st->gain_floor[i]), 15)));
            else
                tmp = spx_sqrt(NTK_SHL32(EXTEND32(st->gain_floor[i]), 15));
#else
            st->gain[i] = MAX16(st->gain[i], st->gain_floor[i]);
            tmp = MULT16_16_P15(p, spx_sqrt(NTK_SHL32(EXTEND32(st->gain[i]), 15))) + MULT16_16_P15(SUB16(Q15_ONE, p), spx_sqrt(NTK_SHL32(EXTEND32(st->gain_floor[i]), 15)));
#endif
            st->gain2[i] = SQR16_Q15(tmp);
        }
        filterbank_compute_psd16(st->bank, st->gain2 + N, st->gain2);
    }

#ifdef ADD_NLP
    if (stNlp.NlpEnable && near && far) {
        // s16 s16NLPRatio;
        // s16 s16NLPR_INV = SUB16(Q15_ONE, stNlp.NlpEffect);
        nlp_preprocess(&stNlp, near, far, x);
#if 1  // new method
        for (i = 0; i < N; i++) {
            if (st->gain2[i] > stNlp.nlp_gain[i]) {
                st->gain2[i] = ADD16(MULT16_16_P15(SUB16(Q15_ONE, st->gain2[i]), stNlp.nlp_gain[i]), MULT16_16_P15(st->gain2[i], st->gain2[i]));
            }
        }
#else
        for (i = 0; i < N; i++) {
            s16NLPRatio = MULT16_16_P15(s16NLPR_INV, st->gain2[i]);
            stNlp.nlp_gain[i] = s16NLPRatio + MULT16_16_P15(SUB16(Q15_ONE, s16NLPRatio), stNlp.nlp_gain[i]);  // nlp_gain = 1*ratio + nlp_gain*ratio
            st->gain2[i] = MULT16_16_P15(st->gain2[i], stNlp.nlp_gain[i]);
        }
#endif
    }
#endif

    /* If noise suppression is off, don't apply the gain (but then why call this in the first place!) */
    // if (!st->denoise_enabled)
    //{
    //     for (i=0;i<N+M;i++)
    //         st->gain2[i]=Q15_ONE;
    // }

    /* Apply computed gain */
    for (i = 1; i < N; i++) {
        st->ft[2 * i - 1] = MULT16_16_P15(st->gain2[i], st->ft[2 * i - 1]);
        st->ft[2 * i] = MULT16_16_P15(st->gain2[i], st->ft[2 * i]);
    }
    st->ft[0] = MULT16_16_P15(st->gain2[0], st->ft[0]);
    st->ft[2 * N - 1] = MULT16_16_P15(st->gain2[N - 1], st->ft[2 * N - 1]);

    /*FIXME: This *will* not work for fixed-point */
#ifndef FIXED_POINT
    if (st->agc_enabled)
        speex_compute_agc(st, Pframe, st->ft);
#endif

    /* Inverse FFT with 1/N scaling */
    spx_ifft(st->fft_lookup, st->ft, st->frame);
    /* Scale back to original (lower) amplitude */
    for (i = 0; i < 2 * N; i++)
        st->frame[i] = (short)NTK_PSHR16((int)st->frame[i], st->frame_shift);

        /*FIXME: This *will* not work for fixed-point */
#ifndef FIXED_POINT
    if (st->agc_enabled) {
        float max_sample = 0;
        for (i = 0; i < 2 * N; i++)
            if (fabs(st->frame[i]) > max_sample)
                max_sample = fabs(st->frame[i]);
        if (max_sample > 28000.f) {
            float damp = 28000.f / max_sample;
            for (i = 0; i < 2 * N; i++)
                st->frame[i] *= damp;
        }
    }
#endif

    /* Synthesis window (for WOLA) */
    for (i = 0; i < 2 * N; i++)
        st->frame[i] = MULT16_16_Q15(st->frame[i], st->window[i]);

    /* Perform overlap and add */
    for (i = 0; i < N3; i++)
        x[num_chan * i + chan_index] = ADD16(st->outbuf[i], st->frame[i]);

    for (i = 0; i < N4; i++)
        x[num_chan * (N3 + i) + chan_index] = st->frame[N3 + i];

    /* Update outbuf */
    for (i = 0; i < N3; i++)
        st->outbuf[i] = st->frame[st->frame_size + i];

        /* FIXME: This VAD is a kludge */
#if 0  // useless 
    st->speech_prob = Pframe;
    if (st->vad_enabled)
    {
        if (st->speech_prob > st->speech_prob_start || (st->was_speech && st->speech_prob > st->speech_prob_continue))
        {
            st->was_speech=1;
            return 1;
        } else
        {
            st->was_speech=0;
            return 0;
        }
    } else {
        return 1;
    }
#endif
    return 0;
}
#endif

#if 0
EXPORT void speex_preprocess_estimate_update(SpeexPreprocessState *st, spx_int16_t *x)
{
    int i;
    int N = st->ps_size;
    int N3 = 2*N - st->frame_size;
    int M;
    spx_word32_t *ps=st->ps;

    M = st->nbands;
    st->min_count++;

    preprocess_analysis(st, x);

    update_noise_prob(st);

    for (i=1;i<N-1;i++)
    {
        if (!st->update_prob[i] || st->ps[i] < PSHR32(st->noise[i],NOISE_SHIFT))
        {
            st->noise[i] = NTK_MULT16_32_Q15(QCONST16(.95f,15),st->noise[i]) + NTK_MULT16_32_Q15(QCONST16(.05f,15),NTK_SHL32(st->ps[i],NOISE_SHIFT));
        }
    }

    for (i=0;i<N3;i++)
        st->outbuf[i] = MULT16_16_Q15(x[st->frame_size-N3+i],st->window[st->frame_size+i]);

    /* Save old power spectrum */
    for (i=0;i<N+M;i++)
        st->old_ps[i] = ps[i];

    for (i=0;i<N;i++)
        st->reverb_estimate[i] = NTK_MULT16_32_Q15(st->reverb_decay, st->reverb_estimate[i]);
}
#endif

#if 0
EXPORT int speex_preprocess_ctl(void *state, int request, void *ptr)
{
    int i;
    SpeexPreprocessState *st;
    st=(SpeexPreprocessState*)state;
    switch(request)
    {
    case SPEEX_PREPROCESS_SET_DENOISE:
        st->denoise_enabled = (*(spx_int32_t*)ptr);
        break;
    case SPEEX_PREPROCESS_GET_DENOISE:
        (*(spx_int32_t*)ptr) = st->denoise_enabled;
        break;
#ifndef FIXED_POINT
    case SPEEX_PREPROCESS_SET_AGC:
        st->agc_enabled = (*(spx_int32_t*)ptr);
        break;
    case SPEEX_PREPROCESS_GET_AGC:
        (*(spx_int32_t*)ptr) = st->agc_enabled;
        break;
#ifndef DISABLE_FLOAT_API
    case SPEEX_PREPROCESS_SET_AGC_LEVEL:
        st->agc_level = (*(float*)ptr);
        if (st->agc_level<1)
            st->agc_level=1;
        if (st->agc_level>32768)
            st->agc_level=32768;
        break;
    case SPEEX_PREPROCESS_GET_AGC_LEVEL:
        (*(float*)ptr) = st->agc_level;
        break;
#endif /* #ifndef DISABLE_FLOAT_API */
    case SPEEX_PREPROCESS_SET_AGC_INCREMENT:
        st->max_increase_step = exp(0.11513f * (*(spx_int32_t*)ptr)*st->frame_size / st->sampling_rate);
        break;
    case SPEEX_PREPROCESS_GET_AGC_INCREMENT:
        (*(spx_int32_t*)ptr) = floor(.5+8.6858*log(st->max_increase_step)*st->sampling_rate/st->frame_size);
        break;
    case SPEEX_PREPROCESS_SET_AGC_DECREMENT:
        st->max_decrease_step = exp(0.11513f * (*(spx_int32_t*)ptr)*st->frame_size / st->sampling_rate);
        break;
    case SPEEX_PREPROCESS_GET_AGC_DECREMENT:
        (*(spx_int32_t*)ptr) = floor(.5+8.6858*log(st->max_decrease_step)*st->sampling_rate/st->frame_size);
        break;
    case SPEEX_PREPROCESS_SET_AGC_MAX_GAIN:
        st->max_gain = exp(0.11513f * (*(spx_int32_t*)ptr));
        break;
    case SPEEX_PREPROCESS_GET_AGC_MAX_GAIN:
        (*(spx_int32_t*)ptr) = floor(.5+8.6858*log(st->max_gain));
        break;
#endif
    case SPEEX_PREPROCESS_SET_VAD:
        speex_warning("The VAD has been replaced by a hack pending a complete rewrite");
        st->vad_enabled = (*(spx_int32_t*)ptr);
        break;
    case SPEEX_PREPROCESS_GET_VAD:
        (*(spx_int32_t*)ptr) = st->vad_enabled;
        break;

    case SPEEX_PREPROCESS_SET_DEREVERB:
        st->dereverb_enabled = (*(spx_int32_t*)ptr);
        for (i=0;i<st->ps_size;i++)
            st->reverb_estimate[i]=0;
        break;
    case SPEEX_PREPROCESS_GET_DEREVERB:
        (*(spx_int32_t*)ptr) = st->dereverb_enabled;
        break;

    case SPEEX_PREPROCESS_SET_DEREVERB_LEVEL:
        /* FIXME: Re-enable when de-reverberation is actually enabled again */
        /*st->reverb_level = (*(float*)ptr);*/
        break;
    case SPEEX_PREPROCESS_GET_DEREVERB_LEVEL:
        /* FIXME: Re-enable when de-reverberation is actually enabled again */
        /*(*(float*)ptr) = st->reverb_level;*/
        break;

    case SPEEX_PREPROCESS_SET_DEREVERB_DECAY:
        /* FIXME: Re-enable when de-reverberation is actually enabled again */
        /*st->reverb_decay = (*(float*)ptr);*/
        break;
    case SPEEX_PREPROCESS_GET_DEREVERB_DECAY:
        /* FIXME: Re-enable when de-reverberation is actually enabled again */
        /*(*(float*)ptr) = st->reverb_decay;*/
        break;

    case SPEEX_PREPROCESS_SET_PROB_START:
        *(spx_int32_t*)ptr = MIN32(100,MAX32(0, *(spx_int32_t*)ptr));
        st->speech_prob_start = DIV32_16(MULT16_16(Q15ONE,*(spx_int32_t*)ptr), 100);
        break;
    case SPEEX_PREPROCESS_GET_PROB_START:
        (*(spx_int32_t*)ptr) = MULT16_16_Q15(st->speech_prob_start, 100);
        break;

    case SPEEX_PREPROCESS_SET_PROB_CONTINUE:
        *(spx_int32_t*)ptr = MIN32(100,MAX32(0, *(spx_int32_t*)ptr));
        st->speech_prob_continue = DIV32_16(MULT16_16(Q15ONE,*(spx_int32_t*)ptr), 100);
        break;
    case SPEEX_PREPROCESS_GET_PROB_CONTINUE:
        (*(spx_int32_t*)ptr) = MULT16_16_Q15(st->speech_prob_continue, 100);
        break;

    case SPEEX_PREPROCESS_SET_NOISE_SUPPRESS:
        st->noise_suppress = -ABS(*(spx_int32_t*)ptr);
        break;
    case SPEEX_PREPROCESS_GET_NOISE_SUPPRESS:
        (*(spx_int32_t*)ptr) = st->noise_suppress;
        break;
    case SPEEX_PREPROCESS_SET_ECHO_SUPPRESS:
        st->echo_suppress = -ABS(*(spx_int32_t*)ptr);
        break;
    case SPEEX_PREPROCESS_GET_ECHO_SUPPRESS:
        (*(spx_int32_t*)ptr) = st->echo_suppress;
        break;
    case SPEEX_PREPROCESS_SET_ECHO_SUPPRESS_ACTIVE:
        st->echo_suppress_active = -ABS(*(spx_int32_t*)ptr);
        break;
    case SPEEX_PREPROCESS_GET_ECHO_SUPPRESS_ACTIVE:
        (*(spx_int32_t*)ptr) = st->echo_suppress_active;
        break;
    case SPEEX_PREPROCESS_SET_ECHO_STATE:
        st->echo_state = (SpeexEchoState*)ptr;
        break;
    case SPEEX_PREPROCESS_GET_ECHO_STATE:
        (*(SpeexEchoState**)ptr) = (SpeexEchoState*)st->echo_state;
        break;
#ifndef FIXED_POINT
    case SPEEX_PREPROCESS_GET_AGC_LOUDNESS:
        (*(spx_int32_t*)ptr) = pow(st->loudness, 1.0/LOUDNESS_EXP);
        break;
    case SPEEX_PREPROCESS_GET_AGC_GAIN:
        (*(spx_int32_t*)ptr) = floor(.5+8.6858*log(st->agc_gain));
        break;
#endif
    case SPEEX_PREPROCESS_GET_PSD_SIZE:
    case SPEEX_PREPROCESS_GET_NOISE_PSD_SIZE:
        (*(spx_int32_t*)ptr) = st->ps_size;
        break;
    case SPEEX_PREPROCESS_GET_PSD:
        for(i=0;i<st->ps_size;i++)
            ((spx_int32_t *)ptr)[i] = (spx_int32_t) st->ps[i];
        break;
    case SPEEX_PREPROCESS_GET_NOISE_PSD:
        for(i=0;i<st->ps_size;i++)
            ((spx_int32_t *)ptr)[i] = (spx_int32_t) PSHR32(st->noise[i], NOISE_SHIFT);
        break;
    case SPEEX_PREPROCESS_GET_PROB:
        (*(spx_int32_t*)ptr) = MULT16_16_Q15(st->speech_prob, 100);
        break;
#ifndef FIXED_POINT
    case SPEEX_PREPROCESS_SET_AGC_TARGET:
        st->agc_level = (*(spx_int32_t*)ptr);
        if (st->agc_level<1)
            st->agc_level=1;
        if (st->agc_level>32768)
            st->agc_level=32768;
        break;
    case SPEEX_PREPROCESS_GET_AGC_TARGET:
        (*(spx_int32_t*)ptr) = st->agc_level;
        break;
#endif
    default:
        speex_warning_int("Unknown speex_preprocess_ctl request: ", request);
        return -1;
    }
    return 0;
}
#endif

#ifdef FIXED_DEBUG
long long spx_mips = 0;
#endif

u32 _Aec_SetPreProcParams(void **ppstPreProcState, EN_AUD_AEC_PARAMS enParamsCMD, void *pParamsValue)
{
    SpeexPreprocessState **ppstPreProcSt = (SpeexPreprocessState **)ppstPreProcState;
    SpeexEchoState *pstEchoSt = (SpeexEchoState *)ppstPreProcSt[0]->echo_state;
    s32 i;
    s32 s32value = *((s32 *)pParamsValue);
    s16 s16value = (s16)s32value;

    switch (enParamsCMD) {
        case EN_AUD_AEC_BANK_SCALE: {
            for (i = 0; i < pstEchoSt->C; i++)
                ppstPreProcSt[i]->bank_scale = s32value;
            break;
        }
        case EN_AUD_AEC_NOISE_SUPPRESS: {
            for (i = 0; i < pstEchoSt->C; i++)
                ppstPreProcSt[i]->noise_suppress = s32value;
            break;
        }
        case EN_AUD_AEC_ECHO_SUPPRESS: {
            for (i = 0; i < pstEchoSt->C; i++)
                ppstPreProcSt[i]->echo_suppress = s32value;
            break;
        }
        case EN_AUD_AEC_ECHO_SUPPRESS_ACTIVE: {
            for (i = 0; i < pstEchoSt->C; i++)
                ppstPreProcSt[i]->echo_suppress_active = s32value;
            break;
        }
        case EN_AUD_AEC_ECHO_NOISE_RATIO: {
            for (i = 0; i < pstEchoSt->C; i++)
                ppstPreProcSt[i]->echo_noise_ratio = s16value;
            break;
        }
        case EN_AUD_AEC_DENOISE: {
            for (i = 0; i < pstEchoSt->C; i++)
                ppstPreProcSt[i]->denoise_enabled = s32value;
            break;
        }
#ifdef ADD_NLP
        case EN_AUD_AEC_NLP_ENABLE: {
            if ((pstEchoSt->C == 1) && (pstEchoSt->K == 1) && (pstEchoSt->sampling_rate == 8000))
                stNlp.NlpEnable = s16value;
            else
                stNlp.NlpEnable = 0;
            break;
        }
        /*case EN_AUD_AEC_NLP_EFFECT:
        {
            stNlp.NlpEffect = s16value;
            break;
        }*/
#endif
        default:
            return 1;
    }
    return 0;
}
#include "kiss_fftr.h"
struct kiss_fftr_state{
    kiss_fft_cfg substate;
    kiss_fft_cpx * tmpbuf;
    kiss_fft_cpx * super_twiddles;
#ifdef USE_SIMD    
    long pad;
#endif    
};

u32 _Aec_GetInternalBufSize(PST_AUD_AEC_INFO pstAecInfo, u32 u32NumPreProc)
{
    u32 u32InitSize = 0;
    u32 u32AECInternalSize = 0;
    u32 u32PreProcInternalSize = 0;
    u32 u32NLPInternalSize = 0;
    u32 u32TotalInternalSize = 0;
    u32 C = pstAecInfo->u32NumMic;
    u32 K = pstAecInfo->u32NumSpeaker;
    u32 N = pstAecInfo->u32FrameSize << 1;
    u32 frame_size = pstAecInfo->u32FrameSize;
    u32 M = (pstAecInfo->u32FilterLen + frame_size - 1) / frame_size;
    u32 KN, tmp;
    KN = (K * N) << 1;
    tmp = (frame_size + 1) << 2;

    u32InitSize += ALLIGN_4BYTE(C * sizeof(void *));
    u32InitSize += ALLIGN_4BYTE((C * frame_size) << 1);

    //===AEC===
    u32AECInternalSize += ALLIGN_4BYTE(sizeof(SpeexEchoState));
    u32AECInternalSize += ALLIGN_4BYTE(SIZE_OF_KISS_CONFIG);             //(sizeof(struct kiss_config));
    u32AECInternalSize += ALLIGN_4BYTE((MIST_NUMBER + 3 * frame_size - 1) << 2);  // kiss_fftr_alloc(size,0,NULL,NULL);
    u32AECInternalSize += ALLIGN_4BYTE((MIST_NUMBER + 3 * frame_size - 1) << 2);  // kiss_fftr_alloc(size,0,NULL,NULL);

    if (pstAecInfo->u32SpkrDualMono) {
        KN = (N) << 1;
        u32AECInternalSize += ALLIGN_4BYTE((N) << 1) * 5;               // e y Y E last_y
        u32AECInternalSize += ALLIGN_4BYTE(tmp) * 5;                    // Yf Rf Xf Yh Wh
        u32AECInternalSize += ALLIGN_4BYTE((frame_size) << 1);          // input
        u32AECInternalSize += ALLIGN_4BYTE(KN);                         // x
        u32AECInternalSize += ALLIGN_4BYTE(KN * (M + 1));               // X
        u32AECInternalSize += ALLIGN_4BYTE(KN * M);                     // foreground
        u32AECInternalSize += ALLIGN_4BYTE(KN * M * 2);                 // W
        u32AECInternalSize += ALLIGN_4BYTE(N << 2);                     // PHI
        u32AECInternalSize += ALLIGN_4BYTE((frame_size + 1) << 2) * 2;  // power power1
        u32AECInternalSize += ALLIGN_4BYTE(N << 1) * 3;                 // window wtmp wtmp2
        u32AECInternalSize += ALLIGN_4BYTE(M << 1);                     // prop
        u32AECInternalSize += ALLIGN_4BYTE(1 << 1);                     // memX
        u32AECInternalSize += ALLIGN_4BYTE(1 << 1) * 2;                 // memD memE
        u32AECInternalSize += ALLIGN_4BYTE(1 << 3);                     // notch_mem
#ifdef USE_THOSE_PARAMS
        u32AECInternalSize += ALLIGN_4BYTE((1 * (PLAYBACK_DELAY + 1) * frame_size) << 1);  // play_buf
#endif
        u32AECInternalSize += ALLIGN_4BYTE(1 << 1);     // ps16AmpRate
        u32AECInternalSize += ALLIGN_4BYTE(1 << 2);     //_ppstPreProcState
        u32AECInternalSize = u32AECInternalSize * (C);  // mono process
    } else {
        u32AECInternalSize += ALLIGN_4BYTE((C * N) << 1) * 5;           // e y Y E last_y
        u32AECInternalSize += ALLIGN_4BYTE(tmp) * 5;                    // Yf Rf Xf Yh Wh
        u32AECInternalSize += ALLIGN_4BYTE((C * frame_size) << 1);      // input
        u32AECInternalSize += ALLIGN_4BYTE(KN);                         // x
        u32AECInternalSize += ALLIGN_4BYTE(KN * (M + 1));               // X
        u32AECInternalSize += ALLIGN_4BYTE(KN * C * M);                 // foreground
        u32AECInternalSize += ALLIGN_4BYTE(KN * C * M * 2);             // W
        u32AECInternalSize += ALLIGN_4BYTE(N << 2);                     // PHI
        u32AECInternalSize += ALLIGN_4BYTE((frame_size + 1) << 2) * 2;  // power power1
        u32AECInternalSize += ALLIGN_4BYTE(N << 1) * 3;                 // window wtmp wtmp2
        u32AECInternalSize += ALLIGN_4BYTE(M << 1);                     // prop
        u32AECInternalSize += ALLIGN_4BYTE(K << 1);                     // memX
        u32AECInternalSize += ALLIGN_4BYTE(C << 1) * 2;                 // memD memE
        u32AECInternalSize += ALLIGN_4BYTE(C << 3);                     // notch_mem
#ifdef USE_THOSE_PARAMS
        u32AECInternalSize += ALLIGN_4BYTE((K * (PLAYBACK_DELAY + 1) * frame_size) << 1);  // play_buf
#endif
        u32AECInternalSize += ALLIGN_4BYTE(C << 1);  // ps16AmpRate

        u32AECInternalSize += ALLIGN_4BYTE(C << 2);  //_ppstPreProcState
    }
    // printf("[PreInit]AECInternalSize = %d\n",u32AECInternalSize);

    //===NS===
    M = NB_BANDS;
    u32PreProcInternalSize += ALLIGN_4BYTE(sizeof(SpeexPreprocessState));
    u32PreProcInternalSize += ALLIGN_4BYTE(sizeof(FilterBank));
    u32PreProcInternalSize += ALLIGN_4BYTE(frame_size << 2) * 2;
    u32PreProcInternalSize += ALLIGN_4BYTE(frame_size << 1) * 2;
    u32PreProcInternalSize += ALLIGN_4BYTE(N << 1) * 3;
    u32PreProcInternalSize += ALLIGN_4BYTE((frame_size + M) << 2) * 5;
#ifdef USE_THOSE_PARAMS
    u32PreProcInternalSize += ALLIGN_4BYTE((frame_size + M) << 2);
#endif
    u32PreProcInternalSize += ALLIGN_4BYTE((frame_size + M) << 1) * 6;
    u32PreProcInternalSize += ALLIGN_4BYTE(frame_size << 2) * 4;
    u32PreProcInternalSize += ALLIGN_4BYTE(frame_size << 1) * 2;
    u32PreProcInternalSize += ALLIGN_4BYTE(SIZE_OF_KISS_CONFIG);             //(sizeof(struct kiss_config));
    u32PreProcInternalSize += ALLIGN_4BYTE((MIST_NUMBER + 3 * frame_size - 1) << 2);  // kiss_fftr_alloc(size,0,NULL,NULL);
    u32PreProcInternalSize += ALLIGN_4BYTE((MIST_NUMBER + 3 * frame_size - 1) << 2);  // kiss_fftr_alloc(size,0,NULL,NULL);
    // printf("[PreInit]PreProcInternalSize = %d\n",u32PreProcInternalSize);

#ifdef ADD_NLP
    //===NLP===
    if ((((C == 1) && (K == 1)) || (pstAecInfo->u32SpkrDualMono)) && (pstAecInfo->u32SamplingRate == 8000)) {
        u32NLPInternalSize += ALLIGN_4BYTE(frame_size << 2) * 3;
        u32NLPInternalSize += ALLIGN_4BYTE(SIZE_OF_KISS_CONFIG);             //(sizeof(struct kiss_config));
        u32NLPInternalSize += ALLIGN_4BYTE((MIST_NUMBER + 3 * frame_size - 1) << 2);  // kiss_fftr_alloc(size,0,NULL,NULL);
        u32NLPInternalSize += ALLIGN_4BYTE((MIST_NUMBER + 3 * frame_size - 1) << 2);  // kiss_fftr_alloc(size,0,NULL,NULL);
        u32NLPInternalSize += ALLIGN_4BYTE(((frame_size << 1) + 2) << 1);
        u32NLPInternalSize += ALLIGN_4BYTE(frame_size << 2);
        u32NLPInternalSize += ALLIGN_4BYTE((frame_size + 1) << 2) * 3;
        u32NLPInternalSize += ALLIGN_4BYTE((frame_size + 1) << 2) * 3;
        u32NLPInternalSize += ALLIGN_4BYTE((frame_size + 1) << 3) * 2;
        u32NLPInternalSize += ALLIGN_4BYTE((frame_size + 1) << 1) * 2;
        u32NLPInternalSize += ALLIGN_4BYTE((frame_size + 1) << 1);
    }
    // printf("[PreInit]NLPInternalSize = %d\n",u32NLPInternalSize);
#endif

    u32TotalInternalSize = u32InitSize + u32AECInternalSize + u32NumPreProc * u32PreProcInternalSize + u32NLPInternalSize;
    // printf("[PreInit]InternalSize = %d\n",u32TotalInternalSize);

    return u32TotalInternalSize;
}

u32 _Ns_SetPreProcParams(void **ppstPreProcState, EN_AUD_NS_PARAMS enParamsCMD, void *pParamsValue, s32 s32ChannelNum)
{
    SpeexPreprocessState **ppstPreProcSt = (SpeexPreprocessState **)ppstPreProcState;
    s32 i;
    s32 s32value = *((s32 *)pParamsValue);
    // s16 s16value = (s16)s32value;

    switch (enParamsCMD) {
        case EN_AUD_NS_BANK_SCALE: {
            for (i = 0; i < s32ChannelNum; i++)
                ppstPreProcSt[i]->bank_scale = s32value;
            break;
        }
        case EN_AUD_NS_NOISE_SUPPRESS: {
            for (i = 0; i < s32ChannelNum; i++)
                ppstPreProcSt[i]->noise_suppress = s32value;
            break;
        }
        case EN_AUD_NS_DENOISE: {
            for (i = 0; i < s32ChannelNum; i++)
                ppstPreProcSt[i]->denoise_enabled = s32value;
            break;
        }
        default:
            return 1;
    }
    return 0;
}

u32 _Ns_GetInternalBufSize(PST_AUD_NS_INFO pstNsInfo, u32 u32NumPreProc)
{
    u32 u32InitSize = 0;
    u32 u32PreProcInternalSize = 0;
    u32 u32TotalInternalSize = 0;
    u32 frame_size = (u32)pstNsInfo->s32FrameSize;
    u32 N = frame_size << 1;
    u32 M;

    u32InitSize += ALLIGN_4BYTE(pstNsInfo->s32ChannelNum * sizeof(void *));
    // printf("[PreInit]u32InitSize = %d\n",u32InitSize);

    //===NS===
    M = NB_BANDS;
    u32PreProcInternalSize += ALLIGN_4BYTE(sizeof(SpeexPreprocessState));
    u32PreProcInternalSize += ALLIGN_4BYTE(sizeof(FilterBank));
    u32PreProcInternalSize += ALLIGN_4BYTE(frame_size << 2) * 2;
    u32PreProcInternalSize += ALLIGN_4BYTE(frame_size << 1) * 2;
    u32PreProcInternalSize += ALLIGN_4BYTE(N << 1) * 3;
    u32PreProcInternalSize += ALLIGN_4BYTE((frame_size + M) << 2) * 5;
#ifdef USE_THOSE_PARAMS
    u32PreProcInternalSize += ALLIGN_4BYTE((frame_size + M) << 2);
#endif
    u32PreProcInternalSize += ALLIGN_4BYTE((frame_size + M) << 1) * 6;
    u32PreProcInternalSize += ALLIGN_4BYTE(frame_size << 2) * 4;
    u32PreProcInternalSize += ALLIGN_4BYTE(frame_size << 1) * 2;
    u32PreProcInternalSize += ALLIGN_4BYTE(SIZE_OF_KISS_CONFIG);             //(sizeof(struct kiss_config));
    u32PreProcInternalSize += ALLIGN_4BYTE((MIST_NUMBER + 3 * frame_size - 1) << 2);  // kiss_fftr_alloc(size,0,NULL,NULL);
    u32PreProcInternalSize += ALLIGN_4BYTE((MIST_NUMBER + 3 * frame_size - 1) << 2);  // kiss_fftr_alloc(size,0,NULL,NULL);
    // printf("[PreInit]PreProcInternalSize = %d\n",u32PreProcInternalSize);

    u32TotalInternalSize = u32InitSize + u32NumPreProc * u32PreProcInternalSize;
    // printf("[PreInit]InternalSize = %d\n",u32TotalInternalSize);

    return u32TotalInternalSize;
}
