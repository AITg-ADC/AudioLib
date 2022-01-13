/* Copyright (C) 2003-2008 Jean-Marc Valin

File: mdf.c
Echo canceller based on the MDF algorithm (see below)

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
The echo canceller is based on the MDF algorithm described in:

J. S. Soo, K. K. Pang Multidelay block frequency adaptive filter,
IEEE Trans. Acoust. Speech Signal Process., Vol. ASSP-38, No. 2,
February 1990.

We use the Alternatively Updated MDF (AUMDF) variant. Robustness to
double-talk is achieved using a variable learning rate as described in:

Valin, J.-M., On Adjusting the Learning Rate in Frequency Domain Echo
Cancellation With Double-Talk. IEEE Transactions on Audio,
Speech and Language Processing, Vol. 15, No. 3, pp. 1030-1034, 2007.
http://people.xiph.org/~jm/papers/valin_taslp2006.pdf

There is no explicit double-talk detection, but a continuous variation
in the learning rate based on residual echo, double-talk and background
noise.

About the fixed-point version:
All the signals are represented with 16-bit words. The filter weights
are represented with 32-bit words, but only the top 16 bits are used
in most cases. The lower 16 bits are completely unreliable (due to the
fact that the update is done only on the top bits), but help in the
adaptation -- probably by removing a "threshold effect" due to
quantization (rounding going to zero) when the gradient is small.

Another kludge that seems to work good: when performing the weight
update, we only move half the way toward the "goal" this seems to
reduce the effect of quantization noise in the update phase. This
can be seen as applying a gradient descent on a "soft constraint"
instead of having a hard constraint.

*/
//#define DUMP_ECHO_CANCEL_DATA

#include "aud_aec_api.h"
#include "speex_echo.h"
#include "basic_op.h"
#include "ntk_basic_operation.h"
#include "ntk_opt_switch.h"
//#include "perf.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#ifdef FIXED_POINT
#define WEIGHT_SHIFT 11
#define NORMALIZE_SCALEDOWN 5
#define NORMALIZE_SCALEUP 3
#else
#define WEIGHT_SHIFT 0
#endif

#ifdef FIXED_POINT
//#define WORD2INT(x) ((x) < -32767 ? -32768 : ((x) > 32766 ? 32767 : (x)))
static inline short int WORD2INT(int y)
{
    int sign = (y) >> 31;
    if (sign != (y) >> (15)) {
        (y) = sign ^ ((1 << (15)) - 1);
    }
    return ((s16)y);
}
#else
#define WORD2INT(x) ((x) < -32767.5f ? -32768 : ((x) > 32766.5f ? 32767 : floor(.5 + (x))))
#endif

#ifdef FIXED_POINT
static const spx_float_t MIN_LEAK = {20972, -22};

/* Constants for the two-path filter */
static const spx_float_t VAR1_SMOOTH = {23593, -16};
static const spx_float_t VAR2_SMOOTH = {23675, -15};
static const spx_float_t VAR1_UPDATE = {16384, -15};
static const spx_float_t VAR2_UPDATE = {16384, -16};
static const spx_float_t VAR_BACKTRACK = {16384, -12};
#define TOP16(x) ((x) >> 16)

#else

static const spx_float_t MIN_LEAK = .005f;

/* Constants for the two-path filter */
static const spx_float_t VAR1_SMOOTH = .36f;
static const spx_float_t VAR2_SMOOTH = .7225f;
static const spx_float_t VAR1_UPDATE = .5f;
static const spx_float_t VAR2_UPDATE = .25f;
static const spx_float_t VAR_BACKTRACK = 4.f;
#define TOP16(x) (x)
#endif

void speex_echo_get_residual(SpeexEchoState *st, spx_word32_t *Yout, int len, int chan_index);

#ifdef _filter_dc_notch16_OPT
static inline void filter_dc_notch16(spx_int16_t *in, spx_word16_t radius, spx_word16_t *out, int len, spx_mem_t *mem, int stride)
{
    int i, mem0, mem1;
    spx_word16_t den2;
    s16 *pVin = in, *pOut = out;
#ifdef FIXED_POINT
#ifdef _MIPS_
    int tmp = MULT16_16_Q15(32767 - radius, 32767 - radius);
    MULT_AC0(radius, radius);
    MADD_AC0(QCONST16(.7, 15), tmp);
    den2 = Shift15_Round_AC0();
#else
    den2 = MULT16_16_Q15(radius, radius) + MULT16_16_Q15(QCONST16(.7, 15), MULT16_16_Q15(32767 - radius, 32767 - radius));
#endif
#else
    den2 = radius * radius + .7 * (1 - radius) * (1 - radius);
#endif
    /*printf ("%d %d %d %d %d %d\n", num[0], num[1], num[2], den[0], den[1], den[2]);*/
    mem0 = mem[0];
    mem1 = mem[1];
    for (i = 0; i < len; i++) {
        // spx_word16_t vin = *pVin;
        spx_word32_t vout = mem0 + NTK_SHL32(EXTEND32(*pVin), 15);
#ifdef FIXED_POINT
        mem0 = mem1 + NTK_SHL32(NTK_SHL32(-EXTEND32(*pVin), 15) + NTK_MULT16_32_Q15(radius, vout), 1);
#else
        mem0 = mem1 + 2 * (-(*pVin) + radius * vout);
#endif
        mem1 = NTK_SHL32(EXTEND32(*pVin), 15) - NTK_MULT16_32_Q15(den2, vout);
        *pOut++ = EXTRACT16(PSHR32(NTK_MULT16_32_Q15(radius, vout), 15));
        pVin += stride;
    }
    mem[0] = mem0;
    mem[1] = mem1;
}

#else
static inline void filter_dc_notch16(spx_int16_t *in, spx_word16_t radius, spx_word16_t *out, int len, spx_mem_t *mem, int stride)
{
    int i;
    spx_word16_t den2;
#ifdef FIXED_POINT
    den2 = MULT16_16_Q15(radius, radius) + MULT16_16_Q15(QCONST16(.7, 15), MULT16_16_Q15(32767 - radius, 32767 - radius));
#else
    den2 = radius * radius + .7 * (1 - radius) * (1 - radius);
#endif
    /*printf ("%d %d %d %d %d %d\n", num[0], num[1], num[2], den[0], den[1], den[2]);*/
    for (i = 0; i < len; i++) {
        spx_word16_t vin = in[i * stride];
        spx_word32_t vout = mem[0] + NTK_SHL32(EXTEND32(vin), 15);
#ifdef FIXED_POINT
        mem[0] = mem[1] + NTK_SHL32(NTK_SHL32(-EXTEND32(vin), 15) + NTK_MULT16_32_Q15(radius, vout), 1);
#else
        mem[0] = mem[1] + 2 * (-vin + radius * vout);
#endif
        mem[1] = NTK_SHL32(EXTEND32(vin), 15) - NTK_MULT16_32_Q15(den2, vout);
        out[i] = SATURATE32(PSHR32(NTK_MULT16_32_Q15(radius, vout), 15), 32767);
    }
}
#endif
/* This inner product is slightly different from the codec version because of fixed-point */

static inline spx_word32_t mdf_inner_prod(spx_word16_t *x, spx_word16_t *y, int len)
{
    spx_word32_t sum = 0;

#ifdef _MIPS_mdf_inner_prod_OPT_NOT_Exactly
    short int *px = x;
    short int *py = y;
    int s32len = len >> 3;
    MTHI_AC0(0);
    MTLO_AC0(0);
    while (s32len--) {
        MADD_AC0(px[0], py[0]);
        MADD_AC0(px[1], py[1]);
        MADD_AC0(px[2], py[2]);
        MADD_AC0(px[3], py[3]);
        MADD_AC0(px[4], py[4]);
        MADD_AC0(px[5], py[5]);
        MADD_AC0(px[6], py[6]);
        MADD_AC0(px[7], py[7]);
        px += 8;
        py += 8;
    }
    SHILO6_AC0();
    sum = MFLO_AC0();
#else
    sum = 0;
    len >>= 1;
    while (len--) {
        spx_word32_t part = 0;
        part = MAC16_16(part, *x++, *y++);
        part = MAC16_16(part, *x++, *y++);
        /* HINT: If you had a 40-bit accumulator, you could shift only at the end */
        sum = ADD32(sum, SHR32(part, 6));
    }
#endif
    return sum;
}

/** Compute power spectrum of a half-complex (packed) vector */
#ifdef _MIPS_power_spectrum_OPT
static inline void power_spectrum(spx_word16_t *X, spx_word32_t *ps, int N)
{
    int i;
    int *ptrA = ps;
    s16 *ptrB = X;

    *ptrA = MULT16_16(*ptrB, *ptrB);
    ptrA++;
    ptrB++;
    // ps[0]=MULT16_16(X[0],X[0]);
    for (i = 1; i < N - 1; i += 2) {
        MULT_AC0(*ptrB, *ptrB);
        ptrB++;
        MADD_AC0(*ptrB, *ptrB);
        ptrB++;
        *ptrA = MFLO_AC0();
        ptrA++;
        // ps[j] =  MULT16_16(X[i],X[i]) + MULT16_16(X[i+1],X[i+1]);
    }
    *ptrA = MULT16_16(*ptrB, *ptrB);
    // ps[j]=MULT16_16(X[i],X[i]);
}
#else
static inline void power_spectrum(spx_word16_t *X, spx_word32_t *ps, int N)
{
    int i, j;
    ps[0] = MULT16_16(X[0], X[0]);
    for (i = 1, j = 1; i < N - 1; i += 2, j++) {
        ps[j] = ADD32(MULT16_16(X[i], X[i]), MULT16_16(X[i + 1], X[i + 1]));
    }
    ps[j] = MULT16_16(X[i], X[i]);
}
#endif

#ifdef _MIPS_power_spectrum_accum_OPT_NOT_Exactly
static inline void power_spectrum_accum(spx_word16_t *X, spx_word32_t *ps, int N)
{
    int i, *pPS = ps;
    s16 *pX = X;

    (*pPS++) += MULT16_16(*pX, *pX);
    pX++;
    for (i = 1; i < N - 1; i += 2) {
        MULT_AC0(*pX, *pX);
        pX++;
        MADD_AC0(*pX, *pX);
        pX++;
        *pPS = MFLO_AC0();
        pPS++;
        //(*pPS++) +=  MULT16_16(X[i],X[i]) + MULT16_16(X[i+1],X[i+1]);
    }
    *pPS += MULT16_16(*pX, *pX);
}
#else
/** Compute power spectrum of a half-complex (packed) vector and accumulate */
static inline void power_spectrum_accum(spx_word16_t *X, spx_word32_t *ps, int N)
{
    int i, j;
    ps[0] += MULT16_16(X[0], X[0]);
    for (i = 1, j = 1; i < N - 1; i += 2, j++) {
        ps[j] += MULT16_16(X[i], X[i]) + MULT16_16(X[i + 1], X[i + 1]);
    }
    ps[j] += MULT16_16(X[i], X[i]);
}
#endif

/** Compute cross-power spectrum of a half-complex (packed) vectors and add to acc */
#ifdef FIXED_POINT
#ifdef _MIPS_spectral_mul_accum_OPT
static inline void spectral_mul_accum(spx_word16_t *X, spx_word32_t *Y, spx_word16_t *acc, int N, int M)
{
    int i, j;
    s16 *pX = X, *pACC = acc + 1;
    int *pY = Y;

    MULT_AC0(*pX, (*pY) >> 16);
    pX += (N - 1);
    pY += (N - 1);
    MULT_AC1(*pX, (*pY) >> 16);
    pX++;
    pY++;
    for (j = 1; j < M; j++) {
        MADD_AC0(*pX, (*pY) >> 16);
        pX += (N - 1);
        pY += (N - 1);
        MADD_AC1(*pX, (*pY) >> 16);
        pX++;
        pY++;
        // tmp1 = MAC16_16(tmp1, X[j*N],TOP16(Y[j*N]));
        // tmp1 = MAC16_16(tmp1, X[(j+1)*N-1],TOP16(Y[(j+1)*N-1]));
    }
#if 0   
    SHILO_AC0();
    tmp1 = MFLO_AC0();
    acc[0] = PSHR32(tmp1,WEIGHT_SHIFT);
    SHILO_AC1();
    tmp1 = MFLO_AC1();
    acc[N-1] = PSHR32(tmp1,WEIGHT_SHIFT);
#else
    acc[0] = Shift11_Round_AC0();
    acc[N - 1] = Shift11_Round_AC1();
#endif

    pX = X + 1;
    pY = Y + 1;
    for (i = 1; i < N - 1; i += 2) {
        s16 *ppX = pX;  // X[i]
        int *ppY = pY;  // Y[i]
        // tmp1 = tmp2 = 0;
        MULT_AC0(ppX[0], ppY[0] >> 16);
        MSUB_AC0(ppX[1], ppY[1] >> 16);
        MULT_AC1(ppX[1], ppY[0] >> 16);
        MADD_AC1(ppX[0], ppY[1] >> 16);
        ppX += N;
        ppY += N;
        for (j = 1; j < M; j++) {
            // tmp1 = SUB32(MAC16_16(tmp1, X[j*N+i],TOP16(Y[j*N+i])), MULT16_16(X[j*N+i+1],TOP16(Y[j*N+i+1])));
            // tmp2 = MAC16_16(MAC16_16(tmp2, X[j*N+i+1],TOP16(Y[j*N+i])), X[j*N+i], TOP16(Y[j*N+i+1]));
            MADD_AC0(ppX[0], ppY[0] >> 16);
            MSUB_AC0(ppX[1], ppY[1] >> 16);
            MADD_AC1(ppX[1], ppY[0] >> 16);
            MADD_AC1(ppX[0], ppY[1] >> 16);
            ppX += N;
            ppY += N;
        }
#if 0     
        SHILO_AC0();
        tmp1 = MFLO_AC0();
        SHILO_AC1();
        tmp2 = MFLO_AC1();
        *pACC++ = PSHR32(tmp1,WEIGHT_SHIFT);
        *pACC++ = PSHR32(tmp2,WEIGHT_SHIFT);
#else
        *pACC++ = Shift11_Round_AC0();
        *pACC++ = Shift11_Round_AC1();
#endif
        pX += 2;
        pY += 2;
    }
}
#else
static inline void spectral_mul_accum(spx_word16_t *X, spx_word32_t *Y, spx_word16_t *acc, int N, int M)
{
    int i, j;
    spx_word32_t tmp1 = 0, tmp2 = 0;
    for (j = 0; j < M; j++) {
        tmp1 = MAC16_16(tmp1, X[j * N], TOP16(Y[j * N]));
    }
    acc[0] = EXTRACT16(PSHR32(tmp1, WEIGHT_SHIFT));
    for (i = 1; i < N - 1; i += 2) {
        tmp1 = tmp2 = 0;
        for (j = 0; j < M; j++) {
            tmp1 = SUB32(MAC16_16(tmp1, X[j * N + i], TOP16(Y[j * N + i])), MULT16_16(X[j * N + i + 1], TOP16(Y[j * N + i + 1])));
            tmp2 = MAC16_16(MAC16_16(tmp2, X[j * N + i + 1], TOP16(Y[j * N + i])), X[j * N + i], TOP16(Y[j * N + i + 1]));
        }
        acc[i] = EXTRACT16(PSHR32(tmp1, WEIGHT_SHIFT));
        acc[i + 1] = EXTRACT16(PSHR32(tmp2, WEIGHT_SHIFT));
    }
    tmp1 = tmp2 = 0;
    for (j = 0; j < M; j++) {
        tmp1 = MAC16_16(tmp1, X[(j + 1) * N - 1], TOP16(Y[(j + 1) * N - 1]));
    }
    acc[N - 1] = EXTRACT16(PSHR32(tmp1, WEIGHT_SHIFT));
}
#endif
#ifdef _MIPS_spectral_mul_accum16_OPT
static inline void spectral_mul_accum16(spx_word16_t *X, spx_word16_t *Y, spx_word16_t *acc, int N, int M)
{
    int i, j;
    s16 *pX = X, *pACC = acc + 1;
    s16 *pY = Y;

    MULT_AC0(*pX, *pY);
    pX += (N - 1);
    pY += (N - 1);
    MULT_AC1(*pX, *pY);
    pX++;
    pY++;
    for (j = 1; j < M; j++) {
        MADD_AC0(*pX, *pY);
        pX += (N - 1);
        pY += (N - 1);
        MADD_AC1(*pX, *pY);
        pX++;
        pY++;
    }
    acc[0] = Shift11_Round_AC0();
    acc[N - 1] = Shift11_Round_AC1();

    pX = X + 1;
    pY = Y + 1;
    for (i = 1; i < N - 1; i += 2) {
        s16 *ppX = pX;  // X[i]
        s16 *ppY = pY;  // Y[i]
        // tmp1 = tmp2 = 0;
        MULT_AC0(ppX[0], ppY[0]);
        MSUB_AC0(ppX[1], ppY[1]);
        MULT_AC1(ppX[1], ppY[0]);
        MADD_AC1(ppX[0], ppY[1]);
        ppX += N;
        ppY += N;
        for (j = 1; j < M; j++) {
            MADD_AC0(ppX[0], ppY[0]);
            MSUB_AC0(ppX[1], ppY[1]);
            MADD_AC1(ppX[1], ppY[0]);
            MADD_AC1(ppX[0], ppY[1]);
            ppX += N;
            ppY += N;
        }
        *pACC++ = Shift11_Round_AC0();
        *pACC++ = Shift11_Round_AC1();

        pX += 2;
        pY += 2;
    }
}
#else
static inline void spectral_mul_accum16(spx_word16_t *X, spx_word16_t *Y, spx_word16_t *acc, int N, int M)
{
    int i, j;
    spx_word32_t tmp1 = 0, tmp2 = 0;
    for (j = 0; j < M; j++) {
        tmp1 = MAC16_16(tmp1, X[j * N], Y[j * N]);
    }
    acc[0] = EXTRACT16(PSHR32(tmp1, WEIGHT_SHIFT));
    for (i = 1; i < N - 1; i += 2) {
        tmp1 = tmp2 = 0;
        for (j = 0; j < M; j++) {
            tmp1 = SUB32(MAC16_16(tmp1, X[j * N + i], Y[j * N + i]), MULT16_16(X[j * N + i + 1], Y[j * N + i + 1]));
            tmp2 = MAC16_16(MAC16_16(tmp2, X[j * N + i + 1], Y[j * N + i]), X[j * N + i], Y[j * N + i + 1]);
        }
        acc[i] = EXTRACT16(PSHR32(tmp1, WEIGHT_SHIFT));
        acc[i + 1] = EXTRACT16(PSHR32(tmp2, WEIGHT_SHIFT));
    }
    tmp1 = tmp2 = 0;
    for (j = 0; j < M; j++) {
        tmp1 = MAC16_16(tmp1, X[(j + 1) * N - 1], Y[(j + 1) * N - 1]);
    }
    acc[N - 1] = EXTRACT16(PSHR32(tmp1, WEIGHT_SHIFT));
}
#endif
#else
static inline void spectral_mul_accum(spx_word16_t *X, spx_word32_t *Y, spx_word16_t *acc, int N, int M)
{
    int i, j;
    for (i = 0; i < N; i++)
        acc[i] = 0;
    for (j = 0; j < M; j++) {
        acc[0] += X[0] * Y[0];
        for (i = 1; i < N - 1; i += 2) {
            acc[i] += (X[i] * Y[i] - X[i + 1] * Y[i + 1]);
            acc[i + 1] += (X[i + 1] * Y[i] + X[i] * Y[i + 1]);
        }
        acc[i] += X[i] * Y[i];
        X += N;
        Y += N;
    }
}
#define spectral_mul_accum16 spectral_mul_accum
#endif

#ifdef _weighted_spectral_mul_conj_OPT
/** Compute weighted cross-power spectrum of a half-complex (packed) vector with conjugate */
static inline void weighted_spectral_mul_conj(spx_float_t *w, spx_float_t p, spx_word16_t *X, spx_word16_t *Y, spx_word32_t *prod, int N)
{
    int i;
    spx_float_t W;
    s16 *pX = X, *pY = Y;
    int *p_prod = prod;
    spx_float_t *pw = w;

    W = FLOAT_AMULT(p, *pw++);
    *p_prod++ = FLOAT_MUL32(W, MULT16_16(*pX++, *pY++));
    for (i = 1; i < N - 1; i += 2) {
        s16 X0, X1, Y0, Y1;
        X0 = *pX++;
        X1 = *pX++;
        Y0 = *pY++;
        Y1 = *pY++;
        W = FLOAT_AMULT(p, *pw++);
#ifdef _MIPS_
        int tmp;
        MULT_AC0(X0, Y0);
        MADD_AC0(X1, Y1);
        tmp = MFLO_AC0();
        *p_prod++ = FLOAT_MUL32(W, tmp);
        MULT_AC0(X0, Y1);
        MSUB_AC0(X1, Y0);
        tmp = MFLO_AC0();
        *p_prod++ = FLOAT_MUL32(W, tmp);
#else
        *p_prod++ = FLOAT_MUL32(W, MAC16_16(MULT16_16(X0, Y0), X1, Y1));
        *p_prod++ = FLOAT_MUL32(W, MAC16_16(MULT16_16(-X1, Y0), X0, Y1));
#endif
    }
    W = FLOAT_AMULT(p, *pw);
    *p_prod = FLOAT_MUL32(W, MULT16_16(*pX, *pY));
}

#else
/** Compute weighted cross-power spectrum of a half-complex (packed) vector with conjugate */
static inline void weighted_spectral_mul_conj(spx_float_t *w, spx_float_t p, spx_word16_t *X, spx_word16_t *Y, spx_word32_t *prod, int N)
{
    int i, j;
    spx_float_t W;
    W = FLOAT_AMULT(p, w[0]);
    prod[0] = FLOAT_MUL32(W, MULT16_16(X[0], Y[0]));
    for (i = 1, j = 1; i < N - 1; i += 2, j++) {
        W = FLOAT_AMULT(p, w[j]);
        prod[i] = FLOAT_MUL32(W, MAC16_16(MULT16_16(X[i], Y[i]), X[i + 1], Y[i + 1]));
        prod[i + 1] = FLOAT_MUL32(W, MAC16_16(MULT16_16(-X[i + 1], Y[i]), X[i], Y[i + 1]));
    }
    W = FLOAT_AMULT(p, w[j]);
    prod[i] = FLOAT_MUL32(W, MULT16_16(X[i], Y[i]));
}
#endif

#ifdef _mdf_adjust_prop_OPT
static inline void mdf_adjust_prop(spx_word32_t *W, int N, int M, int P, spx_word16_t *prop)
{
    int i, j, p, NM = N * M;
    s16 offset, offset2;
    spx_word16_t max_sum = 1;
    spx_word32_t prop_sum = 1;
    s16 *p_prop = prop;

    offset = 0;
    for (i = 0; i < M; i++) {
        spx_word32_t tmp = 1;
        offset2 = 0;
        for (p = 0; p < P; p++) {
            int *pW = W + offset + offset2;
            for (j = 0; j < N; j++) {
                s16 s16tmp = (s16)SHR32(*pW++, 18);
                tmp += MULT16_16(s16tmp, s16tmp);
                // tmp += MULT16_16(EXTRACT16(SHR32(W[p*N*M + i*N+j],18)), EXTRACT16(SHR32(W[p*N*M + i*N+j],18)));
            }
            offset2 += NM;
        }
        offset += N;

#ifdef FIXED_POINT
        /* Just a security in case an overflow were to occur */
        tmp = MIN32(ABS32(tmp), 536870912);
#endif

        *p_prop = spx_sqrt(tmp);
        if (*p_prop > max_sum)
            max_sum = *p_prop;
        p_prop++;
    }

    offset = QCONST16(.1f, 15);
    p_prop = prop;
    for (i = 0; i < M; i++) {
        *p_prop += MULT16_16_Q15(offset, max_sum);
        prop_sum += EXTEND32(*p_prop);
        p_prop++;
    }

    offset = QCONST16(.99f, 15);
    p_prop = prop;
    for (i = 0; i < M; i++) {
        *p_prop = DIV32(MULT16_16(offset, *p_prop), prop_sum);
        p_prop++;
        /*printf ("%f ", prop[i]);*/
    }
    /*printf ("\n");*/
}

#else
static inline void mdf_adjust_prop(spx_word32_t *W, int N, int M, int P, spx_word16_t *prop)
{
    int i, j, p;
    spx_word16_t max_sum = 1;
    spx_word32_t prop_sum = 1;
    for (i = 0; i < M; i++) {
        spx_word32_t tmp = 1;
        for (p = 0; p < P; p++)
            for (j = 0; j < N; j++)
                tmp += MULT16_16(EXTRACT16(SHR32(W[p * N * M + i * N + j], 18)), EXTRACT16(SHR32(W[p * N * M + i * N + j], 18)));
#ifdef FIXED_POINT
        /* Just a security in case an overflow were to occur */
        tmp = MIN32(ABS32(tmp), 536870912);
#endif
        prop[i] = spx_sqrt(tmp);
        if (prop[i] > max_sum)
            max_sum = prop[i];
    }
    for (i = 0; i < M; i++) {
        prop[i] += MULT16_16_Q15(QCONST16(.1f, 15), max_sum);
        prop_sum += EXTEND32(prop[i]);
    }
    for (i = 0; i < M; i++) {
        prop[i] = DIV32(MULT16_16(QCONST16(.99f, 15), prop[i]), prop_sum);
        /*printf ("%f ", prop[i]);*/
    }
    /*printf ("\n");*/
}
#endif

#ifdef DUMP_ECHO_CANCEL_DATA
#include <stdio.h>
static FILE *rFile = NULL, *pFile = NULL, *oFile = NULL;

static void dump_audio(const spx_int16_t *rec, const spx_int16_t *play, const spx_int16_t *out, int len)
{
    if (!(rFile && pFile && oFile)) {
        speex_fatal("Dump files not open");
    }
    fwrite(rec, sizeof(spx_int16_t), len, rFile);
    fwrite(play, sizeof(spx_int16_t), len, pFile);
    fwrite(out, sizeof(spx_int16_t), len, oFile);
}
#endif

#if 0
/** Creates a new echo canceller state */
EXPORT SpeexEchoState *speex_echo_state_init(int frame_size, int filter_length, int nb_mic, int nb_speakers, int sampling_rate)
{
    return speex_echo_state_init_mc(frame_size, filter_length, nb_mic, nb_speakers, sampling_rate);
}
#endif

EXPORT SpeexEchoState *speex_echo_state_init_mc(int frame_size, int filter_length, int nb_mic, int nb_speakers, int sampling_rate, PST_AUD_AEC_PRELOAD pstAecPreload)
{
    int i, N, M, C, K, tmp;
    SpeexEchoState *st = (SpeexEchoState *)speex_alloc(sizeof(SpeexEchoState));
    if (!st) return 0;

    st->K = nb_speakers;
    st->C = nb_mic;
    C = st->C;
    K = st->K;
#ifdef DUMP_ECHO_CANCEL_DATA
    if (rFile || pFile || oFile)
        speex_fatal("Opening dump files twice");
    rFile = fopen("aec_rec.pcm", "wb");
    pFile = fopen("aec_play.pcm", "wb");
    oFile = fopen("aec_out.pcm", "wb");
#endif

    st->frame_size = frame_size;
    st->window_size = frame_size << 1;
    N = st->window_size;
    M = st->M = (filter_length + st->frame_size - 1) / frame_size;
    st->cancel_count = 0;
    st->sum_adapt = 0;
    st->saturated = 0;
    st->screwed_up = 0;
    /* This is the default sampling rate */
    st->sampling_rate = sampling_rate;
#ifdef FIXED_POINT
    st->beta0 = (spx_word16_t)DIV32(NTK_SHL32(EXTEND32(st->frame_size), 16), st->sampling_rate);
    // st->beta_max = (spx_word16_t)DIV32(NTK_SHL32(EXTEND32(st->frame_size), 14), st->sampling_rate);
    st->spec_average = st->beta0 >> 1;  //(spx_word16_t)DIV32(NTK_SHL32(EXTEND32(st->frame_size), 15), st->sampling_rate);
    st->beta_max = st->spec_average >> 1;
#else
    st->beta0 = (2.0f * st->frame_size) / st->sampling_rate;
    st->beta_max = (.5f * st->frame_size) / st->sampling_rate;
    st->spec_average = (spx_word16_t)DIV32(NTK_SHL32(EXTEND32(st->frame_size), 15), st->sampling_rate);
#endif
    st->leak_estimate = 0x2000;

    st->fft_table = spx_fft_init(N);

    tmp = (C * N) << 1;  // C*N*sizeof(spx_word16_t)
    st->e = (spx_word16_t *)speex_alloc(tmp);
    st->y = (spx_word16_t *)speex_alloc(tmp);
    st->Y = (spx_word16_t *)speex_alloc(tmp);
    st->E = (spx_word16_t *)speex_alloc(tmp);
    st->last_y = (spx_word16_t *)speex_alloc(tmp);

    tmp = (frame_size + 1) << 2;  //(st->frame_size+1)*sizeof(spx_word32_t)
    st->Yf = (spx_word32_t *)speex_alloc(tmp);
    st->Rf = (spx_word32_t *)speex_alloc(tmp);
    st->Xf = (spx_word32_t *)speex_alloc(tmp);
    st->Yh = (spx_word32_t *)speex_alloc(tmp);
    st->Eh = (spx_word32_t *)speex_alloc(tmp);

    st->input = (spx_word16_t *)speex_alloc((C * frame_size) << 1);  // C*st->frame_size*sizeof(spx_word16_t)

    tmp = (K * N) << 1;  // K*N*sizeof(spx_word16_t)
    st->x = (spx_word16_t *)speex_alloc(tmp);
    st->X = (spx_word16_t *)speex_alloc(tmp * (M + 1));  // K*(M+1)*N*sizeof(spx_word16_t)
    tmp = tmp * C * M;                                   // M*N*C*K*sizeof(spx_word16_t)
#ifdef TWO_PATH
    st->foreground = (spx_word16_t *)speex_alloc(tmp);
    if (pstAecPreload->u32PreloadEnable && (pstAecPreload->u32ForegroundSize == tmp)) {
        memcpy(st->foreground, pstAecPreload->ps16Foreground, pstAecPreload->u32ForegroundSize);
    }
#endif
    tmp = tmp << 1;
    st->W = (spx_word32_t *)speex_alloc(tmp);  // C*K*M*N*sizeof(spx_word32_t)
    memset(st->W, 0, tmp);
    if (pstAecPreload->u32PreloadEnable && (pstAecPreload->u32BackgroundSize == tmp)) {
        memcpy(st->W, pstAecPreload->ps32Background, pstAecPreload->u32BackgroundSize);
    }

    st->PHI = (spx_word32_t *)speex_alloc(N << 2);
    tmp = (frame_size + 1) << 2;  //(frame_size+1)*sizeof(spx_word32_t)
    st->power = (spx_word32_t *)speex_alloc(tmp);
    st->power_1 = (spx_float_t *)speex_alloc(tmp);
    tmp = N << 1;
    st->window = (spx_word16_t *)speex_alloc(tmp);
    st->prop = (spx_word16_t *)speex_alloc(M << 1);
    st->wtmp = (spx_word16_t *)speex_alloc(tmp);
#ifdef FIXED_POINT
    st->wtmp2 = (spx_word16_t *)speex_alloc(tmp);
    for (i = 0; i < N >> 1; i++) {
        st->window[i] = (16383 - NTK_SHL16(spx_cos(DIV32_16(MULT16_16(25736, i << 1), N)), 1));
        st->window[N - i - 1] = st->window[i];
    }
#else
    for (i = 0; i < N; i++)
        st->window[i] = .5 - .5 * cos(2 * M_PI * i / N);
#endif

    {
        int *ps32des = (int *)&FLOAT_ONE;
        int *ps32src = (int *)st->power_1;
        for (i = 0; i <= frame_size; i++)
            // st->power_1[i] = FLOAT_ONE;
            ps32src[i] = *ps32des;
    }
    // for (i=0;i<N*M*K*C;i++)
    //    st->W[i] = 0;
    {
        spx_word32_t sum = 0;
        /* Ratio of ~10 between adaptation rate of first and last block */
        spx_word16_t decay = SHR32(spx_exp(NEG16(DIV32_16(QCONST16(2.4, 11), M))), 1);
        st->prop[0] = QCONST16(.7, 15);
        sum = EXTEND32(st->prop[0]);
        for (i = 1; i < M; i++) {
            st->prop[i] = MULT16_16_Q15(st->prop[i - 1], decay);
            sum = ADD32(sum, EXTEND32(st->prop[i]));
        }
        for (i = M - 1; i >= 0; i--) {
            st->prop[i] = DIV32(MULT16_16(QCONST16(.8f, 15), st->prop[i]), sum);
        }
    }

    st->memX = (spx_word16_t *)speex_alloc(K << 1);
    st->memD = (spx_word16_t *)speex_alloc(C << 1);
    st->memE = (spx_word16_t *)speex_alloc(C << 1);
    st->preemph = QCONST16(.9, 15);
    // if (st->sampling_rate<12000)
    //     st->notch_radius = QCONST16(.9, 15);
    // else if (st->sampling_rate<24000)
    //     st->notch_radius = QCONST16(.982, 15);
    // else
    st->notch_radius = QCONST16(.992, 15);

    st->notch_mem = (spx_mem_t *)speex_alloc(C << 3);  // 2*C*sizeof(spx_mem_t)
    st->adapted = 0;
    st->Pey = st->Pyy = FLOAT_ONE;

#ifdef TWO_PATH
    st->Davg1 = st->Davg2 = 0;
    st->Dvar1 = st->Dvar2 = FLOAT_ZERO;
#endif

#ifdef USE_THOSE_PARAMS
    st->play_buf = (spx_int16_t *)speex_alloc((K * (PLAYBACK_DELAY + 1) * frame_size) << 1);
    st->play_buf_pos = PLAYBACK_DELAY * frame_size;
    st->play_buf_started = 0;
#endif

    st->ps16AmpRate = (spx_int16_t *)speex_alloc(nb_mic << 1);
    for (i = 0; i < C; i++)
        st->ps16AmpRate[i] = 1;

    st->s32disable_echo_smooth = 0;
    st->s32disable_dc_filter = 0;
    st->s32disable_leak_estimate = 0;

    if (!st->ps16AmpRate) return 0;

    return st;
}

/** Resets echo canceller state */
EXPORT void speex_echo_state_reset(SpeexEchoState *st)
{
    int i, NM, N, C, K;
    st->cancel_count = 0;
    st->screwed_up = 0;
    N = st->window_size;
    NM = N * st->M;
    C = st->C;
    K = st->K;
    // for (i=0;i<N*M;i++)
    //     st->W[i] = 0;
    memset(st->W, 0, NM << 2);
#ifdef TWO_PATH
    // for (i=0;i<N*M;i++)
    //     st->foreground[i] = 0;
    memset(st->foreground, 0, NM << 1);
#endif
    // for (i=0;i<N*(M+1);i++)
    //     st->X[i] = 0;
    memset(st->X, 0, (NM + N) << 1);

    {
        int *ps32des = (int *)&FLOAT_ONE;
        int *ps32src = (int *)st->power_1;
        int size = (st->frame_size + 1) << 2;

        for (i = 0; i <= st->frame_size; i++) {
            // st->power[i] = 0;
            // st->power_1[i] = FLOAT_ONE;
            ps32src[i] = *ps32des;
            // st->Eh[i] = 0;
            // st->Yh[i] = 0;
        }
        memset(st->power, 0, size);
        memset(st->Eh, 0, size);
        memset(st->Yh, 0, size);
    }

    /*for (i=0;i<st->frame_size;i++)
    {
        st->last_y[i] = 0;
    }*/
    memset(st->last_y, 0, st->frame_size << 1);
    /*for (i=0;i<N*C;i++)
    {
        st->E[i] = 0;
    }*/
    memset(st->E, 0, (N * C) << 1);
    /*for (i=0;i<N*K;i++)
    {
        st->x[i] = 0;
    }*/
    memset(st->x, 0, (N * K) << 1);
    /*for (i=0;i<2*C;i++)
        st->notch_mem[i] = 0;*/
    memset(st->notch_mem, 0, (2 * C) << 2);
    /*for (i=0;i<C;i++)
        st->memD[i]=st->memE[i]=0;
    for (i=0;i<K;i++)
        st->memX[i]=0;*/
    memset(st->memD, 0, C << 1);
    memset(st->memE, 0, C << 1);
    memset(st->memX, 0, K << 1);

    st->saturated = 0;
    st->adapted = 0;
    st->sum_adapt = 0;
    st->Pey = st->Pyy = FLOAT_ONE;
#ifdef TWO_PATH
    st->Davg1 = st->Davg2 = 0;
    st->Dvar1 = st->Dvar2 = FLOAT_ZERO;
#endif
#ifdef USE_THOSE_PARAMS
    /*for (i=0;i<3*st->frame_size;i++)
        st->play_buf[i] = 0;*/
    memset(st->play_buf, 0, (3 * st->frame_size) << 1);
    st->play_buf_pos = PLAYBACK_DELAY * st->frame_size;
    st->play_buf_started = 0;
#endif
}

#if 0
/** Destroys an echo canceller state */
EXPORT void speex_echo_state_destroy(void *state)
{
    SpeexEchoState * st = (SpeexEchoState *)state;

    spx_fft_destroy(st->fft_table);

    speex_free(st->e);
    speex_free(st->x);
    speex_free(st->input);
    speex_free(st->y);
    speex_free(st->last_y);
    speex_free(st->Yf);
    speex_free(st->Rf);
    speex_free(st->Xf);
    speex_free(st->Yh);
    speex_free(st->Eh);

    speex_free(st->X);
    speex_free(st->Y);
    speex_free(st->E);
    speex_free(st->W);
#ifdef TWO_PATH
    speex_free(st->foreground);
#endif
    speex_free(st->PHI);
    speex_free(st->power);
    speex_free(st->power_1);
    speex_free(st->window);
    speex_free(st->prop);
    speex_free(st->wtmp);
#ifdef FIXED_POINT
    speex_free(st->wtmp2);
#endif
    speex_free(st->memX);
    speex_free(st->memD);
    speex_free(st->memE);
    speex_free(st->notch_mem);

    speex_free(st->play_buf);
    speex_free(st);

#ifdef DUMP_ECHO_CANCEL_DATA
    fclose(rFile);
    fclose(pFile);
    fclose(oFile);
    rFile = pFile = oFile = NULL;
#endif
}

EXPORT void speex_echo_capture(SpeexEchoState *st, const spx_int16_t *rec, spx_int16_t *out)
{
    int i;
    /*speex_warning_int("capture with fill level ", st->play_buf_pos/st->frame_size);*/
    st->play_buf_started = 1;
    if (st->play_buf_pos>=st->frame_size)
    {
        speex_echo_cancellation(st, rec, st->play_buf, out);
        st->play_buf_pos -= st->frame_size;
        for (i=0;i<st->play_buf_pos;i++)
            st->play_buf[i] = st->play_buf[i+st->frame_size];
    } else {
        speex_warning("No playback frame available (your application is buggy and/or got xruns)");
        if (st->play_buf_pos!=0)
        {
            speex_warning("internal playback buffer corruption?");
            st->play_buf_pos = 0;
        }
        for (i=0;i<st->frame_size;i++)
            out[i] = rec[i];
    }
}

EXPORT void speex_echo_playback(SpeexEchoState *st, const spx_int16_t *play)
{
    /*speex_warning_int("playback with fill level ", st->play_buf_pos/st->frame_size);*/
    if (!st->play_buf_started)
    {
        speex_warning("discarded first playback frame");
        return;
    }
    if (st->play_buf_pos<=PLAYBACK_DELAY*st->frame_size)
    {
        int i;
        for (i=0;i<st->frame_size;i++)
            st->play_buf[st->play_buf_pos+i] = play[i];
        st->play_buf_pos += st->frame_size;
        if (st->play_buf_pos <= (PLAYBACK_DELAY-1)*st->frame_size)
        {
            speex_warning("Auto-filling the buffer (your application is buggy and/or got xruns)");
            for (i=0;i<st->frame_size;i++)
                st->play_buf[st->play_buf_pos+i] = play[i];
            st->play_buf_pos += st->frame_size;
        }
    } else {
        speex_warning("Had to discard a playback frame (your application is buggy and/or got xruns)");
    }
}

/** Performs echo cancellation on a frame (deprecated, last arg now ignored) */
EXPORT void speex_echo_cancel(SpeexEchoState *st, const spx_int16_t *in, const spx_int16_t *far_end, spx_int16_t *out, spx_int32_t *Yout)
{
    speex_echo_cancellation(st, in, far_end, out);
}
#endif

#ifdef _speex_echo_cancellation_OPT
/** Performs echo cancellation on a frame */

#ifndef WIN32
static __attribute__((noinline)) void leak_estimate(SpeexEchoState *st, spx_word32_t Syy, spx_word32_t See, spx_float_t Pey, spx_float_t Pyy)
#else
static void leak_estimate(SpeexEchoState *st, spx_word32_t Syy, spx_word32_t See, spx_float_t Pey, spx_float_t Pyy)
#endif
{
#ifndef _DISABLE_LEAK_ESTIMATE
    if (!st->s32disable_leak_estimate) {
        int tmp32;
        spx_float_t alpha, alpha_1;
        spx_float_t res;
        int *ps32res = (int *)&res;

        Pyy = FLOAT_SQRT(Pyy);
        Pey = FLOAT_DIVU(Pey, Pyy);

        /* Compute correlation updatete rate */
        tmp32 = NTK_MULT16_32_Q15(st->beta0, Syy);
        tmp32 = MIN32(tmp32, NTK_MULT16_32_Q15(st->beta_max, See));

        *ps32res = FLOAT_DIV32_INT(tmp32, See);
        alpha = res;
        alpha_1 = FLOAT_SUB(FLOAT_ONE, alpha);
        /* Update correlations (recursive average) */
        st->Pey = FLOAT_ADD(FLOAT_MULT(alpha_1, st->Pey), FLOAT_MULT(alpha, Pey));
        st->Pyy = FLOAT_ADD(FLOAT_MULT(alpha_1, st->Pyy), FLOAT_MULT(alpha, Pyy));
        if (FLOAT_LT(st->Pyy, FLOAT_ONE))
            st->Pyy = FLOAT_ONE;
        /* We don't really hope to get better than 33 dB (MIN_LEAK-3dB) attenuation anyway */
        res = FLOAT_MULT(MIN_LEAK, st->Pyy);
        if (FLOAT_LT(st->Pey, res))
            st->Pey = res;
        if (FLOAT_GT(st->Pey, st->Pyy))
            st->Pey = st->Pyy;
        /* leak_estimate is the linear regression result */
        *ps32res = FLOAT_SHL_INT(FLOAT_DIVU(st->Pey, st->Pyy), 14);
        st->leak_estimate = FLOAT_EXTRACT16(res);
        /* This looks like a stupid bug, but it's right (because we convert from Q14 to Q15) */
        st->leak_estimate = NTK_SHL16(st->leak_estimate, 1);
    }
#endif
}

EXPORT void speex_echo_cancellation(void *state, spx_int16_t *in, spx_int16_t *far_end, spx_int16_t *out, PST_AUD_AEC_PRELOAD pstAecPreload)
{
    int i, j, chan, speak;
    int N, M, C, K;
    spx_word32_t Syy, See, Sxx, Sdd, Sff;
#ifdef TWO_PATH
    spx_word32_t Dbf;
    int update_foreground;
#endif
    spx_word32_t Sey;
    spx_word16_t ss, ss_1;
    spx_word16_t RER;
    SpeexEchoState *st = (SpeexEchoState *)state;
    int frame_size = st->frame_size;
    int NK, MNK, MNKC, offset, offset2, offset3;
    s16 *ptr_x = st->x, *ptr_X = st->X;
    s16 *foreground = st->foreground;
    s16 preemph = st->preemph;
    s32 saturated = 0;

    N = st->window_size;
    M = st->M;
    C = st->C;
    K = st->K;
    NK = N * K;
    MNK = M * NK;
    MNKC = MNK * C;

    pstAecPreload->ps16Foreground = foreground;
    pstAecPreload->u32ForegroundSize = MNKC << 1;
    pstAecPreload->ps32Background = st->W;
    pstAecPreload->u32BackgroundSize = MNKC << 2;

    st->cancel_count++;
#ifdef FIXED_POINT
    ss = DIV32_16(11469, M);
    ss_1 = 32767 - ss;
#else
    ss = .35 / M;
    ss_1 = 1 - ss;
#endif

    offset = 0;

    //#ifdef    _DISABLE_DC_FILTER
    if (st->s32disable_dc_filter) {
        for (chan = 0; chan < C; chan++) {
            s16 memD = st->memD[chan];
            s16 AmpRate = st->ps16AmpRate[chan];
            s16 *input = st->input + offset;
            s16 *pin = in + chan;
            if (!st->s32enable_AR_func)
                AmpRate = 1;
#ifdef _MIPS_speex_echo_cancellation_OPT_NOT_Exactly
            s16 flag = 0;
            /* Copy input data to buffer and apply pre-emphasis */
            for (i = 0; i < frame_size; i++) {
                s16 tmp16;
                tmp16 = SUB16(*pin, MULT16_16_P15(preemph, memD));
                tmp16 = EXTRACT16_FLAG(MULT16_16(tmp16, AmpRate), &flag);
                memD = *pin;
                *input = tmp16;
                input++;
                pin += C;
            }
            if (flag)
                saturated = 1;
#else
            /* Copy input data to buffer and apply pre-emphasis */
            for (i = 0; i < frame_size; i++) {
                /* FIXME: This core has changed a bit, need to merge properly */
                spx_word32_t tmp32;
                int sign;
                tmp32 = SUB32(EXTEND32(*pin), EXTEND32(MULT16_16_P15(preemph, memD)));
                tmp32 = MULT16_16(EXTRACT16(tmp32), AmpRate);
#ifdef FIXED_POINT
                sign = (tmp32) >> 31;
                if (sign != (tmp32) >> (15)) {
                    (tmp32) = sign ^ ((1 << (15)) - 1);
                    saturated = 1;
                }
#endif
                memD = *pin;
                *input = EXTRACT16(tmp32);
                input++;
                pin += C;
            }
#endif
            st->memD[chan] = memD;
            offset += frame_size;
        }
    } else {
        for (chan = 0; chan < C; chan++) {
            s16 memD = st->memD[chan];
            s16 AmpRate = st->ps16AmpRate[chan];
            s16 *input = st->input + offset;
            if (!st->s32enable_AR_func)
                AmpRate = 1;
#ifdef _MIPS_speex_echo_cancellation_OPT_NOT_Exactly
            s16 flag = 0;
            /* Apply a notch filter to make sure DC doesn't end up causing problems */
            filter_dc_notch16(in + chan, st->notch_radius, input, frame_size, st->notch_mem + (chan << 1), C);
            /* Copy input data to buffer and apply pre-emphasis */
            for (i = 0; i < frame_size; i++) {
                s16 tmp16;
                tmp16 = SUB16(*input, MULT16_16_P15(preemph, memD));
                tmp16 = EXTRACT16_FLAG(MULT16_16(tmp16, AmpRate), &flag);
                memD = *input;
                *input = tmp16;
                input++;
            }
            if (flag)
                saturated = 1;
#else
            /* Apply a notch filter to make sure DC doesn't end up causing problems */
            filter_dc_notch16(in + chan, st->notch_radius, input, frame_size, st->notch_mem + (chan << 1), C);
            /* Copy input data to buffer and apply pre-emphasis */
            for (i = 0; i < frame_size; i++) {
                /* FIXME: This core has changed a bit, need to merge properly */
                spx_word32_t tmp32;
                int sign;
                tmp32 = SUB32(EXTEND32(*input), EXTEND32(MULT16_16_P15(preemph, memD)));
                tmp32 = MULT16_16(EXTRACT16(tmp32), AmpRate);
#ifdef FIXED_POINT
                sign = (tmp32) >> 31;
                if (sign != (tmp32) >> (15)) {
                    (tmp32) = sign ^ ((1 << (15)) - 1);
                    saturated = 1;
                }
#endif
                memD = *input;
                *input = EXTRACT16(tmp32);
                input++;
            }
#endif
            st->memD[chan] = memD;
            offset += frame_size;
        }
    }

    if (st->saturated == 0) {
        st->saturated = saturated;
    }

    saturated = st->saturated;
    offset = 0;
    for (speak = 0; speak < K; speak++) {
        s16 memX = st->memX[speak];
        s16 AmpRate = st->ps16AmpRate[0];
        s16 *far = far_end + speak;
        s16 *x0 = ptr_x + offset;
        s16 *x1 = x0 + frame_size;
        if (!st->s32enable_AR_func)
            AmpRate = 1;
#ifdef _MIPS_speex_echo_cancellation_OPT_NOT_Exactly
        s16 flag = 0;
        for (i = 0; i < frame_size; i++) {
            s16 tmp16;
            *x0 = *x1;  // copy signal of last frame.
            tmp16 = SUB16(*far, MULT16_16_P15(preemph, memX));
            tmp16 = EXTRACT16_FLAG(MULT16_16(tmp16, AmpRate), &flag);
            *x1 = tmp16;  // save signal of this frame //x should be half before half now.
            memX = *far;
            far += K;
            x0++;
            x1++;
        }
        if (flag)
            saturated = M + 1;
#else
        for (i = 0; i < frame_size; i++) {
            int sign;
            spx_word32_t tmp32;
            *x0 = *x1;  // copy signal of last frame.
            tmp32 = SUB32(EXTEND32(*far), EXTEND32(MULT16_16_P15(preemph, memX)));
            tmp32 = MULT16_16(EXTRACT16(tmp32), AmpRate);
#ifdef FIXED_POINT
            /*FIXME: If saturation occurs here, we need to freeze adaptation for M frames (not just one) */
            sign = (tmp32) >> 31;
            if (sign != (tmp32) >> (15)) {
                (tmp32) = sign ^ ((1 << (15)) - 1);
                saturated = M + 1;
            }
#endif
            *x1 = EXTRACT16(tmp32);  // save signal of this frame //x should be half before half now.
            memX = *far;
            far += K;
            x0++;
            x1++;
        }
#endif
        st->memX[speak] = memX;
        offset += N;
    }
    st->saturated = saturated;

    offset = 0;
    for (speak = 0; speak < K; speak++) {
        s16 *pX0 = ptr_X + offset + MNK;  // st->X[(j+1)*NK+offset]
        s16 *pX1 = pX0 - NK;              // st->X[j*NK+offset]
        /* Shift memory: this could be optimized eventually*/
        for (j = M - 1; j >= 0; j--) {
            memcpy((void *)pX0, (void *)pX1, N << 1);
            pX0 -= NK;
            pX1 -= NK;
        }
        /* Convert x (echo input) to frequency domain */
        spx_fft(st->fft_table, ptr_x + offset, &ptr_X[offset]);  // transform (M+1)th spectrum
        offset += N;
    }

#if 0
    Sxx = 0;
    offset = 0;
    for (speak = 0; speak < K; speak++)
    {
        s16 *pSignal = ptr_x+offset+frame_size;
        Sxx = ADD32(Sxx, mdf_inner_prod(pSignal, pSignal, frame_size)); //inner product of this frame
        power_spectrum_accum(ptr_X+offset, st->Xf, N);  // |X(f)|^2
        offset += N;
    }
#endif

    {
        Sff = 0;
        offset = 0;
        offset2 = 0;
        offset3 = 0;
        for (chan = 0; chan < C; chan++) {
#ifdef TWO_PATH
#ifdef _MIPS_speex_echo_cancellation_OPT
            s32 *eDes = (s32 *)(st->e + offset);
            s32 *eSrc = eDes + (frame_size >> 1);
            s32 *input = (s32 *)(st->input + offset2);
            /* Compute foreground filter */
            spectral_mul_accum16(ptr_X, foreground + offset3, st->Y + offset, N, M * K);
            spx_ifft(st->fft_table, st->Y + offset, st->e + offset);
            for (i = 0; i < (frame_size >> 2); i++) {
                MIPS_SUBQ_PH(*eDes, *input, *eSrc);
                eDes++;
                input++;
                eSrc++;
                MIPS_SUBQ_PH(*eDes, *input, *eSrc);
                eDes++;
                input++;
                eSrc++;
            }
            Sff = ADD32(Sff, mdf_inner_prod(st->e + offset, st->e + offset, st->frame_size));
            offset += N;
            offset2 += frame_size;
            offset3 += MNK;
#else
            s16 *eDes = st->e + offset;
            s16 *eSrc = eDes + frame_size;
            s16 *input = st->input + offset2;
            /* Compute foreground filter */
            spectral_mul_accum16(ptr_X, foreground + offset3, st->Y + offset, N, M * K);
            spx_ifft(st->fft_table, st->Y + offset, st->e + offset);
            for (i = 0; i < frame_size; i++)
                *eDes++ = SUB16(*input++, *eSrc++);
            Sff = ADD32(Sff, mdf_inner_prod(st->e + offset, st->e + offset, st->frame_size));
            offset += N;
            offset2 += frame_size;
            offset3 += MNK;
#endif
#endif
        }
    }

    /* Adjust proportional adaption rate */
    /* FIXME: Adjust that for C, K*/
    if (st->adapted)
        mdf_adjust_prop(st->W, N, M, C * K, st->prop);
    /* Compute weight gradient */
    if (st->saturated == 0) {
        int *PHI = st->PHI, *PHI_now;
        s16 *prop = st->prop + (M - 1), *prop_now;
        spx_float_t *power_1 = st->power_1;
        offset = 0;
        offset3 = 0;
        for (chan = 0; chan < C; chan++) {
            offset2 = 0;
            for (speak = 0; speak < K; speak++) {
                int *pW = st->W + offset3 + offset2 + MNK - NK;
                s16 *ptr_Xtmp = ptr_X + MNK + offset2;
                prop_now = prop;
                for (j = M - 1; j >= 0; j--) {
                    int *pWtmp = pW;
                    spx_float_t res;
                    int *ps32res = (int *)&res;
                    PHI_now = PHI;

                    *ps32res = FLOAT_SHL_INT(PSEUDOFLOAT(*prop_now--), -15);
                    weighted_spectral_mul_conj(power_1, res, ptr_Xtmp, st->E + offset, PHI, N);
                    for (i = 0; i < (N >> 2); i++) {
                        *pWtmp = ADD32(*pWtmp, *PHI_now);
                        pWtmp++;
                        PHI_now++;  // st->W[offset3 + j*NK + offset2 + i]
                        *pWtmp = ADD32(*pWtmp, *PHI_now);
                        pWtmp++;
                        PHI_now++;
                        *pWtmp = ADD32(*pWtmp, *PHI_now);
                        pWtmp++;
                        PHI_now++;
                        *pWtmp = ADD32(*pWtmp, *PHI_now);
                        pWtmp++;
                        PHI_now++;
                    }
                    pW -= NK;
                    ptr_Xtmp -= NK;
                }
                offset2 += N;
            }
            offset += N;
            offset3 += MNK;
        }
    } else {
        st->saturated--;
    }

    {
        int cancel_count = st->cancel_count;

        offset = 0;
        /* FIXME: MC conversion required */
        /* Update weight to prevent circular convolution (MDF / AUMDF) */
        for (chan = 0; chan < C; chan++) {
            offset2 = 0;
            for (speak = 0; speak < K; speak++) {
                int *pW = st->W + offset + offset2;
                for (j = 0; j < M; j++) {
                    /* This is a variant of the Alternatively Updated MDF (AUMDF) */
                    /* Remove the "if" to make this an MDF filter */
                    if (j == 0 || (cancel_count % (M - 1) == j - 1))  // M=1, j=0      //M=2, j=0,1        //M=3, j=0,1,2  //M=4, j=0,1,2,3
                    {
#ifdef FIXED_POINT
                        int *pWtmp = pW;
                        s16 *ptmp = st->wtmp2;
                        int normalize = NORMALIZE_SCALEDOWN + 16;
                        for (i = 0; i < (N >> 1); i++) {
                            *ptmp = PSHR32(*pWtmp, normalize);
                            ptmp++;
                            pWtmp++;
                            *ptmp = PSHR32(*pWtmp, normalize);
                            ptmp++;
                            pWtmp++;
                        }
                        spx_ifft(st->fft_table, st->wtmp2, st->wtmp);
                        memset(st->wtmp, 0, frame_size << 1);
                        ptmp = st->wtmp + frame_size;

#ifdef _MIPS_speex_echo_cancellation_SHL16_PH_OPT
                        {
                            int *ps32ptr = (int *)ptmp;
                            for (i = 0; i < (frame_size) >> 2; i++) {
                                MIPS_SHL16_PH_3(*ps32ptr, *ps32ptr);
                                ps32ptr++;
                                MIPS_SHL16_PH_3(*ps32ptr, *ps32ptr);
                                ps32ptr++;
                            }
                        }
#else
                        for (i = frame_size; i < N; i++) {
                            *ptmp = NTK_SHL16(*ptmp, NORMALIZE_SCALEUP);
                            ptmp++;
                        }
#endif
                        spx_fft(st->fft_table, st->wtmp, st->wtmp2);
                        /* The "-1" in the shift is a sort of kludge that trades less efficient update speed for decrease noise */
                        pWtmp = pW;
                        ptmp = st->wtmp2;
                        normalize = 16 + NORMALIZE_SCALEDOWN - NORMALIZE_SCALEUP - 1;
                        for (i = 0; i < N; i++) {
                            *pWtmp -= NTK_SHL32(EXTEND32(*ptmp), normalize);
                            pWtmp++;
                            ptmp++;
                        }
#else
                        spx_ifft(st->fft_table, &st->W[offset + j * NK + offset2], st->wtmp);
                        for (i = st->frame_size; i < N; i++) {
                            st->wtmp[i] = 0;
                        }
                        spx_fft(st->fft_table, st->wtmp, &st->W[offset + j * NK + offset2]);
#endif
                    }
                    pW += NK;
                }
                offset2 += N;
            }
            offset += MNK;
        }
    }

    offset = (frame_size + 1) << 2;
    /* So we can use power_spectrum_accum */
    memset(st->Rf, 0, offset);
    memset(st->Yf, 0, offset);
    memset(st->Xf, 0, offset);

    {
        s16 *Py = st->y + frame_size, *Pe = st->e;
        s16 *py, *pe0, *pe1;

        See = 0;
#ifdef TWO_PATH
        Dbf = 0;
        offset = 0;
        offset2 = 0;
        offset3 = 0;
        /* Difference in response, this is used to estimate the variance of our residual power estimate */
        for (chan = 0; chan < C; chan++) {
            spectral_mul_accum(ptr_X, st->W + offset3, st->Y + offset, N, M * K);
            spx_ifft(st->fft_table, st->Y + offset, st->y + offset);
            py = Py + offset;
            pe0 = Pe + offset;
            pe1 = pe0 + frame_size;
#ifdef _MIPS_speex_echo_cancellation_SUBQ_PH_OPT
            {
                int *ps32pe0 = (int *)pe0;
                int *ps32pe1 = (int *)pe1;
                int *ps32py = (int *)py;
                for (i = 0; i < (frame_size >> 2); i++) {
                    MIPS_SUBQ_PH(*ps32pe0, *ps32pe1, *ps32py);
                    ps32pe0++;
                    ps32pe1++;
                    ps32py++;
                    MIPS_SUBQ_PH(*ps32pe0, *ps32pe1, *ps32py);
                    ps32pe0++;
                    ps32pe1++;
                    ps32py++;
                }
            }
#else
            for (i = 0; i < frame_size; i++) {
                *pe0 = SUB16(*pe1, *py);
                pe0++;
                pe1++;
                py++;
            }
#endif
            Dbf = ADD32(Dbf, 10 + mdf_inner_prod(st->e + offset, st->e + offset, frame_size));
            py = Py + offset;
            pe0 = Pe + offset;
            pe1 = st->input + offset2;
#ifdef _MIPS_speex_echo_cancellation_SUBQ_PH_OPT
            {
                int *ps32pe0 = (int *)pe0;
                int *ps32pe1 = (int *)pe1;
                int *ps32py = (int *)py;
                for (i = 0; i < (frame_size >> 2); i++) {
                    MIPS_SUBQ_PH(*ps32pe0, *ps32pe1, *ps32py);
                    ps32pe0++;
                    ps32pe1++;
                    ps32py++;
                    MIPS_SUBQ_PH(*ps32pe0, *ps32pe1, *ps32py);
                    ps32pe0++;
                    ps32pe1++;
                    ps32py++;
                }
            }
#else
            for (i = 0; i < frame_size; i++) {
                *pe0 = SUB16(*pe1, *py);
                pe0++;
                pe1++;
                py++;
            }
#endif
            See = ADD32(See, mdf_inner_prod(st->e + offset, st->e + offset, frame_size));
            offset += N;
            offset2 += frame_size;
            offset3 += MNK;
        }
#endif
    }

#ifndef TWO_PATH
    Sff = See;
#endif

#ifdef TWO_PATH
    {
        int SffSubSee = SUB32(Sff, See);
        int AbsDavg1, AbsDavg2;

        /* Logic for updating the foreground filter */
        {
            spx_float_t res;
            int *ps32rtn = (int *)&res;

            /* For two time windows, compute the mean of the energy difference, as well as the variance */
            st->Davg1 = (NTK_MULT16_32_Q15(QCONST16(.6f, 15), st->Davg1) + NTK_MULT16_32_Q15(QCONST16(.4f, 15), SffSubSee));
            st->Davg2 = (NTK_MULT16_32_Q15(QCONST16(.85f, 15), st->Davg2) + NTK_MULT16_32_Q15(QCONST16(.15f, 15), SffSubSee));
            // st->Dvar1 = FLOAT_ADD(FLOAT_MULT(VAR1_SMOOTH, st->Dvar1), FLOAT_MUL32U(NTK_MULT16_32_Q15(QCONST16(.4f,15),Sff), NTK_MULT16_32_Q15(QCONST16(.4f,15),Dbf)));
            // st->Dvar2 = FLOAT_ADD(FLOAT_MULT(VAR2_SMOOTH, st->Dvar2), FLOAT_MUL32U(NTK_MULT16_32_Q15(QCONST16(.15f,15),Sff), NTK_MULT16_32_Q15(QCONST16(.15f,15),Dbf)));
            *ps32rtn = FLOAT_MUL32U_INT(NTK_MULT16_32_Q15(QCONST16(.4f, 15), Sff), NTK_MULT16_32_Q15(QCONST16(.4f, 15), Dbf));
            st->Dvar1 = FLOAT_ADD(FLOAT_MULT(VAR1_SMOOTH, st->Dvar1), res);
            *ps32rtn = FLOAT_MUL32U_INT(NTK_MULT16_32_Q15(QCONST16(.15f, 15), Sff), NTK_MULT16_32_Q15(QCONST16(.15f, 15), Dbf));
            st->Dvar2 = FLOAT_ADD(FLOAT_MULT(VAR2_SMOOTH, st->Dvar2), res);
        }

        /* Equivalent float code:
        st->Davg1 = .6*st->Davg1 + .4*(Sff-See);
        st->Davg2 = .85*st->Davg2 + .15*(Sff-See);
        st->Dvar1 = .36*st->Dvar1 + .16*Sff*Dbf;
        st->Dvar2 = .7225*st->Dvar2 + .0225*Sff*Dbf;
        */

        AbsDavg1 = ABS32(st->Davg1);
        AbsDavg2 = ABS32(st->Davg2);
        {
            spx_float_t res, res1, res2, res3;
            int *ps32rtn = (int *)&res;
            int *ps32rtn1 = (int *)&res1;
            int *ps32rtn2 = (int *)&res2;
            int *ps32rtn3 = (int *)&res3;

            *ps32rtn = FLOAT_MUL32U_INT(SffSubSee, ABS32(SffSubSee));
            *ps32rtn1 = FLOAT_MUL32U_INT(Sff, Dbf);
            *ps32rtn2 = FLOAT_MUL32U_INT(st->Davg1, AbsDavg1);
            *ps32rtn3 = FLOAT_MUL32U_INT(st->Davg2, AbsDavg2);

            update_foreground = 0;
            /* Check if we have a statistically significant reduction in the residual echo */
            /* Note that this is *not* Gaussian, so we need to be careful about the longer tail */
            if (FLOAT_GT(res, res1))
                update_foreground = 1;
            else if (FLOAT_GT(res2, FLOAT_MULT(VAR1_UPDATE, (st->Dvar1))))
                update_foreground = 1;
            else if (FLOAT_GT(res3, FLOAT_MULT(VAR2_UPDATE, (st->Dvar2))))
                update_foreground = 1;
        }

        /* Do we update? */
        if (update_foreground) {
            s16 *ptmp = foreground;
            int *pW = st->W;

            st->Davg1 = st->Davg2 = 0;
            st->Dvar1 = st->Dvar2 = FLOAT_ZERO;
            /* Copy background filter to foreground filter */
#ifdef _MIPS_speex_echo_cancellation_OPT
            for (i = 0; i < (MNKC >> 2); i++) {
                MIPS_PSHR32_16(*ptmp, *pW);
                pW++;
                ptmp++;
                MIPS_PSHR32_16(*ptmp, *pW);
                pW++;
                ptmp++;
                MIPS_PSHR32_16(*ptmp, *pW);
                pW++;
                ptmp++;
                MIPS_PSHR32_16(*ptmp, *pW);
                pW++;
                ptmp++;
            }
#else
            for (i = 0; i < MNKC; i++) {
                *ptmp = PSHR32(*pW, 16);
                ptmp++;
                pW++;
            }
#endif

            if (!st->s32disable_echo_smooth) {
                /* Apply a smooth transition so as to not introduce blocking artifacts */
                offset = 0;
                for (chan = 0; chan < C; chan++) {
                    s16 *pe = st->e + offset + frame_size;
                    s16 *pWindow0 = st->window;
                    s16 *pWindow1 = pWindow0 + frame_size;
                    ptmp = st->y + offset + frame_size;
                    for (i = 0; i < (st->frame_size >> 2); i++) {
                        *pe = MULT16_16_Q15(*pWindow1, *pe) + MULT16_16_Q15(*pWindow0, *ptmp);
                        pe++;
                        pWindow1++;
                        pWindow0++;
                        ptmp++;
                        *pe = MULT16_16_Q15(*pWindow1, *pe) + MULT16_16_Q15(*pWindow0, *ptmp);
                        pe++;
                        pWindow1++;
                        pWindow0++;
                        ptmp++;
                        *pe = MULT16_16_Q15(*pWindow1, *pe) + MULT16_16_Q15(*pWindow0, *ptmp);
                        pe++;
                        pWindow1++;
                        pWindow0++;
                        ptmp++;
                        *pe = MULT16_16_Q15(*pWindow1, *pe) + MULT16_16_Q15(*pWindow0, *ptmp);
                        pe++;
                        pWindow1++;
                        pWindow0++;
                        ptmp++;
                    }
                    offset += N;
                }
            }

        } else {
            int reset_background = 0;
            spx_float_t res, res1;
            int *ps32rtn = (int *)&res;
            int *ps32rtn1 = (int *)&res1;

            *ps32rtn = FLOAT_MUL32U_INT(NEG32(SffSubSee), ABS32(SffSubSee));
            *ps32rtn1 = FLOAT_MUL32U_INT(Sff, Dbf);

            /* Otherwise, check if the background filter is significantly worse */
            if (FLOAT_GT(res, FLOAT_MULT(VAR_BACKTRACK, res1)))
                reset_background = 1;

            *ps32rtn = FLOAT_MUL32U_INT(NEG32(st->Davg1), AbsDavg1);
            if (FLOAT_GT(res, FLOAT_MULT(VAR_BACKTRACK, st->Dvar1)))
                reset_background = 1;

            *ps32rtn = FLOAT_MUL32U_INT(NEG32(st->Davg2), AbsDavg2);
            if (FLOAT_GT(res, FLOAT_MULT(VAR_BACKTRACK, st->Dvar2)))
                reset_background = 1;

            if (reset_background) {
                s16 *ptmp = foreground;
                int *pW = st->W;
                /* Copy foreground filter to background filter */
                for (i = 0; i < MNKC >> 2; i++) {
                    *pW = NTK_SHL32(EXTEND32(*ptmp), 16);
                    pW++;
                    ptmp++;
                    *pW = NTK_SHL32(EXTEND32(*ptmp), 16);
                    pW++;
                    ptmp++;
                    *pW = NTK_SHL32(EXTEND32(*ptmp), 16);
                    pW++;
                    ptmp++;
                    *pW = NTK_SHL32(EXTEND32(*ptmp), 16);
                    pW++;
                    ptmp++;
                }
                /* We also need to copy the output so as to get correct adaptation */
                offset = 0;
                offset2 = 0;

                for (chan = 0; chan < C; chan++) {
#ifdef _MIPS_speex_echo_cancellation_OPT
                    s32 *ps32py = (s32 *)(st->y + offset + frame_size);
                    s32 *ps32pe0 = (s32 *)(st->e + offset);
                    s32 *ps32pe1 = (s32 *)(st->e + offset + frame_size);
                    s32 *ps32ptmp = (s32 *)(st->input + offset2);
                    for (i = 0; i < (frame_size >> 2); i++) {
                        *ps32py = *ps32pe1;
                        MIPS_SUBQ_PH(*ps32pe0, *ps32ptmp, *ps32py);
                        ps32pe0++;
                        ps32pe1++;
                        ps32ptmp++;
                        ps32py++;
                        *ps32py = *ps32pe1;
                        MIPS_SUBQ_PH(*ps32pe0, *ps32ptmp, *ps32py);
                        ps32pe0++;
                        ps32pe1++;
                        ps32ptmp++;
                        ps32py++;
                    }
#else
                    s16 *pe0, *pe1, *py;

                    py = st->y + offset + frame_size;
                    pe0 = st->e + offset;
                    pe1 = pe0 + frame_size;
                    ptmp = st->input + offset2;
                    for (i = 0; i < frame_size; i++) {
                        *py = *pe1;
                        *pe0 = SUB16(*ptmp, *py);
                        pe0++;
                        pe1++;
                        ptmp++;
                        py++;
                    }
#endif
                    offset += N;
                    offset2 += frame_size;
                }
                See = Sff;
                st->Davg1 = st->Davg2 = 0;
                st->Dvar1 = st->Dvar2 = FLOAT_ZERO;
            }
        }
#endif
    }

    Sey = Syy = Sdd = 0;
    offset = offset2 = 0;
    offset3 = frame_size << 1;
    for (chan = 0; chan < C; chan++) {
        s16 memE = st->memE[chan];
        s16 AmpRate = st->ps16AmpRate[chan];
        s16 *input = st->input + offset2;
        s16 *pe0 = st->e + offset;
        s16 *pe1 = pe0 + frame_size;
        s16 *pIn = in + chan, *pOut = out + chan;
        s16 *py;
        s16 *ps16last_y = st->last_y + offset;
        s16 *ps16last_y2 = ps16last_y + frame_size;
        if (!st->s32enable_AR_func)
            AmpRate = 1;
#ifndef TWO_PATH
        py = st->y + offset + frame_size;
#endif
        memcpy((void *)ps16last_y, (void *)ps16last_y2, frame_size << 1);
        /* Compute error signal (for the output with de-emphasis) */
        for (i = 0; i < frame_size; i++) {
            s16 s16tmp_out = ABS16(*pIn);
            if (s16tmp_out >= 32000) {
                if (st->saturated == 0)
                    st->saturated = 1;
            }
#ifdef _MIPS_speex_echo_cancellation_OPT_NOT_Exactly
#ifdef TWO_PATH
            s16tmp_out = SUB16(*input, *pe1);
            input++;
            pe1++;
#else
            s16tmp_out = SUB16(*input, *py);
            input++;
            py++;
#endif
            s16tmp_out = DIV32_16(s16tmp_out, AmpRate);
            s16tmp_out = ADD16(s16tmp_out, MULT16_16_P15(preemph, memE));
            *ps16last_y2 = *pIn - s16tmp_out;
            *pOut = s16tmp_out;
            memE = s16tmp_out;
#else
        {
            spx_word32_t tmp_out;
#ifdef TWO_PATH
            tmp_out = SUB32(EXTEND32(*input), EXTEND32(*pe1));
            input++;
            pe1++;
#else
            tmp_out = SUB32(EXTEND32(*input), EXTEND32(*py));
            input++;
            py++;
#endif
            tmp_out = DIV32(tmp_out, (s32)AmpRate);
            tmp_out = ADD32(tmp_out, EXTEND32(MULT16_16_P15(preemph, memE)));
            /* This is an arbitrary test for saturation in the microphone signal */
            memE = EXTRACT16(tmp_out);
            *ps16last_y2 = *pIn - memE;
            *pOut = memE;
        }
#endif
            ps16last_y2++;
            pIn += C;
            pOut += C;
        }
        st->memE[chan] = memE;

#ifdef DUMP_ECHO_CANCEL_DATA
        dump_audio(in, far_end, out, st->frame_size);
#endif

        /* Compute error signal (filter update version) */
        memcpy(st->e + offset + frame_size, st->e + offset, frame_size << 1);
        memset(st->e + offset, 0, offset3);

        /* Compute a bunch of correlations */
        /* FIXME: bad merge */
        pe0 = st->e + offset + frame_size;
        py = st->y + offset + frame_size;
        input = st->input + offset2;
        Sey = ADD32(Sey, mdf_inner_prod(pe0, py, frame_size));
        Syy = ADD32(Syy, mdf_inner_prod(py, py, frame_size));
        Sdd = ADD32(Sdd, mdf_inner_prod(input, input, frame_size));

        py -= frame_size;
        pe0 -= frame_size;
        /* Convert error to frequency domain */
        spx_fft(st->fft_table, pe0, st->E + offset);
        memset(py, 0, offset3);
        spx_fft(st->fft_table, py, st->Y + offset);

        /* Compute power spectrum of echo (X), error (E) and filter response (Y) */
        power_spectrum_accum(st->E + offset, st->Rf, N);
        power_spectrum_accum(st->Y + offset, st->Yf, N);

        offset += N;
        offset2 += frame_size;
    }

#if 0
    /* Do some sanity check */
    if (!(Syy>=0 && Sxx>=0 && See >= 0)
#ifndef FIXED_POINT
        || !(Sff < N*1e9 && Syy < N*1e9 && Sxx < N*1e9)
#endif
        )
    {
        /* Things have gone really bad */
        st->screwed_up += 50;
        memset(out,0,(frame_size*C)<<1);
    } else
#endif
    if (SHR32(Sff, 2) > ADD32(Sdd, SHR32(MULT16_16(N, 10000), 6))) {
        /* AEC seems to add lots of echo instead of removing it, let's see if it will improve */
        st->screwed_up++;
    } else {
        /* Everything's fine */
        st->screwed_up = 0;
    }
    if (st->screwed_up >= 50) {
        speex_warning("The echo canceller started acting funny and got slapped (reset). It swears it will behave now.");
        speex_echo_state_reset(st);
        return;
    }

    /* Add a small noise floor to make sure not to have problems when dividing */
    // See = MAX32(See, SHR32(MULT16_16(N, 100),6));
    See = MAX32(See, N);

    Sxx = 0;
    offset = 0;
    for (speak = 0; speak < K; speak++) {
        s16 *ptmp = ptr_x + offset + frame_size;
        Sxx = ADD32(Sxx, mdf_inner_prod(ptmp, ptmp, frame_size));
        power_spectrum_accum(ptr_X + offset, st->Xf, N);
        offset += N;
    }

    {
        spx_float_t Pey = FLOAT_ONE, Pyy = FLOAT_ONE;
        {
            int *power = st->power + frame_size;
            s16 spec_average = st->spec_average;
            s16 spec_average_2 = 32767 - spec_average;
            spx_float_t Eh, Yh;

            if (st->s32disable_leak_estimate) {
                for (j = frame_size; j >= 0; j--) {
                    *power = (NTK_MULT16_32_Q15(ss_1, *power) + 1 + NTK_MULT16_32_Q15(ss, st->Xf[j]));
                    power--;
                }
            } else {
                /* Compute filtered spectra and (cross-)correlations */
                for (j = frame_size; j >= 0; j--) {
                    /* Smooth far end energy estimate over time */
                    *power = (NTK_MULT16_32_Q15(ss_1, *power) + 1 + NTK_MULT16_32_Q15(ss, st->Xf[j]));
                    power--;

                    Eh = PSEUDOFLOAT(st->Rf[j] - st->Eh[j]);
                    Yh = PSEUDOFLOAT(st->Yf[j] - st->Yh[j]);
                    Pey = FLOAT_ADD(Pey, FLOAT_MULT(Eh, Yh));
                    Pyy = FLOAT_ADD(Pyy, FLOAT_MULT(Yh, Yh));
#ifdef FIXED_POINT
#ifdef _MIPS_speex_echo_cancellation_MAC16_32_Q15_OPT_NOT_Exactly
                    MULT_AC0(spec_average_2, st->Eh[j]);
                    MADD_AC0(spec_average, st->Rf[j]);
                    st->Eh[j] = Shift15_Round_AC0();
                    MULT_AC0(spec_average_2, st->Yh[j]);
                    MADD_AC0(spec_average, st->Yf[j]);
                    st->Yh[j] = Shift15_Round_AC0();
#else
                    st->Eh[j] = NTK_MAC16_32_Q15(NTK_MULT16_32_Q15((spec_average_2), st->Eh[j]), spec_average, st->Rf[j]);
                    st->Yh[j] = NTK_MAC16_32_Q15(NTK_MULT16_32_Q15((spec_average_2), st->Yh[j]), spec_average, st->Yf[j]);
#endif
#else
                st->Eh[j] = (1 - spec_average) * st->Eh[j] + spec_average * st->Rf[j];
                st->Yh[j] = (1 - spec_average) * st->Yh[j] + spec_average * st->Yf[j];
#endif
                }
            }
        }
#if 0
    if(!st->s32disable_leak_estimate)
    {
        int tmp32;   
        spx_float_t alpha, alpha_1;
        spx_float_t res;
        int *ps32res = (int *)&res;

        Pyy = FLOAT_SQRT(Pyy);
        Pey = FLOAT_DIVU(Pey,Pyy);

        /* Compute correlation updatete rate */
        tmp32 = NTK_MULT16_32_Q15(st->beta0,Syy);
        tmp32 = MIN32(tmp32,NTK_MULT16_32_Q15(st->beta_max,See));

        *ps32res = FLOAT_DIV32_INT(tmp32, See);
        alpha = res;
        alpha_1 = FLOAT_SUB(FLOAT_ONE, alpha);
        /* Update correlations (recursive average) */
        st->Pey = FLOAT_ADD(FLOAT_MULT(alpha_1,st->Pey) , FLOAT_MULT(alpha,Pey));
        st->Pyy = FLOAT_ADD(FLOAT_MULT(alpha_1,st->Pyy) , FLOAT_MULT(alpha,Pyy));
        if (FLOAT_LT(st->Pyy, FLOAT_ONE))
            st->Pyy = FLOAT_ONE;
        /* We don't really hope to get better than 33 dB (MIN_LEAK-3dB) attenuation anyway */
        res = FLOAT_MULT(MIN_LEAK,st->Pyy);
        if (FLOAT_LT(st->Pey, res))
            st->Pey = res;
        if (FLOAT_GT(st->Pey, st->Pyy))
            st->Pey = st->Pyy;
        /* leak_estimate is the linear regression result */
        *ps32res = FLOAT_SHL_INT(FLOAT_DIVU(st->Pey, st->Pyy),14);
        st->leak_estimate = FLOAT_EXTRACT16(res);
        /* This looks like a stupid bug, but it's right (because we convert from Q14 to Q15) */
        st->leak_estimate = NTK_SHL16(st->leak_estimate,1);
    }

#else

    leak_estimate(st, Syy, See, Pey, Pyy);

#endif
    }

    {
        int tmp32;
        spx_float_t res;
        int *ps32res = (int *)&res;
        /* Compute Residual to Error Ratio */
#ifdef FIXED_POINT
        tmp32 = NTK_MULT16_32_Q15(st->leak_estimate, Syy);
        tmp32 = ADD32(SHR32(Sxx, 13), ADD32(tmp32, NTK_SHL32(tmp32, 1)));
        /* Check for y in e (lower bound on RER) */
        {
            spx_float_t bound = PSEUDOFLOAT(Sey);
            bound = FLOAT_DIVU(FLOAT_MULT(bound, bound), PSEUDOFLOAT(ADD32(1, Syy)));
            if (FLOAT_GT(bound, PSEUDOFLOAT(See)))
                tmp32 = See;
            else if (tmp32 < FLOAT_EXTRACT32(bound))
                tmp32 = FLOAT_EXTRACT32(bound);
        }
        tmp32 = MIN32(tmp32, SHR32(See, 1));

        *ps32res = FLOAT_DIV32_INT(tmp32, See);
        *ps32res = FLOAT_SHL_INT(res, 15);
        RER = FLOAT_EXTRACT16(res);
#else
    RER = (.0001 * Sxx + 3. * NTK_MULT16_32_Q15(st->leak_estimate, Syy)) / See;
    /* Check for y in e (lower bound on RER) */
    if (RER < Sey * Sey / (1 + See * Syy))
        RER = Sey * Sey / (1 + See * Syy);
    if (RER > .5)
        RER = .5;
#endif
    }

    /* We consider that the filter has had minimal adaptation if the following is true*/
    // if (!st->adapted && st->sum_adapt > NTK_SHL32(EXTEND32(M),15) && NTK_MULT16_32_Q15(st->leak_estimate,Syy) > NTK_MULT16_32_Q15(QCONST16(.03f,15),Syy))
    if (!st->adapted && st->sum_adapt > NTK_SHL32(EXTEND32(M), 15) && st->leak_estimate > QCONST16(.03f, 15)) {
        st->adapted = 1;
    }

    if (st->adapted) {
        // spx_float_t *pfloat = st->power_1;
        int *ps32des = (int *)st->power_1;
        int *pint = st->power;
        /* Normal learning rate calculation once we're past the minimal adaptation phase */
        offset = WEIGHT_SHIFT + 16;
        for (i = 0; i <= frame_size; i++) {
            spx_word32_t r, e;
            spx_float_t res;
            int *ps32rtn = (int *)&res;
            /* Compute frequency-domain adaptation mask */
            r = NTK_MULT16_32_Q15(st->leak_estimate, NTK_SHL32(st->Yf[i], 3));
            e = NTK_SHL32(st->Rf[i], 3) + 1;
#ifdef FIXED_POINT
            r = MIN32(r, SHR32(e, 1));
#else
        if (r > .5 * e)
            r = .5 * e;
#endif
            r = NTK_MULT16_32_Q15(QCONST16(.7, 15), r) + NTK_MULT16_32_Q15(QCONST16(.3, 15), NTK_MULT16_32_Q15(RER, e));
            /*st->power_1[i] = adapt_rate*r/(e*(1+st->power[i]));*/
            //*pfloat = FLOAT_SHL(FLOAT_DIV32_FLOAT(r,FLOAT_MUL32U(e,(*pint)+10)),offset);
            *ps32rtn = FLOAT_MUL32U_INT(e, (*pint) + 10);
            *ps32rtn = FLOAT_DIV32_FLOAT_INT(r, res);
            *ps32des = FLOAT_SHL_INT(res, offset);
            ps32des++;
            pint++;
        }
    } else {
        /* Temporary adaption rate if filter is not yet adapted enough */
        spx_word16_t adapt_rate = 0;
        int *pint = st->power;
        int *ps32des = (int *)st->power_1;
        //        spx_float_t *pfloat = st->power_1;
        spx_float_t res;
        int *ps32res = (int *)&res;

        offset = WEIGHT_SHIFT + 1;
        if (Sxx > SHR32(MULT16_16(N, 1000), 6)) {
            int tmp32 = NTK_MULT16_32_Q15(QCONST16(.25f, 15), Sxx);
#ifdef FIXED_POINT
            tmp32 = MIN32(tmp32, SHR32(See, 2));
#else
        if (tmp32 > .25 * See)
            tmp32 = .25 * See;
#endif
            *ps32res = FLOAT_DIV32_INT(tmp32, See);
            *ps32res = FLOAT_SHL_INT(res, 15);
            adapt_rate = FLOAT_EXTRACT16(res);
        }
        for (i = 0; i <= frame_size; i++) {
            *ps32res = FLOAT_DIV32_INT(EXTEND32(adapt_rate), ADD32(*pint, 10));
            *ps32des = FLOAT_SHL_INT(res, offset);
            ps32des++;
            pint++;
        }

        /* How much have we adapted so far? */
        st->sum_adapt = ADD32(st->sum_adapt, adapt_rate);
    }
}

#else  // No saturate protection        //No support in/out same buffer //No support disable LEAK_ESTIMATE
/** Performs echo cancellation on a frame */
EXPORT void speex_echo_cancellation(void *state, spx_int16_t *in, spx_int16_t *far_end, spx_int16_t *out, PST_AUD_AEC_PRELOAD pstAecPreload)
{
    int i, j, chan, speak;
    int N, M, C, K;
    spx_word32_t Syy, See, Sxx, Sdd, Sff;
#ifdef TWO_PATH
    spx_word32_t Dbf;
    int update_foreground;
#endif
    spx_word32_t Sey;
    spx_word16_t ss, ss_1;
    spx_float_t Pey = FLOAT_ONE, Pyy = FLOAT_ONE;
    spx_float_t alpha, alpha_1;
    spx_word16_t RER;
    spx_word32_t tmp32;
    SpeexEchoState *st = (SpeexEchoState *)state;

    N = st->window_size;
    M = st->M;
    C = st->C;
    K = st->K;

    pstAecPreload->ps16Foreground = st->foreground;
    pstAecPreload->u32ForegroundSize = (M * N * K * C) << 1;
    pstAecPreload->ps32Background = st->W;
    pstAecPreload->u32BackgroundSize = (M * N * K * C) << 2;

    st->cancel_count++;
#ifdef FIXED_POINT
    ss = DIV32_16(11469, M);
    ss_1 = SUB16(32767, ss);
#else
    ss = .35 / M;
    ss_1 = 1 - ss;
#endif

    for (chan = 0; chan < C; chan++) {
        s16 s16AmpRate = st->ps16AmpRate[chan];
        if (!st->s32enable_AR_func)
            s16AmpRate = 1;
        /* Apply a notch filter to make sure DC doesn't end up causing problems */
        filter_dc_notch16(in + chan, st->notch_radius, st->input + chan * st->frame_size, st->frame_size, st->notch_mem + 2 * chan, C);
        /* Copy input data to buffer and apply pre-emphasis */
        /* Copy input data to buffer */
        for (i = 0; i < st->frame_size; i++) {
            spx_word32_t tmp32;
            /* FIXME: This core has changed a bit, need to merge properly */
            tmp32 = SUB32(EXTEND32(st->input[chan * st->frame_size + i]), EXTEND32(MULT16_16_P15(st->preemph, st->memD[chan])));
            tmp32 = MULT16_16((spx_word16_t)tmp32, s16AmpRate);
#ifdef FIXED_POINT
            if (tmp32 > 32767) {
                tmp32 = 32767;
                if (st->saturated == 0)
                    st->saturated = 1;
            }
            if (tmp32 < -32767) {
                tmp32 = -32767;
                if (st->saturated == 0)
                    st->saturated = 1;
            }
#endif
            st->memD[chan] = st->input[chan * st->frame_size + i];
            st->input[chan * st->frame_size + i] = EXTRACT16(tmp32);
        }
    }

    for (speak = 0; speak < K; speak++) {
        s16 s16AmpRate = st->ps16AmpRate[chan];
        if (!st->s32enable_AR_func)
            s16AmpRate = 1;
        for (i = 0; i < st->frame_size; i++) {
            spx_word32_t tmp32;
            st->x[speak * N + i] = st->x[speak * N + i + st->frame_size];  // copy signal of last frame.
            tmp32 = SUB32(EXTEND32(far_end[i * K + speak]), EXTEND32(MULT16_16_P15(st->preemph, st->memX[speak])));
            tmp32 = MULT16_16((spx_word16_t)tmp32, s16AmpRate);
#ifdef FIXED_POINT
            /*FIXME: If saturation occurs here, we need to freeze adaptation for M frames (not just one) */
            if (tmp32 > 32767) {
                tmp32 = 32767;
                st->saturated = M + 1;
            }
            if (tmp32 < -32767) {
                tmp32 = -32767;
                st->saturated = M + 1;
            }
#endif
            st->x[speak * N + i + st->frame_size] = EXTRACT16(tmp32);  // save signal of this frame //x should be half before half now.
            st->memX[speak] = far_end[i * K + speak];
        }
    }

    for (speak = 0; speak < K; speak++) {
        /* Shift memory: this could be optimized eventually*/
        for (j = M - 1; j >= 0; j--) {
            for (i = 0; i < N; i++)
                st->X[(j + 1) * N * K + speak * N + i] = st->X[j * N * K + speak * N + i];  // save previous M spectrum
        }
        /* Convert x (echo input) to frequency domain */
        spx_fft(st->fft_table, st->x + speak * N, &st->X[speak * N]);  // transform (M+1)th spectrum
    }

    Sxx = 0;
    for (speak = 0; speak < K; speak++) {
        Sxx += mdf_inner_prod(st->x + speak * N + st->frame_size, st->x + speak * N + st->frame_size, st->frame_size);  // inner product of this frame
        power_spectrum_accum(st->X + speak * N, st->Xf, N);                                                             // |X(f)|^2
    }

    Sff = 0;
    for (chan = 0; chan < C; chan++) {
#ifdef TWO_PATH
        /* Compute foreground filter */
        spectral_mul_accum16(st->X, st->foreground + chan * N * K * M, st->Y + chan * N, N, M * K);
        spx_ifft(st->fft_table, st->Y + chan * N, st->e + chan * N);
        for (i = 0; i < st->frame_size; i++)
            st->e[chan * N + i] = SUB16(st->input[chan * st->frame_size + i], st->e[chan * N + i + st->frame_size]);
        Sff += mdf_inner_prod(st->e + chan * N, st->e + chan * N, st->frame_size);
#endif
    }

    /* Adjust proportional adaption rate */
    /* FIXME: Adjust that for C, K*/
    if (st->adapted)
        mdf_adjust_prop(st->W, N, M, C * K, st->prop);
    /* Compute weight gradient */
    if (st->saturated == 0) {
        for (chan = 0; chan < C; chan++) {
            for (speak = 0; speak < K; speak++) {
                for (j = M - 1; j >= 0; j--) {
                    weighted_spectral_mul_conj(st->power_1, FLOAT_SHL(PSEUDOFLOAT(st->prop[j]), -15), &st->X[(j + 1) * N * K + speak * N], st->E + chan * N, st->PHI, N);
                    for (i = 0; i < N; i++)
                        st->W[chan * N * K * M + j * N * K + speak * N + i] += st->PHI[i];
                }
            }
        }
    } else {
        st->saturated--;
    }

    /* FIXME: MC conversion required */
    /* Update weight to prevent circular convolution (MDF / AUMDF) */
    for (chan = 0; chan < C; chan++) {
        for (speak = 0; speak < K; speak++) {
            for (j = 0; j < M; j++) {
                /* This is a variant of the Alternatively Updated MDF (AUMDF) */
                /* Remove the "if" to make this an MDF filter */
                if (j == 0 || st->cancel_count % (M - 1) == j - 1) {
#ifdef FIXED_POINT
                    for (i = 0; i < N; i++)
                        st->wtmp2[i] = EXTRACT16(PSHR32(st->W[chan * N * K * M + j * N * K + speak * N + i], NORMALIZE_SCALEDOWN + 16));
                    spx_ifft(st->fft_table, st->wtmp2, st->wtmp);
                    for (i = 0; i < st->frame_size; i++) {
                        st->wtmp[i] = 0;
                    }
                    for (i = st->frame_size; i < N; i++) {
                        st->wtmp[i] = NTK_SHL16(st->wtmp[i], NORMALIZE_SCALEUP);
                    }
                    spx_fft(st->fft_table, st->wtmp, st->wtmp2);
                    /* The "-1" in the shift is a sort of kludge that trades less efficient update speed for decrease noise */
                    for (i = 0; i < N; i++)
                        st->W[chan * N * K * M + j * N * K + speak * N + i] -= NTK_SHL32(EXTEND32(st->wtmp2[i]), 16 + NORMALIZE_SCALEDOWN - NORMALIZE_SCALEUP - 1);
#else
                    spx_ifft(st->fft_table, &st->W[chan * N * K * M + j * N * K + speak * N], st->wtmp);
                    for (i = st->frame_size; i < N; i++) {
                        st->wtmp[i] = 0;
                    }
                    spx_fft(st->fft_table, st->wtmp, &st->W[chan * N * K * M + j * N * K + speak * N]);
#endif
                }
            }
        }
    }

    /* So we can use power_spectrum_accum */
    for (i = 0; i <= st->frame_size; i++)
        st->Rf[i] = st->Yf[i] = st->Xf[i] = 0;

    See = 0;
#ifdef TWO_PATH
    Dbf = 0;
    /* Difference in response, this is used to estimate the variance of our residual power estimate */
    for (chan = 0; chan < C; chan++) {
        spectral_mul_accum(st->X, st->W + chan * N * K * M, st->Y + chan * N, N, M * K);
        spx_ifft(st->fft_table, st->Y + chan * N, st->y + chan * N);
        for (i = 0; i < st->frame_size; i++)
            st->e[chan * N + i] = SUB16(st->e[chan * N + i + st->frame_size], st->y[chan * N + i + st->frame_size]);
        Dbf += 10 + mdf_inner_prod(st->e + chan * N, st->e + chan * N, st->frame_size);
        for (i = 0; i < st->frame_size; i++)
            st->e[chan * N + i] = SUB16(st->input[chan * st->frame_size + i], st->y[chan * N + i + st->frame_size]);
        See += mdf_inner_prod(st->e + chan * N, st->e + chan * N, st->frame_size);
    }
#endif

#ifndef TWO_PATH
    Sff = See;
#endif

#ifdef TWO_PATH
    /* Logic for updating the foreground filter */

    /* For two time windows, compute the mean of the energy difference, as well as the variance */
    st->Davg1 = ADD32(NTK_MULT16_32_Q15(QCONST16(.6f, 15), st->Davg1), NTK_MULT16_32_Q15(QCONST16(.4f, 15), SUB32(Sff, See)));
    st->Davg2 = ADD32(NTK_MULT16_32_Q15(QCONST16(.85f, 15), st->Davg2), NTK_MULT16_32_Q15(QCONST16(.15f, 15), SUB32(Sff, See)));
    st->Dvar1 = FLOAT_ADD(FLOAT_MULT(VAR1_SMOOTH, st->Dvar1), FLOAT_MUL32U(NTK_MULT16_32_Q15(QCONST16(.4f, 15), Sff), NTK_MULT16_32_Q15(QCONST16(.4f, 15), Dbf)));
    st->Dvar2 = FLOAT_ADD(FLOAT_MULT(VAR2_SMOOTH, st->Dvar2), FLOAT_MUL32U(NTK_MULT16_32_Q15(QCONST16(.15f, 15), Sff), NTK_MULT16_32_Q15(QCONST16(.15f, 15), Dbf)));

    /* Equivalent float code:
    st->Davg1 = .6*st->Davg1 + .4*(Sff-See);
    st->Davg2 = .85*st->Davg2 + .15*(Sff-See);
    st->Dvar1 = .36*st->Dvar1 + .16*Sff*Dbf;
    st->Dvar2 = .7225*st->Dvar2 + .0225*Sff*Dbf;
    */

    update_foreground = 0;
    /* Check if we have a statistically significant reduction in the residual echo */
    /* Note that this is *not* Gaussian, so we need to be careful about the longer tail */
    if (FLOAT_GT(FLOAT_MUL32U(SUB32(Sff, See), ABS32(SUB32(Sff, See))), FLOAT_MUL32U(Sff, Dbf)))
        update_foreground = 1;
    else if (FLOAT_GT(FLOAT_MUL32U(st->Davg1, ABS32(st->Davg1)), FLOAT_MULT(VAR1_UPDATE, (st->Dvar1))))
        update_foreground = 1;
    else if (FLOAT_GT(FLOAT_MUL32U(st->Davg2, ABS32(st->Davg2)), FLOAT_MULT(VAR2_UPDATE, (st->Dvar2))))
        update_foreground = 1;

    /* Do we update? */
    if (update_foreground) {
        st->Davg1 = st->Davg2 = 0;
        st->Dvar1 = st->Dvar2 = FLOAT_ZERO;
        /* Copy background filter to foreground filter */
        for (i = 0; i < N * M * C * K; i++) {
            st->foreground[i] = EXTRACT16(PSHR32(st->W[i], 16));
        }
#ifndef _DISABLE_ECHO_SMOOTH
        /* Apply a smooth transition so as to not introduce blocking artifacts */
        for (chan = 0; chan < C; chan++)
            for (i = 0; i < st->frame_size; i++)
                st->e[chan * N + i + st->frame_size] = MULT16_16_Q15(st->window[i + st->frame_size], st->e[chan * N + i + st->frame_size]) + MULT16_16_Q15(st->window[i], st->y[chan * N + i + st->frame_size]);
#endif
    } else {
        int reset_background = 0;
        /* Otherwise, check if the background filter is significantly worse */
        if (FLOAT_GT(FLOAT_MUL32U(NEG32(SUB32(Sff, See)), ABS32(SUB32(Sff, See))), FLOAT_MULT(VAR_BACKTRACK, FLOAT_MUL32U(Sff, Dbf))))
            reset_background = 1;
        if (FLOAT_GT(FLOAT_MUL32U(NEG32(st->Davg1), ABS32(st->Davg1)), FLOAT_MULT(VAR_BACKTRACK, st->Dvar1)))
            reset_background = 1;
        if (FLOAT_GT(FLOAT_MUL32U(NEG32(st->Davg2), ABS32(st->Davg2)), FLOAT_MULT(VAR_BACKTRACK, st->Dvar2)))
            reset_background = 1;
        if (reset_background) {
            /* Copy foreground filter to background filter */
            for (i = 0; i < N * M * C * K; i++)
                st->W[i] = NTK_SHL32(EXTEND32(st->foreground[i]), 16);
            /* We also need to copy the output so as to get correct adaptation */
            for (chan = 0; chan < C; chan++) {
                for (i = 0; i < st->frame_size; i++)
                    st->y[chan * N + i + st->frame_size] = st->e[chan * N + i + st->frame_size];
                for (i = 0; i < st->frame_size; i++)
                    st->e[chan * N + i] = SUB16(st->input[chan * st->frame_size + i], st->y[chan * N + i + st->frame_size]);
            }
            See = Sff;
            st->Davg1 = st->Davg2 = 0;
            st->Dvar1 = st->Dvar2 = FLOAT_ZERO;
        }
    }
#endif

    Sey = Syy = Sdd = 0;
    for (chan = 0; chan < C; chan++) {
        s16 s16AmpRate = st->ps16AmpRate[chan];
        if (!st->s32enable_AR_func)
            s16AmpRate = 1;
        /* Compute error signal (for the output with de-emphasis) */
        for (i = 0; i < st->frame_size; i++) {
            spx_word32_t tmp_out;
#ifdef TWO_PATH
            tmp_out = SUB32(EXTEND32(st->input[chan * st->frame_size + i]), EXTEND32(st->e[chan * N + i + st->frame_size]));
#else
            tmp_out = SUB32(EXTEND32(st->input[chan * st->frame_size + i]), EXTEND32(st->y[chan * N + i + st->frame_size]));
#endif
            tmp_out = DIV32_16(tmp_out, s16AmpRate);
            tmp_out = ADD32(tmp_out, EXTEND32(MULT16_16_P15(st->preemph, st->memE[chan])));
            /* This is an arbitrary test for saturation in the microphone signal */
            if (in[i * C + chan] <= -32000 || in[i * C + chan] >= 32000) {
                if (st->saturated == 0)
                    st->saturated = 1;
            }
            out[i * C + chan] = EXTRACT16(tmp_out);
            st->memE[chan] = tmp_out;
        }

#ifdef DUMP_ECHO_CANCEL_DATA
        dump_audio(in, far_end, out, st->frame_size);
#endif

        /* Compute error signal (filter update version) */
        for (i = 0; i < st->frame_size; i++) {
            st->e[chan * N + i + st->frame_size] = st->e[chan * N + i];
            st->e[chan * N + i] = 0;
        }

        /* Compute a bunch of correlations */
        /* FIXME: bad merge */
        Sey += mdf_inner_prod(st->e + chan * N + st->frame_size, st->y + chan * N + st->frame_size, st->frame_size);
        Syy += mdf_inner_prod(st->y + chan * N + st->frame_size, st->y + chan * N + st->frame_size, st->frame_size);
        Sdd += mdf_inner_prod(st->input + chan * st->frame_size, st->input + chan * st->frame_size, st->frame_size);

        /* Convert error to frequency domain */
        spx_fft(st->fft_table, st->e + chan * N, st->E + chan * N);
        for (i = 0; i < st->frame_size; i++)
            st->y[i + chan * N] = 0;
        spx_fft(st->fft_table, st->y + chan * N, st->Y + chan * N);

        /* Compute power spectrum of echo (X), error (E) and filter response (Y) */
        power_spectrum_accum(st->E + chan * N, st->Rf, N);
        power_spectrum_accum(st->Y + chan * N, st->Yf, N);
    }

    /*printf ("%f %f %f %f\n", Sff, See, Syy, Sdd, st->update_cond);*/

    /* Do some sanity check */
    if (!(Syy >= 0 && Sxx >= 0 && See >= 0)
#ifndef FIXED_POINT
        || !(Sff < N * 1e9 && Syy < N * 1e9 && Sxx < N * 1e9)
#endif
    ) {
        /* Things have gone really bad */
        st->screwed_up += 50;
        memset(out, 0, (st->frame_size * C) << 1);
    } else if (SHR32(Sff, 2) > ADD32(Sdd, SHR32(MULT16_16(N, 10000), 6))) {
        /* AEC seems to add lots of echo instead of removing it, let's see if it will improve */
        st->screwed_up++;
    } else {
        /* Everything's fine */
        st->screwed_up = 0;
    }
    if (st->screwed_up >= 50) {
        speex_warning("The echo canceller started acting funny and got slapped (reset). It swears it will behave now.");
        speex_echo_state_reset(st);
        return;
    }

    /* Add a small noise floor to make sure not to have problems when dividing */
    See = MAX32(See, SHR32(MULT16_16(N, 100), 6));

    for (speak = 0; speak < K; speak++) {
        Sxx += mdf_inner_prod(st->x + speak * N + st->frame_size, st->x + speak * N + st->frame_size, st->frame_size);
        power_spectrum_accum(st->X + speak * N, st->Xf, N);
    }

    /* Smooth far end energy estimate over time */
    for (j = 0; j <= st->frame_size; j++)
        st->power[j] = NTK_MULT16_32_Q15(ss_1, st->power[j]) + 1 + NTK_MULT16_32_Q15(ss, st->Xf[j]);

    /* Compute filtered spectra and (cross-)correlations */
    for (j = st->frame_size; j >= 0; j--) {
        spx_float_t Eh, Yh;
        Eh = PSEUDOFLOAT(st->Rf[j] - st->Eh[j]);
        Yh = PSEUDOFLOAT(st->Yf[j] - st->Yh[j]);
        Pey = FLOAT_ADD(Pey, FLOAT_MULT(Eh, Yh));
        Pyy = FLOAT_ADD(Pyy, FLOAT_MULT(Yh, Yh));
#ifdef FIXED_POINT
        st->Eh[j] = NTK_MAC16_32_Q15(NTK_MULT16_32_Q15(SUB16(32767, st->spec_average), st->Eh[j]), st->spec_average, st->Rf[j]);
        st->Yh[j] = NTK_MAC16_32_Q15(NTK_MULT16_32_Q15(SUB16(32767, st->spec_average), st->Yh[j]), st->spec_average, st->Yf[j]);
#else
        st->Eh[j] = (1 - st->spec_average) * st->Eh[j] + st->spec_average * st->Rf[j];
        st->Yh[j] = (1 - st->spec_average) * st->Yh[j] + st->spec_average * st->Yf[j];
#endif
    }

    Pyy = FLOAT_SQRT(Pyy);
    Pey = FLOAT_DIVU(Pey, Pyy);

    /* Compute correlation updatete rate */
    tmp32 = NTK_MULT16_32_Q15(st->beta0, Syy);
    if (tmp32 > NTK_MULT16_32_Q15(st->beta_max, See))
        tmp32 = NTK_MULT16_32_Q15(st->beta_max, See);
    alpha = FLOAT_DIV32(tmp32, See);
    alpha_1 = FLOAT_SUB(FLOAT_ONE, alpha);
    /* Update correlations (recursive average) */
    st->Pey = FLOAT_ADD(FLOAT_MULT(alpha_1, st->Pey), FLOAT_MULT(alpha, Pey));
    st->Pyy = FLOAT_ADD(FLOAT_MULT(alpha_1, st->Pyy), FLOAT_MULT(alpha, Pyy));
    if (FLOAT_LT(st->Pyy, FLOAT_ONE))
        st->Pyy = FLOAT_ONE;
    /* We don't really hope to get better than 33 dB (MIN_LEAK-3dB) attenuation anyway */
    if (FLOAT_LT(st->Pey, FLOAT_MULT(MIN_LEAK, st->Pyy)))
        st->Pey = FLOAT_MULT(MIN_LEAK, st->Pyy);
    if (FLOAT_GT(st->Pey, st->Pyy))
        st->Pey = st->Pyy;
    /* leak_estimate is the linear regression result */
    st->leak_estimate = FLOAT_EXTRACT16(FLOAT_SHL(FLOAT_DIVU(st->Pey, st->Pyy), 14));
    /* This looks like a stupid bug, but it's right (because we convert from Q14 to Q15) */
    if (st->leak_estimate > 16383)
        st->leak_estimate = 32767;
    else
        st->leak_estimate = NTK_SHL16(st->leak_estimate, 1);
        /*printf ("%f\n", st->leak_estimate);*/

        /* Compute Residual to Error Ratio */
#ifdef FIXED_POINT
    tmp32 = NTK_MULT16_32_Q15(st->leak_estimate, Syy);
    tmp32 = ADD32(SHR32(Sxx, 13), ADD32(tmp32, NTK_SHL32(tmp32, 1)));
    /* Check for y in e (lower bound on RER) */
    {
        spx_float_t bound = PSEUDOFLOAT(Sey);
        bound = FLOAT_DIVU(FLOAT_MULT(bound, bound), PSEUDOFLOAT(ADD32(1, Syy)));
        if (FLOAT_GT(bound, PSEUDOFLOAT(See)))
            tmp32 = See;
        else if (tmp32 < FLOAT_EXTRACT32(bound))
            tmp32 = FLOAT_EXTRACT32(bound);
    }
    if (tmp32 > SHR32(See, 1))
        tmp32 = SHR32(See, 1);
    RER = FLOAT_EXTRACT16(FLOAT_SHL(FLOAT_DIV32(tmp32, See), 15));
#else
    RER = (.0001 * Sxx + 3. * NTK_MULT16_32_Q15(st->leak_estimate, Syy)) / See;
    /* Check for y in e (lower bound on RER) */
    if (RER < Sey * Sey / (1 + See * Syy))
        RER = Sey * Sey / (1 + See * Syy);
    if (RER > .5)
        RER = .5;
#endif

    /* We consider that the filter has had minimal adaptation if the following is true*/
    if (!st->adapted && st->sum_adapt > NTK_SHL32(EXTEND32(M), 15) && NTK_MULT16_32_Q15(st->leak_estimate, Syy) > NTK_MULT16_32_Q15(QCONST16(.03f, 15), Syy)) {
        st->adapted = 1;
    }

    if (st->adapted) {
        /* Normal learning rate calculation once we're past the minimal adaptation phase */
        for (i = 0; i <= st->frame_size; i++) {
            spx_word32_t r, e;
            /* Compute frequency-domain adaptation mask */
            r = NTK_MULT16_32_Q15(st->leak_estimate, NTK_SHL32(st->Yf[i], 3));
            e = NTK_SHL32(st->Rf[i], 3) + 1;
#ifdef FIXED_POINT
            if (r > SHR32(e, 1))
                r = SHR32(e, 1);
#else
            if (r > .5 * e)
                r = .5 * e;
#endif
            r = NTK_MULT16_32_Q15(QCONST16(.7, 15), r) + NTK_MULT16_32_Q15(QCONST16(.3, 15), (spx_word32_t)(NTK_MULT16_32_Q15(RER, e)));
            /*st->power_1[i] = adapt_rate*r/(e*(1+st->power[i]));*/
            st->power_1[i] = FLOAT_SHL(FLOAT_DIV32_FLOAT(r, FLOAT_MUL32U(e, st->power[i] + 10)), WEIGHT_SHIFT + 16);
        }
    } else {
        /* Temporary adaption rate if filter is not yet adapted enough */
        spx_word16_t adapt_rate = 0;

        if (Sxx > SHR32(MULT16_16(N, 1000), 6)) {
            tmp32 = NTK_MULT16_32_Q15(QCONST16(.25f, 15), Sxx);
#ifdef FIXED_POINT
            if (tmp32 > SHR32(See, 2))
                tmp32 = SHR32(See, 2);
#else
            if (tmp32 > .25 * See)
                tmp32 = .25 * See;
#endif
            adapt_rate = FLOAT_EXTRACT16(FLOAT_SHL(FLOAT_DIV32(tmp32, See), 15));
        }
        for (i = 0; i <= st->frame_size; i++)
            st->power_1[i] = FLOAT_SHL(FLOAT_DIV32(EXTEND32(adapt_rate), ADD32(st->power[i], 10)), WEIGHT_SHIFT + 1);

        /* How much have we adapted so far? */
        st->sum_adapt = ADD32(st->sum_adapt, adapt_rate);
    }

    for (chan = 0; chan < C; chan++) {
        /* FIXME: MC conversion required */
        for (i = 0; i < st->frame_size; i++)
            st->last_y[chan * N + i] = st->last_y[chan * N + st->frame_size + i];
        if (st->adapted) {
            /* If the filter is adapted, take the filtered echo */
            for (i = 0; i < st->frame_size; i++)
                st->last_y[chan * N + st->frame_size + i] = in[C * i + chan] - out[C * i + chan];
        } else {
            /* If filter isn't adapted yet, all we can do is take the far end signal directly */
            /* moved earlier: for (i=0;i<N;i++)
            st->last_y[i] = st->x[i];*/
        }
    }
}
#endif

/* Compute spectrum of estimated echo for use in an echo post-filter */
#if (defined(_MIPS_speex_echo_get_residual_OPT_NOT_Exactly) || defined(_MIPS_speex_echo_get_residual_OPT))
void speex_echo_get_residual(SpeexEchoState *st, spx_word32_t *residual_echo, int len, int chan_index)
{
    int i;
    spx_word16_t leak2;
    int N;
    int *ptrD;

    N = st->window_size;
#ifdef _MIPS_speex_echo_get_residual_OPT_NOT_Exactly
    {
        s32 *ps32window = (s32 *)st->window;
        s32 *ps32last_y = (s32 *)(st->last_y + chan_index * N);
        s32 *ps32y = (s32 *)st->y;
        for (i = 0; i < (N >> 1); i++) {
            // st->y[i] = MULT16_16_Q15(st->window[i],st->last_y[chan_index*N+i]);
            *ps32y = MULT16_16_P15_PH(*ps32window, *ps32last_y);
            // MIPS_MULQ_RS_PH(*ps32y, *ps32window, *ps32last_y);
            ps32window++;
            ps32last_y++;
            ps32y++;
        }
    }
#else
    {
        /* Apply hanning window (should pre-compute it)*/
        s16 *ptrA, *ptrB, *ptrC;
        ptrA = st->window;
        ptrB = st->last_y + chan_index * N;
        ptrC = st->y;
        for (i = 0; i < N; i++) {
            // st->y[i] = MULT16_16_Q15(st->window[i],st->last_y[chan_index*N+i]);
            MULT_AC0(*ptrA, *ptrB);
#ifdef _MIPS_speex_echo_get_residual_OPT_NOT_Exactly
            *ptrC = (s16)Shift15_Round_AC0();
#else
            SHILO15_AC0();
            *ptrC = (s16)MFLO_AC0();
#endif
            ptrA++;
            ptrB++;
            ptrC++;
        }
    }
#endif

    /* Compute power spectrum of the echo */
    spx_fft(st->fft_table, st->y, st->Y);
    power_spectrum(st->Y, residual_echo, N);

#ifdef FIXED_POINT
    leak2 = NTK_SHL16(st->leak_estimate, 1);
#else
    if (st->leak_estimate > .5)
        leak2 = 1;
    else
        leak2 = 2 * st->leak_estimate;
#endif

    /* Estimate residual echo */
    ptrD = residual_echo;
    for (i = 0; i <= st->frame_size; i++) {
        // residual_echo[i] = (spx_int32_t)NTK_MULT16_32_Q15(leak2,residual_echo[i]);
        MULT_AC0(leak2, *ptrD);
#ifdef _MIPS_speex_echo_get_residual_OPT_NOT_Exactly
        *ptrD = Shift15_Round_AC0();
#else
        SHILO15_AC0();
        *ptrD = MFLO_AC0();
#endif
        ptrD++;
    }
}
#else
void speex_echo_get_residual(SpeexEchoState *st, spx_word32_t *residual_echo, int len, int chan_index)
{
    int i;
    spx_word16_t leak2;
    int N;

    N = st->window_size;

    /* Apply hanning window (should pre-compute it)*/
    for (i = 0; i < N; i++)
        st->y[i] = MULT16_16_Q15(st->window[i], st->last_y[chan_index * N + i]);

    /* Compute power spectrum of the echo */
    spx_fft(st->fft_table, st->y, st->Y);
    power_spectrum(st->Y, residual_echo, N);

#ifdef FIXED_POINT
    if (st->leak_estimate > 16383)
        leak2 = 32767;
    else
        leak2 = NTK_SHL16(st->leak_estimate, 1);
#else
    if (st->leak_estimate > .5)
        leak2 = 1;
    else
        leak2 = 2 * st->leak_estimate;
#endif
    /* Estimate residual echo */
    for (i = 0; i <= st->frame_size; i++)
        residual_echo[i] = (spx_int32_t)NTK_MULT16_32_Q15(leak2, residual_echo[i]);
}
#endif
#if 0
EXPORT int speex_echo_ctl(SpeexEchoState *st, int request, void *ptr)
{
    switch(request)
    {

    case SPEEX_ECHO_GET_FRAME_SIZE:
        (*(int*)ptr) = st->frame_size;
        break;
    case SPEEX_ECHO_SET_SAMPLING_RATE:
        st->sampling_rate = (*(int*)ptr);
        st->spec_average = (spx_word16_t)DIV32(NTK_SHL32(EXTEND32(st->frame_size), 15), st->sampling_rate);
#ifdef FIXED_POINT
        st->beta0 = (spx_word16_t)DIV32(NTK_SHL32(EXTEND32(st->frame_size), 16), st->sampling_rate);
        st->beta_max = (spx_word16_t)DIV32(NTK_SHL32(EXTEND32(st->frame_size), 14), st->sampling_rate);
#else
        st->beta0 = (2.0f*st->frame_size)/st->sampling_rate;
        st->beta_max = (.5f*st->frame_size)/st->sampling_rate;
#endif
        if (st->sampling_rate<12000)
            st->notch_radius = QCONST16(.9, 15);
        else if (st->sampling_rate<24000)
            st->notch_radius = QCONST16(.982, 15);
        else
            st->notch_radius = QCONST16(.992, 15);
        break;
    case SPEEX_ECHO_GET_SAMPLING_RATE:
        (*(int*)ptr) = st->sampling_rate;
        break;
    case SPEEX_ECHO_GET_IMPULSE_RESPONSE_SIZE:
        /*FIXME: Implement this for multiple channels */
        *((spx_int32_t *)ptr) = st->M * st->frame_size;
        break;
    case SPEEX_ECHO_GET_IMPULSE_RESPONSE:
        {
            int M = st->M, N = st->window_size, n = st->frame_size, i, j;
            spx_int32_t *filt = (spx_int32_t *) ptr;
            for(j=0;j<M;j++)
            {
                /*FIXME: Implement this for multiple channels */
#ifdef FIXED_POINT
                for (i=0;i<N;i++)
                    st->wtmp2[i] = EXTRACT16(PSHR32(st->W[j*N+i],16+NORMALIZE_SCALEDOWN));
                spx_ifft(st->fft_table, st->wtmp2, st->wtmp);
#else
                spx_ifft(st->fft_table, &st->W[j*N], st->wtmp);
#endif
                for(i=0;i<n;i++)
                    filt[j*n+i] = PSHR32(MULT16_16(32767,st->wtmp[i]), WEIGHT_SHIFT-NORMALIZE_SCALEDOWN);
            }
        }
        break;
    default:
        speex_warning_int("Unknown speex_echo_ctl request: ", request);
        return -1;
    }
    return 0;
}
#endif
u32 _Aec_SetEchoParams(void *pstEchoState, EN_AUD_AEC_PARAMS enParamsCMD, void *pParamsValue)
{
    SpeexEchoState *pstEchoSt = (SpeexEchoState *)pstEchoState;
    s32 i;
    s32 s32value = *((s32 *)pParamsValue);
    s16 s16value = (s16)s32value;

    switch (enParamsCMD) {
        case EN_AUD_AEC_ENABLE_AR: {
            pstEchoSt->s32enable_AR_func = s32value;
            break;
        }
        case EN_AUD_AEC_AMP_RATE: {
            s32 *ps32value = (s32 *)pParamsValue;
            for (i = 0; i < pstEchoSt->C; i++)
                pstEchoSt->ps16AmpRate[i] = (s16)ps32value[i];
            break;
        }
        case EN_AUD_AEC_PREEMPH: {
            pstEchoSt->preemph = s16value;
            break;
        }
        case EN_AUD_AEC_NOTCH_RADIUS: {
            pstEchoSt->notch_radius = s16value;
            break;
        }
        case EN_AUD_AEC_LEAK_ESTIMATE: {
            pstEchoSt->leak_estimate = s16value;
            break;
        }
        case EN_AUD_AEC_DISABLE_LEAK_ESTIMTAE: {
            pstEchoSt->s32disable_leak_estimate = s32value;
            break;
        }
        case EN_AUD_AEC_DISABLE_DC_FILTER: {
            pstEchoSt->s32disable_dc_filter = s32value;
            break;
        }
        case EN_AUD_AEC_DISABLE_ECHO_SMOOTH: {
            pstEchoSt->s32disable_echo_smooth = s32value;
            break;
        }
        default:
            return 1;
    }
    return 0;
}
