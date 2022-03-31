/* Copyright (C) Jean-Marc Valin */
/**
   @file speex_echo.h
   @brief Echo cancellation
*/
/*
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

#ifndef SPEEX_ECHO_H
#define SPEEX_ECHO_H
/** @defgroup SpeexEchoState SpeexEchoState: Acoustic echo canceller
 *  This is the acoustic echo canceller module.
 *  @{
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "arch.h"
#include "speex_types.h"
#include "fftwrap.h"
#include "pseudofloat.h"
#include "math_approx.h"
#include "os_support.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Obtain frame size used by the AEC */
#define SPEEX_ECHO_GET_FRAME_SIZE 3

/** Set sampling rate */
#define SPEEX_ECHO_SET_SAMPLING_RATE 24
/** Get sampling rate */
#define SPEEX_ECHO_GET_SAMPLING_RATE 25

/* Can't set window sizes */
/** Get size of impulse response (int32) */
#define SPEEX_ECHO_GET_IMPULSE_RESPONSE_SIZE 27

/* Can't set window content */
/** Get impulse response (int32[]) */
#define SPEEX_ECHO_GET_IMPULSE_RESPONSE 29

/* If enabled, the AEC will use a foreground filter and a background filter to be more robust to double-talk
   and difficult signals in general. The cost is an extra FFT and a matrix-vector multiply */
#define TWO_PATH

#define PLAYBACK_DELAY 2

// #define USE_THOSE_PARAMS

/** Internal echo canceller state. Should never be accessed directly. */
struct SpeexEchoState_;

/** Speex echo cancellation state. */
struct SpeexEchoState_ {
    int frame_size;           /**< Number of samples processed each time */
    int window_size;
    int M;
    int cancel_count;
    int adapted;
    int saturated;
    int screwed_up;
    int C;                    /** Number of input channels (microphones) */
    int K;                    /** Number of output channels (loudspeakers) */
    spx_int32_t sampling_rate;
    spx_word16_t spec_average;
    spx_word16_t beta0;
    spx_word16_t beta_max;
    spx_word32_t sum_adapt;
    spx_word16_t leak_estimate;

    spx_word16_t *e;      /* scratch */
    spx_word16_t *x;      /* Far-end input buffer (2N) */
    spx_word16_t *X;      /* Far-end buffer (M+1 frames) in frequency domain */
    spx_word16_t *input;  /* scratch */
    spx_word16_t *y;      /* scratch */
    spx_word16_t *last_y;
    spx_word16_t *Y;      /* scratch */
    spx_word16_t *E;
    spx_word32_t *PHI;    /* scratch */
    spx_word32_t *W;      /* (Background) filter weights */
#ifdef TWO_PATH
    spx_word16_t *foreground; /* Foreground filter weights */
    spx_word32_t  Davg1;  /* 1st recursive average of the residual power difference */
    spx_word32_t  Davg2;  /* 2nd recursive average of the residual power difference */
    spx_float_t   Dvar1;  /* Estimated variance of 1st estimator */
    spx_float_t   Dvar2;  /* Estimated variance of 2nd estimator */
#endif
    spx_word32_t *power;  /* Power of the far-end signal */
    spx_float_t  *power_1;/* Inverse power of far-end */
    spx_word16_t *wtmp;   /* scratch */
#ifdef FIXED_POINT
    spx_word16_t *wtmp2;  /* scratch */
#endif
    spx_word32_t *Rf;     /* scratch */
    spx_word32_t *Yf;     /* scratch */
    spx_word32_t *Xf;     /* scratch */
    spx_word32_t *Eh;
    spx_word32_t *Yh;
    spx_float_t   Pey;
    spx_float_t   Pyy;
    spx_word16_t *window;
    spx_word16_t *prop;
    void *fft_table;
    spx_word16_t *memX, *memD, *memE;
    spx_word16_t preemph;
    spx_word16_t notch_radius;
    spx_mem_t *notch_mem;

#ifdef USE_THOSE_PARAMS
    /* NOTE: If you only use speex_echo_cancel() and want to save some memory, remove this */
    spx_int16_t *play_buf;
    int play_buf_pos;
    int play_buf_started;
#endif	

    int s32enable_AR_func;
    spx_word16_t *ps16AmpRate; 
	int s32disable_echo_smooth;
	int s32disable_dc_filter;
	int s32disable_leak_estimate;
};

/** @class SpeexEchoState
 * This holds the state of the echo canceller. You need one per channel. 
*/

/** Internal echo canceller state. Should never be accessed directly. */
typedef struct SpeexEchoState_ SpeexEchoState;

/** Creates a new echo canceller state
 * @param frame_size Number of samples to process at one time (should correspond to 10-20 ms)
 * @param filter_length Number of samples of echo to cancel (should generally correspond to 100-500 ms)
 * @return Newly-created echo canceller state
 */
SpeexEchoState *speex_echo_state_init(int frame_size, int filter_length, int nb_mic, int nb_speakers, int sampling_rate, PST_AUD_AEC_PRELOAD pstAecPreload);

/** Creates a new multi-channel echo canceller state
 * @param frame_size Number of samples to process at one time (should correspond to 10-20 ms)
 * @param filter_length Number of samples of echo to cancel (should generally correspond to 100-500 ms)
 * @param nb_mic Number of microphone channels
 * @param nb_speakers Number of speaker channels
 * @return Newly-created echo canceller state
 */
SpeexEchoState *speex_echo_state_init_mc(int frame_size, int filter_length, int nb_mic, int nb_speakers, int sampling_rate, PST_AUD_AEC_PRELOAD pstAecPreload);

/** Destroys an echo canceller state 
 * @param st Echo canceller state
*/
void speex_echo_state_destroy(void *state);

/** Performs echo cancellation a frame, based on the audio sent to the speaker (no delay is added
 * to playback in this form)
 *
 * @param st Echo canceller state
 * @param rec Signal from the microphone (near end + far end echo)
 * @param play Signal played to the speaker (received from far end)
 * @param out Returns near-end signal with echo removed
 */
void speex_echo_cancellation(void *state, spx_int16_t *rec, spx_int16_t *play, spx_int16_t *out, PST_AUD_AEC_PRELOAD pstAecPreload);

/** Performs echo cancellation a frame (deprecated) */
void speex_echo_cancel(SpeexEchoState *st, const spx_int16_t *rec, const spx_int16_t *play, spx_int16_t *out, spx_int32_t *Yout);

/** Perform echo cancellation using internal playback buffer, which is delayed by two frames
 * to account for the delay introduced by most soundcards (but it could be off!)
 * @param st Echo canceller state
 * @param rec Signal from the microphone (near end + far end echo)
 * @param out Returns near-end signal with echo removed
*/
void speex_echo_capture(SpeexEchoState *st, const spx_int16_t *rec, spx_int16_t *out);

/** Let the echo canceller know that a frame was just queued to the soundcard
 * @param st Echo canceller state
 * @param play Signal played to the speaker (received from far end)
*/
void speex_echo_playback(SpeexEchoState *st, const spx_int16_t *play);

/** Reset the echo canceller to its original state 
 * @param st Echo canceller state
 */
void speex_echo_state_reset(SpeexEchoState *st);

/** Used like the ioctl function to control the echo canceller parameters
 *
 * @param st Echo canceller state
 * @param request ioctl-type request (one of the SPEEX_ECHO_* macros)
 * @param ptr Data exchanged to-from function
 * @return 0 if no error, -1 if request in unknown
 */
int speex_echo_ctl(SpeexEchoState *st, int request, void *ptr);



struct SpeexDecorrState_;

typedef struct SpeexDecorrState_ SpeexDecorrState;


/** Create a state for the channel decorrelation algorithm
    This is useful for multi-channel echo cancellation only 
 * @param rate Sampling rate
 * @param channels Number of channels (it's a bit pointless if you don't have at least 2)
 * @param frame_size Size of the frame to process at ones (counting samples *per* channel)
*/
SpeexDecorrState *speex_decorrelate_new(int rate, int channels, int frame_size);

/** Remove correlation between the channels by modifying the phase and possibly
    adding noise in a way that is not (or little) perceptible.
 * @param st Decorrelator state
 * @param in Input audio in interleaved format
 * @param out Result of the decorrelation (out *may* alias in)
 * @param strength How much alteration of the audio to apply from 0 to 100.
*/
void speex_decorrelate(SpeexDecorrState *st, const spx_int16_t *in, spx_int16_t *out, int strength);

/** Destroy a Decorrelation state 
 * @param st State to destroy
*/
void speex_decorrelate_destroy(SpeexDecorrState *st);


#ifdef __cplusplus
}
#endif


/** @}*/
#endif
