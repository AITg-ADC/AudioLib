/* Copyright (C) 2003 Epic Games
   Written by Jean-Marc Valin */
/**
 *  @file speex_preprocess.h
 *  @brief Speex preprocessor. The preprocess can do noise suppression, 
 * residual echo suppression (after using the echo canceller), automatic
 * gain control (AGC) and voice activity detection (VAD).
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

#ifndef SPEEX_PREPROCESS_H
#define SPEEX_PREPROCESS_H
/** @defgroup SpeexPreprocessState SpeexPreprocessState: The Speex preprocessor
 *  This is the Speex preprocessor. The preprocess can do noise suppression, 
 * residual echo suppression (after using the echo canceller), automatic
 * gain control (AGC) and voice activity detection (VAD).
 *  @{
 */
#ifdef HAVE_CONFIG_H
#include "config.h"
#endif
	
#include <math.h>
#include "aud_aec_api.h"
#include "speex_types.h"
#include "speex_echo.h"
#include "arch.h"
#include "fftwrap.h"
#include "filterbank.h"
#include "math_approx.h"
#include "os_support.h"

#define NB_BANDS 24
//#define USE_THOSE_PARAMS


#ifdef __cplusplus
extern "C" {
#endif
   
/** State of the preprocessor (one per channel). Should never be accessed directly. */
struct SpeexPreprocessState_;

/** State of the preprocessor (one per channel). Should never be accessed directly. */
typedef struct SpeexPreprocessState_ SpeexPreprocessState;

/** Speex pre-processor state. */
struct SpeexPreprocessState_ {
   /* Basic info */
   int    frame_size;        /**< Number of samples processed each time */
   int    ps_size;           /**< Number of points in the power spectrum */
   int    sampling_rate;     /**< Sampling rate of the input/output */
   int    nbands;
   FilterBank *bank;
   
   /* Parameters */
#ifdef USE_THOSE_PARAMS   
   int    denoise_enabled;
   int    vad_enabled;
   int    dereverb_enabled;
   spx_word16_t  reverb_decay;
   spx_word16_t  reverb_level;
   spx_word16_t speech_prob_start;
   spx_word16_t speech_prob_continue;
#endif   
   int    noise_suppress;
   int    echo_suppress;
   int    echo_suppress_active;
   SpeexEchoState *echo_state;

#ifdef USE_THOSE_PARAMS   
   spx_word16_t	speech_prob;  /**< Probability last frame was speech */
#endif

   /* DSP-related arrays */
   spx_word16_t *frame;      /**< Processing frame (2*ps_size) */
   spx_word16_t *ft;         /**< Processing frame in freq domain (2*ps_size) */
   spx_word32_t *ps;         /**< Current power spectrum */
   spx_word16_t *gain2;      /**< Adjusted gains */
   spx_word16_t *gain_floor; /**< Minimum gain allowed */
   spx_word16_t *window;     /**< Analysis/Synthesis window */
   spx_word32_t *noise;      /**< Noise estimate */
#ifdef USE_THOSE_PARAMS      
   spx_word32_t *reverb_estimate; /**< Estimate of reverb energy */
#endif
   spx_word32_t *old_ps;     /**< Power spectrum for last frame */
   spx_word16_t *gain;       /**< Ephraim Malah gain */
   spx_word16_t *prior;      /**< A-priori SNR */
   spx_word16_t *post;       /**< A-posteriori SNR */

   spx_word32_t *S;          /**< Smoothed power spectrum */
   spx_word32_t *Smin;       /**< See Cohen paper */
   spx_word32_t *Stmp;       /**< See Cohen paper */
   int *update_prob;         /**< Probability of speech presence for noise update */

   spx_word16_t *zeta;       /**< Smoothed a priori SNR */
   spx_word16_t echo_noise_ratio;
   spx_word32_t *echo_noise;
   spx_word32_t *residual_echo;

   /* Misc */
   spx_word16_t *inbuf;      /**< Input buffer (overlapped analysis) */
   spx_word16_t *outbuf;     /**< Output buffer (for overlap and add) */

   /* AGC stuff, only for floating point for now */
#ifndef FIXED_POINT
   int    agc_enabled;
   float  agc_level;
   float  loudness_accum;
   float *loudness_weight;   /**< Perceptual loudness curve */
   float  loudness;          /**< Loudness estimate */
   float  agc_gain;          /**< Current AGC gain */
   float  max_gain;          /**< Maximum gain allowed */
   float  max_increase_step; /**< Maximum increase in gain from one frame to another */
   float  max_decrease_step; /**< Maximum decrease in gain from one frame to another */
   float  prev_loudness;     /**< Loudness of previous frame */
   float  init_max;          /**< Current gain limit during initialisation */
#endif
   int    nb_adapt;          /**< Number of frames used for adaptation so far */
#ifdef USE_THOSE_PARAMS
   int    was_speech;
#endif
   int    min_count;         /**< Number of frames processed so far */
   void  *fft_lookup;        /**< Lookup table for the FFT */
#ifdef FIXED_POINT
   int    frame_shift;
#endif
   int    bank_scale;		//1: linear(best)  0: bark(fast) 
};


/** Creates a new preprocessing state. You MUST create one state per channel processed.
 * @param frame_size Number of samples to process at one time (should correspond to 10-20 ms). Must be
 * the same value as that used for the echo canceller for residual echo cancellation to work.
 * @param sampling_rate Sampling rate used for the input.
 * @return Newly created preprocessor state
*/
SpeexPreprocessState *speex_preprocess_state_init(int frame_size, int sampling_rate, void *pstEchoState);

/** Destroys a preprocessor state 
 * @param st Preprocessor state to destroy
*/
void speex_preprocess_state_destroy(void *state);

/** Preprocess a frame 
 * @param st Preprocessor state
 * @param x Audio sample vector (in and out). Must be same size as specified in speex_preprocess_state_init().
 * @return Bool value for voice activity (1 for speech, 0 for noise/silence), ONLY if VAD turned on.
*/
#ifndef ADD_NLP
int speex_preprocess_run(void *state, spx_int16_t *x, int num_chan, int chan_index);
#else
int speex_preprocess_run(void *state, spx_int16_t *near, spx_int16_t *far, spx_int16_t *x, int num_chan, int chan_index);
#endif

/** Preprocess a frame (deprecated, use speex_preprocess_run() instead)*/
int speex_preprocess(SpeexPreprocessState *st, spx_int16_t *x, spx_int32_t *echo);

/** Update preprocessor state, but do not compute the output
 * @param st Preprocessor state
 * @param x Audio sample vector (in only). Must be same size as specified in speex_preprocess_state_init().
*/
void speex_preprocess_estimate_update(SpeexPreprocessState *st, spx_int16_t *x);

/** Used like the ioctl function to control the preprocessor parameters 
 * @param st Preprocessor state
 * @param request ioctl-type request (one of the SPEEX_PREPROCESS_* macros)
 * @param ptr Data exchanged to-from function
 * @return 0 if no error, -1 if request in unknown
*/
int speex_preprocess_ctl(void *st, int request, void *ptr);



/** Set preprocessor denoiser state */
#define SPEEX_PREPROCESS_SET_DENOISE 0
/** Get preprocessor denoiser state */
#define SPEEX_PREPROCESS_GET_DENOISE 1

/** Set preprocessor Automatic Gain Control state */
#define SPEEX_PREPROCESS_SET_AGC 2
/** Get preprocessor Automatic Gain Control state */
#define SPEEX_PREPROCESS_GET_AGC 3

/** Set preprocessor Voice Activity Detection state */
#define SPEEX_PREPROCESS_SET_VAD 4
/** Get preprocessor Voice Activity Detection state */
#define SPEEX_PREPROCESS_GET_VAD 5

/** Set preprocessor Automatic Gain Control level (float) */
#define SPEEX_PREPROCESS_SET_AGC_LEVEL 6
/** Get preprocessor Automatic Gain Control level (float) */
#define SPEEX_PREPROCESS_GET_AGC_LEVEL 7

/** Set preprocessor dereverb state */
#define SPEEX_PREPROCESS_SET_DEREVERB 8
/** Get preprocessor dereverb state */
#define SPEEX_PREPROCESS_GET_DEREVERB 9

/** Set preprocessor dereverb level */
#define SPEEX_PREPROCESS_SET_DEREVERB_LEVEL 10
/** Get preprocessor dereverb level */
#define SPEEX_PREPROCESS_GET_DEREVERB_LEVEL 11

/** Set preprocessor dereverb decay */
#define SPEEX_PREPROCESS_SET_DEREVERB_DECAY 12
/** Get preprocessor dereverb decay */
#define SPEEX_PREPROCESS_GET_DEREVERB_DECAY 13

/** Set probability required for the VAD to go from silence to voice */
#define SPEEX_PREPROCESS_SET_PROB_START 14
/** Get probability required for the VAD to go from silence to voice */
#define SPEEX_PREPROCESS_GET_PROB_START 15

/** Set probability required for the VAD to stay in the voice state (integer percent) */
#define SPEEX_PREPROCESS_SET_PROB_CONTINUE 16
/** Get probability required for the VAD to stay in the voice state (integer percent) */
#define SPEEX_PREPROCESS_GET_PROB_CONTINUE 17

/** Set maximum attenuation of the noise in dB (negative number) */
#define SPEEX_PREPROCESS_SET_NOISE_SUPPRESS 18
/** Get maximum attenuation of the noise in dB (negative number) */
#define SPEEX_PREPROCESS_GET_NOISE_SUPPRESS 19

/** Set maximum attenuation of the residual echo in dB (negative number) */
#define SPEEX_PREPROCESS_SET_ECHO_SUPPRESS 20
/** Get maximum attenuation of the residual echo in dB (negative number) */
#define SPEEX_PREPROCESS_GET_ECHO_SUPPRESS 21

/** Set maximum attenuation of the residual echo in dB when near end is active (negative number) */
#define SPEEX_PREPROCESS_SET_ECHO_SUPPRESS_ACTIVE 22
/** Get maximum attenuation of the residual echo in dB when near end is active (negative number) */
#define SPEEX_PREPROCESS_GET_ECHO_SUPPRESS_ACTIVE 23

/** Set the corresponding echo canceller state so that residual echo suppression can be performed (NULL for no residual echo suppression) */
#define SPEEX_PREPROCESS_SET_ECHO_STATE 24
/** Get the corresponding echo canceller state */
#define SPEEX_PREPROCESS_GET_ECHO_STATE 25

/** Set maximal gain increase in dB/second (int32) */
#define SPEEX_PREPROCESS_SET_AGC_INCREMENT 26

/** Get maximal gain increase in dB/second (int32) */
#define SPEEX_PREPROCESS_GET_AGC_INCREMENT 27

/** Set maximal gain decrease in dB/second (int32) */
#define SPEEX_PREPROCESS_SET_AGC_DECREMENT 28

/** Get maximal gain decrease in dB/second (int32) */
#define SPEEX_PREPROCESS_GET_AGC_DECREMENT 29

/** Set maximal gain in dB (int32) */
#define SPEEX_PREPROCESS_SET_AGC_MAX_GAIN 30

/** Get maximal gain in dB (int32) */
#define SPEEX_PREPROCESS_GET_AGC_MAX_GAIN 31

/*  Can't set loudness */
/** Get loudness */
#define SPEEX_PREPROCESS_GET_AGC_LOUDNESS 33

/*  Can't set gain */
/** Get current gain (int32 percent) */
#define SPEEX_PREPROCESS_GET_AGC_GAIN 35

/*  Can't set spectrum size */
/** Get spectrum size for power spectrum (int32) */
#define SPEEX_PREPROCESS_GET_PSD_SIZE 37

/*  Can't set power spectrum */
/** Get power spectrum (int32[] of squared values) */
#define SPEEX_PREPROCESS_GET_PSD 39

/*  Can't set noise size */
/** Get spectrum size for noise estimate (int32)  */
#define SPEEX_PREPROCESS_GET_NOISE_PSD_SIZE 41

/*  Can't set noise estimate */
/** Get noise estimate (int32[] of squared values) */
#define SPEEX_PREPROCESS_GET_NOISE_PSD 43

/* Can't set speech probability */
/** Get speech probability in last frame (int32).  */
#define SPEEX_PREPROCESS_GET_PROB 45

/** Set preprocessor Automatic Gain Control level (int32) */
#define SPEEX_PREPROCESS_SET_AGC_TARGET 46
/** Get preprocessor Automatic Gain Control level (int32) */
#define SPEEX_PREPROCESS_GET_AGC_TARGET 47

#ifdef __cplusplus
}
#endif

/** @}*/
#endif
