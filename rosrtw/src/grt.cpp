//=================================================================================================
// Copyright (c) 2012, Johannes Meyer, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Flight Systems and Automatic Control group,
//       TU Darmstadt, nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

#include "model.h"
#include "grt.h"

#if GRTINTERFACE == 1
# error "classic call interface is not supported"
#endif

/*==================================================*
 * External functions for Simplified Call Interface *
 *==================================================*/
# define MODEL_INITIALIZE    CONCAT(MODEL,_initialize)
#if ONESTEPFCN == 1
# define MODEL_STEP          CONCAT(MODEL,_step)
# define MODEL_OUTPUT        MODEL_STEP
#else
# define MODEL_OUTPUT        CONCAT(MODEL,_output)
# define MODEL_UPDATE        CONCAT(MODEL,_update)
#endif

# define MODEL_TERMINATE     CONCAT(MODEL,_terminate)
# define RT_MDL              CONCAT(MODEL,_M)

extern void MODEL_INITIALIZE(void);
extern void MODEL_TERMINATE(void);

#if !defined(MULTITASKING)
#if ONESTEPFCN == 1
# define MODEL_UPDATE()              /* No op */
extern void MODEL_STEP(void);        /* single rate step function */
#else
extern void MODEL_OUTPUT(void);      /* single rate output function */
extern void MODEL_UPDATE(void);      /* single rate update function */
#endif
#else
#if ONESTEPFCN == 1
# define MODEL_UPDATE(S)             /* No op */
extern void MODEL_STEP(int_T tid);   /* multirate step function */
#else
extern void MODEL_OUTPUT(int_T tid); /* multirate output function */
extern void MODEL_UPDATE(int_T tid); /* multirate update function */
#endif
#endif  /* !defined(MULTITASKING) */

#ifdef EXT_MODE
#  define rtExtModeSingleTaskUpload(S)                          \
   {                                                            \
        int stIdx;                                              \
        rtExtModeUploadCheckTrigger(rtmGetNumSampleTimes(S));   \
        for (stIdx=0; stIdx<NUMST; stIdx++) {                   \
            if (rtmIsSampleHit(S, stIdx, 0 /*unused*/)) {       \
                rtExtModeUpload(stIdx,rtmGetTaskTime(S,stIdx)); \
            }                                                   \
        }                                                       \
   }
#else
#  define rtExtModeSingleTaskUpload(S) /* Do nothing */
#endif

/*=================*
 * Local functions *
 *=================*/

namespace rosrtw {

#if !defined(MULTITASKING)  /* SINGLETASKING */

/* Function: rtOneStep ========================================================
 *
 * Abstract:
 *      Perform one step of the model. This function is modeled such that
 *      it could be called from an interrupt service routine (ISR) with minor
 *      modifications.
 */
void Model::rt_OneStep(RT_MODEL *S)
{
    real_T tnext;

    /***********************************************
     * Check and see if base step time is too fast *
     ***********************************************/
    if (GBLbuf.isrOverrun++) {
        GBLbuf.stopExecutionFlag = 1;
        return;
    }

    /***********************************************
     * Check and see if error status has been set  *
     ***********************************************/
    if (rtmGetErrorStatus(S) != NULL) {
        GBLbuf.stopExecutionFlag = 1;
        return;
    }

    /* enable interrupts here */

    tnext = rt_SimGetNextSampleHit();
    rtsiSetSolverStopTime(rtmGetRTWSolverInfo(S),tnext);

    MODEL_OUTPUT();

    rtExtModeSingleTaskUpload(S);

    GBLbuf.errmsg = rt_UpdateTXYLogVars(rtmGetRTWLogInfo(S),
                                        rtmGetTPtr(S));
    if (GBLbuf.errmsg != NULL) {
        GBLbuf.stopExecutionFlag = 1;
        return;
    }

    rt_UpdateSigLogVars(rtmGetRTWLogInfo(S), rtmGetTPtr(S));

    MODEL_UPDATE();

    GBLbuf.isrOverrun--;

    rtExtModeCheckEndTrigger();

} /* end rtOneStep */

#else /* MULTITASKING */

# if TID01EQ == 1
#  define FIRST_TID 1
# else
#  define FIRST_TID 0
# endif

/* Function: rtOneStep ========================================================
 *
 * Abstract:
 *      Perform one step of the model. This function is modeled such that
 *      it could be called from an interrupt service routine (ISR) with minor
 *      modifications.
 *
 *      This routine is modeled for use in a multitasking environment and
 *	therefore needs to be fully re-entrant when it is called from an
 *	interrupt service routine.
 *
 * Note:
 *      Error checking is provided which will only be used if this routine
 *      is attached to an interrupt.
 *
 */
void Model::rt_OneStep(RT_MODEL *S)
{
    int_T  i;
    real_T tnext;
    int_T  *sampleHit = rtmGetSampleHitPtr(S);

    /***********************************************
     * Check and see if base step time is too fast *
     ***********************************************/
    if (GBLbuf.isrOverrun++) {
        GBLbuf.stopExecutionFlag = 1;
        return;
    }

    /***********************************************
     * Check and see if error status has been set  *
     ***********************************************/
    if (rtmGetErrorStatus(S) != NULL) {
        GBLbuf.stopExecutionFlag = 1;
        return;
    }
    /* enable interrupts here */

    /***********************************************
     * Update discrete events                      *
     ***********************************************/
    tnext = rt_SimUpdateDiscreteEvents(rtmGetNumSampleTimes(S),
                                       rtmGetTimingData(S),
                                       rtmGetSampleHitPtr(S),
                                       rtmGetPerTaskSampleHitsPtr(S));
    rtsiSetSolverStopTime(rtmGetRTWSolverInfo(S),tnext);
    for (i=FIRST_TID+1; i < NUMST; i++) {
        if (sampleHit[i] && GBLbuf.eventFlags[i]++) {
            GBLbuf.isrOverrun--;
            GBLbuf.overrunFlags[i]++;    /* Are we sampling too fast for */
            GBLbuf.stopExecutionFlag=1;  /*   sample time "i"?           */
            return;
        }
    }
    /*******************************************
     * Step the model for the base sample time *
     *******************************************/
    MODEL_OUTPUT(0);

    rtExtModeUploadCheckTrigger(rtmGetNumSampleTimes(S));
    rtExtModeUpload(FIRST_TID,rtmGetTaskTime(S, FIRST_TID));

    GBLbuf.errmsg = rt_UpdateTXYLogVars(rtmGetRTWLogInfo(S),
                                        rtmGetTPtr(S));
    if (GBLbuf.errmsg != NULL) {
        GBLbuf.stopExecutionFlag = 1;
        return;
    }

    rt_UpdateSigLogVars(rtmGetRTWLogInfo(S), rtmGetTPtr(S));

    MODEL_UPDATE(0);

    /************************************************************************
     * Model step complete for base sample time, now it is okay to          *
     * re-interrupt this ISR.                                               *
     ************************************************************************/

    GBLbuf.isrOverrun--;


    /*********************************************
     * Step the model for any other sample times *
     *********************************************/
    for (i=FIRST_TID+1; i<NUMST; i++) {
        /* If task "i" is running, don't run any lower priority task */
        if (GBLbuf.overrunFlags[i]) return;

        if (GBLbuf.eventFlags[i]) {
            GBLbuf.overrunFlags[i]++;

            MODEL_OUTPUT(i);

            rtExtModeUpload(i, rtmGetTaskTime(S,i));

            MODEL_UPDATE(i);

            /* Indicate task complete for sample time "i" */
            GBLbuf.overrunFlags[i]--;
            GBLbuf.eventFlags[i]--;
        }
    }

    rtExtModeCheckEndTrigger();

} /* end rtOneStep */

#endif /* MULTITASKING */


/*===================*
 * Visible functions *
 *===================*/

bool Model::initialize()
{
  if (is_initialized) return false;

  /************************
   * Initialize the model *
   ************************/
  S = RT_MDL;
  MODEL_INITIALIZE();

  if (finaltime >= 0.0 || finaltime == RUN_FOREVER) rtmSetTFinal(S,finaltime);

#ifdef UseMMIDataLogging
  rt_FillStateSigInfoFromMMI(rtmGetRTWLogInfo(S),&rtmGetErrorStatus(S));
  rt_FillSigLogInfoFromMMI(rtmGetRTWLogInfo(S),&rtmGetErrorStatus(S));
#endif
  GBLbuf.errmsg = rt_StartDataLogging(rtmGetRTWLogInfo(S),
                                      rtmGetTFinal(S),
                                      rtmGetStepSize(S),
                                      &rtmGetErrorStatus(S));
  if (GBLbuf.errmsg != NULL) {
      (void)fprintf(stderr,"Error starting data logging: %s\n",GBLbuf.errmsg);
      return(EXIT_FAILURE);
  }

  rtExtModeCheckInit(rtmGetNumSampleTimes(S));
  rtExtModeWaitForStartPkt(rtmGetRTWExtModeInfo(S),
                           rtmGetNumSampleTimes(S),
                           (boolean_T *)&rtmGetStopRequested(S));

  (void)printf("\n** starting the model **\n");

  is_initialized = true;
  return true;
}

void Model::loop()
{
  if (!is_initialized) return;

  /*************************************************************************
   * Execute the model.  You may attach rtOneStep to an ISR, if so replace *
   * the call to rtOneStep (below) with a call to a background task        *
   * application.                                                          *
   *************************************************************************/
  if (rtmGetTFinal(S) == RUN_FOREVER) {
      printf ("\n**May run forever. Model stop time set to infinity.**\n");
  }

  while(isRunning()) step();
}


bool Model::isRunning()
{
  return (!GBLbuf.stopExecutionFlag &&
           (rtmGetTFinal(S) == RUN_FOREVER ||
            rtmGetTFinal(S)-rtmGetT(S) > rtmGetT(S)*DBL_EPSILON));
}

void Model::step()
{
  rtExtModePauseIfNeeded(rtmGetRTWExtModeInfo(S),
                         rtmGetNumSampleTimes(S),
                         (boolean_T *)&rtmGetStopRequested(S));

  if (rtmGetStopRequested(S)) return;
  /* external mode */
  rtExtModeOneStep(rtmGetRTWExtModeInfo(S),
          rtmGetNumSampleTimes(S),
          (boolean_T *)&rtmGetStopRequested(S));

  rt_OneStep(S);
}

void Model::terminate()
{
  if (!is_initialized) return;

  if (!GBLbuf.stopExecutionFlag && !rtmGetStopRequested(S)) {
      /* external mode */
      rtExtModeOneStep(rtmGetRTWExtModeInfo(S),
              rtmGetNumSampleTimes(S),
              (boolean_T *)&rtmGetStopRequested(S));

      /* Execute model last time step */
      rt_OneStep(S);
  }

  /********************
   * Cleanup and exit *
   ********************/
#ifdef UseMMIDataLogging
  rt_CleanUpForStateLogWithMMI(rtmGetRTWLogInfo(S));
  rt_CleanUpForSigLogWithMMI(rtmGetRTWLogInfo(S));
#endif
  rt_StopDataLogging(MATFILE,rtmGetRTWLogInfo(S));

  rtExtModeShutdown(rtmGetNumSampleTimes(S));

  if (GBLbuf.errmsg) {
      (void)fprintf(stderr,"%s\n",GBLbuf.errmsg);
      exit(EXIT_FAILURE);
  }

  if (rtmGetErrorStatus(S) != NULL) {
      (void)fprintf(stderr,"ErrorStatus set: \"%s\"\n", rtmGetErrorStatus(S));
      exit(EXIT_FAILURE);
  }

  if (GBLbuf.isrOverrun) {
      (void)fprintf(stderr,
                    "%s: ISR overrun - base sampling rate is too fast\n",
                    QUOTE(MODEL));
      exit(EXIT_FAILURE);
  }

#ifdef MULTITASKING
  else {
      int_T i;
      for (i=1; i<NUMST; i++) {
          if (GBLbuf.overrunFlags[i]) {
              (void)fprintf(stderr,
                      "%s ISR overrun - sampling rate is too fast for "
                      "sample time index %d\n", QUOTE(MODEL), i);
              exit(EXIT_FAILURE);
          }
      }
  }
#endif

  MODEL_TERMINATE();

  is_initialized = false;
}

} // namespace rosrtw
