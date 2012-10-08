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

#ifndef RT_MALLOC
# error "grt_malloc_main.c require RT_MALLOC to be defined"
#endif

/*====================*
 * External functions *
 *====================*/
#ifdef __cplusplus

extern "C" {

#endif

extern RT_MODEL *MODEL(void);

#ifdef __cplusplus

}
#endif

#if NCSTATES > 0
#ifdef __cplusplus

extern "C" {

#endif
  extern void rt_ODECreateIntegrationData(RTWSolverInfo *si);
  extern void rt_ODEUpdateContinuousStates(RTWSolverInfo *si);
# if defined(RT_MALLOC)
   extern void rt_ODEDestroyIntegrationData(RTWSolverInfo *si);
# endif
#ifdef __cplusplus

}
#endif
#else
# define rt_ODECreateIntegrationData(si)  \
  rtsiSetSolverName(si, "FixedStepDiscrete");
# define rt_ODEUpdateContinuousStates(si) \
  rtsiSetT(si,rtsiGetSolverStopTime(si))
#endif


/*=============*
 * Global data *
 *=============*/

const char *RT_MEMORY_ALLOCATION_ERROR = "memory allocation error";

/*==================================*
 * Global data local to this module *
 *==================================*/

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

    tnext = rt_SimGetNextSampleHit(rtmGetTimingData(S),
                                   rtmGetNumSampleTimes(S));
    rtsiSetSolverStopTime(rtmGetRTWSolverInfo(S),tnext);

    rtmiOutputs(rtmGetRTWRTModelMethodsInfo(S),0);

    rtExtModeSingleTaskUpload(S);

    GBLbuf.errmsg = rt_UpdateTXYLogVars(rtmGetRTWLogInfo(S),
                                        rtmGetTPtr(S));
    if (GBLbuf.errmsg != NULL) {
        GBLbuf.stopExecutionFlag = 1;
        return;
    }

    rtmiUpdate(rtmGetRTWRTModelMethodsInfo(S),0);

    rt_SimUpdateDiscreteTaskSampleHits(rtmGetNumSampleTimes(S),
                                       rtmGetTimingData(S),
                                       rtmGetSampleHitPtr(S),
                                       rtmGetTPtr(S));

    if (rtmGetSampleTime(S,0) == CONTINUOUS_SAMPLE_TIME) {
        rt_ODEUpdateContinuousStates(rtmGetRTWSolverInfo(S));
    }

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
    rtmiOutputs(rtmGetRTWRTModelMethodsInfo(S),FIRST_TID);

    rtExtModeUploadCheckTrigger(rtmGetNumSampleTimes(S));
    rtExtModeUpload(FIRST_TID,rtmGetTaskTime(S, FIRST_TID));

    GBLbuf.errmsg = rt_UpdateTXYLogVars(rtmGetRTWLogInfo(S),
                                        rtmGetTPtr(S));
    if (GBLbuf.errmsg != NULL) {
        GBLbuf.stopExecutionFlag = 1;
        return;
    }

    rtmiUpdate(rtmGetRTWRTModelMethodsInfo(S),FIRST_TID);

    if (rtmGetSampleTime(S,0) == CONTINUOUS_SAMPLE_TIME) {
        rt_ODEUpdateContinuousStates(rtmGetRTWSolverInfo(S));
    }
     else {
        rt_SimUpdateDiscreteTaskTime(rtmGetTPtr(S),
                                     rtmGetTimingData(S),0);
    }

#if FIRST_TID == 1
    rt_SimUpdateDiscreteTaskTime(rtmGetTPtr(S),
                                 rtmGetTimingData(S),1);
#endif


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

            rtmiOutputs(rtmGetRTWRTModelMethodsInfo(S),i);

            rtExtModeUpload(i, rtmGetTaskTime(S,i));

            rtmiUpdate(rtmGetRTWRTModelMethodsInfo(S),i);

            rt_SimUpdateDiscreteTaskTime(rtmGetTPtr(S),
                                         rtmGetTimingData(S),i);

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
  S = MODEL();
  if (S == NULL) {
      (void)fprintf(stderr,"Memory allocation error during model "
                    "registration");
//      exit(EXIT_FAILURE);
      terminate();
      return false;
  }

  if (rtmGetErrorStatus(S) != NULL) {
      (void)fprintf(stderr,"Error during model registration: %s\n",
                    rtmGetErrorStatus(S));
      rtmiTerminate(rtmGetRTWRTModelMethodsInfo(S));
//      exit(EXIT_FAILURE);
      terminate();
      return false;
  }

  if (finaltime >= 0.0 || finaltime == RUN_FOREVER) rtmSetTFinal(S,finaltime);

  rtmiInitializeSizes(rtmGetRTWRTModelMethodsInfo(S));
  rtmiInitializeSampleTimes(rtmGetRTWRTModelMethodsInfo(S));

  const char *status = 0;
  status = rt_SimInitTimingEngine(rtmGetNumSampleTimes(S),
                                  rtmGetStepSize(S),
                                  rtmGetSampleTimePtr(S),
                                  rtmGetOffsetTimePtr(S),
                                  rtmGetSampleHitPtr(S),
                                  rtmGetSampleTimeTaskIDPtr(S),
                                  rtmGetTStart(S),
                                  &rtmGetSimTimeStep(S),
                                  &rtmGetTimingData(S));

  if (status != NULL) {
      (void)fprintf(stderr,
              "Failed to initialize sample time engine: %s\n", status);
//      exit(EXIT_FAILURE);
      terminate();
      return false;
  }
  rt_ODECreateIntegrationData(rtmGetRTWSolverInfo(S));
#if NCSTATES > 0
  if(rtmGetErrorStatus(S) != NULL) {
      (void)fprintf(stderr, "Error creating integration data.\n");
      rt_ODEDestroyIntegrationData(rtmGetRTWSolverInfo(S));
      rtmiTerminate(rtmGetRTWRTModelMethodsInfo(S));
//      exit(EXIT_FAILURE);
      terminate();
      return false;
  }
#endif

  GBLbuf.errmsg = rt_StartDataLogging(rtmGetRTWLogInfo(S),
                                      rtmGetTFinal(S),
                                      rtmGetStepSize(S),
                                      &rtmGetErrorStatus(S));
  if (GBLbuf.errmsg != NULL) {
      (void)fprintf(stderr,"Error starting data logging: %s\n",GBLbuf.errmsg);
      terminate();
      return false;
  }

  rtExtModeCheckInit(rtmGetNumSampleTimes(S));
  rtExtModeWaitForStartPkt(rtmGetRTWExtModeInfo(S),
                           rtmGetNumSampleTimes(S),
                           (boolean_T *)&rtmGetStopRequested(S));

  (void)printf("\n** starting the model **\n");

  rtmiStart(rtmGetRTWRTModelMethodsInfo(S));
  if (rtmGetErrorStatus(S) != NULL) {
    GBLbuf.stopExecutionFlag = 1;
  }

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
  /* External mode */
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
  rt_StopDataLogging(MATFILE,rtmGetRTWLogInfo(S));
  rtExtModeShutdown(rtmGetNumSampleTimes(S));

  if (GBLbuf.errmsg) {
      (void)fprintf(stderr,"%s\n",GBLbuf.errmsg);
//      exit(EXIT_FAILURE);
  }

  if (GBLbuf.isrOverrun) {
      (void)fprintf(stderr,
                    "%s: ISR overrun - base sampling rate is too fast\n",
                    QUOTE(MODEL));
//      exit(EXIT_FAILURE);
  }

  if (rtmGetErrorStatus(S) != NULL) {
      (void)fprintf(stderr,"%s\n", rtmGetErrorStatus(S));
//      exit(EXIT_FAILURE);
  }
#ifdef MULTITASKING
  else {
      int_T i;
      for (i=1; i<NUMST; i++) {
          if (GBLbuf.overrunFlags[i]) {
              (void)fprintf(stderr,
                      "%s ISR overrun - sampling rate is too fast for "
                      "sample time index %d\n", QUOTE(MODEL), i);
//              exit(EXIT_FAILURE);
          }
      }
  }
#endif

  /* timing data */
  rt_SimDestroyTimingEngine(rtmGetTimingData(S));
#if NCSTATES > 0
  /* integration data */
  rt_ODEDestroyIntegrationData(rtmGetRTWSolverInfo(S));
#endif

  rtmiTerminate(rtmGetRTWRTModelMethodsInfo(S));

  is_initialized = false;
}

} // namespace rosrtw
