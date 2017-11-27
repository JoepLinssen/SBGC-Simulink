/*
 * sfun_decode.c: Simulink S-function for SBGC API decoding
 * 
 * Joep Linssen, 8 Dec 2017
 */


#define S_FUNCTION_NAME  sfun_decode_nomem
#define S_FUNCTION_LEVEL 2

/*
 * Need to include simstruc.h for the definition of the SimStruct and
 * its associated macro definitions.
 */
#include "simstruc.h"

/*
 * Include the SBGC API
 */
#include "lib/sbgc-api-examples/libraries/SBGC_lib/SBGC.h"

/* Error handling
 * --------------
 *
 * You should use the following technique to report errors encountered within
 * an S-function:
 *
 *       ssSetErrorStatus(S,"Error encountered due to ...");
 *       return;
 *
 * Note that the 2nd argument to ssSetErrorStatus must be persistent memory.
 * It cannot be a local variable. For example the following will cause
 * unpredictable errors:
 *
 *      mdlOutputs()
 *      {
 *         char msg[256];         {ILLEGAL: to fix use "static char msg[256];"}
 *         sprintf(msg,"Error due to %s", string);
 *         ssSetErrorStatus(S,msg);
 *         return;
 *      }
 *
 */

/*====================*
 * S-function methods *
 *====================*/

/* Function: mdlInitializeSizes ===============================================
 * Abstract:
 *    The sizes information is used by Simulink to determine the S-function
 *    block's characteristics (number of inputs, outputs, states, etc.).
 */
static void mdlInitializeSizes(SimStruct *S)
{
    ssSetNumSFcnParams(S, 0);  /* Number of expected parameters */
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) {
        /* Return if number of expected != number of actual parameters */
        return;
    }

    if (!ssSetNumInputPorts(S, 2)) return;
    ssSetInputPortRequiredContiguous(S, 0, true); /*direct input signal access*/
    ssSetInputPortRequiredContiguous(S, 1, true); /*direct input signal access*/
    ssSetInputPortDirectFeedThrough(S, 0, 1); /* input is used in mdlOutputs */
    ssSetInputPortDirectFeedThrough(S, 1, 1); /* input is used in mdlOutputs */
    ssSetInputPortDataType(S, 0, SS_UINT8);
    ssSetInputPortDataType(S, 1, SS_DOUBLE);
    ssSetInputPortVectorDimension(S, 0, SBGC_CMD_MAX_BYTES);
    ssSetInputPortWidth(S, 1, 1);

    if (!ssSetNumOutputPorts(S, 2)) return; /* Two outputs! */
    
    ssSetOutputPortWidth(S, 0, 1); /* 1st output is length 1 */
    ssSetOutputPortWidth(S, 1, 1); /* 2nd output is length 1 */

    ssSetNumSampleTimes(S, 1);

    /* Specify the sim state compliance to be same as a built-in block */
    ssSetSimStateCompliance(S, USE_DEFAULT_SIM_STATE);

    ssSetOptions(S, 0); /* general options (SS_OPTION_xx) */
}



/* Function: mdlInitializeSampleTimes =========================================
 * Abstract:
 *    This function is used to specify the sample time(s) for your
 *    S-function. You must register the same number of sample times as
 *    specified in ssSetNumSampleTimes.
 */
static void mdlInitializeSampleTimes(SimStruct *S)
{
    ssSetSampleTime(S, 0, INHERITED_SAMPLE_TIME);
    ssSetOffsetTime(S, 0, 0.0);
    ssSetModelReferenceSampleTimeDefaultInheritance(S);
}



#undef MDL_INITIALIZE_CONDITIONS   /* Change to #undef to remove function */
#if defined(MDL_INITIALIZE_CONDITIONS)
  /* Function: mdlInitializeConditions ========================================
   * Abstract:
   *    In this function, you should initialize the continuous and discrete
   *    states for your S-function block.  The initial states are placed
   *    in the state vector, ssGetContStates(S) or ssGetRealDiscStates(S).
   *    You can also perform any other initialization activities that your
   *    S-function may require. Note, this routine will be called at the
   *    start of simulation and if it is present in an enabled subsystem
   *    configured to reset states, it will be call when the enabled subsystem
   *    restarts execution to reset the states.
   */
  static void mdlInitializeConditions(SimStruct *S)
  {
  }
#endif /* MDL_INITIALIZE_CONDITIONS */



#undef MDL_START  /* Change to #undef to remove function */
#if defined(MDL_START) 
  /* Function: mdlStart =======================================================
   * Abstract:
   *    This function is called once at start of model execution. If you
   *    have states that should be initialized once, this is the place
   *    to do it.
   */
  static void mdlStart(SimStruct *S)
  {
  }
#endif /*  MDL_START */



/* Function: mdlOutputs =======================================================
 * Abstract:
 *    In this function, you compute the outputs of your S-function
 *    block.
 */
static void mdlOutputs(SimStruct *S, int_T tid)
{ // Tries to decode a full message from the input (without keeping memory of previous inputs / non-complete messages)
    const uint8_T   *uvec     = (uint8_T*) ssGetInputPortSignal(S, 0);
    real_T          *status   = (real_T*) ssGetInputPortSignal(S, 1); //1 if data available, 0 otherwise
    int_T           len_uvec  = ssGetInputPortWidth(S, 0);
    real_T          *y0       = ssGetOutputPortRealSignal(S, 0);
    real_T          *y1       = ssGetOutputPortRealSignal(S, 1);
    

    if(status[0] < 0.5f) {
      y0[0] = -1;
      y1[0] = -1;
      return;
    }

    y0[0] = 0; //preset y0 for if decoding fails
    
    SBGC_Parser sbgc_parser;
    sbgc_parser.init_noCom();
    
    uint16_T numErr_start = sbgc_parser.get_parse_error_count();
    uint8_T done;
    for (int uidx = 0; uidx < len_uvec; uidx++) {
        /* process_char tries to decode a full message.
         * It discards any first character that is not the header char '>', i.e 62
         * It returns 0 while busy.
         * If parse_error_count is increased while processing chars, an error has occured.
         * It returns 1 when done.
         */
        done = sbgc_parser.process_char(uvec[uidx]);

        if(sbgc_parser.get_parse_error_count() > numErr_start) {
            numErr_start = sbgc_parser.get_parse_error_count();
            sbgc_parser.reset();
        }

        if(done > 0) {
          SerialCommand &cmd = sbgc_parser.in_cmd;
          y0[0] = (real_T) cmd.id; 
          break; 
        }
    }

    y1[0] = (real_T) done;
}



#undef MDL_UPDATE  /* Change to #undef to remove function */
#if defined(MDL_UPDATE)
  /* Function: mdlUpdate ======================================================
   * Abstract:
   *    This function is called once for every major integration time step.
   *    Discrete states are typically updated here, but this function is useful
   *    for performing any tasks that should only take place once per
   *    integration step.
   */
  static void mdlUpdate(SimStruct *S, int_T tid)
  {
  }
#endif /* MDL_UPDATE */


/* Function: mdlTerminate =====================================================
 * Abstract:
 *    In this function, you should perform any actions that are necessary
 *    at the termination of a simulation.  For example, if memory was
 *    allocated in mdlStart, this is the place to free it.
 */
static void mdlTerminate(SimStruct *S)
{
}


/*=============================*
 * Required S-function trailer *
 *=============================*/

#ifdef  MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#include "simulink.c"      /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"       /* Code generation registration function */
#endif
