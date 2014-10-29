/* Include files */

#include "simiam_AvoidObstacle_BestAO_sfun.h"
#include "simiam_AvoidObstacle_BestAO_sfun_debug_macros.h"
#include "c1_simiam_AvoidObstacle_BestAO.h"
#include "c2_simiam_AvoidObstacle_BestAO.h"

/* Type Definitions */

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */
uint32_T _simiam_AvoidObstacle_BestAOMachineNumber_;
real_T _sfTime_;

/* Function Declarations */

/* Function Definitions */
void simiam_AvoidObstacle_BestAO_initializer(void)
{
}

void simiam_AvoidObstacle_BestAO_terminator(void)
{
}

/* SFunction Glue Code */
unsigned int sf_simiam_AvoidObstacle_BestAO_method_dispatcher(SimStruct
  *simstructPtr, unsigned int chartFileNumber, const char* specsCksum, int_T
  method, void *data)
{
  if (chartFileNumber==1) {
    c1_simiam_AvoidObstacle_BestAO_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  if (chartFileNumber==2) {
    c2_simiam_AvoidObstacle_BestAO_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  return 0;
}

unsigned int sf_simiam_AvoidObstacle_BestAO_process_check_sum_call( int nlhs,
  mxArray * plhs[], int nrhs, const mxArray * prhs[] )
{

#ifdef MATLAB_MEX_FILE

  char commandName[20];
  if (nrhs<1 || !mxIsChar(prhs[0]) )
    return 0;

  /* Possible call to get the checksum */
  mxGetString(prhs[0], commandName,sizeof(commandName)/sizeof(char));
  commandName[(sizeof(commandName)/sizeof(char)-1)] = '\0';
  if (strcmp(commandName,"sf_get_check_sum"))
    return 0;
  plhs[0] = mxCreateDoubleMatrix( 1,4,mxREAL);
  if (nrhs>1 && mxIsChar(prhs[1])) {
    mxGetString(prhs[1], commandName,sizeof(commandName)/sizeof(char));
    commandName[(sizeof(commandName)/sizeof(char)-1)] = '\0';
    if (!strcmp(commandName,"machine")) {
      ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(2369467167U);
      ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(3783382300U);
      ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(1803906823U);
      ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(1456185341U);
    } else if (!strcmp(commandName,"exportedFcn")) {
      ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(0U);
      ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(0U);
      ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(0U);
      ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(0U);
    } else if (!strcmp(commandName,"makefile")) {
      ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(1863456369U);
      ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(2339560858U);
      ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(2139790456U);
      ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(453799875U);
    } else if (nrhs==3 && !strcmp(commandName,"chart")) {
      unsigned int chartFileNumber;
      chartFileNumber = (unsigned int)mxGetScalar(prhs[2]);
      switch (chartFileNumber) {
       case 1:
        {
          extern void sf_c1_simiam_AvoidObstacle_BestAO_get_check_sum(mxArray
            *plhs[]);
          sf_c1_simiam_AvoidObstacle_BestAO_get_check_sum(plhs);
          break;
        }

       case 2:
        {
          extern void sf_c2_simiam_AvoidObstacle_BestAO_get_check_sum(mxArray
            *plhs[]);
          sf_c2_simiam_AvoidObstacle_BestAO_get_check_sum(plhs);
          break;
        }

       default:
        ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(0.0);
        ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(0.0);
        ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(0.0);
        ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(0.0);
      }
    } else if (!strcmp(commandName,"target")) {
      ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(3031367619U);
      ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(4001028638U);
      ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(3978939492U);
      ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(838979348U);
    } else {
      return 0;
    }
  } else {
    ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(746139576U);
    ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(344977028U);
    ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(2199484932U);
    ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(1096353493U);
  }

  return 1;

#else

  return 0;

#endif

}

unsigned int sf_simiam_AvoidObstacle_BestAO_autoinheritance_info( int nlhs,
  mxArray * plhs[], int nrhs, const mxArray * prhs[] )
{

#ifdef MATLAB_MEX_FILE

  char commandName[32];
  char aiChksum[64];
  if (nrhs<3 || !mxIsChar(prhs[0]) )
    return 0;

  /* Possible call to get the autoinheritance_info */
  mxGetString(prhs[0], commandName,sizeof(commandName)/sizeof(char));
  commandName[(sizeof(commandName)/sizeof(char)-1)] = '\0';
  if (strcmp(commandName,"get_autoinheritance_info"))
    return 0;
  mxGetString(prhs[2], aiChksum,sizeof(aiChksum)/sizeof(char));
  aiChksum[(sizeof(aiChksum)/sizeof(char)-1)] = '\0';

  {
    unsigned int chartFileNumber;
    chartFileNumber = (unsigned int)mxGetScalar(prhs[1]);
    switch (chartFileNumber) {
     case 1:
      {
        if (strcmp(aiChksum, "wJBTXHHqYkcNsTqdfjhZ") == 0) {
          extern mxArray
            *sf_c1_simiam_AvoidObstacle_BestAO_get_autoinheritance_info(void);
          plhs[0] = sf_c1_simiam_AvoidObstacle_BestAO_get_autoinheritance_info();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     case 2:
      {
        if (strcmp(aiChksum, "vBU4LUTgZNKcxlVyCat2dB") == 0) {
          extern mxArray
            *sf_c2_simiam_AvoidObstacle_BestAO_get_autoinheritance_info(void);
          plhs[0] = sf_c2_simiam_AvoidObstacle_BestAO_get_autoinheritance_info();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     default:
      plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
    }
  }

  return 1;

#else

  return 0;

#endif

}

unsigned int sf_simiam_AvoidObstacle_BestAO_get_eml_resolved_functions_info( int
  nlhs, mxArray * plhs[], int nrhs, const mxArray * prhs[] )
{

#ifdef MATLAB_MEX_FILE

  char commandName[64];
  if (nrhs<2 || !mxIsChar(prhs[0]))
    return 0;

  /* Possible call to get the get_eml_resolved_functions_info */
  mxGetString(prhs[0], commandName,sizeof(commandName)/sizeof(char));
  commandName[(sizeof(commandName)/sizeof(char)-1)] = '\0';
  if (strcmp(commandName,"get_eml_resolved_functions_info"))
    return 0;

  {
    unsigned int chartFileNumber;
    chartFileNumber = (unsigned int)mxGetScalar(prhs[1]);
    switch (chartFileNumber) {
     case 1:
      {
        extern const mxArray
          *sf_c1_simiam_AvoidObstacle_BestAO_get_eml_resolved_functions_info
          (void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c1_simiam_AvoidObstacle_BestAO_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     case 2:
      {
        extern const mxArray
          *sf_c2_simiam_AvoidObstacle_BestAO_get_eml_resolved_functions_info
          (void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c2_simiam_AvoidObstacle_BestAO_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     default:
      plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
    }
  }

  return 1;

#else

  return 0;

#endif

}

unsigned int sf_simiam_AvoidObstacle_BestAO_third_party_uses_info( int nlhs,
  mxArray * plhs[], int nrhs, const mxArray * prhs[] )
{
  char commandName[64];
  char tpChksum[64];
  if (nrhs<3 || !mxIsChar(prhs[0]))
    return 0;

  /* Possible call to get the third_party_uses_info */
  mxGetString(prhs[0], commandName,sizeof(commandName)/sizeof(char));
  commandName[(sizeof(commandName)/sizeof(char)-1)] = '\0';
  mxGetString(prhs[2], tpChksum,sizeof(tpChksum)/sizeof(char));
  tpChksum[(sizeof(tpChksum)/sizeof(char)-1)] = '\0';
  if (strcmp(commandName,"get_third_party_uses_info"))
    return 0;

  {
    unsigned int chartFileNumber;
    chartFileNumber = (unsigned int)mxGetScalar(prhs[1]);
    switch (chartFileNumber) {
     case 1:
      {
        if (strcmp(tpChksum, "dStdf551LIaPukMIRHzvKF") == 0) {
          extern mxArray
            *sf_c1_simiam_AvoidObstacle_BestAO_third_party_uses_info(void);
          plhs[0] = sf_c1_simiam_AvoidObstacle_BestAO_third_party_uses_info();
          break;
        }
      }

     case 2:
      {
        if (strcmp(tpChksum, "x9pKfdaAvva1Gvs3tOKvIF") == 0) {
          extern mxArray
            *sf_c2_simiam_AvoidObstacle_BestAO_third_party_uses_info(void);
          plhs[0] = sf_c2_simiam_AvoidObstacle_BestAO_third_party_uses_info();
          break;
        }
      }

     default:
      plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
    }
  }

  return 1;
}

unsigned int sf_simiam_AvoidObstacle_BestAO_updateBuildInfo_args_info( int nlhs,
  mxArray * plhs[], int nrhs, const mxArray * prhs[] )
{
  char commandName[64];
  char tpChksum[64];
  if (nrhs<3 || !mxIsChar(prhs[0]))
    return 0;

  /* Possible call to get the updateBuildInfo_args_info */
  mxGetString(prhs[0], commandName,sizeof(commandName)/sizeof(char));
  commandName[(sizeof(commandName)/sizeof(char)-1)] = '\0';
  mxGetString(prhs[2], tpChksum,sizeof(tpChksum)/sizeof(char));
  tpChksum[(sizeof(tpChksum)/sizeof(char)-1)] = '\0';
  if (strcmp(commandName,"get_updateBuildInfo_args_info"))
    return 0;

  {
    unsigned int chartFileNumber;
    chartFileNumber = (unsigned int)mxGetScalar(prhs[1]);
    switch (chartFileNumber) {
     case 1:
      {
        if (strcmp(tpChksum, "dStdf551LIaPukMIRHzvKF") == 0) {
          extern mxArray
            *sf_c1_simiam_AvoidObstacle_BestAO_updateBuildInfo_args_info(void);
          plhs[0] = sf_c1_simiam_AvoidObstacle_BestAO_updateBuildInfo_args_info();
          break;
        }
      }

     case 2:
      {
        if (strcmp(tpChksum, "x9pKfdaAvva1Gvs3tOKvIF") == 0) {
          extern mxArray
            *sf_c2_simiam_AvoidObstacle_BestAO_updateBuildInfo_args_info(void);
          plhs[0] = sf_c2_simiam_AvoidObstacle_BestAO_updateBuildInfo_args_info();
          break;
        }
      }

     default:
      plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
    }
  }

  return 1;
}

void simiam_AvoidObstacle_BestAO_debug_initialize(struct SfDebugInstanceStruct*
  debugInstance)
{
  _simiam_AvoidObstacle_BestAOMachineNumber_ = sf_debug_initialize_machine
    (debugInstance,"simiam_AvoidObstacle_BestAO","sfun",0,2,0,0,0);
  sf_debug_set_machine_event_thresholds(debugInstance,
    _simiam_AvoidObstacle_BestAOMachineNumber_,0,0);
  sf_debug_set_machine_data_thresholds(debugInstance,
    _simiam_AvoidObstacle_BestAOMachineNumber_,0);
}

void simiam_AvoidObstacle_BestAO_register_exported_symbols(SimStruct* S)
{
}

static mxArray* sRtwOptimizationInfoStruct= NULL;
mxArray* load_simiam_AvoidObstacle_BestAO_optimization_info(void)
{
  if (sRtwOptimizationInfoStruct==NULL) {
    sRtwOptimizationInfoStruct = sf_load_rtw_optimization_info(
      "simiam_AvoidObstacle_BestAO", "simiam_AvoidObstacle_BestAO");
    mexMakeArrayPersistent(sRtwOptimizationInfoStruct);
  }

  return(sRtwOptimizationInfoStruct);
}

void unload_simiam_AvoidObstacle_BestAO_optimization_info(void)
{
  if (sRtwOptimizationInfoStruct!=NULL) {
    mxDestroyArray(sRtwOptimizationInfoStruct);
    sRtwOptimizationInfoStruct = NULL;
  }
}
