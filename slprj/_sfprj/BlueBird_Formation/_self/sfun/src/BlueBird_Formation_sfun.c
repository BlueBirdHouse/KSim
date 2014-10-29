/* Include files */

#include "BlueBird_Formation_sfun.h"
#include "BlueBird_Formation_sfun_debug_macros.h"
#include "c1_BlueBird_Formation.h"
#include "c2_BlueBird_Formation.h"
#include "c3_BlueBird_Formation.h"
#include "c4_BlueBird_Formation.h"
#include "c5_BlueBird_Formation.h"
#include "c6_BlueBird_Formation.h"
#include "c7_BlueBird_Formation.h"
#include "c8_BlueBird_Formation.h"

/* Type Definitions */

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */
uint32_T _BlueBird_FormationMachineNumber_;

/* Function Declarations */

/* Function Definitions */
void BlueBird_Formation_initializer(void)
{
}

void BlueBird_Formation_terminator(void)
{
}

/* SFunction Glue Code */
unsigned int sf_BlueBird_Formation_method_dispatcher(SimStruct *simstructPtr,
  unsigned int chartFileNumber, const char* specsCksum, int_T method, void *data)
{
  if (chartFileNumber==1) {
    c1_BlueBird_Formation_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  if (chartFileNumber==2) {
    c2_BlueBird_Formation_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  if (chartFileNumber==3) {
    c3_BlueBird_Formation_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  if (chartFileNumber==4) {
    c4_BlueBird_Formation_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  if (chartFileNumber==5) {
    c5_BlueBird_Formation_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  if (chartFileNumber==6) {
    c6_BlueBird_Formation_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  if (chartFileNumber==7) {
    c7_BlueBird_Formation_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  if (chartFileNumber==8) {
    c8_BlueBird_Formation_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  return 0;
}

unsigned int sf_BlueBird_Formation_process_check_sum_call( int nlhs, mxArray *
  plhs[], int nrhs, const mxArray * prhs[] )
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
      ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(868228432U);
      ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(4238327026U);
      ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(88228037U);
      ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(3504131860U);
    } else if (!strcmp(commandName,"exportedFcn")) {
      ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(0U);
      ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(0U);
      ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(0U);
      ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(0U);
    } else if (!strcmp(commandName,"makefile")) {
      ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(1736347604U);
      ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(2147714533U);
      ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(3881118695U);
      ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(84985932U);
    } else if (nrhs==3 && !strcmp(commandName,"chart")) {
      unsigned int chartFileNumber;
      chartFileNumber = (unsigned int)mxGetScalar(prhs[2]);
      switch (chartFileNumber) {
       case 1:
        {
          extern void sf_c1_BlueBird_Formation_get_check_sum(mxArray *plhs[]);
          sf_c1_BlueBird_Formation_get_check_sum(plhs);
          break;
        }

       case 2:
        {
          extern void sf_c2_BlueBird_Formation_get_check_sum(mxArray *plhs[]);
          sf_c2_BlueBird_Formation_get_check_sum(plhs);
          break;
        }

       case 3:
        {
          extern void sf_c3_BlueBird_Formation_get_check_sum(mxArray *plhs[]);
          sf_c3_BlueBird_Formation_get_check_sum(plhs);
          break;
        }

       case 4:
        {
          extern void sf_c4_BlueBird_Formation_get_check_sum(mxArray *plhs[]);
          sf_c4_BlueBird_Formation_get_check_sum(plhs);
          break;
        }

       case 5:
        {
          extern void sf_c5_BlueBird_Formation_get_check_sum(mxArray *plhs[]);
          sf_c5_BlueBird_Formation_get_check_sum(plhs);
          break;
        }

       case 6:
        {
          extern void sf_c6_BlueBird_Formation_get_check_sum(mxArray *plhs[]);
          sf_c6_BlueBird_Formation_get_check_sum(plhs);
          break;
        }

       case 7:
        {
          extern void sf_c7_BlueBird_Formation_get_check_sum(mxArray *plhs[]);
          sf_c7_BlueBird_Formation_get_check_sum(plhs);
          break;
        }

       case 8:
        {
          extern void sf_c8_BlueBird_Formation_get_check_sum(mxArray *plhs[]);
          sf_c8_BlueBird_Formation_get_check_sum(plhs);
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
    ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(3221723284U);
    ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(1581711657U);
    ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(1599569464U);
    ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(3103073205U);
  }

  return 1;

#else

  return 0;

#endif

}

unsigned int sf_BlueBird_Formation_autoinheritance_info( int nlhs, mxArray *
  plhs[], int nrhs, const mxArray * prhs[] )
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
        if (strcmp(aiChksum, "0mLomB2pn7ycyr9uuFC5PD") == 0) {
          extern mxArray *sf_c1_BlueBird_Formation_get_autoinheritance_info(void);
          plhs[0] = sf_c1_BlueBird_Formation_get_autoinheritance_info();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     case 2:
      {
        if (strcmp(aiChksum, "0mLomB2pn7ycyr9uuFC5PD") == 0) {
          extern mxArray *sf_c2_BlueBird_Formation_get_autoinheritance_info(void);
          plhs[0] = sf_c2_BlueBird_Formation_get_autoinheritance_info();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     case 3:
      {
        if (strcmp(aiChksum, "0mLomB2pn7ycyr9uuFC5PD") == 0) {
          extern mxArray *sf_c3_BlueBird_Formation_get_autoinheritance_info(void);
          plhs[0] = sf_c3_BlueBird_Formation_get_autoinheritance_info();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     case 4:
      {
        if (strcmp(aiChksum, "SJBmFm7VL9cH7EPoUcOZ0F") == 0) {
          extern mxArray *sf_c4_BlueBird_Formation_get_autoinheritance_info(void);
          plhs[0] = sf_c4_BlueBird_Formation_get_autoinheritance_info();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     case 5:
      {
        if (strcmp(aiChksum, "TWES86poMjLPrIjqudgrsH") == 0) {
          extern mxArray *sf_c5_BlueBird_Formation_get_autoinheritance_info(void);
          plhs[0] = sf_c5_BlueBird_Formation_get_autoinheritance_info();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     case 6:
      {
        if (strcmp(aiChksum, "TWES86poMjLPrIjqudgrsH") == 0) {
          extern mxArray *sf_c6_BlueBird_Formation_get_autoinheritance_info(void);
          plhs[0] = sf_c6_BlueBird_Formation_get_autoinheritance_info();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     case 7:
      {
        if (strcmp(aiChksum, "TWES86poMjLPrIjqudgrsH") == 0) {
          extern mxArray *sf_c7_BlueBird_Formation_get_autoinheritance_info(void);
          plhs[0] = sf_c7_BlueBird_Formation_get_autoinheritance_info();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     case 8:
      {
        if (strcmp(aiChksum, "0wNfJ3nefvhqgGkcGKNquH") == 0) {
          extern mxArray *sf_c8_BlueBird_Formation_get_autoinheritance_info(void);
          plhs[0] = sf_c8_BlueBird_Formation_get_autoinheritance_info();
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

unsigned int sf_BlueBird_Formation_get_eml_resolved_functions_info( int nlhs,
  mxArray * plhs[], int nrhs, const mxArray * prhs[] )
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
          *sf_c1_BlueBird_Formation_get_eml_resolved_functions_info(void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c1_BlueBird_Formation_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     case 2:
      {
        extern const mxArray
          *sf_c2_BlueBird_Formation_get_eml_resolved_functions_info(void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c2_BlueBird_Formation_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     case 3:
      {
        extern const mxArray
          *sf_c3_BlueBird_Formation_get_eml_resolved_functions_info(void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c3_BlueBird_Formation_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     case 4:
      {
        extern const mxArray
          *sf_c4_BlueBird_Formation_get_eml_resolved_functions_info(void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c4_BlueBird_Formation_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     case 5:
      {
        extern const mxArray
          *sf_c5_BlueBird_Formation_get_eml_resolved_functions_info(void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c5_BlueBird_Formation_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     case 6:
      {
        extern const mxArray
          *sf_c6_BlueBird_Formation_get_eml_resolved_functions_info(void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c6_BlueBird_Formation_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     case 7:
      {
        extern const mxArray
          *sf_c7_BlueBird_Formation_get_eml_resolved_functions_info(void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c7_BlueBird_Formation_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     case 8:
      {
        extern const mxArray
          *sf_c8_BlueBird_Formation_get_eml_resolved_functions_info(void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c8_BlueBird_Formation_get_eml_resolved_functions_info();
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

unsigned int sf_BlueBird_Formation_third_party_uses_info( int nlhs, mxArray *
  plhs[], int nrhs, const mxArray * prhs[] )
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
        if (strcmp(tpChksum, "1J6Y4d9eSZO9aKQRu3QnED") == 0) {
          extern mxArray *sf_c1_BlueBird_Formation_third_party_uses_info(void);
          plhs[0] = sf_c1_BlueBird_Formation_third_party_uses_info();
          break;
        }
      }

     case 2:
      {
        if (strcmp(tpChksum, "1J6Y4d9eSZO9aKQRu3QnED") == 0) {
          extern mxArray *sf_c2_BlueBird_Formation_third_party_uses_info(void);
          plhs[0] = sf_c2_BlueBird_Formation_third_party_uses_info();
          break;
        }
      }

     case 3:
      {
        if (strcmp(tpChksum, "1J6Y4d9eSZO9aKQRu3QnED") == 0) {
          extern mxArray *sf_c3_BlueBird_Formation_third_party_uses_info(void);
          plhs[0] = sf_c3_BlueBird_Formation_third_party_uses_info();
          break;
        }
      }

     case 4:
      {
        if (strcmp(tpChksum, "wwoFcswWK81hAMjM2ydil") == 0) {
          extern mxArray *sf_c4_BlueBird_Formation_third_party_uses_info(void);
          plhs[0] = sf_c4_BlueBird_Formation_third_party_uses_info();
          break;
        }
      }

     case 5:
      {
        if (strcmp(tpChksum, "lvV63jj6ueV5v5JRCqcDhD") == 0) {
          extern mxArray *sf_c5_BlueBird_Formation_third_party_uses_info(void);
          plhs[0] = sf_c5_BlueBird_Formation_third_party_uses_info();
          break;
        }
      }

     case 6:
      {
        if (strcmp(tpChksum, "lvV63jj6ueV5v5JRCqcDhD") == 0) {
          extern mxArray *sf_c6_BlueBird_Formation_third_party_uses_info(void);
          plhs[0] = sf_c6_BlueBird_Formation_third_party_uses_info();
          break;
        }
      }

     case 7:
      {
        if (strcmp(tpChksum, "lvV63jj6ueV5v5JRCqcDhD") == 0) {
          extern mxArray *sf_c7_BlueBird_Formation_third_party_uses_info(void);
          plhs[0] = sf_c7_BlueBird_Formation_third_party_uses_info();
          break;
        }
      }

     case 8:
      {
        if (strcmp(tpChksum, "NZRuE0HGsGypEZJg11HZl") == 0) {
          extern mxArray *sf_c8_BlueBird_Formation_third_party_uses_info(void);
          plhs[0] = sf_c8_BlueBird_Formation_third_party_uses_info();
          break;
        }
      }

     default:
      plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
    }
  }

  return 1;
}

unsigned int sf_BlueBird_Formation_updateBuildInfo_args_info( int nlhs, mxArray *
  plhs[], int nrhs, const mxArray * prhs[] )
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
        if (strcmp(tpChksum, "1J6Y4d9eSZO9aKQRu3QnED") == 0) {
          extern mxArray *sf_c1_BlueBird_Formation_updateBuildInfo_args_info
            (void);
          plhs[0] = sf_c1_BlueBird_Formation_updateBuildInfo_args_info();
          break;
        }
      }

     case 2:
      {
        if (strcmp(tpChksum, "1J6Y4d9eSZO9aKQRu3QnED") == 0) {
          extern mxArray *sf_c2_BlueBird_Formation_updateBuildInfo_args_info
            (void);
          plhs[0] = sf_c2_BlueBird_Formation_updateBuildInfo_args_info();
          break;
        }
      }

     case 3:
      {
        if (strcmp(tpChksum, "1J6Y4d9eSZO9aKQRu3QnED") == 0) {
          extern mxArray *sf_c3_BlueBird_Formation_updateBuildInfo_args_info
            (void);
          plhs[0] = sf_c3_BlueBird_Formation_updateBuildInfo_args_info();
          break;
        }
      }

     case 4:
      {
        if (strcmp(tpChksum, "wwoFcswWK81hAMjM2ydil") == 0) {
          extern mxArray *sf_c4_BlueBird_Formation_updateBuildInfo_args_info
            (void);
          plhs[0] = sf_c4_BlueBird_Formation_updateBuildInfo_args_info();
          break;
        }
      }

     case 5:
      {
        if (strcmp(tpChksum, "lvV63jj6ueV5v5JRCqcDhD") == 0) {
          extern mxArray *sf_c5_BlueBird_Formation_updateBuildInfo_args_info
            (void);
          plhs[0] = sf_c5_BlueBird_Formation_updateBuildInfo_args_info();
          break;
        }
      }

     case 6:
      {
        if (strcmp(tpChksum, "lvV63jj6ueV5v5JRCqcDhD") == 0) {
          extern mxArray *sf_c6_BlueBird_Formation_updateBuildInfo_args_info
            (void);
          plhs[0] = sf_c6_BlueBird_Formation_updateBuildInfo_args_info();
          break;
        }
      }

     case 7:
      {
        if (strcmp(tpChksum, "lvV63jj6ueV5v5JRCqcDhD") == 0) {
          extern mxArray *sf_c7_BlueBird_Formation_updateBuildInfo_args_info
            (void);
          plhs[0] = sf_c7_BlueBird_Formation_updateBuildInfo_args_info();
          break;
        }
      }

     case 8:
      {
        if (strcmp(tpChksum, "NZRuE0HGsGypEZJg11HZl") == 0) {
          extern mxArray *sf_c8_BlueBird_Formation_updateBuildInfo_args_info
            (void);
          plhs[0] = sf_c8_BlueBird_Formation_updateBuildInfo_args_info();
          break;
        }
      }

     default:
      plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
    }
  }

  return 1;
}

void BlueBird_Formation_debug_initialize(struct SfDebugInstanceStruct*
  debugInstance)
{
  _BlueBird_FormationMachineNumber_ = sf_debug_initialize_machine(debugInstance,
    "BlueBird_Formation","sfun",0,8,0,0,0);
  sf_debug_set_machine_event_thresholds(debugInstance,
    _BlueBird_FormationMachineNumber_,0,0);
  sf_debug_set_machine_data_thresholds(debugInstance,
    _BlueBird_FormationMachineNumber_,0);
}

void BlueBird_Formation_register_exported_symbols(SimStruct* S)
{
}

static mxArray* sRtwOptimizationInfoStruct= NULL;
mxArray* load_BlueBird_Formation_optimization_info(void)
{
  if (sRtwOptimizationInfoStruct==NULL) {
    sRtwOptimizationInfoStruct = sf_load_rtw_optimization_info(
      "BlueBird_Formation", "BlueBird_Formation");
    mexMakeArrayPersistent(sRtwOptimizationInfoStruct);
  }

  return(sRtwOptimizationInfoStruct);
}

void unload_BlueBird_Formation_optimization_info(void)
{
  if (sRtwOptimizationInfoStruct!=NULL) {
    mxDestroyArray(sRtwOptimizationInfoStruct);
    sRtwOptimizationInfoStruct = NULL;
  }
}
