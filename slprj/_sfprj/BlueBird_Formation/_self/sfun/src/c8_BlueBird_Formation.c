/* Include files */

#include <stddef.h>
#include "blas.h"
#include "BlueBird_Formation_sfun.h"
#include "c8_BlueBird_Formation.h"
#define CHARTINSTANCE_CHARTNUMBER      (chartInstance->chartNumber)
#define CHARTINSTANCE_INSTANCENUMBER   (chartInstance->instanceNumber)
#include "BlueBird_Formation_sfun_debug_macros.h"
#define _SF_MEX_LISTEN_FOR_CTRL_C(S)   sf_mex_listen_for_ctrl_c(sfGlobalDebugInstanceStruct,S);

/* Type Definitions */

/* Named Constants */
#define CALL_EVENT                     (-1)

/* Variable Declarations */

/* Variable Definitions */
static real_T _sfTime_;
static const char * c8_debug_family_names[5] = { "Rad", "nargin", "nargout",
  "Pose_In", "Pose_Out" };

/* Function Declarations */
static void initialize_c8_BlueBird_Formation
  (SFc8_BlueBird_FormationInstanceStruct *chartInstance);
static void initialize_params_c8_BlueBird_Formation
  (SFc8_BlueBird_FormationInstanceStruct *chartInstance);
static void enable_c8_BlueBird_Formation(SFc8_BlueBird_FormationInstanceStruct
  *chartInstance);
static void disable_c8_BlueBird_Formation(SFc8_BlueBird_FormationInstanceStruct *
  chartInstance);
static void c8_update_debugger_state_c8_BlueBird_Formation
  (SFc8_BlueBird_FormationInstanceStruct *chartInstance);
static const mxArray *get_sim_state_c8_BlueBird_Formation
  (SFc8_BlueBird_FormationInstanceStruct *chartInstance);
static void set_sim_state_c8_BlueBird_Formation
  (SFc8_BlueBird_FormationInstanceStruct *chartInstance, const mxArray *c8_st);
static void finalize_c8_BlueBird_Formation(SFc8_BlueBird_FormationInstanceStruct
  *chartInstance);
static void sf_gateway_c8_BlueBird_Formation
  (SFc8_BlueBird_FormationInstanceStruct *chartInstance);
static void initSimStructsc8_BlueBird_Formation
  (SFc8_BlueBird_FormationInstanceStruct *chartInstance);
static void init_script_number_translation(uint32_T c8_machineNumber, uint32_T
  c8_chartNumber, uint32_T c8_instanceNumber);
static const mxArray *c8_sf_marshallOut(void *chartInstanceVoid, void *c8_inData);
static void c8_emlrt_marshallIn(SFc8_BlueBird_FormationInstanceStruct
  *chartInstance, const mxArray *c8_Pose_Out, const char_T *c8_identifier,
  real_T c8_y[3]);
static void c8_b_emlrt_marshallIn(SFc8_BlueBird_FormationInstanceStruct
  *chartInstance, const mxArray *c8_u, const emlrtMsgIdentifier *c8_parentId,
  real_T c8_y[3]);
static void c8_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c8_mxArrayInData, const char_T *c8_varName, void *c8_outData);
static const mxArray *c8_b_sf_marshallOut(void *chartInstanceVoid, void
  *c8_inData);
static real_T c8_c_emlrt_marshallIn(SFc8_BlueBird_FormationInstanceStruct
  *chartInstance, const mxArray *c8_u, const emlrtMsgIdentifier *c8_parentId);
static void c8_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c8_mxArrayInData, const char_T *c8_varName, void *c8_outData);
static const mxArray *c8_c_sf_marshallOut(void *chartInstanceVoid, void
  *c8_inData);
static int32_T c8_d_emlrt_marshallIn(SFc8_BlueBird_FormationInstanceStruct
  *chartInstance, const mxArray *c8_u, const emlrtMsgIdentifier *c8_parentId);
static void c8_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c8_mxArrayInData, const char_T *c8_varName, void *c8_outData);
static uint8_T c8_e_emlrt_marshallIn(SFc8_BlueBird_FormationInstanceStruct
  *chartInstance, const mxArray *c8_b_is_active_c8_BlueBird_Formation, const
  char_T *c8_identifier);
static uint8_T c8_f_emlrt_marshallIn(SFc8_BlueBird_FormationInstanceStruct
  *chartInstance, const mxArray *c8_u, const emlrtMsgIdentifier *c8_parentId);
static void init_dsm_address_info(SFc8_BlueBird_FormationInstanceStruct
  *chartInstance);

/* Function Definitions */
static void initialize_c8_BlueBird_Formation
  (SFc8_BlueBird_FormationInstanceStruct *chartInstance)
{
  chartInstance->c8_sfEvent = CALL_EVENT;
  _sfTime_ = sf_get_time(chartInstance->S);
  chartInstance->c8_is_active_c8_BlueBird_Formation = 0U;
}

static void initialize_params_c8_BlueBird_Formation
  (SFc8_BlueBird_FormationInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void enable_c8_BlueBird_Formation(SFc8_BlueBird_FormationInstanceStruct
  *chartInstance)
{
  _sfTime_ = sf_get_time(chartInstance->S);
}

static void disable_c8_BlueBird_Formation(SFc8_BlueBird_FormationInstanceStruct *
  chartInstance)
{
  _sfTime_ = sf_get_time(chartInstance->S);
}

static void c8_update_debugger_state_c8_BlueBird_Formation
  (SFc8_BlueBird_FormationInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static const mxArray *get_sim_state_c8_BlueBird_Formation
  (SFc8_BlueBird_FormationInstanceStruct *chartInstance)
{
  const mxArray *c8_st;
  const mxArray *c8_y = NULL;
  int32_T c8_i0;
  real_T c8_u[3];
  const mxArray *c8_b_y = NULL;
  uint8_T c8_hoistedGlobal;
  uint8_T c8_b_u;
  const mxArray *c8_c_y = NULL;
  real_T (*c8_Pose_Out)[3];
  c8_Pose_Out = (real_T (*)[3])ssGetOutputPortSignal(chartInstance->S, 1);
  c8_st = NULL;
  c8_st = NULL;
  c8_y = NULL;
  sf_mex_assign(&c8_y, sf_mex_createcellmatrix(2, 1), false);
  for (c8_i0 = 0; c8_i0 < 3; c8_i0++) {
    c8_u[c8_i0] = (*c8_Pose_Out)[c8_i0];
  }

  c8_b_y = NULL;
  sf_mex_assign(&c8_b_y, sf_mex_create("y", c8_u, 0, 0U, 1U, 0U, 1, 3), false);
  sf_mex_setcell(c8_y, 0, c8_b_y);
  c8_hoistedGlobal = chartInstance->c8_is_active_c8_BlueBird_Formation;
  c8_b_u = c8_hoistedGlobal;
  c8_c_y = NULL;
  sf_mex_assign(&c8_c_y, sf_mex_create("y", &c8_b_u, 3, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c8_y, 1, c8_c_y);
  sf_mex_assign(&c8_st, c8_y, false);
  return c8_st;
}

static void set_sim_state_c8_BlueBird_Formation
  (SFc8_BlueBird_FormationInstanceStruct *chartInstance, const mxArray *c8_st)
{
  const mxArray *c8_u;
  real_T c8_dv0[3];
  int32_T c8_i1;
  real_T (*c8_Pose_Out)[3];
  c8_Pose_Out = (real_T (*)[3])ssGetOutputPortSignal(chartInstance->S, 1);
  chartInstance->c8_doneDoubleBufferReInit = true;
  c8_u = sf_mex_dup(c8_st);
  c8_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c8_u, 0)),
                      "Pose_Out", c8_dv0);
  for (c8_i1 = 0; c8_i1 < 3; c8_i1++) {
    (*c8_Pose_Out)[c8_i1] = c8_dv0[c8_i1];
  }

  chartInstance->c8_is_active_c8_BlueBird_Formation = c8_e_emlrt_marshallIn
    (chartInstance, sf_mex_dup(sf_mex_getcell(c8_u, 1)),
     "is_active_c8_BlueBird_Formation");
  sf_mex_destroy(&c8_u);
  c8_update_debugger_state_c8_BlueBird_Formation(chartInstance);
  sf_mex_destroy(&c8_st);
}

static void finalize_c8_BlueBird_Formation(SFc8_BlueBird_FormationInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void sf_gateway_c8_BlueBird_Formation
  (SFc8_BlueBird_FormationInstanceStruct *chartInstance)
{
  int32_T c8_i2;
  real_T c8_Pose_In[3];
  uint32_T c8_debug_family_var_map[5];
  real_T c8_Rad;
  real_T c8_nargin = 1.0;
  real_T c8_nargout = 1.0;
  real_T c8_Pose_Out[3];
  int32_T c8_i3;
  int32_T c8_i4;
  int32_T c8_i5;
  int32_T c8_i6;
  real_T (*c8_b_Pose_Out)[3];
  real_T (*c8_b_Pose_In)[3];
  c8_b_Pose_In = (real_T (*)[3])ssGetInputPortSignal(chartInstance->S, 0);
  c8_b_Pose_Out = (real_T (*)[3])ssGetOutputPortSignal(chartInstance->S, 1);
  _SFD_SYMBOL_SCOPE_PUSH(0U, 0U);
  _sfTime_ = sf_get_time(chartInstance->S);
  _SFD_CC_CALL(CHART_ENTER_SFUNCTION_TAG, 7U, chartInstance->c8_sfEvent);
  chartInstance->c8_sfEvent = CALL_EVENT;
  _SFD_CC_CALL(CHART_ENTER_DURING_FUNCTION_TAG, 7U, chartInstance->c8_sfEvent);
  for (c8_i2 = 0; c8_i2 < 3; c8_i2++) {
    c8_Pose_In[c8_i2] = (*c8_b_Pose_In)[c8_i2];
  }

  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 5U, 5U, c8_debug_family_names,
    c8_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c8_Rad, 0U, c8_b_sf_marshallOut,
    c8_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c8_nargin, 1U, c8_b_sf_marshallOut,
    c8_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c8_nargout, 2U, c8_b_sf_marshallOut,
    c8_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(c8_Pose_In, 3U, c8_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c8_Pose_Out, 4U, c8_sf_marshallOut,
    c8_sf_marshallIn);
  CV_EML_FCN(0, 0);
  _SFD_EML_CALL(0U, chartInstance->c8_sfEvent, 3);
  c8_Rad = c8_Pose_In[2];
  _SFD_EML_CALL(0U, chartInstance->c8_sfEvent, 4);
  while (CV_EML_WHILE(0, 1, 0, c8_Rad > 3.1415926535897931)) {
    _SFD_EML_CALL(0U, chartInstance->c8_sfEvent, 5);
    c8_Rad -= 6.2831853071795862;
    _SFD_EML_CALL(0U, chartInstance->c8_sfEvent, 4);
    _SF_MEX_LISTEN_FOR_CTRL_C(chartInstance->S);
  }

  _SFD_EML_CALL(0U, chartInstance->c8_sfEvent, 7);
  while (CV_EML_WHILE(0, 1, 1, c8_Rad < -3.1415926535897931)) {
    _SFD_EML_CALL(0U, chartInstance->c8_sfEvent, 8);
    c8_Rad += 6.2831853071795862;
    _SFD_EML_CALL(0U, chartInstance->c8_sfEvent, 7);
    _SF_MEX_LISTEN_FOR_CTRL_C(chartInstance->S);
  }

  _SFD_EML_CALL(0U, chartInstance->c8_sfEvent, 11);
  for (c8_i3 = 0; c8_i3 < 3; c8_i3++) {
    c8_Pose_Out[c8_i3] = c8_Pose_In[c8_i3];
  }

  _SFD_EML_CALL(0U, chartInstance->c8_sfEvent, 12);
  c8_Pose_Out[2] = c8_Rad;
  _SFD_EML_CALL(0U, chartInstance->c8_sfEvent, -12);
  _SFD_SYMBOL_SCOPE_POP();
  for (c8_i4 = 0; c8_i4 < 3; c8_i4++) {
    (*c8_b_Pose_Out)[c8_i4] = c8_Pose_Out[c8_i4];
  }

  _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG, 7U, chartInstance->c8_sfEvent);
  _SFD_SYMBOL_SCOPE_POP();
  _SFD_CHECK_FOR_STATE_INCONSISTENCY(_BlueBird_FormationMachineNumber_,
    chartInstance->chartNumber, chartInstance->instanceNumber);
  for (c8_i5 = 0; c8_i5 < 3; c8_i5++) {
    _SFD_DATA_RANGE_CHECK((*c8_b_Pose_Out)[c8_i5], 0U);
  }

  for (c8_i6 = 0; c8_i6 < 3; c8_i6++) {
    _SFD_DATA_RANGE_CHECK((*c8_b_Pose_In)[c8_i6], 1U);
  }
}

static void initSimStructsc8_BlueBird_Formation
  (SFc8_BlueBird_FormationInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void init_script_number_translation(uint32_T c8_machineNumber, uint32_T
  c8_chartNumber, uint32_T c8_instanceNumber)
{
  (void)c8_machineNumber;
  (void)c8_chartNumber;
  (void)c8_instanceNumber;
}

static const mxArray *c8_sf_marshallOut(void *chartInstanceVoid, void *c8_inData)
{
  const mxArray *c8_mxArrayOutData = NULL;
  int32_T c8_i7;
  real_T c8_b_inData[3];
  int32_T c8_i8;
  real_T c8_u[3];
  const mxArray *c8_y = NULL;
  SFc8_BlueBird_FormationInstanceStruct *chartInstance;
  chartInstance = (SFc8_BlueBird_FormationInstanceStruct *)chartInstanceVoid;
  c8_mxArrayOutData = NULL;
  for (c8_i7 = 0; c8_i7 < 3; c8_i7++) {
    c8_b_inData[c8_i7] = (*(real_T (*)[3])c8_inData)[c8_i7];
  }

  for (c8_i8 = 0; c8_i8 < 3; c8_i8++) {
    c8_u[c8_i8] = c8_b_inData[c8_i8];
  }

  c8_y = NULL;
  sf_mex_assign(&c8_y, sf_mex_create("y", c8_u, 0, 0U, 1U, 0U, 1, 3), false);
  sf_mex_assign(&c8_mxArrayOutData, c8_y, false);
  return c8_mxArrayOutData;
}

static void c8_emlrt_marshallIn(SFc8_BlueBird_FormationInstanceStruct
  *chartInstance, const mxArray *c8_Pose_Out, const char_T *c8_identifier,
  real_T c8_y[3])
{
  emlrtMsgIdentifier c8_thisId;
  c8_thisId.fIdentifier = c8_identifier;
  c8_thisId.fParent = NULL;
  c8_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c8_Pose_Out), &c8_thisId, c8_y);
  sf_mex_destroy(&c8_Pose_Out);
}

static void c8_b_emlrt_marshallIn(SFc8_BlueBird_FormationInstanceStruct
  *chartInstance, const mxArray *c8_u, const emlrtMsgIdentifier *c8_parentId,
  real_T c8_y[3])
{
  real_T c8_dv1[3];
  int32_T c8_i9;
  (void)chartInstance;
  sf_mex_import(c8_parentId, sf_mex_dup(c8_u), c8_dv1, 1, 0, 0U, 1, 0U, 1, 3);
  for (c8_i9 = 0; c8_i9 < 3; c8_i9++) {
    c8_y[c8_i9] = c8_dv1[c8_i9];
  }

  sf_mex_destroy(&c8_u);
}

static void c8_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c8_mxArrayInData, const char_T *c8_varName, void *c8_outData)
{
  const mxArray *c8_Pose_Out;
  const char_T *c8_identifier;
  emlrtMsgIdentifier c8_thisId;
  real_T c8_y[3];
  int32_T c8_i10;
  SFc8_BlueBird_FormationInstanceStruct *chartInstance;
  chartInstance = (SFc8_BlueBird_FormationInstanceStruct *)chartInstanceVoid;
  c8_Pose_Out = sf_mex_dup(c8_mxArrayInData);
  c8_identifier = c8_varName;
  c8_thisId.fIdentifier = c8_identifier;
  c8_thisId.fParent = NULL;
  c8_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c8_Pose_Out), &c8_thisId, c8_y);
  sf_mex_destroy(&c8_Pose_Out);
  for (c8_i10 = 0; c8_i10 < 3; c8_i10++) {
    (*(real_T (*)[3])c8_outData)[c8_i10] = c8_y[c8_i10];
  }

  sf_mex_destroy(&c8_mxArrayInData);
}

static const mxArray *c8_b_sf_marshallOut(void *chartInstanceVoid, void
  *c8_inData)
{
  const mxArray *c8_mxArrayOutData = NULL;
  real_T c8_u;
  const mxArray *c8_y = NULL;
  SFc8_BlueBird_FormationInstanceStruct *chartInstance;
  chartInstance = (SFc8_BlueBird_FormationInstanceStruct *)chartInstanceVoid;
  c8_mxArrayOutData = NULL;
  c8_u = *(real_T *)c8_inData;
  c8_y = NULL;
  sf_mex_assign(&c8_y, sf_mex_create("y", &c8_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_assign(&c8_mxArrayOutData, c8_y, false);
  return c8_mxArrayOutData;
}

static real_T c8_c_emlrt_marshallIn(SFc8_BlueBird_FormationInstanceStruct
  *chartInstance, const mxArray *c8_u, const emlrtMsgIdentifier *c8_parentId)
{
  real_T c8_y;
  real_T c8_d0;
  (void)chartInstance;
  sf_mex_import(c8_parentId, sf_mex_dup(c8_u), &c8_d0, 1, 0, 0U, 0, 0U, 0);
  c8_y = c8_d0;
  sf_mex_destroy(&c8_u);
  return c8_y;
}

static void c8_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c8_mxArrayInData, const char_T *c8_varName, void *c8_outData)
{
  const mxArray *c8_nargout;
  const char_T *c8_identifier;
  emlrtMsgIdentifier c8_thisId;
  real_T c8_y;
  SFc8_BlueBird_FormationInstanceStruct *chartInstance;
  chartInstance = (SFc8_BlueBird_FormationInstanceStruct *)chartInstanceVoid;
  c8_nargout = sf_mex_dup(c8_mxArrayInData);
  c8_identifier = c8_varName;
  c8_thisId.fIdentifier = c8_identifier;
  c8_thisId.fParent = NULL;
  c8_y = c8_c_emlrt_marshallIn(chartInstance, sf_mex_dup(c8_nargout), &c8_thisId);
  sf_mex_destroy(&c8_nargout);
  *(real_T *)c8_outData = c8_y;
  sf_mex_destroy(&c8_mxArrayInData);
}

const mxArray *sf_c8_BlueBird_Formation_get_eml_resolved_functions_info(void)
{
  const mxArray *c8_nameCaptureInfo = NULL;
  c8_nameCaptureInfo = NULL;
  sf_mex_assign(&c8_nameCaptureInfo, sf_mex_create("nameCaptureInfo", NULL, 0,
    0U, 1U, 0U, 2, 0, 1), false);
  return c8_nameCaptureInfo;
}

static const mxArray *c8_c_sf_marshallOut(void *chartInstanceVoid, void
  *c8_inData)
{
  const mxArray *c8_mxArrayOutData = NULL;
  int32_T c8_u;
  const mxArray *c8_y = NULL;
  SFc8_BlueBird_FormationInstanceStruct *chartInstance;
  chartInstance = (SFc8_BlueBird_FormationInstanceStruct *)chartInstanceVoid;
  c8_mxArrayOutData = NULL;
  c8_u = *(int32_T *)c8_inData;
  c8_y = NULL;
  sf_mex_assign(&c8_y, sf_mex_create("y", &c8_u, 6, 0U, 0U, 0U, 0), false);
  sf_mex_assign(&c8_mxArrayOutData, c8_y, false);
  return c8_mxArrayOutData;
}

static int32_T c8_d_emlrt_marshallIn(SFc8_BlueBird_FormationInstanceStruct
  *chartInstance, const mxArray *c8_u, const emlrtMsgIdentifier *c8_parentId)
{
  int32_T c8_y;
  int32_T c8_i11;
  (void)chartInstance;
  sf_mex_import(c8_parentId, sf_mex_dup(c8_u), &c8_i11, 1, 6, 0U, 0, 0U, 0);
  c8_y = c8_i11;
  sf_mex_destroy(&c8_u);
  return c8_y;
}

static void c8_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c8_mxArrayInData, const char_T *c8_varName, void *c8_outData)
{
  const mxArray *c8_b_sfEvent;
  const char_T *c8_identifier;
  emlrtMsgIdentifier c8_thisId;
  int32_T c8_y;
  SFc8_BlueBird_FormationInstanceStruct *chartInstance;
  chartInstance = (SFc8_BlueBird_FormationInstanceStruct *)chartInstanceVoid;
  c8_b_sfEvent = sf_mex_dup(c8_mxArrayInData);
  c8_identifier = c8_varName;
  c8_thisId.fIdentifier = c8_identifier;
  c8_thisId.fParent = NULL;
  c8_y = c8_d_emlrt_marshallIn(chartInstance, sf_mex_dup(c8_b_sfEvent),
    &c8_thisId);
  sf_mex_destroy(&c8_b_sfEvent);
  *(int32_T *)c8_outData = c8_y;
  sf_mex_destroy(&c8_mxArrayInData);
}

static uint8_T c8_e_emlrt_marshallIn(SFc8_BlueBird_FormationInstanceStruct
  *chartInstance, const mxArray *c8_b_is_active_c8_BlueBird_Formation, const
  char_T *c8_identifier)
{
  uint8_T c8_y;
  emlrtMsgIdentifier c8_thisId;
  c8_thisId.fIdentifier = c8_identifier;
  c8_thisId.fParent = NULL;
  c8_y = c8_f_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c8_b_is_active_c8_BlueBird_Formation), &c8_thisId);
  sf_mex_destroy(&c8_b_is_active_c8_BlueBird_Formation);
  return c8_y;
}

static uint8_T c8_f_emlrt_marshallIn(SFc8_BlueBird_FormationInstanceStruct
  *chartInstance, const mxArray *c8_u, const emlrtMsgIdentifier *c8_parentId)
{
  uint8_T c8_y;
  uint8_T c8_u0;
  (void)chartInstance;
  sf_mex_import(c8_parentId, sf_mex_dup(c8_u), &c8_u0, 1, 3, 0U, 0, 0U, 0);
  c8_y = c8_u0;
  sf_mex_destroy(&c8_u);
  return c8_y;
}

static void init_dsm_address_info(SFc8_BlueBird_FormationInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

/* SFunction Glue Code */
#ifdef utFree
#undef utFree
#endif

#ifdef utMalloc
#undef utMalloc
#endif

#ifdef __cplusplus

extern "C" void *utMalloc(size_t size);
extern "C" void utFree(void*);

#else

extern void *utMalloc(size_t size);
extern void utFree(void*);

#endif

void sf_c8_BlueBird_Formation_get_check_sum(mxArray *plhs[])
{
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(1973410847U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(3787110690U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(1985584304U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(4142467063U);
}

mxArray *sf_c8_BlueBird_Formation_get_autoinheritance_info(void)
{
  const char *autoinheritanceFields[] = { "checksum", "inputs", "parameters",
    "outputs", "locals" };

  mxArray *mxAutoinheritanceInfo = mxCreateStructMatrix(1,1,5,
    autoinheritanceFields);

  {
    mxArray *mxChecksum = mxCreateString("0wNfJ3nefvhqgGkcGKNquH");
    mxSetField(mxAutoinheritanceInfo,0,"checksum",mxChecksum);
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,1,3,dataFields);

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(3);
      pr[1] = (double)(1);
      mxSetField(mxData,0,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,0,"type",mxType);
    }

    mxSetField(mxData,0,"complexity",mxCreateDoubleScalar(0));
    mxSetField(mxAutoinheritanceInfo,0,"inputs",mxData);
  }

  {
    mxSetField(mxAutoinheritanceInfo,0,"parameters",mxCreateDoubleMatrix(0,0,
                mxREAL));
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,1,3,dataFields);

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(3);
      pr[1] = (double)(1);
      mxSetField(mxData,0,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,0,"type",mxType);
    }

    mxSetField(mxData,0,"complexity",mxCreateDoubleScalar(0));
    mxSetField(mxAutoinheritanceInfo,0,"outputs",mxData);
  }

  {
    mxSetField(mxAutoinheritanceInfo,0,"locals",mxCreateDoubleMatrix(0,0,mxREAL));
  }

  return(mxAutoinheritanceInfo);
}

mxArray *sf_c8_BlueBird_Formation_third_party_uses_info(void)
{
  mxArray * mxcell3p = mxCreateCellMatrix(1,0);
  return(mxcell3p);
}

mxArray *sf_c8_BlueBird_Formation_updateBuildInfo_args_info(void)
{
  mxArray *mxBIArgs = mxCreateCellMatrix(1,0);
  return mxBIArgs;
}

static const mxArray *sf_get_sim_state_info_c8_BlueBird_Formation(void)
{
  const char *infoFields[] = { "chartChecksum", "varInfo" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 2, infoFields);
  const char *infoEncStr[] = {
    "100 S1x2'type','srcId','name','auxInfo'{{M[1],M[5],T\"Pose_Out\",},{M[8],M[0],T\"is_active_c8_BlueBird_Formation\",}}"
  };

  mxArray *mxVarInfo = sf_mex_decode_encoded_mx_struct_array(infoEncStr, 2, 10);
  mxArray *mxChecksum = mxCreateDoubleMatrix(1, 4, mxREAL);
  sf_c8_BlueBird_Formation_get_check_sum(&mxChecksum);
  mxSetField(mxInfo, 0, infoFields[0], mxChecksum);
  mxSetField(mxInfo, 0, infoFields[1], mxVarInfo);
  return mxInfo;
}

static void chart_debug_initialization(SimStruct *S, unsigned int
  fullDebuggerInitialization)
{
  if (!sim_mode_is_rtw_gen(S)) {
    SFc8_BlueBird_FormationInstanceStruct *chartInstance;
    ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
    ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
    chartInstance = (SFc8_BlueBird_FormationInstanceStruct *)
      chartInfo->chartInstance;
    if (ssIsFirstInitCond(S) && fullDebuggerInitialization==1) {
      /* do this only if simulation is starting */
      {
        unsigned int chartAlreadyPresent;
        chartAlreadyPresent = sf_debug_initialize_chart
          (sfGlobalDebugInstanceStruct,
           _BlueBird_FormationMachineNumber_,
           8,
           1,
           1,
           0,
           2,
           0,
           0,
           0,
           0,
           0,
           &(chartInstance->chartNumber),
           &(chartInstance->instanceNumber),
           (void *)S);

        /* Each instance must initialize ist own list of scripts */
        init_script_number_translation(_BlueBird_FormationMachineNumber_,
          chartInstance->chartNumber,chartInstance->instanceNumber);
        if (chartAlreadyPresent==0) {
          /* this is the first instance */
          sf_debug_set_chart_disable_implicit_casting
            (sfGlobalDebugInstanceStruct,_BlueBird_FormationMachineNumber_,
             chartInstance->chartNumber,1);
          sf_debug_set_chart_event_thresholds(sfGlobalDebugInstanceStruct,
            _BlueBird_FormationMachineNumber_,
            chartInstance->chartNumber,
            0,
            0,
            0);
          _SFD_SET_DATA_PROPS(0,2,0,1,"Pose_Out");
          _SFD_SET_DATA_PROPS(1,1,1,0,"Pose_In");
          _SFD_STATE_INFO(0,0,2);
          _SFD_CH_SUBSTATE_COUNT(0);
          _SFD_CH_SUBSTATE_DECOMP(0);
        }

        _SFD_CV_INIT_CHART(0,0,0,0);

        {
          _SFD_CV_INIT_STATE(0,0,0,0,0,0,NULL,NULL);
        }

        _SFD_CV_INIT_TRANS(0,0,NULL,NULL,0,NULL);

        /* Initialization of MATLAB Function Model Coverage */
        _SFD_CV_INIT_EML(0,1,1,0,0,0,0,0,2,0,0);
        _SFD_CV_INIT_EML_FCN(0,0,"eML_blk_kernel",0,-1,231);
        _SFD_CV_INIT_EML_WHILE(0,1,0,97,113,142);
        _SFD_CV_INIT_EML_WHILE(0,1,1,143,160,189);

        {
          unsigned int dimVector[1];
          dimVector[0]= 3;
          _SFD_SET_DATA_COMPILED_PROPS(0,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c8_sf_marshallOut,(MexInFcnForType)
            c8_sf_marshallIn);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 3;
          _SFD_SET_DATA_COMPILED_PROPS(1,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c8_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          real_T (*c8_Pose_Out)[3];
          real_T (*c8_Pose_In)[3];
          c8_Pose_In = (real_T (*)[3])ssGetInputPortSignal(chartInstance->S, 0);
          c8_Pose_Out = (real_T (*)[3])ssGetOutputPortSignal(chartInstance->S, 1);
          _SFD_SET_DATA_VALUE_PTR(0U, *c8_Pose_Out);
          _SFD_SET_DATA_VALUE_PTR(1U, *c8_Pose_In);
        }
      }
    } else {
      sf_debug_reset_current_state_configuration(sfGlobalDebugInstanceStruct,
        _BlueBird_FormationMachineNumber_,chartInstance->chartNumber,
        chartInstance->instanceNumber);
    }
  }
}

static const char* sf_get_instance_specialization(void)
{
  return "NZRuE0HGsGypEZJg11HZl";
}

static void sf_opaque_initialize_c8_BlueBird_Formation(void *chartInstanceVar)
{
  chart_debug_initialization(((SFc8_BlueBird_FormationInstanceStruct*)
    chartInstanceVar)->S,0);
  initialize_params_c8_BlueBird_Formation((SFc8_BlueBird_FormationInstanceStruct*)
    chartInstanceVar);
  initialize_c8_BlueBird_Formation((SFc8_BlueBird_FormationInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_enable_c8_BlueBird_Formation(void *chartInstanceVar)
{
  enable_c8_BlueBird_Formation((SFc8_BlueBird_FormationInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_disable_c8_BlueBird_Formation(void *chartInstanceVar)
{
  disable_c8_BlueBird_Formation((SFc8_BlueBird_FormationInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_gateway_c8_BlueBird_Formation(void *chartInstanceVar)
{
  sf_gateway_c8_BlueBird_Formation((SFc8_BlueBird_FormationInstanceStruct*)
    chartInstanceVar);
}

extern const mxArray* sf_internal_get_sim_state_c8_BlueBird_Formation(SimStruct*
  S)
{
  ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
  ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_raw2high");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = (mxArray*) get_sim_state_c8_BlueBird_Formation
    ((SFc8_BlueBird_FormationInstanceStruct*)chartInfo->chartInstance);/* raw sim ctx */
  prhs[3] = (mxArray*) sf_get_sim_state_info_c8_BlueBird_Formation();/* state var info */
  mxError = sf_mex_call_matlab(1, plhs, 4, prhs, "sfprivate");
  mxDestroyArray(prhs[0]);
  mxDestroyArray(prhs[1]);
  mxDestroyArray(prhs[2]);
  mxDestroyArray(prhs[3]);
  if (mxError || plhs[0] == NULL) {
    sf_mex_error_message("Stateflow Internal Error: \nError calling 'chart_simctx_raw2high'.\n");
  }

  return plhs[0];
}

extern void sf_internal_set_sim_state_c8_BlueBird_Formation(SimStruct* S, const
  mxArray *st)
{
  ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
  ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[3];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_high2raw");
  prhs[1] = mxDuplicateArray(st);      /* high level simctx */
  prhs[2] = (mxArray*) sf_get_sim_state_info_c8_BlueBird_Formation();/* state var info */
  mxError = sf_mex_call_matlab(1, plhs, 3, prhs, "sfprivate");
  mxDestroyArray(prhs[0]);
  mxDestroyArray(prhs[1]);
  mxDestroyArray(prhs[2]);
  if (mxError || plhs[0] == NULL) {
    sf_mex_error_message("Stateflow Internal Error: \nError calling 'chart_simctx_high2raw'.\n");
  }

  set_sim_state_c8_BlueBird_Formation((SFc8_BlueBird_FormationInstanceStruct*)
    chartInfo->chartInstance, mxDuplicateArray(plhs[0]));
  mxDestroyArray(plhs[0]);
}

static const mxArray* sf_opaque_get_sim_state_c8_BlueBird_Formation(SimStruct* S)
{
  return sf_internal_get_sim_state_c8_BlueBird_Formation(S);
}

static void sf_opaque_set_sim_state_c8_BlueBird_Formation(SimStruct* S, const
  mxArray *st)
{
  sf_internal_set_sim_state_c8_BlueBird_Formation(S, st);
}

static void sf_opaque_terminate_c8_BlueBird_Formation(void *chartInstanceVar)
{
  if (chartInstanceVar!=NULL) {
    SimStruct *S = ((SFc8_BlueBird_FormationInstanceStruct*) chartInstanceVar)
      ->S;
    ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
    if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
      sf_clear_rtw_identifier(S);
      unload_BlueBird_Formation_optimization_info();
    }

    finalize_c8_BlueBird_Formation((SFc8_BlueBird_FormationInstanceStruct*)
      chartInstanceVar);
    utFree((void *)chartInstanceVar);
    if (crtInfo != NULL) {
      utFree((void *)crtInfo);
    }

    ssSetUserData(S,NULL);
  }
}

static void sf_opaque_init_subchart_simstructs(void *chartInstanceVar)
{
  initSimStructsc8_BlueBird_Formation((SFc8_BlueBird_FormationInstanceStruct*)
    chartInstanceVar);
}

extern unsigned int sf_machine_global_initializer_called(void);
static void mdlProcessParameters_c8_BlueBird_Formation(SimStruct *S)
{
  int i;
  for (i=0;i<ssGetNumRunTimeParams(S);i++) {
    if (ssGetSFcnParamTunable(S,i)) {
      ssUpdateDlgParamAsRunTimeParam(S,i);
    }
  }

  if (sf_machine_global_initializer_called()) {
    ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
    ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
    initialize_params_c8_BlueBird_Formation
      ((SFc8_BlueBird_FormationInstanceStruct*)(chartInfo->chartInstance));
  }
}

static void mdlSetWorkWidths_c8_BlueBird_Formation(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
    mxArray *infoStruct = load_BlueBird_Formation_optimization_info();
    int_T chartIsInlinable =
      (int_T)sf_is_chart_inlinable(sf_get_instance_specialization(),infoStruct,8);
    ssSetStateflowIsInlinable(S,chartIsInlinable);
    ssSetRTWCG(S,sf_rtw_info_uint_prop(sf_get_instance_specialization(),
                infoStruct,8,"RTWCG"));
    ssSetEnableFcnIsTrivial(S,1);
    ssSetDisableFcnIsTrivial(S,1);
    ssSetNotMultipleInlinable(S,sf_rtw_info_uint_prop
      (sf_get_instance_specialization(),infoStruct,8,
       "gatewayCannotBeInlinedMultipleTimes"));
    sf_update_buildInfo(sf_get_instance_specialization(),infoStruct,8);
    if (chartIsInlinable) {
      ssSetInputPortOptimOpts(S, 0, SS_REUSABLE_AND_LOCAL);
      sf_mark_chart_expressionable_inputs(S,sf_get_instance_specialization(),
        infoStruct,8,1);
      sf_mark_chart_reusable_outputs(S,sf_get_instance_specialization(),
        infoStruct,8,1);
    }

    {
      unsigned int outPortIdx;
      for (outPortIdx=1; outPortIdx<=1; ++outPortIdx) {
        ssSetOutputPortOptimizeInIR(S, outPortIdx, 1U);
      }
    }

    {
      unsigned int inPortIdx;
      for (inPortIdx=0; inPortIdx < 1; ++inPortIdx) {
        ssSetInputPortOptimizeInIR(S, inPortIdx, 1U);
      }
    }

    sf_set_rtw_dwork_info(S,sf_get_instance_specialization(),infoStruct,8);
    ssSetHasSubFunctions(S,!(chartIsInlinable));
  } else {
  }

  ssSetOptions(S,ssGetOptions(S)|SS_OPTION_WORKS_WITH_CODE_REUSE);
  ssSetChecksum0(S,(1850212332U));
  ssSetChecksum1(S,(3596301504U));
  ssSetChecksum2(S,(2845704458U));
  ssSetChecksum3(S,(3119661742U));
  ssSetmdlDerivatives(S, NULL);
  ssSetExplicitFCSSCtrl(S,1);
  ssSupportsMultipleExecInstances(S,1);
}

static void mdlRTW_c8_BlueBird_Formation(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S)) {
    ssWriteRTWStrParam(S, "StateflowChartType", "Embedded MATLAB");
  }
}

static void mdlStart_c8_BlueBird_Formation(SimStruct *S)
{
  SFc8_BlueBird_FormationInstanceStruct *chartInstance;
  ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)utMalloc(sizeof
    (ChartRunTimeInfo));
  chartInstance = (SFc8_BlueBird_FormationInstanceStruct *)utMalloc(sizeof
    (SFc8_BlueBird_FormationInstanceStruct));
  memset(chartInstance, 0, sizeof(SFc8_BlueBird_FormationInstanceStruct));
  if (chartInstance==NULL) {
    sf_mex_error_message("Could not allocate memory for chart instance.");
  }

  chartInstance->chartInfo.chartInstance = chartInstance;
  chartInstance->chartInfo.isEMLChart = 1;
  chartInstance->chartInfo.chartInitialized = 0;
  chartInstance->chartInfo.sFunctionGateway =
    sf_opaque_gateway_c8_BlueBird_Formation;
  chartInstance->chartInfo.initializeChart =
    sf_opaque_initialize_c8_BlueBird_Formation;
  chartInstance->chartInfo.terminateChart =
    sf_opaque_terminate_c8_BlueBird_Formation;
  chartInstance->chartInfo.enableChart = sf_opaque_enable_c8_BlueBird_Formation;
  chartInstance->chartInfo.disableChart =
    sf_opaque_disable_c8_BlueBird_Formation;
  chartInstance->chartInfo.getSimState =
    sf_opaque_get_sim_state_c8_BlueBird_Formation;
  chartInstance->chartInfo.setSimState =
    sf_opaque_set_sim_state_c8_BlueBird_Formation;
  chartInstance->chartInfo.getSimStateInfo =
    sf_get_sim_state_info_c8_BlueBird_Formation;
  chartInstance->chartInfo.zeroCrossings = NULL;
  chartInstance->chartInfo.outputs = NULL;
  chartInstance->chartInfo.derivatives = NULL;
  chartInstance->chartInfo.mdlRTW = mdlRTW_c8_BlueBird_Formation;
  chartInstance->chartInfo.mdlStart = mdlStart_c8_BlueBird_Formation;
  chartInstance->chartInfo.mdlSetWorkWidths =
    mdlSetWorkWidths_c8_BlueBird_Formation;
  chartInstance->chartInfo.extModeExec = NULL;
  chartInstance->chartInfo.restoreLastMajorStepConfiguration = NULL;
  chartInstance->chartInfo.restoreBeforeLastMajorStepConfiguration = NULL;
  chartInstance->chartInfo.storeCurrentConfiguration = NULL;
  chartInstance->chartInfo.debugInstance = sfGlobalDebugInstanceStruct;
  chartInstance->S = S;
  crtInfo->instanceInfo = (&(chartInstance->chartInfo));
  crtInfo->isJITEnabled = false;
  ssSetUserData(S,(void *)(crtInfo));  /* register the chart instance with simstruct */
  init_dsm_address_info(chartInstance);
  if (!sim_mode_is_rtw_gen(S)) {
  }

  sf_opaque_init_subchart_simstructs(chartInstance->chartInfo.chartInstance);
  chart_debug_initialization(S,1);
}

void c8_BlueBird_Formation_method_dispatcher(SimStruct *S, int_T method, void
  *data)
{
  switch (method) {
   case SS_CALL_MDL_START:
    mdlStart_c8_BlueBird_Formation(S);
    break;

   case SS_CALL_MDL_SET_WORK_WIDTHS:
    mdlSetWorkWidths_c8_BlueBird_Formation(S);
    break;

   case SS_CALL_MDL_PROCESS_PARAMETERS:
    mdlProcessParameters_c8_BlueBird_Formation(S);
    break;

   default:
    /* Unhandled method */
    sf_mex_error_message("Stateflow Internal Error:\n"
                         "Error calling c8_BlueBird_Formation_method_dispatcher.\n"
                         "Can't handle method %d.\n", method);
    break;
  }
}
