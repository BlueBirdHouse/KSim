/* Include files */

#include <stddef.h>
#include "blas.h"
#include "BlueBird_Formation_sfun.h"
#include "c5_BlueBird_Formation.h"
#include "mwmathutil.h"
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
static const char * c5_debug_family_names[20] = { "ir_distances",
  "ObstacleDetected", "IF_Obstacle", "FixIR_9_Number", "Obstacle_AroundRobot",
  "i", "IR_Position_I", "IR_Transformation_Matrix_I", "Obstacle_AroundRobot_I",
  "Obstacle_AroundRobot_World", "ObstaclesSpeed", "nargin", "nargout", "V_x_In",
  "V_y_In", "ir_raw", "Position", "V_x", "V_y", "Counter" };

static const char * c5_b_debug_family_names[9] = { "SafeNumber", "MaxNumber",
  "ir_raw_Internal", "Number_List", "ObstacleDetected", "nargin", "nargout",
  "ir_raw", "Answer" };

static const char * c5_c_debug_family_names[5] = { "nargin", "nargout", "x", "y",
  "theta" };

static const char * c5_d_debug_family_names[4] = { "nargin", "nargout", "deg",
  "rad" };

static const char * c5_e_debug_family_names[5] = { "IR_Position", "nargin",
  "nargout", "IR_Number", "Answer" };

static const char * c5_f_debug_family_names[3] = { "type", "nargin", "nargout" };

static const char * c5_g_debug_family_names[2] = { "nargin", "nargout" };

static const char * c5_h_debug_family_names[6] = { "nargin", "nargout", "x", "y",
  "theta", "R" };

static const char * c5_i_debug_family_names[6] = { "nargin", "nargout", "x", "y",
  "theta", "Answer" };

static const char * c5_j_debug_family_names[12] = { "FixIR_9_Number",
  "sensor_gains", "ir_distances_rf", "u_i", "SumOfObstacleVector_Word",
  "NextAngle", "k", "nargin", "nargout", "Obstacle_AroundRobot_World",
  "Position", "ObstaclesSpeed" };

/* Function Declarations */
static void initialize_c5_BlueBird_Formation
  (SFc5_BlueBird_FormationInstanceStruct *chartInstance);
static void initialize_params_c5_BlueBird_Formation
  (SFc5_BlueBird_FormationInstanceStruct *chartInstance);
static void enable_c5_BlueBird_Formation(SFc5_BlueBird_FormationInstanceStruct
  *chartInstance);
static void disable_c5_BlueBird_Formation(SFc5_BlueBird_FormationInstanceStruct *
  chartInstance);
static void c5_update_debugger_state_c5_BlueBird_Formation
  (SFc5_BlueBird_FormationInstanceStruct *chartInstance);
static const mxArray *get_sim_state_c5_BlueBird_Formation
  (SFc5_BlueBird_FormationInstanceStruct *chartInstance);
static void set_sim_state_c5_BlueBird_Formation
  (SFc5_BlueBird_FormationInstanceStruct *chartInstance, const mxArray *c5_st);
static void finalize_c5_BlueBird_Formation(SFc5_BlueBird_FormationInstanceStruct
  *chartInstance);
static void sf_gateway_c5_BlueBird_Formation
  (SFc5_BlueBird_FormationInstanceStruct *chartInstance);
static void c5_chartstep_c5_BlueBird_Formation
  (SFc5_BlueBird_FormationInstanceStruct *chartInstance);
static void initSimStructsc5_BlueBird_Formation
  (SFc5_BlueBird_FormationInstanceStruct *chartInstance);
static void init_script_number_translation(uint32_T c5_machineNumber, uint32_T
  c5_chartNumber, uint32_T c5_instanceNumber);
static const mxArray *c5_sf_marshallOut(void *chartInstanceVoid, void *c5_inData);
static real_T c5_emlrt_marshallIn(SFc5_BlueBird_FormationInstanceStruct
  *chartInstance, const mxArray *c5_b_Counter, const char_T *c5_identifier);
static real_T c5_b_emlrt_marshallIn(SFc5_BlueBird_FormationInstanceStruct
  *chartInstance, const mxArray *c5_u, const emlrtMsgIdentifier *c5_parentId);
static void c5_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c5_mxArrayInData, const char_T *c5_varName, void *c5_outData);
static const mxArray *c5_b_sf_marshallOut(void *chartInstanceVoid, void
  *c5_inData);
static real_T c5_c_emlrt_marshallIn(SFc5_BlueBird_FormationInstanceStruct
  *chartInstance, const mxArray *c5_V_y, const char_T *c5_identifier);
static real_T c5_d_emlrt_marshallIn(SFc5_BlueBird_FormationInstanceStruct
  *chartInstance, const mxArray *c5_u, const emlrtMsgIdentifier *c5_parentId);
static void c5_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c5_mxArrayInData, const char_T *c5_varName, void *c5_outData);
static const mxArray *c5_c_sf_marshallOut(void *chartInstanceVoid, void
  *c5_inData);
static const mxArray *c5_d_sf_marshallOut(void *chartInstanceVoid, void
  *c5_inData);
static const mxArray *c5_e_sf_marshallOut(void *chartInstanceVoid, void
  *c5_inData);
static void c5_e_emlrt_marshallIn(SFc5_BlueBird_FormationInstanceStruct
  *chartInstance, const mxArray *c5_u, const emlrtMsgIdentifier *c5_parentId,
  real_T c5_y[2]);
static void c5_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c5_mxArrayInData, const char_T *c5_varName, void *c5_outData);
static const mxArray *c5_f_sf_marshallOut(void *chartInstanceVoid, void
  *c5_inData);
static void c5_f_emlrt_marshallIn(SFc5_BlueBird_FormationInstanceStruct
  *chartInstance, const mxArray *c5_u, const emlrtMsgIdentifier *c5_parentId,
  real_T c5_y[27]);
static void c5_d_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c5_mxArrayInData, const char_T *c5_varName, void *c5_outData);
static void c5_g_emlrt_marshallIn(SFc5_BlueBird_FormationInstanceStruct
  *chartInstance, const mxArray *c5_u, const emlrtMsgIdentifier *c5_parentId,
  real_T c5_y[3]);
static void c5_e_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c5_mxArrayInData, const char_T *c5_varName, void *c5_outData);
static const mxArray *c5_g_sf_marshallOut(void *chartInstanceVoid, void
  *c5_inData);
static void c5_h_emlrt_marshallIn(SFc5_BlueBird_FormationInstanceStruct
  *chartInstance, const mxArray *c5_u, const emlrtMsgIdentifier *c5_parentId,
  real_T c5_y[9]);
static void c5_f_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c5_mxArrayInData, const char_T *c5_varName, void *c5_outData);
static void c5_i_emlrt_marshallIn(SFc5_BlueBird_FormationInstanceStruct
  *chartInstance, const mxArray *c5_u, const emlrtMsgIdentifier *c5_parentId,
  real_T c5_y[9]);
static void c5_g_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c5_mxArrayInData, const char_T *c5_varName, void *c5_outData);
static const mxArray *c5_h_sf_marshallOut(void *chartInstanceVoid, real_T
  c5_inData_data[], int32_T *c5_inData_sizes);
static void c5_j_emlrt_marshallIn(SFc5_BlueBird_FormationInstanceStruct
  *chartInstance, const mxArray *c5_u, const emlrtMsgIdentifier *c5_parentId,
  real_T c5_y_data[], int32_T *c5_y_sizes);
static void c5_h_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c5_mxArrayInData, const char_T *c5_varName, real_T c5_outData_data[], int32_T
  *c5_outData_sizes);
static const mxArray *c5_i_sf_marshallOut(void *chartInstanceVoid, void
  *c5_inData);
static void c5_k_emlrt_marshallIn(SFc5_BlueBird_FormationInstanceStruct
  *chartInstance, const mxArray *c5_u, const emlrtMsgIdentifier *c5_parentId,
  real_T c5_y[27]);
static void c5_i_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c5_mxArrayInData, const char_T *c5_varName, void *c5_outData);
static const mxArray *c5_j_sf_marshallOut(void *chartInstanceVoid, void
  *c5_inData);
static void c5_l_emlrt_marshallIn(SFc5_BlueBird_FormationInstanceStruct
  *chartInstance, const mxArray *c5_u, const emlrtMsgIdentifier *c5_parentId,
  char_T c5_y[15]);
static void c5_j_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c5_mxArrayInData, const char_T *c5_varName, void *c5_outData);
static const mxArray *c5_k_sf_marshallOut(void *chartInstanceVoid, void
  *c5_inData);
static void c5_m_emlrt_marshallIn(SFc5_BlueBird_FormationInstanceStruct
  *chartInstance, const mxArray *c5_u, const emlrtMsgIdentifier *c5_parentId,
  real_T c5_y[18]);
static void c5_k_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c5_mxArrayInData, const char_T *c5_varName, void *c5_outData);
static const mxArray *c5_l_sf_marshallOut(void *chartInstanceVoid, void
  *c5_inData);
static void c5_info_helper(const mxArray **c5_info);
static const mxArray *c5_emlrt_marshallOut(const char * c5_u);
static const mxArray *c5_b_emlrt_marshallOut(const uint32_T c5_u);
static void c5_b_info_helper(const mxArray **c5_info);
static void c5_IR_Raw_to_Distances(SFc5_BlueBird_FormationInstanceStruct
  *chartInstance, real_T c5_ir_raw[9], real_T c5_Answer[9], real_T
  *c5_ObstacleDetected);
static void c5_eml_error(SFc5_BlueBird_FormationInstanceStruct *chartInstance);
static void c5_GetIR_Position(SFc5_BlueBird_FormationInstanceStruct
  *chartInstance, real_T c5_IR_Number, real_T c5_Answer[3]);
static void c5_Transformation_Matrix(SFc5_BlueBird_FormationInstanceStruct
  *chartInstance, real_T c5_x, real_T c5_y, real_T c5_theta, real_T c5_Answer[9]);
static void c5_eml_scalar_eg(SFc5_BlueBird_FormationInstanceStruct
  *chartInstance);
static void c5_threshold(SFc5_BlueBird_FormationInstanceStruct *chartInstance);
static void c5_b_eml_scalar_eg(SFc5_BlueBird_FormationInstanceStruct
  *chartInstance);
static void c5_AvoidObstacles_NextPoint(SFc5_BlueBird_FormationInstanceStruct
  *chartInstance, real_T c5_Obstacle_AroundRobot_World[27], real_T c5_Position[3],
  real_T c5_ObstaclesSpeed[2]);
static void c5_repmat(SFc5_BlueBird_FormationInstanceStruct *chartInstance,
                      real_T c5_a[2], real_T c5_b[18]);
static void c5_c_eml_scalar_eg(SFc5_BlueBird_FormationInstanceStruct
  *chartInstance);
static const mxArray *c5_m_sf_marshallOut(void *chartInstanceVoid, void
  *c5_inData);
static int32_T c5_n_emlrt_marshallIn(SFc5_BlueBird_FormationInstanceStruct
  *chartInstance, const mxArray *c5_u, const emlrtMsgIdentifier *c5_parentId);
static void c5_l_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c5_mxArrayInData, const char_T *c5_varName, void *c5_outData);
static uint8_T c5_o_emlrt_marshallIn(SFc5_BlueBird_FormationInstanceStruct
  *chartInstance, const mxArray *c5_b_is_active_c5_BlueBird_Formation, const
  char_T *c5_identifier);
static uint8_T c5_p_emlrt_marshallIn(SFc5_BlueBird_FormationInstanceStruct
  *chartInstance, const mxArray *c5_u, const emlrtMsgIdentifier *c5_parentId);
static void init_dsm_address_info(SFc5_BlueBird_FormationInstanceStruct
  *chartInstance);

/* Function Definitions */
static void initialize_c5_BlueBird_Formation
  (SFc5_BlueBird_FormationInstanceStruct *chartInstance)
{
  chartInstance->c5_sfEvent = CALL_EVENT;
  _sfTime_ = sf_get_time(chartInstance->S);
  chartInstance->c5_Counter_not_empty = false;
  chartInstance->c5_is_active_c5_BlueBird_Formation = 0U;
}

static void initialize_params_c5_BlueBird_Formation
  (SFc5_BlueBird_FormationInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void enable_c5_BlueBird_Formation(SFc5_BlueBird_FormationInstanceStruct
  *chartInstance)
{
  _sfTime_ = sf_get_time(chartInstance->S);
}

static void disable_c5_BlueBird_Formation(SFc5_BlueBird_FormationInstanceStruct *
  chartInstance)
{
  _sfTime_ = sf_get_time(chartInstance->S);
}

static void c5_update_debugger_state_c5_BlueBird_Formation
  (SFc5_BlueBird_FormationInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static const mxArray *get_sim_state_c5_BlueBird_Formation
  (SFc5_BlueBird_FormationInstanceStruct *chartInstance)
{
  const mxArray *c5_st;
  const mxArray *c5_y = NULL;
  real_T c5_hoistedGlobal;
  real_T c5_u;
  const mxArray *c5_b_y = NULL;
  real_T c5_b_hoistedGlobal;
  real_T c5_b_u;
  const mxArray *c5_c_y = NULL;
  real_T c5_c_hoistedGlobal;
  real_T c5_c_u;
  const mxArray *c5_d_y = NULL;
  uint8_T c5_d_hoistedGlobal;
  uint8_T c5_d_u;
  const mxArray *c5_e_y = NULL;
  real_T *c5_V_x;
  real_T *c5_V_y;
  c5_V_y = (real_T *)ssGetOutputPortSignal(chartInstance->S, 2);
  c5_V_x = (real_T *)ssGetOutputPortSignal(chartInstance->S, 1);
  c5_st = NULL;
  c5_st = NULL;
  c5_y = NULL;
  sf_mex_assign(&c5_y, sf_mex_createcellmatrix(4, 1), false);
  c5_hoistedGlobal = *c5_V_x;
  c5_u = c5_hoistedGlobal;
  c5_b_y = NULL;
  sf_mex_assign(&c5_b_y, sf_mex_create("y", &c5_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c5_y, 0, c5_b_y);
  c5_b_hoistedGlobal = *c5_V_y;
  c5_b_u = c5_b_hoistedGlobal;
  c5_c_y = NULL;
  sf_mex_assign(&c5_c_y, sf_mex_create("y", &c5_b_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c5_y, 1, c5_c_y);
  c5_c_hoistedGlobal = chartInstance->c5_Counter;
  c5_c_u = c5_c_hoistedGlobal;
  c5_d_y = NULL;
  if (!chartInstance->c5_Counter_not_empty) {
    sf_mex_assign(&c5_d_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0),
                  false);
  } else {
    sf_mex_assign(&c5_d_y, sf_mex_create("y", &c5_c_u, 0, 0U, 0U, 0U, 0), false);
  }

  sf_mex_setcell(c5_y, 2, c5_d_y);
  c5_d_hoistedGlobal = chartInstance->c5_is_active_c5_BlueBird_Formation;
  c5_d_u = c5_d_hoistedGlobal;
  c5_e_y = NULL;
  sf_mex_assign(&c5_e_y, sf_mex_create("y", &c5_d_u, 3, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c5_y, 3, c5_e_y);
  sf_mex_assign(&c5_st, c5_y, false);
  return c5_st;
}

static void set_sim_state_c5_BlueBird_Formation
  (SFc5_BlueBird_FormationInstanceStruct *chartInstance, const mxArray *c5_st)
{
  const mxArray *c5_u;
  real_T *c5_V_x;
  real_T *c5_V_y;
  c5_V_y = (real_T *)ssGetOutputPortSignal(chartInstance->S, 2);
  c5_V_x = (real_T *)ssGetOutputPortSignal(chartInstance->S, 1);
  chartInstance->c5_doneDoubleBufferReInit = true;
  c5_u = sf_mex_dup(c5_st);
  *c5_V_x = c5_c_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c5_u,
    0)), "V_x");
  *c5_V_y = c5_c_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c5_u,
    1)), "V_y");
  chartInstance->c5_Counter = c5_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c5_u, 2)), "Counter");
  chartInstance->c5_is_active_c5_BlueBird_Formation = c5_o_emlrt_marshallIn
    (chartInstance, sf_mex_dup(sf_mex_getcell(c5_u, 3)),
     "is_active_c5_BlueBird_Formation");
  sf_mex_destroy(&c5_u);
  c5_update_debugger_state_c5_BlueBird_Formation(chartInstance);
  sf_mex_destroy(&c5_st);
}

static void finalize_c5_BlueBird_Formation(SFc5_BlueBird_FormationInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void sf_gateway_c5_BlueBird_Formation
  (SFc5_BlueBird_FormationInstanceStruct *chartInstance)
{
  int32_T c5_i0;
  int32_T c5_i1;
  real_T *c5_V_x_In;
  real_T *c5_V_x;
  real_T *c5_V_y_In;
  real_T *c5_V_y;
  real_T (*c5_Position)[3];
  real_T (*c5_ir_raw)[9];
  c5_Position = (real_T (*)[3])ssGetInputPortSignal(chartInstance->S, 3);
  c5_ir_raw = (real_T (*)[9])ssGetInputPortSignal(chartInstance->S, 2);
  c5_V_y = (real_T *)ssGetOutputPortSignal(chartInstance->S, 2);
  c5_V_y_In = (real_T *)ssGetInputPortSignal(chartInstance->S, 1);
  c5_V_x = (real_T *)ssGetOutputPortSignal(chartInstance->S, 1);
  c5_V_x_In = (real_T *)ssGetInputPortSignal(chartInstance->S, 0);
  _SFD_SYMBOL_SCOPE_PUSH(0U, 0U);
  _sfTime_ = sf_get_time(chartInstance->S);
  _SFD_CC_CALL(CHART_ENTER_SFUNCTION_TAG, 4U, chartInstance->c5_sfEvent);
  _SFD_DATA_RANGE_CHECK(*c5_V_x_In, 0U);
  chartInstance->c5_sfEvent = CALL_EVENT;
  c5_chartstep_c5_BlueBird_Formation(chartInstance);
  _SFD_SYMBOL_SCOPE_POP();
  _SFD_CHECK_FOR_STATE_INCONSISTENCY(_BlueBird_FormationMachineNumber_,
    chartInstance->chartNumber, chartInstance->instanceNumber);
  _SFD_DATA_RANGE_CHECK(*c5_V_x, 1U);
  _SFD_DATA_RANGE_CHECK(*c5_V_y_In, 2U);
  _SFD_DATA_RANGE_CHECK(*c5_V_y, 3U);
  for (c5_i0 = 0; c5_i0 < 9; c5_i0++) {
    _SFD_DATA_RANGE_CHECK((*c5_ir_raw)[c5_i0], 4U);
  }

  for (c5_i1 = 0; c5_i1 < 3; c5_i1++) {
    _SFD_DATA_RANGE_CHECK((*c5_Position)[c5_i1], 5U);
  }
}

static void c5_chartstep_c5_BlueBird_Formation
  (SFc5_BlueBird_FormationInstanceStruct *chartInstance)
{
  real_T c5_hoistedGlobal;
  real_T c5_b_hoistedGlobal;
  real_T c5_V_x_In;
  real_T c5_V_y_In;
  int32_T c5_i2;
  real_T c5_ir_raw[9];
  int32_T c5_i3;
  real_T c5_Position[3];
  uint32_T c5_debug_family_var_map[20];
  real_T c5_ir_distances[9];
  real_T c5_ObstacleDetected;
  real_T c5_IF_Obstacle;
  real_T c5_FixIR_9_Number;
  real_T c5_Obstacle_AroundRobot[27];
  real_T c5_i;
  real_T c5_IR_Position_I[3];
  real_T c5_IR_Transformation_Matrix_I[9];
  real_T c5_Obstacle_AroundRobot_I[3];
  real_T c5_Obstacle_AroundRobot_World[27];
  real_T c5_ObstaclesSpeed[2];
  real_T c5_nargin = 4.0;
  real_T c5_nargout = 2.0;
  real_T c5_V_x;
  real_T c5_V_y;
  int32_T c5_i4;
  real_T c5_b_ir_raw[9];
  real_T c5_b_ObstacleDetected;
  real_T c5_b_ir_distances[9];
  int32_T c5_i5;
  int32_T c5_i6;
  static real_T c5_y[9] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.364 };

  int32_T c5_i7;
  int32_T c5_b_i;
  real_T c5_dv0[3];
  int32_T c5_i8;
  real_T c5_dv1[9];
  int32_T c5_i9;
  int32_T c5_i10;
  real_T c5_a[9];
  real_T c5_b[3];
  int32_T c5_i11;
  int32_T c5_i12;
  int32_T c5_i13;
  real_T c5_C[3];
  int32_T c5_i14;
  int32_T c5_i15;
  int32_T c5_i16;
  int32_T c5_i17;
  int32_T c5_i18;
  int32_T c5_i19;
  int32_T c5_c_i;
  int32_T c5_i20;
  int32_T c5_i21;
  real_T c5_b_b[27];
  int32_T c5_i22;
  int32_T c5_i23;
  int32_T c5_i24;
  real_T c5_b_C[27];
  int32_T c5_i25;
  int32_T c5_i26;
  int32_T c5_i27;
  int32_T c5_i28;
  int32_T c5_i29;
  int32_T c5_i30;
  int32_T c5_i31;
  int32_T c5_i32;
  int32_T c5_i33;
  real_T c5_b_Obstacle_AroundRobot_World[27];
  int32_T c5_i34;
  real_T c5_b_Position[3];
  real_T c5_dv2[2];
  int32_T c5_i35;
  real_T *c5_b_V_x_In;
  real_T *c5_b_V_y_In;
  real_T *c5_b_V_x;
  real_T *c5_b_V_y;
  real_T (*c5_c_Position)[3];
  real_T (*c5_c_ir_raw)[9];
  c5_c_Position = (real_T (*)[3])ssGetInputPortSignal(chartInstance->S, 3);
  c5_c_ir_raw = (real_T (*)[9])ssGetInputPortSignal(chartInstance->S, 2);
  c5_b_V_y = (real_T *)ssGetOutputPortSignal(chartInstance->S, 2);
  c5_b_V_y_In = (real_T *)ssGetInputPortSignal(chartInstance->S, 1);
  c5_b_V_x = (real_T *)ssGetOutputPortSignal(chartInstance->S, 1);
  c5_b_V_x_In = (real_T *)ssGetInputPortSignal(chartInstance->S, 0);
  _SFD_CC_CALL(CHART_ENTER_DURING_FUNCTION_TAG, 4U, chartInstance->c5_sfEvent);
  c5_hoistedGlobal = *c5_b_V_x_In;
  c5_b_hoistedGlobal = *c5_b_V_y_In;
  c5_V_x_In = c5_hoistedGlobal;
  c5_V_y_In = c5_b_hoistedGlobal;
  for (c5_i2 = 0; c5_i2 < 9; c5_i2++) {
    c5_ir_raw[c5_i2] = (*c5_c_ir_raw)[c5_i2];
  }

  for (c5_i3 = 0; c5_i3 < 3; c5_i3++) {
    c5_Position[c5_i3] = (*c5_c_Position)[c5_i3];
  }

  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 20U, 20U, c5_debug_family_names,
    c5_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c5_ir_distances, 0U, c5_d_sf_marshallOut,
    c5_g_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c5_ObstacleDetected, 1U,
    c5_b_sf_marshallOut, c5_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c5_IF_Obstacle, 2U, c5_b_sf_marshallOut,
    c5_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c5_FixIR_9_Number, 3U, c5_b_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c5_Obstacle_AroundRobot, 4U,
    c5_f_sf_marshallOut, c5_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c5_i, 5U, c5_b_sf_marshallOut,
    c5_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c5_IR_Position_I, 6U, c5_c_sf_marshallOut,
    c5_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c5_IR_Transformation_Matrix_I, 7U,
    c5_g_sf_marshallOut, c5_f_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c5_Obstacle_AroundRobot_I, 8U,
    c5_c_sf_marshallOut, c5_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c5_Obstacle_AroundRobot_World, 9U,
    c5_f_sf_marshallOut, c5_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c5_ObstaclesSpeed, 10U,
    c5_e_sf_marshallOut, c5_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c5_nargin, 11U, c5_b_sf_marshallOut,
    c5_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c5_nargout, 12U, c5_b_sf_marshallOut,
    c5_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c5_V_x_In, 13U, c5_b_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c5_V_y_In, 14U, c5_b_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c5_ir_raw, 15U, c5_d_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c5_Position, 16U, c5_c_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c5_V_x, 17U, c5_b_sf_marshallOut,
    c5_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c5_V_y, 18U, c5_b_sf_marshallOut,
    c5_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&chartInstance->c5_Counter, 19U,
    c5_sf_marshallOut, c5_sf_marshallIn);
  CV_EML_FCN(0, 0);
  _SFD_EML_CALL(0U, chartInstance->c5_sfEvent, 3);
  _SFD_EML_CALL(0U, chartInstance->c5_sfEvent, 4);
  if (CV_EML_IF(0, 1, 0, !chartInstance->c5_Counter_not_empty)) {
    _SFD_EML_CALL(0U, chartInstance->c5_sfEvent, 5);
    chartInstance->c5_Counter = 0.0;
    chartInstance->c5_Counter_not_empty = true;
  }

  _SFD_EML_CALL(0U, chartInstance->c5_sfEvent, 9);
  for (c5_i4 = 0; c5_i4 < 9; c5_i4++) {
    c5_b_ir_raw[c5_i4] = c5_ir_raw[c5_i4];
  }

  c5_IR_Raw_to_Distances(chartInstance, c5_b_ir_raw, c5_b_ir_distances,
    &c5_b_ObstacleDetected);
  for (c5_i5 = 0; c5_i5 < 9; c5_i5++) {
    c5_ir_distances[c5_i5] = c5_b_ir_distances[c5_i5];
  }

  c5_ObstacleDetected = c5_b_ObstacleDetected;
  _SFD_EML_CALL(0U, chartInstance->c5_sfEvent, 10);
  if (CV_EML_IF(0, 1, 1, c5_ObstacleDetected > 0.0)) {
    _SFD_EML_CALL(0U, chartInstance->c5_sfEvent, 11);
    c5_IF_Obstacle = 1.0;
    _SFD_EML_CALL(0U, chartInstance->c5_sfEvent, 12);
    chartInstance->c5_Counter = 25.0;
  }

  _SFD_EML_CALL(0U, chartInstance->c5_sfEvent, 15);
  if (CV_EML_IF(0, 1, 2, chartInstance->c5_Counter > 0.0)) {
    _SFD_EML_CALL(0U, chartInstance->c5_sfEvent, 16);
    c5_IF_Obstacle = 1.0;
    _SFD_EML_CALL(0U, chartInstance->c5_sfEvent, 17);
    chartInstance->c5_Counter--;
  } else {
    _SFD_EML_CALL(0U, chartInstance->c5_sfEvent, 19);
    c5_IF_Obstacle = -1.0;
  }

  _SFD_EML_CALL(0U, chartInstance->c5_sfEvent, 22);
  if (CV_EML_IF(0, 1, 3, c5_IF_Obstacle > 0.0)) {
    _SFD_EML_CALL(0U, chartInstance->c5_sfEvent, 25);
    c5_FixIR_9_Number = 0.364;
    _SFD_EML_CALL(0U, chartInstance->c5_sfEvent, 26);
    for (c5_i6 = 0; c5_i6 < 9; c5_i6++) {
      c5_ir_distances[c5_i6] += c5_y[c5_i6];
    }

    _SFD_EML_CALL(0U, chartInstance->c5_sfEvent, 29);
    for (c5_i7 = 0; c5_i7 < 27; c5_i7++) {
      c5_Obstacle_AroundRobot[c5_i7] = 0.0;
    }

    _SFD_EML_CALL(0U, chartInstance->c5_sfEvent, 31);
    c5_i = 1.0;
    c5_b_i = 0;
    while (c5_b_i < 9) {
      c5_i = 1.0 + (real_T)c5_b_i;
      CV_EML_FOR(0, 1, 0, 1);
      _SFD_EML_CALL(0U, chartInstance->c5_sfEvent, 32);
      c5_GetIR_Position(chartInstance, c5_i, c5_dv0);
      for (c5_i8 = 0; c5_i8 < 3; c5_i8++) {
        c5_IR_Position_I[c5_i8] = c5_dv0[c5_i8];
      }

      _SFD_EML_CALL(0U, chartInstance->c5_sfEvent, 33);
      c5_Transformation_Matrix(chartInstance, c5_IR_Position_I[0],
        c5_IR_Position_I[1], c5_IR_Position_I[2], c5_dv1);
      for (c5_i9 = 0; c5_i9 < 9; c5_i9++) {
        c5_IR_Transformation_Matrix_I[c5_i9] = c5_dv1[c5_i9];
      }

      _SFD_EML_CALL(0U, chartInstance->c5_sfEvent, 34);
      for (c5_i10 = 0; c5_i10 < 9; c5_i10++) {
        c5_a[c5_i10] = c5_IR_Transformation_Matrix_I[c5_i10];
      }

      c5_b[0] = c5_ir_distances[(int32_T)c5_i - 1];
      c5_b[1] = 0.0;
      c5_b[2] = 1.0;
      c5_eml_scalar_eg(chartInstance);
      c5_eml_scalar_eg(chartInstance);
      for (c5_i11 = 0; c5_i11 < 3; c5_i11++) {
        c5_Obstacle_AroundRobot_I[c5_i11] = 0.0;
      }

      for (c5_i12 = 0; c5_i12 < 3; c5_i12++) {
        c5_Obstacle_AroundRobot_I[c5_i12] = 0.0;
      }

      for (c5_i13 = 0; c5_i13 < 3; c5_i13++) {
        c5_C[c5_i13] = c5_Obstacle_AroundRobot_I[c5_i13];
      }

      for (c5_i14 = 0; c5_i14 < 3; c5_i14++) {
        c5_Obstacle_AroundRobot_I[c5_i14] = c5_C[c5_i14];
      }

      c5_threshold(chartInstance);
      for (c5_i15 = 0; c5_i15 < 3; c5_i15++) {
        c5_C[c5_i15] = c5_Obstacle_AroundRobot_I[c5_i15];
      }

      for (c5_i16 = 0; c5_i16 < 3; c5_i16++) {
        c5_Obstacle_AroundRobot_I[c5_i16] = c5_C[c5_i16];
      }

      for (c5_i17 = 0; c5_i17 < 3; c5_i17++) {
        c5_Obstacle_AroundRobot_I[c5_i17] = 0.0;
        c5_i18 = 0;
        for (c5_i19 = 0; c5_i19 < 3; c5_i19++) {
          c5_Obstacle_AroundRobot_I[c5_i17] += c5_a[c5_i18 + c5_i17] *
            c5_b[c5_i19];
          c5_i18 += 3;
        }
      }

      _SFD_EML_CALL(0U, chartInstance->c5_sfEvent, 36);
      c5_c_i = (int32_T)c5_i - 1;
      for (c5_i20 = 0; c5_i20 < 3; c5_i20++) {
        c5_Obstacle_AroundRobot[c5_i20 + 3 * c5_c_i] =
          c5_Obstacle_AroundRobot_I[c5_i20];
      }

      c5_b_i++;
      _SF_MEX_LISTEN_FOR_CTRL_C(chartInstance->S);
    }

    CV_EML_FOR(0, 1, 0, 0);
    _SFD_EML_CALL(0U, chartInstance->c5_sfEvent, 39);
    c5_Transformation_Matrix(chartInstance, c5_Position[0], c5_Position[1],
      c5_Position[2], c5_a);
    for (c5_i21 = 0; c5_i21 < 27; c5_i21++) {
      c5_b_b[c5_i21] = c5_Obstacle_AroundRobot[c5_i21];
    }

    c5_b_eml_scalar_eg(chartInstance);
    c5_b_eml_scalar_eg(chartInstance);
    for (c5_i22 = 0; c5_i22 < 27; c5_i22++) {
      c5_Obstacle_AroundRobot_World[c5_i22] = 0.0;
    }

    for (c5_i23 = 0; c5_i23 < 27; c5_i23++) {
      c5_Obstacle_AroundRobot_World[c5_i23] = 0.0;
    }

    for (c5_i24 = 0; c5_i24 < 27; c5_i24++) {
      c5_b_C[c5_i24] = c5_Obstacle_AroundRobot_World[c5_i24];
    }

    for (c5_i25 = 0; c5_i25 < 27; c5_i25++) {
      c5_Obstacle_AroundRobot_World[c5_i25] = c5_b_C[c5_i25];
    }

    c5_threshold(chartInstance);
    for (c5_i26 = 0; c5_i26 < 27; c5_i26++) {
      c5_b_C[c5_i26] = c5_Obstacle_AroundRobot_World[c5_i26];
    }

    for (c5_i27 = 0; c5_i27 < 27; c5_i27++) {
      c5_Obstacle_AroundRobot_World[c5_i27] = c5_b_C[c5_i27];
    }

    for (c5_i28 = 0; c5_i28 < 3; c5_i28++) {
      c5_i29 = 0;
      for (c5_i30 = 0; c5_i30 < 9; c5_i30++) {
        c5_Obstacle_AroundRobot_World[c5_i29 + c5_i28] = 0.0;
        c5_i31 = 0;
        for (c5_i32 = 0; c5_i32 < 3; c5_i32++) {
          c5_Obstacle_AroundRobot_World[c5_i29 + c5_i28] += c5_a[c5_i31 + c5_i28]
            * c5_b_b[c5_i32 + c5_i29];
          c5_i31 += 3;
        }

        c5_i29 += 3;
      }
    }

    _SFD_EML_CALL(0U, chartInstance->c5_sfEvent, 40);
    for (c5_i33 = 0; c5_i33 < 27; c5_i33++) {
      c5_b_Obstacle_AroundRobot_World[c5_i33] =
        c5_Obstacle_AroundRobot_World[c5_i33];
    }

    for (c5_i34 = 0; c5_i34 < 3; c5_i34++) {
      c5_b_Position[c5_i34] = c5_Position[c5_i34];
    }

    c5_AvoidObstacles_NextPoint(chartInstance, c5_b_Obstacle_AroundRobot_World,
      c5_b_Position, c5_dv2);
    for (c5_i35 = 0; c5_i35 < 2; c5_i35++) {
      c5_ObstaclesSpeed[c5_i35] = c5_dv2[c5_i35];
    }

    _SFD_EML_CALL(0U, chartInstance->c5_sfEvent, 41);
    c5_V_x = c5_ObstaclesSpeed[0];
    _SFD_EML_CALL(0U, chartInstance->c5_sfEvent, 42);
    c5_V_y = c5_ObstaclesSpeed[1];
  } else {
    _SFD_EML_CALL(0U, chartInstance->c5_sfEvent, 45);
    c5_V_x = c5_V_x_In;
    _SFD_EML_CALL(0U, chartInstance->c5_sfEvent, 46);
    c5_V_y = c5_V_y_In;
  }

  _SFD_EML_CALL(0U, chartInstance->c5_sfEvent, -46);
  _SFD_SYMBOL_SCOPE_POP();
  *c5_b_V_x = c5_V_x;
  *c5_b_V_y = c5_V_y;
  _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG, 4U, chartInstance->c5_sfEvent);
}

static void initSimStructsc5_BlueBird_Formation
  (SFc5_BlueBird_FormationInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void init_script_number_translation(uint32_T c5_machineNumber, uint32_T
  c5_chartNumber, uint32_T c5_instanceNumber)
{
  (void)c5_machineNumber;
  _SFD_SCRIPT_TRANSLATION(c5_chartNumber, c5_instanceNumber, 0U,
    sf_debug_get_script_id(
    "F:\\BlueBird\\Study\\Robot\\Location\\20140925\\+simiam\\+ui\\Pose2D.m"));
  _SFD_SCRIPT_TRANSLATION(c5_chartNumber, c5_instanceNumber, 1U,
    sf_debug_get_script_id(
    "F:\\BlueBird\\Study\\Robot\\Location\\20140925\\+simiam\\+controller\\Controller.m"));
  _SFD_SCRIPT_TRANSLATION(c5_chartNumber, c5_instanceNumber, 2U,
    sf_debug_get_script_id(
    "F:\\BlueBird\\Study\\Robot\\Location\\20140925\\+simiam\\+controller\\AvoidObstacles.m"));
}

static const mxArray *c5_sf_marshallOut(void *chartInstanceVoid, void *c5_inData)
{
  const mxArray *c5_mxArrayOutData = NULL;
  real_T c5_u;
  const mxArray *c5_y = NULL;
  SFc5_BlueBird_FormationInstanceStruct *chartInstance;
  chartInstance = (SFc5_BlueBird_FormationInstanceStruct *)chartInstanceVoid;
  c5_mxArrayOutData = NULL;
  c5_u = *(real_T *)c5_inData;
  c5_y = NULL;
  if (!chartInstance->c5_Counter_not_empty) {
    sf_mex_assign(&c5_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0), false);
  } else {
    sf_mex_assign(&c5_y, sf_mex_create("y", &c5_u, 0, 0U, 0U, 0U, 0), false);
  }

  sf_mex_assign(&c5_mxArrayOutData, c5_y, false);
  return c5_mxArrayOutData;
}

static real_T c5_emlrt_marshallIn(SFc5_BlueBird_FormationInstanceStruct
  *chartInstance, const mxArray *c5_b_Counter, const char_T *c5_identifier)
{
  real_T c5_y;
  emlrtMsgIdentifier c5_thisId;
  c5_thisId.fIdentifier = c5_identifier;
  c5_thisId.fParent = NULL;
  c5_y = c5_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c5_b_Counter),
    &c5_thisId);
  sf_mex_destroy(&c5_b_Counter);
  return c5_y;
}

static real_T c5_b_emlrt_marshallIn(SFc5_BlueBird_FormationInstanceStruct
  *chartInstance, const mxArray *c5_u, const emlrtMsgIdentifier *c5_parentId)
{
  real_T c5_y;
  real_T c5_d0;
  if (mxIsEmpty(c5_u)) {
    chartInstance->c5_Counter_not_empty = false;
  } else {
    chartInstance->c5_Counter_not_empty = true;
    sf_mex_import(c5_parentId, sf_mex_dup(c5_u), &c5_d0, 1, 0, 0U, 0, 0U, 0);
    c5_y = c5_d0;
  }

  sf_mex_destroy(&c5_u);
  return c5_y;
}

static void c5_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c5_mxArrayInData, const char_T *c5_varName, void *c5_outData)
{
  const mxArray *c5_b_Counter;
  const char_T *c5_identifier;
  emlrtMsgIdentifier c5_thisId;
  real_T c5_y;
  SFc5_BlueBird_FormationInstanceStruct *chartInstance;
  chartInstance = (SFc5_BlueBird_FormationInstanceStruct *)chartInstanceVoid;
  c5_b_Counter = sf_mex_dup(c5_mxArrayInData);
  c5_identifier = c5_varName;
  c5_thisId.fIdentifier = c5_identifier;
  c5_thisId.fParent = NULL;
  c5_y = c5_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c5_b_Counter),
    &c5_thisId);
  sf_mex_destroy(&c5_b_Counter);
  *(real_T *)c5_outData = c5_y;
  sf_mex_destroy(&c5_mxArrayInData);
}

static const mxArray *c5_b_sf_marshallOut(void *chartInstanceVoid, void
  *c5_inData)
{
  const mxArray *c5_mxArrayOutData = NULL;
  real_T c5_u;
  const mxArray *c5_y = NULL;
  SFc5_BlueBird_FormationInstanceStruct *chartInstance;
  chartInstance = (SFc5_BlueBird_FormationInstanceStruct *)chartInstanceVoid;
  c5_mxArrayOutData = NULL;
  c5_u = *(real_T *)c5_inData;
  c5_y = NULL;
  sf_mex_assign(&c5_y, sf_mex_create("y", &c5_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_assign(&c5_mxArrayOutData, c5_y, false);
  return c5_mxArrayOutData;
}

static real_T c5_c_emlrt_marshallIn(SFc5_BlueBird_FormationInstanceStruct
  *chartInstance, const mxArray *c5_V_y, const char_T *c5_identifier)
{
  real_T c5_y;
  emlrtMsgIdentifier c5_thisId;
  c5_thisId.fIdentifier = c5_identifier;
  c5_thisId.fParent = NULL;
  c5_y = c5_d_emlrt_marshallIn(chartInstance, sf_mex_dup(c5_V_y), &c5_thisId);
  sf_mex_destroy(&c5_V_y);
  return c5_y;
}

static real_T c5_d_emlrt_marshallIn(SFc5_BlueBird_FormationInstanceStruct
  *chartInstance, const mxArray *c5_u, const emlrtMsgIdentifier *c5_parentId)
{
  real_T c5_y;
  real_T c5_d1;
  (void)chartInstance;
  sf_mex_import(c5_parentId, sf_mex_dup(c5_u), &c5_d1, 1, 0, 0U, 0, 0U, 0);
  c5_y = c5_d1;
  sf_mex_destroy(&c5_u);
  return c5_y;
}

static void c5_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c5_mxArrayInData, const char_T *c5_varName, void *c5_outData)
{
  const mxArray *c5_V_y;
  const char_T *c5_identifier;
  emlrtMsgIdentifier c5_thisId;
  real_T c5_y;
  SFc5_BlueBird_FormationInstanceStruct *chartInstance;
  chartInstance = (SFc5_BlueBird_FormationInstanceStruct *)chartInstanceVoid;
  c5_V_y = sf_mex_dup(c5_mxArrayInData);
  c5_identifier = c5_varName;
  c5_thisId.fIdentifier = c5_identifier;
  c5_thisId.fParent = NULL;
  c5_y = c5_d_emlrt_marshallIn(chartInstance, sf_mex_dup(c5_V_y), &c5_thisId);
  sf_mex_destroy(&c5_V_y);
  *(real_T *)c5_outData = c5_y;
  sf_mex_destroy(&c5_mxArrayInData);
}

static const mxArray *c5_c_sf_marshallOut(void *chartInstanceVoid, void
  *c5_inData)
{
  const mxArray *c5_mxArrayOutData = NULL;
  int32_T c5_i36;
  real_T c5_b_inData[3];
  int32_T c5_i37;
  real_T c5_u[3];
  const mxArray *c5_y = NULL;
  SFc5_BlueBird_FormationInstanceStruct *chartInstance;
  chartInstance = (SFc5_BlueBird_FormationInstanceStruct *)chartInstanceVoid;
  c5_mxArrayOutData = NULL;
  for (c5_i36 = 0; c5_i36 < 3; c5_i36++) {
    c5_b_inData[c5_i36] = (*(real_T (*)[3])c5_inData)[c5_i36];
  }

  for (c5_i37 = 0; c5_i37 < 3; c5_i37++) {
    c5_u[c5_i37] = c5_b_inData[c5_i37];
  }

  c5_y = NULL;
  sf_mex_assign(&c5_y, sf_mex_create("y", c5_u, 0, 0U, 1U, 0U, 1, 3), false);
  sf_mex_assign(&c5_mxArrayOutData, c5_y, false);
  return c5_mxArrayOutData;
}

static const mxArray *c5_d_sf_marshallOut(void *chartInstanceVoid, void
  *c5_inData)
{
  const mxArray *c5_mxArrayOutData = NULL;
  int32_T c5_i38;
  real_T c5_b_inData[9];
  int32_T c5_i39;
  real_T c5_u[9];
  const mxArray *c5_y = NULL;
  SFc5_BlueBird_FormationInstanceStruct *chartInstance;
  chartInstance = (SFc5_BlueBird_FormationInstanceStruct *)chartInstanceVoid;
  c5_mxArrayOutData = NULL;
  for (c5_i38 = 0; c5_i38 < 9; c5_i38++) {
    c5_b_inData[c5_i38] = (*(real_T (*)[9])c5_inData)[c5_i38];
  }

  for (c5_i39 = 0; c5_i39 < 9; c5_i39++) {
    c5_u[c5_i39] = c5_b_inData[c5_i39];
  }

  c5_y = NULL;
  sf_mex_assign(&c5_y, sf_mex_create("y", c5_u, 0, 0U, 1U, 0U, 1, 9), false);
  sf_mex_assign(&c5_mxArrayOutData, c5_y, false);
  return c5_mxArrayOutData;
}

static const mxArray *c5_e_sf_marshallOut(void *chartInstanceVoid, void
  *c5_inData)
{
  const mxArray *c5_mxArrayOutData = NULL;
  int32_T c5_i40;
  real_T c5_b_inData[2];
  int32_T c5_i41;
  real_T c5_u[2];
  const mxArray *c5_y = NULL;
  SFc5_BlueBird_FormationInstanceStruct *chartInstance;
  chartInstance = (SFc5_BlueBird_FormationInstanceStruct *)chartInstanceVoid;
  c5_mxArrayOutData = NULL;
  for (c5_i40 = 0; c5_i40 < 2; c5_i40++) {
    c5_b_inData[c5_i40] = (*(real_T (*)[2])c5_inData)[c5_i40];
  }

  for (c5_i41 = 0; c5_i41 < 2; c5_i41++) {
    c5_u[c5_i41] = c5_b_inData[c5_i41];
  }

  c5_y = NULL;
  sf_mex_assign(&c5_y, sf_mex_create("y", c5_u, 0, 0U, 1U, 0U, 1, 2), false);
  sf_mex_assign(&c5_mxArrayOutData, c5_y, false);
  return c5_mxArrayOutData;
}

static void c5_e_emlrt_marshallIn(SFc5_BlueBird_FormationInstanceStruct
  *chartInstance, const mxArray *c5_u, const emlrtMsgIdentifier *c5_parentId,
  real_T c5_y[2])
{
  real_T c5_dv3[2];
  int32_T c5_i42;
  (void)chartInstance;
  sf_mex_import(c5_parentId, sf_mex_dup(c5_u), c5_dv3, 1, 0, 0U, 1, 0U, 1, 2);
  for (c5_i42 = 0; c5_i42 < 2; c5_i42++) {
    c5_y[c5_i42] = c5_dv3[c5_i42];
  }

  sf_mex_destroy(&c5_u);
}

static void c5_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c5_mxArrayInData, const char_T *c5_varName, void *c5_outData)
{
  const mxArray *c5_ObstaclesSpeed;
  const char_T *c5_identifier;
  emlrtMsgIdentifier c5_thisId;
  real_T c5_y[2];
  int32_T c5_i43;
  SFc5_BlueBird_FormationInstanceStruct *chartInstance;
  chartInstance = (SFc5_BlueBird_FormationInstanceStruct *)chartInstanceVoid;
  c5_ObstaclesSpeed = sf_mex_dup(c5_mxArrayInData);
  c5_identifier = c5_varName;
  c5_thisId.fIdentifier = c5_identifier;
  c5_thisId.fParent = NULL;
  c5_e_emlrt_marshallIn(chartInstance, sf_mex_dup(c5_ObstaclesSpeed), &c5_thisId,
                        c5_y);
  sf_mex_destroy(&c5_ObstaclesSpeed);
  for (c5_i43 = 0; c5_i43 < 2; c5_i43++) {
    (*(real_T (*)[2])c5_outData)[c5_i43] = c5_y[c5_i43];
  }

  sf_mex_destroy(&c5_mxArrayInData);
}

static const mxArray *c5_f_sf_marshallOut(void *chartInstanceVoid, void
  *c5_inData)
{
  const mxArray *c5_mxArrayOutData = NULL;
  int32_T c5_i44;
  int32_T c5_i45;
  int32_T c5_i46;
  real_T c5_b_inData[27];
  int32_T c5_i47;
  int32_T c5_i48;
  int32_T c5_i49;
  real_T c5_u[27];
  const mxArray *c5_y = NULL;
  SFc5_BlueBird_FormationInstanceStruct *chartInstance;
  chartInstance = (SFc5_BlueBird_FormationInstanceStruct *)chartInstanceVoid;
  c5_mxArrayOutData = NULL;
  c5_i44 = 0;
  for (c5_i45 = 0; c5_i45 < 9; c5_i45++) {
    for (c5_i46 = 0; c5_i46 < 3; c5_i46++) {
      c5_b_inData[c5_i46 + c5_i44] = (*(real_T (*)[27])c5_inData)[c5_i46 +
        c5_i44];
    }

    c5_i44 += 3;
  }

  c5_i47 = 0;
  for (c5_i48 = 0; c5_i48 < 9; c5_i48++) {
    for (c5_i49 = 0; c5_i49 < 3; c5_i49++) {
      c5_u[c5_i49 + c5_i47] = c5_b_inData[c5_i49 + c5_i47];
    }

    c5_i47 += 3;
  }

  c5_y = NULL;
  sf_mex_assign(&c5_y, sf_mex_create("y", c5_u, 0, 0U, 1U, 0U, 2, 3, 9), false);
  sf_mex_assign(&c5_mxArrayOutData, c5_y, false);
  return c5_mxArrayOutData;
}

static void c5_f_emlrt_marshallIn(SFc5_BlueBird_FormationInstanceStruct
  *chartInstance, const mxArray *c5_u, const emlrtMsgIdentifier *c5_parentId,
  real_T c5_y[27])
{
  real_T c5_dv4[27];
  int32_T c5_i50;
  (void)chartInstance;
  sf_mex_import(c5_parentId, sf_mex_dup(c5_u), c5_dv4, 1, 0, 0U, 1, 0U, 2, 3, 9);
  for (c5_i50 = 0; c5_i50 < 27; c5_i50++) {
    c5_y[c5_i50] = c5_dv4[c5_i50];
  }

  sf_mex_destroy(&c5_u);
}

static void c5_d_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c5_mxArrayInData, const char_T *c5_varName, void *c5_outData)
{
  const mxArray *c5_Obstacle_AroundRobot_World;
  const char_T *c5_identifier;
  emlrtMsgIdentifier c5_thisId;
  real_T c5_y[27];
  int32_T c5_i51;
  int32_T c5_i52;
  int32_T c5_i53;
  SFc5_BlueBird_FormationInstanceStruct *chartInstance;
  chartInstance = (SFc5_BlueBird_FormationInstanceStruct *)chartInstanceVoid;
  c5_Obstacle_AroundRobot_World = sf_mex_dup(c5_mxArrayInData);
  c5_identifier = c5_varName;
  c5_thisId.fIdentifier = c5_identifier;
  c5_thisId.fParent = NULL;
  c5_f_emlrt_marshallIn(chartInstance, sf_mex_dup(c5_Obstacle_AroundRobot_World),
                        &c5_thisId, c5_y);
  sf_mex_destroy(&c5_Obstacle_AroundRobot_World);
  c5_i51 = 0;
  for (c5_i52 = 0; c5_i52 < 9; c5_i52++) {
    for (c5_i53 = 0; c5_i53 < 3; c5_i53++) {
      (*(real_T (*)[27])c5_outData)[c5_i53 + c5_i51] = c5_y[c5_i53 + c5_i51];
    }

    c5_i51 += 3;
  }

  sf_mex_destroy(&c5_mxArrayInData);
}

static void c5_g_emlrt_marshallIn(SFc5_BlueBird_FormationInstanceStruct
  *chartInstance, const mxArray *c5_u, const emlrtMsgIdentifier *c5_parentId,
  real_T c5_y[3])
{
  real_T c5_dv5[3];
  int32_T c5_i54;
  (void)chartInstance;
  sf_mex_import(c5_parentId, sf_mex_dup(c5_u), c5_dv5, 1, 0, 0U, 1, 0U, 1, 3);
  for (c5_i54 = 0; c5_i54 < 3; c5_i54++) {
    c5_y[c5_i54] = c5_dv5[c5_i54];
  }

  sf_mex_destroy(&c5_u);
}

static void c5_e_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c5_mxArrayInData, const char_T *c5_varName, void *c5_outData)
{
  const mxArray *c5_Obstacle_AroundRobot_I;
  const char_T *c5_identifier;
  emlrtMsgIdentifier c5_thisId;
  real_T c5_y[3];
  int32_T c5_i55;
  SFc5_BlueBird_FormationInstanceStruct *chartInstance;
  chartInstance = (SFc5_BlueBird_FormationInstanceStruct *)chartInstanceVoid;
  c5_Obstacle_AroundRobot_I = sf_mex_dup(c5_mxArrayInData);
  c5_identifier = c5_varName;
  c5_thisId.fIdentifier = c5_identifier;
  c5_thisId.fParent = NULL;
  c5_g_emlrt_marshallIn(chartInstance, sf_mex_dup(c5_Obstacle_AroundRobot_I),
                        &c5_thisId, c5_y);
  sf_mex_destroy(&c5_Obstacle_AroundRobot_I);
  for (c5_i55 = 0; c5_i55 < 3; c5_i55++) {
    (*(real_T (*)[3])c5_outData)[c5_i55] = c5_y[c5_i55];
  }

  sf_mex_destroy(&c5_mxArrayInData);
}

static const mxArray *c5_g_sf_marshallOut(void *chartInstanceVoid, void
  *c5_inData)
{
  const mxArray *c5_mxArrayOutData = NULL;
  int32_T c5_i56;
  int32_T c5_i57;
  int32_T c5_i58;
  real_T c5_b_inData[9];
  int32_T c5_i59;
  int32_T c5_i60;
  int32_T c5_i61;
  real_T c5_u[9];
  const mxArray *c5_y = NULL;
  SFc5_BlueBird_FormationInstanceStruct *chartInstance;
  chartInstance = (SFc5_BlueBird_FormationInstanceStruct *)chartInstanceVoid;
  c5_mxArrayOutData = NULL;
  c5_i56 = 0;
  for (c5_i57 = 0; c5_i57 < 3; c5_i57++) {
    for (c5_i58 = 0; c5_i58 < 3; c5_i58++) {
      c5_b_inData[c5_i58 + c5_i56] = (*(real_T (*)[9])c5_inData)[c5_i58 + c5_i56];
    }

    c5_i56 += 3;
  }

  c5_i59 = 0;
  for (c5_i60 = 0; c5_i60 < 3; c5_i60++) {
    for (c5_i61 = 0; c5_i61 < 3; c5_i61++) {
      c5_u[c5_i61 + c5_i59] = c5_b_inData[c5_i61 + c5_i59];
    }

    c5_i59 += 3;
  }

  c5_y = NULL;
  sf_mex_assign(&c5_y, sf_mex_create("y", c5_u, 0, 0U, 1U, 0U, 2, 3, 3), false);
  sf_mex_assign(&c5_mxArrayOutData, c5_y, false);
  return c5_mxArrayOutData;
}

static void c5_h_emlrt_marshallIn(SFc5_BlueBird_FormationInstanceStruct
  *chartInstance, const mxArray *c5_u, const emlrtMsgIdentifier *c5_parentId,
  real_T c5_y[9])
{
  real_T c5_dv6[9];
  int32_T c5_i62;
  (void)chartInstance;
  sf_mex_import(c5_parentId, sf_mex_dup(c5_u), c5_dv6, 1, 0, 0U, 1, 0U, 2, 3, 3);
  for (c5_i62 = 0; c5_i62 < 9; c5_i62++) {
    c5_y[c5_i62] = c5_dv6[c5_i62];
  }

  sf_mex_destroy(&c5_u);
}

static void c5_f_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c5_mxArrayInData, const char_T *c5_varName, void *c5_outData)
{
  const mxArray *c5_IR_Transformation_Matrix_I;
  const char_T *c5_identifier;
  emlrtMsgIdentifier c5_thisId;
  real_T c5_y[9];
  int32_T c5_i63;
  int32_T c5_i64;
  int32_T c5_i65;
  SFc5_BlueBird_FormationInstanceStruct *chartInstance;
  chartInstance = (SFc5_BlueBird_FormationInstanceStruct *)chartInstanceVoid;
  c5_IR_Transformation_Matrix_I = sf_mex_dup(c5_mxArrayInData);
  c5_identifier = c5_varName;
  c5_thisId.fIdentifier = c5_identifier;
  c5_thisId.fParent = NULL;
  c5_h_emlrt_marshallIn(chartInstance, sf_mex_dup(c5_IR_Transformation_Matrix_I),
                        &c5_thisId, c5_y);
  sf_mex_destroy(&c5_IR_Transformation_Matrix_I);
  c5_i63 = 0;
  for (c5_i64 = 0; c5_i64 < 3; c5_i64++) {
    for (c5_i65 = 0; c5_i65 < 3; c5_i65++) {
      (*(real_T (*)[9])c5_outData)[c5_i65 + c5_i63] = c5_y[c5_i65 + c5_i63];
    }

    c5_i63 += 3;
  }

  sf_mex_destroy(&c5_mxArrayInData);
}

static void c5_i_emlrt_marshallIn(SFc5_BlueBird_FormationInstanceStruct
  *chartInstance, const mxArray *c5_u, const emlrtMsgIdentifier *c5_parentId,
  real_T c5_y[9])
{
  real_T c5_dv7[9];
  int32_T c5_i66;
  (void)chartInstance;
  sf_mex_import(c5_parentId, sf_mex_dup(c5_u), c5_dv7, 1, 0, 0U, 1, 0U, 1, 9);
  for (c5_i66 = 0; c5_i66 < 9; c5_i66++) {
    c5_y[c5_i66] = c5_dv7[c5_i66];
  }

  sf_mex_destroy(&c5_u);
}

static void c5_g_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c5_mxArrayInData, const char_T *c5_varName, void *c5_outData)
{
  const mxArray *c5_ir_distances;
  const char_T *c5_identifier;
  emlrtMsgIdentifier c5_thisId;
  real_T c5_y[9];
  int32_T c5_i67;
  SFc5_BlueBird_FormationInstanceStruct *chartInstance;
  chartInstance = (SFc5_BlueBird_FormationInstanceStruct *)chartInstanceVoid;
  c5_ir_distances = sf_mex_dup(c5_mxArrayInData);
  c5_identifier = c5_varName;
  c5_thisId.fIdentifier = c5_identifier;
  c5_thisId.fParent = NULL;
  c5_i_emlrt_marshallIn(chartInstance, sf_mex_dup(c5_ir_distances), &c5_thisId,
                        c5_y);
  sf_mex_destroy(&c5_ir_distances);
  for (c5_i67 = 0; c5_i67 < 9; c5_i67++) {
    (*(real_T (*)[9])c5_outData)[c5_i67] = c5_y[c5_i67];
  }

  sf_mex_destroy(&c5_mxArrayInData);
}

static const mxArray *c5_h_sf_marshallOut(void *chartInstanceVoid, real_T
  c5_inData_data[], int32_T *c5_inData_sizes)
{
  const mxArray *c5_mxArrayOutData = NULL;
  int32_T c5_b_inData_sizes;
  int32_T c5_loop_ub;
  int32_T c5_i68;
  real_T c5_b_inData_data[9];
  int32_T c5_u_sizes;
  int32_T c5_b_loop_ub;
  int32_T c5_i69;
  real_T c5_u_data[9];
  const mxArray *c5_y = NULL;
  SFc5_BlueBird_FormationInstanceStruct *chartInstance;
  chartInstance = (SFc5_BlueBird_FormationInstanceStruct *)chartInstanceVoid;
  c5_mxArrayOutData = NULL;
  c5_b_inData_sizes = *c5_inData_sizes;
  c5_loop_ub = *c5_inData_sizes - 1;
  for (c5_i68 = 0; c5_i68 <= c5_loop_ub; c5_i68++) {
    c5_b_inData_data[c5_i68] = c5_inData_data[c5_i68];
  }

  c5_u_sizes = c5_b_inData_sizes;
  c5_b_loop_ub = c5_b_inData_sizes - 1;
  for (c5_i69 = 0; c5_i69 <= c5_b_loop_ub; c5_i69++) {
    c5_u_data[c5_i69] = c5_b_inData_data[c5_i69];
  }

  c5_y = NULL;
  sf_mex_assign(&c5_y, sf_mex_create("y", c5_u_data, 0, 0U, 1U, 0U, 1,
    c5_u_sizes), false);
  sf_mex_assign(&c5_mxArrayOutData, c5_y, false);
  return c5_mxArrayOutData;
}

static void c5_j_emlrt_marshallIn(SFc5_BlueBird_FormationInstanceStruct
  *chartInstance, const mxArray *c5_u, const emlrtMsgIdentifier *c5_parentId,
  real_T c5_y_data[], int32_T *c5_y_sizes)
{
  static uint32_T c5_uv0[1] = { 9U };

  uint32_T c5_uv1[1];
  static boolean_T c5_bv0[1] = { true };

  boolean_T c5_bv1[1];
  int32_T c5_tmp_sizes;
  real_T c5_tmp_data[9];
  int32_T c5_loop_ub;
  int32_T c5_i70;
  (void)chartInstance;
  c5_uv1[0] = c5_uv0[0];
  c5_bv1[0] = c5_bv0[0];
  sf_mex_import_vs(c5_parentId, sf_mex_dup(c5_u), c5_tmp_data, 1, 0, 0U, 1, 0U,
                   1, c5_bv1, c5_uv1, &c5_tmp_sizes);
  *c5_y_sizes = c5_tmp_sizes;
  c5_loop_ub = c5_tmp_sizes - 1;
  for (c5_i70 = 0; c5_i70 <= c5_loop_ub; c5_i70++) {
    c5_y_data[c5_i70] = c5_tmp_data[c5_i70];
  }

  sf_mex_destroy(&c5_u);
}

static void c5_h_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c5_mxArrayInData, const char_T *c5_varName, real_T c5_outData_data[], int32_T
  *c5_outData_sizes)
{
  const mxArray *c5_ObstacleDetected;
  const char_T *c5_identifier;
  emlrtMsgIdentifier c5_thisId;
  int32_T c5_y_sizes;
  real_T c5_y_data[9];
  int32_T c5_loop_ub;
  int32_T c5_i71;
  SFc5_BlueBird_FormationInstanceStruct *chartInstance;
  chartInstance = (SFc5_BlueBird_FormationInstanceStruct *)chartInstanceVoid;
  c5_ObstacleDetected = sf_mex_dup(c5_mxArrayInData);
  c5_identifier = c5_varName;
  c5_thisId.fIdentifier = c5_identifier;
  c5_thisId.fParent = NULL;
  c5_j_emlrt_marshallIn(chartInstance, sf_mex_dup(c5_ObstacleDetected),
                        &c5_thisId, c5_y_data, &c5_y_sizes);
  sf_mex_destroy(&c5_ObstacleDetected);
  *c5_outData_sizes = c5_y_sizes;
  c5_loop_ub = c5_y_sizes - 1;
  for (c5_i71 = 0; c5_i71 <= c5_loop_ub; c5_i71++) {
    c5_outData_data[c5_i71] = c5_y_data[c5_i71];
  }

  sf_mex_destroy(&c5_mxArrayInData);
}

static const mxArray *c5_i_sf_marshallOut(void *chartInstanceVoid, void
  *c5_inData)
{
  const mxArray *c5_mxArrayOutData = NULL;
  int32_T c5_i72;
  int32_T c5_i73;
  int32_T c5_i74;
  real_T c5_b_inData[27];
  int32_T c5_i75;
  int32_T c5_i76;
  int32_T c5_i77;
  real_T c5_u[27];
  const mxArray *c5_y = NULL;
  SFc5_BlueBird_FormationInstanceStruct *chartInstance;
  chartInstance = (SFc5_BlueBird_FormationInstanceStruct *)chartInstanceVoid;
  c5_mxArrayOutData = NULL;
  c5_i72 = 0;
  for (c5_i73 = 0; c5_i73 < 3; c5_i73++) {
    for (c5_i74 = 0; c5_i74 < 9; c5_i74++) {
      c5_b_inData[c5_i74 + c5_i72] = (*(real_T (*)[27])c5_inData)[c5_i74 +
        c5_i72];
    }

    c5_i72 += 9;
  }

  c5_i75 = 0;
  for (c5_i76 = 0; c5_i76 < 3; c5_i76++) {
    for (c5_i77 = 0; c5_i77 < 9; c5_i77++) {
      c5_u[c5_i77 + c5_i75] = c5_b_inData[c5_i77 + c5_i75];
    }

    c5_i75 += 9;
  }

  c5_y = NULL;
  sf_mex_assign(&c5_y, sf_mex_create("y", c5_u, 0, 0U, 1U, 0U, 2, 9, 3), false);
  sf_mex_assign(&c5_mxArrayOutData, c5_y, false);
  return c5_mxArrayOutData;
}

static void c5_k_emlrt_marshallIn(SFc5_BlueBird_FormationInstanceStruct
  *chartInstance, const mxArray *c5_u, const emlrtMsgIdentifier *c5_parentId,
  real_T c5_y[27])
{
  real_T c5_dv8[27];
  int32_T c5_i78;
  (void)chartInstance;
  sf_mex_import(c5_parentId, sf_mex_dup(c5_u), c5_dv8, 1, 0, 0U, 1, 0U, 2, 9, 3);
  for (c5_i78 = 0; c5_i78 < 27; c5_i78++) {
    c5_y[c5_i78] = c5_dv8[c5_i78];
  }

  sf_mex_destroy(&c5_u);
}

static void c5_i_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c5_mxArrayInData, const char_T *c5_varName, void *c5_outData)
{
  const mxArray *c5_IR_Position;
  const char_T *c5_identifier;
  emlrtMsgIdentifier c5_thisId;
  real_T c5_y[27];
  int32_T c5_i79;
  int32_T c5_i80;
  int32_T c5_i81;
  SFc5_BlueBird_FormationInstanceStruct *chartInstance;
  chartInstance = (SFc5_BlueBird_FormationInstanceStruct *)chartInstanceVoid;
  c5_IR_Position = sf_mex_dup(c5_mxArrayInData);
  c5_identifier = c5_varName;
  c5_thisId.fIdentifier = c5_identifier;
  c5_thisId.fParent = NULL;
  c5_k_emlrt_marshallIn(chartInstance, sf_mex_dup(c5_IR_Position), &c5_thisId,
                        c5_y);
  sf_mex_destroy(&c5_IR_Position);
  c5_i79 = 0;
  for (c5_i80 = 0; c5_i80 < 3; c5_i80++) {
    for (c5_i81 = 0; c5_i81 < 9; c5_i81++) {
      (*(real_T (*)[27])c5_outData)[c5_i81 + c5_i79] = c5_y[c5_i81 + c5_i79];
    }

    c5_i79 += 9;
  }

  sf_mex_destroy(&c5_mxArrayInData);
}

static const mxArray *c5_j_sf_marshallOut(void *chartInstanceVoid, void
  *c5_inData)
{
  const mxArray *c5_mxArrayOutData = NULL;
  int32_T c5_i82;
  char_T c5_b_inData[15];
  int32_T c5_i83;
  char_T c5_u[15];
  const mxArray *c5_y = NULL;
  SFc5_BlueBird_FormationInstanceStruct *chartInstance;
  chartInstance = (SFc5_BlueBird_FormationInstanceStruct *)chartInstanceVoid;
  c5_mxArrayOutData = NULL;
  for (c5_i82 = 0; c5_i82 < 15; c5_i82++) {
    c5_b_inData[c5_i82] = (*(char_T (*)[15])c5_inData)[c5_i82];
  }

  for (c5_i83 = 0; c5_i83 < 15; c5_i83++) {
    c5_u[c5_i83] = c5_b_inData[c5_i83];
  }

  c5_y = NULL;
  sf_mex_assign(&c5_y, sf_mex_create("y", c5_u, 10, 0U, 1U, 0U, 2, 1, 15), false);
  sf_mex_assign(&c5_mxArrayOutData, c5_y, false);
  return c5_mxArrayOutData;
}

static void c5_l_emlrt_marshallIn(SFc5_BlueBird_FormationInstanceStruct
  *chartInstance, const mxArray *c5_u, const emlrtMsgIdentifier *c5_parentId,
  char_T c5_y[15])
{
  char_T c5_cv0[15];
  int32_T c5_i84;
  (void)chartInstance;
  sf_mex_import(c5_parentId, sf_mex_dup(c5_u), c5_cv0, 1, 10, 0U, 1, 0U, 2, 1,
                15);
  for (c5_i84 = 0; c5_i84 < 15; c5_i84++) {
    c5_y[c5_i84] = c5_cv0[c5_i84];
  }

  sf_mex_destroy(&c5_u);
}

static void c5_j_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c5_mxArrayInData, const char_T *c5_varName, void *c5_outData)
{
  const mxArray *c5_type;
  const char_T *c5_identifier;
  emlrtMsgIdentifier c5_thisId;
  char_T c5_y[15];
  int32_T c5_i85;
  SFc5_BlueBird_FormationInstanceStruct *chartInstance;
  chartInstance = (SFc5_BlueBird_FormationInstanceStruct *)chartInstanceVoid;
  c5_type = sf_mex_dup(c5_mxArrayInData);
  c5_identifier = c5_varName;
  c5_thisId.fIdentifier = c5_identifier;
  c5_thisId.fParent = NULL;
  c5_l_emlrt_marshallIn(chartInstance, sf_mex_dup(c5_type), &c5_thisId, c5_y);
  sf_mex_destroy(&c5_type);
  for (c5_i85 = 0; c5_i85 < 15; c5_i85++) {
    (*(char_T (*)[15])c5_outData)[c5_i85] = c5_y[c5_i85];
  }

  sf_mex_destroy(&c5_mxArrayInData);
}

static const mxArray *c5_k_sf_marshallOut(void *chartInstanceVoid, void
  *c5_inData)
{
  const mxArray *c5_mxArrayOutData = NULL;
  int32_T c5_i86;
  int32_T c5_i87;
  int32_T c5_i88;
  real_T c5_b_inData[18];
  int32_T c5_i89;
  int32_T c5_i90;
  int32_T c5_i91;
  real_T c5_u[18];
  const mxArray *c5_y = NULL;
  SFc5_BlueBird_FormationInstanceStruct *chartInstance;
  chartInstance = (SFc5_BlueBird_FormationInstanceStruct *)chartInstanceVoid;
  c5_mxArrayOutData = NULL;
  c5_i86 = 0;
  for (c5_i87 = 0; c5_i87 < 9; c5_i87++) {
    for (c5_i88 = 0; c5_i88 < 2; c5_i88++) {
      c5_b_inData[c5_i88 + c5_i86] = (*(real_T (*)[18])c5_inData)[c5_i88 +
        c5_i86];
    }

    c5_i86 += 2;
  }

  c5_i89 = 0;
  for (c5_i90 = 0; c5_i90 < 9; c5_i90++) {
    for (c5_i91 = 0; c5_i91 < 2; c5_i91++) {
      c5_u[c5_i91 + c5_i89] = c5_b_inData[c5_i91 + c5_i89];
    }

    c5_i89 += 2;
  }

  c5_y = NULL;
  sf_mex_assign(&c5_y, sf_mex_create("y", c5_u, 0, 0U, 1U, 0U, 2, 2, 9), false);
  sf_mex_assign(&c5_mxArrayOutData, c5_y, false);
  return c5_mxArrayOutData;
}

static void c5_m_emlrt_marshallIn(SFc5_BlueBird_FormationInstanceStruct
  *chartInstance, const mxArray *c5_u, const emlrtMsgIdentifier *c5_parentId,
  real_T c5_y[18])
{
  real_T c5_dv9[18];
  int32_T c5_i92;
  (void)chartInstance;
  sf_mex_import(c5_parentId, sf_mex_dup(c5_u), c5_dv9, 1, 0, 0U, 1, 0U, 2, 2, 9);
  for (c5_i92 = 0; c5_i92 < 18; c5_i92++) {
    c5_y[c5_i92] = c5_dv9[c5_i92];
  }

  sf_mex_destroy(&c5_u);
}

static void c5_k_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c5_mxArrayInData, const char_T *c5_varName, void *c5_outData)
{
  const mxArray *c5_u_i;
  const char_T *c5_identifier;
  emlrtMsgIdentifier c5_thisId;
  real_T c5_y[18];
  int32_T c5_i93;
  int32_T c5_i94;
  int32_T c5_i95;
  SFc5_BlueBird_FormationInstanceStruct *chartInstance;
  chartInstance = (SFc5_BlueBird_FormationInstanceStruct *)chartInstanceVoid;
  c5_u_i = sf_mex_dup(c5_mxArrayInData);
  c5_identifier = c5_varName;
  c5_thisId.fIdentifier = c5_identifier;
  c5_thisId.fParent = NULL;
  c5_m_emlrt_marshallIn(chartInstance, sf_mex_dup(c5_u_i), &c5_thisId, c5_y);
  sf_mex_destroy(&c5_u_i);
  c5_i93 = 0;
  for (c5_i94 = 0; c5_i94 < 9; c5_i94++) {
    for (c5_i95 = 0; c5_i95 < 2; c5_i95++) {
      (*(real_T (*)[18])c5_outData)[c5_i95 + c5_i93] = c5_y[c5_i95 + c5_i93];
    }

    c5_i93 += 2;
  }

  sf_mex_destroy(&c5_mxArrayInData);
}

static const mxArray *c5_l_sf_marshallOut(void *chartInstanceVoid, void
  *c5_inData)
{
  const mxArray *c5_mxArrayOutData = NULL;
  int32_T c5_i96;
  real_T c5_b_inData[9];
  int32_T c5_i97;
  real_T c5_u[9];
  const mxArray *c5_y = NULL;
  SFc5_BlueBird_FormationInstanceStruct *chartInstance;
  chartInstance = (SFc5_BlueBird_FormationInstanceStruct *)chartInstanceVoid;
  c5_mxArrayOutData = NULL;
  for (c5_i96 = 0; c5_i96 < 9; c5_i96++) {
    c5_b_inData[c5_i96] = (*(real_T (*)[9])c5_inData)[c5_i96];
  }

  for (c5_i97 = 0; c5_i97 < 9; c5_i97++) {
    c5_u[c5_i97] = c5_b_inData[c5_i97];
  }

  c5_y = NULL;
  sf_mex_assign(&c5_y, sf_mex_create("y", c5_u, 0, 0U, 1U, 0U, 2, 1, 9), false);
  sf_mex_assign(&c5_mxArrayOutData, c5_y, false);
  return c5_mxArrayOutData;
}

const mxArray *sf_c5_BlueBird_Formation_get_eml_resolved_functions_info(void)
{
  const mxArray *c5_nameCaptureInfo = NULL;
  c5_nameCaptureInfo = NULL;
  sf_mex_assign(&c5_nameCaptureInfo, sf_mex_createstruct("structure", 2, 102, 1),
                false);
  c5_info_helper(&c5_nameCaptureInfo);
  c5_b_info_helper(&c5_nameCaptureInfo);
  sf_mex_emlrtNameCapturePostProcessR2012a(&c5_nameCaptureInfo);
  return c5_nameCaptureInfo;
}

static void c5_info_helper(const mxArray **c5_info)
{
  const mxArray *c5_rhs0 = NULL;
  const mxArray *c5_lhs0 = NULL;
  const mxArray *c5_rhs1 = NULL;
  const mxArray *c5_lhs1 = NULL;
  const mxArray *c5_rhs2 = NULL;
  const mxArray *c5_lhs2 = NULL;
  const mxArray *c5_rhs3 = NULL;
  const mxArray *c5_lhs3 = NULL;
  const mxArray *c5_rhs4 = NULL;
  const mxArray *c5_lhs4 = NULL;
  const mxArray *c5_rhs5 = NULL;
  const mxArray *c5_lhs5 = NULL;
  const mxArray *c5_rhs6 = NULL;
  const mxArray *c5_lhs6 = NULL;
  const mxArray *c5_rhs7 = NULL;
  const mxArray *c5_lhs7 = NULL;
  const mxArray *c5_rhs8 = NULL;
  const mxArray *c5_lhs8 = NULL;
  const mxArray *c5_rhs9 = NULL;
  const mxArray *c5_lhs9 = NULL;
  const mxArray *c5_rhs10 = NULL;
  const mxArray *c5_lhs10 = NULL;
  const mxArray *c5_rhs11 = NULL;
  const mxArray *c5_lhs11 = NULL;
  const mxArray *c5_rhs12 = NULL;
  const mxArray *c5_lhs12 = NULL;
  const mxArray *c5_rhs13 = NULL;
  const mxArray *c5_lhs13 = NULL;
  const mxArray *c5_rhs14 = NULL;
  const mxArray *c5_lhs14 = NULL;
  const mxArray *c5_rhs15 = NULL;
  const mxArray *c5_lhs15 = NULL;
  const mxArray *c5_rhs16 = NULL;
  const mxArray *c5_lhs16 = NULL;
  const mxArray *c5_rhs17 = NULL;
  const mxArray *c5_lhs17 = NULL;
  const mxArray *c5_rhs18 = NULL;
  const mxArray *c5_lhs18 = NULL;
  const mxArray *c5_rhs19 = NULL;
  const mxArray *c5_lhs19 = NULL;
  const mxArray *c5_rhs20 = NULL;
  const mxArray *c5_lhs20 = NULL;
  const mxArray *c5_rhs21 = NULL;
  const mxArray *c5_lhs21 = NULL;
  const mxArray *c5_rhs22 = NULL;
  const mxArray *c5_lhs22 = NULL;
  const mxArray *c5_rhs23 = NULL;
  const mxArray *c5_lhs23 = NULL;
  const mxArray *c5_rhs24 = NULL;
  const mxArray *c5_lhs24 = NULL;
  const mxArray *c5_rhs25 = NULL;
  const mxArray *c5_lhs25 = NULL;
  const mxArray *c5_rhs26 = NULL;
  const mxArray *c5_lhs26 = NULL;
  const mxArray *c5_rhs27 = NULL;
  const mxArray *c5_lhs27 = NULL;
  const mxArray *c5_rhs28 = NULL;
  const mxArray *c5_lhs28 = NULL;
  const mxArray *c5_rhs29 = NULL;
  const mxArray *c5_lhs29 = NULL;
  const mxArray *c5_rhs30 = NULL;
  const mxArray *c5_lhs30 = NULL;
  const mxArray *c5_rhs31 = NULL;
  const mxArray *c5_lhs31 = NULL;
  const mxArray *c5_rhs32 = NULL;
  const mxArray *c5_lhs32 = NULL;
  const mxArray *c5_rhs33 = NULL;
  const mxArray *c5_lhs33 = NULL;
  const mxArray *c5_rhs34 = NULL;
  const mxArray *c5_lhs34 = NULL;
  const mxArray *c5_rhs35 = NULL;
  const mxArray *c5_lhs35 = NULL;
  const mxArray *c5_rhs36 = NULL;
  const mxArray *c5_lhs36 = NULL;
  const mxArray *c5_rhs37 = NULL;
  const mxArray *c5_lhs37 = NULL;
  const mxArray *c5_rhs38 = NULL;
  const mxArray *c5_lhs38 = NULL;
  const mxArray *c5_rhs39 = NULL;
  const mxArray *c5_lhs39 = NULL;
  const mxArray *c5_rhs40 = NULL;
  const mxArray *c5_lhs40 = NULL;
  const mxArray *c5_rhs41 = NULL;
  const mxArray *c5_lhs41 = NULL;
  const mxArray *c5_rhs42 = NULL;
  const mxArray *c5_lhs42 = NULL;
  const mxArray *c5_rhs43 = NULL;
  const mxArray *c5_lhs43 = NULL;
  const mxArray *c5_rhs44 = NULL;
  const mxArray *c5_lhs44 = NULL;
  const mxArray *c5_rhs45 = NULL;
  const mxArray *c5_lhs45 = NULL;
  const mxArray *c5_rhs46 = NULL;
  const mxArray *c5_lhs46 = NULL;
  const mxArray *c5_rhs47 = NULL;
  const mxArray *c5_lhs47 = NULL;
  const mxArray *c5_rhs48 = NULL;
  const mxArray *c5_lhs48 = NULL;
  const mxArray *c5_rhs49 = NULL;
  const mxArray *c5_lhs49 = NULL;
  const mxArray *c5_rhs50 = NULL;
  const mxArray *c5_lhs50 = NULL;
  const mxArray *c5_rhs51 = NULL;
  const mxArray *c5_lhs51 = NULL;
  const mxArray *c5_rhs52 = NULL;
  const mxArray *c5_lhs52 = NULL;
  const mxArray *c5_rhs53 = NULL;
  const mxArray *c5_lhs53 = NULL;
  const mxArray *c5_rhs54 = NULL;
  const mxArray *c5_lhs54 = NULL;
  const mxArray *c5_rhs55 = NULL;
  const mxArray *c5_lhs55 = NULL;
  const mxArray *c5_rhs56 = NULL;
  const mxArray *c5_lhs56 = NULL;
  const mxArray *c5_rhs57 = NULL;
  const mxArray *c5_lhs57 = NULL;
  const mxArray *c5_rhs58 = NULL;
  const mxArray *c5_lhs58 = NULL;
  const mxArray *c5_rhs59 = NULL;
  const mxArray *c5_lhs59 = NULL;
  const mxArray *c5_rhs60 = NULL;
  const mxArray *c5_lhs60 = NULL;
  const mxArray *c5_rhs61 = NULL;
  const mxArray *c5_lhs61 = NULL;
  const mxArray *c5_rhs62 = NULL;
  const mxArray *c5_lhs62 = NULL;
  const mxArray *c5_rhs63 = NULL;
  const mxArray *c5_lhs63 = NULL;
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(""), "context", "context", 0);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("find"), "name", "name", 0);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("logical"), "dominantType",
                  "dominantType", 0);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/find.m"), "resolved",
                  "resolved", 0);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1303153406U), "fileTimeLo",
                  "fileTimeLo", 0);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 0);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 0);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 0);
  sf_mex_assign(&c5_rhs0, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs0, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs0), "rhs", "rhs", 0);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs0), "lhs", "lhs", 0);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/find.m!eml_find"),
                  "context", "context", 1);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eml_index_class"), "name",
                  "name", 1);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 1);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m"),
                  "resolved", "resolved", 1);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1323174178U), "fileTimeLo",
                  "fileTimeLo", 1);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 1);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 1);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 1);
  sf_mex_assign(&c5_rhs1, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs1, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs1), "rhs", "rhs", 1);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs1), "lhs", "lhs", 1);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/find.m!eml_find"),
                  "context", "context", 2);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eml_scalar_eg"), "name",
                  "name", 2);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("logical"), "dominantType",
                  "dominantType", 2);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "resolved",
                  "resolved", 2);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1375987888U), "fileTimeLo",
                  "fileTimeLo", 2);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 2);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 2);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 2);
  sf_mex_assign(&c5_rhs2, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs2, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs2), "rhs", "rhs", 2);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs2), "lhs", "lhs", 2);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "context",
                  "context", 3);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("coder.internal.scalarEg"),
                  "name", "name", 3);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("logical"), "dominantType",
                  "dominantType", 3);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/scalarEg.p"),
                  "resolved", "resolved", 3);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1389311520U), "fileTimeLo",
                  "fileTimeLo", 3);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 3);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 3);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 3);
  sf_mex_assign(&c5_rhs3, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs3, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs3), "rhs", "rhs", 3);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs3), "lhs", "lhs", 3);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/find.m!eml_find"),
                  "context", "context", 4);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "eml_int_forloop_overflow_check"), "name", "name", 4);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 4);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m"),
                  "resolved", "resolved", 4);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1375987888U), "fileTimeLo",
                  "fileTimeLo", 4);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 4);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 4);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 4);
  sf_mex_assign(&c5_rhs4, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs4, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs4), "rhs", "rhs", 4);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs4), "lhs", "lhs", 4);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m!eml_int_forloop_overflow_check_helper"),
                  "context", "context", 5);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("intmax"), "name", "name", 5);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 5);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m"), "resolved",
                  "resolved", 5);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1362265482U), "fileTimeLo",
                  "fileTimeLo", 5);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 5);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 5);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 5);
  sf_mex_assign(&c5_rhs5, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs5, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs5), "rhs", "rhs", 5);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs5), "lhs", "lhs", 5);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m"), "context",
                  "context", 6);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eml_switch_helper"), "name",
                  "name", 6);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 6);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_switch_helper.m"),
                  "resolved", "resolved", 6);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1381857500U), "fileTimeLo",
                  "fileTimeLo", 6);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 6);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 6);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 6);
  sf_mex_assign(&c5_rhs6, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs6, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs6), "rhs", "rhs", 6);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs6), "lhs", "lhs", 6);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/find.m!eml_find"),
                  "context", "context", 7);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eml_index_plus"), "name",
                  "name", 7);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 7);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m"),
                  "resolved", "resolved", 7);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1372589616U), "fileTimeLo",
                  "fileTimeLo", 7);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 7);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 7);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 7);
  sf_mex_assign(&c5_rhs7, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs7, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs7), "rhs", "rhs", 7);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs7), "lhs", "lhs", 7);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m"), "context",
                  "context", 8);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("coder.internal.indexPlus"),
                  "name", "name", 8);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 8);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexPlus.m"),
                  "resolved", "resolved", 8);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1372590360U), "fileTimeLo",
                  "fileTimeLo", 8);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 8);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 8);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 8);
  sf_mex_assign(&c5_rhs8, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs8, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs8), "rhs", "rhs", 8);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs8), "lhs", "lhs", 8);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(""), "context", "context", 9);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("length"), "name", "name", 9);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 9);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/length.m"), "resolved",
                  "resolved", 9);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1303153406U), "fileTimeLo",
                  "fileTimeLo", 9);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 9);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 9);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 9);
  sf_mex_assign(&c5_rhs9, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs9, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs9), "rhs", "rhs", 9);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs9), "lhs", "lhs", 9);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(""), "context", "context", 10);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("mrdivide"), "name", "name", 10);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 10);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p"), "resolved",
                  "resolved", 10);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1388463696U), "fileTimeLo",
                  "fileTimeLo", 10);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 10);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1370017086U), "mFileTimeLo",
                  "mFileTimeLo", 10);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 10);
  sf_mex_assign(&c5_rhs10, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs10, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs10), "rhs", "rhs",
                  10);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs10), "lhs", "lhs",
                  10);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p"), "context",
                  "context", 11);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("coder.internal.assert"),
                  "name", "name", 11);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 11);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/assert.m"),
                  "resolved", "resolved", 11);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1363718156U), "fileTimeLo",
                  "fileTimeLo", 11);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 11);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 11);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 11);
  sf_mex_assign(&c5_rhs11, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs11, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs11), "rhs", "rhs",
                  11);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs11), "lhs", "lhs",
                  11);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p"), "context",
                  "context", 12);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("rdivide"), "name", "name", 12);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 12);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m"), "resolved",
                  "resolved", 12);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1363717480U), "fileTimeLo",
                  "fileTimeLo", 12);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 12);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 12);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 12);
  sf_mex_assign(&c5_rhs12, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs12, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs12), "rhs", "rhs",
                  12);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs12), "lhs", "lhs",
                  12);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m"), "context",
                  "context", 13);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 13);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 13);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 13);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1363718156U), "fileTimeLo",
                  "fileTimeLo", 13);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 13);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 13);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 13);
  sf_mex_assign(&c5_rhs13, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs13, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs13), "rhs", "rhs",
                  13);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs13), "lhs", "lhs",
                  13);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m"), "context",
                  "context", 14);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eml_scalexp_compatible"),
                  "name", "name", 14);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 14);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_compatible.m"),
                  "resolved", "resolved", 14);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1286825996U), "fileTimeLo",
                  "fileTimeLo", 14);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 14);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 14);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 14);
  sf_mex_assign(&c5_rhs14, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs14, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs14), "rhs", "rhs",
                  14);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs14), "lhs", "lhs",
                  14);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m"), "context",
                  "context", 15);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eml_div"), "name", "name", 15);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 15);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m"), "resolved",
                  "resolved", 15);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1375987888U), "fileTimeLo",
                  "fileTimeLo", 15);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 15);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 15);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 15);
  sf_mex_assign(&c5_rhs15, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs15, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs15), "rhs", "rhs",
                  15);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs15), "lhs", "lhs",
                  15);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m"), "context",
                  "context", 16);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("coder.internal.div"), "name",
                  "name", 16);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 16);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/div.p"), "resolved",
                  "resolved", 16);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1389311520U), "fileTimeLo",
                  "fileTimeLo", 16);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 16);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 16);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 16);
  sf_mex_assign(&c5_rhs16, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs16, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs16), "rhs", "rhs",
                  16);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs16), "lhs", "lhs",
                  16);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(""), "context", "context", 17);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("reallog"), "name", "name", 17);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 17);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/reallog.m"), "resolved",
                  "resolved", 17);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1343837584U), "fileTimeLo",
                  "fileTimeLo", 17);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 17);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 17);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 17);
  sf_mex_assign(&c5_rhs17, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs17, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs17), "rhs", "rhs",
                  17);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs17), "lhs", "lhs",
                  17);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/reallog.m"), "context",
                  "context", 18);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eml_error"), "name", "name",
                  18);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 18);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_error.m"), "resolved",
                  "resolved", 18);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1343837558U), "fileTimeLo",
                  "fileTimeLo", 18);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 18);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 18);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 18);
  sf_mex_assign(&c5_rhs18, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs18, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs18), "rhs", "rhs",
                  18);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs18), "lhs", "lhs",
                  18);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/reallog.m"), "context",
                  "context", 19);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eml_scalar_reallog"), "name",
                  "name", 19);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 19);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_reallog.m"),
                  "resolved", "resolved", 19);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1286825932U), "fileTimeLo",
                  "fileTimeLo", 19);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 19);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 19);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 19);
  sf_mex_assign(&c5_rhs19, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs19, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs19), "rhs", "rhs",
                  19);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs19), "lhs", "lhs",
                  19);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(""), "context", "context", 20);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eml_mtimes_helper"), "name",
                  "name", 20);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 20);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/eml_mtimes_helper.m"),
                  "resolved", "resolved", 20);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1383880894U), "fileTimeLo",
                  "fileTimeLo", 20);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 20);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 20);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 20);
  sf_mex_assign(&c5_rhs20, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs20, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs20), "rhs", "rhs",
                  20);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs20), "lhs", "lhs",
                  20);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/eml_mtimes_helper.m!common_checks"),
                  "context", "context", 21);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 21);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 21);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 21);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1363718156U), "fileTimeLo",
                  "fileTimeLo", 21);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 21);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 21);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 21);
  sf_mex_assign(&c5_rhs21, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs21, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs21), "rhs", "rhs",
                  21);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs21), "lhs", "lhs",
                  21);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[C]F:/BlueBird/Study/Robot/Location/20140925/+simiam/+ui/Pose2D.m"),
                  "context", "context", 22);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "coder.internal.matlabCodegenHandle"), "name", "name", 22);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("unknown"), "dominantType",
                  "dominantType", 22);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXC]$matlabroot$/toolbox/coder/coder/+coder/+internal/matlabCodegenHandle.p"),
                  "resolved", "resolved", 22);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1389311522U), "fileTimeLo",
                  "fileTimeLo", 22);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 22);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 22);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 22);
  sf_mex_assign(&c5_rhs22, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs22, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs22), "rhs", "rhs",
                  22);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs22), "lhs", "lhs",
                  22);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(""), "context", "context", 23);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("simiam.ui.Pose2D"), "name",
                  "name", 23);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 23);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[C]F:/BlueBird/Study/Robot/Location/20140925/+simiam/+ui/Pose2D.m"),
                  "resolved", "resolved", 23);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1389257563U), "fileTimeLo",
                  "fileTimeLo", 23);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 23);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 23);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 23);
  sf_mex_assign(&c5_rhs23, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs23, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs23), "rhs", "rhs",
                  23);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs23), "lhs", "lhs",
                  23);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[C]F:/BlueBird/Study/Robot/Location/20140925/+simiam/+ui/Pose2D.m"),
                  "context", "context", 24);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("mrdivide"), "name", "name", 24);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 24);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p"), "resolved",
                  "resolved", 24);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1388463696U), "fileTimeLo",
                  "fileTimeLo", 24);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 24);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1370017086U), "mFileTimeLo",
                  "mFileTimeLo", 24);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 24);
  sf_mex_assign(&c5_rhs24, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs24, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs24), "rhs", "rhs",
                  24);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs24), "lhs", "lhs",
                  24);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[C]F:/BlueBird/Study/Robot/Location/20140925/+simiam/+controller/Controller.m"),
                  "context", "context", 25);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "coder.internal.matlabCodegenHandle"), "name", "name", 25);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("unknown"), "dominantType",
                  "dominantType", 25);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXC]$matlabroot$/toolbox/coder/coder/+coder/+internal/matlabCodegenHandle.p"),
                  "resolved", "resolved", 25);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1389311522U), "fileTimeLo",
                  "fileTimeLo", 25);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 25);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 25);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 25);
  sf_mex_assign(&c5_rhs25, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs25, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs25), "rhs", "rhs",
                  25);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs25), "lhs", "lhs",
                  25);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[C]F:/BlueBird/Study/Robot/Location/20140925/+simiam/+controller/AvoidObstacles.m"),
                  "context", "context", 26);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("simiam.controller.Controller"),
                  "name", "name", 26);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("unknown"), "dominantType",
                  "dominantType", 26);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[C]F:/BlueBird/Study/Robot/Location/20140925/+simiam/+controller/Controller.m"),
                  "resolved", "resolved", 26);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1389274949U), "fileTimeLo",
                  "fileTimeLo", 26);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 26);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 26);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 26);
  sf_mex_assign(&c5_rhs26, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs26, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs26), "rhs", "rhs",
                  26);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs26), "lhs", "lhs",
                  26);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(""), "context", "context", 27);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "simiam.controller.AvoidObstacles"), "name", "name", 27);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 27);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[C]F:/BlueBird/Study/Robot/Location/20140925/+simiam/+controller/AvoidObstacles.m"),
                  "resolved", "resolved", 27);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1389515879U), "fileTimeLo",
                  "fileTimeLo", 27);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 27);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 27);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 27);
  sf_mex_assign(&c5_rhs27, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs27, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs27), "rhs", "rhs",
                  27);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs27), "lhs", "lhs",
                  27);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[C]F:/BlueBird/Study/Robot/Location/20140925/+simiam/+controller/AvoidObstacles.m"),
                  "context", "context", 28);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("cos"), "name", "name", 28);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 28);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/cos.m"), "resolved",
                  "resolved", 28);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1343837572U), "fileTimeLo",
                  "fileTimeLo", 28);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 28);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 28);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 28);
  sf_mex_assign(&c5_rhs28, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs28, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs28), "rhs", "rhs",
                  28);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs28), "lhs", "lhs",
                  28);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/cos.m"), "context",
                  "context", 29);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eml_scalar_cos"), "name",
                  "name", 29);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 29);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_cos.m"),
                  "resolved", "resolved", 29);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1286825922U), "fileTimeLo",
                  "fileTimeLo", 29);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 29);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 29);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 29);
  sf_mex_assign(&c5_rhs29, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs29, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs29), "rhs", "rhs",
                  29);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs29), "lhs", "lhs",
                  29);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[C]F:/BlueBird/Study/Robot/Location/20140925/+simiam/+controller/AvoidObstacles.m"),
                  "context", "context", 30);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("sin"), "name", "name", 30);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 30);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sin.m"), "resolved",
                  "resolved", 30);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1343837586U), "fileTimeLo",
                  "fileTimeLo", 30);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 30);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 30);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 30);
  sf_mex_assign(&c5_rhs30, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs30, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs30), "rhs", "rhs",
                  30);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs30), "lhs", "lhs",
                  30);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sin.m"), "context",
                  "context", 31);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eml_scalar_sin"), "name",
                  "name", 31);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 31);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_sin.m"),
                  "resolved", "resolved", 31);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1286825936U), "fileTimeLo",
                  "fileTimeLo", 31);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 31);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 31);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 31);
  sf_mex_assign(&c5_rhs31, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs31, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs31), "rhs", "rhs",
                  31);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs31), "lhs", "lhs",
                  31);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/eml_mtimes_helper.m"),
                  "context", "context", 32);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eml_index_class"), "name",
                  "name", 32);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 32);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m"),
                  "resolved", "resolved", 32);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1323174178U), "fileTimeLo",
                  "fileTimeLo", 32);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 32);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 32);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 32);
  sf_mex_assign(&c5_rhs32, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs32, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs32), "rhs", "rhs",
                  32);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs32), "lhs", "lhs",
                  32);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/eml_mtimes_helper.m"),
                  "context", "context", 33);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eml_scalar_eg"), "name",
                  "name", 33);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 33);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "resolved",
                  "resolved", 33);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1375987888U), "fileTimeLo",
                  "fileTimeLo", 33);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 33);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 33);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 33);
  sf_mex_assign(&c5_rhs33, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs33, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs33), "rhs", "rhs",
                  33);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs33), "lhs", "lhs",
                  33);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "context",
                  "context", 34);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("coder.internal.scalarEg"),
                  "name", "name", 34);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 34);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/scalarEg.p"),
                  "resolved", "resolved", 34);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1389311520U), "fileTimeLo",
                  "fileTimeLo", 34);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 34);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 34);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 34);
  sf_mex_assign(&c5_rhs34, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs34, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs34), "rhs", "rhs",
                  34);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs34), "lhs", "lhs",
                  34);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/eml_mtimes_helper.m"),
                  "context", "context", 35);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eml_xgemm"), "name", "name",
                  35);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 35);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgemm.m"),
                  "resolved", "resolved", 35);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1375987890U), "fileTimeLo",
                  "fileTimeLo", 35);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 35);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 35);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 35);
  sf_mex_assign(&c5_rhs35, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs35, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs35), "rhs", "rhs",
                  35);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs35), "lhs", "lhs",
                  35);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgemm.m"), "context",
                  "context", 36);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("coder.internal.blas.inline"),
                  "name", "name", 36);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 36);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/inline.p"),
                  "resolved", "resolved", 36);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1389311522U), "fileTimeLo",
                  "fileTimeLo", 36);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 36);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 36);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 36);
  sf_mex_assign(&c5_rhs36, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs36, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs36), "rhs", "rhs",
                  36);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs36), "lhs", "lhs",
                  36);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgemm.m"), "context",
                  "context", 37);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("coder.internal.blas.xgemm"),
                  "name", "name", 37);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 37);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xgemm.p"),
                  "resolved", "resolved", 37);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1389311522U), "fileTimeLo",
                  "fileTimeLo", 37);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 37);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 37);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 37);
  sf_mex_assign(&c5_rhs37, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs37, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs37), "rhs", "rhs",
                  37);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs37), "lhs", "lhs",
                  37);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xgemm.p"),
                  "context", "context", 38);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "coder.internal.blas.use_refblas"), "name", "name", 38);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 38);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/use_refblas.p"),
                  "resolved", "resolved", 38);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1389311522U), "fileTimeLo",
                  "fileTimeLo", 38);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 38);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 38);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 38);
  sf_mex_assign(&c5_rhs38, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs38, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs38), "rhs", "rhs",
                  38);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs38), "lhs", "lhs",
                  38);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xgemm.p!below_threshold"),
                  "context", "context", 39);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("coder.internal.blas.threshold"),
                  "name", "name", 39);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 39);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/threshold.p"),
                  "resolved", "resolved", 39);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1389311522U), "fileTimeLo",
                  "fileTimeLo", 39);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 39);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 39);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 39);
  sf_mex_assign(&c5_rhs39, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs39, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs39), "rhs", "rhs",
                  39);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs39), "lhs", "lhs",
                  39);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/threshold.p"),
                  "context", "context", 40);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eml_switch_helper"), "name",
                  "name", 40);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 40);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_switch_helper.m"),
                  "resolved", "resolved", 40);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1381857500U), "fileTimeLo",
                  "fileTimeLo", 40);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 40);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 40);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 40);
  sf_mex_assign(&c5_rhs40, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs40, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs40), "rhs", "rhs",
                  40);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs40), "lhs", "lhs",
                  40);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xgemm.p"),
                  "context", "context", 41);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("coder.internal.scalarEg"),
                  "name", "name", 41);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 41);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/scalarEg.p"),
                  "resolved", "resolved", 41);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1389311520U), "fileTimeLo",
                  "fileTimeLo", 41);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 41);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 41);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 41);
  sf_mex_assign(&c5_rhs41, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs41, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs41), "rhs", "rhs",
                  41);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs41), "lhs", "lhs",
                  41);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+blas/xgemm.p"),
                  "context", "context", 42);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("coder.internal.refblas.xgemm"),
                  "name", "name", 42);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 42);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/+refblas/xgemm.p"),
                  "resolved", "resolved", 42);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1389311522U), "fileTimeLo",
                  "fileTimeLo", 42);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 42);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 42);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 42);
  sf_mex_assign(&c5_rhs42, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs42, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs42), "rhs", "rhs",
                  42);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs42), "lhs", "lhs",
                  42);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(""), "context", "context", 43);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("repmat"), "name", "name", 43);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 43);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/repmat.m"), "resolved",
                  "resolved", 43);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1372589614U), "fileTimeLo",
                  "fileTimeLo", 43);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 43);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 43);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 43);
  sf_mex_assign(&c5_rhs43, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs43, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs43), "rhs", "rhs",
                  43);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs43), "lhs", "lhs",
                  43);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/repmat.m"), "context",
                  "context", 44);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("coder.internal.assert"),
                  "name", "name", 44);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 44);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/assert.m"),
                  "resolved", "resolved", 44);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1363718156U), "fileTimeLo",
                  "fileTimeLo", 44);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 44);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 44);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 44);
  sf_mex_assign(&c5_rhs44, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs44, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs44), "rhs", "rhs",
                  44);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs44), "lhs", "lhs",
                  44);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/repmat.m"), "context",
                  "context", 45);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eml_assert_valid_size_arg"),
                  "name", "name", 45);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 45);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m"),
                  "resolved", "resolved", 45);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1368190230U), "fileTimeLo",
                  "fileTimeLo", 45);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 45);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 45);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 45);
  sf_mex_assign(&c5_rhs45, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs45, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs45), "rhs", "rhs",
                  45);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs45), "lhs", "lhs",
                  45);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m"),
                  "context", "context", 46);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 46);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 46);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 46);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1363718156U), "fileTimeLo",
                  "fileTimeLo", 46);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 46);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 46);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 46);
  sf_mex_assign(&c5_rhs46, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs46, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs46), "rhs", "rhs",
                  46);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs46), "lhs", "lhs",
                  46);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m!isintegral"),
                  "context", "context", 47);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("isinf"), "name", "name", 47);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 47);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isinf.m"), "resolved",
                  "resolved", 47);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1363717456U), "fileTimeLo",
                  "fileTimeLo", 47);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 47);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 47);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 47);
  sf_mex_assign(&c5_rhs47, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs47, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs47), "rhs", "rhs",
                  47);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs47), "lhs", "lhs",
                  47);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isinf.m"), "context",
                  "context", 48);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 48);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 48);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 48);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1363718156U), "fileTimeLo",
                  "fileTimeLo", 48);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 48);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 48);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 48);
  sf_mex_assign(&c5_rhs48, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs48, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs48), "rhs", "rhs",
                  48);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs48), "lhs", "lhs",
                  48);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m!isinbounds"),
                  "context", "context", 49);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eml_is_integer_class"), "name",
                  "name", 49);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 49);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_is_integer_class.m"),
                  "resolved", "resolved", 49);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1286825982U), "fileTimeLo",
                  "fileTimeLo", 49);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 49);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 49);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 49);
  sf_mex_assign(&c5_rhs49, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs49, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs49), "rhs", "rhs",
                  49);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs49), "lhs", "lhs",
                  49);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m!isinbounds"),
                  "context", "context", 50);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("intmax"), "name", "name", 50);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 50);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m"), "resolved",
                  "resolved", 50);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1362265482U), "fileTimeLo",
                  "fileTimeLo", 50);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 50);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 50);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 50);
  sf_mex_assign(&c5_rhs50, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs50, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs50), "rhs", "rhs",
                  50);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs50), "lhs", "lhs",
                  50);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m!isinbounds"),
                  "context", "context", 51);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("intmin"), "name", "name", 51);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 51);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmin.m"), "resolved",
                  "resolved", 51);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1362265482U), "fileTimeLo",
                  "fileTimeLo", 51);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 51);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 51);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 51);
  sf_mex_assign(&c5_rhs51, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs51, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs51), "rhs", "rhs",
                  51);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs51), "lhs", "lhs",
                  51);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmin.m"), "context",
                  "context", 52);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eml_switch_helper"), "name",
                  "name", 52);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 52);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_switch_helper.m"),
                  "resolved", "resolved", 52);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1381857500U), "fileTimeLo",
                  "fileTimeLo", 52);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 52);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 52);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 52);
  sf_mex_assign(&c5_rhs52, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs52, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs52), "rhs", "rhs",
                  52);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs52), "lhs", "lhs",
                  52);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m!isinbounds"),
                  "context", "context", 53);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("coder.internal.indexIntRelop"),
                  "name", "name", 53);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 53);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexIntRelop.m"),
                  "resolved", "resolved", 53);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1326731922U), "fileTimeLo",
                  "fileTimeLo", 53);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 53);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 53);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 53);
  sf_mex_assign(&c5_rhs53, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs53, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs53), "rhs", "rhs",
                  53);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs53), "lhs", "lhs",
                  53);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexIntRelop.m!apply_float_relop"),
                  "context", "context", 54);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eml_switch_helper"), "name",
                  "name", 54);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 54);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_switch_helper.m"),
                  "resolved", "resolved", 54);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1381857500U), "fileTimeLo",
                  "fileTimeLo", 54);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 54);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 54);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 54);
  sf_mex_assign(&c5_rhs54, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs54, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs54), "rhs", "rhs",
                  54);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs54), "lhs", "lhs",
                  54);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexIntRelop.m!float_class_contains_indexIntClass"),
                  "context", "context", 55);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eml_float_model"), "name",
                  "name", 55);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 55);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_float_model.m"),
                  "resolved", "resolved", 55);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1326731596U), "fileTimeLo",
                  "fileTimeLo", 55);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 55);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 55);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 55);
  sf_mex_assign(&c5_rhs55, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs55, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs55), "rhs", "rhs",
                  55);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs55), "lhs", "lhs",
                  55);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexIntRelop.m!is_signed_indexIntClass"),
                  "context", "context", 56);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("intmin"), "name", "name", 56);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 56);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmin.m"), "resolved",
                  "resolved", 56);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1362265482U), "fileTimeLo",
                  "fileTimeLo", 56);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 56);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 56);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 56);
  sf_mex_assign(&c5_rhs56, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs56, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs56), "rhs", "rhs",
                  56);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs56), "lhs", "lhs",
                  56);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m"),
                  "context", "context", 57);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eml_index_class"), "name",
                  "name", 57);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 57);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m"),
                  "resolved", "resolved", 57);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1323174178U), "fileTimeLo",
                  "fileTimeLo", 57);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 57);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 57);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 57);
  sf_mex_assign(&c5_rhs57, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs57, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs57), "rhs", "rhs",
                  57);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs57), "lhs", "lhs",
                  57);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m"),
                  "context", "context", 58);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("intmax"), "name", "name", 58);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 58);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m"), "resolved",
                  "resolved", 58);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1362265482U), "fileTimeLo",
                  "fileTimeLo", 58);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 58);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 58);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 58);
  sf_mex_assign(&c5_rhs58, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs58, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs58), "rhs", "rhs",
                  58);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs58), "lhs", "lhs",
                  58);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/repmat.m"), "context",
                  "context", 59);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("max"), "name", "name", 59);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 59);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/max.m"), "resolved",
                  "resolved", 59);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1311262516U), "fileTimeLo",
                  "fileTimeLo", 59);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 59);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 59);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 59);
  sf_mex_assign(&c5_rhs59, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs59, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs59), "rhs", "rhs",
                  59);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs59), "lhs", "lhs",
                  59);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/max.m"), "context",
                  "context", 60);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eml_min_or_max"), "name",
                  "name", 60);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 60);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m"),
                  "resolved", "resolved", 60);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1378303184U), "fileTimeLo",
                  "fileTimeLo", 60);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 60);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 60);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 60);
  sf_mex_assign(&c5_rhs60, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs60, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs60), "rhs", "rhs",
                  60);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs60), "lhs", "lhs",
                  60);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum"),
                  "context", "context", 61);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eml_scalar_eg"), "name",
                  "name", 61);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 61);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "resolved",
                  "resolved", 61);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1375987888U), "fileTimeLo",
                  "fileTimeLo", 61);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 61);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 61);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 61);
  sf_mex_assign(&c5_rhs61, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs61, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs61), "rhs", "rhs",
                  61);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs61), "lhs", "lhs",
                  61);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "context",
                  "context", 62);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("coder.internal.scalarEg"),
                  "name", "name", 62);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 62);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/scalarEg.p"),
                  "resolved", "resolved", 62);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1389311520U), "fileTimeLo",
                  "fileTimeLo", 62);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 62);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 62);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 62);
  sf_mex_assign(&c5_rhs62, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs62, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs62), "rhs", "rhs",
                  62);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs62), "lhs", "lhs",
                  62);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum"),
                  "context", "context", 63);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eml_scalexp_alloc"), "name",
                  "name", 63);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 63);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m"),
                  "resolved", "resolved", 63);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1375987888U), "fileTimeLo",
                  "fileTimeLo", 63);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 63);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 63);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 63);
  sf_mex_assign(&c5_rhs63, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs63, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs63), "rhs", "rhs",
                  63);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs63), "lhs", "lhs",
                  63);
  sf_mex_destroy(&c5_rhs0);
  sf_mex_destroy(&c5_lhs0);
  sf_mex_destroy(&c5_rhs1);
  sf_mex_destroy(&c5_lhs1);
  sf_mex_destroy(&c5_rhs2);
  sf_mex_destroy(&c5_lhs2);
  sf_mex_destroy(&c5_rhs3);
  sf_mex_destroy(&c5_lhs3);
  sf_mex_destroy(&c5_rhs4);
  sf_mex_destroy(&c5_lhs4);
  sf_mex_destroy(&c5_rhs5);
  sf_mex_destroy(&c5_lhs5);
  sf_mex_destroy(&c5_rhs6);
  sf_mex_destroy(&c5_lhs6);
  sf_mex_destroy(&c5_rhs7);
  sf_mex_destroy(&c5_lhs7);
  sf_mex_destroy(&c5_rhs8);
  sf_mex_destroy(&c5_lhs8);
  sf_mex_destroy(&c5_rhs9);
  sf_mex_destroy(&c5_lhs9);
  sf_mex_destroy(&c5_rhs10);
  sf_mex_destroy(&c5_lhs10);
  sf_mex_destroy(&c5_rhs11);
  sf_mex_destroy(&c5_lhs11);
  sf_mex_destroy(&c5_rhs12);
  sf_mex_destroy(&c5_lhs12);
  sf_mex_destroy(&c5_rhs13);
  sf_mex_destroy(&c5_lhs13);
  sf_mex_destroy(&c5_rhs14);
  sf_mex_destroy(&c5_lhs14);
  sf_mex_destroy(&c5_rhs15);
  sf_mex_destroy(&c5_lhs15);
  sf_mex_destroy(&c5_rhs16);
  sf_mex_destroy(&c5_lhs16);
  sf_mex_destroy(&c5_rhs17);
  sf_mex_destroy(&c5_lhs17);
  sf_mex_destroy(&c5_rhs18);
  sf_mex_destroy(&c5_lhs18);
  sf_mex_destroy(&c5_rhs19);
  sf_mex_destroy(&c5_lhs19);
  sf_mex_destroy(&c5_rhs20);
  sf_mex_destroy(&c5_lhs20);
  sf_mex_destroy(&c5_rhs21);
  sf_mex_destroy(&c5_lhs21);
  sf_mex_destroy(&c5_rhs22);
  sf_mex_destroy(&c5_lhs22);
  sf_mex_destroy(&c5_rhs23);
  sf_mex_destroy(&c5_lhs23);
  sf_mex_destroy(&c5_rhs24);
  sf_mex_destroy(&c5_lhs24);
  sf_mex_destroy(&c5_rhs25);
  sf_mex_destroy(&c5_lhs25);
  sf_mex_destroy(&c5_rhs26);
  sf_mex_destroy(&c5_lhs26);
  sf_mex_destroy(&c5_rhs27);
  sf_mex_destroy(&c5_lhs27);
  sf_mex_destroy(&c5_rhs28);
  sf_mex_destroy(&c5_lhs28);
  sf_mex_destroy(&c5_rhs29);
  sf_mex_destroy(&c5_lhs29);
  sf_mex_destroy(&c5_rhs30);
  sf_mex_destroy(&c5_lhs30);
  sf_mex_destroy(&c5_rhs31);
  sf_mex_destroy(&c5_lhs31);
  sf_mex_destroy(&c5_rhs32);
  sf_mex_destroy(&c5_lhs32);
  sf_mex_destroy(&c5_rhs33);
  sf_mex_destroy(&c5_lhs33);
  sf_mex_destroy(&c5_rhs34);
  sf_mex_destroy(&c5_lhs34);
  sf_mex_destroy(&c5_rhs35);
  sf_mex_destroy(&c5_lhs35);
  sf_mex_destroy(&c5_rhs36);
  sf_mex_destroy(&c5_lhs36);
  sf_mex_destroy(&c5_rhs37);
  sf_mex_destroy(&c5_lhs37);
  sf_mex_destroy(&c5_rhs38);
  sf_mex_destroy(&c5_lhs38);
  sf_mex_destroy(&c5_rhs39);
  sf_mex_destroy(&c5_lhs39);
  sf_mex_destroy(&c5_rhs40);
  sf_mex_destroy(&c5_lhs40);
  sf_mex_destroy(&c5_rhs41);
  sf_mex_destroy(&c5_lhs41);
  sf_mex_destroy(&c5_rhs42);
  sf_mex_destroy(&c5_lhs42);
  sf_mex_destroy(&c5_rhs43);
  sf_mex_destroy(&c5_lhs43);
  sf_mex_destroy(&c5_rhs44);
  sf_mex_destroy(&c5_lhs44);
  sf_mex_destroy(&c5_rhs45);
  sf_mex_destroy(&c5_lhs45);
  sf_mex_destroy(&c5_rhs46);
  sf_mex_destroy(&c5_lhs46);
  sf_mex_destroy(&c5_rhs47);
  sf_mex_destroy(&c5_lhs47);
  sf_mex_destroy(&c5_rhs48);
  sf_mex_destroy(&c5_lhs48);
  sf_mex_destroy(&c5_rhs49);
  sf_mex_destroy(&c5_lhs49);
  sf_mex_destroy(&c5_rhs50);
  sf_mex_destroy(&c5_lhs50);
  sf_mex_destroy(&c5_rhs51);
  sf_mex_destroy(&c5_lhs51);
  sf_mex_destroy(&c5_rhs52);
  sf_mex_destroy(&c5_lhs52);
  sf_mex_destroy(&c5_rhs53);
  sf_mex_destroy(&c5_lhs53);
  sf_mex_destroy(&c5_rhs54);
  sf_mex_destroy(&c5_lhs54);
  sf_mex_destroy(&c5_rhs55);
  sf_mex_destroy(&c5_lhs55);
  sf_mex_destroy(&c5_rhs56);
  sf_mex_destroy(&c5_lhs56);
  sf_mex_destroy(&c5_rhs57);
  sf_mex_destroy(&c5_lhs57);
  sf_mex_destroy(&c5_rhs58);
  sf_mex_destroy(&c5_lhs58);
  sf_mex_destroy(&c5_rhs59);
  sf_mex_destroy(&c5_lhs59);
  sf_mex_destroy(&c5_rhs60);
  sf_mex_destroy(&c5_lhs60);
  sf_mex_destroy(&c5_rhs61);
  sf_mex_destroy(&c5_lhs61);
  sf_mex_destroy(&c5_rhs62);
  sf_mex_destroy(&c5_lhs62);
  sf_mex_destroy(&c5_rhs63);
  sf_mex_destroy(&c5_lhs63);
}

static const mxArray *c5_emlrt_marshallOut(const char * c5_u)
{
  const mxArray *c5_y = NULL;
  c5_y = NULL;
  sf_mex_assign(&c5_y, sf_mex_create("y", c5_u, 15, 0U, 0U, 0U, 2, 1, strlen
    (c5_u)), false);
  return c5_y;
}

static const mxArray *c5_b_emlrt_marshallOut(const uint32_T c5_u)
{
  const mxArray *c5_y = NULL;
  c5_y = NULL;
  sf_mex_assign(&c5_y, sf_mex_create("y", &c5_u, 7, 0U, 0U, 0U, 0), false);
  return c5_y;
}

static void c5_b_info_helper(const mxArray **c5_info)
{
  const mxArray *c5_rhs64 = NULL;
  const mxArray *c5_lhs64 = NULL;
  const mxArray *c5_rhs65 = NULL;
  const mxArray *c5_lhs65 = NULL;
  const mxArray *c5_rhs66 = NULL;
  const mxArray *c5_lhs66 = NULL;
  const mxArray *c5_rhs67 = NULL;
  const mxArray *c5_lhs67 = NULL;
  const mxArray *c5_rhs68 = NULL;
  const mxArray *c5_lhs68 = NULL;
  const mxArray *c5_rhs69 = NULL;
  const mxArray *c5_lhs69 = NULL;
  const mxArray *c5_rhs70 = NULL;
  const mxArray *c5_lhs70 = NULL;
  const mxArray *c5_rhs71 = NULL;
  const mxArray *c5_lhs71 = NULL;
  const mxArray *c5_rhs72 = NULL;
  const mxArray *c5_lhs72 = NULL;
  const mxArray *c5_rhs73 = NULL;
  const mxArray *c5_lhs73 = NULL;
  const mxArray *c5_rhs74 = NULL;
  const mxArray *c5_lhs74 = NULL;
  const mxArray *c5_rhs75 = NULL;
  const mxArray *c5_lhs75 = NULL;
  const mxArray *c5_rhs76 = NULL;
  const mxArray *c5_lhs76 = NULL;
  const mxArray *c5_rhs77 = NULL;
  const mxArray *c5_lhs77 = NULL;
  const mxArray *c5_rhs78 = NULL;
  const mxArray *c5_lhs78 = NULL;
  const mxArray *c5_rhs79 = NULL;
  const mxArray *c5_lhs79 = NULL;
  const mxArray *c5_rhs80 = NULL;
  const mxArray *c5_lhs80 = NULL;
  const mxArray *c5_rhs81 = NULL;
  const mxArray *c5_lhs81 = NULL;
  const mxArray *c5_rhs82 = NULL;
  const mxArray *c5_lhs82 = NULL;
  const mxArray *c5_rhs83 = NULL;
  const mxArray *c5_lhs83 = NULL;
  const mxArray *c5_rhs84 = NULL;
  const mxArray *c5_lhs84 = NULL;
  const mxArray *c5_rhs85 = NULL;
  const mxArray *c5_lhs85 = NULL;
  const mxArray *c5_rhs86 = NULL;
  const mxArray *c5_lhs86 = NULL;
  const mxArray *c5_rhs87 = NULL;
  const mxArray *c5_lhs87 = NULL;
  const mxArray *c5_rhs88 = NULL;
  const mxArray *c5_lhs88 = NULL;
  const mxArray *c5_rhs89 = NULL;
  const mxArray *c5_lhs89 = NULL;
  const mxArray *c5_rhs90 = NULL;
  const mxArray *c5_lhs90 = NULL;
  const mxArray *c5_rhs91 = NULL;
  const mxArray *c5_lhs91 = NULL;
  const mxArray *c5_rhs92 = NULL;
  const mxArray *c5_lhs92 = NULL;
  const mxArray *c5_rhs93 = NULL;
  const mxArray *c5_lhs93 = NULL;
  const mxArray *c5_rhs94 = NULL;
  const mxArray *c5_lhs94 = NULL;
  const mxArray *c5_rhs95 = NULL;
  const mxArray *c5_lhs95 = NULL;
  const mxArray *c5_rhs96 = NULL;
  const mxArray *c5_lhs96 = NULL;
  const mxArray *c5_rhs97 = NULL;
  const mxArray *c5_lhs97 = NULL;
  const mxArray *c5_rhs98 = NULL;
  const mxArray *c5_lhs98 = NULL;
  const mxArray *c5_rhs99 = NULL;
  const mxArray *c5_lhs99 = NULL;
  const mxArray *c5_rhs100 = NULL;
  const mxArray *c5_lhs100 = NULL;
  const mxArray *c5_rhs101 = NULL;
  const mxArray *c5_lhs101 = NULL;
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m"),
                  "context", "context", 64);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("coder.internal.scalexpAlloc"),
                  "name", "name", 64);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 64);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/scalexpAlloc.p"),
                  "resolved", "resolved", 64);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1389311520U), "fileTimeLo",
                  "fileTimeLo", 64);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 64);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 64);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 64);
  sf_mex_assign(&c5_rhs64, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs64, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs64), "rhs", "rhs",
                  64);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs64), "lhs", "lhs",
                  64);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum"),
                  "context", "context", 65);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eml_index_class"), "name",
                  "name", 65);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 65);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m"),
                  "resolved", "resolved", 65);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1323174178U), "fileTimeLo",
                  "fileTimeLo", 65);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 65);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 65);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 65);
  sf_mex_assign(&c5_rhs65, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs65, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs65), "rhs", "rhs",
                  65);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs65), "lhs", "lhs",
                  65);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_scalar_bin_extremum"),
                  "context", "context", 66);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eml_scalar_eg"), "name",
                  "name", 66);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 66);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "resolved",
                  "resolved", 66);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1375987888U), "fileTimeLo",
                  "fileTimeLo", 66);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 66);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 66);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 66);
  sf_mex_assign(&c5_rhs66, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs66, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs66), "rhs", "rhs",
                  66);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs66), "lhs", "lhs",
                  66);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_scalar_bin_extremum"),
                  "context", "context", 67);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 67);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 67);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 67);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1363718156U), "fileTimeLo",
                  "fileTimeLo", 67);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 67);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 67);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 67);
  sf_mex_assign(&c5_rhs67, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs67, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs67), "rhs", "rhs",
                  67);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs67), "lhs", "lhs",
                  67);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/repmat.m"), "context",
                  "context", 68);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eml_scalar_eg"), "name",
                  "name", 68);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 68);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "resolved",
                  "resolved", 68);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1375987888U), "fileTimeLo",
                  "fileTimeLo", 68);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 68);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 68);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 68);
  sf_mex_assign(&c5_rhs68, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs68, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs68), "rhs", "rhs",
                  68);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs68), "lhs", "lhs",
                  68);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/repmat.m"), "context",
                  "context", 69);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("coder.internal.prodsize"),
                  "name", "name", 69);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 69);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/prodsize.m"),
                  "resolved", "resolved", 69);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1360286188U), "fileTimeLo",
                  "fileTimeLo", 69);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 69);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 69);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 69);
  sf_mex_assign(&c5_rhs69, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs69, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs69), "rhs", "rhs",
                  69);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs69), "lhs", "lhs",
                  69);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/repmat.m"), "context",
                  "context", 70);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "eml_int_forloop_overflow_check"), "name", "name", 70);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 70);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m"),
                  "resolved", "resolved", 70);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1375987888U), "fileTimeLo",
                  "fileTimeLo", 70);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 70);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 70);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 70);
  sf_mex_assign(&c5_rhs70, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs70, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs70), "rhs", "rhs",
                  70);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs70), "lhs", "lhs",
                  70);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(""), "context", "context", 71);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("diag"), "name", "name", 71);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 71);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/diag.m"), "resolved",
                  "resolved", 71);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1363717454U), "fileTimeLo",
                  "fileTimeLo", 71);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 71);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 71);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 71);
  sf_mex_assign(&c5_rhs71, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs71, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs71), "rhs", "rhs",
                  71);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs71), "lhs", "lhs",
                  71);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/diag.m"), "context",
                  "context", 72);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("ismatrix"), "name", "name", 72);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 72);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/ismatrix.m"), "resolved",
                  "resolved", 72);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1331308458U), "fileTimeLo",
                  "fileTimeLo", 72);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 72);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 72);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 72);
  sf_mex_assign(&c5_rhs72, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs72, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs72), "rhs", "rhs",
                  72);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs72), "lhs", "lhs",
                  72);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/diag.m"), "context",
                  "context", 73);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eml_index_class"), "name",
                  "name", 73);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 73);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m"),
                  "resolved", "resolved", 73);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1323174178U), "fileTimeLo",
                  "fileTimeLo", 73);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 73);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 73);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 73);
  sf_mex_assign(&c5_rhs73, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs73, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs73), "rhs", "rhs",
                  73);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs73), "lhs", "lhs",
                  73);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/diag.m"), "context",
                  "context", 74);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eml_scalar_eg"), "name",
                  "name", 74);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 74);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "resolved",
                  "resolved", 74);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1375987888U), "fileTimeLo",
                  "fileTimeLo", 74);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 74);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 74);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 74);
  sf_mex_assign(&c5_rhs74, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs74, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs74), "rhs", "rhs",
                  74);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs74), "lhs", "lhs",
                  74);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/diag.m"), "context",
                  "context", 75);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "eml_int_forloop_overflow_check"), "name", "name", 75);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 75);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m"),
                  "resolved", "resolved", 75);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1375987888U), "fileTimeLo",
                  "fileTimeLo", 75);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 75);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 75);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 75);
  sf_mex_assign(&c5_rhs75, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs75, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs75), "rhs", "rhs",
                  75);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs75), "lhs", "lhs",
                  75);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(""), "context", "context", 76);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("sum"), "name", "name", 76);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 76);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/sum.m"), "resolved",
                  "resolved", 76);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1363717458U), "fileTimeLo",
                  "fileTimeLo", 76);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 76);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 76);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 76);
  sf_mex_assign(&c5_rhs76, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs76, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs76), "rhs", "rhs",
                  76);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs76), "lhs", "lhs",
                  76);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/sum.m"), "context",
                  "context", 77);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 77);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 77);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 77);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1363718156U), "fileTimeLo",
                  "fileTimeLo", 77);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 77);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 77);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 77);
  sf_mex_assign(&c5_rhs77, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs77, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs77), "rhs", "rhs",
                  77);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs77), "lhs", "lhs",
                  77);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/sum.m"), "context",
                  "context", 78);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eml_assert_valid_dim"), "name",
                  "name", 78);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 78);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_assert_valid_dim.m"),
                  "resolved", "resolved", 78);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1372589616U), "fileTimeLo",
                  "fileTimeLo", 78);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 78);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 78);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 78);
  sf_mex_assign(&c5_rhs78, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs78, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs78), "rhs", "rhs",
                  78);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs78), "lhs", "lhs",
                  78);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_assert_valid_dim.m"),
                  "context", "context", 79);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("coder.internal.assertValidDim"),
                  "name", "name", 79);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 79);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/assertValidDim.m"),
                  "resolved", "resolved", 79);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1372590360U), "fileTimeLo",
                  "fileTimeLo", 79);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 79);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 79);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 79);
  sf_mex_assign(&c5_rhs79, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs79, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs79), "rhs", "rhs",
                  79);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs79), "lhs", "lhs",
                  79);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/assertValidDim.m"),
                  "context", "context", 80);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 80);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 80);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 80);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1363718156U), "fileTimeLo",
                  "fileTimeLo", 80);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 80);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 80);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 80);
  sf_mex_assign(&c5_rhs80, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs80, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs80), "rhs", "rhs",
                  80);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs80), "lhs", "lhs",
                  80);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/assertValidDim.m"),
                  "context", "context", 81);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("floor"), "name", "name", 81);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 81);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/floor.m"), "resolved",
                  "resolved", 81);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1363717454U), "fileTimeLo",
                  "fileTimeLo", 81);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 81);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 81);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 81);
  sf_mex_assign(&c5_rhs81, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs81, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs81), "rhs", "rhs",
                  81);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs81), "lhs", "lhs",
                  81);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/floor.m"), "context",
                  "context", 82);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 82);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 82);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 82);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1363718156U), "fileTimeLo",
                  "fileTimeLo", 82);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 82);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 82);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 82);
  sf_mex_assign(&c5_rhs82, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs82, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs82), "rhs", "rhs",
                  82);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs82), "lhs", "lhs",
                  82);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/floor.m"), "context",
                  "context", 83);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eml_scalar_floor"), "name",
                  "name", 83);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 83);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_floor.m"),
                  "resolved", "resolved", 83);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1286825926U), "fileTimeLo",
                  "fileTimeLo", 83);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 83);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 83);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 83);
  sf_mex_assign(&c5_rhs83, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs83, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs83), "rhs", "rhs",
                  83);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs83), "lhs", "lhs",
                  83);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/assertValidDim.m"),
                  "context", "context", 84);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("intmax"), "name", "name", 84);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 84);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m"), "resolved",
                  "resolved", 84);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1362265482U), "fileTimeLo",
                  "fileTimeLo", 84);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 84);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 84);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 84);
  sf_mex_assign(&c5_rhs84, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs84, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs84), "rhs", "rhs",
                  84);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs84), "lhs", "lhs",
                  84);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/sum.m"), "context",
                  "context", 85);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eml_index_class"), "name",
                  "name", 85);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 85);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m"),
                  "resolved", "resolved", 85);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1323174178U), "fileTimeLo",
                  "fileTimeLo", 85);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 85);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 85);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 85);
  sf_mex_assign(&c5_rhs85, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs85, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs85), "rhs", "rhs",
                  85);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs85), "lhs", "lhs",
                  85);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/sum.m"), "context",
                  "context", 86);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eml_scalar_eg"), "name",
                  "name", 86);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 86);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "resolved",
                  "resolved", 86);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1375987888U), "fileTimeLo",
                  "fileTimeLo", 86);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 86);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 86);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 86);
  sf_mex_assign(&c5_rhs86, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs86, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs86), "rhs", "rhs",
                  86);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs86), "lhs", "lhs",
                  86);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/sum.m"), "context",
                  "context", 87);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eml_matrix_vstride"), "name",
                  "name", 87);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 87);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_matrix_vstride.m"),
                  "resolved", "resolved", 87);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1360285950U), "fileTimeLo",
                  "fileTimeLo", 87);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 87);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 87);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 87);
  sf_mex_assign(&c5_rhs87, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs87, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs87), "rhs", "rhs",
                  87);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs87), "lhs", "lhs",
                  87);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_matrix_vstride.m"),
                  "context", "context", 88);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("coder.internal.prodsize"),
                  "name", "name", 88);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 88);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/prodsize.m"),
                  "resolved", "resolved", 88);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1360286188U), "fileTimeLo",
                  "fileTimeLo", 88);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 88);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 88);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 88);
  sf_mex_assign(&c5_rhs88, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs88, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs88), "rhs", "rhs",
                  88);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs88), "lhs", "lhs",
                  88);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/sum.m"), "context",
                  "context", 89);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eml_matrix_npages"), "name",
                  "name", 89);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 89);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_matrix_npages.m"),
                  "resolved", "resolved", 89);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1360285950U), "fileTimeLo",
                  "fileTimeLo", 89);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 89);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 89);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 89);
  sf_mex_assign(&c5_rhs89, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs89, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs89), "rhs", "rhs",
                  89);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs89), "lhs", "lhs",
                  89);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_matrix_npages.m"),
                  "context", "context", 90);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("coder.internal.prodsize"),
                  "name", "name", 90);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 90);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/prodsize.m"),
                  "resolved", "resolved", 90);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1360286188U), "fileTimeLo",
                  "fileTimeLo", 90);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 90);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 90);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 90);
  sf_mex_assign(&c5_rhs90, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs90, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs90), "rhs", "rhs",
                  90);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs90), "lhs", "lhs",
                  90);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/sum.m"), "context",
                  "context", 91);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "eml_int_forloop_overflow_check"), "name", "name", 91);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 91);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m"),
                  "resolved", "resolved", 91);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1375987888U), "fileTimeLo",
                  "fileTimeLo", 91);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 91);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 91);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 91);
  sf_mex_assign(&c5_rhs91, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs91, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs91), "rhs", "rhs",
                  91);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs91), "lhs", "lhs",
                  91);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/sum.m"), "context",
                  "context", 92);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eml_index_plus"), "name",
                  "name", 92);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 92);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m"),
                  "resolved", "resolved", 92);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1372589616U), "fileTimeLo",
                  "fileTimeLo", 92);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 92);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 92);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 92);
  sf_mex_assign(&c5_rhs92, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs92, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs92), "rhs", "rhs",
                  92);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs92), "lhs", "lhs",
                  92);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/sum.m"), "context",
                  "context", 93);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eml_index_plus"), "name",
                  "name", 93);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 93);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m"),
                  "resolved", "resolved", 93);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1372589616U), "fileTimeLo",
                  "fileTimeLo", 93);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 93);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 93);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 93);
  sf_mex_assign(&c5_rhs93, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs93, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs93), "rhs", "rhs",
                  93);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs93), "lhs", "lhs",
                  93);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m"), "context",
                  "context", 94);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("coder.internal.indexPlus"),
                  "name", "name", 94);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 94);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexPlus.m"),
                  "resolved", "resolved", 94);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1372590360U), "fileTimeLo",
                  "fileTimeLo", 94);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 94);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 94);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 94);
  sf_mex_assign(&c5_rhs94, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs94, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs94), "rhs", "rhs",
                  94);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs94), "lhs", "lhs",
                  94);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(""), "context", "context", 95);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("atan2"), "name", "name", 95);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 95);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/atan2.m"), "resolved",
                  "resolved", 95);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1343837572U), "fileTimeLo",
                  "fileTimeLo", 95);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 95);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 95);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 95);
  sf_mex_assign(&c5_rhs95, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs95, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs95), "rhs", "rhs",
                  95);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs95), "lhs", "lhs",
                  95);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/atan2.m"), "context",
                  "context", 96);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eml_scalar_eg"), "name",
                  "name", 96);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 96);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "resolved",
                  "resolved", 96);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1375987888U), "fileTimeLo",
                  "fileTimeLo", 96);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 96);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 96);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 96);
  sf_mex_assign(&c5_rhs96, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs96, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs96), "rhs", "rhs",
                  96);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs96), "lhs", "lhs",
                  96);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/atan2.m"), "context",
                  "context", 97);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eml_scalexp_alloc"), "name",
                  "name", 97);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 97);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m"),
                  "resolved", "resolved", 97);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1375987888U), "fileTimeLo",
                  "fileTimeLo", 97);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 97);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 97);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 97);
  sf_mex_assign(&c5_rhs97, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs97, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs97), "rhs", "rhs",
                  97);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs97), "lhs", "lhs",
                  97);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m"),
                  "context", "context", 98);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("coder.internal.scalexpAlloc"),
                  "name", "name", 98);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 98);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/coder/coder/+coder/+internal/scalexpAlloc.p"),
                  "resolved", "resolved", 98);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1389311520U), "fileTimeLo",
                  "fileTimeLo", 98);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 98);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 98);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 98);
  sf_mex_assign(&c5_rhs98, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs98, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs98), "rhs", "rhs",
                  98);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs98), "lhs", "lhs",
                  98);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/atan2.m"), "context",
                  "context", 99);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("eml_scalar_atan2"), "name",
                  "name", 99);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 99);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_atan2.m"),
                  "resolved", "resolved", 99);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1286825920U), "fileTimeLo",
                  "fileTimeLo", 99);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 99);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 99);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 99);
  sf_mex_assign(&c5_rhs99, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs99, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs99), "rhs", "rhs",
                  99);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs99), "lhs", "lhs",
                  99);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(""), "context", "context", 100);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("cos"), "name", "name", 100);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 100);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/cos.m"), "resolved",
                  "resolved", 100);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1343837572U), "fileTimeLo",
                  "fileTimeLo", 100);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 100);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 100);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 100);
  sf_mex_assign(&c5_rhs100, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs100, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs100), "rhs", "rhs",
                  100);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs100), "lhs", "lhs",
                  100);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(""), "context", "context", 101);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("sin"), "name", "name", 101);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 101);
  sf_mex_addfield(*c5_info, c5_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sin.m"), "resolved",
                  "resolved", 101);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(1343837586U), "fileTimeLo",
                  "fileTimeLo", 101);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 101);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 101);
  sf_mex_addfield(*c5_info, c5_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 101);
  sf_mex_assign(&c5_rhs101, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_assign(&c5_lhs101, sf_mex_createcellmatrix(0, 1), false);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_rhs101), "rhs", "rhs",
                  101);
  sf_mex_addfield(*c5_info, sf_mex_duplicatearraysafe(&c5_lhs101), "lhs", "lhs",
                  101);
  sf_mex_destroy(&c5_rhs64);
  sf_mex_destroy(&c5_lhs64);
  sf_mex_destroy(&c5_rhs65);
  sf_mex_destroy(&c5_lhs65);
  sf_mex_destroy(&c5_rhs66);
  sf_mex_destroy(&c5_lhs66);
  sf_mex_destroy(&c5_rhs67);
  sf_mex_destroy(&c5_lhs67);
  sf_mex_destroy(&c5_rhs68);
  sf_mex_destroy(&c5_lhs68);
  sf_mex_destroy(&c5_rhs69);
  sf_mex_destroy(&c5_lhs69);
  sf_mex_destroy(&c5_rhs70);
  sf_mex_destroy(&c5_lhs70);
  sf_mex_destroy(&c5_rhs71);
  sf_mex_destroy(&c5_lhs71);
  sf_mex_destroy(&c5_rhs72);
  sf_mex_destroy(&c5_lhs72);
  sf_mex_destroy(&c5_rhs73);
  sf_mex_destroy(&c5_lhs73);
  sf_mex_destroy(&c5_rhs74);
  sf_mex_destroy(&c5_lhs74);
  sf_mex_destroy(&c5_rhs75);
  sf_mex_destroy(&c5_lhs75);
  sf_mex_destroy(&c5_rhs76);
  sf_mex_destroy(&c5_lhs76);
  sf_mex_destroy(&c5_rhs77);
  sf_mex_destroy(&c5_lhs77);
  sf_mex_destroy(&c5_rhs78);
  sf_mex_destroy(&c5_lhs78);
  sf_mex_destroy(&c5_rhs79);
  sf_mex_destroy(&c5_lhs79);
  sf_mex_destroy(&c5_rhs80);
  sf_mex_destroy(&c5_lhs80);
  sf_mex_destroy(&c5_rhs81);
  sf_mex_destroy(&c5_lhs81);
  sf_mex_destroy(&c5_rhs82);
  sf_mex_destroy(&c5_lhs82);
  sf_mex_destroy(&c5_rhs83);
  sf_mex_destroy(&c5_lhs83);
  sf_mex_destroy(&c5_rhs84);
  sf_mex_destroy(&c5_lhs84);
  sf_mex_destroy(&c5_rhs85);
  sf_mex_destroy(&c5_lhs85);
  sf_mex_destroy(&c5_rhs86);
  sf_mex_destroy(&c5_lhs86);
  sf_mex_destroy(&c5_rhs87);
  sf_mex_destroy(&c5_lhs87);
  sf_mex_destroy(&c5_rhs88);
  sf_mex_destroy(&c5_lhs88);
  sf_mex_destroy(&c5_rhs89);
  sf_mex_destroy(&c5_lhs89);
  sf_mex_destroy(&c5_rhs90);
  sf_mex_destroy(&c5_lhs90);
  sf_mex_destroy(&c5_rhs91);
  sf_mex_destroy(&c5_lhs91);
  sf_mex_destroy(&c5_rhs92);
  sf_mex_destroy(&c5_lhs92);
  sf_mex_destroy(&c5_rhs93);
  sf_mex_destroy(&c5_lhs93);
  sf_mex_destroy(&c5_rhs94);
  sf_mex_destroy(&c5_lhs94);
  sf_mex_destroy(&c5_rhs95);
  sf_mex_destroy(&c5_lhs95);
  sf_mex_destroy(&c5_rhs96);
  sf_mex_destroy(&c5_lhs96);
  sf_mex_destroy(&c5_rhs97);
  sf_mex_destroy(&c5_lhs97);
  sf_mex_destroy(&c5_rhs98);
  sf_mex_destroy(&c5_lhs98);
  sf_mex_destroy(&c5_rhs99);
  sf_mex_destroy(&c5_lhs99);
  sf_mex_destroy(&c5_rhs100);
  sf_mex_destroy(&c5_lhs100);
  sf_mex_destroy(&c5_rhs101);
  sf_mex_destroy(&c5_lhs101);
}

static void c5_IR_Raw_to_Distances(SFc5_BlueBird_FormationInstanceStruct
  *chartInstance, real_T c5_ir_raw[9], real_T c5_Answer[9], real_T
  *c5_ObstacleDetected)
{
  uint32_T c5_debug_family_var_map[9];
  real_T c5_SafeNumber;
  real_T c5_MaxNumber;
  real_T c5_ir_raw_Internal[9];
  int32_T c5_Number_List_sizes;
  real_T c5_Number_List_data[9];
  int32_T c5_ObstacleDetected_sizes;
  real_T c5_ObstacleDetected_data[9];
  real_T c5_nargin = 1.0;
  real_T c5_nargout = 2.0;
  int32_T c5_i98;
  int32_T c5_i99;
  boolean_T c5_x[9];
  int32_T c5_idx;
  static int32_T c5_iv0[1] = { 9 };

  int32_T c5_ii_sizes;
  int32_T c5_ii;
  int32_T c5_b_ii;
  int32_T c5_a;
  int32_T c5_b_a;
  int32_T c5_ii_data[9];
  boolean_T c5_b0;
  boolean_T c5_b1;
  boolean_T c5_b2;
  int32_T c5_i100;
  int32_T c5_tmp_sizes;
  int32_T c5_loop_ub;
  int32_T c5_i101;
  int32_T c5_tmp_data[9];
  int32_T c5_b_ii_sizes;
  int32_T c5_b_loop_ub;
  int32_T c5_i102;
  int32_T c5_b_ii_data[9];
  int32_T c5_c_loop_ub;
  int32_T c5_i103;
  int32_T c5_d_loop_ub;
  int32_T c5_i104;
  int32_T c5_e_loop_ub;
  int32_T c5_i105;
  int32_T c5_f_loop_ub;
  int32_T c5_i106;
  int32_T c5_i107;
  int32_T c5_b_idx;
  int32_T c5_c_ii;
  int32_T c5_d_ii;
  int32_T c5_c_a;
  int32_T c5_d_a;
  boolean_T c5_b3;
  boolean_T c5_b4;
  boolean_T c5_b5;
  int32_T c5_i108;
  int32_T c5_g_loop_ub;
  int32_T c5_i109;
  int32_T c5_c_ii_sizes;
  int32_T c5_h_loop_ub;
  int32_T c5_i110;
  int32_T c5_c_ii_data[9];
  int32_T c5_i_loop_ub;
  int32_T c5_i111;
  int32_T c5_j_loop_ub;
  int32_T c5_i112;
  int32_T c5_x_sizes;
  int32_T c5_k_loop_ub;
  int32_T c5_i113;
  real_T c5_x_data[9];
  int32_T c5_i114;
  int32_T c5_c_idx;
  int32_T c5_e_ii;
  int32_T c5_f_ii;
  int32_T c5_e_a;
  int32_T c5_f_a;
  boolean_T c5_b6;
  boolean_T c5_b7;
  boolean_T c5_b8;
  int32_T c5_i115;
  int32_T c5_l_loop_ub;
  int32_T c5_i116;
  int32_T c5_d_ii_sizes;
  int32_T c5_m_loop_ub;
  int32_T c5_i117;
  int32_T c5_d_ii_data[9];
  int32_T c5_n_loop_ub;
  int32_T c5_i118;
  int32_T c5_o_loop_ub;
  int32_T c5_i119;
  int32_T c5_p_loop_ub;
  int32_T c5_i120;
  int32_T c5_q_loop_ub;
  int32_T c5_i121;
  int32_T c5_i122;
  real_T c5_A[9];
  int32_T c5_i123;
  int32_T c5_k;
  real_T c5_b_k;
  int32_T c5_c_k;
  real_T c5_b_x;
  real_T c5_c_x;
  int32_T c5_i124;
  int32_T c5_i125;
  boolean_T exitg1;
  boolean_T exitg2;
  boolean_T exitg3;
  boolean_T guard1 = false;
  boolean_T guard2 = false;
  boolean_T guard3 = false;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 9U, 10U, c5_b_debug_family_names,
    c5_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c5_SafeNumber, 0U, c5_b_sf_marshallOut,
    c5_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c5_MaxNumber, 1U, c5_b_sf_marshallOut,
    c5_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c5_ir_raw_Internal, 2U,
    c5_d_sf_marshallOut, c5_g_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_DYN_IMPORTABLE(c5_Number_List_data, (const int32_T *)
    &c5_Number_List_sizes, NULL, 0, 3, (void *)c5_h_sf_marshallOut, (void *)
    c5_h_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_DYN_IMPORTABLE(c5_ObstacleDetected_data, (const
    int32_T *)&c5_ObstacleDetected_sizes, NULL, 0, -1, (void *)
    c5_h_sf_marshallOut, (void *)c5_h_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c5_nargin, 5U, c5_b_sf_marshallOut,
    c5_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c5_nargout, 6U, c5_b_sf_marshallOut,
    c5_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c5_ir_raw, 7U, c5_d_sf_marshallOut,
    c5_g_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c5_Answer, 8U, c5_d_sf_marshallOut,
    c5_g_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c5_ObstacleDetected, MAX_uint32_T,
    c5_b_sf_marshallOut, c5_b_sf_marshallIn);
  CV_EML_FCN(0, 1);
  _SFD_EML_CALL(0U, chartInstance->c5_sfEvent, 54);
  c5_SafeNumber = 150.0;
  _SFD_EML_CALL(0U, chartInstance->c5_sfEvent, 55);
  c5_MaxNumber = 3960.0;
  _SFD_EML_CALL(0U, chartInstance->c5_sfEvent, 57);
  for (c5_i98 = 0; c5_i98 < 9; c5_i98++) {
    c5_ir_raw_Internal[c5_i98] = c5_ir_raw[c5_i98];
  }

  _SFD_EML_CALL(0U, chartInstance->c5_sfEvent, 60);
  for (c5_i99 = 0; c5_i99 < 9; c5_i99++) {
    c5_x[c5_i99] = (c5_ir_raw_Internal[c5_i99] < 150.0);
  }

  c5_idx = 0;
  c5_ii_sizes = c5_iv0[0];
  c5_ii = 1;
  exitg3 = false;
  while ((exitg3 == false) && (c5_ii < 10)) {
    c5_b_ii = c5_ii;
    guard3 = false;
    if (c5_x[c5_b_ii - 1]) {
      c5_a = c5_idx;
      c5_b_a = c5_a + 1;
      c5_idx = c5_b_a;
      c5_ii_data[c5_idx - 1] = c5_b_ii;
      if (c5_idx >= 9) {
        exitg3 = true;
      } else {
        guard3 = true;
      }
    } else {
      guard3 = true;
    }

    if (guard3 == true) {
      c5_ii++;
    }
  }

  c5_b0 = (1 > c5_idx);
  c5_b1 = c5_b0;
  c5_b2 = c5_b1;
  if (c5_b2) {
    c5_i100 = 0;
  } else {
    c5_i100 = _SFD_EML_ARRAY_BOUNDS_CHECK("", c5_idx, 1, 9, 0, 0);
  }

  c5_tmp_sizes = c5_i100;
  c5_loop_ub = c5_i100 - 1;
  for (c5_i101 = 0; c5_i101 <= c5_loop_ub; c5_i101++) {
    c5_tmp_data[c5_i101] = 1 + c5_i101;
  }

  _SFD_VECTOR_VECTOR_INDEX_CHECK(9, 1, 1, c5_tmp_sizes);
  c5_b_ii_sizes = c5_tmp_sizes;
  c5_b_loop_ub = c5_tmp_sizes - 1;
  for (c5_i102 = 0; c5_i102 <= c5_b_loop_ub; c5_i102++) {
    c5_b_ii_data[c5_i102] = c5_ii_data[c5_tmp_data[c5_i102] - 1];
  }

  c5_ii_sizes = c5_b_ii_sizes;
  c5_c_loop_ub = c5_b_ii_sizes - 1;
  for (c5_i103 = 0; c5_i103 <= c5_c_loop_ub; c5_i103++) {
    c5_ii_data[c5_i103] = c5_b_ii_data[c5_i103];
  }

  c5_Number_List_sizes = c5_ii_sizes;
  c5_d_loop_ub = c5_ii_sizes - 1;
  for (c5_i104 = 0; c5_i104 <= c5_d_loop_ub; c5_i104++) {
    c5_Number_List_data[c5_i104] = (real_T)c5_ii_data[c5_i104];
  }

  _SFD_EML_CALL(0U, chartInstance->c5_sfEvent, 61);
  c5_tmp_sizes = c5_Number_List_sizes;
  c5_e_loop_ub = c5_Number_List_sizes - 1;
  for (c5_i105 = 0; c5_i105 <= c5_e_loop_ub; c5_i105++) {
    c5_tmp_data[c5_i105] = (int32_T)c5_Number_List_data[c5_i105];
  }

  c5_f_loop_ub = c5_tmp_sizes - 1;
  for (c5_i106 = 0; c5_i106 <= c5_f_loop_ub; c5_i106++) {
    c5_ir_raw_Internal[c5_tmp_data[c5_i106] - 1] = 150.0;
  }

  _SFD_EML_CALL(0U, chartInstance->c5_sfEvent, 64);
  for (c5_i107 = 0; c5_i107 < 9; c5_i107++) {
    c5_x[c5_i107] = (c5_ir_raw_Internal[c5_i107] > 150.0);
  }

  c5_b_idx = 0;
  c5_ii_sizes = c5_iv0[0];
  c5_c_ii = 1;
  exitg2 = false;
  while ((exitg2 == false) && (c5_c_ii < 10)) {
    c5_d_ii = c5_c_ii;
    guard2 = false;
    if (c5_x[c5_d_ii - 1]) {
      c5_c_a = c5_b_idx;
      c5_d_a = c5_c_a + 1;
      c5_b_idx = c5_d_a;
      c5_ii_data[c5_b_idx - 1] = c5_d_ii;
      if (c5_b_idx >= 9) {
        exitg2 = true;
      } else {
        guard2 = true;
      }
    } else {
      guard2 = true;
    }

    if (guard2 == true) {
      c5_c_ii++;
    }
  }

  c5_b3 = (1 > c5_b_idx);
  c5_b4 = c5_b3;
  c5_b5 = c5_b4;
  if (c5_b5) {
    c5_i108 = 0;
  } else {
    c5_i108 = _SFD_EML_ARRAY_BOUNDS_CHECK("", c5_b_idx, 1, 9, 0, 0);
  }

  c5_tmp_sizes = c5_i108;
  c5_g_loop_ub = c5_i108 - 1;
  for (c5_i109 = 0; c5_i109 <= c5_g_loop_ub; c5_i109++) {
    c5_tmp_data[c5_i109] = 1 + c5_i109;
  }

  _SFD_VECTOR_VECTOR_INDEX_CHECK(9, 1, 1, c5_tmp_sizes);
  c5_c_ii_sizes = c5_tmp_sizes;
  c5_h_loop_ub = c5_tmp_sizes - 1;
  for (c5_i110 = 0; c5_i110 <= c5_h_loop_ub; c5_i110++) {
    c5_c_ii_data[c5_i110] = c5_ii_data[c5_tmp_data[c5_i110] - 1];
  }

  c5_ii_sizes = c5_c_ii_sizes;
  c5_i_loop_ub = c5_c_ii_sizes - 1;
  for (c5_i111 = 0; c5_i111 <= c5_i_loop_ub; c5_i111++) {
    c5_ii_data[c5_i111] = c5_c_ii_data[c5_i111];
  }

  c5_ObstacleDetected_sizes = c5_ii_sizes;
  c5_j_loop_ub = c5_ii_sizes - 1;
  for (c5_i112 = 0; c5_i112 <= c5_j_loop_ub; c5_i112++) {
    c5_ObstacleDetected_data[c5_i112] = (real_T)c5_ii_data[c5_i112];
  }

  _SFD_SYMBOL_SWITCH(4U, 4U);
  _SFD_EML_CALL(0U, chartInstance->c5_sfEvent, 65);
  c5_x_sizes = c5_ObstacleDetected_sizes;
  c5_k_loop_ub = c5_ObstacleDetected_sizes - 1;
  for (c5_i113 = 0; c5_i113 <= c5_k_loop_ub; c5_i113++) {
    c5_x_data[c5_i113] = c5_ObstacleDetected_data[c5_i113];
  }

  *c5_ObstacleDetected = (real_T)c5_x_sizes;
  _SFD_SYMBOL_SWITCH(4U, 9U);
  _SFD_EML_CALL(0U, chartInstance->c5_sfEvent, 68);
  for (c5_i114 = 0; c5_i114 < 9; c5_i114++) {
    c5_x[c5_i114] = (c5_ir_raw_Internal[c5_i114] > 3960.0);
  }

  c5_c_idx = 0;
  c5_ii_sizes = c5_iv0[0];
  c5_e_ii = 1;
  exitg1 = false;
  while ((exitg1 == false) && (c5_e_ii < 10)) {
    c5_f_ii = c5_e_ii;
    guard1 = false;
    if (c5_x[c5_f_ii - 1]) {
      c5_e_a = c5_c_idx;
      c5_f_a = c5_e_a + 1;
      c5_c_idx = c5_f_a;
      c5_ii_data[c5_c_idx - 1] = c5_f_ii;
      if (c5_c_idx >= 9) {
        exitg1 = true;
      } else {
        guard1 = true;
      }
    } else {
      guard1 = true;
    }

    if (guard1 == true) {
      c5_e_ii++;
    }
  }

  c5_b6 = (1 > c5_c_idx);
  c5_b7 = c5_b6;
  c5_b8 = c5_b7;
  if (c5_b8) {
    c5_i115 = 0;
  } else {
    c5_i115 = _SFD_EML_ARRAY_BOUNDS_CHECK("", c5_c_idx, 1, 9, 0, 0);
  }

  c5_tmp_sizes = c5_i115;
  c5_l_loop_ub = c5_i115 - 1;
  for (c5_i116 = 0; c5_i116 <= c5_l_loop_ub; c5_i116++) {
    c5_tmp_data[c5_i116] = 1 + c5_i116;
  }

  _SFD_VECTOR_VECTOR_INDEX_CHECK(9, 1, 1, c5_tmp_sizes);
  c5_d_ii_sizes = c5_tmp_sizes;
  c5_m_loop_ub = c5_tmp_sizes - 1;
  for (c5_i117 = 0; c5_i117 <= c5_m_loop_ub; c5_i117++) {
    c5_d_ii_data[c5_i117] = c5_ii_data[c5_tmp_data[c5_i117] - 1];
  }

  c5_ii_sizes = c5_d_ii_sizes;
  c5_n_loop_ub = c5_d_ii_sizes - 1;
  for (c5_i118 = 0; c5_i118 <= c5_n_loop_ub; c5_i118++) {
    c5_ii_data[c5_i118] = c5_d_ii_data[c5_i118];
  }

  c5_Number_List_sizes = c5_ii_sizes;
  c5_o_loop_ub = c5_ii_sizes - 1;
  for (c5_i119 = 0; c5_i119 <= c5_o_loop_ub; c5_i119++) {
    c5_Number_List_data[c5_i119] = (real_T)c5_ii_data[c5_i119];
  }

  _SFD_EML_CALL(0U, chartInstance->c5_sfEvent, 69);
  c5_tmp_sizes = c5_Number_List_sizes;
  c5_p_loop_ub = c5_Number_List_sizes - 1;
  for (c5_i120 = 0; c5_i120 <= c5_p_loop_ub; c5_i120++) {
    c5_tmp_data[c5_i120] = (int32_T)c5_Number_List_data[c5_i120];
  }

  c5_q_loop_ub = c5_tmp_sizes - 1;
  for (c5_i121 = 0; c5_i121 <= c5_q_loop_ub; c5_i121++) {
    c5_ir_raw_Internal[c5_tmp_data[c5_i121] - 1] = 3960.0;
  }

  _SFD_EML_CALL(0U, chartInstance->c5_sfEvent, 72);
  for (c5_i122 = 0; c5_i122 < 9; c5_i122++) {
    c5_A[c5_i122] = c5_ir_raw_Internal[c5_i122];
  }

  for (c5_i123 = 0; c5_i123 < 9; c5_i123++) {
    c5_A[c5_i123] /= 3960.0;
  }

  for (c5_k = 0; c5_k < 9; c5_k++) {
    c5_b_k = 1.0 + (real_T)c5_k;
    if (c5_A[(int32_T)c5_b_k - 1] < 0.0) {
      c5_eml_error(chartInstance);
    }
  }

  for (c5_c_k = 0; c5_c_k < 9; c5_c_k++) {
    c5_b_k = 1.0 + (real_T)c5_c_k;
    c5_b_x = c5_A[(int32_T)c5_b_k - 1];
    c5_c_x = c5_b_x;
    c5_c_x = muDoubleScalarLog(c5_c_x);
    c5_A[(int32_T)c5_b_k - 1] = c5_c_x;
  }

  for (c5_i124 = 0; c5_i124 < 9; c5_i124++) {
    c5_A[c5_i124] /= 30.0;
  }

  for (c5_i125 = 0; c5_i125 < 9; c5_i125++) {
    c5_Answer[c5_i125] = 0.02 - c5_A[c5_i125];
  }

  _SFD_EML_CALL(0U, chartInstance->c5_sfEvent, -72);
  _SFD_SYMBOL_SCOPE_POP();
}

static void c5_eml_error(SFc5_BlueBird_FormationInstanceStruct *chartInstance)
{
  int32_T c5_i126;
  static char_T c5_cv1[28] = { 'M', 'A', 'T', 'L', 'A', 'B', ':', 'r', 'e', 'a',
    'l', 'l', 'o', 'g', ':', 'c', 'o', 'm', 'p', 'l', 'e', 'x', 'R', 'e', 's',
    'u', 'l', 't' };

  char_T c5_u[28];
  const mxArray *c5_y = NULL;
  (void)chartInstance;
  for (c5_i126 = 0; c5_i126 < 28; c5_i126++) {
    c5_u[c5_i126] = c5_cv1[c5_i126];
  }

  c5_y = NULL;
  sf_mex_assign(&c5_y, sf_mex_create("y", c5_u, 10, 0U, 1U, 0U, 2, 1, 28), false);
  sf_mex_call_debug(sfGlobalDebugInstanceStruct, "error", 0U, 1U, 14,
                    sf_mex_call_debug(sfGlobalDebugInstanceStruct, "message", 1U,
    1U, 14, c5_y));
}

static void c5_GetIR_Position(SFc5_BlueBird_FormationInstanceStruct
  *chartInstance, real_T c5_IR_Number, real_T c5_Answer[3])
{
  uint32_T c5_debug_family_var_map[5];
  real_T c5_IR_Position[27];
  real_T c5_nargin = 1.0;
  real_T c5_nargout = 1.0;
  real_T c5_x;
  real_T c5_y;
  real_T c5_theta;
  real_T c5_b_nargin = 4.0;
  real_T c5_b_nargout = 1.0;
  int32_T c5_i127;
  static real_T c5_dv10[27] = { -0.038, 0.017, 0.051, 0.067, 0.067, 0.051, 0.017,
    -0.038, -0.052, 0.049, 0.063, 0.045, 0.015, -0.015, -0.045, -0.063, -0.049,
    0.0, 2.2340214425527418, 1.3089969389957472, 0.73303828583761843,
    0.22689280275926285, -0.22689280275926285, -0.73303828583761843,
    -1.3089969389957472, -2.2340214425527418, 3.1415926535897931 };

  int32_T c5_b_IR_Number;
  int32_T c5_i128;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 5U, 5U, c5_e_debug_family_names,
    c5_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c5_IR_Position, 0U, c5_i_sf_marshallOut,
    c5_i_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c5_nargin, 1U, c5_b_sf_marshallOut,
    c5_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c5_nargout, 2U, c5_b_sf_marshallOut,
    c5_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c5_IR_Number, 3U, c5_b_sf_marshallOut,
    c5_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c5_Answer, 4U, c5_c_sf_marshallOut,
    c5_e_sf_marshallIn);
  CV_EML_FCN(0, 2);
  _SFD_EML_CALL(0U, chartInstance->c5_sfEvent, 76);
  c5_x = 0.0;
  c5_y = 0.0;
  c5_theta = 0.0;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 5U, 5U, c5_c_debug_family_names,
    c5_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c5_b_nargin, 0U, c5_b_sf_marshallOut,
    c5_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c5_b_nargout, 1U, c5_b_sf_marshallOut,
    c5_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c5_x, 2U, c5_b_sf_marshallOut,
    c5_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c5_y, 3U, c5_b_sf_marshallOut,
    c5_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c5_theta, 4U, c5_b_sf_marshallOut,
    c5_b_sf_marshallIn);
  CV_SCRIPT_FCN(0, 0);
  _SFD_SCRIPT_CALL(0U, chartInstance->c5_sfEvent, 1);
  _SFD_SCRIPT_CALL(0U, chartInstance->c5_sfEvent, 15);
  _SFD_SCRIPT_CALL(0U, chartInstance->c5_sfEvent, 16);
  _SFD_SCRIPT_CALL(0U, chartInstance->c5_sfEvent, 17);
  _SFD_SCRIPT_CALL(0U, chartInstance->c5_sfEvent, -17);
  _SFD_SYMBOL_SCOPE_POP();
  _SFD_EML_CALL(0U, chartInstance->c5_sfEvent, 77);
  for (c5_i127 = 0; c5_i127 < 27; c5_i127++) {
    c5_IR_Position[c5_i127] = c5_dv10[c5_i127];
  }

  _SFD_EML_CALL(0U, chartInstance->c5_sfEvent, 88);
  c5_b_IR_Number = (int32_T)c5_IR_Number - 1;
  for (c5_i128 = 0; c5_i128 < 3; c5_i128++) {
    c5_Answer[c5_i128] = c5_IR_Position[c5_b_IR_Number + 9 * c5_i128];
  }

  _SFD_EML_CALL(0U, chartInstance->c5_sfEvent, -88);
  _SFD_SYMBOL_SCOPE_POP();
}

static void c5_Transformation_Matrix(SFc5_BlueBird_FormationInstanceStruct
  *chartInstance, real_T c5_x, real_T c5_y, real_T c5_theta, real_T c5_Answer[9])
{
  uint32_T c5_debug_family_var_map[6];
  real_T c5_nargin = 3.0;
  real_T c5_nargout = 1.0;
  uint32_T c5_b_debug_family_var_map[2];
  real_T c5_b_nargin = 1.0;
  real_T c5_b_nargout = 1.0;
  uint32_T c5_c_debug_family_var_map[3];
  char_T c5_type[15];
  real_T c5_c_nargin = 1.0;
  real_T c5_c_nargout = 0.0;
  int32_T c5_i129;
  static char_T c5_cv2[15] = { 'a', 'v', 'o', 'i', 'd', '_', 'o', 'b', 's', 't',
    'a', 'c', 'l', 'e', 's' };

  real_T c5_b_x;
  real_T c5_b_y;
  real_T c5_b_theta;
  real_T c5_d_nargin = 4.0;
  real_T c5_d_nargout = 1.0;
  real_T c5_c_x;
  real_T c5_d_x;
  real_T c5_e_x;
  real_T c5_f_x;
  real_T c5_g_x;
  real_T c5_h_x;
  real_T c5_i_x;
  real_T c5_j_x;
  int32_T c5_i130;
  int32_T c5_i131;
  static real_T c5_dv11[3] = { 0.0, 0.0, 1.0 };

  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 6U, 6U, c5_i_debug_family_names,
    c5_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c5_nargin, 0U, c5_b_sf_marshallOut,
    c5_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c5_nargout, 1U, c5_b_sf_marshallOut,
    c5_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c5_x, 2U, c5_b_sf_marshallOut,
    c5_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c5_y, 3U, c5_b_sf_marshallOut,
    c5_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c5_theta, 4U, c5_b_sf_marshallOut,
    c5_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c5_Answer, 5U, c5_g_sf_marshallOut,
    c5_f_sf_marshallIn);
  CV_EML_FCN(0, 3);
  _SFD_EML_CALL(0U, chartInstance->c5_sfEvent, 94);
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 2U, 2U, c5_g_debug_family_names,
    c5_b_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c5_b_nargin, 0U, c5_b_sf_marshallOut,
    c5_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c5_b_nargout, 1U, c5_b_sf_marshallOut,
    c5_b_sf_marshallIn);
  CV_SCRIPT_FCN(2, 0);
  _SFD_SCRIPT_CALL(2U, chartInstance->c5_sfEvent, 33);
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 3U, 3U, c5_f_debug_family_names,
    c5_c_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c5_type, 0U, c5_j_sf_marshallOut,
    c5_j_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c5_c_nargin, 1U, c5_b_sf_marshallOut,
    c5_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c5_c_nargout, 2U, c5_b_sf_marshallOut,
    c5_b_sf_marshallIn);
  for (c5_i129 = 0; c5_i129 < 15; c5_i129++) {
    c5_type[c5_i129] = c5_cv2[c5_i129];
  }

  CV_SCRIPT_FCN(1, 0);
  _SFD_SCRIPT_CALL(1U, chartInstance->c5_sfEvent, 1);
  _SFD_SCRIPT_CALL(1U, chartInstance->c5_sfEvent, 32);
  _SFD_SCRIPT_CALL(1U, chartInstance->c5_sfEvent, -32);
  _SFD_SYMBOL_SCOPE_POP();
  _SFD_SCRIPT_CALL(2U, chartInstance->c5_sfEvent, 34);
  _SFD_SCRIPT_CALL(2U, chartInstance->c5_sfEvent, 36);
  _SFD_SCRIPT_CALL(2U, chartInstance->c5_sfEvent, 37);
  _SFD_SCRIPT_CALL(2U, chartInstance->c5_sfEvent, 38);
  _SFD_SCRIPT_CALL(2U, chartInstance->c5_sfEvent, 40);
  _SFD_SCRIPT_CALL(2U, chartInstance->c5_sfEvent, 41);
  _SFD_SCRIPT_CALL(2U, chartInstance->c5_sfEvent, -41);
  _SFD_SYMBOL_SCOPE_POP();
  _SFD_EML_CALL(0U, chartInstance->c5_sfEvent, 95);
  c5_b_x = c5_x;
  c5_b_y = c5_y;
  c5_b_theta = c5_theta;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 6U, 6U, c5_h_debug_family_names,
    c5_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c5_d_nargin, 0U, c5_b_sf_marshallOut,
    c5_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c5_d_nargout, 1U, c5_b_sf_marshallOut,
    c5_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c5_b_x, 2U, c5_b_sf_marshallOut,
    c5_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c5_b_y, 3U, c5_b_sf_marshallOut,
    c5_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c5_b_theta, 4U, c5_b_sf_marshallOut,
    c5_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c5_Answer, 5U, c5_g_sf_marshallOut,
    c5_f_sf_marshallIn);
  CV_SCRIPT_FCN(2, 4);
  _SFD_SCRIPT_CALL(2U, chartInstance->c5_sfEvent, 142U);
  c5_c_x = c5_b_theta;
  c5_d_x = c5_c_x;
  c5_d_x = muDoubleScalarCos(c5_d_x);
  c5_e_x = c5_b_theta;
  c5_f_x = c5_e_x;
  c5_f_x = muDoubleScalarSin(c5_f_x);
  c5_g_x = c5_b_theta;
  c5_h_x = c5_g_x;
  c5_h_x = muDoubleScalarSin(c5_h_x);
  c5_i_x = c5_b_theta;
  c5_j_x = c5_i_x;
  c5_j_x = muDoubleScalarCos(c5_j_x);
  c5_Answer[0] = c5_d_x;
  c5_Answer[3] = -c5_f_x;
  c5_Answer[6] = c5_b_x;
  c5_Answer[1] = c5_h_x;
  c5_Answer[4] = c5_j_x;
  c5_Answer[7] = c5_b_y;
  c5_i130 = 0;
  for (c5_i131 = 0; c5_i131 < 3; c5_i131++) {
    c5_Answer[c5_i130 + 2] = c5_dv11[c5_i131];
    c5_i130 += 3;
  }

  _SFD_SCRIPT_CALL(2U, chartInstance->c5_sfEvent, -142);
  _SFD_SYMBOL_SCOPE_POP();
  _SFD_EML_CALL(0U, chartInstance->c5_sfEvent, -95);
  _SFD_SYMBOL_SCOPE_POP();
}

static void c5_eml_scalar_eg(SFc5_BlueBird_FormationInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void c5_threshold(SFc5_BlueBird_FormationInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void c5_b_eml_scalar_eg(SFc5_BlueBird_FormationInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void c5_AvoidObstacles_NextPoint(SFc5_BlueBird_FormationInstanceStruct
  *chartInstance, real_T c5_Obstacle_AroundRobot_World[27], real_T c5_Position[3],
  real_T c5_ObstaclesSpeed[2])
{
  uint32_T c5_debug_family_var_map[12];
  real_T c5_FixIR_9_Number;
  real_T c5_sensor_gains[9];
  real_T c5_ir_distances_rf[18];
  real_T c5_u_i[18];
  real_T c5_SumOfObstacleVector_Word[2];
  real_T c5_NextAngle;
  real_T c5_k;
  real_T c5_nargin = 2.0;
  real_T c5_nargout = 1.0;
  int32_T c5_i132;
  int32_T c5_i133;
  int32_T c5_i134;
  int32_T c5_i135;
  int32_T c5_i136;
  real_T c5_b_Position[2];
  real_T c5_a[18];
  int32_T c5_i137;
  int32_T c5_i138;
  int32_T c5_i139;
  int32_T c5_i140;
  real_T c5_C[18];
  int32_T c5_i141;
  int32_T c5_i142;
  int32_T c5_i143;
  int32_T c5_i144;
  int32_T c5_i145;
  int32_T c5_i146;
  int32_T c5_i147;
  int32_T c5_i148;
  int32_T c5_i149;
  static real_T c5_b[81] = { 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  int32_T c5_i150;
  int32_T c5_iy;
  int32_T c5_ixstart;
  int32_T c5_j;
  int32_T c5_b_a;
  int32_T c5_c_a;
  int32_T c5_ix;
  real_T c5_s;
  int32_T c5_b_k;
  int32_T c5_d_a;
  int32_T c5_e_a;
  int32_T c5_f_a;
  int32_T c5_g_a;
  real_T c5_y;
  real_T c5_x;
  real_T c5_b_y;
  real_T c5_b_x;
  real_T c5_c_x;
  real_T c5_d_x;
  real_T c5_e_x;
  real_T c5_f_x;
  real_T c5_b_b[2];
  int32_T c5_i151;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 12U, 12U, c5_j_debug_family_names,
    c5_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c5_FixIR_9_Number, 0U,
    c5_b_sf_marshallOut, c5_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(c5_sensor_gains, 1U, c5_l_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c5_ir_distances_rf, 2U,
    c5_k_sf_marshallOut, c5_k_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c5_u_i, 3U, c5_k_sf_marshallOut,
    c5_k_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c5_SumOfObstacleVector_Word, 4U,
    c5_e_sf_marshallOut, c5_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c5_NextAngle, 5U, c5_b_sf_marshallOut,
    c5_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c5_k, 6U, c5_b_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c5_nargin, 7U, c5_b_sf_marshallOut,
    c5_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c5_nargout, 8U, c5_b_sf_marshallOut,
    c5_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c5_Obstacle_AroundRobot_World, 9U,
    c5_f_sf_marshallOut, c5_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c5_Position, 10U, c5_c_sf_marshallOut,
    c5_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c5_ObstaclesSpeed, 11U,
    c5_e_sf_marshallOut, c5_c_sf_marshallIn);
  CV_EML_FCN(0, 4);
  _SFD_EML_CALL(0U, chartInstance->c5_sfEvent, 99);
  c5_FixIR_9_Number = 0.364;
  _SFD_EML_CALL(0U, chartInstance->c5_sfEvent, 102);
  for (c5_i132 = 0; c5_i132 < 9; c5_i132++) {
    c5_sensor_gains[c5_i132] = 1.0;
  }

  _SFD_EML_CALL(0U, chartInstance->c5_sfEvent, 105);
  c5_i133 = 0;
  c5_i134 = 0;
  for (c5_i135 = 0; c5_i135 < 9; c5_i135++) {
    for (c5_i136 = 0; c5_i136 < 2; c5_i136++) {
      c5_ir_distances_rf[c5_i136 + c5_i133] =
        c5_Obstacle_AroundRobot_World[c5_i136 + c5_i134];
    }

    c5_i133 += 2;
    c5_i134 += 3;
  }

  _SFD_EML_CALL(0U, chartInstance->c5_sfEvent, 106);
  c5_b_Position[0] = c5_Position[0];
  c5_b_Position[1] = c5_Position[1];
  c5_repmat(chartInstance, c5_b_Position, c5_a);
  for (c5_i137 = 0; c5_i137 < 18; c5_i137++) {
    c5_a[c5_i137] = c5_ir_distances_rf[c5_i137] - c5_a[c5_i137];
  }

  c5_c_eml_scalar_eg(chartInstance);
  c5_c_eml_scalar_eg(chartInstance);
  for (c5_i138 = 0; c5_i138 < 18; c5_i138++) {
    c5_u_i[c5_i138] = 0.0;
  }

  for (c5_i139 = 0; c5_i139 < 18; c5_i139++) {
    c5_u_i[c5_i139] = 0.0;
  }

  for (c5_i140 = 0; c5_i140 < 18; c5_i140++) {
    c5_C[c5_i140] = c5_u_i[c5_i140];
  }

  for (c5_i141 = 0; c5_i141 < 18; c5_i141++) {
    c5_u_i[c5_i141] = c5_C[c5_i141];
  }

  c5_threshold(chartInstance);
  for (c5_i142 = 0; c5_i142 < 18; c5_i142++) {
    c5_C[c5_i142] = c5_u_i[c5_i142];
  }

  for (c5_i143 = 0; c5_i143 < 18; c5_i143++) {
    c5_u_i[c5_i143] = c5_C[c5_i143];
  }

  for (c5_i144 = 0; c5_i144 < 2; c5_i144++) {
    c5_i145 = 0;
    c5_i146 = 0;
    for (c5_i147 = 0; c5_i147 < 9; c5_i147++) {
      c5_u_i[c5_i145 + c5_i144] = 0.0;
      c5_i148 = 0;
      for (c5_i149 = 0; c5_i149 < 9; c5_i149++) {
        c5_u_i[c5_i145 + c5_i144] += c5_a[c5_i148 + c5_i144] * c5_b[c5_i149 +
          c5_i146];
        c5_i148 += 2;
      }

      c5_i145 += 2;
      c5_i146 += 9;
    }
  }

  _SFD_EML_CALL(0U, chartInstance->c5_sfEvent, 109);
  for (c5_i150 = 0; c5_i150 < 18; c5_i150++) {
    c5_a[c5_i150] = c5_u_i[c5_i150];
  }

  c5_iy = 0;
  c5_ixstart = 0;
  for (c5_j = 1; c5_j < 3; c5_j++) {
    c5_b_a = c5_ixstart;
    c5_c_a = c5_b_a + 1;
    c5_ixstart = c5_c_a;
    c5_ix = c5_ixstart;
    c5_s = c5_a[_SFD_EML_ARRAY_BOUNDS_CHECK("", c5_ix, 1, 18, 1, 0) - 1];
    for (c5_b_k = 2; c5_b_k < 10; c5_b_k++) {
      c5_d_a = c5_ix;
      c5_e_a = c5_d_a + 2;
      c5_ix = c5_e_a;
      c5_s += c5_a[_SFD_EML_ARRAY_BOUNDS_CHECK("", c5_ix, 1, 18, 1, 0) - 1];
    }

    c5_f_a = c5_iy;
    c5_g_a = c5_f_a + 1;
    c5_iy = c5_g_a;
    c5_SumOfObstacleVector_Word[_SFD_EML_ARRAY_BOUNDS_CHECK("", c5_iy, 1, 2, 1,
      0) - 1] = c5_s;
  }

  _SFD_EML_CALL(0U, chartInstance->c5_sfEvent, 110);
  c5_y = c5_SumOfObstacleVector_Word[1];
  c5_x = c5_SumOfObstacleVector_Word[0];
  c5_b_y = c5_y;
  c5_b_x = c5_x;
  c5_NextAngle = muDoubleScalarAtan2(c5_b_y, c5_b_x);
  _SFD_EML_CALL(0U, chartInstance->c5_sfEvent, 112);
  c5_k = 0.1;
  _SFD_EML_CALL(0U, chartInstance->c5_sfEvent, 113);
  c5_c_x = c5_NextAngle;
  c5_d_x = c5_c_x;
  c5_d_x = muDoubleScalarCos(c5_d_x);
  c5_e_x = c5_NextAngle;
  c5_f_x = c5_e_x;
  c5_f_x = muDoubleScalarSin(c5_f_x);
  c5_b_b[0] = c5_d_x;
  c5_b_b[1] = c5_f_x;
  for (c5_i151 = 0; c5_i151 < 2; c5_i151++) {
    c5_ObstaclesSpeed[c5_i151] = 0.1 * c5_b_b[c5_i151];
  }

  _SFD_EML_CALL(0U, chartInstance->c5_sfEvent, -113);
  _SFD_SYMBOL_SCOPE_POP();
}

static void c5_repmat(SFc5_BlueBird_FormationInstanceStruct *chartInstance,
                      real_T c5_a[2], real_T c5_b[18])
{
  int32_T c5_jtilecol;
  int32_T c5_b_jtilecol;
  int32_T c5_ibtile;
  int32_T c5_ibmat;
  int32_T c5_ibcol;
  int32_T c5_k;
  int32_T c5_b_k;
  (void)chartInstance;
  for (c5_jtilecol = 1; c5_jtilecol < 10; c5_jtilecol++) {
    c5_b_jtilecol = c5_jtilecol - 1;
    c5_ibtile = c5_b_jtilecol << 1;
    c5_ibmat = c5_ibtile;
    c5_ibcol = c5_ibmat;
    for (c5_k = 1; c5_k < 3; c5_k++) {
      c5_b_k = c5_k - 1;
      c5_b[c5_ibcol + c5_b_k] = c5_a[c5_b_k];
    }
  }
}

static void c5_c_eml_scalar_eg(SFc5_BlueBird_FormationInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static const mxArray *c5_m_sf_marshallOut(void *chartInstanceVoid, void
  *c5_inData)
{
  const mxArray *c5_mxArrayOutData = NULL;
  int32_T c5_u;
  const mxArray *c5_y = NULL;
  SFc5_BlueBird_FormationInstanceStruct *chartInstance;
  chartInstance = (SFc5_BlueBird_FormationInstanceStruct *)chartInstanceVoid;
  c5_mxArrayOutData = NULL;
  c5_u = *(int32_T *)c5_inData;
  c5_y = NULL;
  sf_mex_assign(&c5_y, sf_mex_create("y", &c5_u, 6, 0U, 0U, 0U, 0), false);
  sf_mex_assign(&c5_mxArrayOutData, c5_y, false);
  return c5_mxArrayOutData;
}

static int32_T c5_n_emlrt_marshallIn(SFc5_BlueBird_FormationInstanceStruct
  *chartInstance, const mxArray *c5_u, const emlrtMsgIdentifier *c5_parentId)
{
  int32_T c5_y;
  int32_T c5_i152;
  (void)chartInstance;
  sf_mex_import(c5_parentId, sf_mex_dup(c5_u), &c5_i152, 1, 6, 0U, 0, 0U, 0);
  c5_y = c5_i152;
  sf_mex_destroy(&c5_u);
  return c5_y;
}

static void c5_l_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c5_mxArrayInData, const char_T *c5_varName, void *c5_outData)
{
  const mxArray *c5_b_sfEvent;
  const char_T *c5_identifier;
  emlrtMsgIdentifier c5_thisId;
  int32_T c5_y;
  SFc5_BlueBird_FormationInstanceStruct *chartInstance;
  chartInstance = (SFc5_BlueBird_FormationInstanceStruct *)chartInstanceVoid;
  c5_b_sfEvent = sf_mex_dup(c5_mxArrayInData);
  c5_identifier = c5_varName;
  c5_thisId.fIdentifier = c5_identifier;
  c5_thisId.fParent = NULL;
  c5_y = c5_n_emlrt_marshallIn(chartInstance, sf_mex_dup(c5_b_sfEvent),
    &c5_thisId);
  sf_mex_destroy(&c5_b_sfEvent);
  *(int32_T *)c5_outData = c5_y;
  sf_mex_destroy(&c5_mxArrayInData);
}

static uint8_T c5_o_emlrt_marshallIn(SFc5_BlueBird_FormationInstanceStruct
  *chartInstance, const mxArray *c5_b_is_active_c5_BlueBird_Formation, const
  char_T *c5_identifier)
{
  uint8_T c5_y;
  emlrtMsgIdentifier c5_thisId;
  c5_thisId.fIdentifier = c5_identifier;
  c5_thisId.fParent = NULL;
  c5_y = c5_p_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c5_b_is_active_c5_BlueBird_Formation), &c5_thisId);
  sf_mex_destroy(&c5_b_is_active_c5_BlueBird_Formation);
  return c5_y;
}

static uint8_T c5_p_emlrt_marshallIn(SFc5_BlueBird_FormationInstanceStruct
  *chartInstance, const mxArray *c5_u, const emlrtMsgIdentifier *c5_parentId)
{
  uint8_T c5_y;
  uint8_T c5_u0;
  (void)chartInstance;
  sf_mex_import(c5_parentId, sf_mex_dup(c5_u), &c5_u0, 1, 3, 0U, 0, 0U, 0);
  c5_y = c5_u0;
  sf_mex_destroy(&c5_u);
  return c5_y;
}

static void init_dsm_address_info(SFc5_BlueBird_FormationInstanceStruct
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

void sf_c5_BlueBird_Formation_get_check_sum(mxArray *plhs[])
{
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(3044769010U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(3823907994U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(1580599182U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(2750406912U);
}

mxArray *sf_c5_BlueBird_Formation_get_autoinheritance_info(void)
{
  const char *autoinheritanceFields[] = { "checksum", "inputs", "parameters",
    "outputs", "locals" };

  mxArray *mxAutoinheritanceInfo = mxCreateStructMatrix(1,1,5,
    autoinheritanceFields);

  {
    mxArray *mxChecksum = mxCreateString("TWES86poMjLPrIjqudgrsH");
    mxSetField(mxAutoinheritanceInfo,0,"checksum",mxChecksum);
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,4,3,dataFields);

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
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

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,1,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,1,"type",mxType);
    }

    mxSetField(mxData,1,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(9);
      pr[1] = (double)(1);
      mxSetField(mxData,2,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,2,"type",mxType);
    }

    mxSetField(mxData,2,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(3);
      pr[1] = (double)(1);
      mxSetField(mxData,3,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,3,"type",mxType);
    }

    mxSetField(mxData,3,"complexity",mxCreateDoubleScalar(0));
    mxSetField(mxAutoinheritanceInfo,0,"inputs",mxData);
  }

  {
    mxSetField(mxAutoinheritanceInfo,0,"parameters",mxCreateDoubleMatrix(0,0,
                mxREAL));
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,2,3,dataFields);

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
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

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(1);
      pr[1] = (double)(1);
      mxSetField(mxData,1,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt" };

      mxArray *mxType = mxCreateStructMatrix(1,1,2,typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxData,1,"type",mxType);
    }

    mxSetField(mxData,1,"complexity",mxCreateDoubleScalar(0));
    mxSetField(mxAutoinheritanceInfo,0,"outputs",mxData);
  }

  {
    mxSetField(mxAutoinheritanceInfo,0,"locals",mxCreateDoubleMatrix(0,0,mxREAL));
  }

  return(mxAutoinheritanceInfo);
}

mxArray *sf_c5_BlueBird_Formation_third_party_uses_info(void)
{
  mxArray * mxcell3p = mxCreateCellMatrix(1,0);
  return(mxcell3p);
}

mxArray *sf_c5_BlueBird_Formation_updateBuildInfo_args_info(void)
{
  mxArray *mxBIArgs = mxCreateCellMatrix(1,0);
  return mxBIArgs;
}

static const mxArray *sf_get_sim_state_info_c5_BlueBird_Formation(void)
{
  const char *infoFields[] = { "chartChecksum", "varInfo" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 2, infoFields);
  const char *infoEncStr[] = {
    "100 S1x4'type','srcId','name','auxInfo'{{M[1],M[5],T\"V_x\",},{M[1],M[7],T\"V_y\",},{M[4],M[0],T\"Counter\",S'l','i','p'{{M1x2[93 100],M[0],}}},{M[8],M[0],T\"is_active_c5_BlueBird_Formation\",}}"
  };

  mxArray *mxVarInfo = sf_mex_decode_encoded_mx_struct_array(infoEncStr, 4, 10);
  mxArray *mxChecksum = mxCreateDoubleMatrix(1, 4, mxREAL);
  sf_c5_BlueBird_Formation_get_check_sum(&mxChecksum);
  mxSetField(mxInfo, 0, infoFields[0], mxChecksum);
  mxSetField(mxInfo, 0, infoFields[1], mxVarInfo);
  return mxInfo;
}

static void chart_debug_initialization(SimStruct *S, unsigned int
  fullDebuggerInitialization)
{
  if (!sim_mode_is_rtw_gen(S)) {
    SFc5_BlueBird_FormationInstanceStruct *chartInstance;
    ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
    ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
    chartInstance = (SFc5_BlueBird_FormationInstanceStruct *)
      chartInfo->chartInstance;
    if (ssIsFirstInitCond(S) && fullDebuggerInitialization==1) {
      /* do this only if simulation is starting */
      {
        unsigned int chartAlreadyPresent;
        chartAlreadyPresent = sf_debug_initialize_chart
          (sfGlobalDebugInstanceStruct,
           _BlueBird_FormationMachineNumber_,
           5,
           1,
           1,
           0,
           6,
           0,
           0,
           0,
           0,
           3,
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
          _SFD_SET_DATA_PROPS(0,1,1,0,"V_x_In");
          _SFD_SET_DATA_PROPS(1,2,0,1,"V_x");
          _SFD_SET_DATA_PROPS(2,1,1,0,"V_y_In");
          _SFD_SET_DATA_PROPS(3,2,0,1,"V_y");
          _SFD_SET_DATA_PROPS(4,1,1,0,"ir_raw");
          _SFD_SET_DATA_PROPS(5,1,1,0,"Position");
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
        _SFD_CV_INIT_EML(0,1,5,4,0,0,0,1,0,0,0);
        _SFD_CV_INIT_EML_FCN(0,0,"eML_blk_kernel",0,-1,1652);
        _SFD_CV_INIT_EML_FCN(0,1,"IR_Raw_to_Distances",1654,-1,2366);
        _SFD_CV_INIT_EML_FCN(0,2,"GetIR_Position",2368,-1,2971);
        _SFD_CV_INIT_EML_FCN(0,3,"Transformation_Matrix",2973,-1,3182);
        _SFD_CV_INIT_EML_FCN(0,4,"AvoidObstacles_NextPoint",3184,-1,3914);
        _SFD_CV_INIT_EML_IF(0,1,0,101,120,-1,141);
        _SFD_CV_INIT_EML_IF(0,1,1,258,282,-1,337);
        _SFD_CV_INIT_EML_IF(0,1,2,347,362,423,461);
        _SFD_CV_INIT_EML_IF(0,1,3,471,490,1591,1647);
        _SFD_CV_INIT_EML_FOR(0,1,0,866,880,1235);
        _SFD_CV_INIT_SCRIPT(0,6,0,0,0,0,0,0,0,0);
        _SFD_CV_INIT_SCRIPT_FCN(0,3,"Pose2D_get_transformation_matrix",844,-1,
          1104);
        _SFD_CV_INIT_SCRIPT_FCN(0,2,"Pose2D_unpack",701,-1,826);
        _SFD_CV_INIT_SCRIPT_FCN(0,1,"Pose2D_set_pose",409,-1,683);
        _SFD_CV_INIT_SCRIPT_FCN(0,0,"Pose2D_Pose2D",271,-1,391);
        _SFD_CV_INIT_SCRIPT_FCN(0,5,"Pose2D_rad2deg",1234,-1,1303);
        _SFD_CV_INIT_SCRIPT_FCN(0,4,"Pose2D_deg2rad",1147,-1,1216);
        _SFD_CV_INIT_SCRIPT(1,2,0,0,0,0,0,0,0,0);
        _SFD_CV_INIT_SCRIPT_FCN(1,0,"Controller_Controller",834,-1,1177);
        _SFD_CV_INIT_SCRIPT_FCN(1,1,"Controller_execute",1220,-1,1763);
        _SFD_CV_INIT_SCRIPT(2,5,1,0,0,0,2,0,1,1);
        _SFD_CV_INIT_SCRIPT_FCN(2,4,"AvoidObstacles_get_transformation_matrix",
          4472,-1,4616);
        _SFD_CV_INIT_SCRIPT_FCN(2,3,"AvoidObstacles_set_sensor_geometry",4130,-1,
          4454);
        _SFD_CV_INIT_SCRIPT_FCN(2,2,"AvoidObstacles_apply_sensor_geometry",3177,
          -1,4112);
        _SFD_CV_INIT_SCRIPT_FCN(2,1,"AvoidObstacles_execute",964,-1,3123);
        _SFD_CV_INIT_SCRIPT_FCN(2,0,"AvoidObstacles_AvoidObstacles",571,-1,946);
        _SFD_CV_INIT_SCRIPT_IF(2,0,1107,1126,-1,1226);
        _SFD_CV_INIT_SCRIPT_FOR(2,1,4230,4240,4383);
        _SFD_CV_INIT_SCRIPT_FOR(2,0,3454,3464,3781);

        {
          static int condStart[] = { 1111 };

          static int condEnd[] = { 1125 };

          static int pfixExpr[] = { 0, -1 };

          _SFD_CV_INIT_SCRIPT_MCDC(2,0,1110,1125,1,0,&(condStart[0]),&(condEnd[0]),
            2,&(pfixExpr[0]));
        }

        _SFD_SET_DATA_COMPILED_PROPS(0,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c5_b_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(1,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c5_b_sf_marshallOut,(MexInFcnForType)c5_b_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(2,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c5_b_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(3,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c5_b_sf_marshallOut,(MexInFcnForType)c5_b_sf_marshallIn);

        {
          unsigned int dimVector[1];
          dimVector[0]= 9;
          _SFD_SET_DATA_COMPILED_PROPS(4,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c5_d_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 3;
          _SFD_SET_DATA_COMPILED_PROPS(5,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c5_c_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          real_T *c5_V_x_In;
          real_T *c5_V_x;
          real_T *c5_V_y_In;
          real_T *c5_V_y;
          real_T (*c5_ir_raw)[9];
          real_T (*c5_Position)[3];
          c5_Position = (real_T (*)[3])ssGetInputPortSignal(chartInstance->S, 3);
          c5_ir_raw = (real_T (*)[9])ssGetInputPortSignal(chartInstance->S, 2);
          c5_V_y = (real_T *)ssGetOutputPortSignal(chartInstance->S, 2);
          c5_V_y_In = (real_T *)ssGetInputPortSignal(chartInstance->S, 1);
          c5_V_x = (real_T *)ssGetOutputPortSignal(chartInstance->S, 1);
          c5_V_x_In = (real_T *)ssGetInputPortSignal(chartInstance->S, 0);
          _SFD_SET_DATA_VALUE_PTR(0U, c5_V_x_In);
          _SFD_SET_DATA_VALUE_PTR(1U, c5_V_x);
          _SFD_SET_DATA_VALUE_PTR(2U, c5_V_y_In);
          _SFD_SET_DATA_VALUE_PTR(3U, c5_V_y);
          _SFD_SET_DATA_VALUE_PTR(4U, *c5_ir_raw);
          _SFD_SET_DATA_VALUE_PTR(5U, *c5_Position);
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
  return "lvV63jj6ueV5v5JRCqcDhD";
}

static void sf_opaque_initialize_c5_BlueBird_Formation(void *chartInstanceVar)
{
  chart_debug_initialization(((SFc5_BlueBird_FormationInstanceStruct*)
    chartInstanceVar)->S,0);
  initialize_params_c5_BlueBird_Formation((SFc5_BlueBird_FormationInstanceStruct*)
    chartInstanceVar);
  initialize_c5_BlueBird_Formation((SFc5_BlueBird_FormationInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_enable_c5_BlueBird_Formation(void *chartInstanceVar)
{
  enable_c5_BlueBird_Formation((SFc5_BlueBird_FormationInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_disable_c5_BlueBird_Formation(void *chartInstanceVar)
{
  disable_c5_BlueBird_Formation((SFc5_BlueBird_FormationInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_gateway_c5_BlueBird_Formation(void *chartInstanceVar)
{
  sf_gateway_c5_BlueBird_Formation((SFc5_BlueBird_FormationInstanceStruct*)
    chartInstanceVar);
}

extern const mxArray* sf_internal_get_sim_state_c5_BlueBird_Formation(SimStruct*
  S)
{
  ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
  ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_raw2high");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = (mxArray*) get_sim_state_c5_BlueBird_Formation
    ((SFc5_BlueBird_FormationInstanceStruct*)chartInfo->chartInstance);/* raw sim ctx */
  prhs[3] = (mxArray*) sf_get_sim_state_info_c5_BlueBird_Formation();/* state var info */
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

extern void sf_internal_set_sim_state_c5_BlueBird_Formation(SimStruct* S, const
  mxArray *st)
{
  ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
  ChartInfoStruct * chartInfo = (ChartInfoStruct *)(crtInfo->instanceInfo);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[3];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_high2raw");
  prhs[1] = mxDuplicateArray(st);      /* high level simctx */
  prhs[2] = (mxArray*) sf_get_sim_state_info_c5_BlueBird_Formation();/* state var info */
  mxError = sf_mex_call_matlab(1, plhs, 3, prhs, "sfprivate");
  mxDestroyArray(prhs[0]);
  mxDestroyArray(prhs[1]);
  mxDestroyArray(prhs[2]);
  if (mxError || plhs[0] == NULL) {
    sf_mex_error_message("Stateflow Internal Error: \nError calling 'chart_simctx_high2raw'.\n");
  }

  set_sim_state_c5_BlueBird_Formation((SFc5_BlueBird_FormationInstanceStruct*)
    chartInfo->chartInstance, mxDuplicateArray(plhs[0]));
  mxDestroyArray(plhs[0]);
}

static const mxArray* sf_opaque_get_sim_state_c5_BlueBird_Formation(SimStruct* S)
{
  return sf_internal_get_sim_state_c5_BlueBird_Formation(S);
}

static void sf_opaque_set_sim_state_c5_BlueBird_Formation(SimStruct* S, const
  mxArray *st)
{
  sf_internal_set_sim_state_c5_BlueBird_Formation(S, st);
}

static void sf_opaque_terminate_c5_BlueBird_Formation(void *chartInstanceVar)
{
  if (chartInstanceVar!=NULL) {
    SimStruct *S = ((SFc5_BlueBird_FormationInstanceStruct*) chartInstanceVar)
      ->S;
    ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)(ssGetUserData(S));
    if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
      sf_clear_rtw_identifier(S);
      unload_BlueBird_Formation_optimization_info();
    }

    finalize_c5_BlueBird_Formation((SFc5_BlueBird_FormationInstanceStruct*)
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
  initSimStructsc5_BlueBird_Formation((SFc5_BlueBird_FormationInstanceStruct*)
    chartInstanceVar);
}

extern unsigned int sf_machine_global_initializer_called(void);
static void mdlProcessParameters_c5_BlueBird_Formation(SimStruct *S)
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
    initialize_params_c5_BlueBird_Formation
      ((SFc5_BlueBird_FormationInstanceStruct*)(chartInfo->chartInstance));
  }
}

static void mdlSetWorkWidths_c5_BlueBird_Formation(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
    mxArray *infoStruct = load_BlueBird_Formation_optimization_info();
    int_T chartIsInlinable =
      (int_T)sf_is_chart_inlinable(sf_get_instance_specialization(),infoStruct,5);
    ssSetStateflowIsInlinable(S,chartIsInlinable);
    ssSetRTWCG(S,sf_rtw_info_uint_prop(sf_get_instance_specialization(),
                infoStruct,5,"RTWCG"));
    ssSetEnableFcnIsTrivial(S,1);
    ssSetDisableFcnIsTrivial(S,1);
    ssSetNotMultipleInlinable(S,sf_rtw_info_uint_prop
      (sf_get_instance_specialization(),infoStruct,5,
       "gatewayCannotBeInlinedMultipleTimes"));
    sf_update_buildInfo(sf_get_instance_specialization(),infoStruct,5);
    if (chartIsInlinable) {
      ssSetInputPortOptimOpts(S, 0, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 1, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 2, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 3, SS_REUSABLE_AND_LOCAL);
      sf_mark_chart_expressionable_inputs(S,sf_get_instance_specialization(),
        infoStruct,5,4);
      sf_mark_chart_reusable_outputs(S,sf_get_instance_specialization(),
        infoStruct,5,2);
    }

    {
      unsigned int outPortIdx;
      for (outPortIdx=1; outPortIdx<=2; ++outPortIdx) {
        ssSetOutputPortOptimizeInIR(S, outPortIdx, 1U);
      }
    }

    {
      unsigned int inPortIdx;
      for (inPortIdx=0; inPortIdx < 4; ++inPortIdx) {
        ssSetInputPortOptimizeInIR(S, inPortIdx, 1U);
      }
    }

    sf_set_rtw_dwork_info(S,sf_get_instance_specialization(),infoStruct,5);
    ssSetHasSubFunctions(S,!(chartIsInlinable));
  } else {
  }

  ssSetOptions(S,ssGetOptions(S)|SS_OPTION_WORKS_WITH_CODE_REUSE);
  ssSetChecksum0(S,(1121090160U));
  ssSetChecksum1(S,(1498478436U));
  ssSetChecksum2(S,(949046986U));
  ssSetChecksum3(S,(3973230189U));
  ssSetmdlDerivatives(S, NULL);
  ssSetExplicitFCSSCtrl(S,1);
  ssSupportsMultipleExecInstances(S,1);
}

static void mdlRTW_c5_BlueBird_Formation(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S)) {
    ssWriteRTWStrParam(S, "StateflowChartType", "Embedded MATLAB");
  }
}

static void mdlStart_c5_BlueBird_Formation(SimStruct *S)
{
  SFc5_BlueBird_FormationInstanceStruct *chartInstance;
  ChartRunTimeInfo * crtInfo = (ChartRunTimeInfo *)utMalloc(sizeof
    (ChartRunTimeInfo));
  chartInstance = (SFc5_BlueBird_FormationInstanceStruct *)utMalloc(sizeof
    (SFc5_BlueBird_FormationInstanceStruct));
  memset(chartInstance, 0, sizeof(SFc5_BlueBird_FormationInstanceStruct));
  if (chartInstance==NULL) {
    sf_mex_error_message("Could not allocate memory for chart instance.");
  }

  chartInstance->chartInfo.chartInstance = chartInstance;
  chartInstance->chartInfo.isEMLChart = 1;
  chartInstance->chartInfo.chartInitialized = 0;
  chartInstance->chartInfo.sFunctionGateway =
    sf_opaque_gateway_c5_BlueBird_Formation;
  chartInstance->chartInfo.initializeChart =
    sf_opaque_initialize_c5_BlueBird_Formation;
  chartInstance->chartInfo.terminateChart =
    sf_opaque_terminate_c5_BlueBird_Formation;
  chartInstance->chartInfo.enableChart = sf_opaque_enable_c5_BlueBird_Formation;
  chartInstance->chartInfo.disableChart =
    sf_opaque_disable_c5_BlueBird_Formation;
  chartInstance->chartInfo.getSimState =
    sf_opaque_get_sim_state_c5_BlueBird_Formation;
  chartInstance->chartInfo.setSimState =
    sf_opaque_set_sim_state_c5_BlueBird_Formation;
  chartInstance->chartInfo.getSimStateInfo =
    sf_get_sim_state_info_c5_BlueBird_Formation;
  chartInstance->chartInfo.zeroCrossings = NULL;
  chartInstance->chartInfo.outputs = NULL;
  chartInstance->chartInfo.derivatives = NULL;
  chartInstance->chartInfo.mdlRTW = mdlRTW_c5_BlueBird_Formation;
  chartInstance->chartInfo.mdlStart = mdlStart_c5_BlueBird_Formation;
  chartInstance->chartInfo.mdlSetWorkWidths =
    mdlSetWorkWidths_c5_BlueBird_Formation;
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

void c5_BlueBird_Formation_method_dispatcher(SimStruct *S, int_T method, void
  *data)
{
  switch (method) {
   case SS_CALL_MDL_START:
    mdlStart_c5_BlueBird_Formation(S);
    break;

   case SS_CALL_MDL_SET_WORK_WIDTHS:
    mdlSetWorkWidths_c5_BlueBird_Formation(S);
    break;

   case SS_CALL_MDL_PROCESS_PARAMETERS:
    mdlProcessParameters_c5_BlueBird_Formation(S);
    break;

   default:
    /* Unhandled method */
    sf_mex_error_message("Stateflow Internal Error:\n"
                         "Error calling c5_BlueBird_Formation_method_dispatcher.\n"
                         "Can't handle method %d.\n", method);
    break;
  }
}
