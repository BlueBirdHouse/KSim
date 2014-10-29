/* Include files */

#include <stddef.h>
#include "blas.h"
#include "simiam_AvoidObstacle_BestAO_sfun.h"
#include "c2_simiam_AvoidObstacle_BestAO.h"
#include "mwmathutil.h"
#define CHARTINSTANCE_CHARTNUMBER      (chartInstance->chartNumber)
#define CHARTINSTANCE_INSTANCENUMBER   (chartInstance->instanceNumber)
#include "simiam_AvoidObstacle_BestAO_sfun_debug_macros.h"
#define _SF_MEX_LISTEN_FOR_CTRL_C(S)   sf_mex_listen_for_ctrl_c(sfGlobalDebugInstanceStruct,S);

/* Type Definitions */

/* Named Constants */
#define CALL_EVENT                     (-1)

/* Variable Declarations */

/* Variable Definitions */
static const char * c2_debug_family_names[17] = { "ir_distances",
  "ObstacleDetected", "FixIR_9_Number", "Obstacle_AroundRobot", "i",
  "IR_Position_I", "IR_Transformation_Matrix_I", "Obstacle_AroundRobot_I",
  "Obstacle_AroundRobot_World", "ObstacleDetected_Number", "nargin", "nargout",
  "ir_raw", "Position", "FixAngle", "NextPoint", "OutData" };

static const char * c2_b_debug_family_names[9] = { "SafeNumber", "MaxNumber",
  "ir_raw_Internal", "Number_List", "nargin", "nargout", "ir_raw", "Answer",
  "ObstacleDetected" };

static const char * c2_c_debug_family_names[5] = { "nargin", "nargout", "x", "y",
  "theta" };

static const char * c2_d_debug_family_names[4] = { "nargin", "nargout", "deg",
  "rad" };

static const char * c2_e_debug_family_names[5] = { "IR_Position", "nargin",
  "nargout", "IR_Number", "Answer" };

static const char * c2_f_debug_family_names[3] = { "type", "nargin", "nargout" };

static const char * c2_g_debug_family_names[2] = { "nargin", "nargout" };

static const char * c2_h_debug_family_names[6] = { "nargin", "nargout", "x", "y",
  "theta", "R" };

static const char * c2_i_debug_family_names[6] = { "nargin", "nargout", "x", "y",
  "theta", "Answer" };

static const char * c2_j_debug_family_names[5] = { "IR_Position", "nargin",
  "nargout", "IR_Number", "Answer" };

static const char * c2_k_debug_family_names[27] = { "sensor_gains",
  "ir_distances_rf", "u_i", "u_i_Norm", "u_i_Norm_Fixed", "u_i_sort_Number",
  "u_i_Smallest", "u_i_SecondSmall", "P1", "P2", "Pointer_Wall", "Printer_Robot",
  "Robot_direction", "NextAngle", "ChickAngle", "AngleList", "ChickIndex",
  "d_Chick", "NextPoint_X", "NextPoint_Y", "nargin", "nargout",
  "Obstacle_AroundRobot_World", "Position", "ObstacleDetected", "NextPoint",
  "OutData" };

static const char * c2_l_debug_family_names[12] = { "sensor_gains",
  "ir_distances_rf", "u_i", "NextAngle", "d_List", "NextPoint_X", "NextPoint_Y",
  "nargin", "nargout", "Obstacle_AroundRobot_World", "Position", "NextPoint" };

/* Function Declarations */
static void initialize_c2_simiam_AvoidObstacle_BestAO
  (SFc2_simiam_AvoidObstacle_BestAOInstanceStruct *chartInstance);
static void initialize_params_c2_simiam_AvoidObstacle_BestAO
  (SFc2_simiam_AvoidObstacle_BestAOInstanceStruct *chartInstance);
static void enable_c2_simiam_AvoidObstacle_BestAO
  (SFc2_simiam_AvoidObstacle_BestAOInstanceStruct *chartInstance);
static void disable_c2_simiam_AvoidObstacle_BestAO
  (SFc2_simiam_AvoidObstacle_BestAOInstanceStruct *chartInstance);
static void c2_update_debugger_state_c2_simiam_AvoidObstacle_BestAO
  (SFc2_simiam_AvoidObstacle_BestAOInstanceStruct *chartInstance);
static const mxArray *get_sim_state_c2_simiam_AvoidObstacle_BestAO
  (SFc2_simiam_AvoidObstacle_BestAOInstanceStruct *chartInstance);
static void set_sim_state_c2_simiam_AvoidObstacle_BestAO
  (SFc2_simiam_AvoidObstacle_BestAOInstanceStruct *chartInstance, const mxArray *
   c2_st);
static void finalize_c2_simiam_AvoidObstacle_BestAO
  (SFc2_simiam_AvoidObstacle_BestAOInstanceStruct *chartInstance);
static void sf_c2_simiam_AvoidObstacle_BestAO
  (SFc2_simiam_AvoidObstacle_BestAOInstanceStruct *chartInstance);
static void c2_chartstep_c2_simiam_AvoidObstacle_BestAO
  (SFc2_simiam_AvoidObstacle_BestAOInstanceStruct *chartInstance);
static void initSimStructsc2_simiam_AvoidObstacle_BestAO
  (SFc2_simiam_AvoidObstacle_BestAOInstanceStruct *chartInstance);
static void init_script_number_translation(uint32_T c2_machineNumber, uint32_T
  c2_chartNumber);
static const mxArray *c2_sf_marshallOut(void *chartInstanceVoid, void *c2_inData);
static real_T c2_emlrt_marshallIn(SFc2_simiam_AvoidObstacle_BestAOInstanceStruct
  *chartInstance, const mxArray *c2_OutData, const char_T *c2_identifier);
static real_T c2_b_emlrt_marshallIn
  (SFc2_simiam_AvoidObstacle_BestAOInstanceStruct *chartInstance, const mxArray *
   c2_u, const emlrtMsgIdentifier *c2_parentId);
static void c2_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_b_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static void c2_c_emlrt_marshallIn(SFc2_simiam_AvoidObstacle_BestAOInstanceStruct
  *chartInstance, const mxArray *c2_NextPoint, const char_T *c2_identifier,
  real_T c2_y[2]);
static void c2_d_emlrt_marshallIn(SFc2_simiam_AvoidObstacle_BestAOInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  real_T c2_y[2]);
static void c2_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_c_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static const mxArray *c2_d_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static const mxArray *c2_e_sf_marshallOut(void *chartInstanceVoid, boolean_T
  c2_inData_data[9], int32_T c2_inData_sizes[1]);
static void c2_e_emlrt_marshallIn(SFc2_simiam_AvoidObstacle_BestAOInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  boolean_T c2_y_data[9], int32_T c2_y_sizes[1]);
static void c2_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, boolean_T c2_outData_data[9],
  int32_T c2_outData_sizes[1]);
static const mxArray *c2_f_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static void c2_f_emlrt_marshallIn(SFc2_simiam_AvoidObstacle_BestAOInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  real_T c2_y[27]);
static void c2_d_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static void c2_g_emlrt_marshallIn(SFc2_simiam_AvoidObstacle_BestAOInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  real_T c2_y[3]);
static void c2_e_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_g_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static void c2_h_emlrt_marshallIn(SFc2_simiam_AvoidObstacle_BestAOInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  real_T c2_y[9]);
static void c2_f_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_h_sf_marshallOut(void *chartInstanceVoid, real_T
  c2_inData_data[9], int32_T c2_inData_sizes[1]);
static void c2_i_emlrt_marshallIn(SFc2_simiam_AvoidObstacle_BestAOInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  real_T c2_y_data[9], int32_T c2_y_sizes[1]);
static void c2_g_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, real_T c2_outData_data[9],
  int32_T c2_outData_sizes[1]);
static void c2_j_emlrt_marshallIn(SFc2_simiam_AvoidObstacle_BestAOInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  real_T c2_y[9]);
static void c2_h_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_i_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static void c2_k_emlrt_marshallIn(SFc2_simiam_AvoidObstacle_BestAOInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  boolean_T c2_y[9]);
static void c2_i_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_j_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static void c2_l_emlrt_marshallIn(SFc2_simiam_AvoidObstacle_BestAOInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  real_T c2_y[27]);
static void c2_j_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_k_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static void c2_m_emlrt_marshallIn(SFc2_simiam_AvoidObstacle_BestAOInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  char_T c2_y[15]);
static void c2_k_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_l_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static void c2_n_emlrt_marshallIn(SFc2_simiam_AvoidObstacle_BestAOInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  real_T c2_y[9]);
static void c2_l_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_m_sf_marshallOut(void *chartInstanceVoid, real_T
  c2_inData_data[9], int32_T c2_inData_sizes[2]);
static void c2_o_emlrt_marshallIn(SFc2_simiam_AvoidObstacle_BestAOInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  real_T c2_y_data[9], int32_T c2_y_sizes[2]);
static void c2_m_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, real_T c2_outData_data[9],
  int32_T c2_outData_sizes[2]);
static const mxArray *c2_n_sf_marshallOut(void *chartInstanceVoid, real_T
  c2_inData_data[36], int32_T c2_inData_sizes[2]);
static void c2_p_emlrt_marshallIn(SFc2_simiam_AvoidObstacle_BestAOInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  real_T c2_y_data[36], int32_T c2_y_sizes[2]);
static void c2_n_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, real_T c2_outData_data[36],
  int32_T c2_outData_sizes[2]);
static const mxArray *c2_o_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static void c2_q_emlrt_marshallIn(SFc2_simiam_AvoidObstacle_BestAOInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  real_T c2_y[36]);
static void c2_o_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_p_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static void c2_r_emlrt_marshallIn(SFc2_simiam_AvoidObstacle_BestAOInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  real_T c2_y[4]);
static void c2_p_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_q_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static void c2_s_emlrt_marshallIn(SFc2_simiam_AvoidObstacle_BestAOInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  real_T c2_y[18]);
static void c2_q_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static void c2_info_helper(const mxArray **c2_info);
static const mxArray *c2_emlrt_marshallOut(char * c2_u);
static const mxArray *c2_b_emlrt_marshallOut(uint32_T c2_u);
static void c2_b_info_helper(const mxArray **c2_info);
static void c2_c_info_helper(const mxArray **c2_info);
static void c2_IR_Raw_to_Distances
  (SFc2_simiam_AvoidObstacle_BestAOInstanceStruct *chartInstance, real_T
   c2_ir_raw[9], real_T c2_Answer[9], real_T c2_ObstacleDetected_data[9],
   int32_T c2_ObstacleDetected_sizes[1]);
static void c2_eml_li_find(SFc2_simiam_AvoidObstacle_BestAOInstanceStruct
  *chartInstance, boolean_T c2_x[9], int32_T c2_y_data[9], int32_T c2_y_sizes[1]);
static void c2_eml_error(SFc2_simiam_AvoidObstacle_BestAOInstanceStruct
  *chartInstance);
static void c2_GetIR_Position(SFc2_simiam_AvoidObstacle_BestAOInstanceStruct
  *chartInstance, real_T c2_IR_Number, real_T c2_Answer[3]);
static void c2_Transformation_Matrix
  (SFc2_simiam_AvoidObstacle_BestAOInstanceStruct *chartInstance, real_T c2_x,
   real_T c2_y, real_T c2_theta, real_T c2_Answer[9]);
static void c2_eml_scalar_eg(SFc2_simiam_AvoidObstacle_BestAOInstanceStruct
  *chartInstance);
static void c2_b_eml_scalar_eg(SFc2_simiam_AvoidObstacle_BestAOInstanceStruct
  *chartInstance);
static void c2_AvoidObstacles_NextPoint_BestAO
  (SFc2_simiam_AvoidObstacle_BestAOInstanceStruct *chartInstance, real_T
   c2_Obstacle_AroundRobot_World[27], real_T c2_Position[3], real_T
   c2_ObstacleDetected_data[9], int32_T c2_ObstacleDetected_sizes[1], real_T
   c2_NextPoint[2], real_T *c2_OutData);
static void c2_repmat(SFc2_simiam_AvoidObstacle_BestAOInstanceStruct
                      *chartInstance, real_T c2_a[2], real_T c2_b[18]);
static void c2_c_eml_scalar_eg(SFc2_simiam_AvoidObstacle_BestAOInstanceStruct
  *chartInstance);
static void c2_power(SFc2_simiam_AvoidObstacle_BestAOInstanceStruct
                     *chartInstance, real_T c2_a[9], real_T c2_y[9]);
static void c2_d_eml_scalar_eg(SFc2_simiam_AvoidObstacle_BestAOInstanceStruct
  *chartInstance);
static void c2_sqrt(SFc2_simiam_AvoidObstacle_BestAOInstanceStruct
                    *chartInstance, real_T c2_x[9], real_T c2_b_x[9]);
static void c2_b_eml_error(SFc2_simiam_AvoidObstacle_BestAOInstanceStruct
  *chartInstance);
static real_T c2_abs(SFc2_simiam_AvoidObstacle_BestAOInstanceStruct
                     *chartInstance, real_T c2_x);
static void c2_eml_sort(SFc2_simiam_AvoidObstacle_BestAOInstanceStruct
  *chartInstance, real_T c2_x_data[9], int32_T c2_x_sizes[2], real_T c2_y_data[9],
  int32_T c2_y_sizes[2], int32_T c2_idx_data[9], int32_T c2_idx_sizes[2]);
static void c2_eml_sort_idx(SFc2_simiam_AvoidObstacle_BestAOInstanceStruct
  *chartInstance, real_T c2_x_data[9], int32_T c2_x_sizes[1], int32_T
  c2_idx_data[9], int32_T c2_idx_sizes[1]);
static void c2_e_eml_scalar_eg(SFc2_simiam_AvoidObstacle_BestAOInstanceStruct
  *chartInstance);
static real_T c2_sign(SFc2_simiam_AvoidObstacle_BestAOInstanceStruct
                      *chartInstance, real_T c2_x);
static real_T c2_atan2(SFc2_simiam_AvoidObstacle_BestAOInstanceStruct
  *chartInstance, real_T c2_y, real_T c2_x);
static void c2_AirLine_NextPoint(SFc2_simiam_AvoidObstacle_BestAOInstanceStruct *
  chartInstance, real_T c2_Obstacle_AroundRobot_World[27], real_T c2_Position[3],
  real_T c2_NextPoint[2]);
static real_T c2_mpower(SFc2_simiam_AvoidObstacle_BestAOInstanceStruct
  *chartInstance, real_T c2_a);
static const mxArray *c2_r_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static int32_T c2_t_emlrt_marshallIn
  (SFc2_simiam_AvoidObstacle_BestAOInstanceStruct *chartInstance, const mxArray *
   c2_u, const emlrtMsgIdentifier *c2_parentId);
static void c2_r_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static uint8_T c2_u_emlrt_marshallIn
  (SFc2_simiam_AvoidObstacle_BestAOInstanceStruct *chartInstance, const mxArray *
   c2_b_is_active_c2_simiam_AvoidObstacle_BestAO, const char_T *c2_identifier);
static uint8_T c2_v_emlrt_marshallIn
  (SFc2_simiam_AvoidObstacle_BestAOInstanceStruct *chartInstance, const mxArray *
   c2_u, const emlrtMsgIdentifier *c2_parentId);
static void c2_b_sqrt(SFc2_simiam_AvoidObstacle_BestAOInstanceStruct
                      *chartInstance, real_T c2_x[9]);
static void c2_b_sign(SFc2_simiam_AvoidObstacle_BestAOInstanceStruct
                      *chartInstance, real_T *c2_x);
static void init_dsm_address_info(SFc2_simiam_AvoidObstacle_BestAOInstanceStruct
  *chartInstance);

/* Function Definitions */
static void initialize_c2_simiam_AvoidObstacle_BestAO
  (SFc2_simiam_AvoidObstacle_BestAOInstanceStruct *chartInstance)
{
  chartInstance->c2_sfEvent = CALL_EVENT;
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
  chartInstance->c2_is_active_c2_simiam_AvoidObstacle_BestAO = 0U;
}

static void initialize_params_c2_simiam_AvoidObstacle_BestAO
  (SFc2_simiam_AvoidObstacle_BestAOInstanceStruct *chartInstance)
{
}

static void enable_c2_simiam_AvoidObstacle_BestAO
  (SFc2_simiam_AvoidObstacle_BestAOInstanceStruct *chartInstance)
{
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
}

static void disable_c2_simiam_AvoidObstacle_BestAO
  (SFc2_simiam_AvoidObstacle_BestAOInstanceStruct *chartInstance)
{
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
}

static void c2_update_debugger_state_c2_simiam_AvoidObstacle_BestAO
  (SFc2_simiam_AvoidObstacle_BestAOInstanceStruct *chartInstance)
{
}

static const mxArray *get_sim_state_c2_simiam_AvoidObstacle_BestAO
  (SFc2_simiam_AvoidObstacle_BestAOInstanceStruct *chartInstance)
{
  const mxArray *c2_st;
  const mxArray *c2_y = NULL;
  int32_T c2_i0;
  real_T c2_u[2];
  const mxArray *c2_b_y = NULL;
  real_T c2_hoistedGlobal;
  real_T c2_b_u;
  const mxArray *c2_c_y = NULL;
  uint8_T c2_b_hoistedGlobal;
  uint8_T c2_c_u;
  const mxArray *c2_d_y = NULL;
  real_T *c2_OutData;
  real_T (*c2_NextPoint)[2];
  c2_OutData = (real_T *)ssGetOutputPortSignal(chartInstance->S, 2);
  c2_NextPoint = (real_T (*)[2])ssGetOutputPortSignal(chartInstance->S, 1);
  c2_st = NULL;
  c2_st = NULL;
  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_createcellarray(3), FALSE);
  for (c2_i0 = 0; c2_i0 < 2; c2_i0++) {
    c2_u[c2_i0] = (*c2_NextPoint)[c2_i0];
  }

  c2_b_y = NULL;
  sf_mex_assign(&c2_b_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 1, 2), FALSE);
  sf_mex_setcell(c2_y, 0, c2_b_y);
  c2_hoistedGlobal = *c2_OutData;
  c2_b_u = c2_hoistedGlobal;
  c2_c_y = NULL;
  sf_mex_assign(&c2_c_y, sf_mex_create("y", &c2_b_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_setcell(c2_y, 1, c2_c_y);
  c2_b_hoistedGlobal =
    chartInstance->c2_is_active_c2_simiam_AvoidObstacle_BestAO;
  c2_c_u = c2_b_hoistedGlobal;
  c2_d_y = NULL;
  sf_mex_assign(&c2_d_y, sf_mex_create("y", &c2_c_u, 3, 0U, 0U, 0U, 0), FALSE);
  sf_mex_setcell(c2_y, 2, c2_d_y);
  sf_mex_assign(&c2_st, c2_y, FALSE);
  return c2_st;
}

static void set_sim_state_c2_simiam_AvoidObstacle_BestAO
  (SFc2_simiam_AvoidObstacle_BestAOInstanceStruct *chartInstance, const mxArray *
   c2_st)
{
  const mxArray *c2_u;
  real_T c2_dv0[2];
  int32_T c2_i1;
  real_T *c2_OutData;
  real_T (*c2_NextPoint)[2];
  c2_OutData = (real_T *)ssGetOutputPortSignal(chartInstance->S, 2);
  c2_NextPoint = (real_T (*)[2])ssGetOutputPortSignal(chartInstance->S, 1);
  chartInstance->c2_doneDoubleBufferReInit = TRUE;
  c2_u = sf_mex_dup(c2_st);
  c2_c_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c2_u, 0)),
                        "NextPoint", c2_dv0);
  for (c2_i1 = 0; c2_i1 < 2; c2_i1++) {
    (*c2_NextPoint)[c2_i1] = c2_dv0[c2_i1];
  }

  *c2_OutData = c2_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell
    (c2_u, 1)), "OutData");
  chartInstance->c2_is_active_c2_simiam_AvoidObstacle_BestAO =
    c2_u_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c2_u, 2)),
    "is_active_c2_simiam_AvoidObstacle_BestAO");
  sf_mex_destroy(&c2_u);
  c2_update_debugger_state_c2_simiam_AvoidObstacle_BestAO(chartInstance);
  sf_mex_destroy(&c2_st);
}

static void finalize_c2_simiam_AvoidObstacle_BestAO
  (SFc2_simiam_AvoidObstacle_BestAOInstanceStruct *chartInstance)
{
}

static void sf_c2_simiam_AvoidObstacle_BestAO
  (SFc2_simiam_AvoidObstacle_BestAOInstanceStruct *chartInstance)
{
  int32_T c2_i2;
  int32_T c2_i3;
  int32_T c2_i4;
  real_T *c2_OutData;
  real_T *c2_FixAngle;
  real_T (*c2_NextPoint)[2];
  real_T (*c2_Position)[3];
  real_T (*c2_ir_raw)[9];
  c2_FixAngle = (real_T *)ssGetInputPortSignal(chartInstance->S, 2);
  c2_OutData = (real_T *)ssGetOutputPortSignal(chartInstance->S, 2);
  c2_NextPoint = (real_T (*)[2])ssGetOutputPortSignal(chartInstance->S, 1);
  c2_Position = (real_T (*)[3])ssGetInputPortSignal(chartInstance->S, 1);
  c2_ir_raw = (real_T (*)[9])ssGetInputPortSignal(chartInstance->S, 0);
  _SFD_SYMBOL_SCOPE_PUSH(0U, 0U);
  _sfTime_ = (real_T)ssGetT(chartInstance->S);
  _SFD_CC_CALL(CHART_ENTER_SFUNCTION_TAG, 1U, chartInstance->c2_sfEvent);
  for (c2_i2 = 0; c2_i2 < 9; c2_i2++) {
    _SFD_DATA_RANGE_CHECK((*c2_ir_raw)[c2_i2], 0U);
  }

  for (c2_i3 = 0; c2_i3 < 3; c2_i3++) {
    _SFD_DATA_RANGE_CHECK((*c2_Position)[c2_i3], 1U);
  }

  for (c2_i4 = 0; c2_i4 < 2; c2_i4++) {
    _SFD_DATA_RANGE_CHECK((*c2_NextPoint)[c2_i4], 2U);
  }

  _SFD_DATA_RANGE_CHECK(*c2_OutData, 3U);
  _SFD_DATA_RANGE_CHECK(*c2_FixAngle, 4U);
  chartInstance->c2_sfEvent = CALL_EVENT;
  c2_chartstep_c2_simiam_AvoidObstacle_BestAO(chartInstance);
  _SFD_SYMBOL_SCOPE_POP();
  _SFD_CHECK_FOR_STATE_INCONSISTENCY(_simiam_AvoidObstacle_BestAOMachineNumber_,
    chartInstance->chartNumber, chartInstance->instanceNumber);
}

static void c2_chartstep_c2_simiam_AvoidObstacle_BestAO
  (SFc2_simiam_AvoidObstacle_BestAOInstanceStruct *chartInstance)
{
  real_T c2_hoistedGlobal;
  int32_T c2_i5;
  real_T c2_ir_raw[9];
  int32_T c2_i6;
  real_T c2_Position[3];
  real_T c2_FixAngle;
  uint32_T c2_debug_family_var_map[17];
  real_T c2_ir_distances[9];
  int32_T c2_ObstacleDetected_sizes;
  real_T c2_ObstacleDetected_data[9];
  real_T c2_FixIR_9_Number;
  real_T c2_Obstacle_AroundRobot[27];
  real_T c2_i;
  real_T c2_IR_Position_I[3];
  real_T c2_IR_Transformation_Matrix_I[9];
  real_T c2_Obstacle_AroundRobot_I[3];
  real_T c2_Obstacle_AroundRobot_World[27];
  int32_T c2_ObstacleDetected_Number_sizes;
  boolean_T c2_ObstacleDetected_Number_data[9];
  real_T c2_nargin = 3.0;
  real_T c2_nargout = 2.0;
  real_T c2_NextPoint[2];
  real_T c2_OutData;
  int32_T c2_i7;
  real_T c2_b_ir_raw[9];
  int32_T c2_b_ObstacleDetected_sizes;
  real_T c2_b_ObstacleDetected_data[9];
  real_T c2_b_ir_distances[9];
  int32_T c2_i8;
  int32_T c2_loop_ub;
  int32_T c2_i9;
  int32_T c2_i10;
  int32_T c2_i11;
  int32_T c2_b_i;
  real_T c2_dv1[3];
  int32_T c2_i12;
  real_T c2_dv2[9];
  int32_T c2_i13;
  int32_T c2_i14;
  real_T c2_a[9];
  real_T c2_b[3];
  int32_T c2_i15;
  int32_T c2_i16;
  int32_T c2_i17;
  real_T c2_C[3];
  int32_T c2_i18;
  int32_T c2_i19;
  int32_T c2_i20;
  int32_T c2_i21;
  int32_T c2_i22;
  int32_T c2_i23;
  int32_T c2_c_i;
  int32_T c2_i24;
  int32_T c2_i25;
  real_T c2_b_b[27];
  int32_T c2_i26;
  int32_T c2_i27;
  int32_T c2_i28;
  real_T c2_b_C[27];
  int32_T c2_i29;
  int32_T c2_i30;
  int32_T c2_i31;
  int32_T c2_i32;
  int32_T c2_i33;
  int32_T c2_i34;
  int32_T c2_i35;
  int32_T c2_i36;
  int32_T c2_x_sizes;
  int32_T c2_b_loop_ub;
  int32_T c2_i37;
  boolean_T c2_x_data[9];
  int32_T c2_tmp_sizes;
  int32_T c2_c_loop_ub;
  int32_T c2_i38;
  boolean_T c2_tmp_data[9];
  int32_T c2_d_loop_ub;
  int32_T c2_i39;
  int32_T c2_e_loop_ub;
  int32_T c2_i40;
  int32_T c2_f_loop_ub;
  int32_T c2_i41;
  int32_T c2_g_loop_ub;
  int32_T c2_i42;
  int32_T c2_n;
  int32_T c2_b_n;
  int32_T c2_k;
  int32_T c2_c_n;
  int32_T c2_d_i;
  int32_T c2_e_i;
  int32_T c2_b_a;
  const mxArray *c2_y = NULL;
  int32_T c2_b_tmp_sizes;
  int32_T c2_h_loop_ub;
  int32_T c2_i43;
  int32_T c2_b_tmp_data[9];
  int32_T c2_y_sizes;
  int32_T c2_j;
  int32_T c2_d_n;
  int32_T c2_f_i;
  int32_T c2_g_i;
  int32_T c2_y_data[9];
  int32_T c2_c_a;
  int32_T c2_ObstacleDetected;
  int32_T c2_b_ObstacleDetected[2];
  int32_T c2_c_ObstacleDetected;
  int32_T c2_c_ObstacleDetected_sizes;
  int32_T c2_i_loop_ub;
  int32_T c2_i44;
  real_T c2_c_ObstacleDetected_data[9];
  int32_T c2_j_loop_ub;
  int32_T c2_i45;
  int32_T c2_k_loop_ub;
  int32_T c2_i46;
  real_T c2_e_n;
  int32_T c2_i47;
  real_T c2_b_Obstacle_AroundRobot_World[27];
  int32_T c2_i48;
  real_T c2_b_Position[3];
  int32_T c2_d_ObstacleDetected_sizes;
  int32_T c2_l_loop_ub;
  int32_T c2_i49;
  real_T c2_d_ObstacleDetected_data[9];
  real_T c2_b_OutData;
  real_T c2_b_NextPoint[2];
  int32_T c2_i50;
  int32_T c2_i51;
  real_T c2_c_Obstacle_AroundRobot_World[27];
  int32_T c2_i52;
  real_T c2_c_Position[3];
  real_T c2_dv3[2];
  int32_T c2_i53;
  int32_T c2_i54;
  real_T *c2_b_FixAngle;
  real_T *c2_c_OutData;
  real_T (*c2_c_NextPoint)[2];
  real_T (*c2_d_Position)[3];
  real_T (*c2_c_ir_raw)[9];
  c2_b_FixAngle = (real_T *)ssGetInputPortSignal(chartInstance->S, 2);
  c2_c_OutData = (real_T *)ssGetOutputPortSignal(chartInstance->S, 2);
  c2_c_NextPoint = (real_T (*)[2])ssGetOutputPortSignal(chartInstance->S, 1);
  c2_d_Position = (real_T (*)[3])ssGetInputPortSignal(chartInstance->S, 1);
  c2_c_ir_raw = (real_T (*)[9])ssGetInputPortSignal(chartInstance->S, 0);
  _SFD_CC_CALL(CHART_ENTER_DURING_FUNCTION_TAG, 1U, chartInstance->c2_sfEvent);
  c2_hoistedGlobal = *c2_b_FixAngle;
  for (c2_i5 = 0; c2_i5 < 9; c2_i5++) {
    c2_ir_raw[c2_i5] = (*c2_c_ir_raw)[c2_i5];
  }

  for (c2_i6 = 0; c2_i6 < 3; c2_i6++) {
    c2_Position[c2_i6] = (*c2_d_Position)[c2_i6];
  }

  c2_FixAngle = c2_hoistedGlobal;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 17U, 17U, c2_debug_family_names,
    c2_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_ir_distances, 0U, c2_d_sf_marshallOut,
    c2_h_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_DYN_IMPORTABLE(c2_ObstacleDetected_data, (const
    int32_T *)&c2_ObstacleDetected_sizes, NULL, 0, 1, (void *)
    c2_h_sf_marshallOut, (void *)c2_g_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c2_FixIR_9_Number, 2U, c2_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_Obstacle_AroundRobot, 3U,
    c2_f_sf_marshallOut, c2_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_i, 4U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_IR_Position_I, 5U, c2_c_sf_marshallOut,
    c2_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_IR_Transformation_Matrix_I, 6U,
    c2_g_sf_marshallOut, c2_f_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_Obstacle_AroundRobot_I, 7U,
    c2_c_sf_marshallOut, c2_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_Obstacle_AroundRobot_World, 8U,
    c2_f_sf_marshallOut, c2_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_DYN_IMPORTABLE(c2_ObstacleDetected_Number_data, (
    const int32_T *)&c2_ObstacleDetected_Number_sizes, NULL, 0, 9, (void *)
    c2_e_sf_marshallOut, (void *)c2_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_nargin, 10U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_nargout, 11U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(c2_ir_raw, 12U, c2_d_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c2_Position, 13U, c2_c_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c2_FixAngle, 14U, c2_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_NextPoint, 15U, c2_b_sf_marshallOut,
    c2_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_OutData, 16U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  CV_EML_FCN(0, 0);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 6);
  for (c2_i7 = 0; c2_i7 < 9; c2_i7++) {
    c2_b_ir_raw[c2_i7] = c2_ir_raw[c2_i7];
  }

  c2_IR_Raw_to_Distances(chartInstance, c2_b_ir_raw, c2_b_ir_distances,
    c2_b_ObstacleDetected_data, *(int32_T (*)[1])&c2_b_ObstacleDetected_sizes);
  for (c2_i8 = 0; c2_i8 < 9; c2_i8++) {
    c2_ir_distances[c2_i8] = c2_b_ir_distances[c2_i8];
  }

  c2_ObstacleDetected_sizes = c2_b_ObstacleDetected_sizes;
  c2_loop_ub = c2_b_ObstacleDetected_sizes - 1;
  for (c2_i9 = 0; c2_i9 <= c2_loop_ub; c2_i9++) {
    c2_ObstacleDetected_data[c2_i9] = c2_b_ObstacleDetected_data[c2_i9];
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 10);
  c2_FixIR_9_Number = 0.0;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 11);
  for (c2_i10 = 0; c2_i10 < 9; c2_i10++) {
    c2_ir_distances[c2_i10];
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 13);
  for (c2_i11 = 0; c2_i11 < 27; c2_i11++) {
    c2_Obstacle_AroundRobot[c2_i11] = 0.0;
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 14);
  c2_i = 1.0;
  c2_b_i = 0;
  while (c2_b_i < 9) {
    c2_i = 1.0 + (real_T)c2_b_i;
    CV_EML_FOR(0, 1, 0, 1);
    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 15);
    c2_GetIR_Position(chartInstance, c2_i, c2_dv1);
    for (c2_i12 = 0; c2_i12 < 3; c2_i12++) {
      c2_IR_Position_I[c2_i12] = c2_dv1[c2_i12];
    }

    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 16);
    c2_Transformation_Matrix(chartInstance, c2_IR_Position_I[0],
      c2_IR_Position_I[1], c2_IR_Position_I[2], c2_dv2);
    for (c2_i13 = 0; c2_i13 < 9; c2_i13++) {
      c2_IR_Transformation_Matrix_I[c2_i13] = c2_dv2[c2_i13];
    }

    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 17);
    for (c2_i14 = 0; c2_i14 < 9; c2_i14++) {
      c2_a[c2_i14] = c2_IR_Transformation_Matrix_I[c2_i14];
    }

    c2_b[0] = c2_ir_distances[(int32_T)c2_i - 1];
    c2_b[1] = 0.0;
    c2_b[2] = 1.0;
    c2_eml_scalar_eg(chartInstance);
    c2_eml_scalar_eg(chartInstance);
    for (c2_i15 = 0; c2_i15 < 3; c2_i15++) {
      c2_Obstacle_AroundRobot_I[c2_i15] = 0.0;
    }

    for (c2_i16 = 0; c2_i16 < 3; c2_i16++) {
      c2_Obstacle_AroundRobot_I[c2_i16] = 0.0;
    }

    for (c2_i17 = 0; c2_i17 < 3; c2_i17++) {
      c2_C[c2_i17] = c2_Obstacle_AroundRobot_I[c2_i17];
    }

    for (c2_i18 = 0; c2_i18 < 3; c2_i18++) {
      c2_Obstacle_AroundRobot_I[c2_i18] = c2_C[c2_i18];
    }

    for (c2_i19 = 0; c2_i19 < 3; c2_i19++) {
      c2_C[c2_i19] = c2_Obstacle_AroundRobot_I[c2_i19];
    }

    for (c2_i20 = 0; c2_i20 < 3; c2_i20++) {
      c2_Obstacle_AroundRobot_I[c2_i20] = c2_C[c2_i20];
    }

    for (c2_i21 = 0; c2_i21 < 3; c2_i21++) {
      c2_Obstacle_AroundRobot_I[c2_i21] = 0.0;
      c2_i22 = 0;
      for (c2_i23 = 0; c2_i23 < 3; c2_i23++) {
        c2_Obstacle_AroundRobot_I[c2_i21] += c2_a[c2_i22 + c2_i21] * c2_b[c2_i23];
        c2_i22 += 3;
      }
    }

    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 19);
    c2_c_i = (int32_T)c2_i - 1;
    for (c2_i24 = 0; c2_i24 < 3; c2_i24++) {
      c2_Obstacle_AroundRobot[c2_i24 + 3 * c2_c_i] =
        c2_Obstacle_AroundRobot_I[c2_i24];
    }

    c2_b_i++;
    _SF_MEX_LISTEN_FOR_CTRL_C(chartInstance->S);
  }

  CV_EML_FOR(0, 1, 0, 0);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 22);
  c2_Transformation_Matrix(chartInstance, c2_Position[0], c2_Position[1],
    c2_Position[2], c2_a);
  for (c2_i25 = 0; c2_i25 < 27; c2_i25++) {
    c2_b_b[c2_i25] = c2_Obstacle_AroundRobot[c2_i25];
  }

  c2_b_eml_scalar_eg(chartInstance);
  c2_b_eml_scalar_eg(chartInstance);
  for (c2_i26 = 0; c2_i26 < 27; c2_i26++) {
    c2_Obstacle_AroundRobot_World[c2_i26] = 0.0;
  }

  for (c2_i27 = 0; c2_i27 < 27; c2_i27++) {
    c2_Obstacle_AroundRobot_World[c2_i27] = 0.0;
  }

  for (c2_i28 = 0; c2_i28 < 27; c2_i28++) {
    c2_b_C[c2_i28] = c2_Obstacle_AroundRobot_World[c2_i28];
  }

  for (c2_i29 = 0; c2_i29 < 27; c2_i29++) {
    c2_Obstacle_AroundRobot_World[c2_i29] = c2_b_C[c2_i29];
  }

  for (c2_i30 = 0; c2_i30 < 27; c2_i30++) {
    c2_b_C[c2_i30] = c2_Obstacle_AroundRobot_World[c2_i30];
  }

  for (c2_i31 = 0; c2_i31 < 27; c2_i31++) {
    c2_Obstacle_AroundRobot_World[c2_i31] = c2_b_C[c2_i31];
  }

  for (c2_i32 = 0; c2_i32 < 3; c2_i32++) {
    c2_i33 = 0;
    for (c2_i34 = 0; c2_i34 < 9; c2_i34++) {
      c2_Obstacle_AroundRobot_World[c2_i33 + c2_i32] = 0.0;
      c2_i35 = 0;
      for (c2_i36 = 0; c2_i36 < 3; c2_i36++) {
        c2_Obstacle_AroundRobot_World[c2_i33 + c2_i32] += c2_a[c2_i35 + c2_i32] *
          c2_b_b[c2_i36 + c2_i33];
        c2_i35 += 3;
      }

      c2_i33 += 3;
    }
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 26);
  c2_x_sizes = c2_ObstacleDetected_sizes;
  c2_b_loop_ub = c2_ObstacleDetected_sizes - 1;
  for (c2_i37 = 0; c2_i37 <= c2_b_loop_ub; c2_i37++) {
    c2_x_data[c2_i37] = (c2_ObstacleDetected_data[c2_i37] != 1.0);
  }

  c2_tmp_sizes = c2_ObstacleDetected_sizes;
  c2_c_loop_ub = c2_ObstacleDetected_sizes - 1;
  for (c2_i38 = 0; c2_i38 <= c2_c_loop_ub; c2_i38++) {
    c2_tmp_data[c2_i38] = (c2_ObstacleDetected_data[c2_i38] != 8.0);
  }

  _SFD_SIZE_EQ_CHECK_1D(c2_x_sizes, c2_tmp_sizes);
  c2_x_sizes;
  c2_d_loop_ub = c2_x_sizes - 1;
  for (c2_i39 = 0; c2_i39 <= c2_d_loop_ub; c2_i39++) {
    c2_x_data[c2_i39] = (c2_x_data[c2_i39] && c2_tmp_data[c2_i39]);
  }

  c2_tmp_sizes = c2_ObstacleDetected_sizes;
  c2_e_loop_ub = c2_ObstacleDetected_sizes - 1;
  for (c2_i40 = 0; c2_i40 <= c2_e_loop_ub; c2_i40++) {
    c2_tmp_data[c2_i40] = (c2_ObstacleDetected_data[c2_i40] != 9.0);
  }

  _SFD_SIZE_EQ_CHECK_1D(c2_x_sizes, c2_tmp_sizes);
  c2_ObstacleDetected_Number_sizes = c2_x_sizes;
  c2_f_loop_ub = c2_x_sizes - 1;
  for (c2_i41 = 0; c2_i41 <= c2_f_loop_ub; c2_i41++) {
    c2_ObstacleDetected_Number_data[c2_i41] = (c2_x_data[c2_i41] &&
      c2_tmp_data[c2_i41]);
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 27);
  c2_x_sizes = c2_ObstacleDetected_Number_sizes;
  c2_g_loop_ub = c2_ObstacleDetected_Number_sizes - 1;
  for (c2_i42 = 0; c2_i42 <= c2_g_loop_ub; c2_i42++) {
    c2_x_data[c2_i42] = c2_ObstacleDetected_Number_data[c2_i42];
  }

  c2_n = (int32_T)(real_T)c2_x_sizes;
  c2_b_n = c2_n;
  c2_k = 0;
  c2_c_n = c2_b_n;
  for (c2_d_i = 1; c2_d_i <= c2_c_n; c2_d_i++) {
    c2_e_i = c2_d_i;
    if (c2_x_data[_SFD_EML_ARRAY_BOUNDS_CHECK("", c2_e_i, 1, c2_x_sizes, 1, 0) -
        1]) {
      c2_b_a = c2_k + 1;
      c2_k = c2_b_a;
    }
  }

  if (c2_k <= c2_n) {
  } else {
    c2_y = NULL;
    sf_mex_assign(&c2_y, sf_mex_create("y", "Assertion failed.", 15, 0U, 0U, 0U,
      2, 1, strlen("Assertion failed.")), FALSE);
    sf_mex_call_debug("error", 0U, 1U, 14, c2_y);
  }

  c2_b_tmp_sizes = (int32_T)_SFD_NON_NEGATIVE_CHECK("", (real_T)c2_k);
  c2_h_loop_ub = (int32_T)_SFD_NON_NEGATIVE_CHECK("", (real_T)c2_k) - 1;
  for (c2_i43 = 0; c2_i43 <= c2_h_loop_ub; c2_i43++) {
    c2_b_tmp_data[c2_i43] = 0;
  }

  c2_y_sizes = c2_b_tmp_sizes;
  c2_j = 1;
  c2_d_n = c2_n;
  for (c2_f_i = 1; c2_f_i <= c2_d_n; c2_f_i++) {
    c2_g_i = c2_f_i;
    if (c2_x_data[_SFD_EML_ARRAY_BOUNDS_CHECK("", c2_g_i, 1, c2_x_sizes, 1, 0) -
        1]) {
      c2_y_data[_SFD_EML_ARRAY_BOUNDS_CHECK("", c2_j, 1, c2_y_sizes, 1, 0) - 1] =
        c2_g_i;
      c2_c_a = c2_j + 1;
      c2_j = c2_c_a;
    }
  }

  c2_ObstacleDetected = c2_ObstacleDetected_sizes;
  c2_b_ObstacleDetected[0] = c2_ObstacleDetected;
  c2_b_ObstacleDetected[1] = 1;
  c2_c_ObstacleDetected = c2_ObstacleDetected_sizes;
  c2_c_ObstacleDetected_sizes = c2_y_sizes;
  c2_i_loop_ub = c2_y_sizes - 1;
  for (c2_i44 = 0; c2_i44 <= c2_i_loop_ub; c2_i44++) {
    c2_c_ObstacleDetected_data[c2_i44] =
      c2_ObstacleDetected_data[_SFD_EML_ARRAY_BOUNDS_CHECK("ObstacleDetected",
      c2_y_data[c2_i44], 1, c2_c_ObstacleDetected, 1, 0) - 1];
  }

  c2_ObstacleDetected_sizes = c2_c_ObstacleDetected_sizes;
  c2_j_loop_ub = c2_c_ObstacleDetected_sizes - 1;
  for (c2_i45 = 0; c2_i45 <= c2_j_loop_ub; c2_i45++) {
    c2_ObstacleDetected_data[c2_i45] = c2_c_ObstacleDetected_data[c2_i45];
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 29);
  c2_b_ObstacleDetected_sizes = c2_ObstacleDetected_sizes;
  c2_k_loop_ub = c2_ObstacleDetected_sizes - 1;
  for (c2_i46 = 0; c2_i46 <= c2_k_loop_ub; c2_i46++) {
    c2_b_ObstacleDetected_data[c2_i46] = c2_ObstacleDetected_data[c2_i46];
  }

  c2_e_n = (real_T)c2_b_ObstacleDetected_sizes;
  if (CV_EML_IF(0, 1, 0, c2_e_n > 1.0)) {
    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 31);
    for (c2_i47 = 0; c2_i47 < 27; c2_i47++) {
      c2_b_Obstacle_AroundRobot_World[c2_i47] =
        c2_Obstacle_AroundRobot_World[c2_i47];
    }

    for (c2_i48 = 0; c2_i48 < 3; c2_i48++) {
      c2_b_Position[c2_i48] = c2_Position[c2_i48];
    }

    c2_d_ObstacleDetected_sizes = c2_ObstacleDetected_sizes;
    c2_l_loop_ub = c2_ObstacleDetected_sizes - 1;
    for (c2_i49 = 0; c2_i49 <= c2_l_loop_ub; c2_i49++) {
      c2_d_ObstacleDetected_data[c2_i49] = c2_ObstacleDetected_data[c2_i49];
    }

    c2_AvoidObstacles_NextPoint_BestAO(chartInstance,
      c2_b_Obstacle_AroundRobot_World, c2_b_Position, c2_d_ObstacleDetected_data,
      *(int32_T (*)[1])&c2_d_ObstacleDetected_sizes, c2_b_NextPoint,
      &c2_b_OutData);
    for (c2_i50 = 0; c2_i50 < 2; c2_i50++) {
      c2_NextPoint[c2_i50] = c2_b_NextPoint[c2_i50];
    }

    c2_OutData = c2_b_OutData;
  } else {
    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 34);
    for (c2_i51 = 0; c2_i51 < 27; c2_i51++) {
      c2_c_Obstacle_AroundRobot_World[c2_i51] =
        c2_Obstacle_AroundRobot_World[c2_i51];
    }

    for (c2_i52 = 0; c2_i52 < 3; c2_i52++) {
      c2_c_Position[c2_i52] = c2_Position[c2_i52];
    }

    c2_AirLine_NextPoint(chartInstance, c2_c_Obstacle_AroundRobot_World,
                         c2_c_Position, c2_dv3);
    for (c2_i53 = 0; c2_i53 < 2; c2_i53++) {
      c2_NextPoint[c2_i53] = c2_dv3[c2_i53];
    }

    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 35);
    c2_OutData = 0.0;
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, -35);
  _SFD_SYMBOL_SCOPE_POP();
  for (c2_i54 = 0; c2_i54 < 2; c2_i54++) {
    (*c2_c_NextPoint)[c2_i54] = c2_NextPoint[c2_i54];
  }

  *c2_c_OutData = c2_OutData;
  _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG, 1U, chartInstance->c2_sfEvent);
}

static void initSimStructsc2_simiam_AvoidObstacle_BestAO
  (SFc2_simiam_AvoidObstacle_BestAOInstanceStruct *chartInstance)
{
}

static void init_script_number_translation(uint32_T c2_machineNumber, uint32_T
  c2_chartNumber)
{
  _SFD_SCRIPT_TRANSLATION(c2_chartNumber, 0U, sf_debug_get_script_id(
    "F:/BlueBird/Study/Robot/FollowWall/20140301_DynamicPoint/+simiam/+ui/Pose2D.m"));
  _SFD_SCRIPT_TRANSLATION(c2_chartNumber, 1U, sf_debug_get_script_id(
    "F:/BlueBird/Study/Robot/FollowWall/20140301_DynamicPoint/+simiam/+controller/Controller.m"));
  _SFD_SCRIPT_TRANSLATION(c2_chartNumber, 2U, sf_debug_get_script_id(
    "F:/BlueBird/Study/Robot/FollowWall/20140301_DynamicPoint/+simiam/+controller/AvoidObstacles.m"));
}

static const mxArray *c2_sf_marshallOut(void *chartInstanceVoid, void *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  real_T c2_u;
  const mxArray *c2_y = NULL;
  SFc2_simiam_AvoidObstacle_BestAOInstanceStruct *chartInstance;
  chartInstance = (SFc2_simiam_AvoidObstacle_BestAOInstanceStruct *)
    chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_u = *(real_T *)c2_inData;
  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", &c2_u, 0, 0U, 0U, 0U, 0), FALSE);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, FALSE);
  return c2_mxArrayOutData;
}

static real_T c2_emlrt_marshallIn(SFc2_simiam_AvoidObstacle_BestAOInstanceStruct
  *chartInstance, const mxArray *c2_OutData, const char_T *c2_identifier)
{
  real_T c2_y;
  emlrtMsgIdentifier c2_thisId;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_y = c2_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_OutData), &c2_thisId);
  sf_mex_destroy(&c2_OutData);
  return c2_y;
}

static real_T c2_b_emlrt_marshallIn
  (SFc2_simiam_AvoidObstacle_BestAOInstanceStruct *chartInstance, const mxArray *
   c2_u, const emlrtMsgIdentifier *c2_parentId)
{
  real_T c2_y;
  real_T c2_d0;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), &c2_d0, 1, 0, 0U, 0, 0U, 0);
  c2_y = c2_d0;
  sf_mex_destroy(&c2_u);
  return c2_y;
}

static void c2_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_OutData;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y;
  SFc2_simiam_AvoidObstacle_BestAOInstanceStruct *chartInstance;
  chartInstance = (SFc2_simiam_AvoidObstacle_BestAOInstanceStruct *)
    chartInstanceVoid;
  c2_OutData = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_y = c2_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_OutData), &c2_thisId);
  sf_mex_destroy(&c2_OutData);
  *(real_T *)c2_outData = c2_y;
  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_b_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_i55;
  real_T c2_b_inData[2];
  int32_T c2_i56;
  real_T c2_u[2];
  const mxArray *c2_y = NULL;
  SFc2_simiam_AvoidObstacle_BestAOInstanceStruct *chartInstance;
  chartInstance = (SFc2_simiam_AvoidObstacle_BestAOInstanceStruct *)
    chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  for (c2_i55 = 0; c2_i55 < 2; c2_i55++) {
    c2_b_inData[c2_i55] = (*(real_T (*)[2])c2_inData)[c2_i55];
  }

  for (c2_i56 = 0; c2_i56 < 2; c2_i56++) {
    c2_u[c2_i56] = c2_b_inData[c2_i56];
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 1, 2), FALSE);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, FALSE);
  return c2_mxArrayOutData;
}

static void c2_c_emlrt_marshallIn(SFc2_simiam_AvoidObstacle_BestAOInstanceStruct
  *chartInstance, const mxArray *c2_NextPoint, const char_T *c2_identifier,
  real_T c2_y[2])
{
  emlrtMsgIdentifier c2_thisId;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_d_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_NextPoint), &c2_thisId,
                        c2_y);
  sf_mex_destroy(&c2_NextPoint);
}

static void c2_d_emlrt_marshallIn(SFc2_simiam_AvoidObstacle_BestAOInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  real_T c2_y[2])
{
  real_T c2_dv4[2];
  int32_T c2_i57;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), c2_dv4, 1, 0, 0U, 1, 0U, 1, 2);
  for (c2_i57 = 0; c2_i57 < 2; c2_i57++) {
    c2_y[c2_i57] = c2_dv4[c2_i57];
  }

  sf_mex_destroy(&c2_u);
}

static void c2_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_NextPoint;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y[2];
  int32_T c2_i58;
  SFc2_simiam_AvoidObstacle_BestAOInstanceStruct *chartInstance;
  chartInstance = (SFc2_simiam_AvoidObstacle_BestAOInstanceStruct *)
    chartInstanceVoid;
  c2_NextPoint = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_d_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_NextPoint), &c2_thisId,
                        c2_y);
  sf_mex_destroy(&c2_NextPoint);
  for (c2_i58 = 0; c2_i58 < 2; c2_i58++) {
    (*(real_T (*)[2])c2_outData)[c2_i58] = c2_y[c2_i58];
  }

  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_c_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_i59;
  real_T c2_b_inData[3];
  int32_T c2_i60;
  real_T c2_u[3];
  const mxArray *c2_y = NULL;
  SFc2_simiam_AvoidObstacle_BestAOInstanceStruct *chartInstance;
  chartInstance = (SFc2_simiam_AvoidObstacle_BestAOInstanceStruct *)
    chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  for (c2_i59 = 0; c2_i59 < 3; c2_i59++) {
    c2_b_inData[c2_i59] = (*(real_T (*)[3])c2_inData)[c2_i59];
  }

  for (c2_i60 = 0; c2_i60 < 3; c2_i60++) {
    c2_u[c2_i60] = c2_b_inData[c2_i60];
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 1, 3), FALSE);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, FALSE);
  return c2_mxArrayOutData;
}

static const mxArray *c2_d_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_i61;
  real_T c2_b_inData[9];
  int32_T c2_i62;
  real_T c2_u[9];
  const mxArray *c2_y = NULL;
  SFc2_simiam_AvoidObstacle_BestAOInstanceStruct *chartInstance;
  chartInstance = (SFc2_simiam_AvoidObstacle_BestAOInstanceStruct *)
    chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  for (c2_i61 = 0; c2_i61 < 9; c2_i61++) {
    c2_b_inData[c2_i61] = (*(real_T (*)[9])c2_inData)[c2_i61];
  }

  for (c2_i62 = 0; c2_i62 < 9; c2_i62++) {
    c2_u[c2_i62] = c2_b_inData[c2_i62];
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 1, 9), FALSE);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, FALSE);
  return c2_mxArrayOutData;
}

static const mxArray *c2_e_sf_marshallOut(void *chartInstanceVoid, boolean_T
  c2_inData_data[9], int32_T c2_inData_sizes[1])
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_b_inData_sizes;
  int32_T c2_loop_ub;
  int32_T c2_i63;
  boolean_T c2_b_inData_data[9];
  int32_T c2_u_sizes;
  int32_T c2_b_loop_ub;
  int32_T c2_i64;
  boolean_T c2_u_data[9];
  const mxArray *c2_y = NULL;
  SFc2_simiam_AvoidObstacle_BestAOInstanceStruct *chartInstance;
  chartInstance = (SFc2_simiam_AvoidObstacle_BestAOInstanceStruct *)
    chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_b_inData_sizes = c2_inData_sizes[0];
  c2_loop_ub = c2_inData_sizes[0] - 1;
  for (c2_i63 = 0; c2_i63 <= c2_loop_ub; c2_i63++) {
    c2_b_inData_data[c2_i63] = c2_inData_data[c2_i63];
  }

  c2_u_sizes = c2_b_inData_sizes;
  c2_b_loop_ub = c2_b_inData_sizes - 1;
  for (c2_i64 = 0; c2_i64 <= c2_b_loop_ub; c2_i64++) {
    c2_u_data[c2_i64] = c2_b_inData_data[c2_i64];
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u_data, 11, 0U, 1U, 0U, 1,
    c2_u_sizes), FALSE);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, FALSE);
  return c2_mxArrayOutData;
}

static void c2_e_emlrt_marshallIn(SFc2_simiam_AvoidObstacle_BestAOInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  boolean_T c2_y_data[9], int32_T c2_y_sizes[1])
{
  static uint32_T c2_uv0[1] = { 9U };

  uint32_T c2_uv1[1];
  static boolean_T c2_bv0[1] = { TRUE };

  boolean_T c2_bv1[1];
  int32_T c2_tmp_sizes;
  boolean_T c2_tmp_data[9];
  int32_T c2_loop_ub;
  int32_T c2_i65;
  c2_uv1[0] = c2_uv0[0];
  c2_bv1[0] = c2_bv0[0];
  sf_mex_import_vs(c2_parentId, sf_mex_dup(c2_u), c2_tmp_data, 1, 11, 0U, 1, 0U,
                   1, c2_bv1, c2_uv1, &c2_tmp_sizes);
  c2_y_sizes[0] = c2_tmp_sizes;
  c2_loop_ub = c2_tmp_sizes - 1;
  for (c2_i65 = 0; c2_i65 <= c2_loop_ub; c2_i65++) {
    c2_y_data[c2_i65] = c2_tmp_data[c2_i65];
  }

  sf_mex_destroy(&c2_u);
}

static void c2_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, boolean_T c2_outData_data[9],
  int32_T c2_outData_sizes[1])
{
  const mxArray *c2_ObstacleDetected_Number;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  int32_T c2_y_sizes;
  boolean_T c2_y_data[9];
  int32_T c2_loop_ub;
  int32_T c2_i66;
  SFc2_simiam_AvoidObstacle_BestAOInstanceStruct *chartInstance;
  chartInstance = (SFc2_simiam_AvoidObstacle_BestAOInstanceStruct *)
    chartInstanceVoid;
  c2_ObstacleDetected_Number = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_e_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_ObstacleDetected_Number),
                        &c2_thisId, c2_y_data, *(int32_T (*)[1])&c2_y_sizes);
  sf_mex_destroy(&c2_ObstacleDetected_Number);
  c2_outData_sizes[0] = c2_y_sizes;
  c2_loop_ub = c2_y_sizes - 1;
  for (c2_i66 = 0; c2_i66 <= c2_loop_ub; c2_i66++) {
    c2_outData_data[c2_i66] = c2_y_data[c2_i66];
  }

  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_f_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_i67;
  int32_T c2_i68;
  int32_T c2_i69;
  real_T c2_b_inData[27];
  int32_T c2_i70;
  int32_T c2_i71;
  int32_T c2_i72;
  real_T c2_u[27];
  const mxArray *c2_y = NULL;
  SFc2_simiam_AvoidObstacle_BestAOInstanceStruct *chartInstance;
  chartInstance = (SFc2_simiam_AvoidObstacle_BestAOInstanceStruct *)
    chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_i67 = 0;
  for (c2_i68 = 0; c2_i68 < 9; c2_i68++) {
    for (c2_i69 = 0; c2_i69 < 3; c2_i69++) {
      c2_b_inData[c2_i69 + c2_i67] = (*(real_T (*)[27])c2_inData)[c2_i69 +
        c2_i67];
    }

    c2_i67 += 3;
  }

  c2_i70 = 0;
  for (c2_i71 = 0; c2_i71 < 9; c2_i71++) {
    for (c2_i72 = 0; c2_i72 < 3; c2_i72++) {
      c2_u[c2_i72 + c2_i70] = c2_b_inData[c2_i72 + c2_i70];
    }

    c2_i70 += 3;
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 2, 3, 9), FALSE);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, FALSE);
  return c2_mxArrayOutData;
}

static void c2_f_emlrt_marshallIn(SFc2_simiam_AvoidObstacle_BestAOInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  real_T c2_y[27])
{
  real_T c2_dv5[27];
  int32_T c2_i73;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), c2_dv5, 1, 0, 0U, 1, 0U, 2, 3, 9);
  for (c2_i73 = 0; c2_i73 < 27; c2_i73++) {
    c2_y[c2_i73] = c2_dv5[c2_i73];
  }

  sf_mex_destroy(&c2_u);
}

static void c2_d_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_Obstacle_AroundRobot_World;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y[27];
  int32_T c2_i74;
  int32_T c2_i75;
  int32_T c2_i76;
  SFc2_simiam_AvoidObstacle_BestAOInstanceStruct *chartInstance;
  chartInstance = (SFc2_simiam_AvoidObstacle_BestAOInstanceStruct *)
    chartInstanceVoid;
  c2_Obstacle_AroundRobot_World = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_f_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_Obstacle_AroundRobot_World),
                        &c2_thisId, c2_y);
  sf_mex_destroy(&c2_Obstacle_AroundRobot_World);
  c2_i74 = 0;
  for (c2_i75 = 0; c2_i75 < 9; c2_i75++) {
    for (c2_i76 = 0; c2_i76 < 3; c2_i76++) {
      (*(real_T (*)[27])c2_outData)[c2_i76 + c2_i74] = c2_y[c2_i76 + c2_i74];
    }

    c2_i74 += 3;
  }

  sf_mex_destroy(&c2_mxArrayInData);
}

static void c2_g_emlrt_marshallIn(SFc2_simiam_AvoidObstacle_BestAOInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  real_T c2_y[3])
{
  real_T c2_dv6[3];
  int32_T c2_i77;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), c2_dv6, 1, 0, 0U, 1, 0U, 1, 3);
  for (c2_i77 = 0; c2_i77 < 3; c2_i77++) {
    c2_y[c2_i77] = c2_dv6[c2_i77];
  }

  sf_mex_destroy(&c2_u);
}

static void c2_e_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_Obstacle_AroundRobot_I;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y[3];
  int32_T c2_i78;
  SFc2_simiam_AvoidObstacle_BestAOInstanceStruct *chartInstance;
  chartInstance = (SFc2_simiam_AvoidObstacle_BestAOInstanceStruct *)
    chartInstanceVoid;
  c2_Obstacle_AroundRobot_I = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_g_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_Obstacle_AroundRobot_I),
                        &c2_thisId, c2_y);
  sf_mex_destroy(&c2_Obstacle_AroundRobot_I);
  for (c2_i78 = 0; c2_i78 < 3; c2_i78++) {
    (*(real_T (*)[3])c2_outData)[c2_i78] = c2_y[c2_i78];
  }

  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_g_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_i79;
  int32_T c2_i80;
  int32_T c2_i81;
  real_T c2_b_inData[9];
  int32_T c2_i82;
  int32_T c2_i83;
  int32_T c2_i84;
  real_T c2_u[9];
  const mxArray *c2_y = NULL;
  SFc2_simiam_AvoidObstacle_BestAOInstanceStruct *chartInstance;
  chartInstance = (SFc2_simiam_AvoidObstacle_BestAOInstanceStruct *)
    chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_i79 = 0;
  for (c2_i80 = 0; c2_i80 < 3; c2_i80++) {
    for (c2_i81 = 0; c2_i81 < 3; c2_i81++) {
      c2_b_inData[c2_i81 + c2_i79] = (*(real_T (*)[9])c2_inData)[c2_i81 + c2_i79];
    }

    c2_i79 += 3;
  }

  c2_i82 = 0;
  for (c2_i83 = 0; c2_i83 < 3; c2_i83++) {
    for (c2_i84 = 0; c2_i84 < 3; c2_i84++) {
      c2_u[c2_i84 + c2_i82] = c2_b_inData[c2_i84 + c2_i82];
    }

    c2_i82 += 3;
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 2, 3, 3), FALSE);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, FALSE);
  return c2_mxArrayOutData;
}

static void c2_h_emlrt_marshallIn(SFc2_simiam_AvoidObstacle_BestAOInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  real_T c2_y[9])
{
  real_T c2_dv7[9];
  int32_T c2_i85;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), c2_dv7, 1, 0, 0U, 1, 0U, 2, 3, 3);
  for (c2_i85 = 0; c2_i85 < 9; c2_i85++) {
    c2_y[c2_i85] = c2_dv7[c2_i85];
  }

  sf_mex_destroy(&c2_u);
}

static void c2_f_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_IR_Transformation_Matrix_I;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y[9];
  int32_T c2_i86;
  int32_T c2_i87;
  int32_T c2_i88;
  SFc2_simiam_AvoidObstacle_BestAOInstanceStruct *chartInstance;
  chartInstance = (SFc2_simiam_AvoidObstacle_BestAOInstanceStruct *)
    chartInstanceVoid;
  c2_IR_Transformation_Matrix_I = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_h_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_IR_Transformation_Matrix_I),
                        &c2_thisId, c2_y);
  sf_mex_destroy(&c2_IR_Transformation_Matrix_I);
  c2_i86 = 0;
  for (c2_i87 = 0; c2_i87 < 3; c2_i87++) {
    for (c2_i88 = 0; c2_i88 < 3; c2_i88++) {
      (*(real_T (*)[9])c2_outData)[c2_i88 + c2_i86] = c2_y[c2_i88 + c2_i86];
    }

    c2_i86 += 3;
  }

  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_h_sf_marshallOut(void *chartInstanceVoid, real_T
  c2_inData_data[9], int32_T c2_inData_sizes[1])
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_b_inData_sizes;
  int32_T c2_loop_ub;
  int32_T c2_i89;
  real_T c2_b_inData_data[9];
  int32_T c2_u_sizes;
  int32_T c2_b_loop_ub;
  int32_T c2_i90;
  real_T c2_u_data[9];
  const mxArray *c2_y = NULL;
  SFc2_simiam_AvoidObstacle_BestAOInstanceStruct *chartInstance;
  chartInstance = (SFc2_simiam_AvoidObstacle_BestAOInstanceStruct *)
    chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_b_inData_sizes = c2_inData_sizes[0];
  c2_loop_ub = c2_inData_sizes[0] - 1;
  for (c2_i89 = 0; c2_i89 <= c2_loop_ub; c2_i89++) {
    c2_b_inData_data[c2_i89] = c2_inData_data[c2_i89];
  }

  c2_u_sizes = c2_b_inData_sizes;
  c2_b_loop_ub = c2_b_inData_sizes - 1;
  for (c2_i90 = 0; c2_i90 <= c2_b_loop_ub; c2_i90++) {
    c2_u_data[c2_i90] = c2_b_inData_data[c2_i90];
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u_data, 0, 0U, 1U, 0U, 1,
    c2_u_sizes), FALSE);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, FALSE);
  return c2_mxArrayOutData;
}

static void c2_i_emlrt_marshallIn(SFc2_simiam_AvoidObstacle_BestAOInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  real_T c2_y_data[9], int32_T c2_y_sizes[1])
{
  static uint32_T c2_uv2[1] = { 9U };

  uint32_T c2_uv3[1];
  static boolean_T c2_bv2[1] = { TRUE };

  boolean_T c2_bv3[1];
  int32_T c2_tmp_sizes;
  real_T c2_tmp_data[9];
  int32_T c2_loop_ub;
  int32_T c2_i91;
  c2_uv3[0] = c2_uv2[0];
  c2_bv3[0] = c2_bv2[0];
  sf_mex_import_vs(c2_parentId, sf_mex_dup(c2_u), c2_tmp_data, 1, 0, 0U, 1, 0U,
                   1, c2_bv3, c2_uv3, &c2_tmp_sizes);
  c2_y_sizes[0] = c2_tmp_sizes;
  c2_loop_ub = c2_tmp_sizes - 1;
  for (c2_i91 = 0; c2_i91 <= c2_loop_ub; c2_i91++) {
    c2_y_data[c2_i91] = c2_tmp_data[c2_i91];
  }

  sf_mex_destroy(&c2_u);
}

static void c2_g_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, real_T c2_outData_data[9],
  int32_T c2_outData_sizes[1])
{
  const mxArray *c2_ObstacleDetected;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  int32_T c2_y_sizes;
  real_T c2_y_data[9];
  int32_T c2_loop_ub;
  int32_T c2_i92;
  SFc2_simiam_AvoidObstacle_BestAOInstanceStruct *chartInstance;
  chartInstance = (SFc2_simiam_AvoidObstacle_BestAOInstanceStruct *)
    chartInstanceVoid;
  c2_ObstacleDetected = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_i_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_ObstacleDetected),
                        &c2_thisId, c2_y_data, *(int32_T (*)[1])&c2_y_sizes);
  sf_mex_destroy(&c2_ObstacleDetected);
  c2_outData_sizes[0] = c2_y_sizes;
  c2_loop_ub = c2_y_sizes - 1;
  for (c2_i92 = 0; c2_i92 <= c2_loop_ub; c2_i92++) {
    c2_outData_data[c2_i92] = c2_y_data[c2_i92];
  }

  sf_mex_destroy(&c2_mxArrayInData);
}

static void c2_j_emlrt_marshallIn(SFc2_simiam_AvoidObstacle_BestAOInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  real_T c2_y[9])
{
  real_T c2_dv8[9];
  int32_T c2_i93;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), c2_dv8, 1, 0, 0U, 1, 0U, 1, 9);
  for (c2_i93 = 0; c2_i93 < 9; c2_i93++) {
    c2_y[c2_i93] = c2_dv8[c2_i93];
  }

  sf_mex_destroy(&c2_u);
}

static void c2_h_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_ir_distances;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y[9];
  int32_T c2_i94;
  SFc2_simiam_AvoidObstacle_BestAOInstanceStruct *chartInstance;
  chartInstance = (SFc2_simiam_AvoidObstacle_BestAOInstanceStruct *)
    chartInstanceVoid;
  c2_ir_distances = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_j_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_ir_distances), &c2_thisId,
                        c2_y);
  sf_mex_destroy(&c2_ir_distances);
  for (c2_i94 = 0; c2_i94 < 9; c2_i94++) {
    (*(real_T (*)[9])c2_outData)[c2_i94] = c2_y[c2_i94];
  }

  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_i_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_i95;
  boolean_T c2_b_inData[9];
  int32_T c2_i96;
  boolean_T c2_u[9];
  const mxArray *c2_y = NULL;
  SFc2_simiam_AvoidObstacle_BestAOInstanceStruct *chartInstance;
  chartInstance = (SFc2_simiam_AvoidObstacle_BestAOInstanceStruct *)
    chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  for (c2_i95 = 0; c2_i95 < 9; c2_i95++) {
    c2_b_inData[c2_i95] = (*(boolean_T (*)[9])c2_inData)[c2_i95];
  }

  for (c2_i96 = 0; c2_i96 < 9; c2_i96++) {
    c2_u[c2_i96] = c2_b_inData[c2_i96];
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 11, 0U, 1U, 0U, 1, 9), FALSE);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, FALSE);
  return c2_mxArrayOutData;
}

static void c2_k_emlrt_marshallIn(SFc2_simiam_AvoidObstacle_BestAOInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  boolean_T c2_y[9])
{
  boolean_T c2_bv4[9];
  int32_T c2_i97;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), c2_bv4, 1, 11, 0U, 1, 0U, 1, 9);
  for (c2_i97 = 0; c2_i97 < 9; c2_i97++) {
    c2_y[c2_i97] = c2_bv4[c2_i97];
  }

  sf_mex_destroy(&c2_u);
}

static void c2_i_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_Number_List;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  boolean_T c2_y[9];
  int32_T c2_i98;
  SFc2_simiam_AvoidObstacle_BestAOInstanceStruct *chartInstance;
  chartInstance = (SFc2_simiam_AvoidObstacle_BestAOInstanceStruct *)
    chartInstanceVoid;
  c2_Number_List = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_k_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_Number_List), &c2_thisId,
                        c2_y);
  sf_mex_destroy(&c2_Number_List);
  for (c2_i98 = 0; c2_i98 < 9; c2_i98++) {
    (*(boolean_T (*)[9])c2_outData)[c2_i98] = c2_y[c2_i98];
  }

  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_j_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_i99;
  int32_T c2_i100;
  int32_T c2_i101;
  real_T c2_b_inData[27];
  int32_T c2_i102;
  int32_T c2_i103;
  int32_T c2_i104;
  real_T c2_u[27];
  const mxArray *c2_y = NULL;
  SFc2_simiam_AvoidObstacle_BestAOInstanceStruct *chartInstance;
  chartInstance = (SFc2_simiam_AvoidObstacle_BestAOInstanceStruct *)
    chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_i99 = 0;
  for (c2_i100 = 0; c2_i100 < 3; c2_i100++) {
    for (c2_i101 = 0; c2_i101 < 9; c2_i101++) {
      c2_b_inData[c2_i101 + c2_i99] = (*(real_T (*)[27])c2_inData)[c2_i101 +
        c2_i99];
    }

    c2_i99 += 9;
  }

  c2_i102 = 0;
  for (c2_i103 = 0; c2_i103 < 3; c2_i103++) {
    for (c2_i104 = 0; c2_i104 < 9; c2_i104++) {
      c2_u[c2_i104 + c2_i102] = c2_b_inData[c2_i104 + c2_i102];
    }

    c2_i102 += 9;
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 2, 9, 3), FALSE);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, FALSE);
  return c2_mxArrayOutData;
}

static void c2_l_emlrt_marshallIn(SFc2_simiam_AvoidObstacle_BestAOInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  real_T c2_y[27])
{
  real_T c2_dv9[27];
  int32_T c2_i105;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), c2_dv9, 1, 0, 0U, 1, 0U, 2, 9, 3);
  for (c2_i105 = 0; c2_i105 < 27; c2_i105++) {
    c2_y[c2_i105] = c2_dv9[c2_i105];
  }

  sf_mex_destroy(&c2_u);
}

static void c2_j_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_IR_Position;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y[27];
  int32_T c2_i106;
  int32_T c2_i107;
  int32_T c2_i108;
  SFc2_simiam_AvoidObstacle_BestAOInstanceStruct *chartInstance;
  chartInstance = (SFc2_simiam_AvoidObstacle_BestAOInstanceStruct *)
    chartInstanceVoid;
  c2_IR_Position = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_l_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_IR_Position), &c2_thisId,
                        c2_y);
  sf_mex_destroy(&c2_IR_Position);
  c2_i106 = 0;
  for (c2_i107 = 0; c2_i107 < 3; c2_i107++) {
    for (c2_i108 = 0; c2_i108 < 9; c2_i108++) {
      (*(real_T (*)[27])c2_outData)[c2_i108 + c2_i106] = c2_y[c2_i108 + c2_i106];
    }

    c2_i106 += 9;
  }

  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_k_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_i109;
  char_T c2_b_inData[15];
  int32_T c2_i110;
  char_T c2_u[15];
  const mxArray *c2_y = NULL;
  SFc2_simiam_AvoidObstacle_BestAOInstanceStruct *chartInstance;
  chartInstance = (SFc2_simiam_AvoidObstacle_BestAOInstanceStruct *)
    chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  for (c2_i109 = 0; c2_i109 < 15; c2_i109++) {
    c2_b_inData[c2_i109] = (*(char_T (*)[15])c2_inData)[c2_i109];
  }

  for (c2_i110 = 0; c2_i110 < 15; c2_i110++) {
    c2_u[c2_i110] = c2_b_inData[c2_i110];
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 10, 0U, 1U, 0U, 2, 1, 15), FALSE);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, FALSE);
  return c2_mxArrayOutData;
}

static void c2_m_emlrt_marshallIn(SFc2_simiam_AvoidObstacle_BestAOInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  char_T c2_y[15])
{
  char_T c2_cv0[15];
  int32_T c2_i111;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), c2_cv0, 1, 10, 0U, 1, 0U, 2, 1,
                15);
  for (c2_i111 = 0; c2_i111 < 15; c2_i111++) {
    c2_y[c2_i111] = c2_cv0[c2_i111];
  }

  sf_mex_destroy(&c2_u);
}

static void c2_k_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_type;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  char_T c2_y[15];
  int32_T c2_i112;
  SFc2_simiam_AvoidObstacle_BestAOInstanceStruct *chartInstance;
  chartInstance = (SFc2_simiam_AvoidObstacle_BestAOInstanceStruct *)
    chartInstanceVoid;
  c2_type = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_m_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_type), &c2_thisId, c2_y);
  sf_mex_destroy(&c2_type);
  for (c2_i112 = 0; c2_i112 < 15; c2_i112++) {
    (*(char_T (*)[15])c2_outData)[c2_i112] = c2_y[c2_i112];
  }

  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_l_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_i113;
  real_T c2_b_inData[9];
  int32_T c2_i114;
  real_T c2_u[9];
  const mxArray *c2_y = NULL;
  SFc2_simiam_AvoidObstacle_BestAOInstanceStruct *chartInstance;
  chartInstance = (SFc2_simiam_AvoidObstacle_BestAOInstanceStruct *)
    chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  for (c2_i113 = 0; c2_i113 < 9; c2_i113++) {
    c2_b_inData[c2_i113] = (*(real_T (*)[9])c2_inData)[c2_i113];
  }

  for (c2_i114 = 0; c2_i114 < 9; c2_i114++) {
    c2_u[c2_i114] = c2_b_inData[c2_i114];
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 2, 1, 9), FALSE);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, FALSE);
  return c2_mxArrayOutData;
}

static void c2_n_emlrt_marshallIn(SFc2_simiam_AvoidObstacle_BestAOInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  real_T c2_y[9])
{
  real_T c2_dv10[9];
  int32_T c2_i115;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), c2_dv10, 1, 0, 0U, 1, 0U, 2, 1, 9);
  for (c2_i115 = 0; c2_i115 < 9; c2_i115++) {
    c2_y[c2_i115] = c2_dv10[c2_i115];
  }

  sf_mex_destroy(&c2_u);
}

static void c2_l_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_IR_Number;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y[9];
  int32_T c2_i116;
  SFc2_simiam_AvoidObstacle_BestAOInstanceStruct *chartInstance;
  chartInstance = (SFc2_simiam_AvoidObstacle_BestAOInstanceStruct *)
    chartInstanceVoid;
  c2_IR_Number = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_n_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_IR_Number), &c2_thisId,
                        c2_y);
  sf_mex_destroy(&c2_IR_Number);
  for (c2_i116 = 0; c2_i116 < 9; c2_i116++) {
    (*(real_T (*)[9])c2_outData)[c2_i116] = c2_y[c2_i116];
  }

  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_m_sf_marshallOut(void *chartInstanceVoid, real_T
  c2_inData_data[9], int32_T c2_inData_sizes[2])
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_b_inData_sizes[2];
  int32_T c2_loop_ub;
  int32_T c2_i117;
  real_T c2_b_inData_data[9];
  int32_T c2_u_sizes[2];
  int32_T c2_b_loop_ub;
  int32_T c2_i118;
  real_T c2_u_data[9];
  const mxArray *c2_y = NULL;
  SFc2_simiam_AvoidObstacle_BestAOInstanceStruct *chartInstance;
  chartInstance = (SFc2_simiam_AvoidObstacle_BestAOInstanceStruct *)
    chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_b_inData_sizes[0] = 1;
  c2_b_inData_sizes[1] = c2_inData_sizes[1];
  c2_loop_ub = c2_inData_sizes[1] - 1;
  for (c2_i117 = 0; c2_i117 <= c2_loop_ub; c2_i117++) {
    c2_b_inData_data[c2_b_inData_sizes[0] * c2_i117] =
      c2_inData_data[c2_inData_sizes[0] * c2_i117];
  }

  c2_u_sizes[0] = 1;
  c2_u_sizes[1] = c2_b_inData_sizes[1];
  c2_b_loop_ub = c2_b_inData_sizes[1] - 1;
  for (c2_i118 = 0; c2_i118 <= c2_b_loop_ub; c2_i118++) {
    c2_u_data[c2_u_sizes[0] * c2_i118] = c2_b_inData_data[c2_b_inData_sizes[0] *
      c2_i118];
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u_data, 0, 0U, 1U, 0U, 2,
    c2_u_sizes[0], c2_u_sizes[1]), FALSE);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, FALSE);
  return c2_mxArrayOutData;
}

static void c2_o_emlrt_marshallIn(SFc2_simiam_AvoidObstacle_BestAOInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  real_T c2_y_data[9], int32_T c2_y_sizes[2])
{
  int32_T c2_i119;
  uint32_T c2_uv4[2];
  int32_T c2_i120;
  static boolean_T c2_bv5[2] = { FALSE, TRUE };

  boolean_T c2_bv6[2];
  int32_T c2_tmp_sizes[2];
  real_T c2_tmp_data[9];
  int32_T c2_y;
  int32_T c2_b_y;
  int32_T c2_loop_ub;
  int32_T c2_i121;
  for (c2_i119 = 0; c2_i119 < 2; c2_i119++) {
    c2_uv4[c2_i119] = 1U + ((uint32_T)c2_i119 << 3);
  }

  for (c2_i120 = 0; c2_i120 < 2; c2_i120++) {
    c2_bv6[c2_i120] = c2_bv5[c2_i120];
  }

  sf_mex_import_vs(c2_parentId, sf_mex_dup(c2_u), c2_tmp_data, 1, 0, 0U, 1, 0U,
                   2, c2_bv6, c2_uv4, c2_tmp_sizes);
  c2_y_sizes[0] = 1;
  c2_y_sizes[1] = c2_tmp_sizes[1];
  c2_y = c2_y_sizes[0];
  c2_b_y = c2_y_sizes[1];
  c2_loop_ub = c2_tmp_sizes[0] * c2_tmp_sizes[1] - 1;
  for (c2_i121 = 0; c2_i121 <= c2_loop_ub; c2_i121++) {
    c2_y_data[c2_i121] = c2_tmp_data[c2_i121];
  }

  sf_mex_destroy(&c2_u);
}

static void c2_m_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, real_T c2_outData_data[9],
  int32_T c2_outData_sizes[2])
{
  const mxArray *c2_u_i_Norm_Fixed;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  int32_T c2_y_sizes[2];
  real_T c2_y_data[9];
  int32_T c2_loop_ub;
  int32_T c2_i122;
  SFc2_simiam_AvoidObstacle_BestAOInstanceStruct *chartInstance;
  chartInstance = (SFc2_simiam_AvoidObstacle_BestAOInstanceStruct *)
    chartInstanceVoid;
  c2_u_i_Norm_Fixed = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_o_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_u_i_Norm_Fixed), &c2_thisId,
                        c2_y_data, c2_y_sizes);
  sf_mex_destroy(&c2_u_i_Norm_Fixed);
  c2_outData_sizes[0] = 1;
  c2_outData_sizes[1] = c2_y_sizes[1];
  c2_loop_ub = c2_y_sizes[1] - 1;
  for (c2_i122 = 0; c2_i122 <= c2_loop_ub; c2_i122++) {
    c2_outData_data[c2_outData_sizes[0] * c2_i122] = c2_y_data[c2_y_sizes[0] *
      c2_i122];
  }

  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_n_sf_marshallOut(void *chartInstanceVoid, real_T
  c2_inData_data[36], int32_T c2_inData_sizes[2])
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_b_inData_sizes[2];
  int32_T c2_loop_ub;
  int32_T c2_i123;
  int32_T c2_i124;
  real_T c2_b_inData_data[36];
  int32_T c2_u_sizes[2];
  int32_T c2_b_loop_ub;
  int32_T c2_i125;
  int32_T c2_i126;
  real_T c2_u_data[36];
  const mxArray *c2_y = NULL;
  SFc2_simiam_AvoidObstacle_BestAOInstanceStruct *chartInstance;
  chartInstance = (SFc2_simiam_AvoidObstacle_BestAOInstanceStruct *)
    chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_b_inData_sizes[0] = 4;
  c2_b_inData_sizes[1] = c2_inData_sizes[1];
  c2_loop_ub = c2_inData_sizes[1] - 1;
  for (c2_i123 = 0; c2_i123 <= c2_loop_ub; c2_i123++) {
    for (c2_i124 = 0; c2_i124 < 4; c2_i124++) {
      c2_b_inData_data[c2_i124 + c2_b_inData_sizes[0] * c2_i123] =
        c2_inData_data[c2_i124 + c2_inData_sizes[0] * c2_i123];
    }
  }

  c2_u_sizes[0] = 4;
  c2_u_sizes[1] = c2_b_inData_sizes[1];
  c2_b_loop_ub = c2_b_inData_sizes[1] - 1;
  for (c2_i125 = 0; c2_i125 <= c2_b_loop_ub; c2_i125++) {
    for (c2_i126 = 0; c2_i126 < 4; c2_i126++) {
      c2_u_data[c2_i126 + c2_u_sizes[0] * c2_i125] = c2_b_inData_data[c2_i126 +
        c2_b_inData_sizes[0] * c2_i125];
    }
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u_data, 0, 0U, 1U, 0U, 2,
    c2_u_sizes[0], c2_u_sizes[1]), FALSE);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, FALSE);
  return c2_mxArrayOutData;
}

static void c2_p_emlrt_marshallIn(SFc2_simiam_AvoidObstacle_BestAOInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  real_T c2_y_data[36], int32_T c2_y_sizes[2])
{
  int32_T c2_i127;
  uint32_T c2_uv5[2];
  int32_T c2_i128;
  static boolean_T c2_bv7[2] = { FALSE, TRUE };

  boolean_T c2_bv8[2];
  int32_T c2_tmp_sizes[2];
  real_T c2_tmp_data[36];
  int32_T c2_y;
  int32_T c2_b_y;
  int32_T c2_loop_ub;
  int32_T c2_i129;
  for (c2_i127 = 0; c2_i127 < 2; c2_i127++) {
    c2_uv5[c2_i127] = 4U + 5U * (uint32_T)c2_i127;
  }

  for (c2_i128 = 0; c2_i128 < 2; c2_i128++) {
    c2_bv8[c2_i128] = c2_bv7[c2_i128];
  }

  sf_mex_import_vs(c2_parentId, sf_mex_dup(c2_u), c2_tmp_data, 1, 0, 0U, 1, 0U,
                   2, c2_bv8, c2_uv5, c2_tmp_sizes);
  c2_y_sizes[0] = 4;
  c2_y_sizes[1] = c2_tmp_sizes[1];
  c2_y = c2_y_sizes[0];
  c2_b_y = c2_y_sizes[1];
  c2_loop_ub = c2_tmp_sizes[0] * c2_tmp_sizes[1] - 1;
  for (c2_i129 = 0; c2_i129 <= c2_loop_ub; c2_i129++) {
    c2_y_data[c2_i129] = c2_tmp_data[c2_i129];
  }

  sf_mex_destroy(&c2_u);
}

static void c2_n_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, real_T c2_outData_data[36],
  int32_T c2_outData_sizes[2])
{
  const mxArray *c2_u_i;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  int32_T c2_y_sizes[2];
  real_T c2_y_data[36];
  int32_T c2_loop_ub;
  int32_T c2_i130;
  int32_T c2_i131;
  SFc2_simiam_AvoidObstacle_BestAOInstanceStruct *chartInstance;
  chartInstance = (SFc2_simiam_AvoidObstacle_BestAOInstanceStruct *)
    chartInstanceVoid;
  c2_u_i = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_p_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_u_i), &c2_thisId, c2_y_data,
                        c2_y_sizes);
  sf_mex_destroy(&c2_u_i);
  c2_outData_sizes[0] = 4;
  c2_outData_sizes[1] = c2_y_sizes[1];
  c2_loop_ub = c2_y_sizes[1] - 1;
  for (c2_i130 = 0; c2_i130 <= c2_loop_ub; c2_i130++) {
    for (c2_i131 = 0; c2_i131 < 4; c2_i131++) {
      c2_outData_data[c2_i131 + c2_outData_sizes[0] * c2_i130] =
        c2_y_data[c2_i131 + c2_y_sizes[0] * c2_i130];
    }
  }

  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_o_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_i132;
  int32_T c2_i133;
  int32_T c2_i134;
  real_T c2_b_inData[36];
  int32_T c2_i135;
  int32_T c2_i136;
  int32_T c2_i137;
  real_T c2_u[36];
  const mxArray *c2_y = NULL;
  SFc2_simiam_AvoidObstacle_BestAOInstanceStruct *chartInstance;
  chartInstance = (SFc2_simiam_AvoidObstacle_BestAOInstanceStruct *)
    chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_i132 = 0;
  for (c2_i133 = 0; c2_i133 < 9; c2_i133++) {
    for (c2_i134 = 0; c2_i134 < 4; c2_i134++) {
      c2_b_inData[c2_i134 + c2_i132] = (*(real_T (*)[36])c2_inData)[c2_i134 +
        c2_i132];
    }

    c2_i132 += 4;
  }

  c2_i135 = 0;
  for (c2_i136 = 0; c2_i136 < 9; c2_i136++) {
    for (c2_i137 = 0; c2_i137 < 4; c2_i137++) {
      c2_u[c2_i137 + c2_i135] = c2_b_inData[c2_i137 + c2_i135];
    }

    c2_i135 += 4;
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 2, 4, 9), FALSE);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, FALSE);
  return c2_mxArrayOutData;
}

static void c2_q_emlrt_marshallIn(SFc2_simiam_AvoidObstacle_BestAOInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  real_T c2_y[36])
{
  real_T c2_dv11[36];
  int32_T c2_i138;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), c2_dv11, 1, 0, 0U, 1, 0U, 2, 4, 9);
  for (c2_i138 = 0; c2_i138 < 36; c2_i138++) {
    c2_y[c2_i138] = c2_dv11[c2_i138];
  }

  sf_mex_destroy(&c2_u);
}

static void c2_o_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_u_i;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y[36];
  int32_T c2_i139;
  int32_T c2_i140;
  int32_T c2_i141;
  SFc2_simiam_AvoidObstacle_BestAOInstanceStruct *chartInstance;
  chartInstance = (SFc2_simiam_AvoidObstacle_BestAOInstanceStruct *)
    chartInstanceVoid;
  c2_u_i = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_q_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_u_i), &c2_thisId, c2_y);
  sf_mex_destroy(&c2_u_i);
  c2_i139 = 0;
  for (c2_i140 = 0; c2_i140 < 9; c2_i140++) {
    for (c2_i141 = 0; c2_i141 < 4; c2_i141++) {
      (*(real_T (*)[36])c2_outData)[c2_i141 + c2_i139] = c2_y[c2_i141 + c2_i139];
    }

    c2_i139 += 4;
  }

  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_p_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_i142;
  real_T c2_b_inData[4];
  int32_T c2_i143;
  real_T c2_u[4];
  const mxArray *c2_y = NULL;
  SFc2_simiam_AvoidObstacle_BestAOInstanceStruct *chartInstance;
  chartInstance = (SFc2_simiam_AvoidObstacle_BestAOInstanceStruct *)
    chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  for (c2_i142 = 0; c2_i142 < 4; c2_i142++) {
    c2_b_inData[c2_i142] = (*(real_T (*)[4])c2_inData)[c2_i142];
  }

  for (c2_i143 = 0; c2_i143 < 4; c2_i143++) {
    c2_u[c2_i143] = c2_b_inData[c2_i143];
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 1, 4), FALSE);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, FALSE);
  return c2_mxArrayOutData;
}

static void c2_r_emlrt_marshallIn(SFc2_simiam_AvoidObstacle_BestAOInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  real_T c2_y[4])
{
  real_T c2_dv12[4];
  int32_T c2_i144;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), c2_dv12, 1, 0, 0U, 1, 0U, 1, 4);
  for (c2_i144 = 0; c2_i144 < 4; c2_i144++) {
    c2_y[c2_i144] = c2_dv12[c2_i144];
  }

  sf_mex_destroy(&c2_u);
}

static void c2_p_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_Pointer_Wall;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y[4];
  int32_T c2_i145;
  SFc2_simiam_AvoidObstacle_BestAOInstanceStruct *chartInstance;
  chartInstance = (SFc2_simiam_AvoidObstacle_BestAOInstanceStruct *)
    chartInstanceVoid;
  c2_Pointer_Wall = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_r_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_Pointer_Wall), &c2_thisId,
                        c2_y);
  sf_mex_destroy(&c2_Pointer_Wall);
  for (c2_i145 = 0; c2_i145 < 4; c2_i145++) {
    (*(real_T (*)[4])c2_outData)[c2_i145] = c2_y[c2_i145];
  }

  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_q_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_i146;
  int32_T c2_i147;
  int32_T c2_i148;
  real_T c2_b_inData[18];
  int32_T c2_i149;
  int32_T c2_i150;
  int32_T c2_i151;
  real_T c2_u[18];
  const mxArray *c2_y = NULL;
  SFc2_simiam_AvoidObstacle_BestAOInstanceStruct *chartInstance;
  chartInstance = (SFc2_simiam_AvoidObstacle_BestAOInstanceStruct *)
    chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_i146 = 0;
  for (c2_i147 = 0; c2_i147 < 9; c2_i147++) {
    for (c2_i148 = 0; c2_i148 < 2; c2_i148++) {
      c2_b_inData[c2_i148 + c2_i146] = (*(real_T (*)[18])c2_inData)[c2_i148 +
        c2_i146];
    }

    c2_i146 += 2;
  }

  c2_i149 = 0;
  for (c2_i150 = 0; c2_i150 < 9; c2_i150++) {
    for (c2_i151 = 0; c2_i151 < 2; c2_i151++) {
      c2_u[c2_i151 + c2_i149] = c2_b_inData[c2_i151 + c2_i149];
    }

    c2_i149 += 2;
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 2, 2, 9), FALSE);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, FALSE);
  return c2_mxArrayOutData;
}

static void c2_s_emlrt_marshallIn(SFc2_simiam_AvoidObstacle_BestAOInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  real_T c2_y[18])
{
  real_T c2_dv13[18];
  int32_T c2_i152;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), c2_dv13, 1, 0, 0U, 1, 0U, 2, 2, 9);
  for (c2_i152 = 0; c2_i152 < 18; c2_i152++) {
    c2_y[c2_i152] = c2_dv13[c2_i152];
  }

  sf_mex_destroy(&c2_u);
}

static void c2_q_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_u_i;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y[18];
  int32_T c2_i153;
  int32_T c2_i154;
  int32_T c2_i155;
  SFc2_simiam_AvoidObstacle_BestAOInstanceStruct *chartInstance;
  chartInstance = (SFc2_simiam_AvoidObstacle_BestAOInstanceStruct *)
    chartInstanceVoid;
  c2_u_i = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_s_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_u_i), &c2_thisId, c2_y);
  sf_mex_destroy(&c2_u_i);
  c2_i153 = 0;
  for (c2_i154 = 0; c2_i154 < 9; c2_i154++) {
    for (c2_i155 = 0; c2_i155 < 2; c2_i155++) {
      (*(real_T (*)[18])c2_outData)[c2_i155 + c2_i153] = c2_y[c2_i155 + c2_i153];
    }

    c2_i153 += 2;
  }

  sf_mex_destroy(&c2_mxArrayInData);
}

const mxArray *sf_c2_simiam_AvoidObstacle_BestAO_get_eml_resolved_functions_info
  (void)
{
  const mxArray *c2_nameCaptureInfo = NULL;
  c2_nameCaptureInfo = NULL;
  sf_mex_assign(&c2_nameCaptureInfo, sf_mex_createstruct("structure", 2, 189, 1),
                FALSE);
  c2_info_helper(&c2_nameCaptureInfo);
  c2_b_info_helper(&c2_nameCaptureInfo);
  c2_c_info_helper(&c2_nameCaptureInfo);
  sf_mex_emlrtNameCapturePostProcessR2012a(&c2_nameCaptureInfo);
  return c2_nameCaptureInfo;
}

static void c2_info_helper(const mxArray **c2_info)
{
  const mxArray *c2_rhs0 = NULL;
  const mxArray *c2_lhs0 = NULL;
  const mxArray *c2_rhs1 = NULL;
  const mxArray *c2_lhs1 = NULL;
  const mxArray *c2_rhs2 = NULL;
  const mxArray *c2_lhs2 = NULL;
  const mxArray *c2_rhs3 = NULL;
  const mxArray *c2_lhs3 = NULL;
  const mxArray *c2_rhs4 = NULL;
  const mxArray *c2_lhs4 = NULL;
  const mxArray *c2_rhs5 = NULL;
  const mxArray *c2_lhs5 = NULL;
  const mxArray *c2_rhs6 = NULL;
  const mxArray *c2_lhs6 = NULL;
  const mxArray *c2_rhs7 = NULL;
  const mxArray *c2_lhs7 = NULL;
  const mxArray *c2_rhs8 = NULL;
  const mxArray *c2_lhs8 = NULL;
  const mxArray *c2_rhs9 = NULL;
  const mxArray *c2_lhs9 = NULL;
  const mxArray *c2_rhs10 = NULL;
  const mxArray *c2_lhs10 = NULL;
  const mxArray *c2_rhs11 = NULL;
  const mxArray *c2_lhs11 = NULL;
  const mxArray *c2_rhs12 = NULL;
  const mxArray *c2_lhs12 = NULL;
  const mxArray *c2_rhs13 = NULL;
  const mxArray *c2_lhs13 = NULL;
  const mxArray *c2_rhs14 = NULL;
  const mxArray *c2_lhs14 = NULL;
  const mxArray *c2_rhs15 = NULL;
  const mxArray *c2_lhs15 = NULL;
  const mxArray *c2_rhs16 = NULL;
  const mxArray *c2_lhs16 = NULL;
  const mxArray *c2_rhs17 = NULL;
  const mxArray *c2_lhs17 = NULL;
  const mxArray *c2_rhs18 = NULL;
  const mxArray *c2_lhs18 = NULL;
  const mxArray *c2_rhs19 = NULL;
  const mxArray *c2_lhs19 = NULL;
  const mxArray *c2_rhs20 = NULL;
  const mxArray *c2_lhs20 = NULL;
  const mxArray *c2_rhs21 = NULL;
  const mxArray *c2_lhs21 = NULL;
  const mxArray *c2_rhs22 = NULL;
  const mxArray *c2_lhs22 = NULL;
  const mxArray *c2_rhs23 = NULL;
  const mxArray *c2_lhs23 = NULL;
  const mxArray *c2_rhs24 = NULL;
  const mxArray *c2_lhs24 = NULL;
  const mxArray *c2_rhs25 = NULL;
  const mxArray *c2_lhs25 = NULL;
  const mxArray *c2_rhs26 = NULL;
  const mxArray *c2_lhs26 = NULL;
  const mxArray *c2_rhs27 = NULL;
  const mxArray *c2_lhs27 = NULL;
  const mxArray *c2_rhs28 = NULL;
  const mxArray *c2_lhs28 = NULL;
  const mxArray *c2_rhs29 = NULL;
  const mxArray *c2_lhs29 = NULL;
  const mxArray *c2_rhs30 = NULL;
  const mxArray *c2_lhs30 = NULL;
  const mxArray *c2_rhs31 = NULL;
  const mxArray *c2_lhs31 = NULL;
  const mxArray *c2_rhs32 = NULL;
  const mxArray *c2_lhs32 = NULL;
  const mxArray *c2_rhs33 = NULL;
  const mxArray *c2_lhs33 = NULL;
  const mxArray *c2_rhs34 = NULL;
  const mxArray *c2_lhs34 = NULL;
  const mxArray *c2_rhs35 = NULL;
  const mxArray *c2_lhs35 = NULL;
  const mxArray *c2_rhs36 = NULL;
  const mxArray *c2_lhs36 = NULL;
  const mxArray *c2_rhs37 = NULL;
  const mxArray *c2_lhs37 = NULL;
  const mxArray *c2_rhs38 = NULL;
  const mxArray *c2_lhs38 = NULL;
  const mxArray *c2_rhs39 = NULL;
  const mxArray *c2_lhs39 = NULL;
  const mxArray *c2_rhs40 = NULL;
  const mxArray *c2_lhs40 = NULL;
  const mxArray *c2_rhs41 = NULL;
  const mxArray *c2_lhs41 = NULL;
  const mxArray *c2_rhs42 = NULL;
  const mxArray *c2_lhs42 = NULL;
  const mxArray *c2_rhs43 = NULL;
  const mxArray *c2_lhs43 = NULL;
  const mxArray *c2_rhs44 = NULL;
  const mxArray *c2_lhs44 = NULL;
  const mxArray *c2_rhs45 = NULL;
  const mxArray *c2_lhs45 = NULL;
  const mxArray *c2_rhs46 = NULL;
  const mxArray *c2_lhs46 = NULL;
  const mxArray *c2_rhs47 = NULL;
  const mxArray *c2_lhs47 = NULL;
  const mxArray *c2_rhs48 = NULL;
  const mxArray *c2_lhs48 = NULL;
  const mxArray *c2_rhs49 = NULL;
  const mxArray *c2_lhs49 = NULL;
  const mxArray *c2_rhs50 = NULL;
  const mxArray *c2_lhs50 = NULL;
  const mxArray *c2_rhs51 = NULL;
  const mxArray *c2_lhs51 = NULL;
  const mxArray *c2_rhs52 = NULL;
  const mxArray *c2_lhs52 = NULL;
  const mxArray *c2_rhs53 = NULL;
  const mxArray *c2_lhs53 = NULL;
  const mxArray *c2_rhs54 = NULL;
  const mxArray *c2_lhs54 = NULL;
  const mxArray *c2_rhs55 = NULL;
  const mxArray *c2_lhs55 = NULL;
  const mxArray *c2_rhs56 = NULL;
  const mxArray *c2_lhs56 = NULL;
  const mxArray *c2_rhs57 = NULL;
  const mxArray *c2_lhs57 = NULL;
  const mxArray *c2_rhs58 = NULL;
  const mxArray *c2_lhs58 = NULL;
  const mxArray *c2_rhs59 = NULL;
  const mxArray *c2_lhs59 = NULL;
  const mxArray *c2_rhs60 = NULL;
  const mxArray *c2_lhs60 = NULL;
  const mxArray *c2_rhs61 = NULL;
  const mxArray *c2_lhs61 = NULL;
  const mxArray *c2_rhs62 = NULL;
  const mxArray *c2_lhs62 = NULL;
  const mxArray *c2_rhs63 = NULL;
  const mxArray *c2_lhs63 = NULL;
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "context", "context", 0);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("find"), "name", "name", 0);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("logical"), "dominantType",
                  "dominantType", 0);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/find.m"), "resolved",
                  "resolved", 0);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1303124606U), "fileTimeLo",
                  "fileTimeLo", 0);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 0);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 0);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 0);
  sf_mex_assign(&c2_rhs0, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs0, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs0), "rhs", "rhs", 0);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs0), "lhs", "lhs", 0);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/find.m!eml_find"),
                  "context", "context", 1);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_index_class"), "name",
                  "name", 1);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 1);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m"),
                  "resolved", "resolved", 1);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1323145378U), "fileTimeLo",
                  "fileTimeLo", 1);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 1);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 1);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 1);
  sf_mex_assign(&c2_rhs1, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs1, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs1), "rhs", "rhs", 1);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs1), "lhs", "lhs", 1);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/find.m!eml_find"),
                  "context", "context", 2);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_scalar_eg"), "name",
                  "name", 2);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("logical"), "dominantType",
                  "dominantType", 2);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "resolved",
                  "resolved", 2);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1286797196U), "fileTimeLo",
                  "fileTimeLo", 2);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 2);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 2);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 2);
  sf_mex_assign(&c2_rhs2, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs2, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs2), "rhs", "rhs", 2);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs2), "lhs", "lhs", 2);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/find.m!eml_find"),
                  "context", "context", 3);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "eml_int_forloop_overflow_check"), "name", "name", 3);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 3);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m"),
                  "resolved", "resolved", 3);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1346488740U), "fileTimeLo",
                  "fileTimeLo", 3);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 3);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 3);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 3);
  sf_mex_assign(&c2_rhs3, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs3, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs3), "rhs", "rhs", 3);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs3), "lhs", "lhs", 3);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m!eml_int_forloop_overflow_check_helper"),
                  "context", "context", 4);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("intmax"), "name", "name", 4);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 4);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m"), "resolved",
                  "resolved", 4);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1362236682U), "fileTimeLo",
                  "fileTimeLo", 4);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 4);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 4);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 4);
  sf_mex_assign(&c2_rhs4, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs4, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs4), "rhs", "rhs", 4);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs4), "lhs", "lhs", 4);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/find.m!eml_find"),
                  "context", "context", 5);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_index_plus"), "name",
                  "name", 5);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 5);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m"),
                  "resolved", "resolved", 5);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1286797178U), "fileTimeLo",
                  "fileTimeLo", 5);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 5);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 5);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 5);
  sf_mex_assign(&c2_rhs5, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs5, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs5), "rhs", "rhs", 5);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs5), "lhs", "lhs", 5);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m"), "context",
                  "context", 6);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_index_class"), "name",
                  "name", 6);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 6);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m"),
                  "resolved", "resolved", 6);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1323145378U), "fileTimeLo",
                  "fileTimeLo", 6);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 6);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 6);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 6);
  sf_mex_assign(&c2_rhs6, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs6, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs6), "rhs", "rhs", 6);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs6), "lhs", "lhs", 6);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "context", "context", 7);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_li_find"), "name", "name",
                  7);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 7);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_li_find.m"), "resolved",
                  "resolved", 7);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1286797186U), "fileTimeLo",
                  "fileTimeLo", 7);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 7);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 7);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 7);
  sf_mex_assign(&c2_rhs7, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs7, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs7), "rhs", "rhs", 7);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs7), "lhs", "lhs", 7);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_li_find.m"), "context",
                  "context", 8);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_index_class"), "name",
                  "name", 8);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 8);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m"),
                  "resolved", "resolved", 8);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1323145378U), "fileTimeLo",
                  "fileTimeLo", 8);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 8);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 8);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 8);
  sf_mex_assign(&c2_rhs8, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs8, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs8), "rhs", "rhs", 8);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs8), "lhs", "lhs", 8);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_li_find.m!compute_nones"),
                  "context", "context", 9);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_index_class"), "name",
                  "name", 9);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 9);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m"),
                  "resolved", "resolved", 9);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1323145378U), "fileTimeLo",
                  "fileTimeLo", 9);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 9);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 9);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 9);
  sf_mex_assign(&c2_rhs9, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs9, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs9), "rhs", "rhs", 9);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs9), "lhs", "lhs", 9);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_li_find.m!compute_nones"),
                  "context", "context", 10);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "eml_int_forloop_overflow_check"), "name", "name", 10);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 10);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m"),
                  "resolved", "resolved", 10);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1346488740U), "fileTimeLo",
                  "fileTimeLo", 10);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 10);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 10);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 10);
  sf_mex_assign(&c2_rhs10, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs10, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs10), "rhs", "rhs",
                  10);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs10), "lhs", "lhs",
                  10);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_li_find.m!compute_nones"),
                  "context", "context", 11);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_index_plus"), "name",
                  "name", 11);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 11);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m"),
                  "resolved", "resolved", 11);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1286797178U), "fileTimeLo",
                  "fileTimeLo", 11);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 11);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 11);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 11);
  sf_mex_assign(&c2_rhs11, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs11, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs11), "rhs", "rhs",
                  11);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs11), "lhs", "lhs",
                  11);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_li_find.m"), "context",
                  "context", 12);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "eml_int_forloop_overflow_check"), "name", "name", 12);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 12);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m"),
                  "resolved", "resolved", 12);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1346488740U), "fileTimeLo",
                  "fileTimeLo", 12);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 12);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 12);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 12);
  sf_mex_assign(&c2_rhs12, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs12, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs12), "rhs", "rhs",
                  12);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs12), "lhs", "lhs",
                  12);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_li_find.m"), "context",
                  "context", 13);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_index_plus"), "name",
                  "name", 13);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 13);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m"),
                  "resolved", "resolved", 13);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1286797178U), "fileTimeLo",
                  "fileTimeLo", 13);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 13);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 13);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 13);
  sf_mex_assign(&c2_rhs13, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs13, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs13), "rhs", "rhs",
                  13);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs13), "lhs", "lhs",
                  13);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "context", "context", 14);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("mrdivide"), "name", "name", 14);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 14);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p"), "resolved",
                  "resolved", 14);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1373284908U), "fileTimeLo",
                  "fileTimeLo", 14);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 14);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1319708366U), "mFileTimeLo",
                  "mFileTimeLo", 14);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 14);
  sf_mex_assign(&c2_rhs14, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs14, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs14), "rhs", "rhs",
                  14);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs14), "lhs", "lhs",
                  14);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p"), "context",
                  "context", 15);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("rdivide"), "name", "name", 15);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 15);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m"), "resolved",
                  "resolved", 15);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1363688680U), "fileTimeLo",
                  "fileTimeLo", 15);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 15);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 15);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 15);
  sf_mex_assign(&c2_rhs15, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs15, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs15), "rhs", "rhs",
                  15);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs15), "lhs", "lhs",
                  15);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m"), "context",
                  "context", 16);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 16);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 16);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 16);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1363689356U), "fileTimeLo",
                  "fileTimeLo", 16);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 16);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 16);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 16);
  sf_mex_assign(&c2_rhs16, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs16, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs16), "rhs", "rhs",
                  16);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs16), "lhs", "lhs",
                  16);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m"), "context",
                  "context", 17);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_scalexp_compatible"),
                  "name", "name", 17);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 17);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_compatible.m"),
                  "resolved", "resolved", 17);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1286797196U), "fileTimeLo",
                  "fileTimeLo", 17);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 17);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 17);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 17);
  sf_mex_assign(&c2_rhs17, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs17, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs17), "rhs", "rhs",
                  17);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs17), "lhs", "lhs",
                  17);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/rdivide.m"), "context",
                  "context", 18);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_div"), "name", "name", 18);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 18);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_div.m"), "resolved",
                  "resolved", 18);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1363688666U), "fileTimeLo",
                  "fileTimeLo", 18);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 18);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 18);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 18);
  sf_mex_assign(&c2_rhs18, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs18, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs18), "rhs", "rhs",
                  18);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs18), "lhs", "lhs",
                  18);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "context", "context", 19);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("reallog"), "name", "name", 19);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 19);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/reallog.m"), "resolved",
                  "resolved", 19);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1343808784U), "fileTimeLo",
                  "fileTimeLo", 19);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 19);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 19);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 19);
  sf_mex_assign(&c2_rhs19, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs19, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs19), "rhs", "rhs",
                  19);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs19), "lhs", "lhs",
                  19);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/reallog.m"), "context",
                  "context", 20);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_error"), "name", "name",
                  20);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 20);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_error.m"), "resolved",
                  "resolved", 20);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1343808758U), "fileTimeLo",
                  "fileTimeLo", 20);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 20);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 20);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 20);
  sf_mex_assign(&c2_rhs20, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs20, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs20), "rhs", "rhs",
                  20);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs20), "lhs", "lhs",
                  20);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/reallog.m"), "context",
                  "context", 21);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_scalar_reallog"), "name",
                  "name", 21);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 21);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_reallog.m"),
                  "resolved", "resolved", 21);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1286797132U), "fileTimeLo",
                  "fileTimeLo", 21);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 21);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 21);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 21);
  sf_mex_assign(&c2_rhs21, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs21, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs21), "rhs", "rhs",
                  21);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs21), "lhs", "lhs",
                  21);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "context", "context", 22);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("mtimes"), "name", "name", 22);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 22);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m"), "resolved",
                  "resolved", 22);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1363688678U), "fileTimeLo",
                  "fileTimeLo", 22);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 22);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 22);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 22);
  sf_mex_assign(&c2_rhs22, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs22, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs22), "rhs", "rhs",
                  22);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs22), "lhs", "lhs",
                  22);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m!common_checks"),
                  "context", "context", 23);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 23);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 23);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 23);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1363689356U), "fileTimeLo",
                  "fileTimeLo", 23);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 23);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 23);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 23);
  sf_mex_assign(&c2_rhs23, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs23, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs23), "rhs", "rhs",
                  23);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs23), "lhs", "lhs",
                  23);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "context", "context", 24);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("simiam.ui.Pose2D"), "name",
                  "name", 24);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 24);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[C]F:/BlueBird/Study/Robot/FollowWall/20140301_DynamicPoint/+simiam/+ui/Pose2D.m"),
                  "resolved", "resolved", 24);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1389257563U), "fileTimeLo",
                  "fileTimeLo", 24);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 24);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 24);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 24);
  sf_mex_assign(&c2_rhs24, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs24, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs24), "rhs", "rhs",
                  24);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs24), "lhs", "lhs",
                  24);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[C]F:/BlueBird/Study/Robot/FollowWall/20140301_DynamicPoint/+simiam/+ui/Pose2D.m"),
                  "context", "context", 25);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("mtimes"), "name", "name", 25);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 25);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m"), "resolved",
                  "resolved", 25);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1363688678U), "fileTimeLo",
                  "fileTimeLo", 25);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 25);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 25);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 25);
  sf_mex_assign(&c2_rhs25, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs25, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs25), "rhs", "rhs",
                  25);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs25), "lhs", "lhs",
                  25);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[C]F:/BlueBird/Study/Robot/FollowWall/20140301_DynamicPoint/+simiam/+ui/Pose2D.m"),
                  "context", "context", 26);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("mrdivide"), "name", "name", 26);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 26);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p"), "resolved",
                  "resolved", 26);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1373284908U), "fileTimeLo",
                  "fileTimeLo", 26);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 26);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1319708366U), "mFileTimeLo",
                  "mFileTimeLo", 26);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 26);
  sf_mex_assign(&c2_rhs26, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs26, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs26), "rhs", "rhs",
                  26);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs26), "lhs", "lhs",
                  26);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[C]F:/BlueBird/Study/Robot/FollowWall/20140301_DynamicPoint/+simiam/+controller/AvoidObstacles.m"),
                  "context", "context", 27);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("simiam.controller.Controller"),
                  "name", "name", 27);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("unknown"), "dominantType",
                  "dominantType", 27);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[C]F:/BlueBird/Study/Robot/FollowWall/20140301_DynamicPoint/+simiam/+controller/Controller.m"),
                  "resolved", "resolved", 27);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1389274949U), "fileTimeLo",
                  "fileTimeLo", 27);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 27);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 27);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 27);
  sf_mex_assign(&c2_rhs27, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs27, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs27), "rhs", "rhs",
                  27);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs27), "lhs", "lhs",
                  27);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "context", "context", 28);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "simiam.controller.AvoidObstacles"), "name", "name", 28);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 28);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[C]F:/BlueBird/Study/Robot/FollowWall/20140301_DynamicPoint/+simiam/+controller/AvoidObstacles.m"),
                  "resolved", "resolved", 28);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1389515879U), "fileTimeLo",
                  "fileTimeLo", 28);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 28);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 28);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 28);
  sf_mex_assign(&c2_rhs28, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs28, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs28), "rhs", "rhs",
                  28);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs28), "lhs", "lhs",
                  28);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[C]F:/BlueBird/Study/Robot/FollowWall/20140301_DynamicPoint/+simiam/+controller/AvoidObstacles.m"),
                  "context", "context", 29);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("cos"), "name", "name", 29);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 29);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/cos.m"), "resolved",
                  "resolved", 29);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1343808772U), "fileTimeLo",
                  "fileTimeLo", 29);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 29);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 29);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 29);
  sf_mex_assign(&c2_rhs29, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs29, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs29), "rhs", "rhs",
                  29);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs29), "lhs", "lhs",
                  29);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/cos.m"), "context",
                  "context", 30);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_scalar_cos"), "name",
                  "name", 30);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 30);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_cos.m"),
                  "resolved", "resolved", 30);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1286797122U), "fileTimeLo",
                  "fileTimeLo", 30);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 30);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 30);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 30);
  sf_mex_assign(&c2_rhs30, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs30, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs30), "rhs", "rhs",
                  30);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs30), "lhs", "lhs",
                  30);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[C]F:/BlueBird/Study/Robot/FollowWall/20140301_DynamicPoint/+simiam/+controller/AvoidObstacles.m"),
                  "context", "context", 31);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("sin"), "name", "name", 31);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 31);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sin.m"), "resolved",
                  "resolved", 31);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1343808786U), "fileTimeLo",
                  "fileTimeLo", 31);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 31);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 31);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 31);
  sf_mex_assign(&c2_rhs31, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs31, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs31), "rhs", "rhs",
                  31);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs31), "lhs", "lhs",
                  31);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sin.m"), "context",
                  "context", 32);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_scalar_sin"), "name",
                  "name", 32);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 32);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_sin.m"),
                  "resolved", "resolved", 32);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1286797136U), "fileTimeLo",
                  "fileTimeLo", 32);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 32);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 32);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 32);
  sf_mex_assign(&c2_rhs32, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs32, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs32), "rhs", "rhs",
                  32);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs32), "lhs", "lhs",
                  32);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m"), "context",
                  "context", 33);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_index_class"), "name",
                  "name", 33);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 33);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m"),
                  "resolved", "resolved", 33);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1323145378U), "fileTimeLo",
                  "fileTimeLo", 33);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 33);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 33);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 33);
  sf_mex_assign(&c2_rhs33, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs33, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs33), "rhs", "rhs",
                  33);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs33), "lhs", "lhs",
                  33);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m"), "context",
                  "context", 34);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_scalar_eg"), "name",
                  "name", 34);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 34);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "resolved",
                  "resolved", 34);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1286797196U), "fileTimeLo",
                  "fileTimeLo", 34);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 34);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 34);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 34);
  sf_mex_assign(&c2_rhs34, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs34, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs34), "rhs", "rhs",
                  34);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs34), "lhs", "lhs",
                  34);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m"), "context",
                  "context", 35);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_xgemm"), "name", "name",
                  35);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 35);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgemm.m"),
                  "resolved", "resolved", 35);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1363688670U), "fileTimeLo",
                  "fileTimeLo", 35);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 35);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 35);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 35);
  sf_mex_assign(&c2_rhs35, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs35, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs35), "rhs", "rhs",
                  35);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs35), "lhs", "lhs",
                  35);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xgemm.m"), "context",
                  "context", 36);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_blas_inline"), "name",
                  "name", 36);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 36);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m"),
                  "resolved", "resolved", 36);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1299051568U), "fileTimeLo",
                  "fileTimeLo", 36);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 36);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 36);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 36);
  sf_mex_assign(&c2_rhs36, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs36, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs36), "rhs", "rhs",
                  36);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs36), "lhs", "lhs",
                  36);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xgemm.m!below_threshold"),
                  "context", "context", 37);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("mtimes"), "name", "name", 37);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 37);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m"), "resolved",
                  "resolved", 37);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1363688678U), "fileTimeLo",
                  "fileTimeLo", 37);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 37);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 37);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 37);
  sf_mex_assign(&c2_rhs37, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs37, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs37), "rhs", "rhs",
                  37);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs37), "lhs", "lhs",
                  37);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xgemm.m"),
                  "context", "context", 38);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_index_class"), "name",
                  "name", 38);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 38);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m"),
                  "resolved", "resolved", 38);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1323145378U), "fileTimeLo",
                  "fileTimeLo", 38);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 38);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 38);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 38);
  sf_mex_assign(&c2_rhs38, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs38, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs38), "rhs", "rhs",
                  38);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs38), "lhs", "lhs",
                  38);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xgemm.m"),
                  "context", "context", 39);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_scalar_eg"), "name",
                  "name", 39);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 39);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "resolved",
                  "resolved", 39);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1286797196U), "fileTimeLo",
                  "fileTimeLo", 39);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 39);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 39);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 39);
  sf_mex_assign(&c2_rhs39, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs39, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs39), "rhs", "rhs",
                  39);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs39), "lhs", "lhs",
                  39);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xgemm.m"),
                  "context", "context", 40);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_refblas_xgemm"), "name",
                  "name", 40);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 40);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xgemm.m"),
                  "resolved", "resolved", 40);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1360257150U), "fileTimeLo",
                  "fileTimeLo", 40);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 40);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 40);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 40);
  sf_mex_assign(&c2_rhs40, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs40, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs40), "rhs", "rhs",
                  40);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs40), "lhs", "lhs",
                  40);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "context", "context", 41);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("length"), "name", "name", 41);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 41);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/length.m"), "resolved",
                  "resolved", 41);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1303124606U), "fileTimeLo",
                  "fileTimeLo", 41);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 41);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 41);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 41);
  sf_mex_assign(&c2_rhs41, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs41, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs41), "rhs", "rhs",
                  41);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs41), "lhs", "lhs",
                  41);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "context", "context", 42);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("repmat"), "name", "name", 42);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 42);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/repmat.m"), "resolved",
                  "resolved", 42);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1352399660U), "fileTimeLo",
                  "fileTimeLo", 42);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 42);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 42);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 42);
  sf_mex_assign(&c2_rhs42, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs42, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs42), "rhs", "rhs",
                  42);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs42), "lhs", "lhs",
                  42);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/repmat.m"), "context",
                  "context", 43);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_assert_valid_size_arg"),
                  "name", "name", 43);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 43);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m"),
                  "resolved", "resolved", 43);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1368161430U), "fileTimeLo",
                  "fileTimeLo", 43);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 43);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 43);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 43);
  sf_mex_assign(&c2_rhs43, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs43, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs43), "rhs", "rhs",
                  43);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs43), "lhs", "lhs",
                  43);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m"),
                  "context", "context", 44);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 44);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 44);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 44);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1363689356U), "fileTimeLo",
                  "fileTimeLo", 44);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 44);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 44);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 44);
  sf_mex_assign(&c2_rhs44, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs44, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs44), "rhs", "rhs",
                  44);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs44), "lhs", "lhs",
                  44);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m!isintegral"),
                  "context", "context", 45);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("isinf"), "name", "name", 45);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 45);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isinf.m"), "resolved",
                  "resolved", 45);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1363688656U), "fileTimeLo",
                  "fileTimeLo", 45);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 45);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 45);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 45);
  sf_mex_assign(&c2_rhs45, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs45, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs45), "rhs", "rhs",
                  45);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs45), "lhs", "lhs",
                  45);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isinf.m"), "context",
                  "context", 46);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 46);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 46);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 46);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1363689356U), "fileTimeLo",
                  "fileTimeLo", 46);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 46);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 46);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 46);
  sf_mex_assign(&c2_rhs46, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs46, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs46), "rhs", "rhs",
                  46);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs46), "lhs", "lhs",
                  46);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m!isinbounds"),
                  "context", "context", 47);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_is_integer_class"), "name",
                  "name", 47);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 47);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_is_integer_class.m"),
                  "resolved", "resolved", 47);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1286797182U), "fileTimeLo",
                  "fileTimeLo", 47);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 47);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 47);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 47);
  sf_mex_assign(&c2_rhs47, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs47, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs47), "rhs", "rhs",
                  47);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs47), "lhs", "lhs",
                  47);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m!isinbounds"),
                  "context", "context", 48);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("intmax"), "name", "name", 48);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 48);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m"), "resolved",
                  "resolved", 48);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1362236682U), "fileTimeLo",
                  "fileTimeLo", 48);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 48);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 48);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 48);
  sf_mex_assign(&c2_rhs48, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs48, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs48), "rhs", "rhs",
                  48);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs48), "lhs", "lhs",
                  48);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m!isinbounds"),
                  "context", "context", 49);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("intmin"), "name", "name", 49);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 49);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmin.m"), "resolved",
                  "resolved", 49);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1362236682U), "fileTimeLo",
                  "fileTimeLo", 49);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 49);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 49);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 49);
  sf_mex_assign(&c2_rhs49, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs49, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs49), "rhs", "rhs",
                  49);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs49), "lhs", "lhs",
                  49);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m!isinbounds"),
                  "context", "context", 50);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.indexIntRelop"),
                  "name", "name", 50);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 50);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexIntRelop.m"),
                  "resolved", "resolved", 50);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1326703122U), "fileTimeLo",
                  "fileTimeLo", 50);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 50);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 50);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 50);
  sf_mex_assign(&c2_rhs50, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs50, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs50), "rhs", "rhs",
                  50);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs50), "lhs", "lhs",
                  50);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexIntRelop.m!float_class_contains_indexIntClass"),
                  "context", "context", 51);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_float_model"), "name",
                  "name", 51);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 51);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_float_model.m"),
                  "resolved", "resolved", 51);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1326702796U), "fileTimeLo",
                  "fileTimeLo", 51);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 51);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 51);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 51);
  sf_mex_assign(&c2_rhs51, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs51, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs51), "rhs", "rhs",
                  51);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs51), "lhs", "lhs",
                  51);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/indexIntRelop.m!is_signed_indexIntClass"),
                  "context", "context", 52);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("intmin"), "name", "name", 52);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 52);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmin.m"), "resolved",
                  "resolved", 52);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1362236682U), "fileTimeLo",
                  "fileTimeLo", 52);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 52);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 52);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 52);
  sf_mex_assign(&c2_rhs52, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs52, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs52), "rhs", "rhs",
                  52);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs52), "lhs", "lhs",
                  52);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m!numel_for_size"),
                  "context", "context", 53);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("mtimes"), "name", "name", 53);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 53);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m"), "resolved",
                  "resolved", 53);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1363688678U), "fileTimeLo",
                  "fileTimeLo", 53);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 53);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 53);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 53);
  sf_mex_assign(&c2_rhs53, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs53, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs53), "rhs", "rhs",
                  53);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs53), "lhs", "lhs",
                  53);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m"),
                  "context", "context", 54);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_index_class"), "name",
                  "name", 54);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 54);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m"),
                  "resolved", "resolved", 54);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1323145378U), "fileTimeLo",
                  "fileTimeLo", 54);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 54);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 54);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 54);
  sf_mex_assign(&c2_rhs54, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs54, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs54), "rhs", "rhs",
                  54);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs54), "lhs", "lhs",
                  54);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_assert_valid_size_arg.m"),
                  "context", "context", 55);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("intmax"), "name", "name", 55);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 55);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m"), "resolved",
                  "resolved", 55);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1362236682U), "fileTimeLo",
                  "fileTimeLo", 55);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 55);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 55);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 55);
  sf_mex_assign(&c2_rhs55, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs55, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs55), "rhs", "rhs",
                  55);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs55), "lhs", "lhs",
                  55);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/repmat.m"), "context",
                  "context", 56);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_index_class"), "name",
                  "name", 56);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 56);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m"),
                  "resolved", "resolved", 56);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1323145378U), "fileTimeLo",
                  "fileTimeLo", 56);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 56);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 56);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 56);
  sf_mex_assign(&c2_rhs56, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs56, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs56), "rhs", "rhs",
                  56);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs56), "lhs", "lhs",
                  56);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/repmat.m"), "context",
                  "context", 57);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_index_minus"), "name",
                  "name", 57);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 57);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_minus.m"),
                  "resolved", "resolved", 57);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1286797178U), "fileTimeLo",
                  "fileTimeLo", 57);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 57);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 57);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 57);
  sf_mex_assign(&c2_rhs57, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs57, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs57), "rhs", "rhs",
                  57);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs57), "lhs", "lhs",
                  57);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_minus.m"),
                  "context", "context", 58);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_index_class"), "name",
                  "name", 58);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 58);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m"),
                  "resolved", "resolved", 58);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1323145378U), "fileTimeLo",
                  "fileTimeLo", 58);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 58);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 58);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 58);
  sf_mex_assign(&c2_rhs58, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs58, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs58), "rhs", "rhs",
                  58);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs58), "lhs", "lhs",
                  58);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/repmat.m"), "context",
                  "context", 59);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_scalar_eg"), "name",
                  "name", 59);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 59);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "resolved",
                  "resolved", 59);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1286797196U), "fileTimeLo",
                  "fileTimeLo", 59);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 59);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 59);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 59);
  sf_mex_assign(&c2_rhs59, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs59, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs59), "rhs", "rhs",
                  59);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs59), "lhs", "lhs",
                  59);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/repmat.m"), "context",
                  "context", 60);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_index_prod"), "name",
                  "name", 60);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 60);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_prod.m"),
                  "resolved", "resolved", 60);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1286797180U), "fileTimeLo",
                  "fileTimeLo", 60);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 60);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 60);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 60);
  sf_mex_assign(&c2_rhs60, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs60, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs60), "rhs", "rhs",
                  60);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs60), "lhs", "lhs",
                  60);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_prod.m"), "context",
                  "context", 61);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_index_class"), "name",
                  "name", 61);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 61);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m"),
                  "resolved", "resolved", 61);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1323145378U), "fileTimeLo",
                  "fileTimeLo", 61);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 61);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 61);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 61);
  sf_mex_assign(&c2_rhs61, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs61, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs61), "rhs", "rhs",
                  61);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs61), "lhs", "lhs",
                  61);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_prod.m"), "context",
                  "context", 62);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_index_times"), "name",
                  "name", 62);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 62);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_times.m"),
                  "resolved", "resolved", 62);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1286797180U), "fileTimeLo",
                  "fileTimeLo", 62);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 62);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 62);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 62);
  sf_mex_assign(&c2_rhs62, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs62, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs62), "rhs", "rhs",
                  62);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs62), "lhs", "lhs",
                  62);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_times.m"),
                  "context", "context", 63);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_index_class"), "name",
                  "name", 63);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 63);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m"),
                  "resolved", "resolved", 63);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1323145378U), "fileTimeLo",
                  "fileTimeLo", 63);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 63);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 63);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 63);
  sf_mex_assign(&c2_rhs63, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs63, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs63), "rhs", "rhs",
                  63);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs63), "lhs", "lhs",
                  63);
  sf_mex_destroy(&c2_rhs0);
  sf_mex_destroy(&c2_lhs0);
  sf_mex_destroy(&c2_rhs1);
  sf_mex_destroy(&c2_lhs1);
  sf_mex_destroy(&c2_rhs2);
  sf_mex_destroy(&c2_lhs2);
  sf_mex_destroy(&c2_rhs3);
  sf_mex_destroy(&c2_lhs3);
  sf_mex_destroy(&c2_rhs4);
  sf_mex_destroy(&c2_lhs4);
  sf_mex_destroy(&c2_rhs5);
  sf_mex_destroy(&c2_lhs5);
  sf_mex_destroy(&c2_rhs6);
  sf_mex_destroy(&c2_lhs6);
  sf_mex_destroy(&c2_rhs7);
  sf_mex_destroy(&c2_lhs7);
  sf_mex_destroy(&c2_rhs8);
  sf_mex_destroy(&c2_lhs8);
  sf_mex_destroy(&c2_rhs9);
  sf_mex_destroy(&c2_lhs9);
  sf_mex_destroy(&c2_rhs10);
  sf_mex_destroy(&c2_lhs10);
  sf_mex_destroy(&c2_rhs11);
  sf_mex_destroy(&c2_lhs11);
  sf_mex_destroy(&c2_rhs12);
  sf_mex_destroy(&c2_lhs12);
  sf_mex_destroy(&c2_rhs13);
  sf_mex_destroy(&c2_lhs13);
  sf_mex_destroy(&c2_rhs14);
  sf_mex_destroy(&c2_lhs14);
  sf_mex_destroy(&c2_rhs15);
  sf_mex_destroy(&c2_lhs15);
  sf_mex_destroy(&c2_rhs16);
  sf_mex_destroy(&c2_lhs16);
  sf_mex_destroy(&c2_rhs17);
  sf_mex_destroy(&c2_lhs17);
  sf_mex_destroy(&c2_rhs18);
  sf_mex_destroy(&c2_lhs18);
  sf_mex_destroy(&c2_rhs19);
  sf_mex_destroy(&c2_lhs19);
  sf_mex_destroy(&c2_rhs20);
  sf_mex_destroy(&c2_lhs20);
  sf_mex_destroy(&c2_rhs21);
  sf_mex_destroy(&c2_lhs21);
  sf_mex_destroy(&c2_rhs22);
  sf_mex_destroy(&c2_lhs22);
  sf_mex_destroy(&c2_rhs23);
  sf_mex_destroy(&c2_lhs23);
  sf_mex_destroy(&c2_rhs24);
  sf_mex_destroy(&c2_lhs24);
  sf_mex_destroy(&c2_rhs25);
  sf_mex_destroy(&c2_lhs25);
  sf_mex_destroy(&c2_rhs26);
  sf_mex_destroy(&c2_lhs26);
  sf_mex_destroy(&c2_rhs27);
  sf_mex_destroy(&c2_lhs27);
  sf_mex_destroy(&c2_rhs28);
  sf_mex_destroy(&c2_lhs28);
  sf_mex_destroy(&c2_rhs29);
  sf_mex_destroy(&c2_lhs29);
  sf_mex_destroy(&c2_rhs30);
  sf_mex_destroy(&c2_lhs30);
  sf_mex_destroy(&c2_rhs31);
  sf_mex_destroy(&c2_lhs31);
  sf_mex_destroy(&c2_rhs32);
  sf_mex_destroy(&c2_lhs32);
  sf_mex_destroy(&c2_rhs33);
  sf_mex_destroy(&c2_lhs33);
  sf_mex_destroy(&c2_rhs34);
  sf_mex_destroy(&c2_lhs34);
  sf_mex_destroy(&c2_rhs35);
  sf_mex_destroy(&c2_lhs35);
  sf_mex_destroy(&c2_rhs36);
  sf_mex_destroy(&c2_lhs36);
  sf_mex_destroy(&c2_rhs37);
  sf_mex_destroy(&c2_lhs37);
  sf_mex_destroy(&c2_rhs38);
  sf_mex_destroy(&c2_lhs38);
  sf_mex_destroy(&c2_rhs39);
  sf_mex_destroy(&c2_lhs39);
  sf_mex_destroy(&c2_rhs40);
  sf_mex_destroy(&c2_lhs40);
  sf_mex_destroy(&c2_rhs41);
  sf_mex_destroy(&c2_lhs41);
  sf_mex_destroy(&c2_rhs42);
  sf_mex_destroy(&c2_lhs42);
  sf_mex_destroy(&c2_rhs43);
  sf_mex_destroy(&c2_lhs43);
  sf_mex_destroy(&c2_rhs44);
  sf_mex_destroy(&c2_lhs44);
  sf_mex_destroy(&c2_rhs45);
  sf_mex_destroy(&c2_lhs45);
  sf_mex_destroy(&c2_rhs46);
  sf_mex_destroy(&c2_lhs46);
  sf_mex_destroy(&c2_rhs47);
  sf_mex_destroy(&c2_lhs47);
  sf_mex_destroy(&c2_rhs48);
  sf_mex_destroy(&c2_lhs48);
  sf_mex_destroy(&c2_rhs49);
  sf_mex_destroy(&c2_lhs49);
  sf_mex_destroy(&c2_rhs50);
  sf_mex_destroy(&c2_lhs50);
  sf_mex_destroy(&c2_rhs51);
  sf_mex_destroy(&c2_lhs51);
  sf_mex_destroy(&c2_rhs52);
  sf_mex_destroy(&c2_lhs52);
  sf_mex_destroy(&c2_rhs53);
  sf_mex_destroy(&c2_lhs53);
  sf_mex_destroy(&c2_rhs54);
  sf_mex_destroy(&c2_lhs54);
  sf_mex_destroy(&c2_rhs55);
  sf_mex_destroy(&c2_lhs55);
  sf_mex_destroy(&c2_rhs56);
  sf_mex_destroy(&c2_lhs56);
  sf_mex_destroy(&c2_rhs57);
  sf_mex_destroy(&c2_lhs57);
  sf_mex_destroy(&c2_rhs58);
  sf_mex_destroy(&c2_lhs58);
  sf_mex_destroy(&c2_rhs59);
  sf_mex_destroy(&c2_lhs59);
  sf_mex_destroy(&c2_rhs60);
  sf_mex_destroy(&c2_lhs60);
  sf_mex_destroy(&c2_rhs61);
  sf_mex_destroy(&c2_lhs61);
  sf_mex_destroy(&c2_rhs62);
  sf_mex_destroy(&c2_lhs62);
  sf_mex_destroy(&c2_rhs63);
  sf_mex_destroy(&c2_lhs63);
}

static const mxArray *c2_emlrt_marshallOut(char * c2_u)
{
  const mxArray *c2_y = NULL;
  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 15, 0U, 0U, 0U, 2, 1, strlen
    (c2_u)), FALSE);
  return c2_y;
}

static const mxArray *c2_b_emlrt_marshallOut(uint32_T c2_u)
{
  const mxArray *c2_y = NULL;
  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", &c2_u, 7, 0U, 0U, 0U, 0), FALSE);
  return c2_y;
}

static void c2_b_info_helper(const mxArray **c2_info)
{
  const mxArray *c2_rhs64 = NULL;
  const mxArray *c2_lhs64 = NULL;
  const mxArray *c2_rhs65 = NULL;
  const mxArray *c2_lhs65 = NULL;
  const mxArray *c2_rhs66 = NULL;
  const mxArray *c2_lhs66 = NULL;
  const mxArray *c2_rhs67 = NULL;
  const mxArray *c2_lhs67 = NULL;
  const mxArray *c2_rhs68 = NULL;
  const mxArray *c2_lhs68 = NULL;
  const mxArray *c2_rhs69 = NULL;
  const mxArray *c2_lhs69 = NULL;
  const mxArray *c2_rhs70 = NULL;
  const mxArray *c2_lhs70 = NULL;
  const mxArray *c2_rhs71 = NULL;
  const mxArray *c2_lhs71 = NULL;
  const mxArray *c2_rhs72 = NULL;
  const mxArray *c2_lhs72 = NULL;
  const mxArray *c2_rhs73 = NULL;
  const mxArray *c2_lhs73 = NULL;
  const mxArray *c2_rhs74 = NULL;
  const mxArray *c2_lhs74 = NULL;
  const mxArray *c2_rhs75 = NULL;
  const mxArray *c2_lhs75 = NULL;
  const mxArray *c2_rhs76 = NULL;
  const mxArray *c2_lhs76 = NULL;
  const mxArray *c2_rhs77 = NULL;
  const mxArray *c2_lhs77 = NULL;
  const mxArray *c2_rhs78 = NULL;
  const mxArray *c2_lhs78 = NULL;
  const mxArray *c2_rhs79 = NULL;
  const mxArray *c2_lhs79 = NULL;
  const mxArray *c2_rhs80 = NULL;
  const mxArray *c2_lhs80 = NULL;
  const mxArray *c2_rhs81 = NULL;
  const mxArray *c2_lhs81 = NULL;
  const mxArray *c2_rhs82 = NULL;
  const mxArray *c2_lhs82 = NULL;
  const mxArray *c2_rhs83 = NULL;
  const mxArray *c2_lhs83 = NULL;
  const mxArray *c2_rhs84 = NULL;
  const mxArray *c2_lhs84 = NULL;
  const mxArray *c2_rhs85 = NULL;
  const mxArray *c2_lhs85 = NULL;
  const mxArray *c2_rhs86 = NULL;
  const mxArray *c2_lhs86 = NULL;
  const mxArray *c2_rhs87 = NULL;
  const mxArray *c2_lhs87 = NULL;
  const mxArray *c2_rhs88 = NULL;
  const mxArray *c2_lhs88 = NULL;
  const mxArray *c2_rhs89 = NULL;
  const mxArray *c2_lhs89 = NULL;
  const mxArray *c2_rhs90 = NULL;
  const mxArray *c2_lhs90 = NULL;
  const mxArray *c2_rhs91 = NULL;
  const mxArray *c2_lhs91 = NULL;
  const mxArray *c2_rhs92 = NULL;
  const mxArray *c2_lhs92 = NULL;
  const mxArray *c2_rhs93 = NULL;
  const mxArray *c2_lhs93 = NULL;
  const mxArray *c2_rhs94 = NULL;
  const mxArray *c2_lhs94 = NULL;
  const mxArray *c2_rhs95 = NULL;
  const mxArray *c2_lhs95 = NULL;
  const mxArray *c2_rhs96 = NULL;
  const mxArray *c2_lhs96 = NULL;
  const mxArray *c2_rhs97 = NULL;
  const mxArray *c2_lhs97 = NULL;
  const mxArray *c2_rhs98 = NULL;
  const mxArray *c2_lhs98 = NULL;
  const mxArray *c2_rhs99 = NULL;
  const mxArray *c2_lhs99 = NULL;
  const mxArray *c2_rhs100 = NULL;
  const mxArray *c2_lhs100 = NULL;
  const mxArray *c2_rhs101 = NULL;
  const mxArray *c2_lhs101 = NULL;
  const mxArray *c2_rhs102 = NULL;
  const mxArray *c2_lhs102 = NULL;
  const mxArray *c2_rhs103 = NULL;
  const mxArray *c2_lhs103 = NULL;
  const mxArray *c2_rhs104 = NULL;
  const mxArray *c2_lhs104 = NULL;
  const mxArray *c2_rhs105 = NULL;
  const mxArray *c2_lhs105 = NULL;
  const mxArray *c2_rhs106 = NULL;
  const mxArray *c2_lhs106 = NULL;
  const mxArray *c2_rhs107 = NULL;
  const mxArray *c2_lhs107 = NULL;
  const mxArray *c2_rhs108 = NULL;
  const mxArray *c2_lhs108 = NULL;
  const mxArray *c2_rhs109 = NULL;
  const mxArray *c2_lhs109 = NULL;
  const mxArray *c2_rhs110 = NULL;
  const mxArray *c2_lhs110 = NULL;
  const mxArray *c2_rhs111 = NULL;
  const mxArray *c2_lhs111 = NULL;
  const mxArray *c2_rhs112 = NULL;
  const mxArray *c2_lhs112 = NULL;
  const mxArray *c2_rhs113 = NULL;
  const mxArray *c2_lhs113 = NULL;
  const mxArray *c2_rhs114 = NULL;
  const mxArray *c2_lhs114 = NULL;
  const mxArray *c2_rhs115 = NULL;
  const mxArray *c2_lhs115 = NULL;
  const mxArray *c2_rhs116 = NULL;
  const mxArray *c2_lhs116 = NULL;
  const mxArray *c2_rhs117 = NULL;
  const mxArray *c2_lhs117 = NULL;
  const mxArray *c2_rhs118 = NULL;
  const mxArray *c2_lhs118 = NULL;
  const mxArray *c2_rhs119 = NULL;
  const mxArray *c2_lhs119 = NULL;
  const mxArray *c2_rhs120 = NULL;
  const mxArray *c2_lhs120 = NULL;
  const mxArray *c2_rhs121 = NULL;
  const mxArray *c2_lhs121 = NULL;
  const mxArray *c2_rhs122 = NULL;
  const mxArray *c2_lhs122 = NULL;
  const mxArray *c2_rhs123 = NULL;
  const mxArray *c2_lhs123 = NULL;
  const mxArray *c2_rhs124 = NULL;
  const mxArray *c2_lhs124 = NULL;
  const mxArray *c2_rhs125 = NULL;
  const mxArray *c2_lhs125 = NULL;
  const mxArray *c2_rhs126 = NULL;
  const mxArray *c2_lhs126 = NULL;
  const mxArray *c2_rhs127 = NULL;
  const mxArray *c2_lhs127 = NULL;
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/repmat.m"), "context",
                  "context", 64);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "eml_int_forloop_overflow_check"), "name", "name", 64);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 64);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m"),
                  "resolved", "resolved", 64);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1346488740U), "fileTimeLo",
                  "fileTimeLo", 64);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 64);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 64);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 64);
  sf_mex_assign(&c2_rhs64, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs64, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs64), "rhs", "rhs",
                  64);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs64), "lhs", "lhs",
                  64);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/repmat.m"), "context",
                  "context", 65);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_index_plus"), "name",
                  "name", 65);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 65);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m"),
                  "resolved", "resolved", 65);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1286797178U), "fileTimeLo",
                  "fileTimeLo", 65);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 65);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 65);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 65);
  sf_mex_assign(&c2_rhs65, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs65, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs65), "rhs", "rhs",
                  65);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs65), "lhs", "lhs",
                  65);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "context", "context", 66);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("diag"), "name", "name", 66);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 66);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/diag.m"), "resolved",
                  "resolved", 66);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1363688654U), "fileTimeLo",
                  "fileTimeLo", 66);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 66);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 66);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 66);
  sf_mex_assign(&c2_rhs66, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs66, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs66), "rhs", "rhs",
                  66);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs66), "lhs", "lhs",
                  66);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/diag.m"), "context",
                  "context", 67);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("ismatrix"), "name", "name", 67);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 67);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/ismatrix.m"), "resolved",
                  "resolved", 67);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1331279658U), "fileTimeLo",
                  "fileTimeLo", 67);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 67);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 67);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 67);
  sf_mex_assign(&c2_rhs67, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs67, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs67), "rhs", "rhs",
                  67);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs67), "lhs", "lhs",
                  67);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/diag.m"), "context",
                  "context", 68);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_index_class"), "name",
                  "name", 68);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 68);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m"),
                  "resolved", "resolved", 68);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1323145378U), "fileTimeLo",
                  "fileTimeLo", 68);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 68);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 68);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 68);
  sf_mex_assign(&c2_rhs68, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs68, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs68), "rhs", "rhs",
                  68);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs68), "lhs", "lhs",
                  68);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/diag.m"), "context",
                  "context", 69);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_scalar_eg"), "name",
                  "name", 69);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 69);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "resolved",
                  "resolved", 69);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1286797196U), "fileTimeLo",
                  "fileTimeLo", 69);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 69);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 69);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 69);
  sf_mex_assign(&c2_rhs69, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs69, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs69), "rhs", "rhs",
                  69);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs69), "lhs", "lhs",
                  69);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/diag.m"), "context",
                  "context", 70);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "eml_int_forloop_overflow_check"), "name", "name", 70);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 70);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m"),
                  "resolved", "resolved", 70);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1346488740U), "fileTimeLo",
                  "fileTimeLo", 70);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 70);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 70);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 70);
  sf_mex_assign(&c2_rhs70, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs70, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs70), "rhs", "rhs",
                  70);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs70), "lhs", "lhs",
                  70);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "context", "context", 71);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("power"), "name", "name", 71);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 71);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m"), "resolved",
                  "resolved", 71);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1363688680U), "fileTimeLo",
                  "fileTimeLo", 71);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 71);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 71);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 71);
  sf_mex_assign(&c2_rhs71, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs71, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs71), "rhs", "rhs",
                  71);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs71), "lhs", "lhs",
                  71);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m"), "context",
                  "context", 72);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 72);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 72);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 72);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1363689356U), "fileTimeLo",
                  "fileTimeLo", 72);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 72);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 72);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 72);
  sf_mex_assign(&c2_rhs72, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs72, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs72), "rhs", "rhs",
                  72);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs72), "lhs", "lhs",
                  72);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m!fltpower"), "context",
                  "context", 73);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_scalar_eg"), "name",
                  "name", 73);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 73);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "resolved",
                  "resolved", 73);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1286797196U), "fileTimeLo",
                  "fileTimeLo", 73);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 73);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 73);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 73);
  sf_mex_assign(&c2_rhs73, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs73, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs73), "rhs", "rhs",
                  73);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs73), "lhs", "lhs",
                  73);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m!fltpower"), "context",
                  "context", 74);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_scalexp_alloc"), "name",
                  "name", 74);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 74);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m"),
                  "resolved", "resolved", 74);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1358160940U), "fileTimeLo",
                  "fileTimeLo", 74);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 74);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 74);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 74);
  sf_mex_assign(&c2_rhs74, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs74, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs74), "rhs", "rhs",
                  74);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs74), "lhs", "lhs",
                  74);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m!fltpower"), "context",
                  "context", 75);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("floor"), "name", "name", 75);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 75);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/floor.m"), "resolved",
                  "resolved", 75);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1363688654U), "fileTimeLo",
                  "fileTimeLo", 75);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 75);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 75);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 75);
  sf_mex_assign(&c2_rhs75, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs75, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs75), "rhs", "rhs",
                  75);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs75), "lhs", "lhs",
                  75);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/floor.m"), "context",
                  "context", 76);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 76);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 76);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 76);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1363689356U), "fileTimeLo",
                  "fileTimeLo", 76);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 76);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 76);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 76);
  sf_mex_assign(&c2_rhs76, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs76, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs76), "rhs", "rhs",
                  76);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs76), "lhs", "lhs",
                  76);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/floor.m"), "context",
                  "context", 77);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_scalar_floor"), "name",
                  "name", 77);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 77);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_floor.m"),
                  "resolved", "resolved", 77);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1286797126U), "fileTimeLo",
                  "fileTimeLo", 77);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 77);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 77);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 77);
  sf_mex_assign(&c2_rhs77, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs77, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs77), "rhs", "rhs",
                  77);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs77), "lhs", "lhs",
                  77);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m!scalar_float_power"),
                  "context", "context", 78);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_scalar_eg"), "name",
                  "name", 78);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 78);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "resolved",
                  "resolved", 78);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1286797196U), "fileTimeLo",
                  "fileTimeLo", 78);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 78);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 78);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 78);
  sf_mex_assign(&c2_rhs78, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs78, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs78), "rhs", "rhs",
                  78);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs78), "lhs", "lhs",
                  78);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m!scalar_float_power"),
                  "context", "context", 79);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("mtimes"), "name", "name", 79);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 79);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m"), "resolved",
                  "resolved", 79);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1363688678U), "fileTimeLo",
                  "fileTimeLo", 79);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 79);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 79);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 79);
  sf_mex_assign(&c2_rhs79, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs79, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs79), "rhs", "rhs",
                  79);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs79), "lhs", "lhs",
                  79);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "context", "context", 80);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("sqrt"), "name", "name", 80);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 80);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sqrt.m"), "resolved",
                  "resolved", 80);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1343808786U), "fileTimeLo",
                  "fileTimeLo", 80);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 80);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 80);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 80);
  sf_mex_assign(&c2_rhs80, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs80, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs80), "rhs", "rhs",
                  80);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs80), "lhs", "lhs",
                  80);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sqrt.m"), "context",
                  "context", 81);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_error"), "name", "name",
                  81);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 81);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_error.m"), "resolved",
                  "resolved", 81);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1343808758U), "fileTimeLo",
                  "fileTimeLo", 81);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 81);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 81);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 81);
  sf_mex_assign(&c2_rhs81, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs81, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs81), "rhs", "rhs",
                  81);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs81), "lhs", "lhs",
                  81);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sqrt.m"), "context",
                  "context", 82);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_scalar_sqrt"), "name",
                  "name", 82);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 82);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_sqrt.m"),
                  "resolved", "resolved", 82);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1286797138U), "fileTimeLo",
                  "fileTimeLo", 82);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 82);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 82);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 82);
  sf_mex_assign(&c2_rhs82, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs82, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs82), "rhs", "rhs",
                  82);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs82), "lhs", "lhs",
                  82);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "context", "context", 83);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("colon"), "name", "name", 83);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 83);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m"), "resolved",
                  "resolved", 83);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1366140642U), "fileTimeLo",
                  "fileTimeLo", 83);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 83);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 83);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 83);
  sf_mex_assign(&c2_rhs83, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs83, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs83), "rhs", "rhs",
                  83);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs83), "lhs", "lhs",
                  83);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m"), "context",
                  "context", 84);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("colon"), "name", "name", 84);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 84);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m"), "resolved",
                  "resolved", 84);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1366140642U), "fileTimeLo",
                  "fileTimeLo", 84);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 84);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 84);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 84);
  sf_mex_assign(&c2_rhs84, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs84, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs84), "rhs", "rhs",
                  84);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs84), "lhs", "lhs",
                  84);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m"), "context",
                  "context", 85);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 85);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 85);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 85);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1363689356U), "fileTimeLo",
                  "fileTimeLo", 85);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 85);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 85);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 85);
  sf_mex_assign(&c2_rhs85, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs85, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs85), "rhs", "rhs",
                  85);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs85), "lhs", "lhs",
                  85);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!is_flint_colon"),
                  "context", "context", 86);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("isfinite"), "name", "name", 86);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 86);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isfinite.m"), "resolved",
                  "resolved", 86);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1363688656U), "fileTimeLo",
                  "fileTimeLo", 86);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 86);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 86);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 86);
  sf_mex_assign(&c2_rhs86, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs86, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs86), "rhs", "rhs",
                  86);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs86), "lhs", "lhs",
                  86);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isfinite.m"), "context",
                  "context", 87);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 87);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 87);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 87);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1363689356U), "fileTimeLo",
                  "fileTimeLo", 87);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 87);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 87);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 87);
  sf_mex_assign(&c2_rhs87, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs87, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs87), "rhs", "rhs",
                  87);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs87), "lhs", "lhs",
                  87);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isfinite.m"), "context",
                  "context", 88);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("isinf"), "name", "name", 88);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 88);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isinf.m"), "resolved",
                  "resolved", 88);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1363688656U), "fileTimeLo",
                  "fileTimeLo", 88);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 88);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 88);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 88);
  sf_mex_assign(&c2_rhs88, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs88, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs88), "rhs", "rhs",
                  88);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs88), "lhs", "lhs",
                  88);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isfinite.m"), "context",
                  "context", 89);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("isnan"), "name", "name", 89);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 89);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isnan.m"), "resolved",
                  "resolved", 89);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1363688658U), "fileTimeLo",
                  "fileTimeLo", 89);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 89);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 89);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 89);
  sf_mex_assign(&c2_rhs89, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs89, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs89), "rhs", "rhs",
                  89);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs89), "lhs", "lhs",
                  89);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isnan.m"), "context",
                  "context", 90);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 90);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 90);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 90);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1363689356U), "fileTimeLo",
                  "fileTimeLo", 90);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 90);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 90);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 90);
  sf_mex_assign(&c2_rhs90, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs90, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs90), "rhs", "rhs",
                  90);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs90), "lhs", "lhs",
                  90);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!is_flint_colon"),
                  "context", "context", 91);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("floor"), "name", "name", 91);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 91);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/floor.m"), "resolved",
                  "resolved", 91);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1363688654U), "fileTimeLo",
                  "fileTimeLo", 91);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 91);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 91);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 91);
  sf_mex_assign(&c2_rhs91, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs91, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs91), "rhs", "rhs",
                  91);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs91), "lhs", "lhs",
                  91);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!maxabs"), "context",
                  "context", 92);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("abs"), "name", "name", 92);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 92);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m"), "resolved",
                  "resolved", 92);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1363688652U), "fileTimeLo",
                  "fileTimeLo", 92);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 92);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 92);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 92);
  sf_mex_assign(&c2_rhs92, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs92, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs92), "rhs", "rhs",
                  92);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs92), "lhs", "lhs",
                  92);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m"), "context",
                  "context", 93);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 93);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 93);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 93);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1363689356U), "fileTimeLo",
                  "fileTimeLo", 93);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 93);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 93);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 93);
  sf_mex_assign(&c2_rhs93, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs93, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs93), "rhs", "rhs",
                  93);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs93), "lhs", "lhs",
                  93);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m"), "context",
                  "context", 94);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_scalar_abs"), "name",
                  "name", 94);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 94);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_abs.m"),
                  "resolved", "resolved", 94);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1286797112U), "fileTimeLo",
                  "fileTimeLo", 94);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 94);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 94);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 94);
  sf_mex_assign(&c2_rhs94, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs94, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs94), "rhs", "rhs",
                  94);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs94), "lhs", "lhs",
                  94);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!is_flint_colon"),
                  "context", "context", 95);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eps"), "name", "name", 95);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 95);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eps.m"), "resolved",
                  "resolved", 95);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1326702796U), "fileTimeLo",
                  "fileTimeLo", 95);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 95);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 95);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 95);
  sf_mex_assign(&c2_rhs95, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs95, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs95), "rhs", "rhs",
                  95);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs95), "lhs", "lhs",
                  95);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eps.m"), "context",
                  "context", 96);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_mantissa_nbits"), "name",
                  "name", 96);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 96);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_mantissa_nbits.m"),
                  "resolved", "resolved", 96);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1307629642U), "fileTimeLo",
                  "fileTimeLo", 96);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 96);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 96);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 96);
  sf_mex_assign(&c2_rhs96, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs96, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs96), "rhs", "rhs",
                  96);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs96), "lhs", "lhs",
                  96);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_mantissa_nbits.m"),
                  "context", "context", 97);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_float_model"), "name",
                  "name", 97);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 97);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_float_model.m"),
                  "resolved", "resolved", 97);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1326702796U), "fileTimeLo",
                  "fileTimeLo", 97);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 97);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 97);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 97);
  sf_mex_assign(&c2_rhs97, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs97, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs97), "rhs", "rhs",
                  97);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs97), "lhs", "lhs",
                  97);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eps.m"), "context",
                  "context", 98);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_realmin"), "name", "name",
                  98);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 98);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_realmin.m"), "resolved",
                  "resolved", 98);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1307629644U), "fileTimeLo",
                  "fileTimeLo", 98);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 98);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 98);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 98);
  sf_mex_assign(&c2_rhs98, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs98, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs98), "rhs", "rhs",
                  98);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs98), "lhs", "lhs",
                  98);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_realmin.m"), "context",
                  "context", 99);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_float_model"), "name",
                  "name", 99);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 99);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_float_model.m"),
                  "resolved", "resolved", 99);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1326702796U), "fileTimeLo",
                  "fileTimeLo", 99);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 99);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 99);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 99);
  sf_mex_assign(&c2_rhs99, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs99, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs99), "rhs", "rhs",
                  99);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs99), "lhs", "lhs",
                  99);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eps.m"), "context",
                  "context", 100);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_realmin_denormal"), "name",
                  "name", 100);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 100);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_realmin_denormal.m"),
                  "resolved", "resolved", 100);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1326702798U), "fileTimeLo",
                  "fileTimeLo", 100);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 100);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 100);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 100);
  sf_mex_assign(&c2_rhs100, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs100, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs100), "rhs", "rhs",
                  100);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs100), "lhs", "lhs",
                  100);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_realmin_denormal.m"),
                  "context", "context", 101);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_float_model"), "name",
                  "name", 101);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 101);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_float_model.m"),
                  "resolved", "resolved", 101);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1326702796U), "fileTimeLo",
                  "fileTimeLo", 101);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 101);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 101);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 101);
  sf_mex_assign(&c2_rhs101, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs101, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs101), "rhs", "rhs",
                  101);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs101), "lhs", "lhs",
                  101);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eps.m"), "context",
                  "context", 102);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("abs"), "name", "name", 102);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 102);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m"), "resolved",
                  "resolved", 102);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1363688652U), "fileTimeLo",
                  "fileTimeLo", 102);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 102);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 102);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 102);
  sf_mex_assign(&c2_rhs102, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs102, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs102), "rhs", "rhs",
                  102);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs102), "lhs", "lhs",
                  102);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/eps.m"), "context",
                  "context", 103);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("isfinite"), "name", "name",
                  103);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 103);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isfinite.m"), "resolved",
                  "resolved", 103);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1363688656U), "fileTimeLo",
                  "fileTimeLo", 103);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 103);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 103);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 103);
  sf_mex_assign(&c2_rhs103, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs103, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs103), "rhs", "rhs",
                  103);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs103), "lhs", "lhs",
                  103);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!checkrange"),
                  "context", "context", 104);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("realmax"), "name", "name", 104);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 104);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/realmax.m"), "resolved",
                  "resolved", 104);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1307629642U), "fileTimeLo",
                  "fileTimeLo", 104);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 104);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 104);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 104);
  sf_mex_assign(&c2_rhs104, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs104, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs104), "rhs", "rhs",
                  104);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs104), "lhs", "lhs",
                  104);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/realmax.m"), "context",
                  "context", 105);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_realmax"), "name", "name",
                  105);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 105);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_realmax.m"), "resolved",
                  "resolved", 105);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1326702796U), "fileTimeLo",
                  "fileTimeLo", 105);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 105);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 105);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 105);
  sf_mex_assign(&c2_rhs105, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs105, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs105), "rhs", "rhs",
                  105);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs105), "lhs", "lhs",
                  105);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_realmax.m"), "context",
                  "context", 106);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_float_model"), "name",
                  "name", 106);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 106);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_float_model.m"),
                  "resolved", "resolved", 106);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1326702796U), "fileTimeLo",
                  "fileTimeLo", 106);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 106);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 106);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 106);
  sf_mex_assign(&c2_rhs106, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs106, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs106), "rhs", "rhs",
                  106);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs106), "lhs", "lhs",
                  106);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_realmax.m"), "context",
                  "context", 107);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("mtimes"), "name", "name", 107);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 107);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m"), "resolved",
                  "resolved", 107);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1363688678U), "fileTimeLo",
                  "fileTimeLo", 107);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 107);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 107);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 107);
  sf_mex_assign(&c2_rhs107, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs107, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs107), "rhs", "rhs",
                  107);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs107), "lhs", "lhs",
                  107);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!eml_flint_colon"),
                  "context", "context", 108);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("mrdivide"), "name", "name",
                  108);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 108);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mrdivide.p"), "resolved",
                  "resolved", 108);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1373284908U), "fileTimeLo",
                  "fileTimeLo", 108);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 108);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1319708366U), "mFileTimeLo",
                  "mFileTimeLo", 108);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 108);
  sf_mex_assign(&c2_rhs108, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs108, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs108), "rhs", "rhs",
                  108);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs108), "lhs", "lhs",
                  108);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!eml_flint_colon"),
                  "context", "context", 109);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("floor"), "name", "name", 109);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 109);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/floor.m"), "resolved",
                  "resolved", 109);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1363688654U), "fileTimeLo",
                  "fileTimeLo", 109);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 109);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 109);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 109);
  sf_mex_assign(&c2_rhs109, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs109, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs109), "rhs", "rhs",
                  109);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs109), "lhs", "lhs",
                  109);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!eml_flint_colon"),
                  "context", "context", 110);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_index_class"), "name",
                  "name", 110);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 110);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m"),
                  "resolved", "resolved", 110);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1323145378U), "fileTimeLo",
                  "fileTimeLo", 110);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 110);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 110);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 110);
  sf_mex_assign(&c2_rhs110, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs110, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs110), "rhs", "rhs",
                  110);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs110), "lhs", "lhs",
                  110);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/colon.m!eml_flint_colon"),
                  "context", "context", 111);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("intmax"), "name", "name", 111);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 111);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m"), "resolved",
                  "resolved", 111);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1362236682U), "fileTimeLo",
                  "fileTimeLo", 111);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 111);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 111);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 111);
  sf_mex_assign(&c2_rhs111, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs111, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs111), "rhs", "rhs",
                  111);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs111), "lhs", "lhs",
                  111);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "context", "context", 112);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("sort"), "name", "name", 112);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 112);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/sort.m"), "resolved",
                  "resolved", 112);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1363688656U), "fileTimeLo",
                  "fileTimeLo", 112);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 112);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 112);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 112);
  sf_mex_assign(&c2_rhs112, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs112, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs112), "rhs", "rhs",
                  112);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs112), "lhs", "lhs",
                  112);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/sort.m"), "context",
                  "context", 113);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 113);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 113);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 113);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1363689356U), "fileTimeLo",
                  "fileTimeLo", 113);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 113);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 113);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 113);
  sf_mex_assign(&c2_rhs113, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs113, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs113), "rhs", "rhs",
                  113);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs113), "lhs", "lhs",
                  113);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/sort.m"), "context",
                  "context", 114);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_sort"), "name", "name",
                  114);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 114);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_sort.m"), "resolved",
                  "resolved", 114);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1314715012U), "fileTimeLo",
                  "fileTimeLo", 114);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 114);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 114);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 114);
  sf_mex_assign(&c2_rhs114, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs114, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs114), "rhs", "rhs",
                  114);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs114), "lhs", "lhs",
                  114);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_sort.m"), "context",
                  "context", 115);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_nonsingleton_dim"), "name",
                  "name", 115);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 115);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_nonsingleton_dim.m"),
                  "resolved", "resolved", 115);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1307629642U), "fileTimeLo",
                  "fileTimeLo", 115);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 115);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 115);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 115);
  sf_mex_assign(&c2_rhs115, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs115, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs115), "rhs", "rhs",
                  115);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs115), "lhs", "lhs",
                  115);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_nonsingleton_dim.m"),
                  "context", "context", 116);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_index_class"), "name",
                  "name", 116);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 116);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m"),
                  "resolved", "resolved", 116);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1323145378U), "fileTimeLo",
                  "fileTimeLo", 116);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 116);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 116);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 116);
  sf_mex_assign(&c2_rhs116, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs116, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs116), "rhs", "rhs",
                  116);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs116), "lhs", "lhs",
                  116);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_sort.m"), "context",
                  "context", 117);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_assert_valid_dim"), "name",
                  "name", 117);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 117);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_assert_valid_dim.m"),
                  "resolved", "resolved", 117);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1363688666U), "fileTimeLo",
                  "fileTimeLo", 117);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 117);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 117);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 117);
  sf_mex_assign(&c2_rhs117, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs117, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs117), "rhs", "rhs",
                  117);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs117), "lhs", "lhs",
                  117);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_assert_valid_dim.m"),
                  "context", "context", 118);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 118);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 118);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 118);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1363689356U), "fileTimeLo",
                  "fileTimeLo", 118);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 118);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 118);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 118);
  sf_mex_assign(&c2_rhs118, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs118, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs118), "rhs", "rhs",
                  118);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs118), "lhs", "lhs",
                  118);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_assert_valid_dim.m"),
                  "context", "context", 119);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_scalar_floor"), "name",
                  "name", 119);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 119);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_floor.m"),
                  "resolved", "resolved", 119);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1286797126U), "fileTimeLo",
                  "fileTimeLo", 119);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 119);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 119);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 119);
  sf_mex_assign(&c2_rhs119, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs119, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs119), "rhs", "rhs",
                  119);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs119), "lhs", "lhs",
                  119);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_assert_valid_dim.m"),
                  "context", "context", 120);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_index_class"), "name",
                  "name", 120);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 120);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m"),
                  "resolved", "resolved", 120);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1323145378U), "fileTimeLo",
                  "fileTimeLo", 120);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 120);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 120);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 120);
  sf_mex_assign(&c2_rhs120, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs120, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs120), "rhs", "rhs",
                  120);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs120), "lhs", "lhs",
                  120);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_assert_valid_dim.m"),
                  "context", "context", 121);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("intmax"), "name", "name", 121);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 121);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/intmax.m"), "resolved",
                  "resolved", 121);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1362236682U), "fileTimeLo",
                  "fileTimeLo", 121);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 121);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 121);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 121);
  sf_mex_assign(&c2_rhs121, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs121, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs121), "rhs", "rhs",
                  121);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs121), "lhs", "lhs",
                  121);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_sort.m"), "context",
                  "context", 122);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_scalar_eg"), "name",
                  "name", 122);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 122);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "resolved",
                  "resolved", 122);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1286797196U), "fileTimeLo",
                  "fileTimeLo", 122);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 122);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 122);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 122);
  sf_mex_assign(&c2_rhs122, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs122, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs122), "rhs", "rhs",
                  122);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs122), "lhs", "lhs",
                  122);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_sort.m"), "context",
                  "context", 123);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_index_class"), "name",
                  "name", 123);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 123);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m"),
                  "resolved", "resolved", 123);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1323145378U), "fileTimeLo",
                  "fileTimeLo", 123);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 123);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 123);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 123);
  sf_mex_assign(&c2_rhs123, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs123, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs123), "rhs", "rhs",
                  123);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs123), "lhs", "lhs",
                  123);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_sort.m"), "context",
                  "context", 124);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_matrix_vstride"), "name",
                  "name", 124);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 124);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_matrix_vstride.m"),
                  "resolved", "resolved", 124);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1360257150U), "fileTimeLo",
                  "fileTimeLo", 124);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 124);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 124);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 124);
  sf_mex_assign(&c2_rhs124, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs124, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs124), "rhs", "rhs",
                  124);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs124), "lhs", "lhs",
                  124);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_matrix_vstride.m"),
                  "context", "context", 125);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.prodsize"),
                  "name", "name", 125);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 125);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/prodsize.m"),
                  "resolved", "resolved", 125);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1360257388U), "fileTimeLo",
                  "fileTimeLo", 125);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 125);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 125);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 125);
  sf_mex_assign(&c2_rhs125, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs125, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs125), "rhs", "rhs",
                  125);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs125), "lhs", "lhs",
                  125);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/prodsize.m"),
                  "context", "context", 126);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "eml_int_forloop_overflow_check"), "name", "name", 126);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 126);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m"),
                  "resolved", "resolved", 126);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1346488740U), "fileTimeLo",
                  "fileTimeLo", 126);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 126);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 126);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 126);
  sf_mex_assign(&c2_rhs126, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs126, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs126), "rhs", "rhs",
                  126);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs126), "lhs", "lhs",
                  126);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/prodsize.m"),
                  "context", "context", 127);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("mtimes"), "name", "name", 127);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 127);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m"), "resolved",
                  "resolved", 127);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1363688678U), "fileTimeLo",
                  "fileTimeLo", 127);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 127);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 127);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 127);
  sf_mex_assign(&c2_rhs127, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs127, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs127), "rhs", "rhs",
                  127);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs127), "lhs", "lhs",
                  127);
  sf_mex_destroy(&c2_rhs64);
  sf_mex_destroy(&c2_lhs64);
  sf_mex_destroy(&c2_rhs65);
  sf_mex_destroy(&c2_lhs65);
  sf_mex_destroy(&c2_rhs66);
  sf_mex_destroy(&c2_lhs66);
  sf_mex_destroy(&c2_rhs67);
  sf_mex_destroy(&c2_lhs67);
  sf_mex_destroy(&c2_rhs68);
  sf_mex_destroy(&c2_lhs68);
  sf_mex_destroy(&c2_rhs69);
  sf_mex_destroy(&c2_lhs69);
  sf_mex_destroy(&c2_rhs70);
  sf_mex_destroy(&c2_lhs70);
  sf_mex_destroy(&c2_rhs71);
  sf_mex_destroy(&c2_lhs71);
  sf_mex_destroy(&c2_rhs72);
  sf_mex_destroy(&c2_lhs72);
  sf_mex_destroy(&c2_rhs73);
  sf_mex_destroy(&c2_lhs73);
  sf_mex_destroy(&c2_rhs74);
  sf_mex_destroy(&c2_lhs74);
  sf_mex_destroy(&c2_rhs75);
  sf_mex_destroy(&c2_lhs75);
  sf_mex_destroy(&c2_rhs76);
  sf_mex_destroy(&c2_lhs76);
  sf_mex_destroy(&c2_rhs77);
  sf_mex_destroy(&c2_lhs77);
  sf_mex_destroy(&c2_rhs78);
  sf_mex_destroy(&c2_lhs78);
  sf_mex_destroy(&c2_rhs79);
  sf_mex_destroy(&c2_lhs79);
  sf_mex_destroy(&c2_rhs80);
  sf_mex_destroy(&c2_lhs80);
  sf_mex_destroy(&c2_rhs81);
  sf_mex_destroy(&c2_lhs81);
  sf_mex_destroy(&c2_rhs82);
  sf_mex_destroy(&c2_lhs82);
  sf_mex_destroy(&c2_rhs83);
  sf_mex_destroy(&c2_lhs83);
  sf_mex_destroy(&c2_rhs84);
  sf_mex_destroy(&c2_lhs84);
  sf_mex_destroy(&c2_rhs85);
  sf_mex_destroy(&c2_lhs85);
  sf_mex_destroy(&c2_rhs86);
  sf_mex_destroy(&c2_lhs86);
  sf_mex_destroy(&c2_rhs87);
  sf_mex_destroy(&c2_lhs87);
  sf_mex_destroy(&c2_rhs88);
  sf_mex_destroy(&c2_lhs88);
  sf_mex_destroy(&c2_rhs89);
  sf_mex_destroy(&c2_lhs89);
  sf_mex_destroy(&c2_rhs90);
  sf_mex_destroy(&c2_lhs90);
  sf_mex_destroy(&c2_rhs91);
  sf_mex_destroy(&c2_lhs91);
  sf_mex_destroy(&c2_rhs92);
  sf_mex_destroy(&c2_lhs92);
  sf_mex_destroy(&c2_rhs93);
  sf_mex_destroy(&c2_lhs93);
  sf_mex_destroy(&c2_rhs94);
  sf_mex_destroy(&c2_lhs94);
  sf_mex_destroy(&c2_rhs95);
  sf_mex_destroy(&c2_lhs95);
  sf_mex_destroy(&c2_rhs96);
  sf_mex_destroy(&c2_lhs96);
  sf_mex_destroy(&c2_rhs97);
  sf_mex_destroy(&c2_lhs97);
  sf_mex_destroy(&c2_rhs98);
  sf_mex_destroy(&c2_lhs98);
  sf_mex_destroy(&c2_rhs99);
  sf_mex_destroy(&c2_lhs99);
  sf_mex_destroy(&c2_rhs100);
  sf_mex_destroy(&c2_lhs100);
  sf_mex_destroy(&c2_rhs101);
  sf_mex_destroy(&c2_lhs101);
  sf_mex_destroy(&c2_rhs102);
  sf_mex_destroy(&c2_lhs102);
  sf_mex_destroy(&c2_rhs103);
  sf_mex_destroy(&c2_lhs103);
  sf_mex_destroy(&c2_rhs104);
  sf_mex_destroy(&c2_lhs104);
  sf_mex_destroy(&c2_rhs105);
  sf_mex_destroy(&c2_lhs105);
  sf_mex_destroy(&c2_rhs106);
  sf_mex_destroy(&c2_lhs106);
  sf_mex_destroy(&c2_rhs107);
  sf_mex_destroy(&c2_lhs107);
  sf_mex_destroy(&c2_rhs108);
  sf_mex_destroy(&c2_lhs108);
  sf_mex_destroy(&c2_rhs109);
  sf_mex_destroy(&c2_lhs109);
  sf_mex_destroy(&c2_rhs110);
  sf_mex_destroy(&c2_lhs110);
  sf_mex_destroy(&c2_rhs111);
  sf_mex_destroy(&c2_lhs111);
  sf_mex_destroy(&c2_rhs112);
  sf_mex_destroy(&c2_lhs112);
  sf_mex_destroy(&c2_rhs113);
  sf_mex_destroy(&c2_lhs113);
  sf_mex_destroy(&c2_rhs114);
  sf_mex_destroy(&c2_lhs114);
  sf_mex_destroy(&c2_rhs115);
  sf_mex_destroy(&c2_lhs115);
  sf_mex_destroy(&c2_rhs116);
  sf_mex_destroy(&c2_lhs116);
  sf_mex_destroy(&c2_rhs117);
  sf_mex_destroy(&c2_lhs117);
  sf_mex_destroy(&c2_rhs118);
  sf_mex_destroy(&c2_lhs118);
  sf_mex_destroy(&c2_rhs119);
  sf_mex_destroy(&c2_lhs119);
  sf_mex_destroy(&c2_rhs120);
  sf_mex_destroy(&c2_lhs120);
  sf_mex_destroy(&c2_rhs121);
  sf_mex_destroy(&c2_lhs121);
  sf_mex_destroy(&c2_rhs122);
  sf_mex_destroy(&c2_lhs122);
  sf_mex_destroy(&c2_rhs123);
  sf_mex_destroy(&c2_lhs123);
  sf_mex_destroy(&c2_rhs124);
  sf_mex_destroy(&c2_lhs124);
  sf_mex_destroy(&c2_rhs125);
  sf_mex_destroy(&c2_lhs125);
  sf_mex_destroy(&c2_rhs126);
  sf_mex_destroy(&c2_lhs126);
  sf_mex_destroy(&c2_rhs127);
  sf_mex_destroy(&c2_lhs127);
}

static void c2_c_info_helper(const mxArray **c2_info)
{
  const mxArray *c2_rhs128 = NULL;
  const mxArray *c2_lhs128 = NULL;
  const mxArray *c2_rhs129 = NULL;
  const mxArray *c2_lhs129 = NULL;
  const mxArray *c2_rhs130 = NULL;
  const mxArray *c2_lhs130 = NULL;
  const mxArray *c2_rhs131 = NULL;
  const mxArray *c2_lhs131 = NULL;
  const mxArray *c2_rhs132 = NULL;
  const mxArray *c2_lhs132 = NULL;
  const mxArray *c2_rhs133 = NULL;
  const mxArray *c2_lhs133 = NULL;
  const mxArray *c2_rhs134 = NULL;
  const mxArray *c2_lhs134 = NULL;
  const mxArray *c2_rhs135 = NULL;
  const mxArray *c2_lhs135 = NULL;
  const mxArray *c2_rhs136 = NULL;
  const mxArray *c2_lhs136 = NULL;
  const mxArray *c2_rhs137 = NULL;
  const mxArray *c2_lhs137 = NULL;
  const mxArray *c2_rhs138 = NULL;
  const mxArray *c2_lhs138 = NULL;
  const mxArray *c2_rhs139 = NULL;
  const mxArray *c2_lhs139 = NULL;
  const mxArray *c2_rhs140 = NULL;
  const mxArray *c2_lhs140 = NULL;
  const mxArray *c2_rhs141 = NULL;
  const mxArray *c2_lhs141 = NULL;
  const mxArray *c2_rhs142 = NULL;
  const mxArray *c2_lhs142 = NULL;
  const mxArray *c2_rhs143 = NULL;
  const mxArray *c2_lhs143 = NULL;
  const mxArray *c2_rhs144 = NULL;
  const mxArray *c2_lhs144 = NULL;
  const mxArray *c2_rhs145 = NULL;
  const mxArray *c2_lhs145 = NULL;
  const mxArray *c2_rhs146 = NULL;
  const mxArray *c2_lhs146 = NULL;
  const mxArray *c2_rhs147 = NULL;
  const mxArray *c2_lhs147 = NULL;
  const mxArray *c2_rhs148 = NULL;
  const mxArray *c2_lhs148 = NULL;
  const mxArray *c2_rhs149 = NULL;
  const mxArray *c2_lhs149 = NULL;
  const mxArray *c2_rhs150 = NULL;
  const mxArray *c2_lhs150 = NULL;
  const mxArray *c2_rhs151 = NULL;
  const mxArray *c2_lhs151 = NULL;
  const mxArray *c2_rhs152 = NULL;
  const mxArray *c2_lhs152 = NULL;
  const mxArray *c2_rhs153 = NULL;
  const mxArray *c2_lhs153 = NULL;
  const mxArray *c2_rhs154 = NULL;
  const mxArray *c2_lhs154 = NULL;
  const mxArray *c2_rhs155 = NULL;
  const mxArray *c2_lhs155 = NULL;
  const mxArray *c2_rhs156 = NULL;
  const mxArray *c2_lhs156 = NULL;
  const mxArray *c2_rhs157 = NULL;
  const mxArray *c2_lhs157 = NULL;
  const mxArray *c2_rhs158 = NULL;
  const mxArray *c2_lhs158 = NULL;
  const mxArray *c2_rhs159 = NULL;
  const mxArray *c2_lhs159 = NULL;
  const mxArray *c2_rhs160 = NULL;
  const mxArray *c2_lhs160 = NULL;
  const mxArray *c2_rhs161 = NULL;
  const mxArray *c2_lhs161 = NULL;
  const mxArray *c2_rhs162 = NULL;
  const mxArray *c2_lhs162 = NULL;
  const mxArray *c2_rhs163 = NULL;
  const mxArray *c2_lhs163 = NULL;
  const mxArray *c2_rhs164 = NULL;
  const mxArray *c2_lhs164 = NULL;
  const mxArray *c2_rhs165 = NULL;
  const mxArray *c2_lhs165 = NULL;
  const mxArray *c2_rhs166 = NULL;
  const mxArray *c2_lhs166 = NULL;
  const mxArray *c2_rhs167 = NULL;
  const mxArray *c2_lhs167 = NULL;
  const mxArray *c2_rhs168 = NULL;
  const mxArray *c2_lhs168 = NULL;
  const mxArray *c2_rhs169 = NULL;
  const mxArray *c2_lhs169 = NULL;
  const mxArray *c2_rhs170 = NULL;
  const mxArray *c2_lhs170 = NULL;
  const mxArray *c2_rhs171 = NULL;
  const mxArray *c2_lhs171 = NULL;
  const mxArray *c2_rhs172 = NULL;
  const mxArray *c2_lhs172 = NULL;
  const mxArray *c2_rhs173 = NULL;
  const mxArray *c2_lhs173 = NULL;
  const mxArray *c2_rhs174 = NULL;
  const mxArray *c2_lhs174 = NULL;
  const mxArray *c2_rhs175 = NULL;
  const mxArray *c2_lhs175 = NULL;
  const mxArray *c2_rhs176 = NULL;
  const mxArray *c2_lhs176 = NULL;
  const mxArray *c2_rhs177 = NULL;
  const mxArray *c2_lhs177 = NULL;
  const mxArray *c2_rhs178 = NULL;
  const mxArray *c2_lhs178 = NULL;
  const mxArray *c2_rhs179 = NULL;
  const mxArray *c2_lhs179 = NULL;
  const mxArray *c2_rhs180 = NULL;
  const mxArray *c2_lhs180 = NULL;
  const mxArray *c2_rhs181 = NULL;
  const mxArray *c2_lhs181 = NULL;
  const mxArray *c2_rhs182 = NULL;
  const mxArray *c2_lhs182 = NULL;
  const mxArray *c2_rhs183 = NULL;
  const mxArray *c2_lhs183 = NULL;
  const mxArray *c2_rhs184 = NULL;
  const mxArray *c2_lhs184 = NULL;
  const mxArray *c2_rhs185 = NULL;
  const mxArray *c2_lhs185 = NULL;
  const mxArray *c2_rhs186 = NULL;
  const mxArray *c2_lhs186 = NULL;
  const mxArray *c2_rhs187 = NULL;
  const mxArray *c2_lhs187 = NULL;
  const mxArray *c2_rhs188 = NULL;
  const mxArray *c2_lhs188 = NULL;
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m!common_checks"),
                  "context", "context", 128);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 128);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 128);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 128);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1363689356U), "fileTimeLo",
                  "fileTimeLo", 128);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 128);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 128);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 128);
  sf_mex_assign(&c2_rhs128, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs128, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs128), "rhs", "rhs",
                  128);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs128), "lhs", "lhs",
                  128);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_sort.m"), "context",
                  "context", 129);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_index_minus"), "name",
                  "name", 129);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 129);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_minus.m"),
                  "resolved", "resolved", 129);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1286797178U), "fileTimeLo",
                  "fileTimeLo", 129);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 129);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 129);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 129);
  sf_mex_assign(&c2_rhs129, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs129, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs129), "rhs", "rhs",
                  129);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs129), "lhs", "lhs",
                  129);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_sort.m"), "context",
                  "context", 130);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_index_times"), "name",
                  "name", 130);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 130);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_times.m"),
                  "resolved", "resolved", 130);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1286797180U), "fileTimeLo",
                  "fileTimeLo", 130);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 130);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 130);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 130);
  sf_mex_assign(&c2_rhs130, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs130, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs130), "rhs", "rhs",
                  130);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs130), "lhs", "lhs",
                  130);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_sort.m"), "context",
                  "context", 131);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_matrix_npages"), "name",
                  "name", 131);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 131);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_matrix_npages.m"),
                  "resolved", "resolved", 131);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1360257150U), "fileTimeLo",
                  "fileTimeLo", 131);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 131);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 131);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 131);
  sf_mex_assign(&c2_rhs131, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs131, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs131), "rhs", "rhs",
                  131);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs131), "lhs", "lhs",
                  131);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_matrix_npages.m"),
                  "context", "context", 132);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.prodsize"),
                  "name", "name", 132);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 132);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/prodsize.m"),
                  "resolved", "resolved", 132);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1360257388U), "fileTimeLo",
                  "fileTimeLo", 132);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 132);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 132);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 132);
  sf_mex_assign(&c2_rhs132, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs132, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs132), "rhs", "rhs",
                  132);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs132), "lhs", "lhs",
                  132);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_sort.m"), "context",
                  "context", 133);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "eml_int_forloop_overflow_check"), "name", "name", 133);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 133);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m"),
                  "resolved", "resolved", 133);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1346488740U), "fileTimeLo",
                  "fileTimeLo", 133);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 133);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 133);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 133);
  sf_mex_assign(&c2_rhs133, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs133, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs133), "rhs", "rhs",
                  133);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs133), "lhs", "lhs",
                  133);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_sort.m"), "context",
                  "context", 134);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_index_plus"), "name",
                  "name", 134);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 134);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m"),
                  "resolved", "resolved", 134);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1286797178U), "fileTimeLo",
                  "fileTimeLo", 134);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 134);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 134);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 134);
  sf_mex_assign(&c2_rhs134, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs134, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs134), "rhs", "rhs",
                  134);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs134), "lhs", "lhs",
                  134);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_sort.m"), "context",
                  "context", 135);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_index_plus"), "name",
                  "name", 135);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 135);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m"),
                  "resolved", "resolved", 135);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1286797178U), "fileTimeLo",
                  "fileTimeLo", 135);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 135);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 135);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 135);
  sf_mex_assign(&c2_rhs135, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs135, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs135), "rhs", "rhs",
                  135);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs135), "lhs", "lhs",
                  135);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_sort.m"), "context",
                  "context", 136);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_sort_idx"), "name", "name",
                  136);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 136);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_sort_idx.m"), "resolved",
                  "resolved", 136);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1305296404U), "fileTimeLo",
                  "fileTimeLo", 136);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 136);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 136);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 136);
  sf_mex_assign(&c2_rhs136, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs136, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs136), "rhs", "rhs",
                  136);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs136), "lhs", "lhs",
                  136);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_sort_idx.m"), "context",
                  "context", 137);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_index_class"), "name",
                  "name", 137);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 137);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m"),
                  "resolved", "resolved", 137);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1323145378U), "fileTimeLo",
                  "fileTimeLo", 137);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 137);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 137);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 137);
  sf_mex_assign(&c2_rhs137, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs137, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs137), "rhs", "rhs",
                  137);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs137), "lhs", "lhs",
                  137);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_sort_idx.m"), "context",
                  "context", 138);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_index_plus"), "name",
                  "name", 138);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 138);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m"),
                  "resolved", "resolved", 138);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1286797178U), "fileTimeLo",
                  "fileTimeLo", 138);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 138);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 138);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 138);
  sf_mex_assign(&c2_rhs138, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs138, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs138), "rhs", "rhs",
                  138);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs138), "lhs", "lhs",
                  138);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_sort_idx.m"), "context",
                  "context", 139);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "eml_int_forloop_overflow_check"), "name", "name", 139);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 139);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m"),
                  "resolved", "resolved", 139);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1346488740U), "fileTimeLo",
                  "fileTimeLo", 139);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 139);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 139);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 139);
  sf_mex_assign(&c2_rhs139, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs139, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs139), "rhs", "rhs",
                  139);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs139), "lhs", "lhs",
                  139);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_sort_idx.m"), "context",
                  "context", 140);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_index_minus"), "name",
                  "name", 140);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 140);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_minus.m"),
                  "resolved", "resolved", 140);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1286797178U), "fileTimeLo",
                  "fileTimeLo", 140);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 140);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 140);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 140);
  sf_mex_assign(&c2_rhs140, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs140, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs140), "rhs", "rhs",
                  140);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs140), "lhs", "lhs",
                  140);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_sort_idx.m"), "context",
                  "context", 141);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_index_plus"), "name",
                  "name", 141);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 141);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m"),
                  "resolved", "resolved", 141);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1286797178U), "fileTimeLo",
                  "fileTimeLo", 141);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 141);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 141);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 141);
  sf_mex_assign(&c2_rhs141, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs141, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs141), "rhs", "rhs",
                  141);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs141), "lhs", "lhs",
                  141);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_sort_idx.m"), "context",
                  "context", 142);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_sort_le"), "name", "name",
                  142);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 142);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_sort_le.m"), "resolved",
                  "resolved", 142);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1292165310U), "fileTimeLo",
                  "fileTimeLo", 142);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 142);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 142);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 142);
  sf_mex_assign(&c2_rhs142, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs142, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs142), "rhs", "rhs",
                  142);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs142), "lhs", "lhs",
                  142);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_sort_le.m!eml_sort_ascending_le"),
                  "context", "context", 143);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_relop"), "name", "name",
                  143);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("function_handle"),
                  "dominantType", "dominantType", 143);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_relop.m"), "resolved",
                  "resolved", 143);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1342429582U), "fileTimeLo",
                  "fileTimeLo", 143);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 143);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 143);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 143);
  sf_mex_assign(&c2_rhs143, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs143, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs143), "rhs", "rhs",
                  143);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs143), "lhs", "lhs",
                  143);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_sort_le.m!eml_sort_ascending_le"),
                  "context", "context", 144);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("isnan"), "name", "name", 144);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 144);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isnan.m"), "resolved",
                  "resolved", 144);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1363688658U), "fileTimeLo",
                  "fileTimeLo", 144);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 144);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 144);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 144);
  sf_mex_assign(&c2_rhs144, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs144, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs144), "rhs", "rhs",
                  144);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs144), "lhs", "lhs",
                  144);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_sort_idx.m"), "context",
                  "context", 145);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_index_times"), "name",
                  "name", 145);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 145);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_times.m"),
                  "resolved", "resolved", 145);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1286797180U), "fileTimeLo",
                  "fileTimeLo", 145);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 145);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 145);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 145);
  sf_mex_assign(&c2_rhs145, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs145, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs145), "rhs", "rhs",
                  145);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs145), "lhs", "lhs",
                  145);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_sort_idx.m"), "context",
                  "context", 146);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_index_minus"), "name",
                  "name", 146);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 146);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_minus.m"),
                  "resolved", "resolved", 146);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1286797178U), "fileTimeLo",
                  "fileTimeLo", 146);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 146);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 146);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 146);
  sf_mex_assign(&c2_rhs146, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs146, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs146), "rhs", "rhs",
                  146);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs146), "lhs", "lhs",
                  146);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "context", "context", 147);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("cos"), "name", "name", 147);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 147);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/cos.m"), "resolved",
                  "resolved", 147);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1343808772U), "fileTimeLo",
                  "fileTimeLo", 147);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 147);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 147);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 147);
  sf_mex_assign(&c2_rhs147, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs147, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs147), "rhs", "rhs",
                  147);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs147), "lhs", "lhs",
                  147);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "context", "context", 148);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("sin"), "name", "name", 148);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 148);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sin.m"), "resolved",
                  "resolved", 148);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1343808786U), "fileTimeLo",
                  "fileTimeLo", 148);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 148);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 148);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 148);
  sf_mex_assign(&c2_rhs148, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs148, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs148), "rhs", "rhs",
                  148);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs148), "lhs", "lhs",
                  148);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mtimes.m"), "context",
                  "context", 149);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_xdotu"), "name", "name",
                  149);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 149);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xdotu.m"),
                  "resolved", "resolved", 149);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1363688670U), "fileTimeLo",
                  "fileTimeLo", 149);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 149);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 149);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 149);
  sf_mex_assign(&c2_rhs149, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs149, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs149), "rhs", "rhs",
                  149);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs149), "lhs", "lhs",
                  149);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xdotu.m"), "context",
                  "context", 150);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_blas_inline"), "name",
                  "name", 150);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 150);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m"),
                  "resolved", "resolved", 150);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1299051568U), "fileTimeLo",
                  "fileTimeLo", 150);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 150);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 150);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 150);
  sf_mex_assign(&c2_rhs150, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs150, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs150), "rhs", "rhs",
                  150);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs150), "lhs", "lhs",
                  150);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xdotu.m"), "context",
                  "context", 151);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_xdot"), "name", "name",
                  151);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 151);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xdot.m"), "resolved",
                  "resolved", 151);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1363688668U), "fileTimeLo",
                  "fileTimeLo", 151);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 151);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 151);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 151);
  sf_mex_assign(&c2_rhs151, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs151, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs151), "rhs", "rhs",
                  151);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs151), "lhs", "lhs",
                  151);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_xdot.m"), "context",
                  "context", 152);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_blas_inline"), "name",
                  "name", 152);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 152);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/eml_blas_inline.m"),
                  "resolved", "resolved", 152);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1299051568U), "fileTimeLo",
                  "fileTimeLo", 152);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 152);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 152);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 152);
  sf_mex_assign(&c2_rhs152, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs152, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs152), "rhs", "rhs",
                  152);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs152), "lhs", "lhs",
                  152);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xdot.m"),
                  "context", "context", 153);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_index_class"), "name",
                  "name", 153);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 153);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m"),
                  "resolved", "resolved", 153);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1323145378U), "fileTimeLo",
                  "fileTimeLo", 153);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 153);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 153);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 153);
  sf_mex_assign(&c2_rhs153, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs153, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs153), "rhs", "rhs",
                  153);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs153), "lhs", "lhs",
                  153);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/external/eml_blas_xdot.m"),
                  "context", "context", 154);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_refblas_xdot"), "name",
                  "name", 154);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 154);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xdot.m"),
                  "resolved", "resolved", 154);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1299051572U), "fileTimeLo",
                  "fileTimeLo", 154);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 154);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 154);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 154);
  sf_mex_assign(&c2_rhs154, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs154, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs154), "rhs", "rhs",
                  154);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs154), "lhs", "lhs",
                  154);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xdot.m"),
                  "context", "context", 155);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_refblas_xdotx"), "name",
                  "name", 155);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 155);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xdotx.m"),
                  "resolved", "resolved", 155);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1360257150U), "fileTimeLo",
                  "fileTimeLo", 155);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 155);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 155);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 155);
  sf_mex_assign(&c2_rhs155, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs155, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs155), "rhs", "rhs",
                  155);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs155), "lhs", "lhs",
                  155);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xdotx.m"),
                  "context", "context", 156);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_scalar_eg"), "name",
                  "name", 156);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 156);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "resolved",
                  "resolved", 156);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1286797196U), "fileTimeLo",
                  "fileTimeLo", 156);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 156);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 156);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 156);
  sf_mex_assign(&c2_rhs156, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs156, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs156), "rhs", "rhs",
                  156);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs156), "lhs", "lhs",
                  156);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xdotx.m"),
                  "context", "context", 157);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_index_class"), "name",
                  "name", 157);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 157);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m"),
                  "resolved", "resolved", 157);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1323145378U), "fileTimeLo",
                  "fileTimeLo", 157);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 157);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 157);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 157);
  sf_mex_assign(&c2_rhs157, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs157, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs157), "rhs", "rhs",
                  157);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs157), "lhs", "lhs",
                  157);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xdotx.m"),
                  "context", "context", 158);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_index_minus"), "name",
                  "name", 158);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 158);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_minus.m"),
                  "resolved", "resolved", 158);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1286797178U), "fileTimeLo",
                  "fileTimeLo", 158);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 158);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 158);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 158);
  sf_mex_assign(&c2_rhs158, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs158, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs158), "rhs", "rhs",
                  158);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs158), "lhs", "lhs",
                  158);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xdotx.m"),
                  "context", "context", 159);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_index_times"), "name",
                  "name", 159);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 159);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_times.m"),
                  "resolved", "resolved", 159);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1286797180U), "fileTimeLo",
                  "fileTimeLo", 159);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 159);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 159);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 159);
  sf_mex_assign(&c2_rhs159, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs159, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs159), "rhs", "rhs",
                  159);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs159), "lhs", "lhs",
                  159);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xdotx.m"),
                  "context", "context", 160);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_index_plus"), "name",
                  "name", 160);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 160);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m"),
                  "resolved", "resolved", 160);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1286797178U), "fileTimeLo",
                  "fileTimeLo", 160);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 160);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 160);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 160);
  sf_mex_assign(&c2_rhs160, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs160, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs160), "rhs", "rhs",
                  160);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs160), "lhs", "lhs",
                  160);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/blas/refblas/eml_refblas_xdotx.m"),
                  "context", "context", 161);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "eml_int_forloop_overflow_check"), "name", "name", 161);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 161);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m"),
                  "resolved", "resolved", 161);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1346488740U), "fileTimeLo",
                  "fileTimeLo", 161);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 161);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 161);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 161);
  sf_mex_assign(&c2_rhs161, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs161, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs161), "rhs", "rhs",
                  161);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs161), "lhs", "lhs",
                  161);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "context", "context", 162);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("sign"), "name", "name", 162);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 162);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sign.m"), "resolved",
                  "resolved", 162);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1363688656U), "fileTimeLo",
                  "fileTimeLo", 162);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 162);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 162);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 162);
  sf_mex_assign(&c2_rhs162, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs162, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs162), "rhs", "rhs",
                  162);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs162), "lhs", "lhs",
                  162);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sign.m"), "context",
                  "context", 163);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 163);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 163);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 163);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1363689356U), "fileTimeLo",
                  "fileTimeLo", 163);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 163);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 163);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 163);
  sf_mex_assign(&c2_rhs163, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs163, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs163), "rhs", "rhs",
                  163);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs163), "lhs", "lhs",
                  163);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/sign.m"), "context",
                  "context", 164);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_scalar_sign"), "name",
                  "name", 164);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 164);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_sign.m"),
                  "resolved", "resolved", 164);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1356516294U), "fileTimeLo",
                  "fileTimeLo", 164);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 164);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 164);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 164);
  sf_mex_assign(&c2_rhs164, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs164, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs164), "rhs", "rhs",
                  164);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs164), "lhs", "lhs",
                  164);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "context", "context", 165);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("atan2"), "name", "name", 165);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 165);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/atan2.m"), "resolved",
                  "resolved", 165);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1343808772U), "fileTimeLo",
                  "fileTimeLo", 165);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 165);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 165);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 165);
  sf_mex_assign(&c2_rhs165, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs165, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs165), "rhs", "rhs",
                  165);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs165), "lhs", "lhs",
                  165);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/atan2.m"), "context",
                  "context", 166);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_scalar_eg"), "name",
                  "name", 166);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 166);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "resolved",
                  "resolved", 166);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1286797196U), "fileTimeLo",
                  "fileTimeLo", 166);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 166);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 166);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 166);
  sf_mex_assign(&c2_rhs166, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs166, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs166), "rhs", "rhs",
                  166);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs166), "lhs", "lhs",
                  166);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/atan2.m"), "context",
                  "context", 167);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_scalexp_alloc"), "name",
                  "name", 167);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 167);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m"),
                  "resolved", "resolved", 167);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1358160940U), "fileTimeLo",
                  "fileTimeLo", 167);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 167);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 167);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 167);
  sf_mex_assign(&c2_rhs167, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs167, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs167), "rhs", "rhs",
                  167);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs167), "lhs", "lhs",
                  167);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/atan2.m"), "context",
                  "context", 168);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_scalar_atan2"), "name",
                  "name", 168);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 168);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/eml_scalar_atan2.m"),
                  "resolved", "resolved", 168);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1286797120U), "fileTimeLo",
                  "fileTimeLo", 168);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 168);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 168);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 168);
  sf_mex_assign(&c2_rhs168, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs168, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs168), "rhs", "rhs",
                  168);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs168), "lhs", "lhs",
                  168);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "context", "context", 169);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("abs"), "name", "name", 169);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 169);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elfun/abs.m"), "resolved",
                  "resolved", 169);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1363688652U), "fileTimeLo",
                  "fileTimeLo", 169);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 169);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 169);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 169);
  sf_mex_assign(&c2_rhs169, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs169, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs169), "rhs", "rhs",
                  169);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs169), "lhs", "lhs",
                  169);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "context", "context", 170);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("min"), "name", "name", 170);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 170);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/min.m"), "resolved",
                  "resolved", 170);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1311233718U), "fileTimeLo",
                  "fileTimeLo", 170);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 170);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 170);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 170);
  sf_mex_assign(&c2_rhs170, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs170, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs170), "rhs", "rhs",
                  170);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs170), "lhs", "lhs",
                  170);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/datafun/min.m"), "context",
                  "context", 171);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_min_or_max"), "name",
                  "name", 171);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("char"), "dominantType",
                  "dominantType", 171);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m"),
                  "resolved", "resolved", 171);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1363688670U), "fileTimeLo",
                  "fileTimeLo", 171);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 171);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 171);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 171);
  sf_mex_assign(&c2_rhs171, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs171, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs171), "rhs", "rhs",
                  171);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs171), "lhs", "lhs",
                  171);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_extremum"),
                  "context", "context", 172);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_const_nonsingleton_dim"),
                  "name", "name", 172);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 172);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_const_nonsingleton_dim.m"),
                  "resolved", "resolved", 172);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1286797096U), "fileTimeLo",
                  "fileTimeLo", 172);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 172);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 172);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 172);
  sf_mex_assign(&c2_rhs172, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs172, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs172), "rhs", "rhs",
                  172);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs172), "lhs", "lhs",
                  172);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_extremum"),
                  "context", "context", 173);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_scalar_eg"), "name",
                  "name", 173);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 173);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "resolved",
                  "resolved", 173);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1286797196U), "fileTimeLo",
                  "fileTimeLo", 173);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 173);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 173);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 173);
  sf_mex_assign(&c2_rhs173, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs173, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs173), "rhs", "rhs",
                  173);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs173), "lhs", "lhs",
                  173);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_extremum"),
                  "context", "context", 174);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_index_class"), "name",
                  "name", 174);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 174);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m"),
                  "resolved", "resolved", 174);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1323145378U), "fileTimeLo",
                  "fileTimeLo", 174);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 174);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 174);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 174);
  sf_mex_assign(&c2_rhs174, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs174, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs174), "rhs", "rhs",
                  174);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs174), "lhs", "lhs",
                  174);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_extremum_sub"),
                  "context", "context", 175);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_index_class"), "name",
                  "name", 175);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 175);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m"),
                  "resolved", "resolved", 175);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1323145378U), "fileTimeLo",
                  "fileTimeLo", 175);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 175);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 175);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 175);
  sf_mex_assign(&c2_rhs175, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs175, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs175), "rhs", "rhs",
                  175);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs175), "lhs", "lhs",
                  175);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_extremum_sub"),
                  "context", "context", 176);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("isnan"), "name", "name", 176);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 176);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/isnan.m"), "resolved",
                  "resolved", 176);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1363688658U), "fileTimeLo",
                  "fileTimeLo", 176);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 176);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 176);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 176);
  sf_mex_assign(&c2_rhs176, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs176, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs176), "rhs", "rhs",
                  176);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs176), "lhs", "lhs",
                  176);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_extremum_sub"),
                  "context", "context", 177);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_index_plus"), "name",
                  "name", 177);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("coder.internal.indexInt"),
                  "dominantType", "dominantType", 177);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_plus.m"),
                  "resolved", "resolved", 177);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1286797178U), "fileTimeLo",
                  "fileTimeLo", 177);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 177);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 177);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 177);
  sf_mex_assign(&c2_rhs177, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs177, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs177), "rhs", "rhs",
                  177);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs177), "lhs", "lhs",
                  177);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_extremum_sub"),
                  "context", "context", 178);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "eml_int_forloop_overflow_check"), "name", "name", 178);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 178);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m"),
                  "resolved", "resolved", 178);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1346488740U), "fileTimeLo",
                  "fileTimeLo", 178);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 178);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 178);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 178);
  sf_mex_assign(&c2_rhs178, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs178, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs178), "rhs", "rhs",
                  178);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs178), "lhs", "lhs",
                  178);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_extremum_sub"),
                  "context", "context", 179);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_relop"), "name", "name",
                  179);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("function_handle"),
                  "dominantType", "dominantType", 179);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_relop.m"), "resolved",
                  "resolved", 179);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1342429582U), "fileTimeLo",
                  "fileTimeLo", 179);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 179);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 179);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 179);
  sf_mex_assign(&c2_rhs179, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs179, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs179), "rhs", "rhs",
                  179);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs179), "lhs", "lhs",
                  179);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "context", "context", 180);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("mpower"), "name", "name", 180);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 180);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mpower.m"), "resolved",
                  "resolved", 180);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1363688678U), "fileTimeLo",
                  "fileTimeLo", 180);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 180);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 180);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 180);
  sf_mex_assign(&c2_rhs180, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs180, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs180), "rhs", "rhs",
                  180);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs180), "lhs", "lhs",
                  180);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mpower.m"), "context",
                  "context", 181);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 181);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 181);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 181);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1363689356U), "fileTimeLo",
                  "fileTimeLo", 181);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 181);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 181);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 181);
  sf_mex_assign(&c2_rhs181, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs181, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs181), "rhs", "rhs",
                  181);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs181), "lhs", "lhs",
                  181);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mpower.m"), "context",
                  "context", 182);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("ismatrix"), "name", "name",
                  182);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 182);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/elmat/ismatrix.m"), "resolved",
                  "resolved", 182);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1331279658U), "fileTimeLo",
                  "fileTimeLo", 182);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 182);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 182);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 182);
  sf_mex_assign(&c2_rhs182, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs182, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs182), "rhs", "rhs",
                  182);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs182), "lhs", "lhs",
                  182);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/mpower.m"), "context",
                  "context", 183);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("power"), "name", "name", 183);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 183);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/ops/power.m"), "resolved",
                  "resolved", 183);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1363688680U), "fileTimeLo",
                  "fileTimeLo", 183);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 183);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 183);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 183);
  sf_mex_assign(&c2_rhs183, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs183, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs183), "rhs", "rhs",
                  183);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs183), "lhs", "lhs",
                  183);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum"),
                  "context", "context", 184);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_scalar_eg"), "name",
                  "name", 184);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 184);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "resolved",
                  "resolved", 184);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1286797196U), "fileTimeLo",
                  "fileTimeLo", 184);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 184);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 184);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 184);
  sf_mex_assign(&c2_rhs184, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs184, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs184), "rhs", "rhs",
                  184);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs184), "lhs", "lhs",
                  184);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum"),
                  "context", "context", 185);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_scalexp_alloc"), "name",
                  "name", 185);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 185);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalexp_alloc.m"),
                  "resolved", "resolved", 185);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1358160940U), "fileTimeLo",
                  "fileTimeLo", 185);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 185);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 185);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 185);
  sf_mex_assign(&c2_rhs185, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs185, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs185), "rhs", "rhs",
                  185);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs185), "lhs", "lhs",
                  185);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_bin_extremum"),
                  "context", "context", 186);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_index_class"), "name",
                  "name", 186);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(""), "dominantType",
                  "dominantType", 186);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_index_class.m"),
                  "resolved", "resolved", 186);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1323145378U), "fileTimeLo",
                  "fileTimeLo", 186);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 186);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 186);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 186);
  sf_mex_assign(&c2_rhs186, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs186, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs186), "rhs", "rhs",
                  186);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs186), "lhs", "lhs",
                  186);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_scalar_bin_extremum"),
                  "context", "context", 187);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("eml_scalar_eg"), "name",
                  "name", 187);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 187);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_scalar_eg.m"), "resolved",
                  "resolved", 187);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1286797196U), "fileTimeLo",
                  "fileTimeLo", 187);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 187);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 187);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 187);
  sf_mex_assign(&c2_rhs187, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs187, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs187), "rhs", "rhs",
                  187);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs187), "lhs", "lhs",
                  187);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[ILXE]$matlabroot$/toolbox/eml/lib/matlab/eml/eml_min_or_max.m!eml_scalar_bin_extremum"),
                  "context", "context", 188);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "coder.internal.isBuiltInNumeric"), "name", "name", 188);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut("double"), "dominantType",
                  "dominantType", 188);
  sf_mex_addfield(*c2_info, c2_emlrt_marshallOut(
    "[IXE]$matlabroot$/toolbox/shared/coder/coder/+coder/+internal/isBuiltInNumeric.m"),
                  "resolved", "resolved", 188);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(1363689356U), "fileTimeLo",
                  "fileTimeLo", 188);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "fileTimeHi",
                  "fileTimeHi", 188);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeLo",
                  "mFileTimeLo", 188);
  sf_mex_addfield(*c2_info, c2_b_emlrt_marshallOut(0U), "mFileTimeHi",
                  "mFileTimeHi", 188);
  sf_mex_assign(&c2_rhs188, sf_mex_createcellarray(0), FALSE);
  sf_mex_assign(&c2_lhs188, sf_mex_createcellarray(0), FALSE);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_rhs188), "rhs", "rhs",
                  188);
  sf_mex_addfield(*c2_info, sf_mex_duplicatearraysafe(&c2_lhs188), "lhs", "lhs",
                  188);
  sf_mex_destroy(&c2_rhs128);
  sf_mex_destroy(&c2_lhs128);
  sf_mex_destroy(&c2_rhs129);
  sf_mex_destroy(&c2_lhs129);
  sf_mex_destroy(&c2_rhs130);
  sf_mex_destroy(&c2_lhs130);
  sf_mex_destroy(&c2_rhs131);
  sf_mex_destroy(&c2_lhs131);
  sf_mex_destroy(&c2_rhs132);
  sf_mex_destroy(&c2_lhs132);
  sf_mex_destroy(&c2_rhs133);
  sf_mex_destroy(&c2_lhs133);
  sf_mex_destroy(&c2_rhs134);
  sf_mex_destroy(&c2_lhs134);
  sf_mex_destroy(&c2_rhs135);
  sf_mex_destroy(&c2_lhs135);
  sf_mex_destroy(&c2_rhs136);
  sf_mex_destroy(&c2_lhs136);
  sf_mex_destroy(&c2_rhs137);
  sf_mex_destroy(&c2_lhs137);
  sf_mex_destroy(&c2_rhs138);
  sf_mex_destroy(&c2_lhs138);
  sf_mex_destroy(&c2_rhs139);
  sf_mex_destroy(&c2_lhs139);
  sf_mex_destroy(&c2_rhs140);
  sf_mex_destroy(&c2_lhs140);
  sf_mex_destroy(&c2_rhs141);
  sf_mex_destroy(&c2_lhs141);
  sf_mex_destroy(&c2_rhs142);
  sf_mex_destroy(&c2_lhs142);
  sf_mex_destroy(&c2_rhs143);
  sf_mex_destroy(&c2_lhs143);
  sf_mex_destroy(&c2_rhs144);
  sf_mex_destroy(&c2_lhs144);
  sf_mex_destroy(&c2_rhs145);
  sf_mex_destroy(&c2_lhs145);
  sf_mex_destroy(&c2_rhs146);
  sf_mex_destroy(&c2_lhs146);
  sf_mex_destroy(&c2_rhs147);
  sf_mex_destroy(&c2_lhs147);
  sf_mex_destroy(&c2_rhs148);
  sf_mex_destroy(&c2_lhs148);
  sf_mex_destroy(&c2_rhs149);
  sf_mex_destroy(&c2_lhs149);
  sf_mex_destroy(&c2_rhs150);
  sf_mex_destroy(&c2_lhs150);
  sf_mex_destroy(&c2_rhs151);
  sf_mex_destroy(&c2_lhs151);
  sf_mex_destroy(&c2_rhs152);
  sf_mex_destroy(&c2_lhs152);
  sf_mex_destroy(&c2_rhs153);
  sf_mex_destroy(&c2_lhs153);
  sf_mex_destroy(&c2_rhs154);
  sf_mex_destroy(&c2_lhs154);
  sf_mex_destroy(&c2_rhs155);
  sf_mex_destroy(&c2_lhs155);
  sf_mex_destroy(&c2_rhs156);
  sf_mex_destroy(&c2_lhs156);
  sf_mex_destroy(&c2_rhs157);
  sf_mex_destroy(&c2_lhs157);
  sf_mex_destroy(&c2_rhs158);
  sf_mex_destroy(&c2_lhs158);
  sf_mex_destroy(&c2_rhs159);
  sf_mex_destroy(&c2_lhs159);
  sf_mex_destroy(&c2_rhs160);
  sf_mex_destroy(&c2_lhs160);
  sf_mex_destroy(&c2_rhs161);
  sf_mex_destroy(&c2_lhs161);
  sf_mex_destroy(&c2_rhs162);
  sf_mex_destroy(&c2_lhs162);
  sf_mex_destroy(&c2_rhs163);
  sf_mex_destroy(&c2_lhs163);
  sf_mex_destroy(&c2_rhs164);
  sf_mex_destroy(&c2_lhs164);
  sf_mex_destroy(&c2_rhs165);
  sf_mex_destroy(&c2_lhs165);
  sf_mex_destroy(&c2_rhs166);
  sf_mex_destroy(&c2_lhs166);
  sf_mex_destroy(&c2_rhs167);
  sf_mex_destroy(&c2_lhs167);
  sf_mex_destroy(&c2_rhs168);
  sf_mex_destroy(&c2_lhs168);
  sf_mex_destroy(&c2_rhs169);
  sf_mex_destroy(&c2_lhs169);
  sf_mex_destroy(&c2_rhs170);
  sf_mex_destroy(&c2_lhs170);
  sf_mex_destroy(&c2_rhs171);
  sf_mex_destroy(&c2_lhs171);
  sf_mex_destroy(&c2_rhs172);
  sf_mex_destroy(&c2_lhs172);
  sf_mex_destroy(&c2_rhs173);
  sf_mex_destroy(&c2_lhs173);
  sf_mex_destroy(&c2_rhs174);
  sf_mex_destroy(&c2_lhs174);
  sf_mex_destroy(&c2_rhs175);
  sf_mex_destroy(&c2_lhs175);
  sf_mex_destroy(&c2_rhs176);
  sf_mex_destroy(&c2_lhs176);
  sf_mex_destroy(&c2_rhs177);
  sf_mex_destroy(&c2_lhs177);
  sf_mex_destroy(&c2_rhs178);
  sf_mex_destroy(&c2_lhs178);
  sf_mex_destroy(&c2_rhs179);
  sf_mex_destroy(&c2_lhs179);
  sf_mex_destroy(&c2_rhs180);
  sf_mex_destroy(&c2_lhs180);
  sf_mex_destroy(&c2_rhs181);
  sf_mex_destroy(&c2_lhs181);
  sf_mex_destroy(&c2_rhs182);
  sf_mex_destroy(&c2_lhs182);
  sf_mex_destroy(&c2_rhs183);
  sf_mex_destroy(&c2_lhs183);
  sf_mex_destroy(&c2_rhs184);
  sf_mex_destroy(&c2_lhs184);
  sf_mex_destroy(&c2_rhs185);
  sf_mex_destroy(&c2_lhs185);
  sf_mex_destroy(&c2_rhs186);
  sf_mex_destroy(&c2_lhs186);
  sf_mex_destroy(&c2_rhs187);
  sf_mex_destroy(&c2_lhs187);
  sf_mex_destroy(&c2_rhs188);
  sf_mex_destroy(&c2_lhs188);
}

static void c2_IR_Raw_to_Distances
  (SFc2_simiam_AvoidObstacle_BestAOInstanceStruct *chartInstance, real_T
   c2_ir_raw[9], real_T c2_Answer[9], real_T c2_ObstacleDetected_data[9],
   int32_T c2_ObstacleDetected_sizes[1])
{
  uint32_T c2_debug_family_var_map[9];
  real_T c2_SafeNumber;
  real_T c2_MaxNumber;
  real_T c2_ir_raw_Internal[9];
  boolean_T c2_Number_List[9];
  real_T c2_nargin = 1.0;
  real_T c2_nargout = 2.0;
  int32_T c2_i156;
  int32_T c2_i157;
  int32_T c2_i158;
  boolean_T c2_x[9];
  int32_T c2_idx;
  static int32_T c2_iv0[1] = { 9 };

  int32_T c2_ii_sizes;
  int32_T c2_ii;
  int32_T c2_b_ii;
  int32_T c2_a;
  int32_T c2_ii_data[9];
  boolean_T c2_b0;
  boolean_T c2_b1;
  boolean_T c2_b2;
  int32_T c2_i159;
  int32_T c2_tmp_sizes;
  int32_T c2_loop_ub;
  int32_T c2_i160;
  int32_T c2_tmp_data[9];
  int32_T c2_b_ii_sizes;
  int32_T c2_b_loop_ub;
  int32_T c2_i161;
  int32_T c2_b_ii_data[9];
  int32_T c2_c_loop_ub;
  int32_T c2_i162;
  int32_T c2_d_loop_ub;
  int32_T c2_i163;
  int32_T c2_i164;
  boolean_T c2_b_Number_List[9];
  int32_T c2_e_loop_ub;
  int32_T c2_i165;
  int32_T c2_i166;
  int32_T c2_i167;
  boolean_T c2_c_Number_List[9];
  int32_T c2_f_loop_ub;
  int32_T c2_i168;
  int32_T c2_i169;
  real_T c2_A[9];
  int32_T c2_i170;
  int32_T c2_k;
  real_T c2_b_k;
  int32_T c2_c_k;
  real_T c2_b_x;
  real_T c2_c_x;
  int32_T c2_i171;
  int32_T c2_i172;
  boolean_T exitg1;
  boolean_T guard1 = FALSE;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 9U, 9U, c2_b_debug_family_names,
    c2_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_SafeNumber, 0U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_MaxNumber, 1U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_ir_raw_Internal, 2U,
    c2_d_sf_marshallOut, c2_h_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_Number_List, 3U, c2_i_sf_marshallOut,
    c2_i_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_nargin, 4U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_nargout, 5U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_ir_raw, 6U, c2_d_sf_marshallOut,
    c2_h_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_Answer, 7U, c2_d_sf_marshallOut,
    c2_h_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_DYN_IMPORTABLE(c2_ObstacleDetected_data, (const
    int32_T *)c2_ObstacleDetected_sizes, NULL, 0, 8, (void *)c2_h_sf_marshallOut,
    (void *)c2_g_sf_marshallIn);
  CV_EML_FCN(0, 2);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 52);
  c2_SafeNumber = 150.0;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 53);
  c2_MaxNumber = 3960.0;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 55);
  for (c2_i156 = 0; c2_i156 < 9; c2_i156++) {
    c2_ir_raw_Internal[c2_i156] = c2_ir_raw[c2_i156];
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 58);
  for (c2_i157 = 0; c2_i157 < 9; c2_i157++) {
    c2_Number_List[c2_i157] = (c2_ir_raw_Internal[c2_i157] < 150.0);
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 61);
  for (c2_i158 = 0; c2_i158 < 9; c2_i158++) {
    c2_x[c2_i158] = (c2_ir_raw_Internal[c2_i158] > 150.0);
  }

  c2_idx = 0;
  c2_ii_sizes = c2_iv0[0];
  c2_ii = 1;
  exitg1 = FALSE;
  while ((exitg1 == FALSE) && (c2_ii < 10)) {
    c2_b_ii = c2_ii;
    guard1 = FALSE;
    if (c2_x[c2_b_ii - 1]) {
      c2_a = c2_idx + 1;
      c2_idx = c2_a;
      c2_ii_data[c2_idx - 1] = c2_b_ii;
      if (c2_idx >= 9) {
        exitg1 = TRUE;
      } else {
        guard1 = TRUE;
      }
    } else {
      guard1 = TRUE;
    }

    if (guard1 == TRUE) {
      c2_ii++;
    }
  }

  c2_b0 = (1 > c2_idx);
  c2_b1 = c2_b0;
  c2_b2 = c2_b1;
  if (c2_b2) {
    c2_i159 = 0;
  } else {
    c2_i159 = _SFD_EML_ARRAY_BOUNDS_CHECK("", c2_idx, 1, 9, 0, 0);
  }

  c2_tmp_sizes = c2_i159;
  c2_loop_ub = c2_i159 - 1;
  for (c2_i160 = 0; c2_i160 <= c2_loop_ub; c2_i160++) {
    c2_tmp_data[c2_i160] = 1 + c2_i160;
  }

  _SFD_VECTOR_VECTOR_INDEX_CHECK(9, 1, 1, c2_tmp_sizes);
  c2_b_ii_sizes = c2_tmp_sizes;
  c2_b_loop_ub = c2_tmp_sizes - 1;
  for (c2_i161 = 0; c2_i161 <= c2_b_loop_ub; c2_i161++) {
    c2_b_ii_data[c2_i161] = c2_ii_data[c2_tmp_data[c2_i161] - 1];
  }

  c2_ii_sizes = c2_b_ii_sizes;
  c2_c_loop_ub = c2_b_ii_sizes - 1;
  for (c2_i162 = 0; c2_i162 <= c2_c_loop_ub; c2_i162++) {
    c2_ii_data[c2_i162] = c2_b_ii_data[c2_i162];
  }

  c2_ObstacleDetected_sizes[0] = c2_ii_sizes;
  c2_d_loop_ub = c2_ii_sizes - 1;
  for (c2_i163 = 0; c2_i163 <= c2_d_loop_ub; c2_i163++) {
    c2_ObstacleDetected_data[c2_i163] = (real_T)c2_ii_data[c2_i163];
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 63);
  for (c2_i164 = 0; c2_i164 < 9; c2_i164++) {
    c2_b_Number_List[c2_i164] = c2_Number_List[c2_i164];
  }

  c2_eml_li_find(chartInstance, c2_b_Number_List, c2_tmp_data, *(int32_T (*)[1])
                 &c2_tmp_sizes);
  c2_e_loop_ub = c2_tmp_sizes - 1;
  for (c2_i165 = 0; c2_i165 <= c2_e_loop_ub; c2_i165++) {
    c2_ir_raw_Internal[c2_tmp_data[c2_i165] - 1] = 150.0;
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 66);
  for (c2_i166 = 0; c2_i166 < 9; c2_i166++) {
    c2_Number_List[c2_i166] = (c2_ir_raw_Internal[c2_i166] > 3960.0);
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 67);
  for (c2_i167 = 0; c2_i167 < 9; c2_i167++) {
    c2_c_Number_List[c2_i167] = c2_Number_List[c2_i167];
  }

  c2_eml_li_find(chartInstance, c2_c_Number_List, c2_tmp_data, *(int32_T (*)[1])
                 &c2_tmp_sizes);
  c2_f_loop_ub = c2_tmp_sizes - 1;
  for (c2_i168 = 0; c2_i168 <= c2_f_loop_ub; c2_i168++) {
    c2_ir_raw_Internal[c2_tmp_data[c2_i168] - 1] = 3960.0;
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 69);
  for (c2_i169 = 0; c2_i169 < 9; c2_i169++) {
    c2_A[c2_i169] = c2_ir_raw_Internal[c2_i169];
  }

  for (c2_i170 = 0; c2_i170 < 9; c2_i170++) {
    c2_A[c2_i170] /= 3960.0;
  }

  for (c2_k = 0; c2_k < 9; c2_k++) {
    c2_b_k = 1.0 + (real_T)c2_k;
    if (c2_A[(int32_T)c2_b_k - 1] < 0.0) {
      c2_eml_error(chartInstance);
    }
  }

  for (c2_c_k = 0; c2_c_k < 9; c2_c_k++) {
    c2_b_k = 1.0 + (real_T)c2_c_k;
    c2_b_x = c2_A[(int32_T)c2_b_k - 1];
    c2_c_x = c2_b_x;
    c2_c_x = muDoubleScalarLog(c2_c_x);
    c2_A[(int32_T)c2_b_k - 1] = c2_c_x;
  }

  for (c2_i171 = 0; c2_i171 < 9; c2_i171++) {
    c2_A[c2_i171] /= 30.0;
  }

  for (c2_i172 = 0; c2_i172 < 9; c2_i172++) {
    c2_Answer[c2_i172] = 0.02 - c2_A[c2_i172];
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, -69);
  _SFD_SYMBOL_SCOPE_POP();
}

static void c2_eml_li_find(SFc2_simiam_AvoidObstacle_BestAOInstanceStruct
  *chartInstance, boolean_T c2_x[9], int32_T c2_y_data[9], int32_T c2_y_sizes[1])
{
  int32_T c2_k;
  int32_T c2_i;
  int32_T c2_b_i;
  int32_T c2_a;
  const mxArray *c2_y = NULL;
  int32_T c2_tmp_sizes;
  int32_T c2_loop_ub;
  int32_T c2_i173;
  int32_T c2_tmp_data[9];
  int32_T c2_j;
  int32_T c2_c_i;
  int32_T c2_d_i;
  int32_T c2_b_a;
  c2_k = 0;
  for (c2_i = 1; c2_i < 10; c2_i++) {
    c2_b_i = c2_i - 1;
    if (c2_x[c2_b_i]) {
      c2_a = c2_k + 1;
      c2_k = c2_a;
    }
  }

  if (c2_k <= 9) {
  } else {
    c2_y = NULL;
    sf_mex_assign(&c2_y, sf_mex_create("y", "Assertion failed.", 15, 0U, 0U, 0U,
      2, 1, strlen("Assertion failed.")), FALSE);
    sf_mex_call_debug("error", 0U, 1U, 14, c2_y);
  }

  c2_tmp_sizes = (int32_T)_SFD_NON_NEGATIVE_CHECK("", (real_T)c2_k);
  c2_loop_ub = (int32_T)_SFD_NON_NEGATIVE_CHECK("", (real_T)c2_k) - 1;
  for (c2_i173 = 0; c2_i173 <= c2_loop_ub; c2_i173++) {
    c2_tmp_data[c2_i173] = 0;
  }

  c2_y_sizes[0] = c2_tmp_sizes;
  c2_j = 1;
  for (c2_c_i = 1; c2_c_i < 10; c2_c_i++) {
    c2_d_i = c2_c_i;
    if (c2_x[c2_d_i - 1]) {
      c2_y_data[_SFD_EML_ARRAY_BOUNDS_CHECK("", c2_j, 1, c2_y_sizes[0], 1, 0) -
        1] = c2_d_i;
      c2_b_a = c2_j + 1;
      c2_j = c2_b_a;
    }
  }
}

static void c2_eml_error(SFc2_simiam_AvoidObstacle_BestAOInstanceStruct
  *chartInstance)
{
  int32_T c2_i174;
  static char_T c2_cv1[28] = { 'M', 'A', 'T', 'L', 'A', 'B', ':', 'r', 'e', 'a',
    'l', 'l', 'o', 'g', ':', 'c', 'o', 'm', 'p', 'l', 'e', 'x', 'R', 'e', 's',
    'u', 'l', 't' };

  char_T c2_u[28];
  const mxArray *c2_y = NULL;
  for (c2_i174 = 0; c2_i174 < 28; c2_i174++) {
    c2_u[c2_i174] = c2_cv1[c2_i174];
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 10, 0U, 1U, 0U, 2, 1, 28), FALSE);
  sf_mex_call_debug("error", 0U, 1U, 14, sf_mex_call_debug("message", 1U, 1U, 14,
    c2_y));
}

static void c2_GetIR_Position(SFc2_simiam_AvoidObstacle_BestAOInstanceStruct
  *chartInstance, real_T c2_IR_Number, real_T c2_Answer[3])
{
  uint32_T c2_debug_family_var_map[5];
  real_T c2_IR_Position[27];
  real_T c2_nargin = 1.0;
  real_T c2_nargout = 1.0;
  real_T c2_x;
  real_T c2_y;
  real_T c2_theta;
  real_T c2_b_nargin = 4.0;
  real_T c2_b_nargout = 1.0;
  int32_T c2_i175;
  static real_T c2_dv14[27] = { -0.038, 0.017, 0.051, 0.067, 0.067, 0.051, 0.017,
    -0.038, -0.052, 0.049, 0.063, 0.045, 0.015, -0.015, -0.045, -0.063, -0.049,
    0.0, 2.2340214425527418, 1.3089969389957472, 0.73303828583761843,
    0.22689280275926285, -0.22689280275926285, -0.73303828583761843,
    -1.3089969389957472, -2.2340214425527418, 3.1415926535897931 };

  int32_T c2_b_IR_Number;
  int32_T c2_i176;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 5U, 5U, c2_e_debug_family_names,
    c2_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_IR_Position, 0U, c2_j_sf_marshallOut,
    c2_j_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_nargin, 1U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_nargout, 2U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_IR_Number, 3U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_Answer, 4U, c2_c_sf_marshallOut,
    c2_e_sf_marshallIn);
  CV_EML_FCN(0, 3);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 73);
  c2_x = 0.0;
  c2_y = 0.0;
  c2_theta = 0.0;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 5U, 5U, c2_c_debug_family_names,
    c2_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_b_nargin, 0U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_b_nargout, 1U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_x, 2U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_y, 3U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_theta, 4U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  CV_SCRIPT_FCN(0, 0);
  _SFD_SCRIPT_CALL(0U, chartInstance->c2_sfEvent, 15);
  _SFD_SCRIPT_CALL(0U, chartInstance->c2_sfEvent, 16);
  _SFD_SCRIPT_CALL(0U, chartInstance->c2_sfEvent, 17);
  _SFD_SCRIPT_CALL(0U, chartInstance->c2_sfEvent, -17);
  _SFD_SYMBOL_SCOPE_POP();
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 74);
  for (c2_i175 = 0; c2_i175 < 27; c2_i175++) {
    c2_IR_Position[c2_i175] = c2_dv14[c2_i175];
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 85);
  c2_b_IR_Number = (int32_T)c2_IR_Number - 1;
  for (c2_i176 = 0; c2_i176 < 3; c2_i176++) {
    c2_Answer[c2_i176] = c2_IR_Position[c2_b_IR_Number + 9 * c2_i176];
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, -85);
  _SFD_SYMBOL_SCOPE_POP();
}

static void c2_Transformation_Matrix
  (SFc2_simiam_AvoidObstacle_BestAOInstanceStruct *chartInstance, real_T c2_x,
   real_T c2_y, real_T c2_theta, real_T c2_Answer[9])
{
  uint32_T c2_debug_family_var_map[6];
  real_T c2_nargin = 3.0;
  real_T c2_nargout = 1.0;
  uint32_T c2_b_debug_family_var_map[2];
  real_T c2_b_nargin = 1.0;
  real_T c2_b_nargout = 1.0;
  uint32_T c2_c_debug_family_var_map[3];
  char_T c2_type[15];
  real_T c2_c_nargin = 2.0;
  real_T c2_c_nargout = 0.0;
  int32_T c2_i177;
  static char_T c2_cv2[15] = { 'a', 'v', 'o', 'i', 'd', '_', 'o', 'b', 's', 't',
    'a', 'c', 'l', 'e', 's' };

  real_T c2_b_x;
  real_T c2_b_y;
  real_T c2_b_theta;
  real_T c2_d_nargin = 4.0;
  real_T c2_d_nargout = 1.0;
  real_T c2_c_x;
  real_T c2_d_x;
  real_T c2_e_x;
  real_T c2_f_x;
  real_T c2_g_x;
  real_T c2_h_x;
  real_T c2_i_x;
  real_T c2_j_x;
  int32_T c2_i178;
  int32_T c2_i179;
  static real_T c2_dv15[3] = { 0.0, 0.0, 1.0 };

  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 6U, 6U, c2_i_debug_family_names,
    c2_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_nargin, 0U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_nargout, 1U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_x, 2U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_y, 3U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_theta, 4U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_Answer, 5U, c2_g_sf_marshallOut,
    c2_f_sf_marshallIn);
  CV_EML_FCN(0, 1);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 43);
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 2U, 2U, c2_g_debug_family_names,
    c2_b_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_b_nargin, 0U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_b_nargout, 1U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  CV_SCRIPT_FCN(2, 0);
  _SFD_SCRIPT_CALL(2U, chartInstance->c2_sfEvent, 33);
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 3U, 3U, c2_f_debug_family_names,
    c2_c_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_type, 0U, c2_k_sf_marshallOut,
    c2_k_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_c_nargin, 1U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_c_nargout, 2U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  for (c2_i177 = 0; c2_i177 < 15; c2_i177++) {
    c2_type[c2_i177] = c2_cv2[c2_i177];
  }

  CV_SCRIPT_FCN(1, 0);
  _SFD_SCRIPT_CALL(1U, chartInstance->c2_sfEvent, 32);
  _SFD_SCRIPT_CALL(1U, chartInstance->c2_sfEvent, -32);
  _SFD_SYMBOL_SCOPE_POP();
  _SFD_SCRIPT_CALL(2U, chartInstance->c2_sfEvent, 34);
  _SFD_SCRIPT_CALL(2U, chartInstance->c2_sfEvent, 36);
  _SFD_SCRIPT_CALL(2U, chartInstance->c2_sfEvent, 37);
  _SFD_SCRIPT_CALL(2U, chartInstance->c2_sfEvent, 38);
  _SFD_SCRIPT_CALL(2U, chartInstance->c2_sfEvent, 40);
  _SFD_SCRIPT_CALL(2U, chartInstance->c2_sfEvent, 41);
  _SFD_SCRIPT_CALL(2U, chartInstance->c2_sfEvent, -41);
  _SFD_SYMBOL_SCOPE_POP();
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 44);
  c2_b_x = c2_x;
  c2_b_y = c2_y;
  c2_b_theta = c2_theta;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 6U, 6U, c2_h_debug_family_names,
    c2_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_d_nargin, 0U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_d_nargout, 1U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_b_x, 2U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_b_y, 3U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_b_theta, 4U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_Answer, 5U, c2_g_sf_marshallOut,
    c2_f_sf_marshallIn);
  CV_SCRIPT_FCN(2, 4);
  _SFD_SCRIPT_CALL(2U, chartInstance->c2_sfEvent, 142U);
  c2_c_x = c2_b_theta;
  c2_d_x = c2_c_x;
  c2_d_x = muDoubleScalarCos(c2_d_x);
  c2_e_x = c2_b_theta;
  c2_f_x = c2_e_x;
  c2_f_x = muDoubleScalarSin(c2_f_x);
  c2_g_x = c2_b_theta;
  c2_h_x = c2_g_x;
  c2_h_x = muDoubleScalarSin(c2_h_x);
  c2_i_x = c2_b_theta;
  c2_j_x = c2_i_x;
  c2_j_x = muDoubleScalarCos(c2_j_x);
  c2_Answer[0] = c2_d_x;
  c2_Answer[3] = -c2_f_x;
  c2_Answer[6] = c2_b_x;
  c2_Answer[1] = c2_h_x;
  c2_Answer[4] = c2_j_x;
  c2_Answer[7] = c2_b_y;
  c2_i178 = 0;
  for (c2_i179 = 0; c2_i179 < 3; c2_i179++) {
    c2_Answer[c2_i178 + 2] = c2_dv15[c2_i179];
    c2_i178 += 3;
  }

  _SFD_SCRIPT_CALL(2U, chartInstance->c2_sfEvent, -142);
  _SFD_SYMBOL_SCOPE_POP();
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, -44);
  _SFD_SYMBOL_SCOPE_POP();
}

static void c2_eml_scalar_eg(SFc2_simiam_AvoidObstacle_BestAOInstanceStruct
  *chartInstance)
{
}

static void c2_b_eml_scalar_eg(SFc2_simiam_AvoidObstacle_BestAOInstanceStruct
  *chartInstance)
{
}

static void c2_AvoidObstacles_NextPoint_BestAO
  (SFc2_simiam_AvoidObstacle_BestAOInstanceStruct *chartInstance, real_T
   c2_Obstacle_AroundRobot_World[27], real_T c2_Position[3], real_T
   c2_ObstacleDetected_data[9], int32_T c2_ObstacleDetected_sizes[1], real_T
   c2_NextPoint[2], real_T *c2_OutData)
{
  uint32_T c2_debug_family_var_map[27];
  real_T c2_sensor_gains[9];
  real_T c2_ir_distances_rf[18];
  real_T c2_u_i[18];
  real_T c2_u_i_Norm[9];
  real_T c2_u_i_Norm_Fixed[9];
  int32_T c2_u_i_sort_Number_sizes[2];
  real_T c2_u_i_sort_Number_data[9];
  real_T c2_u_i_Smallest[4];
  real_T c2_u_i_SecondSmall[4];
  real_T c2_P1[4];
  real_T c2_P2[4];
  real_T c2_Pointer_Wall[4];
  real_T c2_Printer_Robot[2];
  real_T c2_Robot_direction;
  real_T c2_NextAngle;
  real_T c2_ChickAngle;
  real_T c2_AngleList[27];
  real_T c2_ChickIndex;
  real_T c2_d_Chick;
  real_T c2_NextPoint_X;
  real_T c2_NextPoint_Y;
  real_T c2_b_u_i[36];
  int32_T c2_u_i_sizes[2];
  real_T c2_u_i_data[36];
  int32_T c2_u_i_Norm_Fixed_sizes[2];
  real_T c2_u_i_Norm_Fixed_data[9];
  real_T c2_b_Pointer_Wall[2];
  real_T c2_b_AngleList[9];
  real_T c2_nargin = 4.0;
  real_T c2_nargout = 2.0;
  int32_T c2_i180;
  int32_T c2_i181;
  int32_T c2_i182;
  int32_T c2_i183;
  int32_T c2_i184;
  real_T c2_b_Position[2];
  real_T c2_a[18];
  int32_T c2_i185;
  int32_T c2_i186;
  int32_T c2_i187;
  int32_T c2_i188;
  real_T c2_C[18];
  int32_T c2_i189;
  int32_T c2_i190;
  int32_T c2_i191;
  int32_T c2_i192;
  int32_T c2_i193;
  int32_T c2_i194;
  int32_T c2_i195;
  int32_T c2_i196;
  int32_T c2_i197;
  static real_T c2_b[81] = { 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  int32_T c2_i198;
  int32_T c2_i199;
  real_T c2_c_u_i[9];
  real_T c2_dv16[9];
  int32_T c2_i200;
  int32_T c2_i201;
  real_T c2_d_u_i[9];
  real_T c2_dv17[9];
  int32_T c2_i202;
  int32_T c2_i203;
  static real_T c2_dv18[9] = { 10.0, 0.0032, 0.0004, 0.0, 0.0, 0.0004, 0.0032,
    10.0, 10.0 };

  int32_T c2_i204;
  int32_T c2_i205;
  int32_T c2_i206;
  int32_T c2_i207;
  int32_T c2_i208;
  int32_T c2_i209;
  int32_T c2_i210;
  int32_T c2_i211;
  int32_T c2_loop_ub;
  int32_T c2_i212;
  int32_T c2_i213;
  int32_T c2_b_loop_ub;
  int32_T c2_i214;
  int32_T c2_x_sizes[2];
  int32_T c2_x;
  int32_T c2_b_x;
  int32_T c2_c_loop_ub;
  int32_T c2_i215;
  real_T c2_x_data[9];
  int32_T c2_b_x_sizes[2];
  int32_T c2_c_x;
  int32_T c2_d_x;
  int32_T c2_d_loop_ub;
  int32_T c2_i216;
  real_T c2_b_x_data[9];
  int32_T c2_iidx_sizes[2];
  int32_T c2_iidx_data[9];
  int32_T c2_e_x;
  int32_T c2_f_x;
  int32_T c2_e_loop_ub;
  int32_T c2_i217;
  int32_T c2_u_i_sort_Number;
  int32_T c2_b_u_i_sort_Number;
  int32_T c2_f_loop_ub;
  int32_T c2_i218;
  int32_T c2_c_u_i_sort_Number[1];
  int32_T c2_d_u_i_sort_Number;
  int32_T c2_i219;
  int32_T c2_e_u_i_sort_Number[1];
  int32_T c2_f_u_i_sort_Number;
  int32_T c2_i220;
  int32_T c2_i221;
  int32_T c2_i222;
  int32_T c2_i223;
  int32_T c2_i224;
  int32_T c2_i225;
  int32_T c2_i226;
  real_T c2_g_x;
  real_T c2_h_x;
  real_T c2_i_x;
  real_T c2_j_x;
  int32_T c2_i227;
  real_T c2_b_a[2];
  int32_T c2_i228;
  real_T c2_b_b[2];
  real_T c2_y;
  int32_T c2_k;
  int32_T c2_b_k;
  real_T c2_c_a;
  real_T c2_c_b;
  real_T c2_b_y;
  real_T c2_d_a;
  real_T c2_d_b;
  real_T c2_c_y;
  int32_T c2_i229;
  real_T c2_IR_Number[9];
  uint32_T c2_b_debug_family_var_map[5];
  real_T c2_IR_Position[27];
  real_T c2_b_nargin = 1.0;
  real_T c2_b_nargout = 1.0;
  real_T c2_k_x;
  real_T c2_d_y;
  real_T c2_theta;
  real_T c2_c_nargin = 4.0;
  real_T c2_c_nargout = 1.0;
  int32_T c2_i230;
  static real_T c2_dv19[27] = { -0.038, 0.017, 0.051, 0.067, 0.067, 0.051, 0.017,
    -0.038, -0.052, 0.049, 0.063, 0.045, 0.015, -0.015, -0.045, -0.063, -0.049,
    0.0, 2.2340214425527418, 1.3089969389957472, 0.73303828583761843,
    0.22689280275926285, -0.22689280275926285, -0.73303828583761843,
    -1.3089969389957472, -2.2340214425527418, 3.1415926535897931 };

  int32_T c2_i231;
  int32_T c2_i232;
  int32_T c2_i233;
  int32_T c2_i234;
  int32_T c2_i235;
  real_T c2_l_x[9];
  int32_T c2_c_k;
  real_T c2_d_k;
  real_T c2_m_x;
  real_T c2_e_y;
  real_T c2_f_y[9];
  int32_T c2_ixstart;
  real_T c2_mtmp;
  int32_T c2_itmp;
  real_T c2_n_x;
  boolean_T c2_e_b;
  int32_T c2_ix;
  int32_T c2_b_ix;
  real_T c2_o_x;
  boolean_T c2_f_b;
  int32_T c2_e_a;
  int32_T c2_i236;
  int32_T c2_c_ix;
  real_T c2_f_a;
  real_T c2_g_b;
  boolean_T c2_p;
  int32_T c2_b_itmp;
  int32_T c2_iindx;
  int32_T c2_b_iindx;
  real_T c2_indx;
  real_T c2_b_ChickIndex;
  real_T c2_p_x;
  real_T c2_q_x;
  real_T c2_g_a;
  real_T c2_h_b;
  real_T c2_g_y;
  real_T c2_r_x;
  real_T c2_s_x;
  real_T c2_h_a;
  real_T c2_i_b;
  real_T c2_h_y;
  boolean_T exitg1;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 27U, 32U, c2_k_debug_family_names,
    c2_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML(c2_sensor_gains, 0U, c2_l_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_ir_distances_rf, 1U,
    c2_q_sf_marshallOut, c2_q_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_u_i, MAX_uint32_T, c2_q_sf_marshallOut,
    c2_q_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_u_i_Norm, 3U, c2_l_sf_marshallOut,
    c2_l_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_u_i_Norm_Fixed, MAX_uint32_T,
    c2_l_sf_marshallOut, c2_l_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_DYN_IMPORTABLE(c2_u_i_sort_Number_data, (const
    int32_T *)&c2_u_i_sort_Number_sizes, NULL, 0, 5, (void *)c2_m_sf_marshallOut,
    (void *)c2_m_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_u_i_Smallest, 6U, c2_p_sf_marshallOut,
    c2_p_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_u_i_SecondSmall, 7U,
    c2_p_sf_marshallOut, c2_p_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_P1, 8U, c2_p_sf_marshallOut,
    c2_p_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_P2, 9U, c2_p_sf_marshallOut,
    c2_p_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_Pointer_Wall, MAX_uint32_T,
    c2_p_sf_marshallOut, c2_p_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_Printer_Robot, 11U,
    c2_b_sf_marshallOut, c2_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_Robot_direction, 12U,
    c2_sf_marshallOut, c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_NextAngle, 13U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_ChickAngle, 14U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_AngleList, MAX_uint32_T,
    c2_f_sf_marshallOut, c2_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_ChickIndex, 16U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_d_Chick, 17U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_NextPoint_X, 18U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_NextPoint_Y, 19U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_b_u_i, MAX_uint32_T,
    c2_o_sf_marshallOut, c2_o_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_DYN_IMPORTABLE(c2_u_i_data, (const int32_T *)
    &c2_u_i_sizes, NULL, 0, -1, (void *)c2_n_sf_marshallOut, (void *)
    c2_n_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_DYN_IMPORTABLE(c2_u_i_Norm_Fixed_data, (const
    int32_T *)&c2_u_i_Norm_Fixed_sizes, NULL, 0, -1, (void *)c2_m_sf_marshallOut,
    (void *)c2_m_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_b_Pointer_Wall, MAX_uint32_T,
    c2_b_sf_marshallOut, c2_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_b_AngleList, MAX_uint32_T,
    c2_l_sf_marshallOut, c2_l_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_nargin, 20U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_nargout, 21U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_Obstacle_AroundRobot_World, 22U,
    c2_f_sf_marshallOut, c2_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_Position, 23U, c2_c_sf_marshallOut,
    c2_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_DYN_IMPORTABLE(c2_ObstacleDetected_data, (const
    int32_T *)c2_ObstacleDetected_sizes, NULL, 1, 24, (void *)
    c2_h_sf_marshallOut, (void *)c2_g_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_NextPoint, 25U, c2_b_sf_marshallOut,
    c2_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_OutData, 26U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  CV_EML_FCN(0, 8);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 199U);
  for (c2_i180 = 0; c2_i180 < 9; c2_i180++) {
    c2_sensor_gains[c2_i180] = 1.0;
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 200U);
  c2_i181 = 0;
  c2_i182 = 0;
  for (c2_i183 = 0; c2_i183 < 9; c2_i183++) {
    for (c2_i184 = 0; c2_i184 < 2; c2_i184++) {
      c2_ir_distances_rf[c2_i184 + c2_i181] =
        c2_Obstacle_AroundRobot_World[c2_i184 + c2_i182];
    }

    c2_i181 += 2;
    c2_i182 += 3;
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 202U);
  c2_b_Position[0] = c2_Position[0];
  c2_b_Position[1] = c2_Position[1];
  c2_repmat(chartInstance, c2_b_Position, c2_a);
  for (c2_i185 = 0; c2_i185 < 18; c2_i185++) {
    c2_a[c2_i185] = c2_ir_distances_rf[c2_i185] - c2_a[c2_i185];
  }

  c2_c_eml_scalar_eg(chartInstance);
  c2_c_eml_scalar_eg(chartInstance);
  for (c2_i186 = 0; c2_i186 < 18; c2_i186++) {
    c2_u_i[c2_i186] = 0.0;
  }

  for (c2_i187 = 0; c2_i187 < 18; c2_i187++) {
    c2_u_i[c2_i187] = 0.0;
  }

  for (c2_i188 = 0; c2_i188 < 18; c2_i188++) {
    c2_C[c2_i188] = c2_u_i[c2_i188];
  }

  for (c2_i189 = 0; c2_i189 < 18; c2_i189++) {
    c2_u_i[c2_i189] = c2_C[c2_i189];
  }

  for (c2_i190 = 0; c2_i190 < 18; c2_i190++) {
    c2_C[c2_i190] = c2_u_i[c2_i190];
  }

  for (c2_i191 = 0; c2_i191 < 18; c2_i191++) {
    c2_u_i[c2_i191] = c2_C[c2_i191];
  }

  for (c2_i192 = 0; c2_i192 < 2; c2_i192++) {
    c2_i193 = 0;
    c2_i194 = 0;
    for (c2_i195 = 0; c2_i195 < 9; c2_i195++) {
      c2_u_i[c2_i193 + c2_i192] = 0.0;
      c2_i196 = 0;
      for (c2_i197 = 0; c2_i197 < 9; c2_i197++) {
        c2_u_i[c2_i193 + c2_i192] += c2_a[c2_i196 + c2_i192] * c2_b[c2_i197 +
          c2_i194];
        c2_i196 += 2;
      }

      c2_i193 += 2;
      c2_i194 += 9;
    }
  }

  _SFD_SYMBOL_SWITCH(2U, 2U);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 203U);
  c2_i198 = 0;
  for (c2_i199 = 0; c2_i199 < 9; c2_i199++) {
    c2_c_u_i[c2_i199] = c2_u_i[c2_i198];
    c2_i198 += 2;
  }

  c2_power(chartInstance, c2_c_u_i, c2_dv16);
  c2_i200 = 0;
  for (c2_i201 = 0; c2_i201 < 9; c2_i201++) {
    c2_d_u_i[c2_i201] = c2_u_i[c2_i200 + 1];
    c2_i200 += 2;
  }

  c2_power(chartInstance, c2_d_u_i, c2_dv17);
  for (c2_i202 = 0; c2_i202 < 9; c2_i202++) {
    c2_u_i_Norm[c2_i202] = c2_dv16[c2_i202] + c2_dv17[c2_i202];
  }

  c2_b_sqrt(chartInstance, c2_u_i_Norm);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 207U);
  for (c2_i203 = 0; c2_i203 < 9; c2_i203++) {
    c2_u_i_Norm_Fixed[c2_i203] = c2_u_i_Norm[c2_i203] + c2_dv18[c2_i203];
  }

  _SFD_SYMBOL_SWITCH(4U, 4U);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 208U);
  c2_i204 = 0;
  c2_i205 = 0;
  for (c2_i206 = 0; c2_i206 < 9; c2_i206++) {
    for (c2_i207 = 0; c2_i207 < 2; c2_i207++) {
      c2_b_u_i[c2_i207 + c2_i204] = c2_u_i[c2_i207 + c2_i205];
    }

    c2_i204 += 4;
    c2_i205 += 2;
  }

  c2_i208 = 0;
  for (c2_i209 = 0; c2_i209 < 9; c2_i209++) {
    c2_b_u_i[c2_i208 + 2] = c2_u_i_Norm[c2_i209];
    c2_i208 += 4;
  }

  c2_i210 = 0;
  for (c2_i211 = 0; c2_i211 < 9; c2_i211++) {
    c2_b_u_i[c2_i210 + 3] = 1.0 + (real_T)c2_i211;
    c2_i210 += 4;
  }

  _SFD_SYMBOL_SWITCH(2U, 20U);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 211U);
  c2_u_i_sizes[0] = 4;
  c2_u_i_sizes[1] = c2_ObstacleDetected_sizes[0];
  c2_loop_ub = c2_ObstacleDetected_sizes[0] - 1;
  for (c2_i212 = 0; c2_i212 <= c2_loop_ub; c2_i212++) {
    for (c2_i213 = 0; c2_i213 < 4; c2_i213++) {
      c2_u_i_data[c2_i213 + c2_u_i_sizes[0] * c2_i212] = c2_b_u_i[c2_i213 +
        (((int32_T)c2_ObstacleDetected_data[c2_i212] - 1) << 2)];
    }
  }

  _SFD_SYMBOL_SWITCH(2U, 21U);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 212U);
  c2_u_i_Norm_Fixed_sizes[0] = 1;
  c2_u_i_Norm_Fixed_sizes[1] = c2_ObstacleDetected_sizes[0];
  c2_b_loop_ub = c2_ObstacleDetected_sizes[0] - 1;
  for (c2_i214 = 0; c2_i214 <= c2_b_loop_ub; c2_i214++) {
    c2_u_i_Norm_Fixed_data[c2_u_i_Norm_Fixed_sizes[0] * c2_i214] =
      c2_u_i_Norm_Fixed[(int32_T)c2_ObstacleDetected_data[c2_i214] - 1];
  }

  _SFD_SYMBOL_SWITCH(4U, 22U);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 214U);
  c2_x_sizes[0] = 1;
  c2_x_sizes[1] = c2_u_i_Norm_Fixed_sizes[1];
  c2_x = c2_x_sizes[0];
  c2_b_x = c2_x_sizes[1];
  c2_c_loop_ub = c2_u_i_Norm_Fixed_sizes[0] * c2_u_i_Norm_Fixed_sizes[1] - 1;
  for (c2_i215 = 0; c2_i215 <= c2_c_loop_ub; c2_i215++) {
    c2_x_data[c2_i215] = c2_u_i_Norm_Fixed_data[c2_i215];
  }

  c2_b_x_sizes[0] = 1;
  c2_b_x_sizes[1] = c2_x_sizes[1];
  c2_c_x = c2_b_x_sizes[0];
  c2_d_x = c2_b_x_sizes[1];
  c2_d_loop_ub = c2_x_sizes[0] * c2_x_sizes[1] - 1;
  for (c2_i216 = 0; c2_i216 <= c2_d_loop_ub; c2_i216++) {
    c2_b_x_data[c2_i216] = c2_x_data[c2_i216];
  }

  c2_eml_sort(chartInstance, c2_b_x_data, c2_b_x_sizes, c2_x_data, c2_x_sizes,
              c2_iidx_data, c2_iidx_sizes);
  c2_x_sizes[0] = 1;
  c2_x_sizes[1] = c2_iidx_sizes[1];
  c2_e_x = c2_x_sizes[0];
  c2_f_x = c2_x_sizes[1];
  c2_e_loop_ub = c2_iidx_sizes[0] * c2_iidx_sizes[1] - 1;
  for (c2_i217 = 0; c2_i217 <= c2_e_loop_ub; c2_i217++) {
    c2_x_data[c2_i217] = (real_T)c2_iidx_data[c2_i217];
  }

  c2_u_i_sort_Number_sizes[0] = 1;
  c2_u_i_sort_Number_sizes[1] = c2_x_sizes[1];
  c2_u_i_sort_Number = c2_u_i_sort_Number_sizes[0];
  c2_b_u_i_sort_Number = c2_u_i_sort_Number_sizes[1];
  c2_f_loop_ub = c2_x_sizes[0] * c2_x_sizes[1] - 1;
  for (c2_i218 = 0; c2_i218 <= c2_f_loop_ub; c2_i218++) {
    c2_u_i_sort_Number_data[c2_i218] = c2_x_data[c2_i218];
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 214U);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 215U);
  (real_T)_SFD_EML_ARRAY_BOUNDS_CHECK("u_i_sort_Number", 1, 1,
    c2_u_i_sort_Number_sizes[1], 1, 0);
  c2_c_u_i_sort_Number[0] = c2_u_i_sort_Number_sizes[1];
  c2_d_u_i_sort_Number = (int32_T)(real_T)_SFD_EML_ARRAY_BOUNDS_CHECK("u_i",
    (int32_T)c2_u_i_sort_Number_data[0], 1, c2_u_i_sizes[1], 2, 0) - 1;
  for (c2_i219 = 0; c2_i219 < 4; c2_i219++) {
    c2_u_i_Smallest[c2_i219] = c2_u_i_data[c2_i219 + c2_u_i_sizes[0] *
      c2_d_u_i_sort_Number];
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 216U);
  (real_T)_SFD_EML_ARRAY_BOUNDS_CHECK("u_i_sort_Number", 2, 1,
    c2_u_i_sort_Number_sizes[1], 1, 0);
  c2_e_u_i_sort_Number[0] = c2_u_i_sort_Number_sizes[1];
  c2_f_u_i_sort_Number = (int32_T)(real_T)_SFD_EML_ARRAY_BOUNDS_CHECK("u_i",
    (int32_T)c2_u_i_sort_Number_data[1], 1, c2_u_i_sizes[1], 2, 0) - 1;
  for (c2_i220 = 0; c2_i220 < 4; c2_i220++) {
    c2_u_i_SecondSmall[c2_i220] = c2_u_i_data[c2_i220 + c2_u_i_sizes[0] *
      c2_f_u_i_sort_Number];
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 221U);
  if (CV_EML_IF(0, 1, 4, c2_u_i_Smallest[3] > c2_u_i_SecondSmall[3])) {
    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 222U);
    for (c2_i221 = 0; c2_i221 < 4; c2_i221++) {
      c2_P1[c2_i221] = c2_u_i_Smallest[c2_i221];
    }

    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 223U);
    for (c2_i222 = 0; c2_i222 < 4; c2_i222++) {
      c2_P2[c2_i222] = c2_u_i_SecondSmall[c2_i222];
    }
  } else {
    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 225U);
    for (c2_i223 = 0; c2_i223 < 4; c2_i223++) {
      c2_P2[c2_i223] = c2_u_i_Smallest[c2_i223];
    }

    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 226U);
    for (c2_i224 = 0; c2_i224 < 4; c2_i224++) {
      c2_P1[c2_i224] = c2_u_i_SecondSmall[c2_i224];
    }
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 229U);
  for (c2_i225 = 0; c2_i225 < 4; c2_i225++) {
    c2_Pointer_Wall[c2_i225] = c2_P2[c2_i225] - c2_P1[c2_i225];
  }

  _SFD_SYMBOL_SWITCH(10U, 10U);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 230U);
  for (c2_i226 = 0; c2_i226 < 2; c2_i226++) {
    c2_b_Pointer_Wall[c2_i226] = c2_Pointer_Wall[c2_i226];
  }

  _SFD_SYMBOL_SWITCH(10U, 23U);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 232U);
  c2_g_x = c2_Position[2];
  c2_h_x = c2_g_x;
  c2_h_x = muDoubleScalarCos(c2_h_x);
  c2_i_x = c2_Position[2];
  c2_j_x = c2_i_x;
  c2_j_x = muDoubleScalarSin(c2_j_x);
  c2_Printer_Robot[0] = c2_h_x;
  c2_Printer_Robot[1] = c2_j_x;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 234U);
  for (c2_i227 = 0; c2_i227 < 2; c2_i227++) {
    c2_b_a[c2_i227] = c2_b_Pointer_Wall[c2_i227];
  }

  for (c2_i228 = 0; c2_i228 < 2; c2_i228++) {
    c2_b_b[c2_i228] = c2_Printer_Robot[c2_i228];
  }

  c2_e_eml_scalar_eg(chartInstance);
  c2_e_eml_scalar_eg(chartInstance);
  c2_y = 0.0;
  for (c2_k = 1; c2_k < 3; c2_k++) {
    c2_b_k = c2_k - 1;
    c2_y += c2_b_a[c2_b_k] * c2_b_b[c2_b_k];
  }

  c2_Robot_direction = c2_y;
  c2_b_sign(chartInstance, &c2_Robot_direction);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 235U);
  if (CV_EML_IF(0, 1, 5, c2_Robot_direction == 0.0)) {
    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 236U);
    c2_Robot_direction = 1.0;
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 239U);
  c2_c_a = c2_Robot_direction;
  c2_c_b = c2_b_Pointer_Wall[1];
  c2_b_y = c2_c_a * c2_c_b;
  c2_d_a = c2_Robot_direction;
  c2_d_b = c2_b_Pointer_Wall[0];
  c2_c_y = c2_d_a * c2_d_b;
  c2_NextAngle = c2_atan2(chartInstance, c2_b_y, c2_c_y);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 245U);
  c2_ChickAngle = c2_abs(chartInstance, c2_NextAngle) - c2_abs(chartInstance,
    c2_Position[2]);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 246U);
  for (c2_i229 = 0; c2_i229 < 9; c2_i229++) {
    c2_IR_Number[c2_i229] = 1.0 + (real_T)c2_i229;
  }

  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 5U, 5U, c2_j_debug_family_names,
    c2_b_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_IR_Position, 0U, c2_j_sf_marshallOut,
    c2_j_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_b_nargin, 1U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_b_nargout, 2U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_IR_Number, 3U, c2_l_sf_marshallOut,
    c2_l_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_AngleList, 4U, c2_f_sf_marshallOut,
    c2_d_sf_marshallIn);
  CV_EML_FCN(0, 3);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 73);
  c2_k_x = 0.0;
  c2_d_y = 0.0;
  c2_theta = 0.0;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 5U, 5U, c2_c_debug_family_names,
    c2_b_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_c_nargin, 0U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_c_nargout, 1U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_k_x, 2U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_d_y, 3U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_theta, 4U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  CV_SCRIPT_FCN(0, 0);
  _SFD_SCRIPT_CALL(0U, chartInstance->c2_sfEvent, 15);
  _SFD_SCRIPT_CALL(0U, chartInstance->c2_sfEvent, 16);
  _SFD_SCRIPT_CALL(0U, chartInstance->c2_sfEvent, 17);
  _SFD_SCRIPT_CALL(0U, chartInstance->c2_sfEvent, -17);
  _SFD_SYMBOL_SCOPE_POP();
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 74);
  for (c2_i230 = 0; c2_i230 < 27; c2_i230++) {
    c2_IR_Position[c2_i230] = c2_dv19[c2_i230];
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 85);
  for (c2_i231 = 0; c2_i231 < 9; c2_i231++) {
    for (c2_i232 = 0; c2_i232 < 3; c2_i232++) {
      c2_AngleList[c2_i232 + 3 * c2_i231] = c2_IR_Position[((int32_T)
        c2_IR_Number[c2_i231] + 9 * c2_i232) - 1];
    }
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, -85);
  _SFD_SYMBOL_SCOPE_POP();
  _SFD_SYMBOL_SWITCH(15U, 15U);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 247U);
  c2_i233 = 0;
  for (c2_i234 = 0; c2_i234 < 9; c2_i234++) {
    c2_b_AngleList[c2_i234] = c2_AngleList[c2_i233 + 2];
    c2_i233 += 3;
  }

  _SFD_SYMBOL_SWITCH(15U, 24U);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 248U);
  for (c2_i235 = 0; c2_i235 < 9; c2_i235++) {
    c2_l_x[c2_i235] = c2_b_AngleList[c2_i235] - c2_ChickAngle;
  }

  for (c2_c_k = 0; c2_c_k < 9; c2_c_k++) {
    c2_d_k = 1.0 + (real_T)c2_c_k;
    c2_m_x = c2_l_x[(int32_T)c2_d_k - 1];
    c2_e_y = muDoubleScalarAbs(c2_m_x);
    c2_f_y[(int32_T)c2_d_k - 1] = c2_e_y;
  }

  c2_ixstart = 1;
  c2_mtmp = c2_f_y[0];
  c2_itmp = 1;
  c2_n_x = c2_mtmp;
  c2_e_b = muDoubleScalarIsNaN(c2_n_x);
  if (c2_e_b) {
    c2_ix = 2;
    exitg1 = FALSE;
    while ((exitg1 == FALSE) && (c2_ix < 10)) {
      c2_b_ix = c2_ix;
      c2_ixstart = c2_b_ix;
      c2_o_x = c2_f_y[c2_b_ix - 1];
      c2_f_b = muDoubleScalarIsNaN(c2_o_x);
      if (!c2_f_b) {
        c2_mtmp = c2_f_y[c2_b_ix - 1];
        c2_itmp = c2_b_ix;
        exitg1 = TRUE;
      } else {
        c2_ix++;
      }
    }
  }

  if (c2_ixstart < 9) {
    c2_e_a = c2_ixstart;
    c2_i236 = c2_e_a;
    for (c2_c_ix = c2_i236 + 1; c2_c_ix < 10; c2_c_ix++) {
      c2_b_ix = c2_c_ix - 1;
      c2_f_a = c2_f_y[c2_b_ix];
      c2_g_b = c2_mtmp;
      c2_p = (c2_f_a < c2_g_b);
      if (c2_p) {
        c2_mtmp = c2_f_y[c2_b_ix];
        c2_itmp = c2_b_ix + 1;
      }
    }
  }

  c2_b_itmp = c2_itmp;
  c2_iindx = c2_b_itmp;
  c2_b_iindx = c2_iindx;
  c2_indx = (real_T)c2_b_iindx;
  c2_b_ChickIndex = c2_indx;
  c2_ChickIndex = c2_b_ChickIndex;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 248U);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 249U);
  c2_d_Chick = c2_u_i_Norm[(int32_T)c2_ChickIndex - 1];
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 251U);
  c2_d_Chick -= 0.07;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 252U);
  if (CV_EML_IF(0, 1, 6, c2_d_Chick < 0.0)) {
    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 253U);
    c2_d_Chick = 0.001;
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 257);
  c2_p_x = c2_NextAngle;
  c2_q_x = c2_p_x;
  c2_q_x = muDoubleScalarCos(c2_q_x);
  c2_g_a = c2_d_Chick;
  c2_h_b = c2_q_x;
  c2_g_y = c2_g_a * c2_h_b;
  c2_NextPoint_X = c2_Position[0] + c2_g_y;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 258);
  c2_r_x = c2_NextAngle;
  c2_s_x = c2_r_x;
  c2_s_x = muDoubleScalarSin(c2_s_x);
  c2_h_a = c2_d_Chick;
  c2_i_b = c2_s_x;
  c2_h_y = c2_h_a * c2_i_b;
  c2_NextPoint_Y = c2_Position[1] + c2_h_y;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 259);
  c2_NextPoint[0] = c2_NextPoint_X;
  c2_NextPoint[1] = c2_NextPoint_Y;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 264);
  *c2_OutData = c2_NextAngle;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, -264);
  _SFD_SYMBOL_SCOPE_POP();
}

static void c2_repmat(SFc2_simiam_AvoidObstacle_BestAOInstanceStruct
                      *chartInstance, real_T c2_a[2], real_T c2_b[18])
{
  int32_T c2_ib;
  int32_T c2_jtilecol;
  int32_T c2_ia;
  int32_T c2_k;
  int32_T c2_b_a;
  int32_T c2_c_a;
  c2_ib = 1;
  for (c2_jtilecol = 1; c2_jtilecol < 10; c2_jtilecol++) {
    c2_ia = 1;
    for (c2_k = 1; c2_k < 3; c2_k++) {
      c2_b[_SFD_EML_ARRAY_BOUNDS_CHECK("", c2_ib, 1, 18, 1, 0) - 1] =
        c2_a[_SFD_EML_ARRAY_BOUNDS_CHECK("", c2_ia, 1, 2, 1, 0) - 1];
      c2_b_a = c2_ia + 1;
      c2_ia = c2_b_a;
      c2_c_a = c2_ib + 1;
      c2_ib = c2_c_a;
    }
  }
}

static void c2_c_eml_scalar_eg(SFc2_simiam_AvoidObstacle_BestAOInstanceStruct
  *chartInstance)
{
}

static void c2_power(SFc2_simiam_AvoidObstacle_BestAOInstanceStruct
                     *chartInstance, real_T c2_a[9], real_T c2_y[9])
{
  int32_T c2_k;
  real_T c2_b_k;
  real_T c2_ak;
  real_T c2_b_a;
  real_T c2_c_a;
  real_T c2_b;
  real_T c2_b_y;
  for (c2_k = 0; c2_k < 9; c2_k++) {
    c2_b_k = 1.0 + (real_T)c2_k;
    c2_ak = c2_a[(int32_T)c2_b_k - 1];
    c2_b_a = c2_ak;
    c2_d_eml_scalar_eg(chartInstance);
    c2_c_a = c2_b_a;
    c2_b = c2_b_a;
    c2_b_y = c2_c_a * c2_b;
    c2_y[(int32_T)c2_b_k - 1] = c2_b_y;
  }
}

static void c2_d_eml_scalar_eg(SFc2_simiam_AvoidObstacle_BestAOInstanceStruct
  *chartInstance)
{
}

static void c2_sqrt(SFc2_simiam_AvoidObstacle_BestAOInstanceStruct
                    *chartInstance, real_T c2_x[9], real_T c2_b_x[9])
{
  int32_T c2_i237;
  for (c2_i237 = 0; c2_i237 < 9; c2_i237++) {
    c2_b_x[c2_i237] = c2_x[c2_i237];
  }

  c2_b_sqrt(chartInstance, c2_b_x);
}

static void c2_b_eml_error(SFc2_simiam_AvoidObstacle_BestAOInstanceStruct
  *chartInstance)
{
  int32_T c2_i238;
  static char_T c2_cv3[30] = { 'C', 'o', 'd', 'e', 'r', ':', 't', 'o', 'o', 'l',
    'b', 'o', 'x', ':', 'E', 'l', 'F', 'u', 'n', 'D', 'o', 'm', 'a', 'i', 'n',
    'E', 'r', 'r', 'o', 'r' };

  char_T c2_u[30];
  const mxArray *c2_y = NULL;
  int32_T c2_i239;
  static char_T c2_cv4[4] = { 's', 'q', 'r', 't' };

  char_T c2_b_u[4];
  const mxArray *c2_b_y = NULL;
  for (c2_i238 = 0; c2_i238 < 30; c2_i238++) {
    c2_u[c2_i238] = c2_cv3[c2_i238];
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 10, 0U, 1U, 0U, 2, 1, 30), FALSE);
  for (c2_i239 = 0; c2_i239 < 4; c2_i239++) {
    c2_b_u[c2_i239] = c2_cv4[c2_i239];
  }

  c2_b_y = NULL;
  sf_mex_assign(&c2_b_y, sf_mex_create("y", c2_b_u, 10, 0U, 1U, 0U, 2, 1, 4),
                FALSE);
  sf_mex_call_debug("error", 0U, 1U, 14, sf_mex_call_debug("message", 1U, 2U, 14,
    c2_y, 14, c2_b_y));
}

static real_T c2_abs(SFc2_simiam_AvoidObstacle_BestAOInstanceStruct
                     *chartInstance, real_T c2_x)
{
  real_T c2_b_x;
  c2_b_x = c2_x;
  return muDoubleScalarAbs(c2_b_x);
}

static void c2_eml_sort(SFc2_simiam_AvoidObstacle_BestAOInstanceStruct
  *chartInstance, real_T c2_x_data[9], int32_T c2_x_sizes[2], real_T c2_y_data[9],
  int32_T c2_y_sizes[2], int32_T c2_idx_data[9], int32_T c2_idx_sizes[2])
{
  real_T c2_d1;
  real_T c2_vlen;
  real_T c2_dv20[2];
  int32_T c2_tmp_sizes;
  int32_T c2_loop_ub;
  int32_T c2_i240;
  real_T c2_tmp_data[9];
  int32_T c2_vwork_sizes;
  int32_T c2_i241;
  int32_T c2_i242;
  int32_T c2_b_tmp_sizes[2];
  int32_T c2_iv1[2];
  int32_T c2_i243;
  int32_T c2_i244;
  int32_T c2_b_loop_ub;
  int32_T c2_i245;
  int32_T c2_b_tmp_data[9];
  int32_T c2_i246;
  real_T c2_a;
  int32_T c2_c;
  int32_T c2_b_a;
  int32_T c2_vspread;
  int32_T c2_i2;
  int32_T c2_i;
  int32_T c2_i1;
  int32_T c2_c_a;
  int32_T c2_b;
  int32_T c2_j;
  int32_T c2_d_a;
  int32_T c2_e_a;
  int32_T c2_ix;
  real_T c2_b_vlen;
  int32_T c2_i247;
  int32_T c2_k;
  real_T c2_b_k;
  real_T c2_vwork_data[9];
  int32_T c2_f_a;
  int32_T c2_b_vwork_sizes;
  int32_T c2_c_loop_ub;
  int32_T c2_i248;
  real_T c2_b_vwork_data[9];
  int32_T c2_iidx_sizes;
  int32_T c2_iidx_data[9];
  real_T c2_c_vlen;
  int32_T c2_i249;
  int32_T c2_c_k;
  int32_T c2_g_a;
  c2_d1 = (real_T)c2_x_sizes[1];
  c2_vlen = c2_d1;
  c2_dv20[0] = c2_vlen;
  c2_dv20[1] = 1.0;
  c2_tmp_sizes = (int32_T)c2_dv20[0];
  c2_loop_ub = (int32_T)c2_dv20[0] - 1;
  for (c2_i240 = 0; c2_i240 <= c2_loop_ub; c2_i240++) {
    c2_tmp_data[c2_i240] = 0.0;
  }

  c2_vwork_sizes = c2_tmp_sizes;
  for (c2_i241 = 0; c2_i241 < 2; c2_i241++) {
    c2_y_sizes[c2_i241] = c2_x_sizes[c2_i241];
  }

  for (c2_i242 = 0; c2_i242 < 2; c2_i242++) {
    c2_dv20[c2_i242] = (real_T)c2_x_sizes[c2_i242];
  }

  c2_b_tmp_sizes[0] = 1;
  c2_iv1[0] = 1;
  c2_iv1[1] = (int32_T)c2_dv20[1];
  c2_b_tmp_sizes[1] = c2_iv1[1];
  c2_i243 = c2_b_tmp_sizes[0];
  c2_i244 = c2_b_tmp_sizes[1];
  c2_b_loop_ub = (int32_T)c2_dv20[1] - 1;
  for (c2_i245 = 0; c2_i245 <= c2_b_loop_ub; c2_i245++) {
    c2_b_tmp_data[c2_i245] = 0;
  }

  for (c2_i246 = 0; c2_i246 < 2; c2_i246++) {
    c2_idx_sizes[c2_i246] = c2_b_tmp_sizes[c2_i246];
  }

  c2_a = c2_vlen;
  c2_c = (int32_T)c2_a;
  c2_b_a = c2_c - 1;
  c2_vspread = c2_b_a;
  c2_i2 = 0;
  c2_i = 1;
  while (c2_i <= 1) {
    c2_i1 = c2_i2;
    c2_c_a = c2_i2;
    c2_b = c2_vspread;
    c2_i2 = c2_c_a + c2_b;
    c2_j = 1;
    while (c2_j <= 1) {
      c2_d_a = c2_i1 + 1;
      c2_i1 = c2_d_a;
      c2_e_a = c2_i2 + 1;
      c2_i2 = c2_e_a;
      c2_ix = c2_i1;
      c2_b_vlen = c2_vlen;
      c2_i247 = (int32_T)c2_b_vlen - 1;
      for (c2_k = 0; c2_k <= c2_i247; c2_k++) {
        c2_b_k = 1.0 + (real_T)c2_k;
        c2_vwork_data[(int32_T)(real_T)_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)
          c2_b_k, 1, c2_vwork_sizes, 1, 0) - 1] =
          c2_x_data[_SFD_EML_ARRAY_BOUNDS_CHECK("", c2_ix, 1, c2_x_sizes[1], 1,
          0) - 1];
        c2_f_a = c2_ix + 1;
        c2_ix = c2_f_a;
      }

      c2_b_vwork_sizes = c2_vwork_sizes;
      c2_c_loop_ub = c2_vwork_sizes - 1;
      for (c2_i248 = 0; c2_i248 <= c2_c_loop_ub; c2_i248++) {
        c2_b_vwork_data[c2_i248] = c2_vwork_data[c2_i248];
      }

      c2_eml_sort_idx(chartInstance, c2_b_vwork_data, *(int32_T (*)[1])&
                      c2_b_vwork_sizes, c2_iidx_data, *(int32_T (*)[1])&
                      c2_iidx_sizes);
      c2_ix = c2_i1;
      c2_c_vlen = c2_vlen;
      c2_i249 = (int32_T)c2_c_vlen - 1;
      for (c2_c_k = 0; c2_c_k <= c2_i249; c2_c_k++) {
        c2_b_k = 1.0 + (real_T)c2_c_k;
        c2_y_data[_SFD_EML_ARRAY_BOUNDS_CHECK("", c2_ix, 1, c2_y_sizes[1], 1, 0)
          - 1] = c2_vwork_data[_SFD_EML_ARRAY_BOUNDS_CHECK("", c2_iidx_data
          [(int32_T)(real_T)_SFD_EML_ARRAY_BOUNDS_CHECK("", (int32_T)c2_b_k, 1,
          c2_iidx_sizes, 1, 0) - 1], 1, c2_vwork_sizes, 1, 0) - 1];
        c2_idx_data[_SFD_EML_ARRAY_BOUNDS_CHECK("", c2_ix, 1, c2_idx_sizes[1], 1,
          0) - 1] = c2_iidx_data[(int32_T)(real_T)_SFD_EML_ARRAY_BOUNDS_CHECK("",
          (int32_T)c2_b_k, 1, c2_iidx_sizes, 1, 0) - 1];
        c2_g_a = c2_ix + 1;
        c2_ix = c2_g_a;
      }

      c2_j = 2;
    }

    c2_i = 2;
  }
}

static void c2_eml_sort_idx(SFc2_simiam_AvoidObstacle_BestAOInstanceStruct
  *chartInstance, real_T c2_x_data[9], int32_T c2_x_sizes[1], int32_T
  c2_idx_data[9], int32_T c2_idx_sizes[1])
{
  int32_T c2_n;
  real_T c2_dv21[2];
  int32_T c2_idx0_sizes;
  int32_T c2_loop_ub;
  int32_T c2_i250;
  int32_T c2_idx0_data[9];
  int32_T c2_a;
  int32_T c2_np1;
  int32_T c2_b_n;
  int32_T c2_k;
  int32_T c2_b_k;
  int32_T c2_c_n;
  int32_T c2_c_k;
  int32_T c2_b_a;
  int32_T c2_i251;
  int32_T c2_d_k;
  int32_T c2_c_a;
  int32_T c2_c;
  int32_T c2_irow1;
  int32_T c2_irow2;
  real_T c2_d_a;
  real_T c2_b;
  real_T c2_e_a;
  real_T c2_b_b;
  boolean_T c2_p;
  real_T c2_x;
  boolean_T c2_c_b;
  boolean_T c2_b3;
  boolean_T c2_b_p;
  int32_T c2_f_a;
  int32_T c2_b_c;
  int32_T c2_g_a;
  int32_T c2_c_c;
  int32_T c2_b_loop_ub;
  int32_T c2_i252;
  int32_T c2_i;
  int32_T c2_h_a;
  int32_T c2_i2;
  int32_T c2_j;
  int32_T c2_d_b;
  int32_T c2_pEnd;
  int32_T c2_c_p;
  int32_T c2_q;
  int32_T c2_i_a;
  int32_T c2_e_b;
  int32_T c2_qEnd;
  int32_T c2_j_a;
  int32_T c2_f_b;
  int32_T c2_kEnd;
  int32_T c2_b_irow1;
  int32_T c2_b_irow2;
  real_T c2_k_a;
  real_T c2_g_b;
  real_T c2_l_a;
  real_T c2_h_b;
  boolean_T c2_d_p;
  real_T c2_b_x;
  boolean_T c2_i_b;
  boolean_T c2_b4;
  boolean_T c2_e_p;
  int32_T c2_m_a;
  int32_T c2_n_a;
  int32_T c2_o_a;
  int32_T c2_p_a;
  int32_T c2_q_a;
  int32_T c2_r_a;
  int32_T c2_s_a;
  int32_T c2_t_a;
  int32_T c2_b_kEnd;
  int32_T c2_e_k;
  int32_T c2_u_a;
  int32_T c2_j_b;
  int32_T c2_d_c;
  int32_T c2_v_a;
  int32_T c2_k_b;
  boolean_T guard1 = FALSE;
  boolean_T guard2 = FALSE;
  c2_n = (int32_T)(real_T)c2_x_sizes[0];
  c2_dv21[0] = (real_T)c2_x_sizes[0];
  c2_dv21[1] = 1.0;
  c2_idx0_sizes = (int32_T)c2_dv21[0];
  c2_loop_ub = (int32_T)c2_dv21[0] - 1;
  for (c2_i250 = 0; c2_i250 <= c2_loop_ub; c2_i250++) {
    c2_idx0_data[c2_i250] = 0;
  }

  c2_idx_sizes[0] = c2_idx0_sizes;
  c2_a = c2_n + 1;
  c2_np1 = c2_a;
  if (c2_x_sizes[0] == 0) {
    c2_b_n = c2_n;
    for (c2_k = 1; c2_k <= c2_b_n; c2_k++) {
      c2_b_k = c2_k;
      c2_idx_data[_SFD_EML_ARRAY_BOUNDS_CHECK("", c2_b_k, 1, c2_idx_sizes[0], 1,
        0) - 1] = c2_b_k;
    }
  } else {
    c2_c_n = c2_n;
    for (c2_c_k = 1; c2_c_k <= c2_c_n; c2_c_k++) {
      c2_b_k = c2_c_k;
      c2_idx_data[_SFD_EML_ARRAY_BOUNDS_CHECK("", c2_b_k, 1, c2_idx_sizes[0], 1,
        0) - 1] = c2_b_k;
    }

    c2_b_a = c2_n;
    c2_i251 = c2_b_a;
    for (c2_d_k = 1; c2_d_k <= c2_i251 - 1; c2_d_k += 2) {
      c2_b_k = c2_d_k;
      c2_c_a = c2_b_k;
      c2_c = c2_c_a;
      c2_irow1 = c2_b_k;
      c2_irow2 = c2_c + 1;
      c2_d_a = c2_x_data[_SFD_EML_ARRAY_BOUNDS_CHECK("", c2_irow1, 1,
        c2_x_sizes[0], 1, 0) - 1];
      c2_b = c2_x_data[_SFD_EML_ARRAY_BOUNDS_CHECK("", c2_irow2, 1, c2_x_sizes[0],
        1, 0) - 1];
      c2_e_a = c2_d_a;
      c2_b_b = c2_b;
      c2_p = (c2_e_a <= c2_b_b);
      guard2 = FALSE;
      if (c2_p) {
        guard2 = TRUE;
      } else {
        c2_x = c2_b;
        c2_c_b = muDoubleScalarIsNaN(c2_x);
        if (c2_c_b) {
          guard2 = TRUE;
        } else {
          c2_b3 = FALSE;
        }
      }

      if (guard2 == TRUE) {
        c2_b3 = TRUE;
      }

      c2_b_p = c2_b3;
      if (c2_b_p) {
      } else {
        c2_f_a = c2_b_k;
        c2_b_c = c2_f_a;
        c2_idx_data[_SFD_EML_ARRAY_BOUNDS_CHECK("", c2_b_k, 1, c2_idx_sizes[0],
          1, 0) - 1] = c2_b_c + 1;
        c2_g_a = c2_b_k;
        c2_c_c = c2_g_a;
        c2_idx_data[_SFD_EML_ARRAY_BOUNDS_CHECK("", c2_c_c + 1, 1, c2_idx_sizes
          [0], 1, 0) - 1] = c2_b_k;
      }
    }

    c2_idx0_sizes = c2_n;
    c2_b_loop_ub = c2_n - 1;
    for (c2_i252 = 0; c2_i252 <= c2_b_loop_ub; c2_i252++) {
      c2_idx0_data[c2_i252] = 1;
    }

    c2_i = 2;
    while (c2_i < c2_n) {
      c2_h_a = c2_i;
      c2_i2 = c2_h_a << 1;
      c2_j = 1;
      c2_d_b = c2_i + 1;
      for (c2_pEnd = c2_d_b; c2_pEnd < c2_np1; c2_pEnd = c2_v_a + c2_k_b) {
        c2_c_p = c2_j;
        c2_q = c2_pEnd;
        c2_i_a = c2_j;
        c2_e_b = c2_i2;
        c2_qEnd = c2_i_a + c2_e_b;
        if (c2_qEnd > c2_np1) {
          c2_qEnd = c2_np1;
        }

        c2_b_k = 1;
        c2_j_a = c2_qEnd;
        c2_f_b = c2_j;
        c2_kEnd = c2_j_a - c2_f_b;
        while (c2_b_k <= c2_kEnd) {
          c2_b_irow1 = c2_idx_data[_SFD_EML_ARRAY_BOUNDS_CHECK("", c2_c_p, 1,
            c2_idx_sizes[0], 1, 0) - 1];
          c2_b_irow2 = c2_idx_data[_SFD_EML_ARRAY_BOUNDS_CHECK("", c2_q, 1,
            c2_idx_sizes[0], 1, 0) - 1];
          c2_k_a = c2_x_data[_SFD_EML_ARRAY_BOUNDS_CHECK("", c2_b_irow1, 1,
            c2_x_sizes[0], 1, 0) - 1];
          c2_g_b = c2_x_data[_SFD_EML_ARRAY_BOUNDS_CHECK("", c2_b_irow2, 1,
            c2_x_sizes[0], 1, 0) - 1];
          c2_l_a = c2_k_a;
          c2_h_b = c2_g_b;
          c2_d_p = (c2_l_a <= c2_h_b);
          guard1 = FALSE;
          if (c2_d_p) {
            guard1 = TRUE;
          } else {
            c2_b_x = c2_g_b;
            c2_i_b = muDoubleScalarIsNaN(c2_b_x);
            if (c2_i_b) {
              guard1 = TRUE;
            } else {
              c2_b4 = FALSE;
            }
          }

          if (guard1 == TRUE) {
            c2_b4 = TRUE;
          }

          c2_e_p = c2_b4;
          if (c2_e_p) {
            c2_idx0_data[_SFD_EML_ARRAY_BOUNDS_CHECK("", c2_b_k, 1,
              c2_idx0_sizes, 1, 0) - 1] =
              c2_idx_data[_SFD_EML_ARRAY_BOUNDS_CHECK("", c2_c_p, 1,
              c2_idx_sizes[0], 1, 0) - 1];
            c2_m_a = c2_c_p + 1;
            c2_c_p = c2_m_a;
            if (c2_c_p == c2_pEnd) {
              while (c2_q < c2_qEnd) {
                c2_n_a = c2_b_k + 1;
                c2_b_k = c2_n_a;
                c2_idx0_data[_SFD_EML_ARRAY_BOUNDS_CHECK("", c2_b_k, 1,
                  c2_idx0_sizes, 1, 0) - 1] =
                  c2_idx_data[_SFD_EML_ARRAY_BOUNDS_CHECK("", c2_q, 1,
                  c2_idx_sizes[0], 1, 0) - 1];
                c2_o_a = c2_q + 1;
                c2_q = c2_o_a;
              }
            }
          } else {
            c2_idx0_data[_SFD_EML_ARRAY_BOUNDS_CHECK("", c2_b_k, 1,
              c2_idx0_sizes, 1, 0) - 1] =
              c2_idx_data[_SFD_EML_ARRAY_BOUNDS_CHECK("", c2_q, 1, c2_idx_sizes
              [0], 1, 0) - 1];
            c2_p_a = c2_q + 1;
            c2_q = c2_p_a;
            if (c2_q == c2_qEnd) {
              while (c2_c_p < c2_pEnd) {
                c2_q_a = c2_b_k + 1;
                c2_b_k = c2_q_a;
                c2_idx0_data[_SFD_EML_ARRAY_BOUNDS_CHECK("", c2_b_k, 1,
                  c2_idx0_sizes, 1, 0) - 1] =
                  c2_idx_data[_SFD_EML_ARRAY_BOUNDS_CHECK("", c2_c_p, 1,
                  c2_idx_sizes[0], 1, 0) - 1];
                c2_r_a = c2_c_p + 1;
                c2_c_p = c2_r_a;
              }
            }
          }

          c2_s_a = c2_b_k + 1;
          c2_b_k = c2_s_a;
        }

        c2_t_a = c2_j;
        c2_c_p = c2_t_a;
        c2_b_kEnd = c2_kEnd;
        for (c2_e_k = 1; c2_e_k <= c2_b_kEnd; c2_e_k++) {
          c2_b_k = c2_e_k;
          c2_u_a = c2_c_p - 1;
          c2_j_b = c2_b_k;
          c2_d_c = c2_u_a + c2_j_b;
          c2_idx_data[_SFD_EML_ARRAY_BOUNDS_CHECK("", c2_d_c, 1, c2_idx_sizes[0],
            1, 0) - 1] = c2_idx0_data[_SFD_EML_ARRAY_BOUNDS_CHECK("", c2_b_k, 1,
            c2_idx0_sizes, 1, 0) - 1];
        }

        c2_j = c2_qEnd;
        c2_v_a = c2_j;
        c2_k_b = c2_i;
      }

      c2_i = c2_i2;
    }
  }
}

static void c2_e_eml_scalar_eg(SFc2_simiam_AvoidObstacle_BestAOInstanceStruct
  *chartInstance)
{
}

static real_T c2_sign(SFc2_simiam_AvoidObstacle_BestAOInstanceStruct
                      *chartInstance, real_T c2_x)
{
  real_T c2_b_x;
  c2_b_x = c2_x;
  c2_b_sign(chartInstance, &c2_b_x);
  return c2_b_x;
}

static real_T c2_atan2(SFc2_simiam_AvoidObstacle_BestAOInstanceStruct
  *chartInstance, real_T c2_y, real_T c2_x)
{
  real_T c2_b_y;
  real_T c2_b_x;
  c2_d_eml_scalar_eg(chartInstance);
  c2_b_y = c2_y;
  c2_b_x = c2_x;
  return muDoubleScalarAtan2(c2_b_y, c2_b_x);
}

static void c2_AirLine_NextPoint(SFc2_simiam_AvoidObstacle_BestAOInstanceStruct *
  chartInstance, real_T c2_Obstacle_AroundRobot_World[27], real_T c2_Position[3],
  real_T c2_NextPoint[2])
{
  uint32_T c2_debug_family_var_map[12];
  real_T c2_sensor_gains[9];
  real_T c2_ir_distances_rf[18];
  real_T c2_u_i[18];
  real_T c2_NextAngle;
  real_T c2_d_List;
  real_T c2_NextPoint_X;
  real_T c2_NextPoint_Y;
  real_T c2_nargin = 2.0;
  real_T c2_nargout = 1.0;
  int32_T c2_i253;
  int32_T c2_i254;
  int32_T c2_i255;
  int32_T c2_i256;
  int32_T c2_i257;
  real_T c2_b_Position[2];
  real_T c2_a[18];
  int32_T c2_i258;
  int32_T c2_i259;
  int32_T c2_i260;
  int32_T c2_i261;
  real_T c2_C[18];
  int32_T c2_i262;
  int32_T c2_i263;
  int32_T c2_i264;
  int32_T c2_i265;
  int32_T c2_i266;
  int32_T c2_i267;
  int32_T c2_i268;
  int32_T c2_i269;
  int32_T c2_i270;
  static real_T c2_b[81] = { 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  real_T c2_x;
  real_T c2_b_x;
  real_T c2_c_x;
  real_T c2_d_x;
  real_T c2_varargin_1;
  real_T c2_varargin_2;
  real_T c2_b_varargin_2;
  real_T c2_varargin_3;
  real_T c2_e_x;
  real_T c2_y;
  real_T c2_f_x;
  real_T c2_b_y;
  real_T c2_xk;
  real_T c2_yk;
  real_T c2_g_x;
  real_T c2_c_y;
  real_T c2_h_x;
  real_T c2_i_x;
  real_T c2_b_a;
  real_T c2_b_b;
  real_T c2_d_y;
  real_T c2_j_x;
  real_T c2_k_x;
  real_T c2_c_a;
  real_T c2_c_b;
  real_T c2_e_y;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 12U, 12U, c2_l_debug_family_names,
    c2_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML(c2_sensor_gains, 0U, c2_l_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_ir_distances_rf, 1U,
    c2_q_sf_marshallOut, c2_q_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_u_i, 2U, c2_q_sf_marshallOut,
    c2_q_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_NextAngle, 3U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_d_List, 4U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_NextPoint_X, 5U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_NextPoint_Y, 6U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_nargin, 7U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_nargout, 8U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_Obstacle_AroundRobot_World, 9U,
    c2_f_sf_marshallOut, c2_d_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_Position, 10U, c2_c_sf_marshallOut,
    c2_e_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_NextPoint, 11U, c2_b_sf_marshallOut,
    c2_b_sf_marshallIn);
  CV_EML_FCN(0, 7);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 174U);
  for (c2_i253 = 0; c2_i253 < 9; c2_i253++) {
    c2_sensor_gains[c2_i253] = 1.0;
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 177U);
  c2_i254 = 0;
  c2_i255 = 0;
  for (c2_i256 = 0; c2_i256 < 9; c2_i256++) {
    for (c2_i257 = 0; c2_i257 < 2; c2_i257++) {
      c2_ir_distances_rf[c2_i257 + c2_i254] =
        c2_Obstacle_AroundRobot_World[c2_i257 + c2_i255];
    }

    c2_i254 += 2;
    c2_i255 += 3;
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 178U);
  c2_b_Position[0] = c2_Position[0];
  c2_b_Position[1] = c2_Position[1];
  c2_repmat(chartInstance, c2_b_Position, c2_a);
  for (c2_i258 = 0; c2_i258 < 18; c2_i258++) {
    c2_a[c2_i258] = c2_ir_distances_rf[c2_i258] - c2_a[c2_i258];
  }

  c2_c_eml_scalar_eg(chartInstance);
  c2_c_eml_scalar_eg(chartInstance);
  for (c2_i259 = 0; c2_i259 < 18; c2_i259++) {
    c2_u_i[c2_i259] = 0.0;
  }

  for (c2_i260 = 0; c2_i260 < 18; c2_i260++) {
    c2_u_i[c2_i260] = 0.0;
  }

  for (c2_i261 = 0; c2_i261 < 18; c2_i261++) {
    c2_C[c2_i261] = c2_u_i[c2_i261];
  }

  for (c2_i262 = 0; c2_i262 < 18; c2_i262++) {
    c2_u_i[c2_i262] = c2_C[c2_i262];
  }

  for (c2_i263 = 0; c2_i263 < 18; c2_i263++) {
    c2_C[c2_i263] = c2_u_i[c2_i263];
  }

  for (c2_i264 = 0; c2_i264 < 18; c2_i264++) {
    c2_u_i[c2_i264] = c2_C[c2_i264];
  }

  for (c2_i265 = 0; c2_i265 < 2; c2_i265++) {
    c2_i266 = 0;
    c2_i267 = 0;
    for (c2_i268 = 0; c2_i268 < 9; c2_i268++) {
      c2_u_i[c2_i266 + c2_i265] = 0.0;
      c2_i269 = 0;
      for (c2_i270 = 0; c2_i270 < 9; c2_i270++) {
        c2_u_i[c2_i266 + c2_i265] += c2_a[c2_i269 + c2_i265] * c2_b[c2_i270 +
          c2_i267];
        c2_i269 += 2;
      }

      c2_i266 += 2;
      c2_i267 += 9;
    }
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 184U);
  c2_NextAngle = c2_Position[2];
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 185U);
  c2_x = c2_mpower(chartInstance, c2_u_i[6]) + c2_mpower(chartInstance, c2_u_i[7]);
  c2_b_x = c2_x;
  if (c2_b_x < 0.0) {
    c2_b_eml_error(chartInstance);
  }

  c2_b_x = muDoubleScalarSqrt(c2_b_x);
  c2_c_x = c2_mpower(chartInstance, c2_u_i[8]) + c2_mpower(chartInstance,
    c2_u_i[9]);
  c2_d_x = c2_c_x;
  if (c2_d_x < 0.0) {
    c2_b_eml_error(chartInstance);
  }

  c2_d_x = muDoubleScalarSqrt(c2_d_x);
  c2_varargin_1 = c2_b_x;
  c2_varargin_2 = c2_d_x;
  c2_b_varargin_2 = c2_varargin_1;
  c2_varargin_3 = c2_varargin_2;
  c2_e_x = c2_b_varargin_2;
  c2_y = c2_varargin_3;
  c2_f_x = c2_e_x;
  c2_b_y = c2_y;
  c2_d_eml_scalar_eg(chartInstance);
  c2_xk = c2_f_x;
  c2_yk = c2_b_y;
  c2_g_x = c2_xk;
  c2_c_y = c2_yk;
  c2_d_eml_scalar_eg(chartInstance);
  c2_d_List = muDoubleScalarMin(c2_g_x, c2_c_y);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 186U);
  c2_d_List -= 0.07;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 187U);
  if (CV_EML_IF(0, 1, 3, c2_d_List < 0.0)) {
    _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 188U);
    c2_d_List = 0.001;
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 192U);
  c2_h_x = c2_NextAngle;
  c2_i_x = c2_h_x;
  c2_i_x = muDoubleScalarCos(c2_i_x);
  c2_b_a = c2_d_List;
  c2_b_b = c2_i_x;
  c2_d_y = c2_b_a * c2_b_b;
  c2_NextPoint_X = c2_Position[0] + c2_d_y;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 193U);
  c2_j_x = c2_NextAngle;
  c2_k_x = c2_j_x;
  c2_k_x = muDoubleScalarSin(c2_k_x);
  c2_c_a = c2_d_List;
  c2_c_b = c2_k_x;
  c2_e_y = c2_c_a * c2_c_b;
  c2_NextPoint_Y = c2_Position[1] + c2_e_y;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 194U);
  c2_NextPoint[0] = c2_NextPoint_X;
  c2_NextPoint[1] = c2_NextPoint_Y;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, -194);
  _SFD_SYMBOL_SCOPE_POP();
}

static real_T c2_mpower(SFc2_simiam_AvoidObstacle_BestAOInstanceStruct
  *chartInstance, real_T c2_a)
{
  real_T c2_b_a;
  real_T c2_c_a;
  real_T c2_ak;
  real_T c2_d_a;
  real_T c2_e_a;
  real_T c2_b;
  c2_b_a = c2_a;
  c2_c_a = c2_b_a;
  c2_d_eml_scalar_eg(chartInstance);
  c2_ak = c2_c_a;
  c2_d_a = c2_ak;
  c2_d_eml_scalar_eg(chartInstance);
  c2_e_a = c2_d_a;
  c2_b = c2_d_a;
  return c2_e_a * c2_b;
}

static const mxArray *c2_r_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData = NULL;
  int32_T c2_u;
  const mxArray *c2_y = NULL;
  SFc2_simiam_AvoidObstacle_BestAOInstanceStruct *chartInstance;
  chartInstance = (SFc2_simiam_AvoidObstacle_BestAOInstanceStruct *)
    chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_u = *(int32_T *)c2_inData;
  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", &c2_u, 6, 0U, 0U, 0U, 0), FALSE);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, FALSE);
  return c2_mxArrayOutData;
}

static int32_T c2_t_emlrt_marshallIn
  (SFc2_simiam_AvoidObstacle_BestAOInstanceStruct *chartInstance, const mxArray *
   c2_u, const emlrtMsgIdentifier *c2_parentId)
{
  int32_T c2_y;
  int32_T c2_i271;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), &c2_i271, 1, 6, 0U, 0, 0U, 0);
  c2_y = c2_i271;
  sf_mex_destroy(&c2_u);
  return c2_y;
}

static void c2_r_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_b_sfEvent;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  int32_T c2_y;
  SFc2_simiam_AvoidObstacle_BestAOInstanceStruct *chartInstance;
  chartInstance = (SFc2_simiam_AvoidObstacle_BestAOInstanceStruct *)
    chartInstanceVoid;
  c2_b_sfEvent = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_y = c2_t_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_b_sfEvent),
    &c2_thisId);
  sf_mex_destroy(&c2_b_sfEvent);
  *(int32_T *)c2_outData = c2_y;
  sf_mex_destroy(&c2_mxArrayInData);
}

static uint8_T c2_u_emlrt_marshallIn
  (SFc2_simiam_AvoidObstacle_BestAOInstanceStruct *chartInstance, const mxArray *
   c2_b_is_active_c2_simiam_AvoidObstacle_BestAO, const char_T *c2_identifier)
{
  uint8_T c2_y;
  emlrtMsgIdentifier c2_thisId;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_y = c2_v_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c2_b_is_active_c2_simiam_AvoidObstacle_BestAO), &c2_thisId);
  sf_mex_destroy(&c2_b_is_active_c2_simiam_AvoidObstacle_BestAO);
  return c2_y;
}

static uint8_T c2_v_emlrt_marshallIn
  (SFc2_simiam_AvoidObstacle_BestAOInstanceStruct *chartInstance, const mxArray *
   c2_u, const emlrtMsgIdentifier *c2_parentId)
{
  uint8_T c2_y;
  uint8_T c2_u0;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), &c2_u0, 1, 3, 0U, 0, 0U, 0);
  c2_y = c2_u0;
  sf_mex_destroy(&c2_u);
  return c2_y;
}

static void c2_b_sqrt(SFc2_simiam_AvoidObstacle_BestAOInstanceStruct
                      *chartInstance, real_T c2_x[9])
{
  int32_T c2_k;
  real_T c2_b_k;
  int32_T c2_c_k;
  real_T c2_b_x;
  real_T c2_c_x;
  for (c2_k = 0; c2_k < 9; c2_k++) {
    c2_b_k = 1.0 + (real_T)c2_k;
    if (c2_x[(int32_T)c2_b_k - 1] < 0.0) {
      c2_b_eml_error(chartInstance);
    }
  }

  for (c2_c_k = 0; c2_c_k < 9; c2_c_k++) {
    c2_b_k = 1.0 + (real_T)c2_c_k;
    c2_b_x = c2_x[(int32_T)c2_b_k - 1];
    c2_c_x = c2_b_x;
    c2_c_x = muDoubleScalarSqrt(c2_c_x);
    c2_x[(int32_T)c2_b_k - 1] = c2_c_x;
  }
}

static void c2_b_sign(SFc2_simiam_AvoidObstacle_BestAOInstanceStruct
                      *chartInstance, real_T *c2_x)
{
  *c2_x = muDoubleScalarSign(*c2_x);
}

static void init_dsm_address_info(SFc2_simiam_AvoidObstacle_BestAOInstanceStruct
  *chartInstance)
{
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

void sf_c2_simiam_AvoidObstacle_BestAO_get_check_sum(mxArray *plhs[])
{
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(1070400699U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(3247406490U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(2070543389U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(2612359901U);
}

mxArray *sf_c2_simiam_AvoidObstacle_BestAO_get_autoinheritance_info(void)
{
  const char *autoinheritanceFields[] = { "checksum", "inputs", "parameters",
    "outputs", "locals" };

  mxArray *mxAutoinheritanceInfo = mxCreateStructMatrix(1,1,5,
    autoinheritanceFields);

  {
    mxArray *mxChecksum = mxCreateString("vBU4LUTgZNKcxlVyCat2dB");
    mxSetField(mxAutoinheritanceInfo,0,"checksum",mxChecksum);
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,3,3,dataFields);

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,2,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(9);
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
      pr[0] = (double)(3);
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
      pr[0] = (double)(1);
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
      pr[0] = (double)(2);
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

mxArray *sf_c2_simiam_AvoidObstacle_BestAO_third_party_uses_info(void)
{
  mxArray * mxcell3p = mxCreateCellMatrix(1,0);
  return(mxcell3p);
}

mxArray *sf_c2_simiam_AvoidObstacle_BestAO_updateBuildInfo_args_info(void)
{
  mxArray *mxBIArgs = mxCreateCellMatrix(1,0);
  return mxBIArgs;
}

static const mxArray *sf_get_sim_state_info_c2_simiam_AvoidObstacle_BestAO(void)
{
  const char *infoFields[] = { "chartChecksum", "varInfo" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 2, infoFields);
  const char *infoEncStr[] = {
    "100 S1x3'type','srcId','name','auxInfo'{{M[1],M[11],T\"NextPoint\",},{M[1],M[13],T\"OutData\",},{M[8],M[0],T\"is_active_c2_simiam_AvoidObstacle_BestAO\",}}"
  };

  mxArray *mxVarInfo = sf_mex_decode_encoded_mx_struct_array(infoEncStr, 3, 10);
  mxArray *mxChecksum = mxCreateDoubleMatrix(1, 4, mxREAL);
  sf_c2_simiam_AvoidObstacle_BestAO_get_check_sum(&mxChecksum);
  mxSetField(mxInfo, 0, infoFields[0], mxChecksum);
  mxSetField(mxInfo, 0, infoFields[1], mxVarInfo);
  return mxInfo;
}

static void chart_debug_initialization(SimStruct *S, unsigned int
  fullDebuggerInitialization)
{
  if (!sim_mode_is_rtw_gen(S)) {
    SFc2_simiam_AvoidObstacle_BestAOInstanceStruct *chartInstance;
    chartInstance = (SFc2_simiam_AvoidObstacle_BestAOInstanceStruct *)
      ((ChartInfoStruct *)(ssGetUserData(S)))->chartInstance;
    if (ssIsFirstInitCond(S) && fullDebuggerInitialization==1) {
      /* do this only if simulation is starting */
      {
        unsigned int chartAlreadyPresent;
        chartAlreadyPresent = sf_debug_initialize_chart
          (sfGlobalDebugInstanceStruct,
           _simiam_AvoidObstacle_BestAOMachineNumber_,
           2,
           1,
           1,
           5,
           0,
           0,
           0,
           0,
           3,
           &(chartInstance->chartNumber),
           &(chartInstance->instanceNumber),
           ssGetPath(S),
           (void *)S);
        if (chartAlreadyPresent==0) {
          /* this is the first instance */
          init_script_number_translation
            (_simiam_AvoidObstacle_BestAOMachineNumber_,
             chartInstance->chartNumber);
          sf_debug_set_chart_disable_implicit_casting
            (sfGlobalDebugInstanceStruct,
             _simiam_AvoidObstacle_BestAOMachineNumber_,
             chartInstance->chartNumber,1);
          sf_debug_set_chart_event_thresholds(sfGlobalDebugInstanceStruct,
            _simiam_AvoidObstacle_BestAOMachineNumber_,
            chartInstance->chartNumber,
            0,
            0,
            0);
          _SFD_SET_DATA_PROPS(0,1,1,0,"ir_raw");
          _SFD_SET_DATA_PROPS(1,1,1,0,"Position");
          _SFD_SET_DATA_PROPS(2,2,0,1,"NextPoint");
          _SFD_SET_DATA_PROPS(3,2,0,1,"OutData");
          _SFD_SET_DATA_PROPS(4,1,1,0,"FixAngle");
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
        _SFD_CV_INIT_EML(0,1,9,7,0,0,0,1,0,0,0);
        _SFD_CV_INIT_EML_FCN(0,0,"eML_blk_kernel",65,-1,1730);
        _SFD_CV_INIT_EML_FCN(0,1,"Transformation_Matrix",1752,-1,1961);
        _SFD_CV_INIT_EML_FCN(0,2,"IR_Raw_to_Distances",1963,-1,2639);
        _SFD_CV_INIT_EML_FCN(0,3,"GetIR_Position",2641,-1,3244);
        _SFD_CV_INIT_EML_FCN(0,4,"diff_to_uni",3246,-1,3378);
        _SFD_CV_INIT_EML_FCN(0,5,"AvoidObstacles_GeorgiaInstitute",3380,-1,4493);
        _SFD_CV_INIT_EML_FCN(0,6,"AvoidObstacles_NextPoint",4495,-1,5746);
        _SFD_CV_INIT_EML_FCN(0,7,"AirLine_NextPoint",5748,-1,6841);
        _SFD_CV_INIT_EML_FCN(0,8,"AvoidObstacles_NextPoint_BestAO",6945,-1,9653);
        _SFD_CV_INIT_EML_IF(0,1,0,1315,1347,1568,1725);
        _SFD_CV_INIT_EML_IF(0,1,1,3544,3564,-1,3604);
        _SFD_CV_INIT_EML_IF(0,1,2,5418,5432,-1,5543);
        _SFD_CV_INIT_EML_IF(0,1,3,6524,6538,-1,6649);
        _SFD_CV_INIT_EML_IF(0,1,4,8258,8302,8364,8433);
        _SFD_CV_INIT_EML_IF(0,1,5,8670,8694,-1,8731);
        _SFD_CV_INIT_EML_IF(0,1,6,9221,9236,-1,9356);
        _SFD_CV_INIT_EML_FOR(0,1,0,600,614,934);
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

        {
          unsigned int dimVector[1];
          dimVector[0]= 9;
          _SFD_SET_DATA_COMPILED_PROPS(0,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c2_d_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 3;
          _SFD_SET_DATA_COMPILED_PROPS(1,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c2_c_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 2;
          _SFD_SET_DATA_COMPILED_PROPS(2,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c2_b_sf_marshallOut,(MexInFcnForType)
            c2_b_sf_marshallIn);
        }

        _SFD_SET_DATA_COMPILED_PROPS(3,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c2_sf_marshallOut,(MexInFcnForType)c2_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(4,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c2_sf_marshallOut,(MexInFcnForType)NULL);

        {
          real_T *c2_OutData;
          real_T *c2_FixAngle;
          real_T (*c2_ir_raw)[9];
          real_T (*c2_Position)[3];
          real_T (*c2_NextPoint)[2];
          c2_FixAngle = (real_T *)ssGetInputPortSignal(chartInstance->S, 2);
          c2_OutData = (real_T *)ssGetOutputPortSignal(chartInstance->S, 2);
          c2_NextPoint = (real_T (*)[2])ssGetOutputPortSignal(chartInstance->S,
            1);
          c2_Position = (real_T (*)[3])ssGetInputPortSignal(chartInstance->S, 1);
          c2_ir_raw = (real_T (*)[9])ssGetInputPortSignal(chartInstance->S, 0);
          _SFD_SET_DATA_VALUE_PTR(0U, *c2_ir_raw);
          _SFD_SET_DATA_VALUE_PTR(1U, *c2_Position);
          _SFD_SET_DATA_VALUE_PTR(2U, *c2_NextPoint);
          _SFD_SET_DATA_VALUE_PTR(3U, c2_OutData);
          _SFD_SET_DATA_VALUE_PTR(4U, c2_FixAngle);
        }
      }
    } else {
      sf_debug_reset_current_state_configuration(sfGlobalDebugInstanceStruct,
        _simiam_AvoidObstacle_BestAOMachineNumber_,chartInstance->chartNumber,
        chartInstance->instanceNumber);
    }
  }
}

static const char* sf_get_instance_specialization(void)
{
  return "x9pKfdaAvva1Gvs3tOKvIF";
}

static void sf_opaque_initialize_c2_simiam_AvoidObstacle_BestAO(void
  *chartInstanceVar)
{
  chart_debug_initialization(((SFc2_simiam_AvoidObstacle_BestAOInstanceStruct*)
    chartInstanceVar)->S,0);
  initialize_params_c2_simiam_AvoidObstacle_BestAO
    ((SFc2_simiam_AvoidObstacle_BestAOInstanceStruct*) chartInstanceVar);
  initialize_c2_simiam_AvoidObstacle_BestAO
    ((SFc2_simiam_AvoidObstacle_BestAOInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_enable_c2_simiam_AvoidObstacle_BestAO(void
  *chartInstanceVar)
{
  enable_c2_simiam_AvoidObstacle_BestAO
    ((SFc2_simiam_AvoidObstacle_BestAOInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_disable_c2_simiam_AvoidObstacle_BestAO(void
  *chartInstanceVar)
{
  disable_c2_simiam_AvoidObstacle_BestAO
    ((SFc2_simiam_AvoidObstacle_BestAOInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_gateway_c2_simiam_AvoidObstacle_BestAO(void
  *chartInstanceVar)
{
  sf_c2_simiam_AvoidObstacle_BestAO
    ((SFc2_simiam_AvoidObstacle_BestAOInstanceStruct*) chartInstanceVar);
}

extern const mxArray* sf_internal_get_sim_state_c2_simiam_AvoidObstacle_BestAO
  (SimStruct* S)
{
  ChartInfoStruct *chartInfo = (ChartInfoStruct*) ssGetUserData(S);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_raw2high");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = (mxArray*) get_sim_state_c2_simiam_AvoidObstacle_BestAO
    ((SFc2_simiam_AvoidObstacle_BestAOInstanceStruct*)chartInfo->chartInstance);/* raw sim ctx */
  prhs[3] = (mxArray*) sf_get_sim_state_info_c2_simiam_AvoidObstacle_BestAO();/* state var info */
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

extern void sf_internal_set_sim_state_c2_simiam_AvoidObstacle_BestAO(SimStruct*
  S, const mxArray *st)
{
  ChartInfoStruct *chartInfo = (ChartInfoStruct*) ssGetUserData(S);
  mxArray *plhs[1] = { NULL };

  mxArray *prhs[4];
  int mxError = 0;
  prhs[0] = mxCreateString("chart_simctx_high2raw");
  prhs[1] = mxCreateDoubleScalar(ssGetSFuncBlockHandle(S));
  prhs[2] = mxDuplicateArray(st);      /* high level simctx */
  prhs[3] = (mxArray*) sf_get_sim_state_info_c2_simiam_AvoidObstacle_BestAO();/* state var info */
  mxError = sf_mex_call_matlab(1, plhs, 4, prhs, "sfprivate");
  mxDestroyArray(prhs[0]);
  mxDestroyArray(prhs[1]);
  mxDestroyArray(prhs[2]);
  mxDestroyArray(prhs[3]);
  if (mxError || plhs[0] == NULL) {
    sf_mex_error_message("Stateflow Internal Error: \nError calling 'chart_simctx_high2raw'.\n");
  }

  set_sim_state_c2_simiam_AvoidObstacle_BestAO
    ((SFc2_simiam_AvoidObstacle_BestAOInstanceStruct*)chartInfo->chartInstance,
     mxDuplicateArray(plhs[0]));
  mxDestroyArray(plhs[0]);
}

static const mxArray* sf_opaque_get_sim_state_c2_simiam_AvoidObstacle_BestAO
  (SimStruct* S)
{
  return sf_internal_get_sim_state_c2_simiam_AvoidObstacle_BestAO(S);
}

static void sf_opaque_set_sim_state_c2_simiam_AvoidObstacle_BestAO(SimStruct* S,
  const mxArray *st)
{
  sf_internal_set_sim_state_c2_simiam_AvoidObstacle_BestAO(S, st);
}

static void sf_opaque_terminate_c2_simiam_AvoidObstacle_BestAO(void
  *chartInstanceVar)
{
  if (chartInstanceVar!=NULL) {
    SimStruct *S = ((SFc2_simiam_AvoidObstacle_BestAOInstanceStruct*)
                    chartInstanceVar)->S;
    if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
      sf_clear_rtw_identifier(S);
      unload_simiam_AvoidObstacle_BestAO_optimization_info();
    }

    finalize_c2_simiam_AvoidObstacle_BestAO
      ((SFc2_simiam_AvoidObstacle_BestAOInstanceStruct*) chartInstanceVar);
    utFree((void *)chartInstanceVar);
    ssSetUserData(S,NULL);
  }
}

static void sf_opaque_init_subchart_simstructs(void *chartInstanceVar)
{
  initSimStructsc2_simiam_AvoidObstacle_BestAO
    ((SFc2_simiam_AvoidObstacle_BestAOInstanceStruct*) chartInstanceVar);
}

extern unsigned int sf_machine_global_initializer_called(void);
static void mdlProcessParameters_c2_simiam_AvoidObstacle_BestAO(SimStruct *S)
{
  int i;
  for (i=0;i<ssGetNumRunTimeParams(S);i++) {
    if (ssGetSFcnParamTunable(S,i)) {
      ssUpdateDlgParamAsRunTimeParam(S,i);
    }
  }

  if (sf_machine_global_initializer_called()) {
    initialize_params_c2_simiam_AvoidObstacle_BestAO
      ((SFc2_simiam_AvoidObstacle_BestAOInstanceStruct*)(((ChartInfoStruct *)
         ssGetUserData(S))->chartInstance));
  }
}

static void mdlSetWorkWidths_c2_simiam_AvoidObstacle_BestAO(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
    mxArray *infoStruct = load_simiam_AvoidObstacle_BestAO_optimization_info();
    int_T chartIsInlinable =
      (int_T)sf_is_chart_inlinable(S,sf_get_instance_specialization(),infoStruct,
      2);
    ssSetStateflowIsInlinable(S,chartIsInlinable);
    ssSetRTWCG(S,sf_rtw_info_uint_prop(S,sf_get_instance_specialization(),
                infoStruct,2,"RTWCG"));
    ssSetEnableFcnIsTrivial(S,1);
    ssSetDisableFcnIsTrivial(S,1);
    ssSetNotMultipleInlinable(S,sf_rtw_info_uint_prop(S,
      sf_get_instance_specialization(),infoStruct,2,
      "gatewayCannotBeInlinedMultipleTimes"));
    sf_update_buildInfo(S,sf_get_instance_specialization(),infoStruct,2);
    if (chartIsInlinable) {
      ssSetInputPortOptimOpts(S, 0, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 1, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 2, SS_REUSABLE_AND_LOCAL);
      sf_mark_chart_expressionable_inputs(S,sf_get_instance_specialization(),
        infoStruct,2,3);
      sf_mark_chart_reusable_outputs(S,sf_get_instance_specialization(),
        infoStruct,2,2);
    }

    {
      unsigned int outPortIdx;
      for (outPortIdx=1; outPortIdx<=2; ++outPortIdx) {
        ssSetOutputPortOptimizeInIR(S, outPortIdx, 1U);
      }
    }

    {
      unsigned int inPortIdx;
      for (inPortIdx=0; inPortIdx < 3; ++inPortIdx) {
        ssSetInputPortOptimizeInIR(S, inPortIdx, 1U);
      }
    }

    sf_set_rtw_dwork_info(S,sf_get_instance_specialization(),infoStruct,2);
    ssSetHasSubFunctions(S,!(chartIsInlinable));
  } else {
  }

  ssSetOptions(S,ssGetOptions(S)|SS_OPTION_WORKS_WITH_CODE_REUSE);
  ssSetChecksum0(S,(1112215502U));
  ssSetChecksum1(S,(3010738115U));
  ssSetChecksum2(S,(2532809172U));
  ssSetChecksum3(S,(3578578758U));
  ssSetmdlDerivatives(S, NULL);
  ssSetExplicitFCSSCtrl(S,1);
  ssSupportsMultipleExecInstances(S,1);
}

static void mdlRTW_c2_simiam_AvoidObstacle_BestAO(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S)) {
    ssWriteRTWStrParam(S, "StateflowChartType", "Embedded MATLAB");
  }
}

static void mdlStart_c2_simiam_AvoidObstacle_BestAO(SimStruct *S)
{
  SFc2_simiam_AvoidObstacle_BestAOInstanceStruct *chartInstance;
  chartInstance = (SFc2_simiam_AvoidObstacle_BestAOInstanceStruct *)utMalloc
    (sizeof(SFc2_simiam_AvoidObstacle_BestAOInstanceStruct));
  memset(chartInstance, 0, sizeof(SFc2_simiam_AvoidObstacle_BestAOInstanceStruct));
  if (chartInstance==NULL) {
    sf_mex_error_message("Could not allocate memory for chart instance.");
  }

  chartInstance->chartInfo.chartInstance = chartInstance;
  chartInstance->chartInfo.isEMLChart = 1;
  chartInstance->chartInfo.chartInitialized = 0;
  chartInstance->chartInfo.sFunctionGateway =
    sf_opaque_gateway_c2_simiam_AvoidObstacle_BestAO;
  chartInstance->chartInfo.initializeChart =
    sf_opaque_initialize_c2_simiam_AvoidObstacle_BestAO;
  chartInstance->chartInfo.terminateChart =
    sf_opaque_terminate_c2_simiam_AvoidObstacle_BestAO;
  chartInstance->chartInfo.enableChart =
    sf_opaque_enable_c2_simiam_AvoidObstacle_BestAO;
  chartInstance->chartInfo.disableChart =
    sf_opaque_disable_c2_simiam_AvoidObstacle_BestAO;
  chartInstance->chartInfo.getSimState =
    sf_opaque_get_sim_state_c2_simiam_AvoidObstacle_BestAO;
  chartInstance->chartInfo.setSimState =
    sf_opaque_set_sim_state_c2_simiam_AvoidObstacle_BestAO;
  chartInstance->chartInfo.getSimStateInfo =
    sf_get_sim_state_info_c2_simiam_AvoidObstacle_BestAO;
  chartInstance->chartInfo.zeroCrossings = NULL;
  chartInstance->chartInfo.outputs = NULL;
  chartInstance->chartInfo.derivatives = NULL;
  chartInstance->chartInfo.mdlRTW = mdlRTW_c2_simiam_AvoidObstacle_BestAO;
  chartInstance->chartInfo.mdlStart = mdlStart_c2_simiam_AvoidObstacle_BestAO;
  chartInstance->chartInfo.mdlSetWorkWidths =
    mdlSetWorkWidths_c2_simiam_AvoidObstacle_BestAO;
  chartInstance->chartInfo.extModeExec = NULL;
  chartInstance->chartInfo.restoreLastMajorStepConfiguration = NULL;
  chartInstance->chartInfo.restoreBeforeLastMajorStepConfiguration = NULL;
  chartInstance->chartInfo.storeCurrentConfiguration = NULL;
  chartInstance->S = S;
  ssSetUserData(S,(void *)(&(chartInstance->chartInfo)));/* register the chart instance with simstruct */
  init_dsm_address_info(chartInstance);
  if (!sim_mode_is_rtw_gen(S)) {
  }

  sf_opaque_init_subchart_simstructs(chartInstance->chartInfo.chartInstance);
  chart_debug_initialization(S,1);
}

void c2_simiam_AvoidObstacle_BestAO_method_dispatcher(SimStruct *S, int_T method,
  void *data)
{
  switch (method) {
   case SS_CALL_MDL_START:
    mdlStart_c2_simiam_AvoidObstacle_BestAO(S);
    break;

   case SS_CALL_MDL_SET_WORK_WIDTHS:
    mdlSetWorkWidths_c2_simiam_AvoidObstacle_BestAO(S);
    break;

   case SS_CALL_MDL_PROCESS_PARAMETERS:
    mdlProcessParameters_c2_simiam_AvoidObstacle_BestAO(S);
    break;

   default:
    /* Unhandled method */
    sf_mex_error_message("Stateflow Internal Error:\n"
                         "Error calling c2_simiam_AvoidObstacle_BestAO_method_dispatcher.\n"
                         "Can't handle method %d.\n", method);
    break;
  }
}
