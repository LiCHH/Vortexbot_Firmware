#ifndef __KF_H__ 
#define __KF_H__

#include "stm32f4xx_hal.h"
#include "arm_math.h"

#define mat         arm_matrix_instance_f32 
#define mat_64      arm_matrix_instance_f64
#define mat_init    arm_mat_init_f32
#define mat_add     arm_mat_add_f32
#define mat_sub     arm_mat_sub_f32
#define mat_mult    arm_mat_mult_f32
#define mat_trans   arm_mat_trans_f32
#define mat_inv     arm_mat_inverse_f32
#define mat_inv_f64 arm_mat_inverse_f64

#define THETA_VAR 0.01
#define ROT_VAR   0.01
#define TRAN_VAR  0.01

#define ZX_VAR    0.1
#define ZY_VAR    0.1

typedef struct 
{
  mat mu_last;
  mat mu_hat;
  mat mu_curr;

  mat sigma_last;
  mat sigma_hat;
  mat sigma_curr;

  mat z;

  mat A;
  mat AT;
  mat V;
  mat VT;
  mat M;
  mat VM;
  mat VMVT;
  mat Q;
  mat C;
  mat CT;
  mat z_Cmu;
  mat K_z_Cmu;
  mat I;
  mat I_KC;
  mat K;

} kf_t;

typedef struct 
{
  float mu_last_data[2];
  float mu_hat_data[2];
  float mu_curr_data[2];

  float sigma_last_data[4];
  float sigma_hat_data[4];
  float sigma_curr_data[4];

  float z_data[2];

  float A_data[4];                
  float AT_data[4];
  float V_data[4];
  float VT_data[4];
  float M_data[4];
  float VM_data[4];
  float VMVT_data[4];
  float Q_data[4];
  float C_data[4];
  float CT_data[4];
  float z_Cmu_data[2];
  float K_z_Cmu_data[2]; // 2x1
  float KC_data[4]; // 2x2
  float I_data[4];
  float I_KC_data[4];
  float K_data[4]; // 2x2

} kf_init_t;

void kfInit(void);

#endif // !1