#ifndef __EKF_H__ 
#define __EKF_H__

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

typedef struct 
{
  mat mu_last;
  mat mu_hat;
  mat mu_curr;

  mat sigma_last;
  mat sigma_hat;
  mat sigma_curr;

  mat G;
  mat GT;
  mat Gsigma_last;
  mat Gsigma_lastGT;
  mat V;
  mat VT;
  mat M;
  mat VM;
  mat VMVT;
  mat Q;
  mat C;
  mat CT;
  mat K;

} ekf_t;

typedef struct 
{
  float mu_last_data[3];
  float mu_hat_data[3];
  float mu_curr_data[3];

  float sigma_last_data[9];
  float sigma_hat_data[9];
  float sigma_curr_data[9];

  float G_curr_data[9];
  float V_curr_data[9];
  float M_curr_data[9];
  float Q_curr_data[9];
  float C_curr_data[6];
  float K_curr_data[6];

} ekf_init_t;

void ekf_init(void);

#endif // !1