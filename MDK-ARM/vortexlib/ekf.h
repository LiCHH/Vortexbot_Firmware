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

  mat z;

  mat G;
  mat GT;
  mat Gsigma;
  mat GsigmaGT;
  mat V;
  mat VT;
  mat M;
  mat VM;
  mat VMVT;
  mat Q;
  mat C;
  mat CT;
  mat sigmaCT;
  mat CsigmaCT;
  mat CsigmaCT_Q;
  mat invCsigmaCT_Q;
  mat Cmu;
  mat z_Cmu;
  mat K_z_Cmu;
  mat KC;
  mat I;
  mat I_KC;
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

  float z_data[2];

  float G_data[9];                
  float GT_data[9];
  float Gsigma_data[9];
  float GsigmaGT_data[9];
  float V_data[9];
  float VT_data[9];
  float M_data[9];
  float VM_data[9];
  float VMVT_data[9];
  float Q_data[4];
  float C_data[6];
  float CT_data[6];
  float sigmaCT_data[6];
  float CsigmaCT_data[4];
  float CsigmaCT_Q_data[4];
  float invCsigmaCT_Q_data[4];
  float Cmu_data[2];
  float z_Cmu_data[2];
  float K_z_Cmu_data[3]; // 3x1
  float KC_data[9]; // 3x3
  float I_data[9];
  float I_KC_data[9];
  float K_data[6]; // 3x2

} ekf_init_t;

void ekf_init(void);

#endif // !1