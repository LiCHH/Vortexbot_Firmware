#include "kf.h"
#include "string.h"

kf_init_t kf_init;
kf_t kf;

void kfInit(void)
{
  memset(&kf_init, 0, sizeof(kf_init));

  mat_init(&kf.mu_last, 2, 1, (float *)kf_init.mu_last_data);
  mat_init(&kf.mu_hat, 2, 1, (float *)kf_init.mu_hat_data);
  mat_init(&kf.mu_curr, 2, 1, (float *)kf_init.mu_curr_data);

  mat_init(&kf.sigma_last, 2, 2, (float *)kf_init.sigma_last_data);
  mat_init(&kf.sigma_hat, 2, 2, (float *)kf_init.sigma_hat_data);
  mat_init(&kf.sigma_curr, 2, 2, (float *)kf_init.sigma_curr_data);

  mat_init(&kf.z, 2, 1, (float *)kf_init.z_data);

  mat_init(&kf.A, 2, 2, (float *)kf_init.A_data);
  kf.A.pData[0] = 1;
  kf.A.pData[3] = 1;
  mat_init(&kf.AT, 2, 2, (float *)kf_init.AT_data);
  mat_trans(&kf.A, &kf.AT);

  mat_init(&kf.V, 2, 3, (float *)kf_init.V_data);
  mat_init(&kf.VT, 3, 2, (float *)kf_init.VT_data);
  mat_init(&kf.M, 3, 3, (float *)kf_init.M_data);
  kf.M.pData[0] = THETA_VAR;
  kf.M.pData[4] = ROT_VAR;
  kf.M.pData[8] = TRAN_VAR;
  mat_init(&kf.VM, 2, 3, (float *)kf_init.VM_data);
  mat_init(&kf.VMVT, 2, 2, (float *)kf_init.VMVT_data);

  mat_init(&kf.Q, 2, 2, (float *)kf_init.Q_data);
  kf.Q.pData[0] = ZX_VAR;
  kf.Q.pData[3] = ZY_VAR;
  mat_init(&kf.CsigmaCT_Q, 2, 2, (float *)kf_init.CsigmaCT_Q_data);
  mat_init(&kf.invCsigmaCT_Q, 2, 2, (float *)kf_init.invCsigmaCT_Q_data);

  mat_init(&kf.C, 2, 2, (float *)kf_init.C_data);
  kf.C.pData[0] = 1;
  kf.C.pData[3] = 1;
  mat_init(&kf.CT, 2, 2, (float *)kf_init.CT_data);
  mat_trans(&kf.C, &kf.CT);

  mat_init(&kf.z_Cmu, 2, 2, (float *)kf_init.z_Cmu_data);
  mat_init(&kf.K_z_Cmu, 2, 2, (float *)kf_init.K_z_Cmu_data);
  mat_init(&kf.I_KC, 2, 2, (float *)kf_init.I_KC_data);
  mat_init(&kf.K, 2, 2, (float *)kf_init.K_data);

  mat_init(&kf.I, 2, 2, (float *)kf_init.I_data);
  kf.I.pData[0] = 1;
  kf.I.pData[3] = 1;

}

void kfPredict(float theta, float rot, float trans)
{
  kf.mu_last.pData[0] = kf.mu_curr.pData[0];
  kf.mu_last.pData[1] = kf.mu_curr.pData[1];
  kf.sigma_last.pData[0] = kf.sigma_curr.pData[0];
  kf.sigma_last.pData[1] = kf.sigma_curr.pData[1];
  kf.sigma_last.pData[2] = kf.sigma_curr.pData[2];
  kf.sigma_last.pData[3] = kf.sigma_curr.pData[3];

  float sin_data = arm_sin_f32(theta + rot);
  float cos_data = arm_cos_f32(theta + rot);

  kf.V.pData[0] = -trans * sin_data;
  kf.V.pData[1] = -trans * sin_data;
  kf.V.pData[2] = cos_data;
  kf.V.pData[3] = trans * cos_data;
  kf.V.pData[4] = trans * cos_data;
  kf.V.pData[5] = sin_data;

  mat_trans(&kf.V, &kf.VT);
  mat_mult(&kf.V, &kf.M, &kf.VM);
  mat_mult(&kf.VM, &kf.VT, &kf.VMVT);

  kf.mu_hat.pData[0] = kf.mu_last.pData[0] + trans * cos_data;
  kf.mu_hat.pData[1] = kf.mu_last.pData[1] + trans * sin_data;
  mat_add(&kf.sigma_last, &kf.VMVT, &kf.sigma_hat);

  //! prepare data for next movement without measurement
  //! does not matter if there exists a measurement update after this
  kf.mu_curr.pData[0] = kf.mu_hat.pData[0];
  kf.mu_curr.pData[1] = kf.mu_hat.pData[1];
  kf.sigma_curr.pData[0] = kf.sigma_hat.pData[0];
  kf.sigma_curr.pData[1] = kf.sigma_hat.pData[1];
  kf.sigma_curr.pData[2] = kf.sigma_hat.pData[2];
  kf.sigma_curr.pData[3] = kf.sigma_hat.pData[3];
}

void kfMeasure(float zx, float zy)
{
  kf.z.pData[0] = zx;
  kf.z.pData[1] = zy;
  
  //! prepare matrices
  mat_add(&kf.sigma_hat, &kf.Q, &kf.CsigmaCT_Q);
  mat_inv(&kf.CsigmaCT_Q, &kf.invCsigmaCT_Q);
  mat_mult(&kf.sigma_hat, &kf.invCsigmaCT_Q, &kf.K);
  mat_sub(&kf.z, &kf.mu_hat, &kf.z_Cmu);
  mat_mult(&kf.K, &kf.z_Cmu, &kf.K_z_Cmu);
  mat_sub(&kf.I, &kf.K, &kf.I_KC);

  //! get result
  mat_add(&kf.mu_hat, &kf.K_z_Cmu, &kf.mu_curr);
  mat_mult(&kf.I_KC, &kf.sigma_hat, &kf.sigma_curr);
}