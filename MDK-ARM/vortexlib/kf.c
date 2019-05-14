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

  mat_init(&kf.V, 2, 2, (float *)kf_init.V_data);
  mat_init(&kf.VT, 2, 2, (float *)kf_init.VT_data);
  mat_init(&kf.M, 2, 2, (float *)kf_init.M_data);
  mat_init(&kf.VM, 2, 2, (float *)kf_init.VM_data);
  mat_init(&kf.VMVT, 2, 2, (float *)kf_init.VMVT_data);
  mat_init(&kf.Q, 2, 2, (float *)kf_init.Q_data);

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