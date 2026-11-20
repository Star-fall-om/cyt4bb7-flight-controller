#ifndef VOFA_TUNE_H
#define VOFA_TUNE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define VOFA_TUNE_TARGET_PA   (1u)
#define VOFA_TUNE_TARGET_PG   (2u)
#define VOFA_TUNE_TARGET_RA   (3u)
#define VOFA_TUNE_TARGET_RG   (4u)
#define VOFA_TUNE_TARGET_YA   (5u)
#define VOFA_TUNE_TARGET_YG   (6u)
#define VOFA_TUNE_TARGET_HP   (7u)
#define VOFA_TUNE_TARGET_HV   (8u)

#define VOFA_TUNE_GUARD_OK      (0u)
#define VOFA_TUNE_GUARD_ANGLE   (1u)
#define VOFA_TUNE_GUARD_LOCK    (2u)
#define VOFA_TUNE_GUARD_TARGET  (3u)
#define VOFA_TUNE_GUARD_CMD     (4u)

void VofaTune_Process10ms(void);
void VofaTune_UpdateMotorOutputs(float m1, float m2, float m3, float m4);
void VofaTune_AbortAndRestore(void);

uint8_t VofaTune_IsActiveTarget(uint8_t target_id);
uint8_t VofaTune_GetActiveCommand(uint8_t target_id, float *cmd);
uint8_t VofaTune_GetYawTarget(float *target_deg);
uint8_t VofaTune_GetHeightTarget(float *target_mm);

#ifdef __cplusplus
}
#endif

#endif
