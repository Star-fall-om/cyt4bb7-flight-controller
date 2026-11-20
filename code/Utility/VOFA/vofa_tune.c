#include "Utility/VOFA/vofa_tune.h"

#include "Algorithm/PID/pid.h"
#include "Motor/motor.h"
#include "System/uart_irq_compat.h"
#include "System/usr_system.h"
#include "System/usr_uart.h"
#include "zf_common_headfile.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define VOFA_RX_BATCH_LEN       (64u)
#define VOFA_HW_RX_POLL_LEN     (16u)
#define VOFA_CMD_MAX_LEN        (32u)
#define VOFA_TX_DIV_20HZ        (5u)
#define VOFA_PARAM_MAX          (10000.0f)

#define VOFA_CMD_ID_PA_P        (1u)
#define VOFA_CMD_ID_PA_I        (2u)
#define VOFA_CMD_ID_PA_D        (3u)
#define VOFA_CMD_ID_PG_P        (4u)
#define VOFA_CMD_ID_PG_I        (5u)
#define VOFA_CMD_ID_PG_D        (6u)
#define VOFA_CMD_ID_RA_P        (7u)
#define VOFA_CMD_ID_RA_I        (8u)
#define VOFA_CMD_ID_RA_D        (9u)
#define VOFA_CMD_ID_RG_P        (10u)
#define VOFA_CMD_ID_RG_I        (11u)
#define VOFA_CMD_ID_RG_D        (12u)
#define VOFA_CMD_ID_YA_P        (13u)
#define VOFA_CMD_ID_YA_I        (14u)
#define VOFA_CMD_ID_YA_D        (15u)
#define VOFA_CMD_ID_YG_P        (16u)
#define VOFA_CMD_ID_YG_I        (17u)
#define VOFA_CMD_ID_YG_D        (18u)
#define VOFA_CMD_ID_HP_P        (19u)
#define VOFA_CMD_ID_HP_I        (20u)
#define VOFA_CMD_ID_HP_D        (21u)
#define VOFA_CMD_ID_HV_P        (22u)
#define VOFA_CMD_ID_HV_I        (23u)
#define VOFA_CMD_ID_HV_D        (24u)
#define VOFA_CMD_ID_VON         (25u)
#define VOFA_CMD_ID_RST         (26u)
#define VOFA_CMD_ID_ATM         (27u)
#define VOFA_CMD_ID_ATS         (28u)
#define VOFA_CMD_ID_ATA         (29u)
#define VOFA_CMD_ID_ATH         (30u)
#define VOFA_CMD_ID_ATC         (31u)
#define VOFA_CMD_ID_ATB         (32u)
#define VOFA_CMD_ID_ATE         (33u)
#define VOFA_CMD_ID_ATR         (34u)
#define VOFA_CMD_ID_ATZ         (35u)

#define VOFA_AT_STATE_IDLE      (0u)
#define VOFA_AT_STATE_READY     (1u)
#define VOFA_AT_STATE_RUNNING   (2u)
#define VOFA_AT_STATE_DONE      (3u)
#define VOFA_AT_STATE_FAILED    (4u)

#define VOFA_AT_PHASE_ZERO_A    (0u)
#define VOFA_AT_PHASE_POS       (1u)
#define VOFA_AT_PHASE_ZERO_B    (2u)
#define VOFA_AT_PHASE_NEG       (3u)
#define VOFA_AT_PHASE_ZERO_C    (4u)

#define VOFA_AT_DEFAULT_HOLD_MS (500u)
#define VOFA_AT_DEFAULT_CYCLES  (3u)
#define VOFA_AT_STEP_MS         (10u)
#define VOFA_AT_START_ANGLE_LIMIT_DEG (12.0f)
#define VOFA_AT_RUN_ANGLE_LIMIT_DEG   (30.0f)

static char s_cmd_buf[VOFA_CMD_MAX_LEN];
static uint8_t s_cmd_len = 0u;
static uint8_t s_tx_enabled = 0u;
static uint8_t s_tx_divider = 0u;
static uint32_t s_cmd_ok = 0u;
static uint32_t s_cmd_err = 0u;

static uint8_t s_at_mode = 0u;
static uint8_t s_at_state = VOFA_AT_STATE_IDLE;
static uint8_t s_at_target_id = VOFA_TUNE_TARGET_PG;
static uint8_t s_at_phase = VOFA_AT_PHASE_ZERO_A;
static uint8_t s_at_cycle_index = 0u;
static uint8_t s_at_cycle_count = VOFA_AT_DEFAULT_CYCLES;
static uint8_t s_at_session_locked = 0u;
static uint8_t s_at_baseline_valid = 0u;
static uint8_t s_at_guard_reason = VOFA_TUNE_GUARD_OK;
static uint16_t s_at_hold_ms = VOFA_AT_DEFAULT_HOLD_MS;
static uint16_t s_at_phase_ms = 0u;
static float s_at_amp = 5.0f;
static float s_at_cmd = 0.0f;
static float s_at_base_pid[3] = {0.0f, 0.0f, 0.0f};
static float s_at_base_yaw = 0.0f;
static float s_at_base_height = 0.0f;

static float s_motor_m1 = 1000.0f;
static float s_motor_m2 = 1000.0f;
static float s_motor_m3 = 1000.0f;
static float s_motor_m4 = 1000.0f;

static uint8_t VofaTune_ParseValue(const char *text, float *value)
{
    char *end = NULL;
    double parsed;

    if (text == NULL || value == NULL) {
        return 0u;
    }

    while (*text == ' ' || *text == '\t') {
        text++;
    }

    parsed = strtod(text, &end);
    if (end == text) {
        return 0u;
    }

    while (*end == ' ' || *end == '\t') {
        end++;
    }
    if (*end != '\0') {
        return 0u;
    }

    if (!(parsed >= 0.0) || parsed > (double)VOFA_PARAM_MAX) {
        return 0u;
    }

    *value = (float)parsed;
    return 1u;
}

static uint8_t VofaTune_TargetFromKey(const char *key)
{
    if (key == NULL || strlen(key) != 3u) {
        return 0u;
    }
    if (key[0] == 'P' && key[1] == 'A') return VOFA_TUNE_TARGET_PA;
    if (key[0] == 'P' && key[1] == 'G') return VOFA_TUNE_TARGET_PG;
    if (key[0] == 'R' && key[1] == 'A') return VOFA_TUNE_TARGET_RA;
    if (key[0] == 'R' && key[1] == 'G') return VOFA_TUNE_TARGET_RG;
    if (key[0] == 'Y' && key[1] == 'A') return VOFA_TUNE_TARGET_YA;
    if (key[0] == 'Y' && key[1] == 'G') return VOFA_TUNE_TARGET_YG;
    if (key[0] == 'H' && key[1] == 'P') return VOFA_TUNE_TARGET_HP;
    if (key[0] == 'H' && key[1] == 'V') return VOFA_TUNE_TARGET_HV;
    return 0u;
}

static uint8_t VofaTune_IsValidTarget(uint8_t target_id)
{
    return (target_id >= VOFA_TUNE_TARGET_PA && target_id <= VOFA_TUNE_TARGET_HV) ? 1u : 0u;
}

static uint8_t VofaTune_SelectTargetPid(uint8_t target_id, pid_param_t **pid, uint16_t *base_id)
{
    if (pid == NULL || base_id == NULL) {
        return 0u;
    }

    switch (target_id) {
    case VOFA_TUNE_TARGET_PA:
        *pid = &pitch_angle_pid;
        *base_id = VOFA_CMD_ID_PA_P;
        return 1u;
    case VOFA_TUNE_TARGET_PG:
        *pid = &pitch_acc_pid;
        *base_id = VOFA_CMD_ID_PG_P;
        return 1u;
    case VOFA_TUNE_TARGET_RA:
        *pid = &roll_angle_pid;
        *base_id = VOFA_CMD_ID_RA_P;
        return 1u;
    case VOFA_TUNE_TARGET_RG:
        *pid = &roll_acc_pid;
        *base_id = VOFA_CMD_ID_RG_P;
        return 1u;
    case VOFA_TUNE_TARGET_YA:
        *pid = &yaw_angle_pid;
        *base_id = VOFA_CMD_ID_YA_P;
        return 1u;
    case VOFA_TUNE_TARGET_YG:
        *pid = &yaw_acc_pid;
        *base_id = VOFA_CMD_ID_YG_P;
        return 1u;
    case VOFA_TUNE_TARGET_HP:
        *pid = &height_pos_pid;
        *base_id = VOFA_CMD_ID_HP_P;
        return 1u;
    case VOFA_TUNE_TARGET_HV:
        *pid = &height_vel_pid;
        *base_id = VOFA_CMD_ID_HV_P;
        return 1u;
    default:
        break;
    }

    return 0u;
}

static uint8_t VofaTune_SelectAxisPid(const char *key, pid_param_t **pid, uint16_t *base_id)
{
    uint8_t target_id;

    if (key == NULL || pid == NULL || base_id == NULL) {
        return 0u;
    }

    target_id = VofaTune_TargetFromKey(key);
    if (target_id != 0u) {
        return VofaTune_SelectTargetPid(target_id, pid, base_id);
    }

    if (key[0] == 'P' && key[1] == 'A') {
        *pid = &pitch_angle_pid;
        *base_id = VOFA_CMD_ID_PA_P;
        return 1u;
    }
    if (key[0] == 'P' && key[1] == 'G') {
        *pid = &pitch_acc_pid;
        *base_id = VOFA_CMD_ID_PG_P;
        return 1u;
    }
    if (key[0] == 'R' && key[1] == 'A') {
        *pid = &roll_angle_pid;
        *base_id = VOFA_CMD_ID_RA_P;
        return 1u;
    }
    if (key[0] == 'R' && key[1] == 'G') {
        *pid = &roll_acc_pid;
        *base_id = VOFA_CMD_ID_RG_P;
        return 1u;
    }
    if (key[0] == 'Y' && key[1] == 'A') {
        *pid = &yaw_angle_pid;
        *base_id = VOFA_CMD_ID_YA_P;
        return 1u;
    }
    if (key[0] == 'Y' && key[1] == 'G') {
        *pid = &yaw_acc_pid;
        *base_id = VOFA_CMD_ID_YG_P;
        return 1u;
    }
    if (key[0] == 'H' && key[1] == 'P') {
        *pid = &height_pos_pid;
        *base_id = VOFA_CMD_ID_HP_P;
        return 1u;
    }
    if (key[0] == 'H' && key[1] == 'V') {
        *pid = &height_vel_pid;
        *base_id = VOFA_CMD_ID_HV_P;
        return 1u;
    }

    return 0u;
}

static void VofaTune_ResetPidState(pid_param_t *pid)
{
    if (pid == NULL) {
        return;
    }

    PidReset(pid);
    pid->last_last_error = 0.0f;
    pid->last_derivative = 0.0f;
    pid->last_d_output = 0.0f;
    pid->meas_inited = 0u;
}

static void VofaTune_ResetCorePidState(void)
{
    VofaTune_ResetPidState(&pitch_angle_pid);
    VofaTune_ResetPidState(&pitch_acc_pid);
    VofaTune_ResetPidState(&roll_angle_pid);
    VofaTune_ResetPidState(&roll_acc_pid);
    VofaTune_ResetPidState(&yaw_angle_pid);
    VofaTune_ResetPidState(&yaw_acc_pid);
    VofaTune_ResetPidState(&height_pos_pid);
    VofaTune_ResetPidState(&height_vel_pid);
}

static void VofaTune_ResetSelectedPidState(void)
{
    pid_param_t *pid = NULL;
    uint16_t base_id = 0u;

    if (VofaTune_SelectTargetPid(s_at_target_id, &pid, &base_id)) {
        VofaTune_ResetPidState(pid);
    }
}

static void VofaTune_SaveBaseline(void)
{
    pid_param_t *pid = NULL;
    uint16_t base_id = 0u;

    if (!VofaTune_SelectTargetPid(s_at_target_id, &pid, &base_id)) {
        s_at_baseline_valid = 0u;
        return;
    }

    s_at_base_pid[0] = pid->kp;
    s_at_base_pid[1] = pid->ki;
    s_at_base_pid[2] = pid->kd;
    s_at_base_yaw = yaw;
    s_at_base_height = height;
    s_at_baseline_valid = 1u;
}

static void VofaTune_RestoreBaseline(void)
{
    pid_param_t *pid = NULL;
    uint16_t base_id = 0u;

    if (!s_at_baseline_valid) {
        return;
    }
    if (!VofaTune_SelectTargetPid(s_at_target_id, &pid, &base_id)) {
        return;
    }

    PID_SetParam(pid, s_at_base_pid[0], s_at_base_pid[1], s_at_base_pid[2]);
    VofaTune_ResetPidState(pid);
}

static void VofaTune_ClearRunState(uint8_t state, uint8_t guard_reason)
{
    s_at_state = state;
    s_at_guard_reason = guard_reason;
    s_at_phase = VOFA_AT_PHASE_ZERO_A;
    s_at_phase_ms = 0u;
    s_at_cycle_index = 0u;
    s_at_cmd = 0.0f;
}

static void VofaTune_StopAndRestore(uint8_t guard_reason)
{
    VofaTune_RestoreBaseline();
    VofaTune_ClearRunState(VOFA_AT_STATE_FAILED, guard_reason);
    s_at_mode = 0u;
    s_at_session_locked = 0u;
}

static float VofaTune_CommandForPhase(uint8_t phase)
{
    if (phase == VOFA_AT_PHASE_POS) {
        return s_at_amp;
    }
    if (phase == VOFA_AT_PHASE_NEG) {
        return -s_at_amp;
    }
    return 0.0f;
}

static void VofaTune_AdvancePhase(void)
{
    if (s_at_phase < VOFA_AT_PHASE_ZERO_C) {
        s_at_phase++;
        s_at_phase_ms = 0u;
        s_at_cmd = VofaTune_CommandForPhase(s_at_phase);
        return;
    }

    s_at_cycle_index++;
    if (s_at_cycle_index >= s_at_cycle_count) {
        VofaTune_ClearRunState(VOFA_AT_STATE_DONE, VOFA_TUNE_GUARD_OK);
        return;
    }

    s_at_phase = VOFA_AT_PHASE_ZERO_A;
    s_at_phase_ms = 0u;
    s_at_cmd = 0.0f;
}

static void VofaTune_BeginRun(void)
{
    if (!s_at_mode || !VofaTune_IsValidTarget(s_at_target_id)) {
        s_at_guard_reason = VOFA_TUNE_GUARD_TARGET;
        s_cmd_err++;
        return;
    }
    if (lockflag != 0u) {
        s_at_guard_reason = VOFA_TUNE_GUARD_LOCK;
        s_cmd_err++;
        return;
    }
    if (fabsf(pitch) >= VOFA_AT_START_ANGLE_LIMIT_DEG ||
        fabsf(roll) >= VOFA_AT_START_ANGLE_LIMIT_DEG) {
        s_at_guard_reason = VOFA_TUNE_GUARD_ANGLE;
        s_cmd_err++;
        return;
    }
    if (s_at_state == VOFA_AT_STATE_RUNNING) {
        s_at_guard_reason = VOFA_TUNE_GUARD_CMD;
        s_cmd_err++;
        return;
    }

    VofaTune_SaveBaseline();
    if (!s_at_baseline_valid) {
        s_at_guard_reason = VOFA_TUNE_GUARD_TARGET;
        s_cmd_err++;
        return;
    }

    s_at_session_locked = 1u;
    s_at_state = VOFA_AT_STATE_RUNNING;
    s_at_guard_reason = VOFA_TUNE_GUARD_OK;
    s_at_phase = VOFA_AT_PHASE_ZERO_A;
    s_at_phase_ms = 0u;
    s_at_cycle_index = 0u;
    s_at_cmd = 0.0f;
    VofaTune_ResetSelectedPidState();
}

static void VofaTune_ProcessAutoStep(void)
{
    if (s_at_state != VOFA_AT_STATE_RUNNING) {
        return;
    }

    if (fabsf(pitch) >= VOFA_AT_RUN_ANGLE_LIMIT_DEG ||
        fabsf(roll) >= VOFA_AT_RUN_ANGLE_LIMIT_DEG) {
        VofaTune_StopAndRestore(VOFA_TUNE_GUARD_ANGLE);
        return;
    }
    if (lockflag != 0u) {
        VofaTune_StopAndRestore(VOFA_TUNE_GUARD_LOCK);
        return;
    }

    s_at_phase_ms = (uint16_t)(s_at_phase_ms + VOFA_AT_STEP_MS);
    if (s_at_phase_ms >= s_at_hold_ms) {
        VofaTune_AdvancePhase();
    }
}

static uint8_t VofaTune_SetPidValue(const char *key, float value, uint16_t *cmd_id)
{
    pid_param_t *pid = NULL;
    uint16_t base_id = 0u;
    uint16_t offset = 0u;
    uint8_t target_id = 0u;

    if (key == NULL || cmd_id == NULL) {
        return 0u;
    }
    if (strlen(key) != 3u) {
        return 0u;
    }
    target_id = VofaTune_TargetFromKey(key);
    if (!VofaTune_SelectAxisPid(key, &pid, &base_id)) {
        return 0u;
    }
    if (s_at_session_locked && target_id != s_at_target_id) {
        s_at_guard_reason = VOFA_TUNE_GUARD_TARGET;
        return 0u;
    }

    if (key[2] == 'P') {
        pid->kp = value;
        offset = 0u;
    } else if (key[2] == 'I') {
        pid->ki = value;
        offset = 1u;
    } else if (key[2] == 'D') {
        pid->kd = value;
        offset = 2u;
    } else {
        return 0u;
    }

    *cmd_id = (uint16_t)(base_id + offset);
    return 1u;
}

static void VofaTune_RecordOk(uint16_t cmd_id, float value)
{
    (void)cmd_id;
    (void)value;
    s_cmd_ok++;
}

static void VofaTune_HandleCommand(char *cmd)
{
    char *eq = NULL;
    float value = 0.0f;
    uint16_t cmd_id = 0u;

    if (cmd == NULL || cmd[0] == '\0') {
        s_cmd_err++;
        return;
    }

    eq = strchr(cmd, '=');
    if (eq == NULL || eq == cmd || eq[1] == '\0') {
        s_cmd_err++;
        return;
    }

    *eq = '\0';
    if (!VofaTune_ParseValue(eq + 1, &value)) {
        s_cmd_err++;
        return;
    }

    if (strcmp(cmd, "ATM") == 0) {
        if (value == 1.0f) {
            s_at_mode = 1u;
            if (s_at_state == VOFA_AT_STATE_IDLE || s_at_state == VOFA_AT_STATE_FAILED) {
                s_at_state = VOFA_AT_STATE_READY;
            }
            s_at_guard_reason = VOFA_TUNE_GUARD_OK;
            VofaTune_RecordOk(VOFA_CMD_ID_ATM, value);
        } else if (value == 0.0f) {
            if (s_at_state == VOFA_AT_STATE_RUNNING) {
                VofaTune_RestoreBaseline();
            }
            s_at_mode = 0u;
            s_at_session_locked = 0u;
            s_at_baseline_valid = 0u;
            VofaTune_ClearRunState(VOFA_AT_STATE_IDLE, VOFA_TUNE_GUARD_OK);
            VofaTune_RecordOk(VOFA_CMD_ID_ATM, value);
        } else {
            s_cmd_err++;
        }
        return;
    }

    if (strcmp(cmd, "ATS") == 0) {
        uint8_t target_id = (uint8_t)value;
        if (s_at_state == VOFA_AT_STATE_RUNNING) {
            s_cmd_err++;
            s_at_guard_reason = VOFA_TUNE_GUARD_CMD;
        } else if (value == (float)target_id && VofaTune_IsValidTarget(target_id)) {
            if (s_at_session_locked && target_id != s_at_target_id) {
                s_cmd_err++;
                s_at_guard_reason = VOFA_TUNE_GUARD_TARGET;
            } else {
                s_at_target_id = target_id;
                s_at_guard_reason = VOFA_TUNE_GUARD_OK;
                VofaTune_RecordOk(VOFA_CMD_ID_ATS, value);
            }
        } else {
            s_cmd_err++;
            s_at_guard_reason = VOFA_TUNE_GUARD_TARGET;
        }
        return;
    }

    if (strcmp(cmd, "ATA") == 0) {
        if (value > 0.0f) {
            s_at_amp = value;
            VofaTune_RecordOk(VOFA_CMD_ID_ATA, value);
        } else {
            s_cmd_err++;
        }
        return;
    }

    if (strcmp(cmd, "ATH") == 0) {
        uint16_t hold_ms = (uint16_t)value;
        if (value == (float)hold_ms && hold_ms >= VOFA_AT_STEP_MS) {
            s_at_hold_ms = hold_ms;
            VofaTune_RecordOk(VOFA_CMD_ID_ATH, value);
        } else {
            s_cmd_err++;
        }
        return;
    }

    if (strcmp(cmd, "ATC") == 0) {
        uint8_t cycle_count = (uint8_t)value;
        if (value == (float)cycle_count && cycle_count >= 1u && cycle_count <= 20u) {
            s_at_cycle_count = cycle_count;
            VofaTune_RecordOk(VOFA_CMD_ID_ATC, value);
        } else {
            s_cmd_err++;
        }
        return;
    }

    if (strcmp(cmd, "ATB") == 0) {
        if (value == 1.0f) {
            uint32_t before_err = s_cmd_err;
            VofaTune_BeginRun();
            if (s_cmd_err == before_err && s_at_state == VOFA_AT_STATE_RUNNING) {
                VofaTune_RecordOk(VOFA_CMD_ID_ATB, value);
            }
        } else {
            s_cmd_err++;
        }
        return;
    }

    if (strcmp(cmd, "ATE") == 0) {
        if (value == 1.0f) {
            VofaTune_RestoreBaseline();
            s_at_session_locked = 0u;
            VofaTune_ClearRunState(VOFA_AT_STATE_FAILED, VOFA_TUNE_GUARD_CMD);
            VofaTune_RecordOk(VOFA_CMD_ID_ATE, value);
        } else {
            s_cmd_err++;
        }
        return;
    }

    if (strcmp(cmd, "ATR") == 0) {
        if (value == 1.0f) {
            VofaTune_RestoreBaseline();
            s_at_session_locked = 0u;
            VofaTune_ClearRunState(s_at_mode ? VOFA_AT_STATE_READY : VOFA_AT_STATE_IDLE,
                                   VOFA_TUNE_GUARD_OK);
            VofaTune_RecordOk(VOFA_CMD_ID_ATR, value);
        } else {
            s_cmd_err++;
        }
        return;
    }

    if (strcmp(cmd, "ATZ") == 0) {
        if (value == 1.0f && VofaTune_IsValidTarget(s_at_target_id)) {
            VofaTune_ResetSelectedPidState();
            VofaTune_RecordOk(VOFA_CMD_ID_ATZ, value);
        } else {
            s_cmd_err++;
        }
        return;
    }

    if (strcmp(cmd, "VON") == 0) {
        if (value == 0.0f || value == 1.0f) {
            s_tx_enabled = (value > 0.5f) ? 1u : 0u;
            VofaTune_RecordOk(VOFA_CMD_ID_VON, value);
        } else {
            s_cmd_err++;
        }
        return;
    }

    if (strcmp(cmd, "RST") == 0) {
        if (value == 1.0f) {
            VofaTune_ResetCorePidState();
            VofaTune_RecordOk(VOFA_CMD_ID_RST, value);
        } else {
            s_cmd_err++;
        }
        return;
    }

    if (VofaTune_SetPidValue(cmd, value, &cmd_id)) {
        VofaTune_RecordOk(cmd_id, value);
    } else {
        s_cmd_err++;
    }
}

static void VofaTune_FeedByte(uint8_t byte)
{
    if (byte == '\r') {
        return;
    }

    if (byte == '\n') {
        if (s_cmd_len > 0u) {
            s_cmd_buf[s_cmd_len] = '\0';
            VofaTune_HandleCommand(s_cmd_buf);
            s_cmd_len = 0u;
        }
        return;
    }

    if (byte < 32u || byte > 126u) {
        s_cmd_len = 0u;
        s_cmd_err++;
        return;
    }

    if (s_cmd_len >= (VOFA_CMD_MAX_LEN - 1u)) {
        s_cmd_len = 0u;
        s_cmd_err++;
        return;
    }

    s_cmd_buf[s_cmd_len++] = (char)byte;
}

static void VofaTune_ProcessRx(void)
{
    uint8_t rx[VOFA_RX_BATCH_LEN];
    uint32_t len = VOFA_RX_BATCH_LEN;
    uint8 byte = 0u;

    fifo_read_buffer(puart_fifo_s[0], rx, &len, FIFO_READ_AND_CLEAN);
    for (uint32_t i = 0u; i < len; i++) {
        VofaTune_FeedByte(rx[i]);
    }

    for (uint8_t i = 0u; i < VOFA_HW_RX_POLL_LEN; i++) {
        if (!uart_irq_query_byte(UART_0, &byte)) {
            break;
        }
        VofaTune_FeedByte(byte);
    }
}

static void VofaTune_GetSelectedTelemetry(float *cmd_sp, float *meas, float *error,
                                          float *out, float *kp, float *ki, float *kd)
{
    pid_param_t *pid = NULL;
    uint16_t base_id = 0u;
    float local_cmd = s_at_cmd;
    float local_meas = 0.0f;
    float local_error = 0.0f;
    float local_out = 0.0f;

    if (!VofaTune_SelectTargetPid(s_at_target_id, &pid, &base_id)) {
        pid = NULL;
    }

    switch (s_at_target_id) {
    case VOFA_TUNE_TARGET_PA:
        local_meas = pitch;
        local_out = pitch_angle_pid.out;
        local_error = local_cmd - local_meas;
        break;
    case VOFA_TUNE_TARGET_PG:
        local_meas = pitch_gyro;
        local_out = pitch_acc_pid.out;
        local_error = local_cmd - local_meas;
        break;
    case VOFA_TUNE_TARGET_RA:
        local_meas = roll;
        local_out = roll_angle_pid.out;
        local_error = local_cmd - local_meas;
        break;
    case VOFA_TUNE_TARGET_RG:
        local_meas = roll_gyro;
        local_out = roll_acc_pid.out;
        local_error = local_cmd - local_meas;
        break;
    case VOFA_TUNE_TARGET_YA:
        local_meas = s_at_baseline_valid ? (yaw - s_at_base_yaw) : 0.0f;
        local_out = yaw_angle_pid.out;
        local_error = local_cmd - local_meas;
        break;
    case VOFA_TUNE_TARGET_YG:
        local_meas = yaw_gyro;
        local_out = yaw_acc_pid.out;
        local_error = local_cmd - local_meas;
        break;
    case VOFA_TUNE_TARGET_HP:
        local_cmd = s_at_baseline_valid ? (s_at_base_height + s_at_cmd) : target_height;
        local_meas = height;
        local_out = height_pos_pid.out;
        local_error = local_cmd - local_meas;
        break;
    case VOFA_TUNE_TARGET_HV:
        local_meas = imu_vertical_velocity_smoothed;
        local_out = height_vel_pid.out;
        local_error = local_cmd - local_meas;
        break;
    default:
        break;
    }

    if (cmd_sp != NULL) *cmd_sp = local_cmd;
    if (meas != NULL) *meas = local_meas;
    if (error != NULL) *error = local_error;
    if (out != NULL) *out = local_out;
    if (kp != NULL) *kp = (pid != NULL) ? pid->kp : 0.0f;
    if (ki != NULL) *ki = (pid != NULL) ? pid->ki : 0.0f;
    if (kd != NULL) *kd = (pid != NULL) ? pid->kd : 0.0f;
}

static void VofaTune_SendTelemetry(void)
{
    static char line[960];
    horizontal_velocity_debug_t hvel_debug;
    float cmd_sp = 0.0f;
    float meas = 0.0f;
    float error = 0.0f;
    float out = 0.0f;
    float kp = 0.0f;
    float ki = 0.0f;
    float kd = 0.0f;
    int len;

    VofaTune_GetSelectedTelemetry(&cmd_sp, &meas, &error, &out, &kp, &ki, &kd);
    horizontal_velocity_get_debug(&hvel_debug);

    len = snprintf(line,
                   sizeof(line),
                   "%lu,%u,%u,%u,%u,%u,"
                   "%.3f,%.3f,%.3f,%.3f,%.5f,%.5f,%.5f,"
                   "%.3f,%.3f,%.3f,%.3f,"
                   "%.3f,%.3f,%.3f,%.3f,"
                   "%.3f,%.3f,%.3f,%.3f,"
                   "%.3f,%.3f,%.3f,%.3f,%.3f,"
                   "%.3f,%.3f,%.3f,%.3f,"
                   "%u,%u,%lu,%lu,"
                   "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,"
                   "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,"
                   "%u,%u\r\n",
                   (unsigned long)(GetSystemTimer() / 1000u),
                   (unsigned int)s_at_mode,
                   (unsigned int)s_at_state,
                   (unsigned int)s_at_target_id,
                   (unsigned int)s_at_phase,
                   (unsigned int)s_at_guard_reason,
                   cmd_sp,
                   meas,
                   error,
                   out,
                   kp,
                   ki,
                   kd,
                   pitch,
                   pitch_gyro,
                   pitch_angle_pid.out,
                   pitch_acc_pid.out,
                   roll,
                   roll_gyro,
                   roll_angle_pid.out,
                   roll_acc_pid.out,
                   yaw,
                   yaw_gyro,
                   yaw_angle_pid.out,
                   yaw_acc_pid.out,
                   height,
                   target_height,
                   height_pos_pid.out,
                   height_vel_pid.out,
                   min_motor,
                   s_motor_m1,
                   s_motor_m2,
                   s_motor_m3,
                   s_motor_m4,
                   (unsigned int)lockflag,
                   (unsigned int)height_Loc_flag,
                   (unsigned long)s_cmd_ok,
                   (unsigned long)s_cmd_err,
                   hvel_debug.pitch.target_cmd_cms,
                   hvel_debug.pitch.target_ref_cms,
                   hvel_debug.pitch.target_accel_cms2,
                   hvel_debug.pitch.measured_cms,
                   hvel_debug.pitch.derivative_raw_cms2,
                   hvel_debug.pitch.derivative_filtered_cms2,
                   hvel_debug.pitch.out_p_deg,
                   hvel_debug.pitch.out_i_deg,
                   hvel_debug.pitch.out_d_deg,
                   hvel_debug.pitch.out_ff_deg,
                   hvel_debug.pitch.out_total_deg,
                   hvel_debug.roll.target_cmd_cms,
                   hvel_debug.roll.target_ref_cms,
                   hvel_debug.roll.target_accel_cms2,
                   hvel_debug.roll.measured_cms,
                   hvel_debug.roll.derivative_raw_cms2,
                   hvel_debug.roll.derivative_filtered_cms2,
                   hvel_debug.roll.out_p_deg,
                   hvel_debug.roll.out_i_deg,
                   hvel_debug.roll.out_d_deg,
                   hvel_debug.roll.out_ff_deg,
                   hvel_debug.roll.out_total_deg,
                   (unsigned int)hvel_debug.pitch.flow_valid,
                   (unsigned int)hvel_debug.roll.flow_valid);

    if (len <= 0) {
        return;
    }
    if ((uint32_t)len >= sizeof(line)) {
        len = (int)(sizeof(line) - 1u);
    }

    uartSend0((uint8_t *)line, (uint16_t)len);
}

void VofaTune_Process10ms(void)
{
    VofaTune_ProcessRx();
    VofaTune_ProcessAutoStep();

    if (!s_tx_enabled) {
        s_tx_divider = 0u;
        return;
    }

    s_tx_divider++;
    if (s_tx_divider >= VOFA_TX_DIV_20HZ) {
        s_tx_divider = 0u;
        VofaTune_SendTelemetry();
    }
}

void VofaTune_UpdateMotorOutputs(float m1, float m2, float m3, float m4)
{
    s_motor_m1 = m1;
    s_motor_m2 = m2;
    s_motor_m3 = m3;
    s_motor_m4 = m4;
}

void VofaTune_AbortAndRestore(void)
{
    if (s_at_state != VOFA_AT_STATE_RUNNING &&
        !s_at_session_locked &&
        fabsf(s_at_cmd) <= 0.001f) {
        return;
    }

    VofaTune_RestoreBaseline();
    s_at_mode = 0u;
    s_at_session_locked = 0u;
    s_at_baseline_valid = 0u;
    VofaTune_ClearRunState(VOFA_AT_STATE_IDLE, VOFA_TUNE_GUARD_LOCK);
}

uint8_t VofaTune_IsActiveTarget(uint8_t target_id)
{
    return (s_at_mode &&
            s_at_state == VOFA_AT_STATE_RUNNING &&
            s_at_target_id == target_id) ? 1u : 0u;
}

uint8_t VofaTune_GetActiveCommand(uint8_t target_id, float *cmd)
{
    if (!VofaTune_IsActiveTarget(target_id) || cmd == NULL) {
        return 0u;
    }
    *cmd = s_at_cmd;
    return 1u;
}

uint8_t VofaTune_GetYawTarget(float *target_deg)
{
    if (!VofaTune_IsActiveTarget(VOFA_TUNE_TARGET_YA) || target_deg == NULL) {
        return 0u;
    }
    *target_deg = s_at_base_yaw + s_at_cmd;
    while (*target_deg > 180.0f) *target_deg -= 360.0f;
    while (*target_deg < -180.0f) *target_deg += 360.0f;
    return 1u;
}

uint8_t VofaTune_GetHeightTarget(float *target_mm)
{
    if (!VofaTune_IsActiveTarget(VOFA_TUNE_TARGET_HP) || target_mm == NULL) {
        return 0u;
    }
    *target_mm = s_at_base_height + s_at_cmd;
    return 1u;
}
