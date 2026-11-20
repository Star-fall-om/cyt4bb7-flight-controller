#include "m70_fault.h"
#include "zf_common_headfile.h"
#include "Motor/motor.h"
#include "FreeRTOS.h"
#include "task.h"

#define M70_FAULT_RECORD_MAGIC          (0x4D374652u)
#define M70_FAULT_RECORD_VERSION        (1u)
#define M70_FAULT_RECORD_WORDS          (32u)
#define M70_FAULT_CHECKSUM_SEED         (0xA55AA55Au)

#define M70_FAULT_KIND_HARDFAULT        (1u)
#define M70_FAULT_KIND_STACK_OVERFLOW   (2u)
#define M70_FAULT_KIND_MALLOC_FAILED    (3u)
#define M70_FAULT_KIND_THREAD_CREATE    (4u)

typedef struct {
    uint32_t magic;
    uint32_t magic_inv;
    uint32_t version;
    uint32_t kind;
    uint32_t sequence;
    uint32_t reported;
    uint32_t reset_reason;
    uint32_t cfsr;
    uint32_t hfsr;
    uint32_t shcsr;
    uint32_t mmfar;
    uint32_t bfar;
    uint32_t r0;
    uint32_t r1;
    uint32_t r2;
    uint32_t r3;
    uint32_t r12;
    uint32_t lr;
    uint32_t pc;
    uint32_t psr;
    uint32_t msp;
    uint32_t psp;
    uint32_t control;
    uint32_t basepri;
    uint32_t primask;
    uint32_t task_handle;
    uint32_t task_index;
    uint32_t checksum;
    uint32_t task_name[4];
} M70FaultRecord_t;

typedef union {
    M70FaultRecord_t record;
    uint32_t words[M70_FAULT_RECORD_WORDS];
} M70FaultRecordStorage_t;

typedef char M70FaultRecordSizeCheck[
    (sizeof(M70FaultRecordStorage_t) == (M70_FAULT_RECORD_WORDS * sizeof(uint32_t))) ? 1 : -1];

#if defined(__ICCARM__)
#pragma data_alignment=32
__no_init static volatile M70FaultRecordStorage_t g_m70_fault_storage;
#else
static volatile M70FaultRecordStorage_t g_m70_fault_storage
    __attribute__((section(".noinit"), aligned(32)));
#endif

static uint8_t g_m70_fault_record_initialized = 0u;
static uint8_t g_m70_fault_report_pending = 0u;

extern uint8_t lockflag;

static uint32_t M70FaultRecordChecksum(const volatile M70FaultRecord_t *record)
{
    uint32_t checksum = M70_FAULT_CHECKSUM_SEED;

    checksum ^= record->version;
    checksum ^= record->kind;
    checksum ^= record->sequence;
    checksum ^= record->reported;
    checksum ^= record->reset_reason;
    checksum ^= record->cfsr;
    checksum ^= record->hfsr;
    checksum ^= record->shcsr;
    checksum ^= record->mmfar;
    checksum ^= record->bfar;
    checksum ^= record->r0;
    checksum ^= record->r1;
    checksum ^= record->r2;
    checksum ^= record->r3;
    checksum ^= record->r12;
    checksum ^= record->lr;
    checksum ^= record->pc;
    checksum ^= record->psr;
    checksum ^= record->msp;
    checksum ^= record->psp;
    checksum ^= record->control;
    checksum ^= record->basepri;
    checksum ^= record->primask;
    checksum ^= record->task_handle;
    checksum ^= record->task_index;
    checksum ^= record->task_name[0];
    checksum ^= record->task_name[1];
    checksum ^= record->task_name[2];
    checksum ^= record->task_name[3];

    return checksum;
}

static uint8_t M70FaultRecordIsValid(void)
{
    const volatile M70FaultRecord_t *record = &g_m70_fault_storage.record;

    return (record->magic == M70_FAULT_RECORD_MAGIC &&
            record->magic_inv == ~M70_FAULT_RECORD_MAGIC &&
            record->version == M70_FAULT_RECORD_VERSION &&
            record->checksum == M70FaultRecordChecksum(record)) ? 1u : 0u;
}

static void M70FaultRecordClearWords(void)
{
    uint32_t i;

    for (i = 0u; i < M70_FAULT_RECORD_WORDS; i++) {
        g_m70_fault_storage.words[i] = 0u;
    }
}

static void M70FaultRecordCleanCache(void)
{
    SCB_CleanDCache_by_Addr((volatile void *)&g_m70_fault_storage,
                            (int32_t)sizeof(g_m70_fault_storage));
    __DSB();
}

static void M70FaultRecordSetTaskName(const char *task_name)
{
    volatile char *dst = (volatile char *)g_m70_fault_storage.record.task_name;
    uint32_t i;

    for (i = 0u; i < sizeof(g_m70_fault_storage.record.task_name); i++) {
        dst[i] = '\0';
    }
    if (task_name == NULL) {
        return;
    }
    for (i = 0u; i < (sizeof(g_m70_fault_storage.record.task_name) - 1u); i++) {
        char value = task_name[i];
        dst[i] = value;
        if (value == '\0') {
            break;
        }
    }
}

static void M70FaultRecordCommit(void)
{
    volatile M70FaultRecord_t *record = &g_m70_fault_storage.record;

    record->magic = 0u;
    record->magic_inv = ~M70_FAULT_RECORD_MAGIC;
    record->checksum = M70FaultRecordChecksum(record);
    __DMB();
    record->magic = M70_FAULT_RECORD_MAGIC;
    M70FaultRecordCleanCache();
}

static void M70FaultRecordCapture(uint32_t kind,
                                  uint32_t task_handle,
                                  uint32_t task_index,
                                  const char *task_name,
                                  uint8_t copy_fault_frame)
{
    volatile M70FaultRecord_t *record = &g_m70_fault_storage.record;
    uint32_t sequence = 1u;

    if (g_m70_fault_record_initialized && M70FaultRecordIsValid()) {
        sequence = record->sequence + 1u;
    }

    M70FaultRecordClearWords();
    g_m70_fault_record_initialized = 1u;

    record->version = M70_FAULT_RECORD_VERSION;
    record->kind = kind;
    record->sequence = sequence;
    record->reported = 0u;
    record->msp = __get_MSP();
    record->psp = __get_PSP();
    record->control = __get_CONTROL();
    record->basepri = __get_BASEPRI();
    record->primask = __get_PRIMASK();
    record->task_handle = task_handle;
    record->task_index = task_index;
    M70FaultRecordSetTaskName(task_name);

    if (copy_fault_frame) {
        record->cfsr = cy_faultFrame.cfsr;
        record->hfsr = SCB->HFSR;
        record->shcsr = SCB->SHCSR;
        record->mmfar = SCB->MMFAR;
        record->bfar = SCB->BFAR;
        record->r0 = cy_faultFrame.r0;
        record->r1 = cy_faultFrame.r1;
        record->r2 = cy_faultFrame.r2;
        record->r3 = cy_faultFrame.r3;
        record->r12 = cy_faultFrame.r12;
        record->lr = cy_faultFrame.lr;
        record->pc = cy_faultFrame.pc;
        record->psr = cy_faultFrame.psr;
    }

    M70FaultRecordCommit();
}

static void M70FaultRecordCaptureAndReset(uint32_t kind,
                                          uint32_t task_handle,
                                          uint32_t task_index,
                                          const char *task_name,
                                          uint8_t copy_fault_frame)
{
    interrupt_global_disable();
    M70FaultRecordCapture(kind, task_handle, task_index, task_name, copy_fault_frame);
    NVIC_SystemReset();
}

void M70Fault_BootInit(void)
{
    uint32_t reset_reason = Cy_SysReset_GetResetReason();
    volatile M70FaultRecord_t *record = &g_m70_fault_storage.record;

    g_m70_fault_report_pending = 0u;

    if ((reset_reason & CY_SYSRESET_SOFT) != 0u) {
        SCB_InvalidateDCache_by_Addr((volatile void *)&g_m70_fault_storage,
                                    (int32_t)sizeof(g_m70_fault_storage));
        g_m70_fault_record_initialized = 1u;
        if (M70FaultRecordIsValid()) {
            record->reset_reason = reset_reason;
            record->checksum = M70FaultRecordChecksum(record);
            M70FaultRecordCleanCache();
            if (record->reported == 0u) {
                g_m70_fault_report_pending = 1u;
            }
        } else {
            M70FaultRecordClearWords();
            M70FaultRecordCleanCache();
        }
    } else {
        M70FaultRecordClearWords();
        g_m70_fault_record_initialized = 1u;
        M70FaultRecordCleanCache();
    }

    Cy_SysReset_ClearAllResetReasons();
}

void M70Fault_Report(void)
{
    volatile M70FaultRecord_t *record = &g_m70_fault_storage.record;
    const char *task_name = (const char *)record->task_name;

    if (!g_m70_fault_report_pending || !M70FaultRecordIsValid()) {
        return;
    }

    printf("[M70 FAULT] kind=%lu seq=%lu task=%08lX idx=%lu name=%.15s reset=%08lX\r\n",
           (unsigned long)record->kind,
           (unsigned long)record->sequence,
           (unsigned long)record->task_handle,
           (unsigned long)record->task_index,
           task_name,
           (unsigned long)record->reset_reason);
    printf("CFSR=%08lX HFSR=%08lX PC=%08lX LR=%08lX PSR=%08lX\r\n",
           (unsigned long)record->cfsr,
           (unsigned long)record->hfsr,
           (unsigned long)record->pc,
           (unsigned long)record->lr,
           (unsigned long)record->psr);
    printf("PSP=%08lX MSP=%08lX SHCSR=%08lX MMFAR=%08lX BFAR=%08lX\r\n",
           (unsigned long)record->psp,
           (unsigned long)record->msp,
           (unsigned long)record->shcsr,
           (unsigned long)record->mmfar,
           (unsigned long)record->bfar);

    record->reported = 1u;
    record->checksum = M70FaultRecordChecksum(record);
    M70FaultRecordCleanCache();
    g_m70_fault_report_pending = 0u;
}

void M70Fault_ThreadCreateFailed(uint32_t task_index,
                                 const char *task_name,
                                 uint8_t runtime_started)
{
    lockflag = 1u;
    if (runtime_started) {
        motor(1, 1000);
        motor(2, 1000);
        motor(3, 1000);
        motor(4, 1000);
    }
    M70FaultRecordCaptureAndReset(M70_FAULT_KIND_THREAD_CREATE,
                                  0u,
                                  task_index,
                                  task_name,
                                  0u);
}

void Cy_SysLib_ProcessingFault(void)
{
    M70FaultRecordCaptureAndReset(M70_FAULT_KIND_HARDFAULT,
                                  0u,
                                  0u,
                                  NULL,
                                  1u);
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, signed char *pcTaskName)
{
    M70FaultRecordCaptureAndReset(M70_FAULT_KIND_STACK_OVERFLOW,
                                  (uint32_t)(uintptr_t)xTask,
                                  0u,
                                  (const char *)pcTaskName,
                                  0u);
}

void vApplicationMallocFailedHook(void)
{
    M70FaultRecordCaptureAndReset(M70_FAULT_KIND_MALLOC_FAILED,
                                  0u,
                                  0u,
                                  "malloc",
                                  0u);
}
