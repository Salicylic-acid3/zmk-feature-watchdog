/*
 * Copyright (c) 2026 cormoran
 *
 * SPDX-License-Identifier: MIT
 */

#include <string.h>

#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/task_wdt/task_wdt.h>

#include <cormoran/zmk/watchdog.h>
#include <zmk/workqueue.h>

#if defined(CONFIG_CPU_CORTEX_M)
#include <cmsis_core.h>
#include <zephyr/linker/linker-defs.h>
#endif

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

/*
 * Monitor table: one task_wdt channel per monitored work queue, fed by a
 * self-rescheduling k_work_delayable submitted to that same queue. If the
 * queue is blocked/starved longer than CONFIG_ZMK_WATCHDOG_FREEZE_TIMEOUT_MS,
 * the feed work never runs and the channel's callback fires (ISR/timer
 * context). See DESIGN.md SS4.1.
 *
 * Data-driven so adding a monitored queue is a one-line change: append an
 * entry to watchdog_freeze_monitors[] below.
 */

struct watchdog_freeze_monitor {
    /* Returns the queue to monitor. Called once at init. */
    struct k_work_q *(*get_queue)(void);
    /* Name recorded in the incident (truncated to
     * ZMK_WATCHDOG_QUEUE_NAME_LEN - 1 if longer). */
    const char *name;

    /* Filled in at init. */
    int channel_id;
    struct k_work_delayable feed_work;
};

static struct k_work_q *get_sys_work_q(void) { return &k_sys_work_q; }

static struct watchdog_freeze_monitor watchdog_freeze_monitors[] = {
    {
        .get_queue = get_sys_work_q,
        .name = "sysworkq",
    },
#if IS_ENABLED(CONFIG_ZMK_WATCHDOG_FREEZE_MONITOR_LOWPRIO_QUEUE)
    {
        /* ZMK_LOW_PRIORITY_WORK_QUEUE is select'd by CONFIG_ZMK_WATCHDOG
         * (see Kconfig), so this queue always exists whenever this file is
         * compiled in -- no extra #ifdef needed for the queue itself, only
         * for whether we monitor it (see CONFIG_ZMK_WATCHDOG_FREEZE_MONITOR_
         * LOWPRIO_QUEUE's help: disabling this halves the module's own
         * periodic k_timer footprint). */
        .get_queue = zmk_workqueue_lowprio_work_q,
        .name = "lowprio_workq",
    },
#endif
};

#if defined(CONFIG_CPU_CORTEX_M)
/*
 * Where the frozen queue's thread is.
 *
 * A freeze record used to say only which queue stopped. That was enough to
 * know something was wrong and never enough to know what: the same
 * "lowprio_workq" record can mean a Bluetooth buffer wait, a Studio RPC
 * write that never drains, or a deadlock on a mutex. So the callback now
 * also reads the thread's saved context and keeps a few code addresses,
 * which the app turns into function names with the build's ELF, the same
 * way it does for a crash's PC and LR.
 *
 * Runs in the task_wdt timer ISR. Everything here is plain memory reads
 * bounded to the thread's own stack; nothing locks or allocates.
 */

/* How far up the stack to look for return addresses, in words. */
#define FREEZE_STACK_SCAN_WORDS 256

static bool freeze_addr_in_text(uint32_t addr) {
    return addr >= (uint32_t)(uintptr_t)__text_region_start &&
           addr < (uint32_t)(uintptr_t)__text_region_end;
}

/* A Thumb return address: odd, in .text, and the instruction just before it
 * is a BL (32-bit) or a BLX Rm (16-bit). Filters out data that merely looks
 * like a code pointer, such as function pointers stored on the stack. */
static bool freeze_is_return_address(uint32_t value) {
    if ((value & 1U) == 0U) {
        return false;
    }
    const uint32_t ret = value & ~1U;
    if (!freeze_addr_in_text(ret - 4U) || !freeze_addr_in_text(ret)) {
        return false;
    }
    const uint16_t *hw = (const uint16_t *)(uintptr_t)(ret - 4U);
    if ((hw[0] & 0xF800U) == 0xF000U && (hw[1] & 0xD000U) == 0xD000U) {
        return true; /* BL <label> */
    }
    if ((hw[1] & 0xFF87U) == 0x4780U) {
        return true; /* BLX Rm */
    }
    return false;
}

static void freeze_capture_thread(k_tid_t thread,
                                  struct zmk_watchdog_incident_freeze_detail *detail) {
    if (thread == NULL) {
        return;
    }

    detail->thread_state = thread->base.thread_state;
    detail->pended_on = (uint32_t)(uintptr_t)thread->base.pended_on;

    /* A thread switched out keeps its exception frame at callee_saved.psp.
     * The one this timer interrupted is still running: its frame is at the
     * live PSP, pushed by the hardware on entry to this ISR. */
    const uint32_t psp =
        (thread == k_current_get()) ? __get_PSP() : (uint32_t)thread->callee_saved.psp;

    uintptr_t lo = CONFIG_SRAM_BASE_ADDRESS;
    uintptr_t hi = CONFIG_SRAM_BASE_ADDRESS + (CONFIG_SRAM_SIZE * 1024U);
#if defined(CONFIG_THREAD_STACK_INFO)
    lo = MAX(lo, thread->stack_info.start);
    hi = MIN(hi, thread->stack_info.start + thread->stack_info.size);
#endif
    /* The basic exception frame is 8 words: r0-r3, r12, lr, pc, xpsr. */
    if (psp < lo || psp % 4U != 0U || (uintptr_t)psp + 8U * 4U > hi) {
        return;
    }

    const uint32_t *frame = (const uint32_t *)(uintptr_t)psp;
    uint8_t n = 0;
    detail->frames[n++] = frame[6]; /* PC */
    detail->frames[n++] = frame[5]; /* LR */

    const uint32_t *end = frame + 8 + FREEZE_STACK_SCAN_WORDS;
    if ((uintptr_t)end > hi) {
        end = (const uint32_t *)hi;
    }
    for (const uint32_t *p = frame + 8; p < end && n < ZMK_WATCHDOG_FREEZE_FRAMES; p++) {
        const uint32_t value = *p;
        if (freeze_is_return_address(value) && value != detail->frames[n - 1]) {
            detail->frames[n++] = value;
        }
    }
    detail->frame_count = n;
}
#endif /* CONFIG_CPU_CORTEX_M */

/* task_wdt callback: ISR (timer) context. Must be minimal -- no flash, no
 * locking, no logging subsystem calls beyond what zmk_watchdog_pending_set()
 * already restricts itself to. */
static void watchdog_freeze_channel_fired(int channel_id, void *user_data) {
    struct watchdog_freeze_monitor *mon = user_data;

    struct zmk_watchdog_incident_record rec = {0};
    rec.type = ZMK_WATCHDOG_INCIDENT_FREEZE;
    rec.uptime_s = (uint32_t)(k_uptime_get() / 1000);
    rec.detail.freeze.channel_id = (uint8_t)channel_id;
    strncpy(rec.detail.freeze.queue_name, mon->name, sizeof(rec.detail.freeze.queue_name) - 1);
#if defined(CONFIG_CPU_CORTEX_M)
    freeze_capture_thread(k_work_queue_thread_get(mon->get_queue()), &rec.detail.freeze);
#endif

    zmk_watchdog_pending_set(&rec);
    zmk_watchdog_reboot();
}

static void watchdog_freeze_feed_work_handler(struct k_work *work) {
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct watchdog_freeze_monitor *mon =
        CONTAINER_OF(dwork, struct watchdog_freeze_monitor, feed_work);

    task_wdt_feed(mon->channel_id);

    k_work_reschedule_for_queue(
        mon->get_queue(), &mon->feed_work,
        K_MSEC(CONFIG_ZMK_WATCHDOG_FREEZE_TIMEOUT_MS / CONFIG_ZMK_WATCHDOG_FREEZE_FEED_DIVISOR));
}

static struct k_work_delayable watchdog_freeze_arm_work;

/* Arms task_wdt and starts the feed timers. Runs on the system workqueue,
 * CONFIG_ZMK_WATCHDOG_FREEZE_STARTUP_DELAY_MS after boot rather than
 * synchronously inside SYS_INIT -- see that option's Kconfig help for why
 * (avoids adding this module's own periodic timer/work activity during the
 * initial BLE connection-establishment window, when the radio schedule is
 * busiest). */
static void watchdog_freeze_arm_work_handler(struct k_work *work) {
    /* Software-only task watchdog: no hardware watchdog peripheral backs
     * this. See DESIGN.md SS4.1 for why a hardware fallback was prototyped
     * and then deliberately removed. */
    int ret = task_wdt_init(NULL);
    if (ret < 0) {
        LOG_ERR("task_wdt_init failed: %d", ret);
        return;
    }

    for (size_t i = 0; i < ARRAY_SIZE(watchdog_freeze_monitors); i++) {
        struct watchdog_freeze_monitor *mon = &watchdog_freeze_monitors[i];

        int channel_id =
            task_wdt_add(CONFIG_ZMK_WATCHDOG_FREEZE_TIMEOUT_MS, watchdog_freeze_channel_fired, mon);
        if (channel_id < 0) {
            LOG_ERR("task_wdt_add failed for queue '%s': %d", mon->name, channel_id);
            return;
        }
        mon->channel_id = channel_id;

        k_work_init_delayable(&mon->feed_work, watchdog_freeze_feed_work_handler);
        k_work_schedule_for_queue(
            mon->get_queue(), &mon->feed_work,
            K_MSEC(CONFIG_ZMK_WATCHDOG_FREEZE_TIMEOUT_MS / CONFIG_ZMK_WATCHDOG_FREEZE_FEED_DIVISOR));

        LOG_INF("Watchdog freeze monitor armed: queue='%s' channel=%d timeout=%dms", mon->name,
                channel_id, CONFIG_ZMK_WATCHDOG_FREEZE_TIMEOUT_MS);
    }
}

static int watchdog_freeze_init(void) {
    /* Both monitored queues (system workqueue, ZMK low-prio workqueue) are
     * started well before this by their own earlier-level SYS_INIT hooks, so
     * scheduling onto either is safe immediately -- only *arming* is
     * deferred. */
    k_work_init_delayable(&watchdog_freeze_arm_work, watchdog_freeze_arm_work_handler);
    k_work_schedule(&watchdog_freeze_arm_work, K_MSEC(CONFIG_ZMK_WATCHDOG_FREEZE_STARTUP_DELAY_MS));
    return 0;
}

SYS_INIT(watchdog_freeze_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

void zmk_watchdog_freeze_disarm_sysworkq_channel_for_test(void) {
    /* "sysworkq" is always watchdog_freeze_monitors[0] -- see the table
     * above. task_wdt_delete() is safe to call from ordinary thread
     * context. */
    task_wdt_delete(watchdog_freeze_monitors[0].channel_id);
}
