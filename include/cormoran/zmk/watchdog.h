/*
 * Copyright (c) 2026 cormoran
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <zephyr/toolchain.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Incident record shared by the store, the retained-RAM pending slot, and
 * (in a later phase) the Studio RPC / split relay layers. Keep this struct:
 *  - fixed-size and packed (no padding-dependent layout assumptions),
 *  - <= 64 bytes (must fit CONFIG_ZMK_SPLIT_RELAY_EVENT_DATA_LEN with
 *    headroom, see DESIGN.md SS6-SS7),
 *  - free of 64-bit fields (nanopb + CONFIG_NANOPB_WITHOUT_64BIT forbids
 *    them once this is exposed over Studio RPC).
 */

#define ZMK_WATCHDOG_RECORD_VERSION 1

/* Incident kind discriminator for zmk_watchdog_incident_record.type. */
enum zmk_watchdog_incident_type {
    ZMK_WATCHDOG_INCIDENT_FREEZE = 0,
    ZMK_WATCHDOG_INCIDENT_FATAL = 1,
    ZMK_WATCHDOG_INCIDENT_RESET_CAUSE = 2,
};

#define ZMK_WATCHDOG_QUEUE_NAME_LEN 16
#define ZMK_WATCHDOG_THREAD_NAME_LEN 16

/* How many code addresses a freeze record keeps (see frames[] below). */
#define ZMK_WATCHDOG_FREEZE_FRAMES 6

struct zmk_watchdog_incident_freeze_detail {
    /* task_wdt channel id that fired. */
    uint8_t channel_id;
    /* The queue thread's kernel thread_state bits when the freeze was
     * detected (pending on an object, sleeping, ready but not running...).
     * 0 in records written before this was captured. */
    uint8_t thread_state;
    /* How many entries of frames[] are filled; 0 when nothing was captured
     * (older records, or a non-Cortex-M build). */
    uint8_t frame_count;
    uint8_t _reserved;
    /* Name of the monitored work queue, NUL-terminated, truncated if needed. */
    char queue_name[ZMK_WATCHDOG_QUEUE_NAME_LEN];
    /* The wait queue the thread was pended on -- the address of the kernel
     * object it was waiting for (a mutex, a semaphore...), which an ELF's
     * symbol table names. 0 when it was not waiting on anything. */
    uint32_t pended_on;
    /* Where the queue's thread was stuck: frames[0] is the PC and frames[1]
     * the LR from its saved context, then return addresses found on its
     * stack, innermost first. The stack part is a scan, not an unwind: a
     * word counts when it points just after a BL/BLX in .text, so an old
     * return address left on the stack can appear. Read it as "these
     * functions are on the way", which is what locating a freeze needs. */
    uint32_t frames[ZMK_WATCHDOG_FREEZE_FRAMES];
} __packed;

struct zmk_watchdog_incident_fatal_detail {
    /* One of Zephyr's K_ERR_* reason codes (unsigned int truncated to u32). */
    uint32_t reason;
    uint32_t pc;
    uint32_t lr;
    /* k_thread_name_get() of the faulting thread, "?" if unavailable. */
    char thread_name[ZMK_WATCHDOG_THREAD_NAME_LEN];
} __packed;

struct zmk_watchdog_incident_reset_detail {
    /* Raw hwinfo_get_reset_cause() bit mask. */
    uint32_t cause_bits;
} __packed;

union zmk_watchdog_incident_detail {
    struct zmk_watchdog_incident_freeze_detail freeze;
    struct zmk_watchdog_incident_fatal_detail fatal;
    struct zmk_watchdog_incident_reset_detail reset;
} __packed;

struct zmk_watchdog_incident_record {
    /* Record format version, for forward compatibility. */
    uint8_t version;
    /* enum zmk_watchdog_incident_type. */
    uint8_t type;
    /* Reserved bit flags, currently unused. */
    uint8_t flags;
    uint8_t _reserved;
    /* Monotonically increasing ordinal, incremented only when an incident
     * is actually persisted (see zmk_watchdog_store_append()). Doubles as
     * a relative "which boot" indicator since there is no RTC. */
    uint32_t boot_ordinal;
    /* Uptime (seconds) at incident time. Distinguishes incidents within the
     * same boot_ordinal. */
    uint32_t uptime_s;
    union zmk_watchdog_incident_detail detail;
} __packed;

/* Hard limit: must fit a split relay event payload (DESIGN.md SS6-SS7),
 * i.e. <= CONFIG_ZMK_SPLIT_RELAY_EVENT_DATA_LEN (default 128) with
 * headroom for the relay event envelope. Also keeps well under the
 * "target <= 64 bytes" design goal. */
BUILD_ASSERT(sizeof(struct zmk_watchdog_incident_record) <= 64,
             "zmk_watchdog_incident_record grew past the 64-byte target size");

/*
 * ---------------------------------------------------------------------
 * Store: settings-backed slot store (src/watchdog_store.c).
 *
 * Slots 0..CONFIG_ZMK_WATCHDOG_MAX_INCIDENTS-1, keyed "wdg/i/<slot>" in the
 * Zephyr settings subsystem. append() never overwrites: once every slot is
 * used the incident is dropped (only zmk_watchdog_store_dropped_since_boot()
 * increments) until the user deletes something. No flash write happens on
 * every boot; boot_ordinal bookkeeping piggybacks on the same flash write
 * as the incident it orders (see DESIGN.md SS6).
 * ---------------------------------------------------------------------
 */

/* Append a new incident. Picks the first free slot and persists it (plus
 * the "wdg/meta" boot_ordinal bookkeeping) via the settings subsystem.
 * Fills rec->boot_ordinal with the ordinal actually assigned.
 * Returns 0 if stored, -ENOSPC if the cap was reached (incident dropped,
 * only the dropped-since-boot counter is incremented; no flash write). */
int zmk_watchdog_store_append(struct zmk_watchdog_incident_record *rec);

/* Delete the incident with the given id. Returns 0 on success, -ENOENT if
 * no incident with that id is currently stored. */
int zmk_watchdog_store_delete(uint16_t id);

/* Delete every stored incident and resume recording (clears the
 * recording-stopped condition). Returns 0 on success (including when the
 * store was already empty). */
int zmk_watchdog_store_delete_all(void);

/* Read the stored incident at the given enumeration index (0-based, dense
 * over currently-used slots, order is unspecified but stable between
 * mutations). Returns 0 and fills *out on success, -ENOENT if index is out
 * of range. Intended for RPC listing in a later phase. */
int zmk_watchdog_store_get(uint16_t index, struct zmk_watchdog_incident_record *out);

/* Look up a stored incident by its id. Returns 0 and fills *out on success,
 * -ENOENT if no incident with that id is currently stored. */
int zmk_watchdog_store_get_by_id(uint16_t id, struct zmk_watchdog_incident_record *out);

/* Like zmk_watchdog_store_get(), but also hands back the RPC-visible id the
 * incident was assigned when appended (the same id zmk_watchdog_store_get_by_id()
 * and zmk_watchdog_store_delete() take). Added for the Studio RPC listing
 * handler (src/studio/watchdog_handler.c), which needs the id to let the web
 * UI delete individual incidents by id. Returns 0 and fills out/id_out on
 * success, -ENOENT if index is out of range. */
int zmk_watchdog_store_get_with_id(uint16_t index, struct zmk_watchdog_incident_record *out,
                                   uint16_t *id_out);

/* Maximum number of incidents the store can hold (CONFIG_ZMK_WATCHDOG_MAX_INCIDENTS). */
uint16_t zmk_watchdog_store_capacity(void);

/* Number of incidents currently stored. */
uint16_t zmk_watchdog_store_count(void);

/* Number of incidents dropped (cap reached) since this boot. Reset to 0 on
 * every boot; never persisted (that would itself cost a flash write). */
uint32_t zmk_watchdog_store_dropped_since_boot(void);

/* True once the store has reached capacity and append() would drop the next
 * incident. Cleared by zmk_watchdog_store_delete()/delete_all() once a slot
 * frees up. */
bool zmk_watchdog_store_recording_stopped(void);

/*
 * ---------------------------------------------------------------------
 * Pending slot: crossing the reboot (src/watchdog_pending.c).
 *
 * A single retained-RAM (__noinit) slot used to carry one incident record
 * across a sys_reboot(). See DESIGN.md SS5.
 * ---------------------------------------------------------------------
 */

/* Fill the pending slot with rec (magic + CRC) so it survives the upcoming
 * reboot. Safe to call from ISR / fatal-handler context: no flash, no
 * locking, no logging. Callers (freeze/fatal detectors, a later phase) are
 * expected to call zmk_watchdog_reboot() right after. */
void zmk_watchdog_pending_set(const struct zmk_watchdog_incident_record *rec);

/* Boot-time conversion: validate the pending slot's magic+CRC, hand a valid
 * record to the store (append, or count as dropped if the store is full),
 * then zero the slot so it is never reprocessed. No-op (returns 0) if the
 * slot is empty/invalid. Requires the settings subsystem to already be
 * loaded (calls into the store) -- production code must not call this
 * directly from a SYS_INIT level; it is scheduled to run afterwards from
 * the low-priority workqueue. Exposed here so tests can call it directly
 * without relying on that timing. Returns 0 on success (including "nothing
 * pending"), a negative errno if the store append itself failed. */
int zmk_watchdog_pending_convert(void);

/* Test-only helper: flip a bit in the pending slot's stored CRC so the next
 * zmk_watchdog_pending_convert() call takes the "corrupt, discard" path.
 * No-op if no record is pending. Not for production use. */
void zmk_watchdog_pending_corrupt_crc_for_test(void);

/* True if the most recent zmk_watchdog_pending_convert() call found a valid
 * pending record (whether or not the store had room for it) -- i.e. this
 * boot followed a reboot that a freeze/fatal detector itself initiated and
 * already explained. False if the slot was empty/invalid, or
 * zmk_watchdog_pending_convert() has not run yet this boot.
 *
 * Used by the boot-time reset-cause audit (src/watchdog_reset_cause.c) to
 * decide whether a WATCHDOG/CPU_LOCKUP/BROWNOUT hwinfo reset cause is
 * redundant with an incident already recorded by the pending-slot path, or
 * is a "degraded" reset with no detail (hard lockup, HW-WDT-only reset)
 * that deserves its own RESET_CAUSE incident. See DESIGN.md SS4.3. */
bool zmk_watchdog_pending_had_incident_this_boot(void);

/*
 * ---------------------------------------------------------------------
 * Reboot wrapper (src/watchdog_pending.c).
 *
 * Thin wrapper around sys_reboot(SYS_REBOOT_WARM) so native_sim unit tests
 * can intercept it instead of actually tearing down the test process.
 * Production code (freeze/fatal detectors, later phases) must always call
 * this instead of sys_reboot() directly.
 * ---------------------------------------------------------------------
 */

typedef void (*zmk_watchdog_reboot_fn_t)(void);

/* Reboot the device (SYS_REBOOT_WARM), or call the test override installed
 * via zmk_watchdog_reboot_set_override() if one is set. Never returns in
 * production. */
void zmk_watchdog_reboot(void);

/* Test-only hook: replace what zmk_watchdog_reboot() does. Pass NULL to
 * restore the real sys_reboot() behavior. Not for production use. */
void zmk_watchdog_reboot_set_override(zmk_watchdog_reboot_fn_t override);

/*
 * ---------------------------------------------------------------------
 * Fatal-error record builder (src/watchdog_fatal.c), gated behind
 * CONFIG_ZMK_WATCHDOG_FATAL_DETECT.
 *
 * Factored out of the k_sys_fatal_error_handler() override so tests can
 * exercise the record-building logic directly with a synthetic esf, rather
 * than actually faulting native_sim (unsafe/unreliable) or re-registering
 * the weak k_sys_fatal_error_handler symbol.
 * ---------------------------------------------------------------------
 */

struct arch_esf; /* Forward-declared by zephyr/arch/arch_interface.h. */

/* Fill *rec with a FATAL incident describing (reason, esf): reason code,
 * esf->basic.pc/lr on ARM (0 on architectures without that field, e.g.
 * native_sim), and the current thread's name ("?" if CONFIG_THREAD_NAME is
 * off or the name is unset). esf may be NULL. Does not touch the pending
 * slot or reboot -- callers (the real handler, or a test) do that. */
void watchdog_fatal_build_record(struct zmk_watchdog_incident_record *rec, unsigned int reason,
                                 const struct arch_esf *esf);

/*
 * ---------------------------------------------------------------------
 * Test/fault injection (src/watchdog_inject.c), gated behind
 * CONFIG_ZMK_WATCHDOG_TEST_INJECTION (default n -- dangerous, never enable
 * in a production build). Exercises the freeze/fatal detectors end-to-end.
 * No RPC is wired to these yet; a later phase adds that.
 * ---------------------------------------------------------------------
 */

/* Submits a k_work to the system workqueue whose handler calls
 * k_sleep(K_FOREVER), simulating a frozen sysworkq. Combined with
 * CONFIG_ZMK_WATCHDOG_FREEZE_DETECT, the "sysworkq" task_wdt channel will
 * eventually fire (after CONFIG_ZMK_WATCHDOG_FREEZE_TIMEOUT_MS of
 * unanswered feed work). */
void zmk_watchdog_inject_freeze(void);

/* Triggers a kernel oops (k_oops()), exercising
 * CONFIG_ZMK_WATCHDOG_FATAL_DETECT's k_sys_fatal_error_handler override. */
void zmk_watchdog_inject_fatal(void);

/* Test-only helper (src/watchdog_freeze.c, requires
 * CONFIG_ZMK_WATCHDOG_FREEZE_DETECT): stop the task_wdt channel monitoring
 * the system workqueue from ever firing again. Production code never needs
 * this (a real firing reboots the device); it exists so a test that
 * deliberately froze sysworkq via zmk_watchdog_inject_freeze() -- which
 * cannot be unblocked afterwards -- can stop the channel from repeatedly
 * re-firing (and calling zmk_watchdog_reboot() again) for the rest of the
 * test process's life. Not for production use. */
void zmk_watchdog_freeze_disarm_sysworkq_channel_for_test(void);

/*
 * ---------------------------------------------------------------------
 * Store append hook (src/watchdog_store.c): lets an optional listener (the
 * Studio RPC handler, when CONFIG_ZMK_WATCHDOG_STUDIO_RPC is enabled) learn
 * about newly-persisted incidents without the store itself depending on
 * Studio RPC -- keeps the store usable on peripherals / native_sim builds
 * with no RPC subsystem at all (DESIGN.md SS6-SS7: the store is shared by
 * both split roles, only the central has Studio RPC).
 * ---------------------------------------------------------------------
 */

typedef void (*zmk_watchdog_store_appended_cb_t)(uint16_t id,
                                                 const struct zmk_watchdog_incident_record *rec);

/* Registers a callback invoked at the end of a successful
 * zmk_watchdog_store_append() (not called when the incident was dropped due
 * to the cap). Only one listener is supported; pass NULL to unregister. Not
 * called from ISR context -- append() itself never is (see DESIGN.md SS5:
 * detectors go through the retained-RAM pending slot instead). */
void zmk_watchdog_store_set_appended_callback(zmk_watchdog_store_appended_cb_t cb);

#ifdef __cplusplus
}
#endif
