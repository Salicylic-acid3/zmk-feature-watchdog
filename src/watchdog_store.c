/*
 * Copyright (c) 2026 cormoran
 *
 * SPDX-License-Identifier: MIT
 */

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/settings/settings.h>
#include <zephyr/sys/util.h>

#include <cormoran/zmk/watchdog.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#define SETTINGS_SUBTREE "wdg"
#define SETTINGS_META_KEY SETTINGS_SUBTREE "/meta"

/*
 * On-flash slot blob: the incident record plus the RPC-visible id it was
 * assigned when stored. Keeping id in the blob (rather than deriving it
 * from the slot index) means ids survive a delete-then-append that reuses
 * the freed slot with a *different* id, and survive a settings reload.
 */
struct watchdog_store_slot_blob {
    uint16_t id;
    uint16_t _reserved;
    struct zmk_watchdog_incident_record rec;
} __packed;

/* sizeof(struct watchdog_store_slot_blob) before the freeze location was
 * added: 4-byte header + 12-byte record header + 28-byte detail union. */
#define WATCHDOG_STORE_BLOB_V1_SIZE 44

struct watchdog_store_slot {
    bool used;
    struct watchdog_store_slot_blob blob;
};

struct watchdog_store_meta {
    uint32_t next_boot_ordinal;
    uint16_t next_id;
    uint16_t _reserved;
} __packed;

static K_MUTEX_DEFINE(watchdog_store_lock);
static struct watchdog_store_slot slots[CONFIG_ZMK_WATCHDOG_MAX_INCIDENTS];
static struct watchdog_store_meta meta = {.next_boot_ordinal = 1, .next_id = 1};
static uint32_t dropped_since_boot;
static zmk_watchdog_store_appended_cb_t appended_cb;

static int slot_settings_name(uint16_t slot, char *name, size_t name_size) {
    int ret = snprintf(name, name_size, SETTINGS_SUBTREE "/i/%u", (unsigned int)slot);
    if (ret < 0 || (size_t)ret >= name_size) {
        return -ENAMETOOLONG;
    }
    return 0;
}

static int find_free_slot_locked(void) {
    for (size_t i = 0; i < ARRAY_SIZE(slots); i++) {
        if (!slots[i].used) {
            return (int)i;
        }
    }
    return -1;
}

static int find_slot_by_id_locked(uint16_t id) {
    for (size_t i = 0; i < ARRAY_SIZE(slots); i++) {
        if (slots[i].used && slots[i].blob.id == id) {
            return (int)i;
        }
    }
    return -1;
}

/* Dense (0-based) enumeration over currently-used slots, in slot-index
 * order. Order is unspecified by the API contract but stable between
 * mutations, which is all a later RPC "list" phase needs. */
static int find_slot_by_enum_index_locked(uint16_t index) {
    uint16_t seen = 0;
    for (size_t i = 0; i < ARRAY_SIZE(slots); i++) {
        if (!slots[i].used) {
            continue;
        }
        if (seen == index) {
            return (int)i;
        }
        seen++;
    }
    return -1;
}

static uint16_t count_used_locked(void) {
    uint16_t count = 0;
    for (size_t i = 0; i < ARRAY_SIZE(slots); i++) {
        if (slots[i].used) {
            count++;
        }
    }
    return count;
}

int zmk_watchdog_store_append(struct zmk_watchdog_incident_record *rec) {
    if (!rec) {
        return -EINVAL;
    }

    k_mutex_lock(&watchdog_store_lock, K_FOREVER);

    int slot = find_free_slot_locked();
    if (slot < 0) {
        dropped_since_boot++;
        k_mutex_unlock(&watchdog_store_lock);
        LOG_WRN("Watchdog store full (cap=%d); dropping incident (dropped_since_boot=%u)",
                CONFIG_ZMK_WATCHDOG_MAX_INCIDENTS, dropped_since_boot);
        return -ENOSPC;
    }

    rec->version = ZMK_WATCHDOG_RECORD_VERSION;
    rec->boot_ordinal = meta.next_boot_ordinal;

    struct watchdog_store_slot_blob blob = {
        .id = meta.next_id,
        .rec = *rec,
    };

    struct watchdog_store_meta new_meta = {
        .next_boot_ordinal = meta.next_boot_ordinal + 1,
        .next_id = meta.next_id + 1,
    };

    char name[SETTINGS_MAX_NAME_LEN];
    int ret = slot_settings_name((uint16_t)slot, name, sizeof(name));
    if (ret < 0) {
        k_mutex_unlock(&watchdog_store_lock);
        return ret;
    }

    /* Persist the incident first, then the bumped meta (ordinal/id)
     * bookkeeping -- if we crash between the two writes the worst case is
     * a duplicate ordinal/id on the next incident, never data loss. */
    ret = settings_save_one(name, &blob, sizeof(blob));
    if (ret < 0) {
        k_mutex_unlock(&watchdog_store_lock);
        LOG_ERR("Failed to persist watchdog incident: %d", ret);
        return ret;
    }

    ret = settings_save_one(SETTINGS_META_KEY, &new_meta, sizeof(new_meta));
    if (ret < 0) {
        /* Non-fatal: the incident is safely stored; ordinal/id bookkeeping
         * just won't advance until the next successful incident. */
        LOG_WRN("Failed to persist watchdog meta (ordinal bookkeeping): %d", ret);
    } else {
        meta = new_meta;
    }

    slots[slot].used = true;
    slots[slot].blob = blob;

    k_mutex_unlock(&watchdog_store_lock);

    LOG_INF("Watchdog incident stored: id=%u slot=%d type=%u ordinal=%u", blob.id, slot, rec->type,
            rec->boot_ordinal);

    if (appended_cb) {
        appended_cb(blob.id, rec);
    }

    return 0;
}

int zmk_watchdog_store_delete(uint16_t id) {
    k_mutex_lock(&watchdog_store_lock, K_FOREVER);

    int slot = find_slot_by_id_locked(id);
    if (slot < 0) {
        k_mutex_unlock(&watchdog_store_lock);
        return -ENOENT;
    }

    char name[SETTINGS_MAX_NAME_LEN];
    int ret = slot_settings_name((uint16_t)slot, name, sizeof(name));
    if (ret < 0) {
        k_mutex_unlock(&watchdog_store_lock);
        return ret;
    }

    ret = settings_delete(name);
    if (ret < 0 && ret != -ENOENT) {
        k_mutex_unlock(&watchdog_store_lock);
        LOG_ERR("Failed to delete watchdog incident id=%u: %d", id, ret);
        return ret;
    }

    slots[slot].used = false;
    memset(&slots[slot].blob, 0, sizeof(slots[slot].blob));

    k_mutex_unlock(&watchdog_store_lock);

    LOG_INF("Watchdog incident deleted: id=%u slot=%d", id, slot);
    return 0;
}

int zmk_watchdog_store_delete_all(void) {
    k_mutex_lock(&watchdog_store_lock, K_FOREVER);

    int first_error = 0;
    for (size_t i = 0; i < ARRAY_SIZE(slots); i++) {
        if (!slots[i].used) {
            continue;
        }

        char name[SETTINGS_MAX_NAME_LEN];
        int ret = slot_settings_name((uint16_t)i, name, sizeof(name));
        if (ret == 0) {
            ret = settings_delete(name);
        }
        if (ret < 0 && ret != -ENOENT && first_error == 0) {
            first_error = ret;
            LOG_ERR("Failed to delete watchdog incident slot=%zu: %d", i, ret);
            continue;
        }

        slots[i].used = false;
        memset(&slots[i].blob, 0, sizeof(slots[i].blob));
    }

    k_mutex_unlock(&watchdog_store_lock);

    LOG_INF("Watchdog store cleared");
    return first_error;
}

int zmk_watchdog_store_get(uint16_t index, struct zmk_watchdog_incident_record *out) {
    if (!out) {
        return -EINVAL;
    }

    k_mutex_lock(&watchdog_store_lock, K_FOREVER);
    int slot = find_slot_by_enum_index_locked(index);
    if (slot < 0) {
        k_mutex_unlock(&watchdog_store_lock);
        return -ENOENT;
    }
    *out = slots[slot].blob.rec;
    k_mutex_unlock(&watchdog_store_lock);
    return 0;
}

int zmk_watchdog_store_get_by_id(uint16_t id, struct zmk_watchdog_incident_record *out) {
    if (!out) {
        return -EINVAL;
    }

    k_mutex_lock(&watchdog_store_lock, K_FOREVER);
    int slot = find_slot_by_id_locked(id);
    if (slot < 0) {
        k_mutex_unlock(&watchdog_store_lock);
        return -ENOENT;
    }
    *out = slots[slot].blob.rec;
    k_mutex_unlock(&watchdog_store_lock);
    return 0;
}

int zmk_watchdog_store_get_with_id(uint16_t index, struct zmk_watchdog_incident_record *out,
                                   uint16_t *id_out) {
    if (!out || !id_out) {
        return -EINVAL;
    }

    k_mutex_lock(&watchdog_store_lock, K_FOREVER);
    int slot = find_slot_by_enum_index_locked(index);
    if (slot < 0) {
        k_mutex_unlock(&watchdog_store_lock);
        return -ENOENT;
    }
    *out = slots[slot].blob.rec;
    *id_out = slots[slot].blob.id;
    k_mutex_unlock(&watchdog_store_lock);
    return 0;
}

uint16_t zmk_watchdog_store_capacity(void) { return CONFIG_ZMK_WATCHDOG_MAX_INCIDENTS; }

uint16_t zmk_watchdog_store_count(void) {
    k_mutex_lock(&watchdog_store_lock, K_FOREVER);
    uint16_t count = count_used_locked();
    k_mutex_unlock(&watchdog_store_lock);
    return count;
}

uint32_t zmk_watchdog_store_dropped_since_boot(void) { return dropped_since_boot; }

bool zmk_watchdog_store_recording_stopped(void) {
    return zmk_watchdog_store_count() >= CONFIG_ZMK_WATCHDOG_MAX_INCIDENTS;
}

void zmk_watchdog_store_set_appended_callback(zmk_watchdog_store_appended_cb_t cb) {
    appended_cb = cb;
}

/* --------------------------------------------------------------------
 * settings subsystem wiring: rebuild the RAM index (slots[] + meta) from
 * flash during the single global settings_load() call in main(). No
 * separate settings_load_subtree("wdg") call is needed here -- the
 * top-level settings_load() already walks every registered subtree
 * handler, this one included.
 * -------------------------------------------------------------------- */

static int watchdog_settings_set(const char *name, size_t len, settings_read_cb read_cb,
                                 void *cb_arg) {
    const char *next;
    int name_len = settings_name_next(name, &next);

    if (name_len == 4 && strncmp(name, "meta", 4) == 0) {
        struct watchdog_store_meta loaded;
        if (len != sizeof(loaded)) {
            return -EINVAL;
        }
        ssize_t read = read_cb(cb_arg, &loaded, sizeof(loaded));
        if (read < 0) {
            return (int)read;
        }
        k_mutex_lock(&watchdog_store_lock, K_FOREVER);
        meta = loaded;
        k_mutex_unlock(&watchdog_store_lock);
        return 0;
    }

    if (name_len == 1 && strncmp(name, "i", 1) == 0 && next) {
        char *endptr;
        unsigned long slot_index = strtoul(next, &endptr, 10);
        if (*endptr != '\0' || slot_index >= ARRAY_SIZE(slots)) {
            return -ENOENT;
        }

        /* A record written before freeze records carried a location is
         * shorter (the freeze detail grew; the record kept its version,
         * since every new field reads as "not captured" when zero). Load it
         * zero-extended rather than dropping the user's incident history. */
        struct watchdog_store_slot_blob blob = {0};
        if (len > sizeof(blob) || len < WATCHDOG_STORE_BLOB_V1_SIZE) {
            return -EINVAL;
        }
        ssize_t read = read_cb(cb_arg, &blob, len);
        if (read < 0) {
            return (int)read;
        }

        k_mutex_lock(&watchdog_store_lock, K_FOREVER);
        slots[slot_index].used = true;
        slots[slot_index].blob = blob;
        k_mutex_unlock(&watchdog_store_lock);
        return 0;
    }

    return -ENOENT;
}

SETTINGS_STATIC_HANDLER_DEFINE(watchdog, SETTINGS_SUBTREE, NULL, watchdog_settings_set, NULL, NULL);
