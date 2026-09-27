/*
 * Copyright (c) 2026 cormoran
 *
 * SPDX-License-Identifier: MIT
 */

#include <stdint.h>
#include <stdio.h>

#include <zephyr/sys/util.h>

#include <cormoran/watchdog/watchdog.pb.h>
#include <cormoran/zmk/watchdog.h>
#include <cormoran/zmk/watchdog_request_exec.h>

/*
 * WATCHDOG_RPC_PAGE_SIZE must match watchdog.options'
 * IncidentPageResponse.incidents max_count (4) -- a pure UX choice
 * (DESIGN.md SS7.1/SS7.2): the split relay path streams one incident per
 * relay event rather than embedding a whole page in one relay event, so
 * this is no longer constrained by the split relay transport's 255-byte
 * hard ceiling, nor by CONFIG_ZMK_STUDIO_RPC_TX_BUF_SIZE (a genuine
 * streaming ring buffer that never needs a whole response to fit in one
 * buffer either -- see src/studio/watchdog_handler.c).
 */
#define WATCHDOG_RPC_PAGE_SIZE 4

void watchdog_incident_record_to_proto(uint16_t id, uint32_t source,
                                       const struct zmk_watchdog_incident_record *rec,
                                       cormoran_watchdog_Incident *out) {
    *out = (cormoran_watchdog_Incident)cormoran_watchdog_Incident_init_zero;

    out->id = id;
    out->source = source;
    out->boot_ordinal = rec->boot_ordinal;
    out->uptime_s = rec->uptime_s;

    switch (rec->type) {
    case ZMK_WATCHDOG_INCIDENT_FREEZE:
        out->type = cormoran_watchdog_IncidentType_FREEZE;
        out->which_detail = cormoran_watchdog_Incident_freeze_tag;
        out->detail.freeze.channel_id = rec->detail.freeze.channel_id;
        snprintf(out->detail.freeze.queue_name, sizeof(out->detail.freeze.queue_name), "%s",
                 rec->detail.freeze.queue_name);
        out->detail.freeze.thread_state = rec->detail.freeze.thread_state;
        out->detail.freeze.pended_on = rec->detail.freeze.pended_on;
        out->detail.freeze.frames_count =
            MIN(rec->detail.freeze.frame_count, ARRAY_SIZE(out->detail.freeze.frames));
        for (pb_size_t i = 0; i < out->detail.freeze.frames_count; i++) {
            out->detail.freeze.frames[i] = rec->detail.freeze.frames[i];
        }
        break;
    case ZMK_WATCHDOG_INCIDENT_FATAL:
        out->type = cormoran_watchdog_IncidentType_FATAL;
        out->which_detail = cormoran_watchdog_Incident_fatal_tag;
        out->detail.fatal.reason = rec->detail.fatal.reason;
        out->detail.fatal.pc = rec->detail.fatal.pc;
        out->detail.fatal.lr = rec->detail.fatal.lr;
        snprintf(out->detail.fatal.thread_name, sizeof(out->detail.fatal.thread_name), "%s",
                 rec->detail.fatal.thread_name);
        break;
    case ZMK_WATCHDOG_INCIDENT_RESET_CAUSE:
    default:
        out->type = cormoran_watchdog_IncidentType_RESET_CAUSE;
        out->which_detail = cormoran_watchdog_Incident_reset_tag;
        out->detail.reset.cause_bits = rec->detail.reset.cause_bits;
        break;
    }
}

static void set_error(cormoran_watchdog_Response *resp, const char *message) {
    cormoran_watchdog_ErrorResponse err = cormoran_watchdog_ErrorResponse_init_zero;
    snprintf(err.message, sizeof(err.message), "%s", message);
    resp->which_response_type = cormoran_watchdog_Response_error_tag;
    resp->response_type.error = err;
}

static void handle_get_status(cormoran_watchdog_Response *resp) {
    cormoran_watchdog_StatusResponse status = cormoran_watchdog_StatusResponse_init_zero;
    status.capacity = zmk_watchdog_store_capacity();
    status.stored = zmk_watchdog_store_count();
    status.dropped_since_boot = zmk_watchdog_store_dropped_since_boot();
    status.recording_stopped = zmk_watchdog_store_recording_stopped();

    resp->which_response_type = cormoran_watchdog_Response_status_tag;
    resp->response_type.status = status;
}

static void handle_list_incidents(const cormoran_watchdog_ListIncidentsRequest *req,
                                  cormoran_watchdog_Response *resp) {
    cormoran_watchdog_IncidentPageResponse page = cormoran_watchdog_IncidentPageResponse_init_zero;

    uint16_t total = zmk_watchdog_store_count();
    uint32_t start = req->start_index;

    page.total = total;
    page.start_index = start;
    page.incidents_count = 0;

    for (uint32_t i = start; i < (uint32_t)total && page.incidents_count < WATCHDOG_RPC_PAGE_SIZE;
         i++) {
        struct zmk_watchdog_incident_record rec;
        uint16_t id;
        int ret = zmk_watchdog_store_get_with_id((uint16_t)i, &rec, &id);
        if (ret < 0) {
            /* Store mutated concurrently (delete) between count() and
             * get_with_id() -- stop the page here rather than erroring the
             * whole request; the client will see a shorter page than
             * `total` implied and can re-request if it cares. */
            break;
        }

        /* source == 0: this function only ever runs against *this half's
         * own* store -- the local RPC handler (already source 0) and the
         * relay peripheral responder (which stamps the real source back in
         * on the central side once the response relays back, see
         * src/split/watchdog_relay.c) both want 0 here. */
        watchdog_incident_record_to_proto(id, 0, &rec, &page.incidents[page.incidents_count]);
        page.incidents_count++;
    }

    resp->which_response_type = cormoran_watchdog_Response_incident_page_tag;
    resp->response_type.incident_page = page;
}

static void handle_delete_incidents(const cormoran_watchdog_DeleteIncidentsRequest *req,
                                    cormoran_watchdog_Response *resp) {
    uint32_t deleted = 0;

    if (req->all) {
        uint16_t count_before = zmk_watchdog_store_count();
        int ret = zmk_watchdog_store_delete_all();
        if (ret < 0) {
            set_error(resp, "Failed to delete all incidents");
            return;
        }
        deleted = count_before;
    } else {
        for (size_t i = 0; i < req->ids_count; i++) {
            uint32_t id = req->ids[i];
            if (id > UINT16_MAX) {
                continue;
            }
            int ret = zmk_watchdog_store_delete((uint16_t)id);
            if (ret == 0) {
                deleted++;
            }
        }
    }

    cormoran_watchdog_DeleteResultResponse result =
        cormoran_watchdog_DeleteResultResponse_init_zero;
    result.deleted = deleted;

    resp->which_response_type = cormoran_watchdog_Response_delete_result_tag;
    resp->response_type.delete_result = result;
}

#if IS_ENABLED(CONFIG_ZMK_WATCHDOG_TEST_INJECTION)
/*
 * Dangerous by design -- only compiled in when CONFIG_ZMK_WATCHDOG_TEST_INJECTION=y
 * (default n). See DESIGN.md SS4.4 and src/watchdog_inject.c.
 *
 *  - FREEZE_KIND: zmk_watchdog_inject_freeze() only submits a k_work to the
 *    system workqueue and returns immediately -- the actual freeze (and the
 *    eventual reboot once CONFIG_ZMK_WATCHDOG_FREEZE_TIMEOUT_MS elapses)
 *    happens asynchronously, well after this function returns and the
 *    InjectAckResponse below has been handed back to the RPC transport. Safe
 *    to call from the RPC handler's own thread context.
 *  - FATAL_KIND: zmk_watchdog_inject_fatal() calls k_oops() synchronously, in
 *    this same call stack -- it does not return, and the device reboots
 *    almost immediately. The InjectAckResponse this function fills in is
 *    typically never actually encoded/sent: the fatal-error handler's own
 *    zmk_watchdog_reboot() call happens first. That is expected, not a bug
 *    (see the proto doc comment on InjectTestRequest).
 */
static void handle_inject_test(const cormoran_watchdog_InjectTestRequest *req,
                               cormoran_watchdog_Response *resp) {
    switch (req->kind) {
    case cormoran_watchdog_InjectTestKind_FATAL_KIND:
        zmk_watchdog_inject_fatal();
        break;
    case cormoran_watchdog_InjectTestKind_FREEZE_KIND:
    default:
        zmk_watchdog_inject_freeze();
        break;
    }

    cormoran_watchdog_InjectAckResponse ack = cormoran_watchdog_InjectAckResponse_init_zero;
    ack.kind = req->kind;
    resp->which_response_type = cormoran_watchdog_Response_inject_ack_tag;
    resp->response_type.inject_ack = ack;
}
#endif /* CONFIG_ZMK_WATCHDOG_TEST_INJECTION */

void watchdog_request_exec_handle(const cormoran_watchdog_Request *req,
                                  cormoran_watchdog_Response *resp) {
    *resp = (cormoran_watchdog_Response)cormoran_watchdog_Response_init_zero;

    switch (req->which_request_type) {
    case cormoran_watchdog_Request_get_status_tag:
        handle_get_status(resp);
        break;
    case cormoran_watchdog_Request_list_incidents_tag:
        handle_list_incidents(&req->request_type.list_incidents, resp);
        break;
    case cormoran_watchdog_Request_delete_incidents_tag:
        handle_delete_incidents(&req->request_type.delete_incidents, resp);
        break;
    case cormoran_watchdog_Request_inject_test_tag:
#if IS_ENABLED(CONFIG_ZMK_WATCHDOG_TEST_INJECTION)
        handle_inject_test(&req->request_type.inject_test, resp);
#else
        /* The proto tag exists unconditionally for a stable wire format
         * (DESIGN.md SS4.4), but a non-test-injection build must never
         * actually freeze/fault itself just because a client asked --
         * answer with the same error a genuinely unsupported request kind
         * would get. */
        set_error(resp, "Unsupported or missing watchdog request");
#endif
        break;
    default:
        set_error(resp, "Unsupported or missing watchdog request");
        break;
    }
}
