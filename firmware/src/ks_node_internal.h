/*
 * ks_node_internal.h - helpers shared by ks_node.c and ks_negotiate.c.
 */
#ifndef KS_NODE_INTERNAL_H
#define KS_NODE_INTERNAL_H

#include "ks/ks_node.h"

bool ks__send(ks_node_t *n, uint8_t type, uint16_t dst, const ks_payload_t *pl);
void ks__log_trial(ks_node_t *n, const ks_log_trial_t *t);
void ks__log_event(ks_node_t *n, uint16_t code, uint16_t arg, float a0, float a1, uint32_t u0,
                   uint32_t u1);
bool ks__alive(const ks_node_t *n, uint16_t id);

/* negotiation (ks_negotiate.c) */
void ks__neg_init(ks_node_t *n);
void ks__neg_epoch(ks_node_t *n, uint32_t epoch);
void ks__neg_tick(ks_node_t *n);
void ks__neg_sample(ks_node_t *n, float p_w);
void ks__neg_abort_all(ks_node_t *n, uint8_t reason_q);
void ks__on_propose(ks_node_t *n, const ks_hdr_t *h, const ks_propose_t *m);
void ks__on_report(ks_node_t *n, const ks_hdr_t *h, const ks_report_t *m);
void ks__on_decide(ks_node_t *n, const ks_hdr_t *h, const ks_decide_t *m);
float ks__neg_offset(const ks_node_t *n);

#endif /* KS_NODE_INTERNAL_H */
