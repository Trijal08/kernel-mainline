/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * No-op stubs for the trusty-ipc tracepoints.  The upstream driver can grow
 * real tracepoints later; nothing here depends on them.
 */
#ifndef _TRUSTY_IPC_TRACE_H
#define _TRUSTY_IPC_TRACE_H

#define trace_trusty_ipc_connect(...)		do { } while (0)
#define trace_trusty_ipc_connect_end(...)	do { } while (0)
#define trace_trusty_ipc_handle_event(...)	do { } while (0)
#define trace_trusty_ipc_poll(...)		do { } while (0)
#define trace_trusty_ipc_read(...)		do { } while (0)
#define trace_trusty_ipc_read_end(...)		do { } while (0)
#define trace_trusty_ipc_rx(...)		do { } while (0)
#define trace_trusty_ipc_write(...)		do { } while (0)

#endif /* _TRUSTY_IPC_TRACE_H */
