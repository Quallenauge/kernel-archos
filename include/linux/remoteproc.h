/*
 * Remote Processor Framework
 *
 * Copyright(c) 2011 Texas Instruments. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * * Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in
 *   the documentation and/or other materials provided with the
 *   distribution.
 * * Neither the name Texas Instruments nor the names of its
 *   contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef REMOTEPROC_H
#define REMOTEPROC_H

#include <linux/mutex.h>
#include <linux/completion.h>
#include <linux/workqueue.h>
#include <linux/notifier.h>
#include <linux/pm_qos.h>

#include <linux/virtio.h>

/* Must match the BIOS version embeded in the BIOS firmware image */
#define RPROC_BIOS_VERSION	2

/* Maximum number of entries that can be added for lookup */
#define RPROC_MAX_MEM_ENTRIES	20

/**
 * The following enums and structures define the binary format of the images
 * we load and run the remote processors with.
 *
 * The binary format is as follows:
 *
 * struct {
 *     char magic[4] = { 'R', 'P', 'R', 'C' };
 *     u32 version;
 *     u32 header_len;
 *     char header[...] = { header_len bytes of unformatted, textual header };
 *     struct section {
 *         u32 type;
 *         u64 da;
 *         u32 len;
 *         u8 content[...] = { len bytes of binary data };
 *     } [ no limit on number of sections ];
 * } __packed;
 */
struct fw_header {
	char magic[4];
	u32 version;
	u32 header_len;
	char header[0];
} __packed;

struct fw_section {
	u32 type;
	u64 da;
	u32 len;
	char content[0];
} __packed;

enum fw_section_type {
	FW_RESOURCE	= 0,
	FW_TEXT		= 1,
	FW_DATA		= 2,
	FW_MMU		= 3,
	FW_SIGNATURE	= 4,
};

struct fw_resource {
	u32 type;
	u64 da;
	u64 pa;
	u32 len;
	u32 reserved;
	u8 name[48];
} __packed;

enum fw_resource_type {
	RSC_CARVEOUT	= 0,
	RSC_DEVMEM	= 1,
	RSC_DEVICE	= 2,
	RSC_IRQ		= 3,
	RSC_TRACE	= 4,
	RSC_BOOTADDR	= 5,
	RSC_CRASHDUMP	= 6,
	RSC_SUSPENDADDR	= 7,
	RSC_END		= 8,
};

/**
 * struct rproc_mem_pool - descriptor for the rproc's contiguous memory pool data
 *
 * @mem_base: starting physical address of the dynamic pool
 * @mem_size: size of the initial dynamic pool
 * @cur_base: current available physical address in the pool
 * @cur_size: remaining available memory in the pool
 * @st_base:  starting physical address of the static pool
 * @st_size:  size of the static pool
 */
struct rproc_mem_pool {
	phys_addr_t mem_base;
	u32 mem_size;
	phys_addr_t cur_base;
	u32 cur_size;
	phys_addr_t st_base;
	u32 st_size;
};

/**
 * struct rproc_mem_entry - descriptor of a remote memory region
 *
 * @da:		virtual address as seen by the device (aka device address)
 * @pa:		physical address
 * @size:	size of this memory region
 */
struct rproc_mem_entry {
	u64 da;
	phys_addr_t pa;
	u32 size;
	bool core;
};

enum rproc_constraint {
	RPROC_CONSTRAINT_SCALE,
	RPROC_CONSTRAINT_LATENCY,
	RPROC_CONSTRAINT_BANDWIDTH,

	RPROC_CONSTRAINT_FREQUENCY,
};

/*
 * enum - Predefined Mailbox Messages
 *
 * @RP_MBOX_READY: informs the M3's that we're up and running. this is
 * part of the init sequence sent that the M3 expects to see immediately
 * after it is booted.
 *
 * @RP_MBOX_PENDING_MSG: informs the receiver that there is an inbound
 * message waiting in its own receive-side vring. please note that currently
 * this message is optional: alternatively, one can explicitly send the index
 * of the triggered virtqueue itself. the preferred approach will be decided
 * as we progress and experiment with those two different approaches.
 *
 * @RP_MBOX_CRASH: this message is sent if BIOS crashes
 *
 * @RP_MBOX_ECHO_REQUEST: a mailbox-level "ping" message.
 *
 * @RP_MBOX_ECHO_REPLY: a mailbox-level reply to a "ping"
 *
 * @RP_MBOX_ABORT_REQUEST: a "please crash" request, used for testing the
 * recovery mechanism (to some extent).
 *
 * @RP_MBOX_SUSPEND: suspend request for the remote processor
 *
 * @RP_MBOX_SUSPEND_FORCED: forced suspend for system suspend request
 *
 * @RP_MBOX_SUSPEND_ACK: remote processor acked suspend request
 *
 * @P_MBOX_SUSPEND_CANCEL: remote processor canceled suspend request
 *
 * Intoduce new message definitions if any here.
 *
 * @RP_MBOX_END_MSG: Indicates end of known/defined messages from remote core
 * This should be the last definition.
 *
 */
enum omap_rp_mbox_messages {
	RP_MBOX_READY		= 0xFFFFFF00,
	RP_MBOX_PENDING_MSG	= 0xFFFFFF01,
	RP_MBOX_CRASH		= 0xFFFFFF02,
	RP_MBOX_ECHO_REQUEST	= 0xFFFFFF03,
	RP_MBOX_ECHO_REPLY	= 0xFFFFFF04,
	RP_MBOX_ABORT_REQUEST	= 0xFFFFFF05,
	RP_MBOX_SUSPEND		= 0xFFFFFF10,
	RP_MBOX_SUSPEND_FORCED	= 0xFFFFFF11,
	RP_MBOX_SUSPEND_ACK	= 0xFFFFFF12,
	RP_MBOX_SUSPEND_CANCEL	= 0xFFFFFF13,
	RP_MBOX_END_MSG		= 0xFFFFFF14,

	RP_MSG_BOOTINIT_DONE	= 0xFFFFFF08,
};

struct rproc;

struct rproc_ops {
	int (*start)(struct rproc *rproc, u64 bootaddr);
	int (*stop)(struct rproc *rproc);
	int (*suspend)(struct rproc *rproc, bool force);
	int (*resume)(struct rproc *rproc);
	int (*iommu_init)(struct rproc *, int (*)(struct rproc *, u64, u32));
	int (*iommu_exit)(struct rproc *);
	int (*set_lat)(struct rproc *rproc, long v);
	int (*set_bw)(struct rproc *rproc, long v);
	int (*scale)(struct rproc *rproc, long v);
	int (*watchdog_init)(struct rproc *, int (*)(struct rproc *));
	int (*watchdog_exit)(struct rproc *);
	void (*dump_registers)(struct rproc *);
	int (*pm_init)(struct rproc *rproc, u64 suspaddr);
};

/*
 * enum rproc_state - remote processor states
 *
 * @RPROC_OFFLINE: needs firmware load and init to exit this state.
 *
 * @RPROC_SUSPENDED: needs to be woken up to receive a message.
 *
 * @RPROC_RUNNING: up and running.
 *
 * @RPROC_LOADING: asynchronous firmware loading has started
 *
 * @RPROC_CRASHED: needs to be logged, connections torn down, resources
 * released, and returned to OFFLINE.
 */
enum rproc_state {
	RPROC_OFFLINE,
	RPROC_SUSPENDED,
	RPROC_RUNNING,
	RPROC_LOADING,
	RPROC_CRASHED,
};

/*
 * enum rproc_event - remote processor events
 *
 * @RPROC_ERROR: Fatal error has happened on the remote processor.
 *
 * @RPROC_PRE_SUSPEND: users can register for that event in order to cancel
 *		       autosuspend, they just need to return an error in the
 *		       callback function.
 *
 * @RPROC_POS_SUSPEND: users can register for that event in order to release
 *		       resources not needed when the remote processor is
 *		       sleeping or if they need to save some context.
 *
 * @RPROC_RESUME: users should use this event to revert what was done in the
 *		  POS_SUSPEND event.
 *
 * @RPROC_SECURE: remote processor secure mode has changed.
 *
 * @RPROC_LOAD_ERROR: an error has occurred during loading the remote processor
 *                    binary. users can use this event to release any resources
 *                    acquired after a request to start the processor.
 *
 * @RPROC_PRELOAD: users can register for this event to perform any actions
 *                 before the remoteproc starts loading the binary into memory.
 */
enum rproc_event {
	RPROC_ERROR,
	RPROC_PRE_SUSPEND,
	RPROC_POS_SUSPEND,
	RPROC_RESUME,
	RPROC_SECURE,
	RPROC_LOAD_ERROR,
	RPROC_PRELOAD,
};

#define RPROC_MAX_NAME	100

/*
 * struct rproc - a physical remote processor device
 *
 * @next: next rproc entry in the list
 * @name: human readable name of the rproc, cannot exceed RPROC_MAX_NAME bytes
 * @memory_maps: table of da-to-pa memory maps (relevant if device is behind
 *               an iommu)
 * @memory_pool: platform-specific contiguous memory pool data (relevant for
 *               allocating memory needed for the remote processor image)
 * @firmware: name of firmware file to be loaded
 * @owner: reference to the platform-specific rproc module
 * @priv: private data which belongs to the platform-specific rproc module
 * @ops: platform-specific start/stop rproc handlers
 * @dev: reference to the platform-specific rproc dev
 * @count: usage refcount
 * @state: rproc_state enum value representing the state of the device
 * @lock: lock which protects concurrent manipulations of the rproc
 * @dbg_dir: debugfs directory of this rproc device
 * @trace_buf0: main trace buffer of the remote processor
 * @trace_buf1: second, optional, trace buffer of the remote processor
 * @trace_len0: length of main trace buffer of the remote processor
 * @trace_len1: length of the second (and optional) trace buffer
 * @cdump_buf0: main exception/crash dump buffer of the remote processor
 * @cdump_buf1: second exception/crash dump buffer of the remote processor
 * @cdump_len0: length of main crash dump buffer of the remote processor
 * @cdump_len1: length of the second (and optional) crash dump buffer
 * @firmware_loading_complete: flags e/o asynchronous firmware loading
 * @mmufault_work: work in charge of notifing mmufault
 * @nb_error: notify block for fatal errors
 * @error_comp: completion used when an error happens
 * @secure_ttb: private data for configuring iommu in secure mode
 * @secure_restart: completion event notifier for the secure restart process
 * @secure_mode: flag to dictate whether to enable secure loading
 * @secure_ok: restart status flag to be looked up upon the event's completion
 * @secure_reset: flag to uninstall the firewalls
 */
struct rproc {
	struct list_head next;
	const char *name;
	struct rproc_mem_entry memory_maps[RPROC_MAX_MEM_ENTRIES];
	struct rproc_mem_pool *memory_pool;
	const char *firmware;
	struct module *owner;
	void *priv;
	const struct rproc_ops *ops;
	struct device *dev;
	int count;
	int state;
	struct mutex lock;
	struct dentry *dbg_dir;
	char *trace_buf0, *trace_buf1;
	char *last_trace_buf0, *last_trace_buf1;
	int trace_len0, trace_len1;
	int last_trace_len0, last_trace_len1;
	void *cdump_buf0, *cdump_buf1;
	int cdump_len0, cdump_len1;
	struct mutex tlock;
	struct completion firmware_loading_complete;
	struct work_struct error_work;
	struct blocking_notifier_head nbh;
	struct completion error_comp;
#ifdef CONFIG_REMOTEPROC_AUTOSUSPEND
	unsigned sus_timeout;
	bool force_suspend;
	bool need_resume;
	struct mutex pm_lock;
#endif
	struct pm_qos_request *qos_request;
	void *secure_ttb;
	struct completion secure_restart;
	struct mutex secure_lock;
	bool secure_mode;
	bool secure_ok;
	bool secure_reset;
	bool halt_on_crash;
	char *header;
	int header_len;
};

int rproc_set_secure(const char *, bool);
struct rproc *rproc_get(const char *);
void rproc_put(struct rproc *);
int rproc_event_register(struct rproc *, struct notifier_block *);
int rproc_event_unregister(struct rproc *, struct notifier_block *);
int rproc_register(struct device *, const char *, const struct rproc_ops *,
		const char *, struct rproc_mem_pool *, struct module *,
		unsigned int timeout);
int rproc_unregister(const char *);
void rproc_last_busy(struct rproc *);
int rproc_da_to_pa(struct rproc *, u64, phys_addr_t *);
int rproc_pa_to_da(struct rproc *, phys_addr_t, u64 *);
#ifdef CONFIG_REMOTEPROC_AUTOSUSPEND
extern const struct dev_pm_ops rproc_gen_pm_ops;
#define GENERIC_RPROC_PM_OPS	(&rproc_gen_pm_ops)
#else
#define GENERIC_RPROC_PM_OPS	NULL
#endif
int rproc_set_constraints(struct rproc *, enum rproc_constraint type, long v);
int rproc_error_notify(struct rproc *rproc);


/**
 * enum rproc_err - remote processor errors
 * @RPROC_ERR_MMUFAULT:		iommu fault error
 * @RPROC_ERR_EXCEPTION:	generic device exception
 * @RPROC_ERR_WATCHDOG:		rproc watchdog error
 *
 * Each element of the enum is used as an array index. So, the value of
 * the elements should be always something sane.
 */
enum rproc_err {
	RPROC_ERR_MMUFAULT	= 0,
	RPROC_ERR_EXCEPTION	= 1,
	RPROC_ERR_WATCHDOG	= 2,
};


/* we currently support only two vrings per rvdev */
#define RVDEV_NUM_VRINGS 2

/**
 * struct rproc_vring - remoteproc vring state
 * @va:	virtual address
 * @dma: dma address
 * @len: length, in bytes
 * @da: device address
 * @align: vring alignment
 * @notifyid: rproc-specific unique vring index
 * @rvdev: remote vdev
 * @vq: the virtqueue of this vring
 */
struct rproc_vring {
	void *va;
	dma_addr_t dma;
	int len;
	u32 da;
	u32 align;
	int notifyid;
	struct rproc_vdev *rvdev;
	struct virtqueue *vq;
};

/**
 * struct rproc_vdev - remoteproc state for a supported virtio device
 * @node: list node
 * @rproc: the rproc handle
 * @vdev: the virio device
 * @vring: the vrings for this vdev
 * @dfeatures: virtio device features
 * @gfeatures: virtio guest features
 */
struct rproc_vdev {
	struct list_head node;
	struct rproc *rproc;
	struct virtio_device vdev;
	struct rproc_vring vring[RVDEV_NUM_VRINGS];
	unsigned long dfeatures;
	unsigned long gfeatures;
};

static inline struct rproc_vdev *vdev_to_rvdev(struct virtio_device *vdev)
{
	return container_of(vdev, struct rproc_vdev, vdev);
}

static inline struct rproc *vdev_to_rproc(struct virtio_device *vdev)
{
	struct rproc_vdev *rvdev = vdev_to_rvdev(vdev);

	return rvdev->rproc;
}

extern int omap_rproc_stop(struct rproc *rproc);

#endif /* REMOTEPROC_H */
