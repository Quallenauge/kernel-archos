/*
 * linux/drivers/video/omap2/dsscomp/isr.c
 *
 * DSS Composition file isr queue support
 *
 * Copyright (C) 2011 Archos SA
 * Author: Vladimir Pantelic <pantelic@archos.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/err.h>
#include <linux/slab.h>
#include <linux/list.h>
#include <linux/uaccess.h>

#define MODULE_NAME	"dsscomp"

#include <video/omapdss.h>
#include <video/dsscomp.h>
#include <plat/dsscomp.h>
#include "dsscomp.h"
#include <mach/tiler.h>

int debug_isr;
module_param(debug_isr, int, 0644);

int set_dss_ovl_addr(struct dsscomp_buffer *buf);
int convert_dss_ovl_addr(struct dsscomp_buffer *buf);
int ovl_isr_start(struct dsscomp_dev *cdev);
int ovl_isr_stop(struct dsscomp_dev *cdev);

struct dsscomp_buffer_internal {
	struct dsscomp_buffer b;
	
	/* no user servicable parts down here */
	struct list_head node;
};

static LIST_HEAD(pending_queue);
static LIST_HEAD(done_queue);
static DEFINE_SPINLOCK(frame_queue_lock);
static struct dsscomp_buffer_internal *current_frame;
static struct dsscomp_buffer_internal *next_frame;

static inline unsigned long get_reftime(struct dsscomp_dev *cdev, unsigned long *next )
{
	unsigned long long s = cdev->vsync_cnt * (__u64)cdev->scale;
	do_div( s, cdev->rate );
	
	if( next ) {
		unsigned long long s = (cdev->vsync_cnt + 1) * (__u64)cdev->scale;
		do_div( s, cdev->rate );
		*next = s;
	}
	
	return s;
}

static void dsscomp_isr(void *data, u32 mask)
{
	struct dsscomp_dev *cdev = data;
	
	while (!list_empty(&pending_queue)) {
		unsigned long time_next;
		unsigned long time_now = get_reftime( cdev, &time_next );
		int not_ready = 0;
		struct dsscomp_buffer_internal *next = list_entry(pending_queue.next, struct dsscomp_buffer_internal, node);
		if( time_before( (unsigned long)next->b.ts, time_now ) ) {
			if(debug_isr) 
				dev_info(DEV(cdev), "DROP[%2d]  %8lu < %8lu\n", next->b.id, (unsigned long)next->b.ts, time_now );
			// drop, take next pending frame from queue
			list_del(&next->node);
			// and add to done list
			list_add_tail(&next->node, &done_queue);

		} else if( time_before( (unsigned long)next->b.ts, time_next ) ) {
			int ret;

			if(debug_isr) 
				dev_info(DEV(cdev), "SHOW[%2d]  %8u < %8lu  %08X\r\n", next->b.id, next->b.ts, time_next, (int)next->b.address );
			// show, take next pending frame from queue
			list_del(&next->node);
			if (not_ready == -ETIME) {
				//should be displayed but not ready so too late...
				//Drop the frame
				if(debug_isr)
				dev_info(DEV(cdev), "DROP[%2d]  %8lu < %8lu because resizer too late\n", next->b.id, (unsigned long)next->b.ts, time_now );
				list_add_tail(&next->node, &done_queue);
			} else {
				// and show it
				ret = set_dss_ovl_addr(&next->b);
				if (ret)
					printk("dsscomp_isr: set_dss_ovl_addr failed: %d\n", ret);

				if (current_frame != NULL) {
					list_add_tail(&current_frame->node, &done_queue);
				}

				/* advance the frame mini queue */
				current_frame = next_frame;
				next_frame    = next;
			}
			break;
		} else {
			if(debug_isr) 
				dev_info(DEV(cdev), "....[%2d]  %8u < %8lu\r\n", next->b.id, next->b.ts, time_next );
			// wait
			goto out;
		} 
	}
out:
	cdev->vsync_cnt++;
	
	return;	
}

static u32 get_isr_mask(struct dsscomp_dev *cdev)
{
	return cdev->vsync_src ? DISPC_IRQ_EVSYNC_EVEN : cdev->displays[cdev->vsync_src]->channel == OMAP_DSS_CHANNEL_LCD ? DISPC_IRQ_VSYNC : DISPC_IRQ_VSYNC2;
}

long isr_start( struct dsscomp_dev *cdev, void __user *ptr )
{
	struct omap_dss_device *dev;
	struct omap_video_timings *t;
	struct dsscomp_isr_cfg isr_cfg;
	int r, i;

	if( (r=copy_from_user(&isr_cfg, ptr, sizeof(isr_cfg))) ) {
		return r;
	}

	cdev->vsync_src = isr_cfg.vsync_src;

	if (cdev->vsync_src >= cdev->num_displays) {
		printk("isr_start failed: vsync_src >= num_displays\n");
		return -EINVAL;
	}

	cdev->vsync_cnt = 0;
	current_frame = NULL;
	next_frame    = NULL;
	
	dev = cdev->displays[cdev->vsync_src];
	if (!dev) {
		printk("isr_start failed: dev NULL\n");
		return -EINVAL;
	}
	
	t = &dev->panel.timings;
	
	cdev->rate  = t->pixel_clock;
	cdev->scale = (t->x_res + t->hsw + t->hfp + t->hbp) * (t->y_res + t->vsw + t->vfp + t->vbp);

	ovl_isr_start(cdev);

	if(debug_isr) 
		dev_info(DEV(cdev), "DSSCOMP_ISR_START vsync_src %d  scale %d  rate %d\n", cdev->vsync_src, cdev->scale, cdev->rate );

	cdev->isr_registered = 1;

	return omap_dispc_register_isr(dsscomp_isr, cdev, get_isr_mask(cdev));
}

long isr_stop( struct dsscomp_dev *cdev )
{
	long ret;

	if(debug_isr) 
		dev_info(DEV(cdev), "DSSCOMP_ISR_STOP\n");
	if (cdev->isr_registered) {
		cdev->isr_registered = 0;
		ret = omap_dispc_unregister_isr(dsscomp_isr, cdev, get_isr_mask(cdev));
	} else {
		ret = 0;
	}

	ovl_isr_stop(cdev);

	return ret;
}

long isr_resume( struct dsscomp_dev *cdev )
{
	if (cdev->isr_registered)
		return 0;
	cdev->isr_registered = 1;
	return omap_dispc_register_isr(dsscomp_isr, cdev, get_isr_mask(cdev));
}

long isr_suspend( struct dsscomp_dev *cdev )
{
	if (!cdev->isr_registered)
		return 0;
	cdev->isr_registered = 0;
	return omap_dispc_unregister_isr(dsscomp_isr, cdev, get_isr_mask(cdev));
}

long isr_reftime( struct dsscomp_dev *cdev, void __user *ptr )
{	
	struct dsscomp_buffer buf;

	int r;
	if( (r=copy_from_user(&buf, ptr, sizeof(buf))) ) {
		return r;
	} 

	buf.ref_ts = get_reftime( cdev, NULL );

	if(debug_isr) 
		dev_info(DEV(cdev), "DSSCOMP_ISR_REFTIME: %8d\n", buf.ref_ts);

	return copy_to_user(ptr, &buf, sizeof(buf));
}

long isr_put( struct dsscomp_dev *cdev, void __user *ptr )
{
	unsigned long flags;
	struct dsscomp_buffer_internal* buf;
	
	buf = kmalloc(sizeof(*buf), GFP_KERNEL);
	if (buf == NULL)
		return -ENOMEM;

	if (copy_from_user(buf, ptr, sizeof(struct dsscomp_buffer))) {
		kfree(buf);
		return -EFAULT;
	}

	buf->b.ref_ts = get_reftime( cdev, NULL );

	if  (copy_to_user(ptr, buf, sizeof(struct dsscomp_buffer))) {
		kfree(buf);
		return -EFAULT;
	}

	if( convert_dss_ovl_addr( &buf->b ) ) {
		kfree(buf);
		return -EFAULT;
	}

	spin_lock_irqsave( &frame_queue_lock, flags );
	list_add_tail(&buf->node, &pending_queue);
	spin_unlock_irqrestore( &frame_queue_lock, flags );

	return 0;
}

long isr_get( struct dsscomp_dev *cdev, void __user *ptr )
{
	unsigned long flags;
	struct dsscomp_buffer_internal* buf;

	if (list_empty(&done_queue))
		return -EAGAIN;

	buf = list_entry(done_queue.next, struct dsscomp_buffer_internal, node);

	buf->b.ref_ts = get_reftime( cdev, NULL );

	if (copy_to_user(ptr, buf, sizeof(struct dsscomp_buffer))) {
		return -EFAULT;
	}

	spin_lock_irqsave( &frame_queue_lock, flags );
	list_del(&buf->node);
	spin_unlock_irqrestore( &frame_queue_lock, flags );
	kfree(buf);
		
	return 0;
}

long isr_flush( struct dsscomp_dev *cdev )
{
	unsigned long flags;
	spin_lock_irqsave( &frame_queue_lock, flags );
			
	if(debug_isr) 
		dev_info(DEV(cdev), "DSSCOMP: isr_flush\n");
	// empty the mini_queue(tm)
	if (current_frame != NULL) {
		list_add_tail(&current_frame->node, &done_queue);
		current_frame = NULL;
	}
	if (next_frame != NULL) {
		list_add_tail(&next_frame->node, &done_queue);
		next_frame = NULL;
	}

	// empty the pending queue into the done queue
	while (!list_empty(&pending_queue)) {
		struct dsscomp_buffer_internal *next_pending = list_entry(pending_queue.next, struct dsscomp_buffer_internal, node);
		list_del(&next_pending->node);
		list_add_tail( &next_pending->node, &done_queue);
	}

	cdev->vsync_cnt = 0;

	spin_unlock_irqrestore( &frame_queue_lock, flags );

	return 0;
}
