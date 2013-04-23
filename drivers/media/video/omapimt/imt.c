/*
 * drivers/media/video/omapimt/imt.c
 *
 * Copyright (C) 2012 Archos
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

#include <linux/module.h>
#include <linux/types.h>
#include <linux/rpmsg.h>
#include <linux/ion.h>

int debug_imt = 1;
int debug_iva = 1;

module_param(debug_imt, int, 0644);
module_param(debug_iva, int, 0644);

#define DBG(fmt,...)  if (debug_imt)     printk(fmt"\n", ##__VA_ARGS__)
#define DBG2(fmt,...) if (debug_imt > 1) printk(fmt"\n", ##__VA_ARGS__)
#define ERR(fmt,...)                     printk(fmt"\n", ##__VA_ARGS__)

static struct rpmsg_channel *rpdev;

#if defined(CONFIG_ION_OMAP)
#include <linux/ion.h>
#include <linux/omap_ion.h>
static struct ion_client *imt_ion_client;
extern struct ion_device *omap_ion_device;

#define MAX_ALLOC 128

struct alloc_t {
	ion_phys_addr_t phys;
	struct ion_handle *handle;
} alloc_list[MAX_ALLOC];

#endif

struct imt_rpc_req {
	uint16_t id;
	uint16_t len;
	uint32_t cmd;
	uint32_t par;
	uint32_t rsp;
	uint8_t  data[1];
} __packed;

enum IMT_CMD {
	IMT_CONNECT = 0x01,
	IMT_CONNECTED,
	IMT_TRACE,
	IMT_TRACED,
	IMT_MALLOC,
	IMT_MALLOCED,
	IMT_FREE,
	IMT_FREED,
	IMT_CONFUSED,
};

#if defined(CONFIG_ION_OMAP)
static struct alloc_t *get_alloc( void ) 
{
	int i;
	for( i = 0; i < MAX_ALLOC; i++ ) {
		if( !alloc_list[i].handle && !alloc_list[i].phys ) {
			return &alloc_list[i];
		}
	}
	ERR("imt: out of allocations!!!");
	return NULL;
}

static struct ion_handle *put_alloc( ion_phys_addr_t phys ) 
{
	int i;
	for( i = 0; i < MAX_ALLOC; i++ ) {
		if( alloc_list[i].phys == phys ) {
			struct ion_handle *handle = alloc_list[i].handle;
			alloc_list[i].handle = NULL;
			alloc_list[i].phys   = 0;
			return handle;
		}
	}
	ERR("imt: cannot find alloc for %08lX!", phys);
	return NULL;
}

static uint32_t tiler_alloc( int size ) 
{
	int res;
	
	struct omap_ion_tiler_alloc_data alloc_data = {
		.w = size,
		.h = 1,
		.fmt = TILER_PIXEL_FMT_PAGE,
		.flags = 0,
	};

	struct alloc_t *alloc = get_alloc();
	if( !alloc ) {
		return 0;
	}
	
	res = omap_ion_tiler_alloc(imt_ion_client, &alloc_data);

	if (res < 0) {
		ERR("imt: could not alloc %d\n", res);
		return 0;
	} else {
		ion_phys_addr_t phys;
		size_t len;
		ion_phys(imt_ion_client, alloc_data.handle, &phys, &len);
		DBG2("imt: ion: siz %8d  %08X  phys %08lX  len %8d", size, (unsigned int)alloc_data.handle, phys, (int)len );
		alloc->phys   = phys;
		alloc->handle = alloc_data.handle;
		return phys;
	}
}

static void tiler_free( uint32_t data )
{
	struct ion_handle *handle = put_alloc( (ion_phys_addr_t)data );
	if( handle ) {
		ion_free(imt_ion_client, handle);
	}
}

static void free_all( void ) 
{
	int i;
	for( i = 0; i < MAX_ALLOC; i++ ) {
		if( alloc_list[i].handle ) {
			DBG("imt: cleanup[%d] %08X  phys %08lX", i, (unsigned int)alloc_list[i].handle, alloc_list[i].phys);
			ion_free(imt_ion_client, alloc_list[i].handle);
			alloc_list[i].handle = NULL;
			alloc_list[i].phys   = 0;
		}
	}
}
#endif

void imt_cleanup( void ) 
{
	DBG("imt: cleanup\n");
#if defined(CONFIG_ION_OMAP)
	free_all();
#endif
}
EXPORT_SYMBOL(imt_cleanup);

static void output_trace( const char *b )
{
	if (debug_iva) {
		if( b[strlen(b) - 1] == '\n' ) {
			printk("iva: %s", b);	
		} else {
			printk("iva: %s\n", b);	
		}
	}
}

static int connect(void)
{
	char buffer[16];
	struct imt_rpc_req *req = (struct imt_rpc_req*)buffer;
	int ret;
	
	// send a connect msg	
	req->id  = 0x00;
	req->len = 16;
	req->cmd = IMT_CONNECT;
	req->par = 0x42;
	req->rsp = 0x00;
 	
	ret = rpmsg_send(rpdev, req, req->len);
	if (ret) {
		ERR("imt: rpsend failed: %d", ret);
		return ret;
	}
	return 0;
}

static int reply( struct imt_rpc_req *req, int cmd, int len, uint32_t rsp )
{
	int ret;
	
	req->cmd = cmd;
	req->len = len;
	req->rsp = rsp;
	
	ret = rpmsg_send(rpdev, req, req->len);
	if (ret) {
		ERR("imt: rpsend failed: %d", ret);
		return 1;
	}
	return 0;
}

static void handle_request( struct imt_rpc_req *req )
{
	DBG2("imt: req: id  %4d  cmd %X  len %3d  par %08X  rsp %08X", req->id, req->cmd, req->len, req->par, req->rsp );

	if( req->cmd == IMT_CONNECTED ) {
		DBG("imt: CONNECTED: %02X", req->rsp);	
		
		// no need to reply
		return;
	} else if( req->cmd == IMT_TRACE ) {
		DBG2("imt: TRACE: %d", req->par);	
		
		output_trace(req->data);
		
		// TRACE does not respond!
		// reply( req, IMT_TRACED, 16, req->par );
	} else if( req->cmd == IMT_MALLOC ) {
		uint32_t data = 0;
		DBG2("imt: MALLOC: %d", req->par);	
#if defined(CONFIG_ION_OMAP)
		data = tiler_alloc( req->par );
#endif				
		reply( req, IMT_MALLOCED, 16, data );
	} else if( req->cmd == IMT_FREE ) {
		DBG2("imt: FREE: %08X", req->par);	
#if defined(CONFIG_ION_OMAP)
		tiler_free( req->par );
#endif				
		reply( req, IMT_FREED, 16, req->par );
	} else {
		// we dont know that command
		ERR("imt: CONFUSED!: %X", req->cmd);	
		
		reply( req, IMT_CONFUSED, 16, req->par );
	}
}

/*
 * RPMSG API:
 */
static int rpmsg_probe(struct rpmsg_channel *_rpdev)
{
	DBG("imt: rpmsg_probe");
	rpdev = _rpdev;
	
	return connect();
}

static void __devexit rpmsg_remove(struct rpmsg_channel *_rpdev)
{
	DBG("imt: rpmsg_remove");
	rpdev = NULL;
}

static void rpmsg_cb(struct rpmsg_channel *rpdev, void *data,
		int len, void *priv, u32 src)
{
	char buffer[16 + 512];
	struct imt_rpc_req *req = (struct imt_rpc_req*)buffer;

//	DBG("imt: rpmsg_cb: len=%d, src=%d", len, src);

	memcpy( req, data, len );
	
	handle_request( req );
}

static struct rpmsg_device_id rpmsg_id_table[] = {
		{ .name = "rpmsg-imt" },
		{ },
};

static struct rpmsg_driver rpmsg_driver = {
		.drv.name	= KBUILD_MODNAME,
		.drv.owner	= THIS_MODULE,
		.id_table	= rpmsg_id_table,
		.probe		= rpmsg_probe,
		.callback	= rpmsg_cb,
		.remove		= __devexit_p(rpmsg_remove),
};

static int __init omap_imt_init(void)
{
	int r = -EINVAL;

	DBG("omap_imt_init");

	r = register_rpmsg_driver(&rpmsg_driver);

#ifdef CONFIG_ION_OMAP
	imt_ion_client = ion_client_create(omap_ion_device,
					    (1<< ION_HEAP_TYPE_CARVEOUT) |
					    (1 << OMAP_ION_HEAP_TYPE_TILER),
					    "omapimt");
#endif

	return r;
}

static void __exit omap_imt_exit(void)
{
	DBG("omap_imt_exit");
#ifdef CONFIG_ION_OMAP
	free_all();
	ion_client_destroy(imt_ion_client);
#endif
	unregister_rpmsg_driver(&rpmsg_driver);
}

module_init(omap_imt_init);
module_exit(omap_imt_exit);

MODULE_AUTHOR("Vladimir Pantelic <pantelic@archos.com>");
MODULE_DESCRIPTION("OMAP IVAHD Memory & Trace");
MODULE_LICENSE("GPL v2");
