#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/stddef.h>
#include <linux/slab.h>
//#include <init.h>

#include <asm/setup.h>
#include <asm/feature_list.h>

static const struct feature_tag *taglist;
static int taglist_size;

static int __init parse_tag_feature_list(const struct tag *tag)
{
//printk(KERN_INFO "parse_tag_feature_list: tagsize: %d, size: %d\n", tag->hdr.size<<2, tag->u.feature_list.size);
	if (tag->hdr.size <= 2 || tag->u.feature_list.size == 0)
		return -1;
		
	taglist = (struct feature_tag *)tag->u.feature_list.data;
	taglist_size = tag->u.feature_list.size;

	{
		/* only an example */
		struct feature_tag_product_name *product_name = get_feature_tag(FTAG_PRODUCT_NAME, feature_tag_size(feature_tag_product_name));
		if (product_name)
			printk(KERN_INFO "Product Name: %s\n", product_name->name);
	}
	return 0;
}

__tagtable(ATAG_FEATURE_LIST, parse_tag_feature_list);


void *get_feature_tag(u32 tag, u32 size)
{
	const struct feature_tag *params = taglist;
	const struct feature_tag_core *core;	
	const struct feature_tag *ptr;
		
	if (!params || params->hdr.tag != FTAG_CORE || params->hdr.size != feature_tag_size(feature_tag_core)) {
		printk(KERN_ERR "Invalid feature tag type\n");
		return NULL;
	}
	
	core = &params->u.core;	
	if  (core->magic != FEATURE_LIST_MAGIC) {
		printk(KERN_ERR "Magic of feature tag list invalid\n");
		return NULL;
	}

	for_each_feature_tag(ptr, params) {
		if (ptr->hdr.tag == tag) {
		
			if (ptr->hdr.size != size) {
				printk(KERN_ERR "get_feature_tag: Invalid tag size\n");
				return NULL;
			}
			return (void*)&ptr->u;
		}		
	}
	
	return NULL;
}

static __init int feature_tag_list_init(void) 
{
	 struct feature_tag *tmp;
//printk(KERN_INFO "feature_tag_list_init: tagsize: %d, %08x\n",taglist_size, taglist);
	if (!taglist || taglist_size == 0)
		return -1;

	/* make a permanent copy of the taglist */
	tmp = kmalloc(taglist_size, GFP_KERNEL);
	if (tmp)
		return -1;
	
	if (((unsigned long)tmp) & 0x3) {
		printk(KERN_ERR "taglist not 4 byte aligned\n");
		kfree(tmp);
		return -1;
	}
	
	taglist = tmp;	
	return 0;
}

core_initcall(feature_tag_list_init);
