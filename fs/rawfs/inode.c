/*
 * RAWFS file system, Linux implementation
 *
 * ino  |   section
 * ----------------
 *   0  |   root
 *   1  |   avboot
 *   2  |   banner
 *   ...
 *
 *
 */

#include <linux/module.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/rawfs_fs.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/pagemap.h>
#include <linux/buffer_head.h>
#include <linux/vfs.h>
#include <linux/mpage.h>
#include <linux/ioctl.h>
#include <linux/writeback.h>

#include <asm/uaccess.h>

struct rawfs_inode_info {
	struct inode vfs_inode;
	unsigned long offset;
	unsigned short max_size;
	unsigned long flags;
};

static struct inode *rawfs_iget(struct super_block *, unsigned long);
static struct dentry *rawfs_lookup(struct inode *dir, struct dentry *dentry, struct nameidata *nd);

/* instead of private superblock data */
static inline unsigned long rawfs_maxsize(struct super_block *sb)
{
	return (unsigned long)sb->s_fs_info;
}

static inline struct rawfs_inode_info *rawfs_inode(struct inode *inode)
{
	return container_of(inode, struct rawfs_inode_info, vfs_inode);
}

static const struct super_operations rawfs_ops;

static int rawfs_fill_super(struct super_block *s, void *data, int silent)
{
	struct buffer_head *bh;
	struct rawfs_super_block *rsb;
	struct inode *root;
	int ret = -EINVAL;

	/* I would parse the options here, but there are none.. :) */
	sb_set_blocksize(s, RAWFS_BLOCK_SIZE);

	s->s_maxbytes = 0xFFFFFFFF;

	bh = sb_bread(s, 0);
	if (!bh) {
		/* XXX merge with other printk? */
                printk ("rawfs: unable to read superblock\n");
		goto outnobh;
	}

	rsb = kmalloc(sizeof(struct rawfs_super_block), GFP_KERNEL);
	if (!rsb)
		goto out;
		
	memcpy(&rsb->table, bh->b_data, sizeof(struct rawfs_super_block));

	if (rsb->table.magic != MAGIC_RAWFS) {
		if (!silent)
			printk ("VFS: Can't find a rawfs filesystem on dev "
				"%s.\n", s->s_id);
		goto outkfree;
	}
	
	s->s_magic = MAGIC_RAWFS;
	s->s_flags |= MS_NODIRATIME | MS_NOATIME;
	s->s_op	= &rawfs_ops;
	s->s_fs_info = (void *)rsb;
	
	root = rawfs_iget(s, RAWFS_ROOT_INO);
	if (IS_ERR(root)) {
		ret = PTR_ERR(root);
		goto outkfree;
	}

	ret = -ENOMEM;
	s->s_root = d_alloc_root(root);
	if (!s->s_root)
		goto outiput;

	brelse(bh);
	return 0;

outiput:
	iput(root);
outkfree:
	kfree(rsb);	
out:
	brelse(bh);
outnobh:

	return ret;
}

static void rawfs_put_super(struct super_block *sb)
{
	struct rawfs_super_block *rsb = sb->s_fs_info;
	kfree(rsb);
}

static int rawfs_statfs(struct dentry *dentry, struct kstatfs *buf)
{
	struct inode *inode;
	struct super_block *s = dentry->d_inode->i_sb;
	struct rawfs_super_block *rsb = s->s_fs_info;
	int ind;
	
	buf->f_blocks = 0;
	buf->f_ffree = 0;
	
	for (ind=0; ind<rsb->table.num_sections; ind++) {
		buf->f_blocks += rsb->table.sections[ind].size;
	
		inode = rawfs_iget(s, ind+1);
		if (IS_ERR(inode)) {
			return PTR_ERR(inode);
		}
		
		buf->f_ffree += (rsb->table.sections[ind].size - (inode->i_size >> RAWFS_BLOCK_BITS));
	}
	
	buf->f_type = MAGIC_RAWFS;
	buf->f_bsize = RAWFS_BLOCK_SIZE;
	buf->f_bfree = buf->f_bavail = buf->f_ffree;
	buf->f_namelen = RAWFS_MAX_NAMELEN;

	return 0;
}

static int rawfs_readdir(struct file *filp, void *dirent, filldir_t filldir)
{
	struct inode *i = filp->f_path.dentry->d_inode;
	int stored = 0;
	unsigned long offset;
	struct super_block *s = i->i_sb;
	struct rawfs_super_block *rsb = s->s_fs_info;
	int ind;

	offset = filp->f_pos;
	if (offset >= rsb->table.num_sections) {
		goto out;
	}
	
	for (ind=0; ind<rsb->table.num_sections; ind++) {
		if (filldir(dirent, rsb->table.sections[ind].name, strlen(rsb->table.sections[ind].name), offset, ind+1,
			    DT_REG) < 0) {
			goto out;
		}
		
		offset++;
		stored++;
	}

	filp->f_pos = offset;
out:
	return stored;
}

static struct dentry *rawfs_lookup(struct inode *dir, struct dentry *dentry, struct nameidata *nd)
{
	struct super_block *s = dir->i_sb;
	struct rawfs_super_block *rsb = s->s_fs_info;
	struct inode *inode = NULL;
	const char *name;
	int res, len, ind;

	res = -EACCES;
	
	name = dentry->d_name.name;
	len = dentry->d_name.len;

	for (ind = 0; ind<rsb->table.num_sections; ind++) {
		if (strncmp(name, rsb->table.sections[ind].name, len) == 0)
			break;
	}
	
	if (ind == rsb->table.num_sections)
		goto error; 
	
	inode = rawfs_iget(s, ind+1);
	if (IS_ERR(inode)) {
		res = PTR_ERR(inode);
		goto error;
	}

	d_add(dentry, inode);
	res = 0;
error:
	return ERR_PTR(res);
}

static int rawfs_bmap(struct inode *inode, sector_t sector, sector_t *phys,
	     unsigned long *mapped_blocks, int create)
{
	struct super_block *sb = inode->i_sb;
	const unsigned long blocksize = sb->s_blocksize;
	const unsigned char blocksize_bits = sb->s_blocksize_bits;
	sector_t last_block;

	*phys = 0;
	*mapped_blocks = 0;
	if (inode->i_ino == RAWFS_ROOT_INO) {
		return 0;
	}

	last_block = (i_size_read(inode) + (blocksize - 1)) >> blocksize_bits;
	if (sector >= last_block) {
		if (!create)
			return 0;

		last_block = rawfs_inode(inode)->max_size - 1;
		if (sector >= last_block)
			return 0;

	}

	*phys = sector + rawfs_inode(inode)->offset + 1;
	*mapped_blocks = 1; // last_block - sector;

	return 0;
}

static int rawfs_get_block(struct inode *inode, sector_t iblock,
			 struct buffer_head *bh_result, int create)
{
	struct super_block *sb = inode->i_sb;
	unsigned long mapped_blocks;
	unsigned long max_blocks = bh_result->b_size >> inode->i_blkbits;
	sector_t phys;
	int err;
	
	err = rawfs_bmap(inode, iblock, &phys, &mapped_blocks, create);
	if (err)
		return err;

	if (phys) {
		map_bh(bh_result, sb, phys);
		max_blocks = min(mapped_blocks, max_blocks);
	}

	if (create) {
		/* available blocks of this section */
		mapped_blocks = rawfs_inode(inode)->max_size-1;
		max_blocks = min(mapped_blocks, max_blocks);

		err = rawfs_bmap(inode, iblock, &phys, &mapped_blocks, create);
		if (err)
			return err;

		//BUG_ON(*max_blocks != mapped_blocks);
		set_buffer_new(bh_result);
		map_bh(bh_result, sb, phys);

	}
	
	bh_result->b_size = max_blocks << sb->s_blocksize_bits;
	return 0;
}

static int rawfs_readpage(struct file *file, struct page *page)
{
	return mpage_readpage(page, rawfs_get_block);
}

static int rawfs_readpages(struct file *file, struct address_space *mapping,
			 struct list_head *pages, unsigned nr_pages)
{
	return mpage_readpages(mapping, pages, nr_pages, rawfs_get_block);
}

static int rawfs_writepage(struct page *page, struct writeback_control *wbc)
{
	return block_write_full_page(page, rawfs_get_block, wbc);
}

static int rawfs_writepages(struct address_space *mapping,
			  struct writeback_control *wbc)
{
	return mpage_writepages(mapping, wbc, rawfs_get_block);
}

static int rawfs_write_begin(struct file *file, struct address_space *mapping,
			loff_t pos, unsigned len, unsigned flags,
			struct page **pagep, void **fsdata)
{
	*pagep = NULL;
	return block_write_begin(mapping, pos, len, flags, pagep,
				rawfs_get_block);
}

#if 0
static sector_t _rawfs_bmap(struct address_space *mapping, sector_t block)
{
	return generic_block_bmap(mapping, block, rawfs_get_block);
}
#endif

static int rawfs_write_end(struct file *file, struct address_space *mapping,
			loff_t pos, unsigned len, unsigned copied,
			struct page *pagep, void *fsdata)
{
	//struct inode *inode = mapping->host;
	int err;

	err = generic_write_end(file, mapping, pos, len, copied, pagep, fsdata);
	if (err < 0)
		return err;
	
	//inode->i_mtime = inode->i_ctime = CURRENT_TIME_SEC;
	//mark_inode_dirty(inode);

	return err;
}

static int rawfs_file_release(struct inode *inode, struct file *filp)
{
	return 0;
}

long rawfs_file_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct inode *inode = filp->f_dentry->d_inode;
	
	switch (cmd) {
		case GETMAXSIZE: 
		{
			unsigned int max_size = le16_to_cpu(rawfs_inode(inode)->max_size) * RAWFS_BLOCK_SIZE;
			if (copy_to_user((void __user *)arg, (void *) (&max_size), sizeof(max_size))) {
				printk("RAWFS: GETMAXSIZE could not copy_to_user\n");
				return -EFAULT;
			}
		}
			break;
		default:
//printk("RAWFS: ioctl not supported (cmd: %d)\n", cmd);
			return -ENOTTY;
	}
	
	return 0;
}

static int rawfs_file_permission(struct inode *inode, int mask, unsigned int flags)
{
//printk("rawfs_file_permission: %08x, %04o\n", mask, mask);
	if (current_fsuid() == 0) {
		return 0;
	}

	if (S_ISREG(inode->i_mode) && mask & MAY_WRITE) {
		umode_t mode = inode->i_mode;
	
		if (current_fsuid() == inode->i_uid)
			mode >>= 6;
		else if (in_group_p(inode->i_gid))
			mode >>= 3;

		if (mode & MAY_WRITE)
			return 0;
		
		return -EACCES;;	
	}
	
	return generic_permission(inode, mask, flags, NULL);
}

static int rawfs_setattr(struct dentry *dentry, struct iattr *attr)
{
	struct inode *inode = dentry->d_inode;
	int err = -EPERM;
//printk("rawfs_setattr, inode: %08x, %04o\n", inode->i_mode, inode->i_mode);
#ifndef CONFIG_RAWFS_FS_DISABLE_WRITEPROT
	if (rawfs_inode(inode)->flags & RAWFS_FLAGS_WRITABLE) 
#endif
	{
		if (attr->ia_valid & ATTR_UID || attr->ia_valid & ATTR_GID) {
			if (attr->ia_valid & ATTR_UID) {
				inode->i_uid = attr->ia_uid;
			}
	
			if (attr->ia_valid & ATTR_GID) {
				inode->i_gid = attr->ia_gid;		
			}
		
			mark_inode_dirty(inode);
		
			return 0;		
		}

	}
	
	return err;
}
	
/* Mapping from our types to the kernel */

static const struct address_space_operations rawfs_aops = {
	.readpage 	= rawfs_readpage,
	.readpages 	= rawfs_readpages,
	.writepage 	= rawfs_writepage,
	.writepages 	= rawfs_writepages,
	/* FIXME: do we need to support set_page_dirty? */
	/*.sync_page	= block_sync_page,*/
	.write_begin	= rawfs_write_begin,
	.write_end 	= rawfs_write_end,
};

const struct file_operations rawfs_file_operations = {
	.llseek		= generic_file_llseek,
	.read		= do_sync_read,
	.write		= do_sync_write,
	.aio_read	= generic_file_aio_read,
	.aio_write	= generic_file_aio_write,
	.mmap		= generic_file_mmap,
	.release	= rawfs_file_release,
	.unlocked_ioctl	= rawfs_file_ioctl,
	.fsync		= generic_file_fsync,
	.splice_read	= generic_file_splice_read,
};

static const struct file_operations rawfs_dir_operations = {
	.read		= generic_read_dir,
	.readdir	= rawfs_readdir,
};

static const struct inode_operations rawfs_dir_inode_operations = {
	.lookup		= rawfs_lookup,
};

static const struct inode_operations rawfs_inode_operations = {
	.permission	= rawfs_file_permission,
	.setattr	= rawfs_setattr,
};

static struct inode * rawfs_iget(struct super_block *sb, unsigned long ino)
{
	struct inode *i = NULL;
	struct buffer_head *bh;
	struct rawfs_super_block *rsb = sb->s_fs_info;

	i = iget_locked(sb, ino);
	if (!i)
		return ERR_PTR(-ENOMEM);
	if (!(i->i_state & I_NEW))
		return i;

	i->i_mode = 0;
	i->i_uid = 0;
	i->i_gid = 0;
		
	/* root */
	if (ino == RAWFS_ROOT_INO) {
		i->i_mode = S_IFDIR | S_IRUGO | S_IWUSR;
		i->i_size = 4096;
		i->i_op = &rawfs_dir_inode_operations;
		i->i_fop = &rawfs_dir_operations;
	} else if (ino-1 < rsb->table.num_sections) { 
		struct rawfs_section *section = &rsb->table.sections[ino-1];
		unsigned long offset = le32_to_cpu(section->offset)+1;
		unsigned short max_size = le16_to_cpu(section->size);
		
		struct rawfs_section_header *section_header;

		rawfs_inode(i)->offset = offset;
		rawfs_inode(i)->max_size = max_size;
		rawfs_inode(i)->flags = le32_to_cpu(section->flags);
		
		bh = sb_bread(sb, offset);
		if (!bh) {
			return ERR_PTR(-ENOMEM); /* error */
		}
		section_header = (struct rawfs_section_header *)bh->b_data;
	
 		if (section_header->magic == MAGIC_RAWFS_SECTION) {
			i->i_size = le32_to_cpu(section_header->size);
			i->i_uid = le32_to_cpu(section_header->uid);
			i->i_gid = le32_to_cpu(section_header->gid);
		} else {
			i->i_size = 0;
		}

		i->i_mode = S_IFREG | S_IRUGO;
#ifndef CONFIG_RAWFS_FS_DISABLE_WRITEPROT
		if (rawfs_inode(i)->flags & RAWFS_FLAGS_WRITABLE) 
#endif
		{
			i->i_mode |= S_IWUSR | S_IWGRP;
		}
		i->i_op = &rawfs_inode_operations;
		i->i_fop = &rawfs_file_operations;
		i->i_data.a_ops = &rawfs_aops;
		
		brelse(bh);
	} else {
		printk("ino out of range\n");
		iget_failed(i);
		return ERR_PTR(-EINVAL);		
	}

	i->i_nlink = 1;
	i->i_mtime.tv_sec = i->i_atime.tv_sec = i->i_ctime.tv_sec = 0;
	i->i_mtime.tv_nsec = i->i_atime.tv_nsec = i->i_ctime.tv_nsec = 0;

	unlock_new_inode(i);
	return i;
}

static int __rawfs_write_inode(struct inode *inode, int wait)
{
	struct super_block *sb = inode->i_sb;
	struct buffer_head *bh;
	struct rawfs_section_header *section_header;
	int err;

	if (inode->i_ino == RAWFS_ROOT_INO)
		return 0;

	bh = sb_bread(sb, rawfs_inode(inode)->offset);
	if (!bh) {
		printk(KERN_ERR "RAWFS: unable to read inode block for updating\n");
		return -EIO;
	}

	section_header = (struct rawfs_section_header *)bh->b_data;
	
	section_header->magic = MAGIC_RAWFS_SECTION;
	section_header->uid = cpu_to_le32(inode->i_uid);
	section_header->gid = cpu_to_le32(inode->i_gid);	
	section_header->size = cpu_to_le32(inode->i_size);

	mark_buffer_dirty(bh);

	err = 0;
	if (wait)
		err = sync_dirty_buffer(bh);
	brelse(bh);
	return err;
}

static int rawfs_write_inode(struct inode *inode, struct writeback_control *wbc)
{
	return __rawfs_write_inode(inode, wbc->sync_mode == WB_SYNC_ALL);
}

static struct kmem_cache *rawfs_inode_cachep;

static struct inode *rawfs_alloc_inode(struct super_block *sb)
{
	struct rawfs_inode_info *ei;

	ei = kmem_cache_alloc(rawfs_inode_cachep, GFP_KERNEL);
	if (!ei)
		return NULL;
	return &ei->vfs_inode;
}

static void rawfs_destroy_inode(struct inode *inode)
{
	kmem_cache_free(rawfs_inode_cachep, rawfs_inode(inode));
}

static void init_once(void *foo)
{
	struct rawfs_inode_info *ei = foo;
	inode_init_once(&ei->vfs_inode);
}

static int init_inodecache(void)
{
	rawfs_inode_cachep = kmem_cache_create("rawfs_inode_cache",
					     sizeof(struct rawfs_inode_info),
					     0, (SLAB_RECLAIM_ACCOUNT|
						SLAB_MEM_SPREAD),
					     init_once);
	if (rawfs_inode_cachep == NULL)
		return -ENOMEM;
	return 0;
}

static void destroy_inodecache(void)
{
	kmem_cache_destroy(rawfs_inode_cachep);
}

static int rawfs_remount(struct super_block *sb, int *flags, char *data)
{
	*flags |= MS_NODIRATIME | MS_NOATIME;
	return 0;
}

static const struct super_operations rawfs_ops = {
	.alloc_inode	= rawfs_alloc_inode,
	.destroy_inode	= rawfs_destroy_inode,
	.write_inode	= rawfs_write_inode,
	.put_super	= rawfs_put_super,
	.statfs		= rawfs_statfs,
	.remount_fs	= rawfs_remount,
};

/*
 * get a superblock for mounting
 */
static struct dentry *rawfs_mount(struct file_system_type *fs_type,
			int flags, const char *dev_name,
			void *data)
{
	return mount_bdev(fs_type, flags, dev_name, data,
				  rawfs_fill_super);
}

static struct file_system_type rawfs_fs_type = {
	.owner		= THIS_MODULE,
	.name		= "rawfs",
	.mount		= rawfs_mount,
	.kill_sb	= kill_block_super,
	.fs_flags	= FS_REQUIRES_DEV,
};

static int __init init_rawfs_fs(void)
{
	int err = init_inodecache();
	if (err)
		goto out1;
        err = register_filesystem(&rawfs_fs_type);
	if (err)
		goto out;
	return 0;
out:
	destroy_inodecache();
out1:
	return err;
}

static void __exit exit_rawfs_fs(void)
{
	unregister_filesystem(&rawfs_fs_type);
	destroy_inodecache();
}

module_init(init_rawfs_fs)
module_exit(exit_rawfs_fs)
