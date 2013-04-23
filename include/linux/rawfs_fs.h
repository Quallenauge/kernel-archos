#ifndef __LINUX_RAWFS_FS_H
#define __LINUX_RAWFS_FS_H

/* The basic structures of the romfs filesystem */

#define RAWFS_BLOCK_SIZE 	512
#define RAWFS_BLOCK_BITS 	9

#define RAWFS_MAX_NAMELEN	32

#define MAGIC_RAWFS 		0x77617261
#define MAGIC_RAWFS_SECTION 	0x77617266

#define MAX_NUM_SECTIONS	10
#define MAX_LENGTH_SECTION_NAME	32

#define RAWFSSECT_OFS_APPEND	(-1)
#define RAWFSSECT_SIZ_FULL	(0)

#define RAWFS_ROOT_INO		0

typedef enum {
	RAWFS_FLAGS_WRITABLE = 0x00000001,
} RAWFS_FLAGS;
 
struct rawfs_section {
	unsigned long offset;		/* in blocks */
	unsigned short size;		/* in blocks */
	char name[MAX_LENGTH_SECTION_NAME];
	unsigned long flags;
	unsigned char reserved[4];	
} __attribute__((packed));

struct rawfs_section_table {
	unsigned long magic;
	unsigned short chksum;
	unsigned char num_sections;
	unsigned char reserved[25];
	struct rawfs_section sections[MAX_NUM_SECTIONS];	
} __attribute__((packed));

struct rawfs_super_block {
	struct rawfs_section_table table;
};

struct rawfs_section_header {
	unsigned long magic;
	unsigned short checksum;
	unsigned char reserved[2];
	unsigned long size;		/* in bytes */
	unsigned long flags;
	unsigned long uid;
	unsigned long gid;
	unsigned long mode;
} __attribute__((packed));


#define GETMAXSIZE		_IOR('m', 1, unsigned int)

#endif /* __LINUX_RAWFS_FS_H */
