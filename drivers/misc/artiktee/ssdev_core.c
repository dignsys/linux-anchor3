/*********************************************************
 * Copyright (C) 2011 - 2015 Samsung Electronics Co., Ltd All Rights Reserved
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation version 2 and no later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.
 *
 *********************************************************/

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/ioctl.h>
#include <linux/mutex.h>
#include <linux/sched.h>
#include <linux/spinlock.h>
#include <linux/completion.h>
#include <linux/percpu.h>
#include <linux/sysfs.h>
#include <linux/vmalloc.h>
#include <linux/crc32.h>
#include <linux/fs.h>
#include <linux/slab.h>

#include "ssdev_rpmb.h"
#include "ssdev_core.h"
#include "ssdev_file.h"
#include "tzlog_core.h"

#include "tzdev.h"
#include "tzdev_internal.h"
#include "tzdev_smc.h"
#include "nsrpc_ree_slave.h"
#include "tzlog_print.h"
#include "tee_messages.h"

#ifndef CONFIG_SECOS_NO_SECURE_STORAGE

#define SS_WSM_SIZE         (0x00020000)

enum scm_watch_target {
	TARGET_SS_DEV = 0x1001,
};

struct ss_wsm {
	struct mutex wsm_lock;
	int wsm_id;
	unsigned int max_size;
	char *payload;
};

enum ss_dirty_type {
	SS_OBJ_DIRTY_UPDATE,
	SS_OBJ_DIRTY_DELETE,
};

enum ss_event_type {
	SS_EVENT_QUERY_OBJECT = 0x00000001,
	SS_EVENT_FILE_LOAD_OBJECT = 0x00000010,
	SS_EVENT_FILE_UPDATE_OBJECT = 0x00000020,
	SS_EVENT_RPMB_LOAD_OBJECT = 0x00000100,
	SS_EVENT_RPMB_UPDATE_OBJECT = 0x00000200,
};

enum ss_cmd_type {
	/*common */
	SS_CMD_REGISTER_WSM = 0,
	SS_CMD_QUERY_OBJECT,
	SS_CMD_FILE_LOAD_OBJECT,
	SS_CMD_FILE_LOAD_OBJECT_LIST,
	SS_CMD_FILE_READ_DATA,
	SS_CMD_FILE_DELETE_DATA,
	SS_CMD_FILE_CREATE_DATA,
	SS_CMD_FILE_APPEND_DATA,
	SS_CMD_FILE_DELETE_FILE,
	SS_CMD_RPMB_GET_WRITE_COUNTER,
	SS_CMD_RPMB_LOAD_FRAMES,
	SS_CMD_RPMB_WRITE_FRAMES,
	SS_CMD_RPMB_GET_PARTITION_SIZE,
};

enum ss_sub_cmd_type {
	/*for object query */
	SS_SUB_CMD_QUERY_OBJECT_PREPARE,
	SS_SUB_CMD_FILE_QUERY_OBJECT,	/*unused */
	SS_SUB_CMD_RPMB_QUERY_OBJECT,
	SS_SUB_CMD_QUERY_OBJECT_POST,

	/*for file load */
	SS_SUB_CMD_FILE_LOAD_OBJECT_PREPARE,
	SS_SUB_CMD_FILE_LOAD_OBJECT,
	SS_SUB_CMD_FILE_LOAD_OBJECT_PARSE,
	SS_SUB_CMD_FILE_LOAD_OBJECT_POST,

	/*for file update */
	SS_SUB_CMD_FILE_UPDATE_OBJECT_PREPARE,
	SS_SUB_CMD_FILE_UPDATE_OBJECT_START,
	SS_SUB_CMD_FILE_UPDATE_OBJECT,
	SS_SUB_CMD_FILE_UPDATE_OBJECT_END,
	SS_SUB_CMD_FILE_UPDATE_OBJECT_POST,

	/*for rpmb load */
	SS_SUB_CMD_RPMB_LOAD_OBJECT_PREPARE,
	SS_SUB_CMD_RPMB_LOAD_OBJECT_GET_NONCE,
	SS_SUB_CMD_RPMB_LOAD_OBJECT,
	SS_SUB_CMD_RPMB_LOAD_OBJECT_PARSE,
	SS_SUB_CMD_RPMB_LOAD_OBJECT_POST,

	/*for rpmb update */
	SS_SUB_CMD_RPMB_UPDATE_OBJECT_PREPARE,

	SS_SUB_CMD_RPMB_UPDATE_OBJECT_START,
	SS_SUB_CMD_RPMB_UPDATE_OBJECT_DATA,
	SS_SUB_CMD_RPMB_UPDATE_OBJECT_VERIFY,
	SS_SUB_CMD_RPMB_UPDATE_OBJECT_END,

	SS_SUB_CMD_RPMB_UPDATE_OBJECT_ENTRY_START,
	SS_SUB_CMD_RPMB_UPDATE_OBJECT_ENTRY_END,

	SS_SUB_CMD_RPMB_UPDATE_OBJECT_POST,

	/*for rpmb header */
	SS_SUB_CMD_RPMB_HEADER_QUERY,
	SS_SUB_CMD_RPMB_HEADER_WRITE,
	SS_SUB_CMD_RPMB_HEADER_VERIFY,
	SS_SUB_CMD_RPMB_HEADER_DONE,
};

enum ss_file_main_back {
	SS_MAIN_OBJECT_FILE,
	SS_BAKE_OBJECT_FILE,
	SS_INVALID_OBJECT_FILE,
};

#ifdef CONFIG_TEE_LIBRARY_PROVISION
enum {
	LIBPROV_LIBRARY_GET = 0,
	LIBPROV_LIBRARY_FREE,
};
#endif

#define TEE_STORAGE_PRIVATE         0x00000001
#define TEE_STORAGE_RPMB            0x00000002
#define MAX_FILE_OBJECT_SIZE    (128*1024)
#define MAX_RPMB_OBJECT_SIZE    (4*1024)
#define OBJ_HEADER_SIZE					(256)

#define RPMB_HEADER_STATE_ERROR         (-1)
#define RPMB_HEADER_STATE_OK            (0)
#define RPMB_HEADER_STATE_UPDATE        (1)

#define SS_PATH_PREFIX		"tee/storage/"
#define SS_FILE_MAIN_EXT	".dat"
#define SS_FILE_BACK_EXT	".bak"

#define HASH_SIZE		32
#define HASH_NAME_SIZE		((HASH_SIZE*2)+1)
#define OBJECT_NAME_LEN		((HASH_SIZE*2) + sizeof(SS_FILE_MAIN_EXT))
#define FILE_NAME_SIZE		256

/* 1217 defined in tzdev */
/* 1220 rollbacked tzdev */
#define ssdev_scm_watch(devid, cmdid, p0, p1)   \
	tzdev_scm_watch((0), (devid), (cmdid), (p0), (p1))

static struct ss_wsm ss_wsm_channel;
#ifdef CONFIG_TEE_LIBRARY_PROVISION
static struct ss_wsm libprov_wsm_channel;
#endif

#define ENUM_MAGIC 0x454E554D
struct enum_node {
	unsigned int magic;
	unsigned char hash_name[HASH_SIZE];
};

#define OBJECT_FILE_PATH "/opt/usr/apps/tee/storage/"
#define ssdev_create_filename(_buf, _sz, _ext, _p, hash_str)	\
	do {															\
		convert_dirname_to_str((unsigned char *)p + 16, hash_str);	\
		snprintf(_buf, _sz, "%s/%s%08x%08x%08x%08x/.%s%s",		\
				tzpath_buf, SS_PATH_PREFIX,								\
				*_p, *(_p + 1), *(_p + 2), *(_p + 3),					\
				hash_str, _ext);										\
	} while (0)

#define ssdev_get_dirname(_buf, _sz, _p)						\
	do {															\
		snprintf(_buf, _sz, "%s/%s%08x%08x%08x%08x/",			\
				tzpath_buf, SS_PATH_PREFIX,						\
				*_p, *(_p + 1), *(_p + 2), *(_p + 3));			\
	} while (0)

static unsigned char char_to_hexdigit(unsigned char ch)
{
	if (ch >= 'a' && ch <= 'f')
		return ch - 'a' + 10;
	else if (ch >= '0' && ch <= '9')
		return ch - '0';
	else if (ch >= 'A' && ch <= 'F')
		return ch - 'A' + 10;
	else
		return ch;
}
static void convert_dirname_to_hexdigit(unsigned char *in, unsigned char *out)
{
	int i;
	int idx = 0;
	unsigned char hn, ln;

	for (i = 1; in[i] != '.' && i < (HASH_NAME_SIZE - 1); i += 2) {
		hn = char_to_hexdigit(in[i]);
		ln = char_to_hexdigit(in[i + 1]);
		out[idx++] = (hn << 4) | ln;
	}
}

static unsigned char hexdigit_to_char(unsigned char ch)
{
	if (ch >= 0x0a)
		return ch + 'a' - 10;
	else
		return ch + '0';
}

static void convert_dirname_to_str(unsigned char *in, unsigned char *out)
{
	int i;
	int idx = 0;

	for (i = 0; i < HASH_SIZE; i++) {
		out[idx++] = hexdigit_to_char((in[i] >> 4));
		out[idx++] = hexdigit_to_char((in[i] & 0x0F));
	}
}

static inline void lock_dir_entry(struct dentry *dentry)
{
	if (dentry == NULL)
		return;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 7, 0))
	inode_lock_nested(dentry->d_inode, I_MUTEX_PARENT);
#else
	mutex_lock_nested(&(dentry->d_inode->i_mutex), I_MUTEX_PARENT);
#endif
}
static inline void unlock_dir_entry(struct dentry *dentry)
{
	if (dentry == NULL)
		return;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 7, 0))
	inode_unlock(dentry->d_inode);
#else
	mutex_unlock(&(dentry->d_inode->i_mutex));
#endif
}

static void ssdev_query_object(NSRPCTransaction_t *tsx)
{
	tzlog_print(TZLOG_DEBUG,
		    "Query object of requested storage %d, RPMB object exists = %d\n",
		    nsrpc_get_arg(tsx, 0), nsrpc_get_arg(tsx, 1));
	/*
	 *  Transaction arguments:
	 *  0 - requested type
	 *  1 - true if RPMB object of this type exists
	 */

	if (nsrpc_get_arg(tsx, 0) == TEE_STORAGE_PRIVATE) {
		/*PATH + 64 Bytes UUID hash name + ".dat" + NULL */
		char file_main[FILE_NAME_SIZE] = { 0 };
		char file_back[FILE_NAME_SIZE] = { 0 };
		unsigned char hash_name[HASH_NAME_SIZE] = { 0 };
		int *p = NULL;
		int file_size;

		p = (int *)nsrpc_payload_ptr(tsx);

		ssdev_create_filename(file_main, sizeof(file_main),
				      SS_FILE_MAIN_EXT, p, hash_name);

		ssdev_create_filename(file_back, sizeof(file_back),
				      SS_FILE_BACK_EXT, p, hash_name);

		tzlog_print(TZLOG_DEBUG, "query object, file_main:%s\n",
			    file_main);

		file_size = ss_file_object_size(file_main);
		if (file_size > 0) {
			nsrpc_set_arg(tsx, 0, TEE_STORAGE_PRIVATE);
			tzlog_print(TZLOG_DEBUG,
				    " Same object id in main File.\n");
		} else {
			file_size = ss_file_object_size(file_back);
			if (file_size > 0) {
				nsrpc_set_arg(tsx, 0, TEE_STORAGE_PRIVATE);
				tzlog_print(TZLOG_DEBUG,
					    " Same object id in back File.\n");
			} else {
				nsrpc_set_arg(tsx, 0, 0);
				tzlog_print(TZLOG_DEBUG,
					    " object id does not found in normal world.\n");
			}
		}
	} else if (nsrpc_get_arg(tsx, 0) == TEE_STORAGE_RPMB) {
		if (nsrpc_get_arg(tsx, 1)) {
			nsrpc_set_arg(tsx, 0, TEE_STORAGE_RPMB);
			tzlog_print(TZLOG_DEBUG, " Same object id in RPMB.\n");
		} else {
			nsrpc_set_arg(tsx, 0, 0);
			tzlog_print(TZLOG_DEBUG,
				    " object id does not found in normal world.\n");
		}
	} else {
		tzlog_print(TZLOG_DEBUG, " Query incorrect object type.\n");
		nsrpc_set_arg(tsx, 0, 0);
	}

	nsrpc_complete(tsx, 0);
}

static int ssdev_file_copy_object(char *dest, char *src)
{
	int size = 0;
	int read_size = 0;
	int write_size = 0;
	char *buf = NULL;

	ss_file_delete_object(dest);

	size = ss_file_object_size(src);
	if (size <= 0 || size > (MAX_FILE_OBJECT_SIZE + OBJ_HEADER_SIZE)) {
		tzlog_print(TZLOG_ERROR,
			    "Failed to get object size: size =  %d.\n", size);
		return -1;
	}

	buf = vmalloc(size);
	if (NULL == buf) {
		tzlog_print(TZLOG_ERROR,
			    "Failed to malloc %d Bytes memory with vmalloc.\n",
			    size);
		return -ENOMEM;
	}

	read_size = ss_file_read_object(src, buf, size, 0);
	if (read_size != size) {
		vfree(buf);
		tzlog_print(TZLOG_ERROR, "Failed to get object data from %s.\n",
			    src);
		return -1;
	}

	write_size = ss_file_create_object(dest, buf, size);
	if (write_size != size) {
		vfree(buf);
		tzlog_print(TZLOG_ERROR, "Failed to write object data to %s.\n",
			    dest);
		return -1;
	}

	vfree(buf);
	buf = NULL;

	return 0;
}

static void ssdev_file_load_object(NSRPCTransaction_t *tsx)
{
	char file_main[FILE_NAME_SIZE] = { 0 };
	char file_back[FILE_NAME_SIZE] = { 0 };
	char hash_name[HASH_NAME_SIZE] = { 0 };
	int index;
	ssize_t obj_size;
	char *paths[] = { file_main, file_back };
	int *p = (int *)nsrpc_payload_ptr(tsx);

	ssdev_create_filename(file_main, sizeof(file_main),
			      SS_FILE_MAIN_EXT, p, hash_name);

	ssdev_create_filename(file_back, sizeof(file_back),
			      SS_FILE_BACK_EXT, p, hash_name);

	tzlog_print(TZLOG_DEBUG, "load object, file_main:%s\n", file_main);

	for (index = 0; index < 2; ++index) {
		tzlog_print(TZLOG_DEBUG, "Check if file exists '%s'\n",
			    paths[index]);

		if (ss_file_object_exist(paths[index]) != 1) {
			tzlog_print(TZLOG_DEBUG, "File '%s' doesn't exist\n",
				    paths[index]);
			continue;
		}

		obj_size = ss_file_object_size(paths[index]);
		if ((obj_size <= 0) ||
		    (obj_size > (MAX_FILE_OBJECT_SIZE + OBJ_HEADER_SIZE))) {
			tzlog_print(TZLOG_DEBUG,
				    "Failed to obtain proper object size. obj_size = %zd\n",
				    obj_size);
			continue;
		}

		tzlog_print(TZLOG_DEBUG, "Size for %s id %zd bytes\n",
			    paths[index], obj_size);

		nsrpc_set_arg(tsx, 0, obj_size);
		nsrpc_complete(tsx, 0);
		return;
	}

	tzlog_print(TZLOG_ERROR, "File '%s' doesn't exist\n", paths[0]);
	nsrpc_complete(tsx, -ENOENT);
}

struct po_list {
	unsigned char name[FILE_NAME_SIZE];
	struct list_head list;
};

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 11, 0))
struct po_dir_ctx {
	struct dir_context ctx;
	struct list_head names;
};

static int fill_po_list(struct dir_context *__ctx, const char *name,
			int namelen, loff_t offset, u64 ino,
			unsigned int d_type)
{
	struct po_dir_ctx *ctx = container_of(__ctx, struct po_dir_ctx, ctx);
	struct po_list *entry;

	if (d_type != DT_REG)
		return 0;

	if (namelen != OBJECT_NAME_LEN) {
		tzlog_print(TZLOG_DEBUG, "wrong object for secure storage\n");
		return 0;
	}

	entry = kmalloc(sizeof(struct po_list), GFP_KERNEL);
	if (entry == NULL) {
		tzlog_print(TZLOG_ERROR, "kmalloc for entry is failed\n");
		return -ENOMEM;
	}
	memset(entry->name, 0, FILE_NAME_SIZE);
	memcpy(entry->name, name, strlen(name));
	list_add(&entry->list, &ctx->names);
	return 0;
}

static void ssdev_file_load_object_list(NSRPCTransaction_t *tsx)
{
	int err;
	int nsrpc_ret = 0;
	ssize_t obj_cnt = 0;
	ssize_t obj_size = 0;
	size_t wsm_size = 0;
	int *p = (int *)nsrpc_payload_ptr(tsx);
	char *wsm_buffer = (char *)ss_wsm_channel.payload
	    + nsrpc_wsm_offset(tsx, &wsm_size);
	struct dentry *dentry;
	char dir_name[FILE_NAME_SIZE] = { 0, };
	struct file *filp = NULL;
	struct enum_node *obj_list = (struct enum_node *)wsm_buffer;
	unsigned char hash_hexdigit[HASH_SIZE];
	struct po_dir_ctx ctx = {
		.ctx.actor = fill_po_list,
		.names = LIST_HEAD_INIT(ctx.names)
	};

	ssdev_get_dirname(dir_name, sizeof(dir_name), p);
	tzlog_print(TZLOG_DEBUG, ":%s, rx dirname:%s\n", __func__, dir_name);

	filp = filp_open(dir_name, O_DIRECTORY | O_RDONLY, 0);
	if (IS_ERR(filp)) {
		tzlog_print(TZLOG_ERROR, "filp_open directory failed\n");
		nsrpc_ret = -ENOENT;
		goto dir_error;
	}

	dentry = filp->f_path.dentry;
	err = iterate_dir(filp, &ctx.ctx);

	lock_dir_entry(dentry);

	while (!list_empty(&ctx.names)) {
		struct po_list *entry;

		entry = list_entry(ctx.names.next, struct po_list, list);
		if (!err) {
			if (strstr(entry->name, ".bak")) {
				tzlog_print(TZLOG_DEBUG,
					    ".bak file will be ignored\n");
			} else {
				obj_list[obj_cnt].magic = ENUM_MAGIC;
				obj_size += 4;
				convert_dirname_to_hexdigit(&entry->name[0],
							    hash_hexdigit);
				memcpy(obj_list[obj_cnt].hash_name,
				       hash_hexdigit, HASH_SIZE);
				obj_size += HASH_SIZE;
				tzlog_print(TZLOG_DEBUG,
					    "obj_idx:%d, obj_name:%s, total_obj_size:%d\n",
					    obj_cnt, &entry->name, obj_size);
				obj_cnt++;
			}

		}
		list_del(&entry->list);
		kfree(entry);
	}

	unlock_dir_entry(dentry);
	filp_close(filp, NULL);

	memcpy(wsm_buffer, obj_list, obj_size);
dir_error:
	nsrpc_set_arg(tsx, 1, obj_cnt);
	nsrpc_set_arg(tsx, 2, obj_size);
	memcpy(wsm_buffer, obj_list, obj_size);

	nsrpc_complete(tsx, nsrpc_ret);
}
#else
static int fillonedir(void *arg, const char *name, int namlen,
		      loff_t offset, u64 ino, unsigned int d_type)
{
	struct list_head *names = arg;
	struct po_list *entry;

	if (d_type != DT_REG)
		return 0;

	if (namlen != OBJECT_NAME_LEN) {
		tzlog_print(TZLOG_DEBUG, "wrong object for secure storage\n");
		return 0;
	}

	entry = kmalloc(sizeof(struct po_list), GFP_KERNEL);
	if (entry == NULL) {
		tzlog_print(TZLOG_ERROR, "kmalloc for entry is failed\n");
		return -ENOMEM;
	}
	memset(entry->name, 0, FILE_NAME_SIZE);
	memcpy(entry->name, name, strlen(name));
	list_add(&entry->list, names);
	return 0;
}

static void ssdev_file_load_object_list(NSRPCTransaction_t *tsx)
{
	int err;
	int nsrpc_ret = 0;
	ssize_t obj_cnt = 0;
	ssize_t obj_size = 0;
	size_t wsm_size = 0;
	int *p = (int *)nsrpc_payload_ptr(tsx);
	char *wsm_buffer = (char *)ss_wsm_channel.payload
		+ nsrpc_wsm_offset(tsx, &wsm_size);
	struct dentry *dentry;
	char dir_name[FILE_NAME_SIZE] = { 0, };
	struct file *filp = NULL;
	struct enum_node *obj_list;
	unsigned char hash_hexdigit[HASH_SIZE];
	LIST_HEAD(names);

	ssdev_get_dirname(dir_name, sizeof(dir_name), p);
	tzlog_print(TZLOG_DEBUG, ":%s, rx dirname:%s\n", __func__, dir_name);

	filp = filp_open(dir_name, O_DIRECTORY | O_RDONLY, 0);
	if (IS_ERR(filp)) {
		tzlog_print(TZLOG_ERROR, "filp_open directory failed\n");
		nrpc_ret = -ENOENT;
		goto dir_error;
	}

	dentry = filp->f_path.dentry;
	err = vfs_readdir(filp, fillonedir, &names);
	obj_list = (struct enum_node *)wsm_buffer;

	lock_dir_entry(dentry);
	while (!list_empty(&names)) {
		struct po_list *entry;

		entry = list_entry(names.next, struct po_list, list);
		if (!err) {
			if (strstr(entry->name, ".bak")) {
				tzlog_print(TZLOG_DEBUG,
						".bak file will be ignored\n");
			} else {
				obj_list[obj_cnt].magic = ENUM_MAGIC;
				obj_size += 4;
				convert_dirname_to_hexdigit(&entry->name[0],
						hash_hexdigit);
				memcpy(obj_list[obj_cnt].hash_name,
						hash_hexdigit, HASH_SIZE);
				obj_size += HASH_SIZE;
				tzlog_print(TZLOG_DEBUG,
						"obj_idx:%d, obj_name:%s, total_obj_size:%d\n",
						obj_cnt, &entry->name, obj_size);
				obj_cnt++;
			}
		}
		list_del(&entry->list);
		kfree(entry);
	}
	unlock_dir_entry(dentry);
	filp_close(filp, NULL);

	memcpy(wsm_buffer, obj_list, obj_size);
dir_error:
	nsrpc_set_arg(tsx, 1, obj_cnt);
	nsrpc_set_arg(tsx, 2, obj_size);
	nsrpc_complete(tsx, nsrpc_ret);
}
#endif

static void ssdev_file_read_data(NSRPCTransaction_t *tsx)
{

	char file_main[FILE_NAME_SIZE] = { 0 };
	char hash_name[HASH_NAME_SIZE] = { 0 };

	size_t wsm_size = 0;
	void *wsm_buffer = (char *)ss_wsm_channel.payload
	    + nsrpc_wsm_offset(tsx, &wsm_size);

	int *p = (int *)nsrpc_payload_ptr(tsx);
	int read_size;

	if (wsm_size < nsrpc_get_arg(tsx, 0)) {
		nsrpc_complete(tsx, -E2BIG);
		return;
	}

	ssdev_create_filename(file_main, sizeof(file_main),
			      SS_FILE_MAIN_EXT, p, hash_name);

	tzlog_print(TZLOG_DEBUG,
		    "Read file data '%s' at offset %d and length %d\n",
		    file_main, nsrpc_get_arg(tsx, 0), nsrpc_get_arg(tsx, 1));

	read_size = ss_file_read_object(file_main,
					wsm_buffer,
					nsrpc_get_arg(tsx, 0),
					nsrpc_get_arg(tsx, 1));

	if (read_size != nsrpc_get_arg(tsx, 0)) {
		tzlog_print(TZLOG_ERROR, "Unable to read file data (%d)\n",
			    read_size);

		nsrpc_complete(tsx, -EIO);
	} else {
		tzlog_print(TZLOG_DEBUG, "File data read complete\n");
#ifdef _STORAGE_DEBUG_
		print_hex_dump(KERN_DEBUG, "file data: ", DUMP_PREFIX_ADDRESS,
			       16, 4, wsm_buffer, wsm_size, 1);
#endif
		nsrpc_complete(tsx, 0);
	}
}

static void ssdev_file_delete_file(NSRPCTransaction_t *tsx)
{
	ss_file_delete_object((char *)nsrpc_payload_ptr(tsx));
	nsrpc_complete(tsx, 0);
}

static void ssdev_file_create_data(NSRPCTransaction_t *tsx)
{
	/*PATH + 32 Bytes UUID hash + ".dat" + NULL */
	char file_main[FILE_NAME_SIZE] = { 0 };
	char file_back[FILE_NAME_SIZE] = { 0 };
	char hash_name[HASH_NAME_SIZE] = { 0 };
	int *p = (int *)nsrpc_payload_ptr(tsx);
	size_t wsm_size = 0;
	int ret;
	int file_size;
	void *wsm_buffer = (char *)ss_wsm_channel.payload + nsrpc_wsm_offset(tsx,
							      &wsm_size);

	ssdev_get_dirname(file_main, sizeof(file_main), p);
	ret = tzlog_create_dir(file_main);
	if (!(ret == 0 || ret == -EEXIST)) {
		tzlog_print(TZLOG_ERROR, "create dir failed by:%d\n", ret);
		return;
	}

	ssdev_create_filename(file_main, sizeof(file_main),
			      SS_FILE_MAIN_EXT, p, hash_name);

	ssdev_create_filename(file_back, sizeof(file_back),
			      SS_FILE_BACK_EXT, p, hash_name);

	tzlog_print(TZLOG_DEBUG, "Create data, file_main '%s'\n", file_main);

	file_size = ss_file_object_size(file_main);
	if (file_size > 0) {
		tzlog_print(TZLOG_DEBUG, "Copy main to bak file\n");

		ret = ssdev_file_copy_object(file_back, file_main);
		if (ret < 0) {
			tzlog_print(TZLOG_ERROR,
				    "Failed to make bak file. result : %d\n",
				    ret);
			nsrpc_complete(tsx, -EIO);
			return;
		}
	}

	tzlog_print(TZLOG_DEBUG,
		    "create file object '%s' with buffer %p and size %zd\n",
		    file_main, wsm_buffer, wsm_size);
#ifdef _STORAGE_DEBUG_
	print_hex_dump(KERN_DEBUG, "file data: ", DUMP_PREFIX_ADDRESS, 16, 4,
		       wsm_buffer, wsm_size, 1);
#endif
	ret = ss_file_create_object(file_main, wsm_buffer, wsm_size);

	if (ret != wsm_size) {
		tzlog_print(TZLOG_ERROR,
			    "Can't create file '%s' - error = %d\n",
			    file_main, ret);
		nsrpc_complete(tsx, -EIO);
	} else {
		tzlog_print(TZLOG_DEBUG, "Written %d bytes to '%s'\n",
			    ret, file_main);
		nsrpc_complete(tsx, 0);
	}
}

static void ssdev_file_append_data(NSRPCTransaction_t *tsx)
{
	/*PATH + 32 Bytes UUID hash + ".dat" + NULL */
	char file_main[FILE_NAME_SIZE] = { 0 };
	char hash_name[HASH_NAME_SIZE] = { 0 };
	int *p = (int *)nsrpc_payload_ptr(tsx);
	size_t wsm_size = 0;
	int ret;
	void *wsm_buffer = (char *)ss_wsm_channel.payload
	    + nsrpc_wsm_offset(tsx, &wsm_size);

	ssdev_create_filename(file_main, sizeof(file_main),
			      SS_FILE_MAIN_EXT, p, hash_name);

	tzlog_print(TZLOG_DEBUG,
		    "append file object '%s' with buffer %p and size %zd\n",
		    file_main, wsm_buffer, wsm_size);
#ifdef _STORAGE_DEBUG_
	print_hex_dump(KERN_DEBUG, "file data: ", DUMP_PREFIX_ADDRESS, 16, 4,
		       wsm_buffer, wsm_size, 1);
#endif
	ret = ss_file_append_object(file_main, wsm_buffer, wsm_size);

	if (ret != wsm_size) {
		tzlog_print(TZLOG_ERROR,
			    "Can't append to file '%s' - error = %d\n",
			    file_main, ret);

		nsrpc_complete(tsx, -EIO);
	} else {
		tzlog_print(TZLOG_DEBUG, "Appended %d bytes to '%s'\n", ret,
			    file_main);

		nsrpc_complete(tsx, 0);
	}
}

static void ssdev_file_delete_data(NSRPCTransaction_t *tsx)
{
	/*PATH + 32 Bytes UUID hash + ".dat" + NULL */
	char file_main[FILE_NAME_SIZE] = { 0 };
	char file_back[FILE_NAME_SIZE] = { 0 };
	char hash_name[HASH_NAME_SIZE] = { 0 };
	int *p = (int *)nsrpc_payload_ptr(tsx);

	ssdev_create_filename(file_main, sizeof(file_main),
			      SS_FILE_MAIN_EXT, p, hash_name);

	ssdev_create_filename(file_back, sizeof(file_back),
			      SS_FILE_BACK_EXT, p, hash_name);

	tzlog_print(TZLOG_DEBUG, "delete data, file_main:%s\n", file_main);

	ss_file_delete_object(file_main);
	ss_file_delete_object(file_back);

	nsrpc_complete(tsx, 0);
}

#ifndef CONFIG_SECOS_NO_RPMB
static void ssdev_rpmb_get_write_counter(NSRPCTransaction_t *tsx)
{
#if defined(CONFIG_MMC)
	u32 write_counter = 0;
	int ret = ss_rpmb_get_wctr(&write_counter);

	if (ret < 0) {
		tzlog_print(TZLOG_ERROR,
			    "Can't get RPMB write counter value - error %d\n",
			    ret);

		nsrpc_complete(tsx, -EIO);
	} else {
		tzlog_print(TZLOG_DEBUG, "Got RPMB write counter %u\n",
			    write_counter);

		nsrpc_set_arg(tsx, 0, write_counter);
		nsrpc_complete(tsx, 0);
	}
#else
	nsrpc_complete(tsx, -EIO);
#endif
}

static void ssdev_rpmb_get_partition_size(NSRPCTransaction_t *tsx)
{
#if defined(CONFIG_MMC)
	u32 size = 0;
	int ret = ss_rpmb_get_partition_size(&size);

	if (ret < 0) {
		tzlog_print(TZLOG_ERROR,
			    "Can't get RPMB Partition size. error %d\n",
			    ret);

		nsrpc_complete(tsx, -EIO);
	} else {
		tzlog_print(TZLOG_DEBUG, "Got RPMB Partition size : %u\n",
			    size);

		nsrpc_set_arg(tsx, 0, size);
		nsrpc_complete(tsx, 0);
	}
#else
	nsrpc_complete(tsx, -EIO);
#endif
}


static void ssdev_rpmb_load_frames(NSRPCTransaction_t *tsx)
{
#if defined(CONFIG_MMC)
	const size_t blk_nums = nsrpc_get_arg(tsx, 0);
	const size_t object_size = blk_nums * RPMB_SECOTR;
	const size_t piece_size = RPMB_SECOTR * RPMB_READ_BLOCKS_UNIT;
	const size_t count = (object_size + piece_size - 1) / piece_size;
	const size_t extra_size = object_size - piece_size * (count - 1);
	size_t piece;
	size_t pkg_size;
	size_t start_blk;
	size_t total_frames_bytes;

	uint8_t *nonce_ptr;
	uint8_t *frame_ptr;
	size_t wsm_size = 0;
	void *wsm_buffer =
	    (char *)ss_wsm_channel.payload + nsrpc_wsm_offset(tsx,
							      &wsm_size);

	start_blk = nsrpc_get_arg(tsx, 1);

	tzlog_print(TZLOG_DEBUG,
			"Load RPMB %zd blocks. Object size = %zd, piece size = %zd,\
			piece count = %zd, extra = %zd\n",
			blk_nums, object_size, piece_size, count, extra_size);

	total_frames_bytes = sizeof(struct rpmb_frame) * count + object_size;

	tzlog_print(TZLOG_DEBUG,
			"Total WSM frame bytes should be %zd, nonce should be %d bytes\n",
			total_frames_bytes, 16 * count);

	if (wsm_size != ((16 * count) + total_frames_bytes)) {
		tzlog_print(TZLOG_ERROR,
				"WSM size mismatch for RPMB frames - %zd, should be %zd\n",
				wsm_size,
				(size_t) ((16 * count) + total_frames_bytes));

		nsrpc_complete(tsx, -EINVAL);
		return;
	}

	nonce_ptr = wsm_buffer;
	frame_ptr = nonce_ptr + 16 * count;

	for (piece = 0; piece < count; piece++) {
		struct rpmb_frame *frame = (struct rpmb_frame *)frame_ptr;

		if (piece == (count - 1))
			pkg_size = extra_size;
		else
			pkg_size = piece_size;

		tzlog_print(TZLOG_DEBUG,
			    "Load piece %zd of %zd, with pkg_size = %zd\n",
			    piece, count, pkg_size);

		frame_ptr += sizeof(struct rpmb_frame) + pkg_size;

		memset(frame, 0, sizeof(struct rpmb_frame));
		memcpy(frame->nonce, nonce_ptr, 16);
		frame->addr =
		    cpu_to_be16(start_blk + RPMB_READ_BLOCKS_UNIT * piece);

		if (ss_rpmb_read_block(frame, (u8 *) (frame + 1), pkg_size) !=
		    pkg_size) {
			tzlog_print(TZLOG_ERROR,
				    "Failed to read %zd Bytes data from rpmb.\n",
				    pkg_size);
			nsrpc_complete(tsx, -EIO);
			return;
		}
	}

	tzlog_print(TZLOG_DEBUG,
		    "Complete transaction after reading RPMB blocks\n");

	nsrpc_complete(tsx, 0);
#else
	nsrpc_complete(tsx, -EIO);
#endif /* defined(CONFIG_MMC) */
}

static void ssdev_rpmb_store_frames(NSRPCTransaction_t *tsx)
{
#if defined(CONFIG_MMC)
	const size_t blk_nums = nsrpc_get_arg(tsx, 0);
	size_t wsm_size = 0;
	void *wsm_buffer =
	    (char *)ss_wsm_channel.payload + nsrpc_wsm_offset(tsx,
							      &wsm_size);
	u32 write_counter;
	int ret;
	struct rpmb_frame *frame;

	const size_t total_frames_bytes = sizeof(struct rpmb_frame) * blk_nums;

	if (wsm_size != total_frames_bytes) {
		tzlog_print(TZLOG_ERROR,
			    "WSM size mismatch for RPMB frames - %zd, should be %zd\n",
			    wsm_size, total_frames_bytes);

		nsrpc_complete(tsx, -EINVAL);
		return;
	}

	ret = ss_rpmb_get_wctr(&write_counter);
	if (ret < 0) {
		tzlog_print(TZLOG_ERROR, "Can't get write counter - %d\n", ret);
		nsrpc_complete(tsx, -EIO);
		return;
	}

	nsrpc_set_arg(tsx, 0, write_counter);

	frame = wsm_buffer;

	if ((blk_nums * RPMB_SECOTR) !=
	    ss_rpmb_write_block(frame, blk_nums * RPMB_SECOTR)) {
		tzlog_print(TZLOG_ERROR,
			    "Failed to write %d Bytes data to rpmb.\n",
			    RPMB_SECOTR);
		nsrpc_complete(tsx, -EIO);
		return;
	}

	ret = ss_rpmb_get_wctr(&write_counter);
	if (ret < 0) {
		tzlog_print(TZLOG_ERROR, "Can't get write counter - %d\n", ret);
		nsrpc_complete(tsx, -EIO);
		return;
	}

	nsrpc_set_arg(tsx, 1, write_counter);

	nsrpc_complete(tsx, 0);
#else
	nsrpc_complete(tsx, -EIO);
#endif /* defined(CONFIG_MMC) */
}
#endif /*CONFIG_SECOS_NO_RPMB */

int storage_path_init(void)
{
	int ret;

	ret = tzpath_fullpath_create(SS_PATH_PREFIX);
	if (ret != 0)
		tzlog_print(K_ERR, "Failed to create secure storage path\n");

	return ret;
}

int storage_register_wsm(void)
{
	int ret;
	struct ss_wsm *wsm_page = &ss_wsm_channel;
	void *vmem;

	mutex_init(&wsm_page->wsm_lock);
	mutex_lock(&wsm_page->wsm_lock);

	wsm_page->wsm_id = -1;

	vmem = vmalloc(SS_WSM_SIZE);

	if (!vmem)
		panic("Unable to allocate storage driver WSM");

	memset(vmem, 0, SS_WSM_SIZE);

	wsm_page->payload = vmem;

	ret =
	    tzwsm_register_kernel_memory(wsm_page->payload, SS_WSM_SIZE,
					 GFP_KERNEL);

	if (ret < 0)
		panic("Unable to register WSM buffer in Secure Kernel\n");

	wsm_page->wsm_id = ret;
	wsm_page->max_size = SS_WSM_SIZE;

	mutex_unlock(&wsm_page->wsm_lock);
	return 0;
}

void ssdev_handler(NSRPCTransaction_t *txn_object)
{
	uint32_t command = nsrpc_get_command(txn_object);

	tzlog_print(TZLOG_DEBUG, "Received command %d\n", command);

	switch (command) {
	case SS_CMD_REGISTER_WSM:
		nsrpc_set_arg(txn_object, 0, ss_wsm_channel.wsm_id);
		nsrpc_set_arg(txn_object, 1, ss_wsm_channel.max_size);
		nsrpc_complete(txn_object, 0);
		break;
	case SS_CMD_QUERY_OBJECT:
		ssdev_query_object(txn_object);
		break;
	case SS_CMD_FILE_LOAD_OBJECT:
		ssdev_file_load_object(txn_object);
		break;
	case SS_CMD_FILE_LOAD_OBJECT_LIST:
		ssdev_file_load_object_list(txn_object);
		break;
	case SS_CMD_FILE_READ_DATA:
		ssdev_file_read_data(txn_object);
		break;
	case SS_CMD_FILE_DELETE_DATA:
		ssdev_file_delete_data(txn_object);
		break;
	case SS_CMD_FILE_CREATE_DATA:
		ssdev_file_create_data(txn_object);
		break;
	case SS_CMD_FILE_APPEND_DATA:
		ssdev_file_append_data(txn_object);
		break;
	case SS_CMD_FILE_DELETE_FILE:
		ssdev_file_delete_file(txn_object);
		break;
#ifndef CONFIG_SECOS_NO_RPMB
	case SS_CMD_RPMB_GET_WRITE_COUNTER:
		ssdev_rpmb_get_write_counter(txn_object);
		break;
	case SS_CMD_RPMB_LOAD_FRAMES:
		ssdev_rpmb_load_frames(txn_object);
		break;
	case SS_CMD_RPMB_WRITE_FRAMES:
		ssdev_rpmb_store_frames(txn_object);
		break;
	case SS_CMD_RPMB_GET_PARTITION_SIZE:
		ssdev_rpmb_get_partition_size(txn_object);
		break;
#endif /* CONFIG_SECOS_NO_RPMB */
	default:
		tzlog_print(TZLOG_WARNING, "Received unsupported command %x\n",
			    command);
		nsrpc_complete(txn_object, -EINVAL);
		break;
	}

	tzlog_print(TZLOG_DEBUG, "Finished handling command %d\n", command);
}

#endif /* CONFIG_SECOS_NO_SECURE_STORAGE */

#ifdef CONFIG_TEE_LIBRARY_PROVISION
int libprov_register_wsm(void *addr, size_t size)
{
	struct ss_wsm *wsm_page = &libprov_wsm_channel;
	int ret;

	if (!addr) {
		tzlog_print(TZLOG_ERROR,
			    "Libprov WSM: wrong buffer address!\n");
		return -EINVAL;
	}

	mutex_init(&wsm_page->wsm_lock);
	mutex_lock(&wsm_page->wsm_lock);

	wsm_page->wsm_id = -1;

	/* TODO: use USER memory instead of KERNEL */
	ret = tzwsm_register_kernel_memory(addr, size, GFP_KERNEL);
	if (ret < 0)
		panic("Unable to register WSM buffer in Secure Kernel\n");

	wsm_page->wsm_id = ret;
	wsm_page->payload = addr;
	wsm_page->max_size = size;

	mutex_unlock(&wsm_page->wsm_lock);

	return 0;
}

#define LIBRARY_SEARCH_PATH	"/usr/apps/teelib"
#define MAX_PATH_LEN		256

void *libprov_load_file(const char *libname, size_t *ret_size)
{
	char path[MAX_PATH_LEN];
	unsigned int filesize, offset = 0, read_size;
	struct trm_ta_image *ta_image = NULL;
	struct file *file;

	*ret_size = 0;

	snprintf(path, MAX_PATH_LEN, "%s/%s", LIBRARY_SEARCH_PATH, libname);

	file = filp_open(path, O_RDONLY, 0);
	if (IS_ERR(file)) {
		tzlog_print(TZLOG_ERROR, "Can't open file: %s\n", path);
		return NULL;
	}

	filesize = vfs_llseek(file, 0, SEEK_END);
	if (!filesize) {
		tzlog_print(TZLOG_ERROR, "Empty file!\n");
		goto end;
	}

	ta_image = vmalloc(filesize + sizeof(struct trm_ta_image));
	if (!ta_image) {
		tzlog_print(TZLOG_ERROR,
			    "Can't vmalloc memory for file size: %lu\n",
			    filesize);
		goto end;
	}

	read_size =
	    kernel_read(file, offset, (char *)&ta_image->buffer, filesize);
	if (read_size != filesize) {
		tzlog_print(TZLOG_ERROR,
			    "ERROR: (%s) - read only %llu of %llu\n", path,
			    read_size, filesize);
		vfree(ta_image);
		ta_image = NULL;
		goto end;
	}

	ta_image->type = 0;

	*ret_size = filesize;
end:
	filp_close(file, NULL);
	return ta_image;
}

int libprov_handler(NSRPCTransaction_t *txn_object)
{
	static void *filebuff;
	ssize_t filesize;
	uint32_t command;
	char *libpath;
	int memid;

	command = nsrpc_get_command(txn_object);
	switch (command) {
	case LIBPROV_LIBRARY_GET:
		libpath = nsrpc_payload_ptr(txn_object);
		if (!libpath) {
			tzlog_print(TZLOG_ERROR,
				    "Received LIBRARY GET command for BAD path!\n");
			return -EINVAL;
		}

		filebuff = libprov_load_file(libpath, &filesize);
		if (!filebuff) {
			tzlog_print(TZLOG_ERROR,
				    "LIBRARY GET: Can't read local file!\n");
			return -ENOENT;
		}

		if (libprov_register_wsm(filebuff, filesize)) {
			tzlog_print(TZLOG_ERROR,
				    "LIBRARY GET: Can't register file buffer!\n");
			vfree(filebuff);
			return -ENOMEM;
		}

		nsrpc_set_arg(txn_object, 0, libprov_wsm_channel.wsm_id);
		nsrpc_set_arg(txn_object, 1, libprov_wsm_channel.max_size);

		break;
	case LIBPROV_LIBRARY_FREE:
		if (!filebuff)
			break;

		memid = nsrpc_get_arg(txn_object, 0);
		tzwsm_unregister_kernel_memory(memid);
		vfree(filebuff);
		filebuff = NULL;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}
#endif /* CONFIG_TEE_LIBRARY_PROVISION */
