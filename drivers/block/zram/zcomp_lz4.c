/*
 * Copyright (C) 2014 Sergey Senozhatsky.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version
 * 2 of the License, or (at your option) any later version.
 */

#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/lz4.h>

#include "zcomp_lz4.h"

static void *zcomp_lz4_create(void)
{
#ifndef VENDOR_EDIT
//yixue.ge@bsp.drv 20160810 modify for kernel patch 40942069b040b0e2908077292a29111d20b2287d
	return kzalloc(LZ4_MEM_COMPRESS, GFP_KERNEL);
#else
	return kzalloc(LZ4_MEM_COMPRESS, GFP_NOIO);
#endif
}

static void zcomp_lz4_destroy(void *private)
{
	kfree(private);
}

static int zcomp_lz4_compress(const unsigned char *src, unsigned char *dst,
		size_t *dst_len, void *private)
{
	/* return  : Success if return 0 */
	return lz4_compress(src, PAGE_SIZE, dst, dst_len, private);
}

static int zcomp_lz4_decompress(const unsigned char *src, size_t src_len,
		unsigned char *dst)
{
	size_t dst_len = PAGE_SIZE;
	/* return  : Success if return 0 */
	return lz4_decompress_unknownoutputsize(src, src_len, dst, &dst_len);
}

struct zcomp_backend zcomp_lz4 = {
	.compress = zcomp_lz4_compress,
	.decompress = zcomp_lz4_decompress,
	.create = zcomp_lz4_create,
	.destroy = zcomp_lz4_destroy,
	.name = "lz4",
};
