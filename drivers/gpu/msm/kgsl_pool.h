/* Copyright (c) 2016, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#ifndef __KGSL_POOL_H
#define __KGSL_POOL_H

#include <linux/mm_types.h>
#include "kgsl_sharedmem.h"

static inline unsigned int
kgsl_gfp_mask(unsigned int page_order)
{
	unsigned int gfp_mask = __GFP_HIGHMEM;

	if (page_order > 0)
		gfp_mask |= __GFP_COMP | __GFP_NORETRY |
			__GFP_NO_KSWAPD | __GFP_NOWARN;
	else
		gfp_mask |= GFP_KERNEL;

	if (kgsl_sharedmem_get_noretry() == true)
		gfp_mask |= __GFP_NORETRY | __GFP_NOWARN;

	return gfp_mask;
}

void kgsl_pool_free_sgt(struct sg_table *sgt);
#ifdef VENDOR_EDIT
//Deliang.Peng@MultiMedia.Display.GPU.Perf, 2017/2/20,
//add for GPU performance
void kgsl_pool_free_pages(struct page **pages, unsigned int page_count);
#endif /**VENDOR_EDIT*/
void kgsl_init_page_pools(void);
void kgsl_exit_page_pools(void);
#ifdef VENDOR_EDIT
//Deliang.Peng@MultiMedia.Display.GPU.Perf, 2017/2/20,
//add for GPU performance
int kgsl_pool_alloc_page(int *page_size, struct page **pages,
            unsigned int pages_len, unsigned int *align);
#else
int kgsl_pool_alloc_page(int page_size, struct page **pages,
                        unsigned int pages_len);
#endif /**VENDOR_EDIT*/
void kgsl_pool_free_page(struct page *p);
#endif /* __KGSL_POOL_H */

