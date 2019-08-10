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

#include <linux/mm.h>
#include <linux/io.h>

#include "tzdev.h"
#include "tzdev_internal.h"

void tzdev_free_watch_page(tzdev_page_handle h)
{
	__free_page((struct page *)h);
}
EXPORT_SYMBOL(tzdev_free_watch_page);

tzdev_page_handle tzdev_alloc_watch_page(void)
{
	return (tzdev_page_handle) alloc_page(GFP_ATOMIC);
}
EXPORT_SYMBOL(tzdev_alloc_watch_page);

void *tzdev_get_virt_addr(tzdev_page_handle h)
{
	return page_address((struct page *)h);
}
EXPORT_SYMBOL(tzdev_get_virt_addr);

phys_addr_t tzdev_get_phys_addr(tzdev_page_handle h)
{
	return page_to_phys((struct page *)h);
}
EXPORT_SYMBOL(tzdev_get_phys_addr);
