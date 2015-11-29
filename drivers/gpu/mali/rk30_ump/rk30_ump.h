/*
 * Copyright (C) 2015 olegvedi@gmail.com
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 */
#define USI_GET_INFO		_IOR('u', 0, struct usi_ump_mbs_info)
#define USI_FREE_ALL_BLKS	_IO('u', 1)
#define USI_ALLOC_MEM_BLK	_IOWR('u', 3, struct usi_ump_mbs)
#define USI_FREE_MEM_BLK	_IOW('u', 4, ump_secure_id)

#define UMP_MINIMUM_SIZE         4096
#define UMP_MINIMUM_SIZE_MASK    (~(UMP_MINIMUM_SIZE-1))
#define UMP_SIZE_ALIGN(x)        (((x)+UMP_MINIMUM_SIZE-1)&UMP_MINIMUM_SIZE_MASK)
#define UMP_ADDR_ALIGN_OFFSET(x) ((x)&(UMP_MINIMUM_SIZE-1))

#define USI_UMP_DRV_NAME "usi_ump"

struct usi_ump_mbs {
	unsigned int	addr;
	unsigned int	size;
	ump_secure_id	secure_id;
};

struct usi_ump_mbs_info {
	unsigned int	size_full;
	unsigned int	size_used;
};
