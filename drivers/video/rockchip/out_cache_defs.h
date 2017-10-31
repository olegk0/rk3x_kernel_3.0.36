/*------------------------------------------------------------------------------
* 2016 olegk0 <olegvedi@gmail.com>
*
*
*-----------------------------------------------------------------------------*/
#ifndef __OUT_CACHE_DEFS_H_
#define __OUT_CACHE_DEFS_H_

#define MAX_CACHE_PAGES 20

enum{
	STS_CACHE_NOINIT=-1,
	STS_CACHE_STOP=0,
};

typedef struct cache_page_prm_s{
	int             num_blk;
	unsigned long   size_blk;
    unsigned long   yuv_offs;
	unsigned long   first_paddr;
} cache_page_prm_t;

typedef struct cache_put_prm_s{
    unsigned long   PutFbPaddr;
    int             NBlk;
} cache_put_prm_t;

#define RK_FBIOSET_CACHE_INIT _IOW('r', 0, struct cache_page_prm_s)
#define RK_FBIOSET_CACHE_GPUT _IOR('r', 1, struct cache_put_prm_s)
#define RK_FBIOSET_CACHE_CTRL _IOW('r', 2, unsigned int)
#define RK_FBIOSET_CACHE_STAT _IOR('r', 2, int)

#endif