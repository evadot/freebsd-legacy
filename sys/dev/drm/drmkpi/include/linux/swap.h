#ifndef __DRM_LINUX_SWAP_H__
#define __DRM_LINUX_SWAP_H__

#include <linux/types.h>

static inline long
get_nr_swap_pages(void)
{
	UNIMPLEMENTED();
	return 0;
}

static inline int
current_is_kswapd(void)
{
	UNIMPLEMENTED();
	return 0;
}

#endif /* __DRM_LINUX_SWAP_H__ */
