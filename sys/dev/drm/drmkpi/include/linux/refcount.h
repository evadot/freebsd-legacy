#ifndef __DRM_LINUX_REFCOUNT_H__
#define __DRM_LINUX_REFCOUNT_H__

#include <linux/atomic.h>


static inline bool
refcount_dec_and_test(atomic_t *r)
{

	return atomic_dec_and_test(r);
}


#endif /* __DRM_LINUX_REFCOUNT_H__ */
