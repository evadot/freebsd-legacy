#ifndef DRM_OS_CONFIG_H_
#define DRM_OS_CONFIG_H_

#define COMPAT_FREEBSD32 1
#ifdef COMPAT_FREEBSD32
#define CONFIG_COMPAT 1
#endif
#ifdef notyet
#define CONFIG_MMU_NOTIFIER 1
#endif

#ifdef _KERNEL
#define	__KERNEL__
#endif


#define	CONFIG_FB	1

#define CONFIG_DRM_FBDEV_EMULATION

	 
// Let try to do without this CONFIG_LOCKDEP. Opens a can of worms.
// FreeBSD does some lock checking even without this macro.
// See $SRC/sys/compat/linuxkpi/common/include/linux/lockdep.h
// For the functions that we implement, override IS_ENABLED(CONFIG_LOCKDEP)
// by using #if IS_ENABLED(CONFIG_LOCKDEP) || defined(__FreeBSD__) in
// drm drivers
//#define CONFIG_LOCKDEP 1


// Overallocation of the fbdev buffer
// Defines the fbdev buffer overallocation in percent. Default is 100.
// Typical values for double buffering will be 200, triple buffering 300.
#define CONFIG_DRM_FBDEV_OVERALLOC 100


#endif
