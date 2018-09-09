#ifndef __DRM_LINUX_CAPABILITY_H__
#define	__DRM_LINUX_CAPABILITY_H__

enum linux_capabilities {
	CAP_SYS_ADMIN,
	CAP_SYS_NICE
};
bool capable(enum linux_capabilities cap);

#endif
