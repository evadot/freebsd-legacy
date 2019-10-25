#include <linux/device.h>
#include <linux/kdev_t.h>
#include <linux/gfp.h>
#include <linux/err.h>
#include <linux/export.h>

#include <drm/drm_sysfs.h>
#include <drm/drmP.h>
#include "../../drm_internal.h"

struct cdev *
drm_sysfs_minor_alloc(struct drm_minor *minor)
{
	return (void *)0xDEADBEAF;
}

int
drm_sysfs_connector_add(struct drm_connector *connector __unused)
{
	return 0;
}

void
drm_sysfs_connector_remove(struct drm_connector *connector __unused)
{
}

void
drm_sysfs_hotplug_event(struct drm_device *dev __unused)
{
}

