#include <dev/agp/agpvar.h>

struct drm_agp_head {
	struct list_head	memory;
	struct agp_info		agp_info;
	unsigned long		base;
	int			cant_use_aperture;
	int			agp_mtrr;
};
