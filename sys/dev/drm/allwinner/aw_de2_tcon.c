/*-
 * Copyright (c) 2019 Emmanuel Vadot <manu@FreeBSD.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * $FreeBSD$
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/rman.h>
#include <sys/resource.h>
#include <machine/bus.h>
#include <machine/atomic.h>

#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>
#include <dev/ofw/ofw_graph.h>

#include <dev/extres/clk/clk.h>
#include <dev/extres/hwreset/hwreset.h>

#include <drm/drm_atomic_helper.h>
#include <drm/drm_vblank.h>

#include <dev/drm/allwinner/aw_de2_tcon.h>

#include "aw_de2_tcon_if.h"
#include "aw_de2_mixer_if.h"
#include "dw_hdmi_if.h"
#include "drm_bridge_if.h"

enum tcon_model {
	A83T_TCON_TV = 1,
	A83T_TCON_LCD,
};

static struct ofw_compat_data compat_data[] = {
	{ "allwinner,sun8i-a83t-tcon-tv",	A83T_TCON_TV },
	{ "allwinner,sun8i-a83t-tcon-lcd",	A83T_TCON_LCD },
	{ NULL,					0 }
};

static struct resource_spec aw_de2_tcon_spec[] = {
	{ SYS_RES_MEMORY,	0,	RF_ACTIVE },
	{ SYS_RES_IRQ,		0,	RF_ACTIVE },
	{ -1, 0 }
};

struct aw_de2_tcon_softc {
	device_t	dev;
	struct resource	*res[2];
	void *		intrhand;
	struct mtx	mtx;

	enum tcon_model	model;
	clk_t		clk_ahb;
	clk_t		clk_tcon;
	hwreset_t	rst_lcd;
	hwreset_t	rst_lvds;

	device_t	mixer;
	device_t	outport;

	struct drm_pending_vblank_event	*event;
	struct drm_device		*drm;
	struct drm_crtc			crtc;
	struct drm_encoder		encoder;

	uint32_t	vbl_counter;

	int	attach_done;
};

#define	AW_DE2_TCON_READ_4(sc, reg)		bus_read_4((sc)->res[0], (reg))
#define	AW_DE2_TCON_WRITE_4(sc, reg, val)	bus_write_4((sc)->res[0], (reg), (val))

#define	AW_DE2_TCON_LOCK(sc)			mtx_lock(&(sc)->mtx)
#define	AW_DE2_TCON_UNLOCK(sc)			mtx_unlock(&(sc)->mtx)

static int aw_de2_tcon_probe(device_t dev);
static int aw_de2_tcon_attach(device_t dev);
static int aw_de2_tcon_detach(device_t dev);
static void aw_de2_tcon_intr(void *arg);

/* #define	TCON_DEBUG */

#ifdef TCON_DEBUG
static void aw_de2_tcon_dump_regs(struct aw_de2_tcon_softc *sc);
#endif

/*
 * VBLANK functions
 */
static int
aw_de2_tcon_enable_vblank(struct drm_crtc *crtc)
{
	struct aw_de2_tcon_softc *sc;

	sc = container_of(crtc, struct aw_de2_tcon_softc, crtc);
#ifdef TCON_DEBUG
	device_printf(sc->dev, "%s: start\n", __func__);
	aw_de2_tcon_dump_regs(sc);
#endif
	AW_DE2_TCON_WRITE_4(sc, TCON_GINT0, TCON0_GINT0_VB_EN | TCON1_GINT0_VB_EN);
#ifdef TCON_DEBUG
	aw_de2_tcon_dump_regs(sc);
	device_printf(sc->dev, "%s: done\n", __func__);
#endif
	return (0);
}

static void
aw_de2_tcon_disable_vblank(struct drm_crtc *crtc)
{
	struct aw_de2_tcon_softc *sc;

	sc = container_of(crtc, struct aw_de2_tcon_softc, crtc);
#ifdef TCON_DEBUG
	device_printf(sc->dev, "%s start\n", __func__);
	aw_de2_tcon_dump_regs(sc);
#endif
	AW_DE2_TCON_WRITE_4(sc, TCON_GINT0, 0x00);
#ifdef TCON_DEBUG
	aw_de2_tcon_dump_regs(sc);
	device_printf(sc->dev, "%s end\n", __func__);
#endif
}

static uint32_t
aw_de2_tcon_get_vblank_counter(struct drm_crtc *crtc)
{
	struct aw_de2_tcon_softc *sc;

	sc = container_of(crtc, struct aw_de2_tcon_softc, crtc);

	return (sc->vbl_counter);
}

static const struct drm_crtc_funcs aw_de2_tcon_funcs = {
	.atomic_destroy_state	= drm_atomic_helper_crtc_destroy_state,
	.atomic_duplicate_state	= drm_atomic_helper_crtc_duplicate_state,
	.destroy		= drm_crtc_cleanup,
	.page_flip		= drm_atomic_helper_page_flip,
	.reset			= drm_atomic_helper_crtc_reset,
	.set_config		= drm_atomic_helper_set_config,

	.get_vblank_counter	= aw_de2_tcon_get_vblank_counter,
	.enable_vblank		= aw_de2_tcon_enable_vblank,
	.disable_vblank		= aw_de2_tcon_disable_vblank,
};

static int
aw_crtc_atomic_check(struct drm_crtc *crtc,
    struct drm_crtc_state *state)
{

	/* Not sure we need to something here, should replace with an helper */
	return (0);
}

static void
aw_crtc_atomic_begin(struct drm_crtc *crtc,
    struct drm_crtc_state *old_state)
{
	struct aw_de2_tcon_softc *sc;
	unsigned long flags;

	sc = container_of(crtc, struct aw_de2_tcon_softc, crtc);

	if (crtc->state->event == NULL)
		return;

	spin_lock_irqsave(&crtc->dev->event_lock, flags);

	if (drm_crtc_vblank_get(crtc) != 0)
		drm_crtc_send_vblank_event(crtc, crtc->state->event);
	else
		drm_crtc_arm_vblank_event(crtc, crtc->state->event);

	crtc->state->event = NULL;
	spin_unlock_irqrestore(&crtc->dev->event_lock, flags);
}

static void
aw_crtc_atomic_flush(struct drm_crtc *crtc,
    struct drm_crtc_state *old_state)
{
	struct aw_de2_tcon_softc *sc;
	struct drm_pending_vblank_event *event = crtc->state->event;

	sc = container_of(crtc, struct aw_de2_tcon_softc, crtc);

	AW_DE2_MIXER_COMMIT(sc->mixer);

	if (event) {
		crtc->state->event = NULL;

		spin_lock_irq(&sc->drm->event_lock);
		/* 
		 * If not in page flip, arm it for later 
		 * Else send it
		 */
		if (drm_crtc_vblank_get(crtc) == 0)
			drm_crtc_arm_vblank_event(crtc, event);
		else
			drm_crtc_send_vblank_event(crtc, event);
		spin_unlock_irq(&sc->drm->event_lock);
	}
}

static void
aw_crtc_atomic_disable(struct drm_crtc *crtc,
    struct drm_crtc_state *old_state)
{
	struct aw_de2_tcon_softc *sc;
	uint32_t reg, irqflags;

	sc = container_of(crtc, struct aw_de2_tcon_softc, crtc);

	/* Disable TCON */
	AW_DE2_TCON_LOCK(sc);
	reg = AW_DE2_TCON_READ_4(sc, TCON_CTL);
	reg &= ~TCON_CTL_EN;
	AW_DE2_TCON_WRITE_4(sc, TCON_CTL, reg);
	AW_DE2_TCON_UNLOCK(sc);

	/* Disable VBLANK events */
	drm_crtc_vblank_off(crtc);

	spin_lock_irqsave(&crtc->dev->event_lock, irqflags);

	if (crtc->state->event) {
		drm_crtc_send_vblank_event(crtc, crtc->state->event);
		crtc->state->event = NULL;
	}

	spin_unlock_irqrestore(&crtc->dev->event_lock, irqflags);

}

static void
aw_crtc_atomic_enable(struct drm_crtc *crtc,
    struct drm_crtc_state *old_state)
{
	struct aw_de2_tcon_softc *sc;
	uint32_t reg;

	sc = container_of(crtc, struct aw_de2_tcon_softc, crtc);

	/* Enable TCON */
	AW_DE2_TCON_LOCK(sc);
	reg = AW_DE2_TCON_READ_4(sc, TCON_CTL);
	reg |= TCON_CTL_EN;
	AW_DE2_TCON_WRITE_4(sc, TCON_CTL, reg);
	AW_DE2_TCON_UNLOCK(sc);

	/* Enable VBLANK events */
	drm_crtc_vblank_on(crtc);
}

static void
aw_crtc_mode_set_nofb(struct drm_crtc *crtc)
{
	struct aw_de2_tcon_softc *sc;
	struct drm_display_mode *mode;
	uint32_t reg;
#ifdef TCON_DEBUG
	uint64_t freq;
#endif

	sc = container_of(crtc, struct aw_de2_tcon_softc, crtc);
	mode = &crtc->state->adjusted_mode;

#ifdef TCON_DEBUG
	device_printf(sc->dev, "%s: start\n", __func__);
	aw_de2_tcon_dump_regs(sc);
	clk_get_freq(sc->clk_tcon, &freq);
	device_printf(sc->dev, "Current freq: %lu, Changing to %lu\n", freq, mode->crtc_clock * 1000);
#endif
	clk_set_freq(sc->clk_tcon, mode->crtc_clock * 1000, CLK_SET_ROUND_ANY);
#ifdef TCON_DEBUG
	clk_get_freq(sc->clk_tcon, &freq);
	device_printf(sc->dev, "New freq: %lu\n", freq);
#endif
	AW_DE2_TCON_LOCK(sc);

	/* Clock delay, writing what u-boot left, need to figure what it is */
	reg = AW_DE2_TCON_READ_4(sc, TCON_CTL);
	reg &= ~TCON_CTL_DELAY_MASK;
	reg |= 28 << TCON_CTL_DELAY_SHIFT;
	AW_DE2_TCON_WRITE_4(sc, TCON_CTL, reg);

	/* Input resolution */
	AW_DE2_TCON_WRITE_4(sc, TCON_TIMING0,
	    TCON_TIMING0_YI(mode->crtc_vdisplay) |
	    TCON_TIMING0_XI(mode->crtc_hdisplay));

	/* Upscale resolution */
	AW_DE2_TCON_WRITE_4(sc, TCON_TIMING1,
	    TCON_TIMING1_LS_YO(mode->crtc_vdisplay) |
	    TCON_TIMING1_LS_XO(mode->crtc_hdisplay));

	/* Output resolution */
	AW_DE2_TCON_WRITE_4(sc, TCON_TIMING2,
	    TCON_TIMING2_YO(mode->crtc_vdisplay) |
	    TCON_TIMING2_XO(mode->crtc_hdisplay));

	/* Horizontal total and backporch */
	AW_DE2_TCON_WRITE_4(sc, TCON_TIMING3,
	    TCON_TIMING3_HT(mode->crtc_htotal) |
	    TCON_TIMING3_HBP(mode->crtc_htotal - mode->crtc_hsync_start));

	/* Vertical total and backporch */
	AW_DE2_TCON_WRITE_4(sc, TCON_TIMING4,
	    TCON_TIMING4_VT(mode->crtc_vtotal) |
	    TCON_TIMING4_VBP(mode->crtc_vtotal - mode->crtc_vsync_start));

	/* Hsync and Vsync length */
	AW_DE2_TCON_WRITE_4(sc, TCON_TIMING5,
	    TCON_TIMING5_HSPW(mode->crtc_hsync_end - mode->crtc_hsync_start) |
	    TCON_TIMING5_VSPW(mode->crtc_vsync_end - mode->crtc_vsync_start));

#ifdef TCON_DEBUG
	aw_de2_tcon_dump_regs(sc);
	device_printf(sc->dev, "%s: end\n", __func__);
#endif
	AW_DE2_TCON_UNLOCK(sc);
}

static const struct drm_crtc_helper_funcs aw_crtc_helper_funcs = {
	.atomic_check	= aw_crtc_atomic_check,
	.atomic_begin	= aw_crtc_atomic_begin,
	.atomic_flush	= aw_crtc_atomic_flush,
	.atomic_enable	= aw_crtc_atomic_enable,
	.atomic_disable	= aw_crtc_atomic_disable,
	.mode_set_nofb	= aw_crtc_mode_set_nofb,
};

static void aw_de2_tcon_encoder_mode_set(struct drm_encoder *encoder,
    struct drm_display_mode *mode,
    struct drm_display_mode *adj_mode)

{

	printf("%s: called\n", __func__);
}

static const struct drm_encoder_helper_funcs aw_de2_tcon_encoder_helper_funcs = {
	.mode_set = aw_de2_tcon_encoder_mode_set,
};

static const struct drm_encoder_funcs aw_de2_tcon_encoder_funcs = {
	.destroy = drm_encoder_cleanup,
};

static int
aw_de2_tcon_create_crtc(device_t dev, struct drm_device *drm,
    struct drm_plane *main_plane, struct drm_plane *cursor_plane)
{
	struct aw_de2_tcon_softc *sc;

	sc = device_get_softc(dev);
	sc->drm = drm;

	if (drm_crtc_init_with_planes(drm, &sc->crtc,
					main_plane,
					cursor_plane,
					&aw_de2_tcon_funcs,
	    NULL) != 0)
		device_printf(sc->dev,
		  "%s: drm_crtc_init_with_planes failed\n", __func__);

	drm_crtc_helper_add(&sc->crtc, &aw_crtc_helper_funcs);

	if (sc->model == A83T_TCON_LCD) {
		drm_encoder_helper_add(&sc->encoder, &aw_de2_tcon_encoder_helper_funcs);
		sc->encoder.possible_crtcs = drm_crtc_mask(&sc->crtc);
		drm_encoder_init(drm, &sc->encoder, &aw_de2_tcon_encoder_funcs,
		    DRM_MODE_ENCODER_NONE, NULL);
		DRM_BRIDGE_ADD_BRIDGE(sc->outport, &sc->encoder, drm);
	} else {
		DW_HDMI_ADD_ENCODER(sc->outport, &sc->crtc, drm);
	}

	return (0);
}

#ifdef TCON_DEBUG
static void
aw_de2_tcon_dump_regs(struct aw_de2_tcon_softc *sc)
{
	uint32_t reg;

	reg = AW_DE2_TCON_READ_4(sc, TCON_GCTL);
	device_printf(sc->dev, "GCTL: %x, TCON %sable, Gamma %sable\n", reg,
	  (reg & (1 << 31)) ? "En" : "Dis",
	  (reg & (1 << 30)) ? "En" : "Dis");

	reg = AW_DE2_TCON_READ_4(sc, TCON_GINT0);
	device_printf(sc->dev, "GINT0: %x, VBlank %sable, Line %sable\n", reg,
	  (reg & (1 << 30)) ? "En" : "Dis",
	  (reg & (1 << 28)) ? "En" : "Dis");

	reg = AW_DE2_TCON_READ_4(sc, TCON_CTL);
	device_printf(sc->dev, "CTL: %x, TCON %sable, delay: %d\n", reg,
	  (reg & (1 << 31)) ? "En" : "Dis",
	  (reg & TCON_CTL_DELAY_MASK) >> TCON_CTL_DELAY_SHIFT);

	reg = AW_DE2_TCON_READ_4(sc, TCON_TIMING0);
	device_printf(sc->dev, "TIMING0: %x, XI: %d, YI: %d\n", reg,
	  ((reg & TCON_TIMING0_XI_MASK) >> TCON_TIMING0_XI_SHIFT) + 1,
	  ((reg & TCON_TIMING0_YI_MASK) >> TCON_TIMING0_YI_SHIFT) + 1);

	reg = AW_DE2_TCON_READ_4(sc, TCON_TIMING1);
	device_printf(sc->dev, "TIMING1: %x, LS_XO: %d, LS_YO: %d\n", reg,
	  ((reg & TCON_TIMING1_LS_XO_MASK) >> TCON_TIMING1_LS_XO_SHIFT) + 1,
	  ((reg & TCON_TIMING1_LS_YO_MASK) >> TCON_TIMING1_LS_YO_SHIFT) + 1);

	reg = AW_DE2_TCON_READ_4(sc, TCON_TIMING2);
	device_printf(sc->dev, "TIMING2: %x, XO: %d, YO: %d\n", reg,
	  ((reg & TCON_TIMING2_XO_MASK) >> TCON_TIMING2_XO_SHIFT) + 1,
	  ((reg & TCON_TIMING2_YO_MASK) >> TCON_TIMING2_YO_SHIFT) + 1);

	reg = AW_DE2_TCON_READ_4(sc, TCON_TIMING3);
	device_printf(sc->dev, "TIMING3: %x, HT: %d, HBP: %d\n", reg,
	  (reg & TCON_TIMING3_HT_MASK) >> TCON_TIMING3_HT_SHIFT,
	  (reg & TCON_TIMING3_HBP_MASK) >> TCON_TIMING3_HBP_SHIFT);

	reg = AW_DE2_TCON_READ_4(sc, TCON_TIMING4);
	device_printf(sc->dev, "TIMING4: %x, VT: %d, VBP: %d\n", reg,
	  (reg & TCON_TIMING4_VT_MASK) >> TCON_TIMING4_VT_SHIFT,
	  (reg & TCON_TIMING4_VBP_MASK) >> TCON_TIMING4_VBP_SHIFT);

	reg = AW_DE2_TCON_READ_4(sc, TCON_TIMING5);
	device_printf(sc->dev, "TIMING5: %x, HSPW: %d, VSPW: %d\n", reg,
	  (reg & TCON_TIMING5_HSPW_MASK) >> TCON_TIMING5_HSPW_SHIFT,
	  (reg & TCON_TIMING5_VSPW_MASK) >> TCON_TIMING5_HSPW_SHIFT);
}
#endif /* TCON_DEBUG */

static void
aw_de2_tcon_intr(void *arg)
{
	struct aw_de2_tcon_softc *sc;
	uint32_t reg;

	sc = (struct aw_de2_tcon_softc *)arg;

	reg = AW_DE2_TCON_READ_4(sc, TCON_GINT0);
	if (reg & (TCON0_GINT0_VB_FLAG | TCON1_GINT0_VB_FLAG)) {
		/* Ack interrupts */
		AW_DE2_TCON_WRITE_4(sc, TCON_GINT0,
		    ~(TCON0_GINT0_VB_FLAG | TCON1_GINT0_VB_FLAG));

		atomic_add_32(&sc->vbl_counter, 1);
		drm_crtc_handle_vblank(&sc->crtc);
	}
}

static int
aw_de2_tcon_probe(device_t dev)
{
	enum tcon_model model;

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	model = (enum tcon_model)ofw_bus_search_compatible(dev, compat_data)->ocd_data;
	if (model == 0)
		return (ENXIO);

	switch (model) {
	case A83T_TCON_TV:
		device_set_desc(dev, "Allwinner DE2 TV TCON");
		break;
	case A83T_TCON_LCD:
		device_set_desc(dev, "Allwinner DE2 LCD TCON");
		break;
	}

	return (BUS_PROBE_DEFAULT);
}

static int
aw_de2_tcon_attach(device_t dev)
{
	struct aw_de2_tcon_softc *sc;
	phandle_t node;
	int error, endpoint;

	node = ofw_bus_get_node(dev);
	sc = device_get_softc(dev);
	sc->dev = dev;

	sc->model = (enum tcon_model)ofw_bus_search_compatible(dev, compat_data)->ocd_data;

	if (bus_alloc_resources(dev, aw_de2_tcon_spec, sc->res) != 0) {
		device_printf(dev, "cannot allocate resources for device\n");
		error = ENXIO;
		goto fail;
	}
	if (bus_setup_intr(dev, sc->res[1],
	    INTR_TYPE_MISC | INTR_MPSAFE, NULL, aw_de2_tcon_intr, sc,
	    &sc->intrhand)) {
		bus_release_resources(dev, aw_de2_tcon_spec, sc->res);
		device_printf(dev, "cannot setup interrupt handler\n");
		return (ENXIO);
	}

	mtx_init(&sc->mtx, device_get_nameunit(dev), "aw_de2_tcon", MTX_DEF);

	if ((error = clk_get_by_ofw_name(dev, node, "ahb", &sc->clk_ahb)) != 0) {
		device_printf(dev, "Cannot get ahb clock\n");
		goto fail;
	}
	if ((error = clk_enable(sc->clk_ahb)) != 0) {
		device_printf(dev, "Cannot enable ahb clock\n");
		goto fail;
	}

	if (sc->model == A83T_TCON_LCD) {
		if ((error = clk_get_by_ofw_name(dev, node, "tcon-ch0", &sc->clk_tcon)) != 0) {
			device_printf(dev, "Cannot get tcon clock\n");
			goto fail;
		}
	} else if (sc->model == A83T_TCON_TV) {
		if ((error = clk_get_by_ofw_name(dev, node, "tcon-ch1", &sc->clk_tcon)) != 0) {
			device_printf(dev, "Cannot get tcon clock\n");
			goto fail;
		}
	}
	if ((error = clk_enable(sc->clk_tcon)) != 0) {
		device_printf(dev, "Cannot enable tcon clock\n");
		goto fail;
	}
	if ((error = hwreset_get_by_ofw_name(dev, node, "lcd", &sc->rst_lcd)) != 0) {
		device_printf(dev, "Cannot get lcd reset\n");
		goto fail;
	}
	if ((error = hwreset_deassert(sc->rst_lcd)) != 0) {
		device_printf(dev, "Cannot de-assert lcd reset\n");
		goto fail;
	}

	if (sc->model == A83T_TCON_LCD) {
		if ((error = hwreset_get_by_ofw_name(dev, node, "lvds", &sc->rst_lvds)) != 0) {
			device_printf(dev, "Cannot get lvds reset\n");
			goto fail;
	}
		if ((error = hwreset_deassert(sc->rst_lvds)) != 0) {
			device_printf(dev, "Cannot de-assert lvds reset\n");
			goto fail;
		}
	}

	endpoint = 0;
	if (sc->model == A83T_TCON_TV)
		endpoint = 1;
	sc->outport = ofw_graph_get_device_by_port_ep(node, 1, endpoint);
	if (sc->outport == NULL) {
		device_printf(dev, "Cannot get remote endpoint device for port 1 and endpoint %d\n", endpoint);
		error = ENXIO;
		goto fail;
	}

	/* Register ourself */
	OF_device_register_xref(OF_xref_from_node(node), dev);

	/* Clear and disable interrupts */
	AW_DE2_TCON_WRITE_4(sc, TCON_GINT0, 0x00);

	/* Enable module */
	AW_DE2_TCON_WRITE_4(sc, TCON_GCTL, TCON_GCTL_EN);

#ifdef TCON_DEBUG
	aw_de2_tcon_dump_regs(sc);
#endif

	return (0);

fail:
	aw_de2_tcon_detach(dev);
	return (error);
}

static void
aw_de2_tcon_new_pass(device_t dev)
{
	struct aw_de2_tcon_softc *sc;
	phandle_t node;

	sc = device_get_softc(dev);
	node = ofw_bus_get_node(dev);

	if (sc->attach_done ||
	    bus_current_pass < (BUS_PASS_SUPPORTDEV + BUS_PASS_ORDER_MIDDLE)) {
		bus_generic_new_pass(dev);
		return;
	}
	sc->attach_done = 1;

	sc->mixer = ofw_graph_get_device_by_port_ep(node, 0, 0);
	if (sc->mixer == NULL) {
		device_printf(dev, "Cannot get remote endpoint device for port 0 and endpoint 0\n");
	}
}

static int
aw_de2_tcon_detach(device_t dev)
{
	struct aw_de2_tcon_softc *sc;

	sc = device_get_softc(dev);

	bus_generic_detach(sc->dev);
	bus_release_resources(dev, aw_de2_tcon_spec, sc->res);
	mtx_destroy(&sc->mtx);

	return (0);
}

static device_method_t aw_de2_tcon_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		aw_de2_tcon_probe),
	DEVMETHOD(device_attach,	aw_de2_tcon_attach),
	DEVMETHOD(device_detach,	aw_de2_tcon_detach),

	/* Bus interface */
	DEVMETHOD(bus_new_pass,		aw_de2_tcon_new_pass),

	/* AW_DE2_TCON interface */
	DEVMETHOD(aw_de2_tcon_create_crtc,	aw_de2_tcon_create_crtc),
	DEVMETHOD_END
};

static driver_t aw_de2_tcon_driver = {
	"aw_de2_tcon",
	aw_de2_tcon_methods,
	sizeof(struct aw_de2_tcon_softc),
};

static devclass_t aw_de2_tcon_devclass;

EARLY_DRIVER_MODULE(aw_tcon, simplebus, aw_de2_tcon_driver,
  aw_de2_tcon_devclass, 0, 0, BUS_PASS_SUPPORTDEV + BUS_PASS_ORDER_MIDDLE);
