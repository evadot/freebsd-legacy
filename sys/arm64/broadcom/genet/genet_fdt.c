/*-
 * Copyright (c) 2020 Michael J Karels
 * Copyright (c) 2016, 2020 Jared McNeill <jmcneill@invisible.ca>
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
#include <sys/rman.h>
#include <sys/kernel.h>
#include <sys/endian.h>
#include <sys/mbuf.h>
#include <sys/socket.h>
#include <sys/sockio.h>
#include <sys/module.h>

#include <net/bpf.h>
#include <net/if.h>
#include <net/ethernet.h>
#include <net/if_dl.h>
#include <net/if_media.h>
#include <net/if_types.h>
#include <net/if_var.h>

#include <machine/bus.h>

#define __BIT(_x)	(1 << (_x))
#include "if_genetreg.h"

#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <dev/mii/mii.h>
#include <dev/mii/miivar.h>
#include <dev/mii/mii_fdt.h>

#include <netinet/in.h>
#include <netinet/ip.h>
#include <netinet/ip6.h>
#define ICMPV6_HACK	/* workaround for chip issue */
#ifdef ICMPV6_HACK
#include <netinet/icmp6.h>
#endif

#include "miibus_if.h"

#include "if_genet.h"

static struct ofw_compat_data compat_data[] = {
	{ "brcm,genet-v1",		1 },
	{ "brcm,genet-v2",		2 },
	{ "brcm,genet-v3",		3 },
	{ "brcm,genet-v4",		4 },
	{ "brcm,genet-v5",		5 },
	{ NULL,				0 }
};

static int
gen_get_phy_mode(device_t dev)
{
	struct gen_softc *sc;
	phandle_t node;
	mii_contype_t type;
	int error = 0;

	sc = device_get_softc(dev);
	node = ofw_bus_get_node(dev);
	type = mii_fdt_get_contype(node);

	switch (type) {
	case MII_CONTYPE_RGMII:
	case MII_CONTYPE_RGMII_ID:
	case MII_CONTYPE_RGMII_RXID:
	case MII_CONTYPE_RGMII_TXID:
		sc->phy_mode = type;
		break;
	default:
		device_printf(dev, "unknown phy-mode '%s'\n",
		    mii_fdt_contype_to_name(type));
		error = ENXIO;
		break;
	}

	return (error);
}

static void
gen_fdt_get_eaddr(struct gen_softc *sc)
{
	phandle_t node;

	sc->eaddr_found = false;
	node = ofw_bus_get_node(sc->dev);
	if (OF_getprop(node, "mac-address", sc->eaddr.octet,
	    ETHER_ADDR_LEN) != -1 ||
	    OF_getprop(node, "local-mac-address", sc->eaddr.octet,
	    ETHER_ADDR_LEN) != -1 ||
	  OF_getprop(node, "address", sc->eaddr.octet, ETHER_ADDR_LEN) != -1) {
		sc->eaddr_found = true;
		return;
	}

	device_printf(sc->dev, "No Ethernet address found in fdt!\n");
}

static int
genet_fdt_probe(device_t dev)
{
	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data == 0)
		return (ENXIO);

	device_set_desc(dev, "RPi4 Gigabit Ethernet");
	return (BUS_PROBE_DEFAULT);
}

static int
genet_fdt_attach(device_t dev)
{
	struct gen_softc *sc;

	sc = device_get_softc(dev);
	sc->dev = dev;
	sc->type = ofw_bus_search_compatible(dev, compat_data)->ocd_data;

	if (gen_get_phy_mode(dev) != 0)
		return (ENXIO);

	gen_fdt_get_eaddr(sc);

	return genet_attach(sc);
}

static device_method_t gen_fdt_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		genet_fdt_probe),
	DEVMETHOD(device_attach,	genet_fdt_attach),

	/* MII interface */
	DEVMETHOD(miibus_readreg,	genet_miibus_readreg),
	DEVMETHOD(miibus_writereg,	genet_miibus_writereg),
	DEVMETHOD(miibus_statchg,	genet_miibus_statchg),

	DEVMETHOD_END
};

static driver_t gen_driver = {
	"genet_fdt",
	gen_fdt_methods,
	sizeof(struct gen_softc),
};

static devclass_t gen_devclass;

DRIVER_MODULE(genet_fdt, simplebus, gen_driver, gen_devclass, 0, 0);
DRIVER_MODULE(miibus, genet_fdt, miibus_driver, miibus_devclass, 0, 0);
MODULE_DEPEND(genet_fdt, ether, 1, 1, 1);
MODULE_DEPEND(genet_fdt, miibus, 1, 1, 1);
