#include "opt_acpi.h"

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

#include <dev/mii/mii.h>
#include <dev/mii/miivar.h>

#include <netinet/in.h>
#include <netinet/ip.h>
#include <netinet/ip6.h>
#define ICMPV6_HACK	/* workaround for chip issue */
#ifdef ICMPV6_HACK
#include <netinet/icmp6.h>
#endif

#include "miibus_if.h"

#include "if_genet.h"

#include <contrib/dev/acpica/include/acpi.h>
#include <dev/acpica/acpivar.h>

static int
genet_acpi_probe(device_t dev)
{
	ACPI_HANDLE h;

	if ((h = acpi_get_handle(dev)) == NULL ||
	    acpi_MatchHid(h, "BCM6E4E") == ACPI_MATCHHID_NOMATCH)
		return (ENXIO);

	device_set_desc(dev, "RPi4 Gigabit Ethernet");
	return (BUS_PROBE_DEFAULT);
}

static int
genet_acpi_attach(device_t dev)
{
	struct gen_softc *sc;

	sc = device_get_softc(dev);
	sc->dev = dev;

	return genet_attach(sc);
}

static device_method_t genet_acpi_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		genet_acpi_probe),
	DEVMETHOD(device_attach,	genet_acpi_attach),

	/* MII interface */
	DEVMETHOD(miibus_readreg,	genet_miibus_readreg),
	DEVMETHOD(miibus_writereg,	genet_miibus_writereg),
	DEVMETHOD(miibus_statchg,	genet_miibus_statchg),

	DEVMETHOD_END
};

static driver_t genet_acpi_driver = {
	"genet_acpi",
	genet_acpi_methods,
	sizeof(struct gen_softc),
};

static devclass_t genet_acpi_devclass;

DRIVER_MODULE(genet_acpi, acpi, genet_acpi_driver, genet_acpi_devclass, NULL,
    NULL);
MODULE_DEPEND(genet_acpi, ether, 1, 1, 1);
MODULE_DEPEND(genet_acpi, miibus, 1, 1, 1);
