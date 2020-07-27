/* copied from FreeBSD sys/dev/iwm/if_iwm_pcie_trans.c */

/*	$OpenBSD: if_iwm.c,v 1.39 2015/03/23 00:35:19 jsg Exp $	*/

/*
 * Copyright (c) 2014 genua mbh <info@genua.de>
 * Copyright (c) 2014 Fixup Software Ltd.
 *
 * Permission to use, copy, modify, and distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

/*-
 * Based on BSD-licensed source modules in the Linux iwlwifi driver,
 * which were used as the reference documentation for this implementation.
 *
 * Driver version we are currently based off of is
 * Linux 3.14.3 (tag id a2df521e42b1d9a23f620ac79dbfe8655a8391dd)
 *
 ***********************************************************************
 *
 * This file is provided under a dual BSD/GPLv2 license.  When using or
 * redistributing this file, you may do so under either license.
 *
 * GPL LICENSE SUMMARY
 *
 * Copyright(c) 2007 - 2013 Intel Corporation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of version 2 of the GNU General Public License as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110,
 * USA
 *
 * The full GNU General Public License is included in this distribution
 * in the file called COPYING.
 *
 * Contact Information:
 *  Intel Linux Wireless <ilw@linux.intel.com>
 * Intel Corporation, 5200 N.E. Elam Young Parkway, Hillsboro, OR 97124-6497
 *
 *
 * BSD LICENSE
 *
 * Copyright(c) 2005 - 2013 Intel Corporation. All rights reserved.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  * Neither the name Intel Corporation nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*-
 * Copyright (c) 2007-2010 Damien Bergamini <damien.bergamini@free.fr>
 *
 * Permission to use, copy, modify, and distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */
#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include "opt_wlan.h"
#include "opt_iwx.h"

#include <sys/param.h>
#include <sys/bus.h>
#include <sys/conf.h>
#include <sys/endian.h>
#include <sys/firmware.h>
#include <sys/kernel.h>
#include <sys/malloc.h>
#include <sys/mbuf.h>
#include <sys/mutex.h>
#include <sys/module.h>
#include <sys/proc.h>
#include <sys/rman.h>
#include <sys/socket.h>
#include <sys/sockio.h>
#include <sys/sysctl.h>
#include <sys/linker.h>

#include <machine/bus.h>
#include <machine/endian.h>
#include <machine/resource.h>

#include <dev/pci/pcivar.h>
#include <dev/pci/pcireg.h>

#include <net/bpf.h>

#include <net/if.h>
#include <net/if_var.h>
#include <net/if_arp.h>
#include <net/if_dl.h>
#include <net/if_media.h>
#include <net/if_types.h>

#include <netinet/in.h>
#include <netinet/in_systm.h>
#include <netinet/if_ether.h>
#include <netinet/ip.h>

#include <net80211/ieee80211_var.h>
#include <net80211/ieee80211_regdomain.h>
#include <net80211/ieee80211_ratectl.h>
#include <net80211/ieee80211_radiotap.h>

#include <dev/iwx/if_iwxreg.h>
#include <dev/iwx/if_iwxvar.h>
#include <dev/iwx/if_iwx_config.h>
#include <dev/iwx/if_iwx_debug.h>
#include <dev/iwx/if_iwx_pcie_trans.h>

/*
 * This is a subset of what's in linux iwlwifi/pcie/trans.c.
 * The rest can be migrated out into here once they're no longer in
 * if_iwm.c.
 */

/*
 * basic device access
 */

uint32_t
iwx_read_prph(struct iwx_softc *sc, uint32_t addr)
{
	IWX_WRITE(sc,
	    IWX_HBUS_TARG_PRPH_RADDR, ((addr & 0x000fffff) | (3 << 24)));
	IWX_BARRIER_READ_WRITE(sc);
	return IWX_READ(sc, IWX_HBUS_TARG_PRPH_RDAT);
}

void
iwx_write_prph(struct iwx_softc *sc, uint32_t addr, uint32_t val)
{
	IWX_WRITE(sc,
	    IWX_HBUS_TARG_PRPH_WADDR, ((addr & 0x000fffff) | (3 << 24)));
	IWX_BARRIER_WRITE(sc);
	IWX_WRITE(sc, IWX_HBUS_TARG_PRPH_WDAT, val);
}

void
iwx_write_prph64(struct iwx_softc *sc, uint64_t addr, uint64_t val)
{
	iwx_write_prph(sc, (uint32_t)addr, val & 0xffffffff);
	iwx_write_prph(sc, (uint32_t)addr + 4, val >> 32);
}

/* source: iwlwifi, iwl-io.h */
uint32_t iwx_read_umac_prph(struct iwx_softc *sc, uint32_t offset)
{
	return iwx_read_prph(sc, offset + sc->cfg->umac_prph_offset);
}

/* source: iwlwifi, iwl-io.h */
void iwx_write_umac_prph(struct iwx_softc *sc, uint32_t offset,
		uint32_t val)
{
	iwx_write_prph(sc, offset + sc->cfg->umac_prph_offset, val);
}

#ifdef not_in_iwx
int
iwx_poll_prph(struct iwx_softc *sc, uint32_t addr, uint32_t bits, uint32_t mask,
    int timeout)
{
	do {
		if ((iwx_read_prph(sc, addr) & mask) == (bits & mask))
			return (0);
		DELAY(10);
		timeout -= 10;
	} while (timeout > 0);

	return (ETIMEDOUT);
}
#endif

#ifdef IWX_DEBUG
/* iwlwifi: pcie/trans.c */
int
iwx_read_mem(struct iwx_softc *sc, uint32_t addr, void *buf, int dwords)
{
	int offs, ret = 0;
	uint32_t *vals = buf;

	if (iwx_nic_lock(sc)) {
		IWX_WRITE(sc, IWX_HBUS_TARG_MEM_RADDR, addr);
		for (offs = 0; offs < dwords; offs++)
			vals[offs] = le32toh(IWX_READ(sc, IWX_HBUS_TARG_MEM_RDAT));
		iwx_nic_unlock(sc);
	} else {
		ret = EBUSY;
	}
	return ret;
}
#endif

/* iwlwifi: pcie/trans.c */
int
iwx_write_mem(struct iwx_softc *sc, uint32_t addr, const void *buf, int dwords)
{
	int offs;
	const uint32_t *vals = buf;

	if (iwx_nic_lock(sc)) {
		IWX_WRITE(sc, IWX_HBUS_TARG_MEM_WADDR, addr);
		/* WADDR auto-increments */
		for (offs = 0; offs < dwords; offs++) {
			uint32_t val = vals ? vals[offs] : 0;
			IWX_WRITE(sc, IWX_HBUS_TARG_MEM_WDAT, val);
		}
		iwx_nic_unlock(sc);
	} else {
		IWX_DPRINTF(sc, IWX_DEBUG_TRANS,
		    "%s: write_mem failed\n", __func__);
		return EBUSY;
	}
	return 0;
}

int
iwx_write_mem32(struct iwx_softc *sc, uint32_t addr, uint32_t val)
{
	return iwx_write_mem(sc, addr, &val, 1);
}

int
iwx_poll_bit(struct iwx_softc *sc, int reg,
	uint32_t bits, uint32_t mask, int timo)
{
	for (;;) {
		if ((IWX_READ(sc, reg) & mask) == (bits & mask)) {
			return 1;
		}
		if (timo < 10) {
			return 0;
		}
		timo -= 10;
		DELAY(10);
	}
}

int
iwx_nic_lock(struct iwx_softc *sc)
{
	int rv = 0;

	if (sc->cmd_hold_nic_awake)
		return 1;

	IWX_SETBITS(sc, IWX_CSR_GP_CNTRL,
	    IWX_CSR_GP_CNTRL_REG_FLAG_MAC_ACCESS_REQ);

	DELAY(2);

	if (iwx_poll_bit(sc, IWX_CSR_GP_CNTRL,
	    IWX_CSR_GP_CNTRL_REG_VAL_MAC_ACCESS_EN,
	    IWX_CSR_GP_CNTRL_REG_FLAG_MAC_CLOCK_READY
	     | IWX_CSR_GP_CNTRL_REG_FLAG_GOING_TO_SLEEP, 15000)) {
		rv = 1;
	} else {
		/* jolt */
		IWX_DPRINTF(sc, IWX_DEBUG_RESET,
		    "%s: resetting device via NMI\n", __func__);
		IWX_WRITE(sc, IWX_CSR_RESET, IWX_CSR_RESET_REG_FLAG_FORCE_NMI);
	}

	return rv;
}

void
iwx_nic_unlock(struct iwx_softc *sc)
{
	if (sc->cmd_hold_nic_awake)
		return;

	IWX_CLRBITS(sc, IWX_CSR_GP_CNTRL,
	    IWX_CSR_GP_CNTRL_REG_FLAG_MAC_ACCESS_REQ);
}

void
iwx_set_bits_mask_prph(struct iwx_softc *sc,
	uint32_t reg, uint32_t bits, uint32_t mask)
{
	uint32_t val;

	/* XXX: no error path? */
	if (iwx_nic_lock(sc)) {
		val = iwx_read_prph(sc, reg) & mask;
		val |= bits;
		iwx_write_prph(sc, reg, val);
		iwx_nic_unlock(sc);
	}
}

void
iwx_set_bits_prph(struct iwx_softc *sc, uint32_t reg, uint32_t bits)
{
	iwx_set_bits_mask_prph(sc, reg, bits, ~0);
}

void
iwx_clear_bits_prph(struct iwx_softc *sc, uint32_t reg, uint32_t bits)
{
	iwx_set_bits_mask_prph(sc, reg, 0, ~bits);
}

/*
 * High-level hardware frobbing routines
 */

void
iwx_enable_rfkill_int(struct iwx_softc *sc)
{
	sc->sc_intmask = IWX_CSR_INT_BIT_RF_KILL;
	IWX_WRITE(sc, IWX_CSR_INT_MASK, sc->sc_intmask);
	IWX_SETBITS(sc, IWX_CSR_GP_CNTRL,
	    IWX_CSR_GP_CNTRL_REG_FLAG_RFKILL_WAKE_L1A_EN);
}

int
iwx_check_rfkill(struct iwx_softc *sc)
{
	uint32_t v;
	int rv;

	/*
	 * "documentation" is not really helpful here:
	 *  27:	HW_RF_KILL_SW
	 *	Indicates state of (platform's) hardware RF-Kill switch
	 *
	 * But apparently when it's off, it's on ...
	 */
	v = IWX_READ(sc, IWX_CSR_GP_CNTRL);
	rv = (v & IWX_CSR_GP_CNTRL_REG_FLAG_HW_RF_KILL_SW) == 0;
	if (rv) {
		sc->sc_flags |= IWX_FLAG_RFKILL;
	} else {
		sc->sc_flags &= ~IWX_FLAG_RFKILL;
	}

	return rv;
}

#define IWX_HW_READY_TIMEOUT 50
int
iwx_set_hw_ready(struct iwx_softc *sc)
{
	int ready;

	IWX_SETBITS(sc, IWX_CSR_HW_IF_CONFIG_REG,
	    IWX_CSR_HW_IF_CONFIG_REG_BIT_NIC_READY);

	ready = iwx_poll_bit(sc, IWX_CSR_HW_IF_CONFIG_REG,
	    IWX_CSR_HW_IF_CONFIG_REG_BIT_NIC_READY,
	    IWX_CSR_HW_IF_CONFIG_REG_BIT_NIC_READY,
	    IWX_HW_READY_TIMEOUT);
	if (ready) {
		IWX_SETBITS(sc, IWX_CSR_MBOX_SET_REG,
		    IWX_CSR_MBOX_SET_REG_OS_ALIVE);
	}
	return ready;
}
#undef IWX_HW_READY_TIMEOUT

int
iwx_prepare_card_hw(struct iwx_softc *sc)
{
	int rv = 0;
	int t = 0;

	IWX_DPRINTF(sc, IWX_DEBUG_RESET, "->%s\n", __func__);
	if (iwx_set_hw_ready(sc))
		goto out;

	IWX_SETBITS(sc, IWX_CSR_DBG_LINK_PWR_MGMT_REG,
	    IWX_CSR_RESET_LINK_PWR_MGMT_DISABLED);
	DELAY(1000);

	/* If HW is not ready, prepare the conditions to check again */
	IWX_SETBITS(sc, IWX_CSR_HW_IF_CONFIG_REG,
	    IWX_CSR_HW_IF_CONFIG_REG_PREPARE);

	do {
		if (iwx_set_hw_ready(sc))
			goto out;
		DELAY(200);
		t += 200;
	} while (t < 150000);

	rv = ETIMEDOUT;

 out:
	IWX_DPRINTF(sc, IWX_DEBUG_RESET, "<-%s\n", __func__);
	return rv;
}

void
iwx_apm_config(struct iwx_softc *sc)
{
	uint16_t lctl, cap;
	int pcie_ptr;

	/*
	 * HW bug W/A for instability in PCIe bus L0S->L1 transition.
	 * Check if BIOS (or OS) enabled L1-ASPM on this device.
	 * If so (likely), disable L0S, so device moves directly L0->L1;
	 *    costs negligible amount of power savings.
	 * If not (unlikely), enable L0S, so there is at least some
	 *    power savings, even without L1.
	 */
	int error;

	error = pci_find_cap(sc->sc_dev, PCIY_EXPRESS, &pcie_ptr);
	if (error != 0)
		return;
	lctl = pci_read_config(sc->sc_dev, pcie_ptr + PCIER_LINK_CTL,
	    sizeof(lctl));
	if (lctl & PCIEM_LINK_CTL_ASPMC_L1)  {
		IWX_SETBITS(sc, IWX_CSR_GIO_REG,
		    IWX_CSR_GIO_REG_VAL_L0S_ENABLED);
	} else {
		IWX_CLRBITS(sc, IWX_CSR_GIO_REG,
		    IWX_CSR_GIO_REG_VAL_L0S_ENABLED);
	}

	cap = pci_read_config(sc->sc_dev, pcie_ptr + PCIER_DEVICE_CTL2,
	    sizeof(cap));
	sc->sc_ltr_enabled = (cap & PCIEM_CTL2_LTR_ENABLE) ? 1 : 0;
	IWX_DPRINTF(sc, IWX_DEBUG_RESET | IWX_DEBUG_PWRSAVE,
	    "L1 %sabled - LTR %sabled\n",
	    (lctl & PCIEM_LINK_CTL_ASPMC_L1) ? "En" : "Dis",
	    sc->sc_ltr_enabled ? "En" : "Dis");
}

/*
 * Start up NIC's basic functionality after it has been reset
 * (e.g. after platform boot, or shutdown via iwm_pcie_apm_stop())
 * NOTE:  This does not load uCode nor start the embedded processor
 */
int
iwx_apm_init(struct iwx_softc *sc)
{
	int error = 0;

	IWX_DPRINTF(sc, IWX_DEBUG_RESET, "iwx apm start\n");

	/*
	 * Disable L0s without affecting L1;
	 *  don't wait for ICH L0s (ICH bug W/A)
	 */
	IWX_SETBITS(sc, IWX_CSR_GIO_CHICKEN_BITS,
	    IWX_CSR_GIO_CHICKEN_BITS_REG_BIT_L1A_NO_L0S_RX);

	/* Set FH wait threshold to maximum (HW error during stress W/A) */
	IWX_SETBITS(sc, IWX_CSR_DBG_HPET_MEM_REG, IWX_CSR_DBG_HPET_MEM_REG_VAL);

	/*
	 * Enable HAP INTA (interrupt from management bus) to
	 * wake device's PCI Express link L1a -> L0s
	 */
	IWX_SETBITS(sc, IWX_CSR_HW_IF_CONFIG_REG,
	    IWX_CSR_HW_IF_CONFIG_REG_BIT_HAP_WAKE_L1A);

	iwx_apm_config(sc);

	/*
	 * Set "initialization complete" bit to move adapter from
	 * D0U* --> D0A* (powered-up active) state.
	 */
	IWX_SETBITS(sc, IWX_CSR_GP_CNTRL, IWX_CSR_GP_CNTRL_REG_FLAG_INIT_DONE);

	/*
	 * Wait for clock stabilization; once stabilized, access to
	 * device-internal resources is supported, e.g. iwm_write_prph()
	 * and accesses to uCode SRAM.
	 */
	if (!iwx_poll_bit(sc, IWX_CSR_GP_CNTRL,
	    IWX_CSR_GP_CNTRL_REG_FLAG_MAC_CLOCK_READY,
	    IWX_CSR_GP_CNTRL_REG_FLAG_MAC_CLOCK_READY, 25000)) {
		device_printf(sc->sc_dev,
		    "timeout waiting for clock stabilization\n");
		error = ETIMEDOUT;
		goto out;
	}

 out:
	if (error)
		device_printf(sc->sc_dev, "apm init error %d\n", error);
	return error;
}

/* iwlwifi/pcie/trans-gen2.c */
void
iwx_apm_stop(struct iwx_softc *sc)
{
	IWX_SETBITS(sc, IWX_CSR_DBG_LINK_PWR_MGMT_REG,
			IWX_CSR_RESET_LINK_PWR_MGMT_DISABLED);
	IWX_SETBITS(sc, IWX_CSR_HW_IF_CONFIG_REG,
			IWX_CSR_HW_IF_CONFIG_REG_PREPARE |
			IWX_CSR_HW_IF_CONFIG_REG_ENABLE_PME);
	DELAY(1000);
	IWX_CLRBITS(sc, IWX_CSR_DBG_LINK_PWR_MGMT_REG,
			IWX_CSR_RESET_LINK_PWR_MGMT_DISABLED);
	DELAY(5000);

	/* stop device's busmaster DMA activity */
	IWX_SETBITS(sc, IWX_CSR_RESET, IWX_CSR_RESET_REG_FLAG_STOP_MASTER);

	if (!iwx_poll_bit(sc, IWX_CSR_RESET,
				IWX_CSR_RESET_REG_FLAG_MASTER_DISABLED,
				IWX_CSR_RESET_REG_FLAG_MASTER_DISABLED, 100))
		device_printf(sc->sc_dev, "timeout waiting for master\n");

	/*
	 * Clear "initialization complete" bit to move adapter from
	 * D0A* (powered-up Active) --> D0U* (Uninitialized) state.
	 */
	IWX_CLRBITS(sc, IWX_CSR_GP_CNTRL,
			IWX_CSR_GP_CNTRL_REG_FLAG_INIT_DONE);

	IWX_DPRINTF(sc, IWX_DEBUG_TRANS, "%s: iwx apm stop\n", __func__);
}

#ifdef from_openbsd
void
iwx_init_msix_hw(struct iwx_softc *sc)
{
	iwx_conf_msix_hw(sc, 0);

	if (!sc->sc_msix)
		return;

	sc->sc_fh_init_mask = ~IWX_READ(sc, IWX_CSR_MSIX_FH_INT_MASK_AD);
	sc->sc_fh_mask = sc->sc_fh_init_mask;
	sc->sc_hw_init_mask = ~IWX_READ(sc, IWX_CSR_MSIX_HW_INT_MASK_AD);
	sc->sc_hw_mask = sc->sc_hw_init_mask;
}
void
iwx_conf_msix_hw(struct iwx_softc *sc, int stopped)
{
	int vector = 0;

	if (!sc->sc_msix) {
		/* Newer chips default to MSIX. */
		if (!stopped && iwx_nic_lock(sc)) {
			iwx_write_prph(sc, IWX_UREG_CHICK,
			    IWX_UREG_CHICK_MSI_ENABLE);
			iwx_nic_unlock(sc);
		}
		return;
	}

	if (!stopped && iwx_nic_lock(sc)) {
		iwx_write_prph(sc, IWX_UREG_CHICK, IWX_UREG_CHICK_MSIX_ENABLE);
		iwx_nic_unlock(sc);
	}

	/* Disable all interrupts */
	IWX_WRITE(sc, IWX_CSR_MSIX_FH_INT_MASK_AD, ~0);
	IWX_WRITE(sc, IWX_CSR_MSIX_HW_INT_MASK_AD, ~0);

	/* Map fallback-queue (command/mgmt) to a single vector */
	IWX_WRITE_1(sc, IWX_CSR_MSIX_RX_IVAR(0),
	    vector | IWX_MSIX_NON_AUTO_CLEAR_CAUSE);
	/* Map RSS queue (data) to the same vector */
	IWX_WRITE_1(sc, IWX_CSR_MSIX_RX_IVAR(1),
	    vector | IWX_MSIX_NON_AUTO_CLEAR_CAUSE);

	/* Enable the RX queues cause interrupts */
	IWX_CLRBITS(sc, IWX_CSR_MSIX_FH_INT_MASK_AD,
	    IWX_MSIX_FH_INT_CAUSES_Q0 | IWX_MSIX_FH_INT_CAUSES_Q1);

	/* Map non-RX causes to the same vector */
	IWX_WRITE_1(sc, IWX_CSR_MSIX_IVAR(IWX_MSIX_IVAR_CAUSE_D2S_CH0_NUM),
	    vector | IWX_MSIX_NON_AUTO_CLEAR_CAUSE);
	IWX_WRITE_1(sc, IWX_CSR_MSIX_IVAR(IWX_MSIX_IVAR_CAUSE_D2S_CH1_NUM),
	    vector | IWX_MSIX_NON_AUTO_CLEAR_CAUSE);
	IWX_WRITE_1(sc, IWX_CSR_MSIX_IVAR(IWX_MSIX_IVAR_CAUSE_S2D),
	    vector | IWX_MSIX_NON_AUTO_CLEAR_CAUSE);
	IWX_WRITE_1(sc, IWX_CSR_MSIX_IVAR(IWX_MSIX_IVAR_CAUSE_FH_ERR),
	    vector | IWX_MSIX_NON_AUTO_CLEAR_CAUSE);
	IWX_WRITE_1(sc, IWX_CSR_MSIX_IVAR(IWX_MSIX_IVAR_CAUSE_REG_ALIVE),
	    vector | IWX_MSIX_NON_AUTO_CLEAR_CAUSE);
	IWX_WRITE_1(sc, IWX_CSR_MSIX_IVAR(IWX_MSIX_IVAR_CAUSE_REG_WAKEUP),
	    vector | IWX_MSIX_NON_AUTO_CLEAR_CAUSE);
	IWX_WRITE_1(sc, IWX_CSR_MSIX_IVAR(IWX_MSIX_IVAR_CAUSE_REG_IML),
	    vector | IWX_MSIX_NON_AUTO_CLEAR_CAUSE);
	IWX_WRITE_1(sc, IWX_CSR_MSIX_IVAR(IWX_MSIX_IVAR_CAUSE_REG_CT_KILL),
	    vector | IWX_MSIX_NON_AUTO_CLEAR_CAUSE);
	IWX_WRITE_1(sc, IWX_CSR_MSIX_IVAR(IWX_MSIX_IVAR_CAUSE_REG_RF_KILL),
	    vector | IWX_MSIX_NON_AUTO_CLEAR_CAUSE);
	IWX_WRITE_1(sc, IWX_CSR_MSIX_IVAR(IWX_MSIX_IVAR_CAUSE_REG_PERIODIC),
	    vector | IWX_MSIX_NON_AUTO_CLEAR_CAUSE);
	IWX_WRITE_1(sc, IWX_CSR_MSIX_IVAR(IWX_MSIX_IVAR_CAUSE_REG_SW_ERR),
	    vector | IWX_MSIX_NON_AUTO_CLEAR_CAUSE);
	IWX_WRITE_1(sc, IWX_CSR_MSIX_IVAR(IWX_MSIX_IVAR_CAUSE_REG_SCD),
	    vector | IWX_MSIX_NON_AUTO_CLEAR_CAUSE);
	IWX_WRITE_1(sc, IWX_CSR_MSIX_IVAR(IWX_MSIX_IVAR_CAUSE_REG_FH_TX),
	    vector | IWX_MSIX_NON_AUTO_CLEAR_CAUSE);
	IWX_WRITE_1(sc, IWX_CSR_MSIX_IVAR(IWX_MSIX_IVAR_CAUSE_REG_HW_ERR),
	    vector | IWX_MSIX_NON_AUTO_CLEAR_CAUSE);
	IWX_WRITE_1(sc, IWX_CSR_MSIX_IVAR(IWX_MSIX_IVAR_CAUSE_REG_HAP),
	    vector | IWX_MSIX_NON_AUTO_CLEAR_CAUSE);

	/* Enable non-RX causes interrupts */
	IWX_CLRBITS(sc, IWX_CSR_MSIX_FH_INT_MASK_AD,
	    IWX_MSIX_FH_INT_CAUSES_D2S_CH0_NUM |
	    IWX_MSIX_FH_INT_CAUSES_D2S_CH1_NUM |
	    IWX_MSIX_FH_INT_CAUSES_S2D |
	    IWX_MSIX_FH_INT_CAUSES_FH_ERR);
	IWX_CLRBITS(sc, IWX_CSR_MSIX_HW_INT_MASK_AD,
	    IWX_MSIX_HW_INT_CAUSES_REG_ALIVE |
	    IWX_MSIX_HW_INT_CAUSES_REG_WAKEUP |
	    IWX_MSIX_HW_INT_CAUSES_REG_IML |
	    IWX_MSIX_HW_INT_CAUSES_REG_CT_KILL |
	    IWX_MSIX_HW_INT_CAUSES_REG_RF_KILL |
	    IWX_MSIX_HW_INT_CAUSES_REG_PERIODIC |
	    IWX_MSIX_HW_INT_CAUSES_REG_SW_ERR |
	    IWX_MSIX_HW_INT_CAUSES_REG_SCD |
	    IWX_MSIX_HW_INT_CAUSES_REG_FH_TX |
	    IWX_MSIX_HW_INT_CAUSES_REG_HW_ERR |
	    IWX_MSIX_HW_INT_CAUSES_REG_HAP);
}
#endif

/* iwlwifi pcie/trans.c */
int
iwx_start_hw(struct iwx_softc *sc)
{
	int error;

	if ((error = iwx_prepare_card_hw(sc)) != 0)
		return error;

	/* Reset the entire device */
	IWX_SETBITS(sc, IWX_CSR_RESET, IWX_CSR_RESET_REG_FLAG_SW_RESET);
	DELAY(5000);

	if ((error = iwx_apm_init(sc)) != 0)
		return error;

	/* On newer chipsets MSI is disabled by default. */
	/* todo if_iwx: replace this with iwx_init_msix_hw() - import it first */
	if (sc->cfg->mqrx_supported)
		iwx_write_prph(sc, IWX_UREG_CHICK, IWX_UREG_CHICK_MSI_ENABLE);

	iwx_enable_rfkill_int(sc);
	iwx_check_rfkill(sc);

	return 0;
}

/* iwlwifi pcie/trans.c (always main power) */
void
iwx_set_pwr(struct iwx_softc *sc)
{
	iwx_set_bits_mask_prph(sc, IWX_APMG_PS_CTRL_REG,
			IWX_APMG_PS_CTRL_VAL_PWR_SRC_VMAIN, ~IWX_APMG_PS_CTRL_MSK_PWR_SRC);
}

/* iwlwifi pcie/rx.c */
// is this iwx_disable_rx_dma() from openbsd?
int
iwx_pcie_rx_stop(struct iwx_softc *sc)
{
	int ret = 0;

	int ntries;
	if (iwx_nic_lock(sc)) {
		iwx_write_prph(sc, IWX_RFH_RXF_DMA_CFG, 0);
		for (ntries = 0; ntries < 1000; ntries++) {
			if (iwx_read_prph(sc, IWX_RFH_GEN_STATUS) &
			    IWX_RXF_DMA_IDLE)
				break;
			DELAY(10);
		}
		iwx_nic_unlock(sc);
	}
	return ret;
}

void
iwx_pcie_clear_cmd_in_flight(struct iwx_softc *sc)
{
#ifdef tbd
	if (!sc->cfg->apmg_wake_up_wa)
		return;

	if (!sc->cmd_hold_nic_awake) {
		device_printf(sc->sc_dev,
				"%s: cmd_hold_nic_awake not set\n", __func__);
		return;
	}

	sc->cmd_hold_nic_awake = 0;
#endif
	IWX_CLRBITS(sc, IWX_CSR_GP_CNTRL,
			IWX_CSR_GP_CNTRL_REG_FLAG_MAC_ACCESS_REQ);
}

int
iwx_pcie_set_cmd_in_flight(struct iwx_softc *sc)
{
#ifdef tbd
	int ret;
	/*
	 * wake up the NIC to make sure that the firmware will see the host
	 * command - we will let the NIC sleep once all the host commands
	 * returned. This needs to be done only on NICs that have
	 * apmg_wake_up_wa set.
	 */
	if (sc->cfg->apmg_wake_up_wa &&
			!sc->cmd_hold_nic_awake) {

		IWX_SETBITS(sc, IWX_CSR_GP_CNTRL,
				IWX_CSR_GP_CNTRL_REG_FLAG_MAC_ACCESS_REQ);

		ret = iwx_poll_bit(sc, IWX_CSR_GP_CNTRL,
				IWX_CSR_GP_CNTRL_REG_VAL_MAC_ACCESS_EN,
				(IWX_CSR_GP_CNTRL_REG_FLAG_MAC_CLOCK_READY |
				 IWX_CSR_GP_CNTRL_REG_FLAG_GOING_TO_SLEEP),
				15000);
		if (ret == 0) {
			IWX_CLRBITS(sc, IWX_CSR_GP_CNTRL,
					IWX_CSR_GP_CNTRL_REG_FLAG_MAC_ACCESS_REQ);
			device_printf(sc->sc_dev,
					"%s: Failed to wake NIC for hcmd\n", __func__);
			return EIO;
		}
		sc->cmd_hold_nic_awake = 1;
	}
#endif

	return 0;
}
