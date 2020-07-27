/* copied from FreeBSD sys/dev/iwm/if_iwm_binding.c */

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
 * Copyright(c) 2012 - 2014 Intel Corporation. All rights reserved.
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
 *  Intel Linux Wireless <linuxwifi@intel.com>
 * Intel Corporation, 5200 N.E. Elam Young Parkway, Hillsboro, OR 97124-6497
 *
 * BSD LICENSE
 *
 * Copyright(c) 2012 - 2014 Intel Corporation. All rights reserved.
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
#include <dev/iwx/if_iwx_debug.h>
#include <dev/iwx/if_iwx_util.h>
#include <dev/iwx/if_iwx_binding.h>
#include <dev/iwx/if_iwx_sf.h>

/*
 * BEGIN iwlwifi/mvm/binding.c
 */

struct iwx_iface_iterator_data {
	int idx;

	struct iwx_phy_ctxt *phyctxt;

	uint16_t ids[IWX_MAX_MACS_IN_BINDING];
	int16_t colors[IWX_MAX_MACS_IN_BINDING];
};

static int
iwx_binding_cmd(struct iwx_softc *sc, uint32_t action,
	struct iwx_iface_iterator_data *data)
{
	struct iwx_binding_cmd cmd;
	struct iwx_phy_ctxt *phyctxt = data->phyctxt;
	int i, ret;
	uint32_t status;

	memset(&cmd, 0, sizeof(cmd));

	cmd.id_and_color
	    = htole32(IWX_FW_CMD_ID_AND_COLOR(phyctxt->id, phyctxt->color));
	cmd.action = htole32(action);
	cmd.phy = htole32(IWX_FW_CMD_ID_AND_COLOR(phyctxt->id, phyctxt->color));

	for (i = 0; i < IWX_MAX_MACS_IN_BINDING; i++)
		cmd.macs[i] = htole32(IWX_FW_CTXT_INVALID);
	for (i = 0; i < data->idx; i++)
		cmd.macs[i] = htole32(IWX_FW_CMD_ID_AND_COLOR(data->ids[i],
							      data->colors[i]));

	if (IEEE80211_IS_CHAN_2GHZ(phyctxt->channel))
		cmd.lmac_id = htole32(IWX_LMAC_24G_INDEX);
	else
		cmd.lmac_id = htole32(IWX_LMAC_5G_INDEX);

	status = 0;
	ret = iwx_send_cmd_pdu_status(sc, IWX_BINDING_CONTEXT_CMD,
	    sizeof(cmd), &cmd, &status);
	if (ret) {
		device_printf(sc->sc_dev,
		    "Failed to send binding (action:%d): %d\n", action, ret);
		return ret;
	}

	if (status) {
		device_printf(sc->sc_dev,
		    "Binding command failed: %u\n", status);
		ret = EIO;
	}

	return ret;
}

static int
iwx_binding_update(struct iwx_softc *sc, struct iwx_vap *ivp,
	struct iwx_phy_ctxt *phyctxt, boolean_t add)
{
	struct iwx_iface_iterator_data data = {
		.phyctxt = phyctxt,
	};
	uint32_t action;

	if (add)
		action = IWX_FW_CTXT_ACTION_ADD;
	else
		action = IWX_FW_CTXT_ACTION_REMOVE;

	if (add) {
		data.ids[0] = ivp->id;
		data.colors[0] = ivp->color;
		data.idx++;
	}

	return iwx_binding_cmd(sc, action, &data);
}

int
iwx_binding_add_vif(struct iwx_softc *sc, struct iwx_vap *ivp)
{
	if (!ivp->phy_ctxt)
		return EINVAL;

	/*
	 * Update SF - Disable if needed. if this fails, SF might still be on
	 * while many macs are bound, which is forbidden - so fail the binding.
	 */
	if (iwx_sf_update(sc, &ivp->iv_vap, FALSE))
		return EINVAL;

	return iwx_binding_update(sc, ivp, ivp->phy_ctxt, TRUE);
}

int
iwx_binding_remove_vif(struct iwx_softc *sc, struct iwx_vap *ivp)
{
	int ret;

	if (!ivp->phy_ctxt)
		return EINVAL;

	ret = iwx_binding_update(sc, ivp, ivp->phy_ctxt, FALSE);

	if (!ret) {
		if (iwx_sf_update(sc, &ivp->iv_vap, TRUE))
			device_printf(sc->sc_dev,
			    "Failed to update SF state\n");
	}

	return ret;
}
