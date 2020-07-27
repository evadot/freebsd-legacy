/* copied from FreeBSD sys/dev/iwm/if_iwm_phy_ctxt.c */

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
#include <dev/iwx/if_iwx_debug.h>
#include <dev/iwx/if_iwx_util.h>
#include <dev/iwx/if_iwx_phy_ctxt.h>

/*
 * BEGIN iwlwifi/mvm/phy-ctxt.c
 */

/*
 * Construct the generic fields of the PHY context command
 */
static void
iwx_phy_ctxt_cmd_hdr(struct iwx_softc *sc, struct iwx_phy_ctxt *ctxt,
	struct iwx_phy_context_cmd *cmd, uint32_t action, uint32_t apply_time)
{
	memset(cmd, 0, sizeof(struct iwx_phy_context_cmd));

	IWX_DPRINTF(sc, IWX_DEBUG_RESET | IWX_DEBUG_CMD,
	    "%s: id=%d, colour=%d, action=%d, apply_time=%d\n",
	    __func__,
	    ctxt->id,
	    ctxt->color,
	    action,
	    apply_time);

	cmd->id_and_color = htole32(IWX_FW_CMD_ID_AND_COLOR(ctxt->id,
	    ctxt->color));
	cmd->action = htole32(action);
	cmd->apply_time = htole32(apply_time);
}

/*
 * Add the phy configuration to the PHY context command
 */
static void
iwx_phy_ctxt_cmd_data(struct iwx_softc *sc,
	struct iwx_phy_context_cmd *cmd, struct ieee80211_channel *chan,
	uint8_t chains_static, uint8_t chains_dynamic)
{
	struct ieee80211com *ic = &sc->sc_ic;
	uint8_t active_cnt, idle_cnt;

	IWX_DPRINTF(sc, IWX_DEBUG_RESET | IWX_DEBUG_CMD,
	    "%s: 2ghz=%d, channel=%d, chains static=0x%x, dynamic=0x%x, "
	    "rx_ant=0x%x, tx_ant=0x%x\n",
	    __func__,
	    !! IEEE80211_IS_CHAN_2GHZ(chan),
	    ieee80211_chan2ieee(ic, chan),
	    chains_static,
	    chains_dynamic,
	    iwx_get_valid_rx_ant(sc),
	    iwx_get_valid_tx_ant(sc));

//	if (isset(sc->sc_enabled_capa, IWX_UCODE_TLV_CAPA_ULTRA_HB_CHANNELS)) {
	if (iwx_fw_has_capa(sc, IWX_UCODE_TLV_CAPA_ULTRA_HB_CHANNELS)) {
		cmd->ci.band = IEEE80211_IS_CHAN_2GHZ(chan) ?
		    IWX_PHY_BAND_24 : IWX_PHY_BAND_5;
		cmd->ci.channel = htole32(ieee80211_chan2ieee(ic, chan));
		cmd->ci.width = IWX_PHY_VHT_CHANNEL_MODE20;
		cmd->ci.ctrl_pos = IWX_PHY_VHT_CTRL_POS_1_BELOW;
	} else {
		struct iwx_fw_channel_info_v1 *ci_v1;
		ci_v1 = (struct iwx_fw_channel_info_v1 *)&cmd->ci;
		ci_v1->band = IEEE80211_IS_CHAN_2GHZ(chan) ?
		    IWX_PHY_BAND_24 : IWX_PHY_BAND_5;
		ci_v1->channel = ieee80211_chan2ieee(ic, chan);
		ci_v1->width = IWX_PHY_VHT_CHANNEL_MODE20;
		ci_v1->ctrl_pos = IWX_PHY_VHT_CTRL_POS_1_BELOW;
	}
	/* Set rx the chains */
	idle_cnt = chains_static;
	active_cnt = chains_dynamic;

	/* In scenarios where we only ever use a single-stream rates,
	 * i.e. legacy 11b/g/a associations, single-stream APs or even
	 * static SMPS, enable both chains to get diversity, improving
	 * the case where we're far enough from the AP that attenuation
	 * between the two antennas is sufficiently different to impact
	 * performance.
	 */
	if (active_cnt == 1 && iwx_rx_diversity_allowed(sc)) {
		idle_cnt = 2;
		active_cnt = 2;
	}

	cmd->rxchain_info = htole32(iwx_get_valid_rx_ant(sc) <<
					IWX_PHY_RX_CHAIN_VALID_POS);
	cmd->rxchain_info |= htole32(idle_cnt << IWX_PHY_RX_CHAIN_CNT_POS);
	cmd->rxchain_info |= htole32(active_cnt <<
	    IWX_PHY_RX_CHAIN_MIMO_CNT_POS);

	cmd->txchain_info = htole32(iwx_get_valid_tx_ant(sc));
}

/*
 * Send a command
 * only if something in the configuration changed: in case that this is the
 * first time that the phy configuration is applied or in case that the phy
 * configuration changed from the previous apply.
 */
static int
iwx_phy_ctxt_apply(struct iwx_softc *sc,
	struct iwx_phy_ctxt *ctxt,
	uint8_t chains_static, uint8_t chains_dynamic,
	uint32_t action, uint32_t apply_time)
{
	struct iwx_phy_context_cmd cmd;
	int ret;
	size_t len;

	IWX_DPRINTF(sc, IWX_DEBUG_RESET | IWX_DEBUG_CMD,
	    "%s: called; channel=%p\n",
	    __func__,
	    ctxt->channel);

	/* Set the command header fields */
	iwx_phy_ctxt_cmd_hdr(sc, ctxt, &cmd, action, apply_time);

	/*
	 * Intel resized fw_channel_info struct and neglected to resize the
	 * phy_context_cmd struct which contains it; so magic happens with
	 * command length adjustments at run-time... :(
	 */
	/* Set the command data */
	iwx_phy_ctxt_cmd_data(sc, &cmd, ctxt->channel,
	    chains_static, chains_dynamic);

	len = sizeof(struct iwx_phy_context_cmd);
//	if (!isset(sc->sc_enabled_capa, IWX_UCODE_TLV_CAPA_ULTRA_HB_CHANNELS))
	if (!iwx_fw_has_capa(sc, IWX_UCODE_TLV_CAPA_ULTRA_HB_CHANNELS))
		len -= (sizeof(struct iwx_fw_channel_info) -
		    sizeof(struct iwx_fw_channel_info_v1));
	ret = iwx_send_cmd_pdu(sc, IWX_PHY_CONTEXT_CMD, 0, len, &cmd);
	if (ret) {
		device_printf(sc->sc_dev,
		    "PHY ctxt cmd error. ret=%d\n", ret);
	}
	return ret;
}

/*
 * Send a command to add a PHY context based on the current HW configuration.
 */
int
iwx_phy_ctxt_add(struct iwx_softc *sc, struct iwx_phy_ctxt *ctxt,
	struct ieee80211_channel *chan,
	uint8_t chains_static, uint8_t chains_dynamic)
{
	ctxt->channel = chan;

	IWX_DPRINTF(sc, IWX_DEBUG_RESET | IWX_DEBUG_CMD,
	    "%s: called; channel=%d\n",
	    __func__,
	    ieee80211_chan2ieee(&sc->sc_ic, chan));

	return iwx_phy_ctxt_apply(sc, ctxt,
	    chains_static, chains_dynamic, IWX_FW_CTXT_ACTION_ADD, 0);
}

/*
 * Send a command to modify the PHY context based on the current HW
 * configuration. Note that the function does not check that the configuration
 * changed.
 */
int
iwx_phy_ctxt_changed(struct iwx_softc *sc,
	struct iwx_phy_ctxt *ctxt, struct ieee80211_channel *chan,
	uint8_t chains_static, uint8_t chains_dynamic)
{
	ctxt->channel = chan;

	IWX_DPRINTF(sc, IWX_DEBUG_RESET | IWX_DEBUG_CMD,
	    "%s: called; channel=%d\n",
	    __func__,
	    ieee80211_chan2ieee(&sc->sc_ic, chan));

	return iwx_phy_ctxt_apply(sc, ctxt,
	    chains_static, chains_dynamic, IWX_FW_CTXT_ACTION_MODIFY, 0);
}

/*
 * END iwlwifi/mvm/phy-ctxt.c
 */
