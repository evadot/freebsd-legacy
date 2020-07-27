/* copied from FreeBSD sys/dev/iwm/if_iwm_sta.c */

/*-
 * Based on BSD-licensed source modules in the Linux iwlwifi driver,
 * which were used as the reference documentation for this implementation.
 *
 * Driver version we are currently based off of is
 * Linux 4.7.3 (tag id d7f6728f57e3ecbb7ef34eb7d9f564d514775d75)
 *
 ***********************************************************************
 *
 * This file is provided under a dual BSD/GPLv2 license.  When using or
 * redistributing this file, you may do so under either license.
 *
 * GPL LICENSE SUMMARY
 *
 * Copyright(c) 2012 - 2015 Intel Corporation. All rights reserved.
 * Copyright(c) 2013 - 2015 Intel Mobile Communications GmbH
 * Copyright(c) 2016 Intel Deutschland GmbH
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
 * Copyright(c) 2012 - 2015 Intel Corporation. All rights reserved.
 * Copyright(c) 2013 - 2015 Intel Mobile Communications GmbH
 * Copyright(c) 2016 Intel Deutschland GmbH
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
 *
 *****************************************************************************/

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
#include <dev/iwx/if_iwx_constants.h>
#include <dev/iwx/if_iwx_util.h>
#include <dev/iwx/if_iwx_mac_ctxt.h>
#include <dev/iwx/if_iwx_sta.h>

/*
 * New version of ADD_STA_sta command added new fields at the end of the
 * structure, so sending the size of the relevant API's structure is enough to
 * support both API versions.
 */
#ifdef tbd
static inline int
iwx_add_sta_cmd_size(struct iwx_softc *sc)
{
	return sc->cfg->mqrx_supported ? sizeof(struct iwx_add_sta_cmd) :
	    sizeof(struct iwx_add_sta_cmd_v7);
}
#endif

/* send station add/update command to firmware */
int
iwx_sta_send_to_fw(struct iwx_softc *sc, struct iwx_node *in,
	boolean_t update)
{
	struct iwx_vap *ivp = IWX_VAP(in->in_ni.ni_vap);
	struct iwx_add_sta_cmd add_sta_cmd = {
		.sta_id = IWX_STATION_ID,
		.mac_id_n_color =
		    htole32(IWX_FW_CMD_ID_AND_COLOR(ivp->id, ivp->color)),
		.add_modify = update ? 1 : 0,
		.station_flags_msk = htole32(IWX_STA_FLG_FAT_EN_MSK |
					     IWX_STA_FLG_MIMO_EN_MSK),
		.tid_disable_tx = htole16(0xffff),
	};
	int ret = 0;
	uint32_t status;
	uint32_t agg_size = 0, mpdu_dens = 0;

	if (!update) {
		int ac;
		for (ac = 0; ac < WME_NUM_AC; ac++) {
			add_sta_cmd.tfd_queue_msk |=
			    htole32(1 << iwx_ac_to_tx_fifo[ac]);
		}
		IEEE80211_ADDR_COPY(&add_sta_cmd.addr, in->in_ni.ni_bssid);
	}

	add_sta_cmd.station_flags |=
		htole32(agg_size << IWX_STA_FLG_MAX_AGG_SIZE_SHIFT);
	add_sta_cmd.station_flags |=
		htole32(mpdu_dens << IWX_STA_FLG_AGG_MPDU_DENS_SHIFT);

	status = IWX_ADD_STA_SUCCESS;
#ifdef tbd
	ret = iwx_send_cmd_pdu_status(sc, IWX_ADD_STA,
					  iwx_add_sta_cmd_size(sc),
					  &add_sta_cmd, &status);
	if (ret)
		return ret;
#endif

	switch (status & IWX_ADD_STA_STATUS_MASK) {
	case IWX_ADD_STA_SUCCESS:
		IWX_DPRINTF(sc, IWX_DEBUG_NODE, "IWX_ADD_STA PASSED\n");
		break;
	default:
		ret = EIO;
		device_printf(sc->sc_dev, "IWX_ADD_STA failed\n");
		break;
	}

	return ret;
}

int
iwx_add_sta(struct iwx_softc *sc, struct iwx_node *in)
{
	return iwx_sta_send_to_fw(sc, in, FALSE);
}

int
iwx_update_sta(struct iwx_softc *sc, struct iwx_node *in)
{
	return iwx_sta_send_to_fw(sc, in, TRUE);
}

int
iwx_drain_sta(struct iwx_softc *sc, struct iwx_vap *ivp, boolean_t drain)
{
	struct iwx_add_sta_cmd cmd = {};
	int ret = 0;
	uint32_t status;

	cmd.mac_id_n_color =
	    htole32(IWX_FW_CMD_ID_AND_COLOR(ivp->id, ivp->color));
	cmd.sta_id = IWX_STATION_ID;
	cmd.add_modify = IWX_STA_MODE_MODIFY;
	cmd.station_flags = drain ? htole32(IWX_STA_FLG_DRAIN_FLOW) : 0;
	cmd.station_flags_msk = htole32(IWX_STA_FLG_DRAIN_FLOW);

	status = IWX_ADD_STA_SUCCESS;
#ifdef tbd
	ret = iwx_send_cmd_pdu_status(sc, IWX_ADD_STA,
					  iwx_add_sta_cmd_size(sc),
					  &cmd, &status);
#endif
	if (ret)
		return ret;

	switch (status & IWX_ADD_STA_STATUS_MASK) {
	case IWX_ADD_STA_SUCCESS:
		IWX_DPRINTF(sc, IWX_DEBUG_NODE,
		    "Frames for staid %d will drained in fw\n", IWX_STATION_ID);
		break;
	default:
		ret = EIO;
		device_printf(sc->sc_dev,
		    "Couldn't drain frames for staid %d\n", IWX_STATION_ID);
		break;
	}

	return ret;
}

/*
 * Remove a station from the FW table. Before sending the command to remove
 * the station validate that the station is indeed known to the driver (sanity
 * only).
 */
static int
iwx_rm_sta_common(struct iwx_softc *sc)
{
	struct iwx_rm_sta_cmd rm_sta_cmd = {
		.sta_id = IWX_STATION_ID,
	};
	int ret;

	ret = iwx_send_cmd_pdu(sc, IWX_REMOVE_STA, 0,
				   sizeof(rm_sta_cmd), &rm_sta_cmd);
	if (ret) {
		device_printf(sc->sc_dev,
		    "Failed to remove station. Id=%d\n", IWX_STATION_ID);
		return ret;
	}

	return 0;
}

int
iwx_rm_sta(struct iwx_softc *sc, struct ieee80211vap *vap,
	boolean_t is_assoc)
{
	uint32_t tfd_queue_msk = 0;
	int ret;
	int ac;

	ret = iwx_drain_sta(sc, IWX_VAP(vap), TRUE);
	if (ret)
		return ret;
	for (ac = 0; ac < WME_NUM_AC; ac++) {
		tfd_queue_msk |= htole32(1 << iwx_ac_to_tx_fifo[ac]);
	}
//	ret = iwx_flush_tx_path(sc, tfd_queue_msk, IWX_CMD_SYNC);
	ret = iwx_flush_tx_path(sc, IWX_CMD_SYNC);
	if (ret)
		return ret;
#ifdef notyet /* function not yet implemented */
	ret = iwl_trans_wait_tx_queue_empty(mvm->trans,
					    mvm_sta->tfd_queue_msk);
	if (ret)
		return ret;
#endif
	ret = iwx_drain_sta(sc, IWX_VAP(vap), FALSE);

	/* if we are associated - we can't remove the AP STA now */
	if (is_assoc)
		return ret;

	/* XXX wait until STA is drained */

	ret = iwx_rm_sta_common(sc);

	return ret;
}

int
iwx_rm_sta_id(struct iwx_softc *sc, struct ieee80211vap *vap)
{
	/* XXX wait until STA is drained */

	return iwx_rm_sta_common(sc);
}

static int
iwx_add_int_sta_common(struct iwx_softc *sc, struct iwx_int_sta *sta,
    const uint8_t *addr, uint16_t mac_id, uint16_t color)
{
	struct iwx_add_sta_cmd cmd;
	int ret = 0;
	uint32_t status = 0;

	memset(&cmd, 0, sizeof(cmd));
	cmd.sta_id = sta->sta_id;
	cmd.mac_id_n_color = htole32(IWX_FW_CMD_ID_AND_COLOR(mac_id, color));
#ifdef not_in_iwx
	if (sta->sta_id == IWX_AUX_STA_ID && sc->cfg->mqrx_supported)
		cmd.station_type = IWX_STA_AUX_ACTIVITY;
#endif

	cmd.tfd_queue_msk = htole32(sta->tfd_queue_msk);
	cmd.tid_disable_tx = htole16(0xffff);

	if (addr)
		IEEE80211_ADDR_COPY(cmd.addr, addr);

#ifdef tbd
	ret = iwx_send_cmd_pdu_status(sc, IWX_ADD_STA,
					  iwx_add_sta_cmd_size(sc),
					  &cmd, &status);
#endif
	if (ret)
		return ret;

	switch (status & IWX_ADD_STA_STATUS_MASK) {
	case IWX_ADD_STA_SUCCESS:
		IWX_DPRINTF(sc, IWX_DEBUG_NODE, "Internal station added.\n");
		return 0;
	default:
		ret = EIO;
		device_printf(sc->sc_dev,
		    "Add internal station failed, status=0x%x\n", status);
		break;
	}
	return ret;
}

int
iwx_add_aux_sta(struct iwx_softc *sc)
{
	int ret;

	sc->sc_aux_sta.sta_id = IWX_AUX_STA_ID;
	sc->sc_aux_sta.tfd_queue_msk = (1 << IWX_AUX_QUEUE);

	/* Map Aux queue to fifo - needs to happen before adding Aux station */
	ret = iwx_enable_txq(sc, IWX_AUX_STA_ID, IWX_DQA_AUX_QUEUE,
			IWX_MGMT_TID, IWX_TX_RING_COUNT);
	if (ret)
		return ret;

	ret = iwx_add_int_sta_common(sc, &sc->sc_aux_sta, NULL,
					 IWX_MAC_INDEX_AUX, 0);

	if (ret) {
		memset(&sc->sc_aux_sta, 0, sizeof(sc->sc_aux_sta));
		sc->sc_aux_sta.sta_id = IWX_STATION_COUNT;
#ifdef tbd
		sc->sc_aux_sta.station_type = IWX_STA_AUX_ACTIVITY;
#endif
	}
	return ret;
}
