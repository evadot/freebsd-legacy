/* copied from FreeBSD sys/dev/iwm/if_iwm_scan.c */

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
#include <dev/iwx/if_iwx_notif_wait.h>
#include <dev/iwx/if_iwx_util.h>
#include <dev/iwx/if_iwx_scan.h>

/*
 * BEGIN mvm/scan.c
 */

#define IWM_DENSE_EBS_SCAN_RATIO 5
#define IWM_SPARSE_EBS_SCAN_RATIO 1

#if 0
static uint32_t
iwm_scan_rxon_flags(struct ieee80211_channel *c)
{
	if (IEEE80211_IS_CHAN_2GHZ(c))
		return htole32(IWM_PHY_BAND_24);
	else
		return htole32(IWM_PHY_BAND_5);
}
#endif

static inline boolean_t
iwx_rrm_scan_needed(struct iwx_softc *sc)
{
	/* require rrm scan whenever the fw supports it */
	return iwx_fw_has_capa(sc, IWX_UCODE_TLV_CAPA_DS_PARAM_SET_IE_SUPPORT);
}

#ifdef IWX_DEBUG
static const char *
iwx_ebs_status_str(enum iwx_scan_ebs_status status)
{
	switch (status) {
	case IWX_SCAN_EBS_SUCCESS:
		return "successful";
	case IWX_SCAN_EBS_INACTIVE:
		return "inactive";
	case IWX_SCAN_EBS_FAILED:
	case IWX_SCAN_EBS_CHAN_NOT_FOUND:
	default:
		return "failed";
	}
}

static const char *
iwx_offload_status_str(enum iwx_scan_offload_complete_status status)
{
	return (status == IWX_SCAN_OFFLOAD_ABORTED) ? "aborted" : "completed";
}
#endif

#if 0
void
iwm_rx_lmac_scan_complete_notif(struct iwm_softc *sc,
    struct iwm_rx_packet *pkt)
{
	struct iwm_periodic_scan_complete *scan_notif = (void *)pkt->data;

	/* If this happens, the firmware has mistakenly sent an LMAC
	 * notification during UMAC scans -- warn and ignore it.
	 */
	if (iwm_fw_has_capa(sc, IWM_UCODE_TLV_CAPA_UMAC_SCAN)) {
		device_printf(sc->sc_dev,
		    "%s: Mistakenly got LMAC notification during UMAC scan\n",
		    __func__);
		return;
	}

	IWM_DPRINTF(sc, IWM_DEBUG_SCAN, "Regular scan %s, EBS status %s (FW)\n",
	    iwm_offload_status_str(scan_notif->status),
	    iwm_ebs_status_str(scan_notif->ebs_status));

	sc->last_ebs_successful =
			scan_notif->ebs_status == IWM_SCAN_EBS_SUCCESS ||
			scan_notif->ebs_status == IWM_SCAN_EBS_INACTIVE;

}
#endif

void
iwx_rx_umac_scan_complete_notif(struct iwx_softc *sc,
    struct iwx_rx_packet *pkt)
{
	struct iwx_umac_scan_complete *notif = (void *)pkt->data;

	IWX_DPRINTF(sc, IWX_DEBUG_SCAN,
	    "Scan completed, uid %u, status %s, EBS status %s\n",
	    le32toh(notif->uid),
	    iwx_offload_status_str(notif->status),
	    iwx_ebs_status_str(notif->ebs_status));

	if (notif->ebs_status != IWX_SCAN_EBS_SUCCESS &&
	    notif->ebs_status != IWX_SCAN_EBS_INACTIVE)
		sc->last_ebs_successful = FALSE;
}

static int
iwx_scan_skip_channel(struct ieee80211_channel *c)
{
	if (IEEE80211_IS_CHAN_2GHZ(c) && IEEE80211_IS_CHAN_B(c))
		return 0;
	else if (IEEE80211_IS_CHAN_5GHZ(c) && IEEE80211_IS_CHAN_A(c))
		return 0;
	else
		return 1;
}

static uint8_t
iwx_umac_scan_fill_channels(struct iwx_softc *sc,
    struct iwx_scan_channel_cfg_umac *chan, int n_ssids)
{
	struct ieee80211com *ic = &sc->sc_ic;
	struct ieee80211_scan_state *ss = ic->ic_scan;
	struct ieee80211_channel *c;
	uint8_t nchan;
	int j;

	for (nchan = j = 0;
	    j < ss->ss_last && nchan < sc->sc_fw.ucode_capa.n_scan_channels;
	    j++) {
		c = ss->ss_chans[j];
		/*
		 * Catch other channels, in case we have 900MHz channels or
		 * something in the chanlist.
		 */
		if (!IEEE80211_IS_CHAN_2GHZ(c) && !IEEE80211_IS_CHAN_5GHZ(c)) {
			IWX_DPRINTF(sc, IWX_DEBUG_RESET | IWX_DEBUG_EEPROM,
			    "%s: skipping channel (freq=%d, ieee=%d, flags=0x%08x)\n",
			    __func__, c->ic_freq, c->ic_ieee, c->ic_flags);
			continue;
		}

		IWX_DPRINTF(sc, IWX_DEBUG_RESET | IWX_DEBUG_EEPROM,
		    "Adding channel %d (%d Mhz) to the list\n",
		    nchan, c->ic_freq);
		chan->channel_num = ieee80211_mhz2ieee(c->ic_freq, 0);
		chan->iter_count = 1;
		chan->iter_interval = htole16(0);
		chan->flags = htole32(IWX_SCAN_CHANNEL_UMAC_NSSIDS(n_ssids));
		chan++;
		nchan++;
	}

	return nchan;
}

static int
iwx_fill_probe_req(struct iwx_softc *sc, struct iwx_scan_probe_req *preq)
{
	struct ieee80211com *ic = &sc->sc_ic;
	struct ieee80211vap *vap = TAILQ_FIRST(&ic->ic_vaps);
	struct ieee80211_frame *wh = (struct ieee80211_frame *)preq->buf;
	struct ieee80211_rateset *rs;
	size_t remain = sizeof(preq->buf);
	uint8_t *frm, *pos;

	memset(preq, 0, sizeof(*preq));

	/* Ensure enough space for header and SSID IE. */
	if (remain < sizeof(*wh) + 2)
		return ENOBUFS;

	/*
	 * Build a probe request frame.  Most of the following code is a
	 * copy & paste of what is done in net80211.
	 */
	wh->i_fc[0] = IEEE80211_FC0_VERSION_0 | IEEE80211_FC0_TYPE_MGT |
	    IEEE80211_FC0_SUBTYPE_PROBE_REQ;
	wh->i_fc[1] = IEEE80211_FC1_DIR_NODS;
	IEEE80211_ADDR_COPY(wh->i_addr1, ieee80211broadcastaddr);
	IEEE80211_ADDR_COPY(wh->i_addr2, vap ? vap->iv_myaddr : ic->ic_macaddr);
	IEEE80211_ADDR_COPY(wh->i_addr3, ieee80211broadcastaddr);
	*(uint16_t *)&wh->i_dur[0] = 0; /* filled by HW */
	*(uint16_t *)&wh->i_seq[0] = 0; /* filled by HW */

	frm = (uint8_t *)(wh + 1);
	frm = ieee80211_add_ssid(frm, NULL, 0);

	/* Tell the firmware where the MAC header is. */
	preq->mac_header.offset = 0;
	preq->mac_header.len = htole16(frm - (uint8_t *)wh);
	remain -= frm - (uint8_t *)wh;

	/* Fill in 2GHz IEs and tell firmware where they are. */
	rs = &ic->ic_sup_rates[IEEE80211_MODE_11G];
	if (rs->rs_nrates > IEEE80211_RATE_SIZE) {
		if (remain < 4 + rs->rs_nrates)
			return ENOBUFS;
	} else if (remain < 2 + rs->rs_nrates) {
		return ENOBUFS;
	}
	preq->band_data[0].offset = htole16(frm - (uint8_t *)wh);
	pos = frm;
	frm = ieee80211_add_rates(frm, rs);
	if (rs->rs_nrates > IEEE80211_RATE_SIZE)
		frm = ieee80211_add_xrates(frm, rs);
	preq->band_data[0].len = htole16(frm - pos);
	remain -= frm - pos;

	if (iwx_rrm_scan_needed(sc)) {
		if (remain < 3)
			return ENOBUFS;
		*frm++ = IEEE80211_ELEMID_DSPARMS;
		*frm++ = 1;
		*frm++ = 0;
		remain -= 3;
	}

	if (sc->nvm_data->sku_cap_band_52GHz_enable) {
		/* Fill in 5GHz IEs. */
		rs = &ic->ic_sup_rates[IEEE80211_MODE_11A];
		if (rs->rs_nrates > IEEE80211_RATE_SIZE) {
			if (remain < 4 + rs->rs_nrates)
				return ENOBUFS;
		} else if (remain < 2 + rs->rs_nrates) {
			return ENOBUFS;
		}
		preq->band_data[1].offset = htole16(frm - (uint8_t *)wh);
		pos = frm;
		frm = ieee80211_add_rates(frm, rs);
		if (rs->rs_nrates > IEEE80211_RATE_SIZE)
			frm = ieee80211_add_xrates(frm, rs);
		preq->band_data[1].len = htole16(frm - pos);
		remain -= frm - pos;
	}

	/* Send 11n IEs on both 2GHz and 5GHz bands. */
	preq->common_data.offset = htole16(frm - (uint8_t *)wh);
	pos = frm;
#if 0
	if (ic->ic_flags & IEEE80211_F_HTON) {
		if (remain < 28)
			return ENOBUFS;
		frm = ieee80211_add_htcaps(frm, ic);
		/* XXX add WME info? */
	}
#endif
	preq->common_data.len = htole16(frm - pos);

	return 0;
}

int
iwx_config_umac_scan(struct iwx_softc *sc)
{
	struct ieee80211com *ic = &sc->sc_ic;
	struct ieee80211vap *vap = TAILQ_FIRST(&ic->ic_vaps);

	struct iwx_scan_config *scan_config;
	int ret, j, nchan;
	size_t cmd_size;
	struct ieee80211_channel *c;
	struct iwx_host_cmd hcmd = {
		.id = iwx_cmd_id(IWX_SCAN_CFG_CMD, IWX_ALWAYS_LONG_GROUP, 0),
		.flags = IWX_CMD_SYNC,
	};
	static const uint32_t rates = (IWX_SCAN_CONFIG_RATE_1M |
	    IWX_SCAN_CONFIG_RATE_2M | IWX_SCAN_CONFIG_RATE_5M |
	    IWX_SCAN_CONFIG_RATE_11M | IWX_SCAN_CONFIG_RATE_6M |
	    IWX_SCAN_CONFIG_RATE_9M | IWX_SCAN_CONFIG_RATE_12M |
	    IWX_SCAN_CONFIG_RATE_18M | IWX_SCAN_CONFIG_RATE_24M |
	    IWX_SCAN_CONFIG_RATE_36M | IWX_SCAN_CONFIG_RATE_48M |
	    IWX_SCAN_CONFIG_RATE_54M);

	cmd_size = sizeof(*scan_config) + sc->sc_fw.ucode_capa.n_scan_channels;

	scan_config = malloc(cmd_size, M_DEVBUF, M_NOWAIT | M_ZERO);
	if (scan_config == NULL)
		return ENOMEM;

	scan_config->tx_chains = htole32(iwx_get_valid_tx_ant(sc));
	scan_config->rx_chains = htole32(iwx_get_valid_rx_ant(sc));
	scan_config->legacy_rates = htole32(rates |
	    IWX_SCAN_CONFIG_SUPPORTED_RATE(rates));

	/* These timings correspond to iwlwifi's UNASSOC scan. */
	scan_config->dwell_active = 10;
	scan_config->dwell_passive = 110;
	scan_config->dwell_fragmented = 44;
	scan_config->dwell_extended = 90;
	scan_config->out_of_channel_time[IWX_SCAN_LB_LMAC_IDX] = htole32(0);
	scan_config->out_of_channel_time[IWX_SCAN_HB_LMAC_IDX] = htole32(0);
	scan_config->suspend_time[IWX_SCAN_LB_LMAC_IDX] = htole32(0);
	scan_config->suspend_time[IWX_SCAN_HB_LMAC_IDX] = htole32(0);

	IEEE80211_ADDR_COPY(scan_config->mac_addr,
	    vap ? vap->iv_myaddr : ic->ic_macaddr);

	scan_config->bcast_sta_id = sc->sc_aux_sta.sta_id;
	scan_config->channel_flags = IWX_CHANNEL_FLAG_EBS |
	    IWX_CHANNEL_FLAG_ACCURATE_EBS | IWX_CHANNEL_FLAG_EBS_ADD |
	    IWX_CHANNEL_FLAG_PRE_SCAN_PASSIVE2ACTIVE;

	for (nchan = j = 0;
	    j < ic->ic_nchans && nchan < sc->sc_fw.ucode_capa.n_scan_channels;
	    j++) {
		c = &ic->ic_channels[j];
		/* For 2GHz, only populate 11b channels */
		/* For 5GHz, only populate 11a channels */
		/*
		 * Catch other channels, in case we have 900MHz channels or
		 * something in the chanlist.
		 */
		if (iwx_scan_skip_channel(c))
			continue;
		scan_config->channel_array[nchan++] =
		    ieee80211_mhz2ieee(c->ic_freq, 0);
	}

	scan_config->flags = htole32(IWX_SCAN_CONFIG_FLAG_ACTIVATE |
	    IWX_SCAN_CONFIG_FLAG_ALLOW_CHUB_REQS |
	    IWX_SCAN_CONFIG_FLAG_SET_TX_CHAINS |
	    IWX_SCAN_CONFIG_FLAG_SET_RX_CHAINS |
	    IWX_SCAN_CONFIG_FLAG_SET_AUX_STA_ID |
	    IWX_SCAN_CONFIG_FLAG_SET_ALL_TIMES |
	    IWX_SCAN_CONFIG_FLAG_SET_LEGACY_RATES |
	    IWX_SCAN_CONFIG_FLAG_SET_MAC_ADDR |
	    IWX_SCAN_CONFIG_FLAG_SET_CHANNEL_FLAGS|
	    IWX_SCAN_CONFIG_N_CHANNELS(nchan) |
	    IWX_SCAN_CONFIG_FLAG_CLEAR_FRAGMENTED);

	hcmd.data[0] = scan_config;
	hcmd.len[0] = cmd_size;

	IWX_DPRINTF(sc, IWX_DEBUG_SCAN, "Sending UMAC scan config\n");

	ret = iwx_send_cmd(sc, &hcmd);
	if (!ret)
		IWX_DPRINTF(sc, IWX_DEBUG_SCAN,
		    "UMAC scan config was sent successfully\n");

	free(scan_config, M_DEVBUF);
	return ret;
}

static boolean_t
iwx_scan_use_ebs(struct iwx_softc *sc)
{
	const struct iwx_ucode_capabilities *capa = &sc->sc_fw.ucode_capa;

	/* We can only use EBS if:
	 *	1. the feature is supported;
	 *	2. the last EBS was successful;
	 *	3. if only single scan, the single scan EBS API is supported;
	 *	4. it's not a p2p find operation.
	 */
	return ((capa->flags & IWX_UCODE_TLV_FLAGS_EBS_SUPPORT) &&
		sc->last_ebs_successful);
}

static int
iwx_scan_size(struct iwx_softc *sc)
{
	int base_size;

	if (iwx_fw_has_capa(sc, IWX_UCODE_TLV_CAPA_UMAC_SCAN)) {
		if (iwx_fw_has_api(sc, IWX_UCODE_TLV_API_ADAPTIVE_DWELL))
			base_size = IWX_SCAN_REQ_UMAC_SIZE_V7;
		else
			base_size = IWX_SCAN_REQ_UMAC_SIZE_V1;

		return base_size +
		    sizeof(struct iwx_scan_channel_cfg_umac) *
		    sc->sc_fw.ucode_capa.n_scan_channels +
		    sizeof(struct iwx_scan_req_umac_tail);
	} else {
		return sizeof(struct iwx_scan_req_lmac) +
		    sizeof(struct iwx_scan_channel_cfg_lmac) *
		    sc->sc_fw.ucode_capa.n_scan_channels +
		    sizeof(struct iwx_scan_probe_req);
	}
}

int
iwx_umac_scan(struct iwx_softc *sc)
{
	struct iwx_host_cmd hcmd = {
		.id = iwx_cmd_id(IWX_SCAN_REQ_UMAC, IWX_ALWAYS_LONG_GROUP, 0),
		.len = { 0, },
		.data = { NULL, },
		.flags = IWX_CMD_SYNC,
	};
	struct ieee80211_scan_state *ss = sc->sc_ic.ic_scan;
	struct iwx_scan_req_umac *req;
	struct iwx_scan_req_umac_tail *tail;
	size_t req_len;
	uint16_t general_flags;
	uint8_t channel_flags, i, nssid;
	int ret;

	req_len = iwx_scan_size(sc);
	if (req_len > IWX_MAX_CMD_PAYLOAD_SIZE)
		return ENOMEM;
	req = malloc(req_len, M_DEVBUF, M_NOWAIT | M_ZERO);
	if (req == NULL)
		return ENOMEM;

	hcmd.len[0] = (uint16_t)req_len;
	hcmd.data[0] = (void *)req;

	IWX_DPRINTF(sc, IWX_DEBUG_SCAN, "Handling ieee80211 scan request\n");

	nssid = MIN(ss->ss_nssid, IWX_PROBE_OPTION_MAX);

	general_flags = IWX_UMAC_SCAN_GEN_FLAGS_PASS_ALL |
	    IWX_UMAC_SCAN_GEN_FLAGS_ITER_COMPLETE;
	/* guesswork, see openbsd sys/dev/pci/if_iwx.c:5002 */
	if (iwx_fw_has_api(sc, IWX_UCODE_TLV_API_ADAPTIVE_DWELL_V2))
		general_flags |= IWX_UMAC_SCAN_GEN_FLAGS2_ALLOW_CHNL_REORDER;
	if (!iwx_fw_has_api(sc, IWX_UCODE_TLV_API_ADAPTIVE_DWELL))
		general_flags |= IWX_UMAC_SCAN_GEN_FLAGS_EXTENDED_DWELL;
	if (iwx_rrm_scan_needed(sc))
		general_flags |= IWX_UMAC_SCAN_GEN_FLAGS_RRM_ENABLED;
#if 0 // guesswork
	if (nssid != 0)
		general_flags |= IWM_UMAC_SCAN_GEN_FLAGS_PRE_CONNECT;
	else
#endif
	general_flags |= IWX_UMAC_SCAN_GEN_FLAGS_PASSIVE;

	channel_flags = 0;
	if (iwx_scan_use_ebs(sc))
		channel_flags = IWX_SCAN_CHANNEL_FLAG_EBS |
		    IWX_SCAN_CHANNEL_FLAG_EBS_ACCURATE |
		    IWX_SCAN_CHANNEL_FLAG_CACHE_ADD;

	req->general_flags = htole16(general_flags);
	req->ooc_priority = htole32(IWX_SCAN_PRIORITY_HIGH);

	/* These timings correspond to iwlwifi's UNASSOC scan. */
	if (iwx_fw_has_api(sc, IWX_UCODE_TLV_API_ADAPTIVE_DWELL)) {
		req->v7.active_dwell = 10;
		req->v7.passive_dwell = 110;
		req->v7.fragmented_dwell = 44;
		req->v7.adwell_default_n_aps_social = 10;
		req->v7.adwell_default_n_aps = 2;
		req->v7.adwell_max_budget = htole16(300);
		req->v7.scan_priority = htole32(IWX_SCAN_PRIORITY_HIGH);
		req->v7.channel.flags = channel_flags;
		req->v7.channel.count = iwx_umac_scan_fill_channels(sc,
		    (struct iwx_scan_channel_cfg_umac *)req->v7.data, nssid);

		tail = (void *)((char *)&req->v7.data +
		    sizeof(struct iwx_scan_channel_cfg_umac) *
		    sc->sc_fw.ucode_capa.n_scan_channels);
	} else {
		req->v1.active_dwell = 10;
		req->v1.passive_dwell = 110;
		req->v1.fragmented_dwell = 44;
		req->v1.extended_dwell = 90;
		req->v1.scan_priority = htole32(IWX_SCAN_PRIORITY_HIGH);
		req->v1.channel.flags = channel_flags;
		req->v1.channel.count = iwx_umac_scan_fill_channels(sc,
		    (struct iwx_scan_channel_cfg_umac *)req->v1.data, nssid);

		tail = (void *)((char *)&req->v1.data +
		    sizeof(struct iwx_scan_channel_cfg_umac) *
		    sc->sc_fw.ucode_capa.n_scan_channels);
	}

	/* Check if we're doing an active directed scan. */
	for (i = 0; i < nssid; i++) {
		tail->direct_scan[i].id = IEEE80211_ELEMID_SSID;
		tail->direct_scan[i].len = MIN(ss->ss_ssid[i].len,
		    IEEE80211_NWID_LEN);
		memcpy(tail->direct_scan[i].ssid, ss->ss_ssid[i].ssid,
		    tail->direct_scan[i].len);
		/* XXX debug */
	}

	ret = iwx_fill_probe_req(sc, &tail->preq);
	if (ret) {
		free(req, M_DEVBUF);
		return ret;
	}

	/* Specify the scan plan: We'll do one iteration. */
	tail->schedule[0].interval = 0;
	tail->schedule[0].iter_count = 1;

	ret = iwx_send_cmd(sc, &hcmd);
	if (!ret)
		IWX_DPRINTF(sc, IWX_DEBUG_SCAN,
		    "Scan request was sent successfully\n");
	free(req, M_DEVBUF);
	return ret;
}

//#ifdef not_in_iwx
static int
iwx_lmac_scan_abort(struct iwx_softc *sc)
{
	int ret;
	struct iwx_host_cmd hcmd = {
		.id = IWX_SCAN_OFFLOAD_ABORT_CMD,
		.len = { 0, },
		.data = { NULL, },
		.flags = IWX_CMD_SYNC,
	};
	uint32_t status;

	ret = iwx_send_cmd_status(sc, &hcmd, &status);
	if (ret)
		return ret;

	if (status != IWX_CAN_ABORT_STATUS) {
		/*
		 * The scan abort will return 1 for success or
		 * 2 for "failure".  A failure condition can be
		 * due to simply not being in an active scan which
		 * can occur if we send the scan abort before the
		 * microcode has notified us that a scan is completed.
		 */
		IWX_DPRINTF(sc, IWX_DEBUG_SCAN,
		    "SCAN OFFLOAD ABORT ret %d.\n", status);
		ret = ENOENT;
	}

	return ret;
}
//#endif

static int
iwx_umac_scan_abort(struct iwx_softc *sc)
{
	struct iwx_umac_scan_abort cmd = {};
	int uid, ret;

	uid = 0;
	cmd.uid = htole32(uid);

	IWX_DPRINTF(sc, IWX_DEBUG_SCAN, "Sending scan abort, uid %u\n", uid);

	ret = iwx_send_cmd_pdu(sc,
				   iwx_cmd_id(IWX_SCAN_ABORT_UMAC,
					      IWX_ALWAYS_LONG_GROUP, 0),
				   0, sizeof(cmd), &cmd);

	return ret;
}
int	iwx_scan_abort(struct iwx_softc *);
 
/* source: OpenBSD */
int
iwx_scan_abort(struct iwx_softc *sc)
{
	int err;

	err = iwx_umac_scan_abort(sc);
	if (err == 0)
		sc->sc_flags &= ~(IWX_FLAG_SCANNING | IWX_FLAG_BGSCAN);
	return err;
}

/* source: OpenBSD */
int
iwx_enable_data_tx_queues(struct iwx_softc *sc)
{
	int err, ac;

	/* openbsd: sys/net80211/ieee80211.h #define EDCA_NUM_AC 4 */
//	for (ac = 0; ac < EDCA_NUM_AC; ac++) {
	for (ac = 0; ac < 4; ac++) {
		int qid = ac + IWX_DQA_AUX_QUEUE + 1;
		/*
		 * Regular data frames use the "MGMT" TID and queue.
		 * Other TIDs and queues are reserved for frame aggregation.
		 */
		err = iwx_enable_txq(sc, IWX_STATION_ID, qid, IWX_MGMT_TID,
		    IWX_TX_RING_COUNT);
		if (err) {
			device_printf(sc->sc_dev, "could not enable Tx queue %d (error %d)\n", ac, err);
			return err;
		}
	}

	return 0;
}

int
iwx_scan_stop_wait(struct iwx_softc *sc)
{
	struct iwx_notification_wait wait_scan_done;
	static const uint16_t scan_done_notif[] = { IWX_SCAN_COMPLETE_UMAC,
						   IWX_SCAN_OFFLOAD_COMPLETE, };
	int ret;

	iwx_init_notification_wait(sc->sc_notif_wait, &wait_scan_done,
				   scan_done_notif, nitems(scan_done_notif),
				   NULL, NULL);

	IWX_DPRINTF(sc, IWX_DEBUG_SCAN, "Preparing to stop scan\n");

	if (iwx_fw_has_capa(sc, IWX_UCODE_TLV_CAPA_UMAC_SCAN))
		ret = iwx_umac_scan_abort(sc);
	else
		ret = iwx_lmac_scan_abort(sc);

	if (ret) {
		IWX_DPRINTF(sc, IWX_DEBUG_SCAN, "couldn't stop scan\n");
		iwx_remove_notification(sc->sc_notif_wait, &wait_scan_done);
		return ret;
	}

	IWX_UNLOCK(sc);
	ret = iwx_wait_notification(sc->sc_notif_wait, &wait_scan_done, hz);
	IWX_LOCK(sc);

	return ret;
}
