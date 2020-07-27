/* copied from FreeBSD sys/dev/iwx/if_iwx.c */

/*	$OpenBSD: if_iwm.c,v 1.167 2017/04/04 00:40:52 claudio Exp $	*/

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
#include <dev/iwx/if_iwx_notif_wait.h>
#include <dev/iwx/if_iwx_util.h>
#include <dev/iwx/if_iwx_binding.h>
#include <dev/iwx/if_iwx_phy_db.h>
#include <dev/iwx/if_iwx_mac_ctxt.h>
#include <dev/iwx/if_iwx_phy_ctxt.h>
#include <dev/iwx/if_iwx_time_event.h>
#include <dev/iwx/if_iwx_power.h>
#include <dev/iwx/if_iwx_scan.h>
#include <dev/iwx/if_iwx_sf.h>
#include <dev/iwx/if_iwx_sta.h>

#include <dev/iwx/if_iwx_pcie_trans.h>
#include <dev/iwx/if_iwx_fw.h>

/* From DragonflyBSD */
#define mtodoff(m, t, off)      ((t)((m)->m_data + (off)))

const uint8_t iwx_nvm_channels[] = {
	/* 2.4 GHz */
	1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14,
	/* 5 GHz */
	36, 40, 44, 48, 52, 56, 60, 64, 68, 72, 76, 80, 84, 88, 92,
	96, 100, 104, 108, 112, 116, 120, 124, 128, 132, 136, 140, 144,
	149, 153, 157, 161, 165, 169, 173, 177, 181
};
_Static_assert(nitems(iwx_nvm_channels) <= IWX_NUM_CHANNELS,
    "IWX_NUM_CHANNELS is too small");

#define IWX_NUM_2GHZ_CHANNELS	14
#define IWX_N_HW_ADDR_MASK	0xF

/*
 * XXX For now, there's simply a fixed set of rate table entries
 * that are populated.
 */
const struct iwx_rate {
	uint16_t rate;
	uint8_t plcp;
} iwx_rates[] = {
	{   2,	IWM_RATE_1M_PLCP  },
	{   4,	IWM_RATE_2M_PLCP  },
	{  11,	IWM_RATE_5M_PLCP  },
	{  22,	IWM_RATE_11M_PLCP },
	{  12,	IWM_RATE_6M_PLCP  },
	{  18,	IWM_RATE_9M_PLCP  },
	{  24,	IWM_RATE_12M_PLCP },
	{  36,	IWM_RATE_18M_PLCP },
	{  48,	IWM_RATE_24M_PLCP },
	{  72,	IWM_RATE_36M_PLCP },
	{  96,	IWM_RATE_48M_PLCP },
	{ 108,	IWM_RATE_54M_PLCP },
	/* todo if_iwx: implement other rates (128 up to and including 260) */
};

/* unused in iwm and in iwx */
#ifdef not_in_iwx
// #define IWM_RIDX_CCK	0
#endif
#define IWX_RIDX_OFDM	4
#define IWX_RIDX_MAX	(nitems(iwx_rates)-1)
#define IWX_RIDX_IS_CCK(_i_) ((_i_) < IWX_RIDX_OFDM)
#define IWX_RIDX_IS_OFDM(_i_) ((_i_) >= IWX_RIDX_OFDM)

struct iwx_nvm_section {
	uint16_t length;
	uint8_t *data;
};

#define IWX_UCODE_ALIVE_TIMEOUT	hz
#define IWX_UCODE_CALIB_TIMEOUT	(2*hz)

struct iwx_alive_data {
	int valid;
	uint32_t scd_base_addr;
};

/* copy/paste from freebsd's sys/dev/iwm/iwm.c */
static int	iwx_store_cscheme(struct iwx_softc *, const uint8_t *, size_t);
static int	iwx_firmware_store_section(struct iwx_softc *,
                                           enum iwx_ucode_type,
                                           const uint8_t *, size_t);
static int	iwx_set_default_calib(struct iwx_softc *, const void *);
static void	iwx_fw_info_free(struct iwx_fw_info *);
static int	iwx_read_firmware(struct iwx_softc *);
static int	iwx_alloc_fwmem(struct iwx_softc *);
static int	iwx_alloc_sched(struct iwx_softc *);
static int	iwx_alloc_ict(struct iwx_softc *);
static int	iwx_alloc_rx_ring(struct iwx_softc *, struct iwx_rx_ring *);
static void	iwx_reset_rx_ring(struct iwx_softc *, struct iwx_rx_ring *);
static void	iwx_free_rx_ring(struct iwx_softc *, struct iwx_rx_ring *);
static int	iwx_alloc_tx_ring(struct iwx_softc *, struct iwx_tx_ring *,
                                  int);
static void	iwx_reset_tx_ring(struct iwx_softc *, struct iwx_tx_ring *);
static void	iwx_free_tx_ring(struct iwx_softc *, struct iwx_tx_ring *);
static void	iwx_enable_interrupts(struct iwx_softc *);
static void	iwx_restore_interrupts(struct iwx_softc *);
static void	iwx_disable_interrupts(struct iwx_softc *);
static void	iwx_ict_reset(struct iwx_softc *);
static int	iwx_allow_mcast(struct ieee80211vap *, struct iwx_softc *);
static void	iwx_stop_device(struct iwx_softc *);
static void	iwx_nic_config(struct iwx_softc *);
static int	iwx_nic_rx_init(struct iwx_softc *);
static int	iwx_nic_tx_init(struct iwx_softc *, int, int);
static int	iwx_nic_init(struct iwx_softc *);
static void	iwx_trans_pcie_fw_alive(struct iwx_softc *, uint32_t);
static int	iwx_nvm_read_chunk(struct iwx_softc *, uint16_t, uint16_t,
                                   uint16_t, uint8_t *, uint16_t *);
static int	iwx_nvm_read_section(struct iwx_softc *, uint16_t, uint8_t *,
				     uint16_t *, uint32_t);
static uint32_t	iwx_eeprom_channel_flags(uint16_t);
static void	iwx_add_channel_band(struct iwx_softc *,
		    struct ieee80211_channel[], int, int *, int, size_t,
		    const uint8_t[]);
static void	iwx_init_channel_map(struct ieee80211com *, int, int *,
		    struct ieee80211_channel[]);
static struct iwx_nvm_data *
	iwx_parse_nvm_data(struct iwx_softc *, const uint16_t *,
			   const uint16_t *, const uint16_t *,
			   const uint16_t *, const uint16_t *,
			   const uint16_t *);
static void	iwx_free_nvm_data(struct iwx_nvm_data *);
static void	iwx_set_hw_address_family_8000(struct iwx_softc *,
					       struct iwx_nvm_data *,
					       const uint16_t *,
					       const uint16_t *);
static int	iwx_get_sku(const struct iwx_softc *, const uint16_t *,
			    const uint16_t *);
static int	iwx_get_nvm_version(const struct iwx_softc *, const uint16_t *);
static int	iwx_get_radio_cfg(const struct iwx_softc *, const uint16_t *,
				  const uint16_t *);
static int	iwx_get_n_hw_addrs(const struct iwx_softc *,
				   const uint16_t *);
static void	iwx_set_radio_cfg(const struct iwx_softc *,
				  struct iwx_nvm_data *, uint32_t);
static struct iwx_nvm_data *
	iwx_parse_nvm_sections(struct iwx_softc *, struct iwx_nvm_section *);
static int	iwx_nvm_init(struct iwx_softc *);
static int	iwx_pcie_load_given_ucode(struct iwx_softc *,
					  const struct iwx_fw_img *);
static int	iwx_start_fw(struct iwx_softc *, const struct iwx_fw_img *);
static int	iwx_send_tx_ant_cfg(struct iwx_softc *, uint8_t);
static int	iwx_send_phy_cfg_cmd(struct iwx_softc *);
static int	iwx_load_ucode_wait_alive(struct iwx_softc *);
static int	iwx_run_init_unified_ucode(struct iwx_softc *, bool);
static int	iwx_config_ltr(struct iwx_softc *sc);
static int	iwx_rx_addbuf(struct iwx_softc *, int, int);
static void	iwx_rx_rx_phy_cmd(struct iwx_softc *,
                                      struct iwx_rx_packet *);
static int	iwx_get_noise(struct iwx_softc *,
		    const struct iwx_statistics_rx_non_phy *);
static void	iwx_handle_rx_statistics(struct iwx_softc *,
		    struct iwx_rx_packet *);
static bool	iwx_rx_mpdu(struct iwx_softc *, struct mbuf *,
		    uint32_t, bool);
static int	iwx_rx_tx_cmd_single(struct iwx_softc *,
                                         struct iwx_rx_packet *,
				         struct iwx_node *);
static void	iwx_rx_tx_cmd(struct iwx_softc *, struct iwx_rx_packet *);
// static void	iwx_cmd_done(struct iwx_softc *, struct iwx_rx_packet *);
static void	iwx_cmd_done(struct iwx_softc *, struct iwx_rx_packet *, int code);
// static const struct iwx_rate *
//	iwx_tx_fill_cmd(struct iwx_softc *, struct iwx_node *,
//			struct mbuf *, struct iwx_tx_cmd *);
static const struct iwx_rate *
	iwx_tx_fill_cmd(struct iwx_softc *, struct iwx_node *,
			struct mbuf *, struct iwx_tx_cmd_gen2 *);
static int	iwx_tx(struct iwx_softc *, struct mbuf *,
                       struct ieee80211_node *, int);
static int	iwx_raw_xmit(struct ieee80211_node *, struct mbuf *,
			     const struct ieee80211_bpf_params *);
static int	iwx_update_quotas(struct iwx_softc *, struct iwx_vap *);
static int	iwx_auth(struct ieee80211vap *, struct iwx_softc *);
static struct ieee80211_node *
		iwx_node_alloc(struct ieee80211vap *,
		               const uint8_t[IEEE80211_ADDR_LEN]);
static uint8_t	iwx_rate_from_ucode_rate(uint32_t);
static int	iwx_rate2ridx(struct iwx_softc *, uint8_t);
static void	iwx_setrates(struct iwx_softc *, struct iwx_node *, int);
static int	iwx_media_change(struct ifnet *);
static int	iwx_newstate(struct ieee80211vap *, enum ieee80211_state, int);
static void	iwx_endscan_cb(void *, int);
#if 0
static int	iwx_send_bt_init_conf(struct iwx_softc *);
#endif
static boolean_t iwx_is_lar_supported(struct iwx_softc *);
static boolean_t iwx_is_wifi_mcc_supported(struct iwx_softc *);
static int	iwx_send_update_mcc_cmd(struct iwx_softc *, const char *);
static int	iwx_init_hw(struct iwx_softc *);
static void	iwx_init(struct iwx_softc *);
static void	iwx_start(struct iwx_softc *);
static void	iwx_stop(struct iwx_softc *);
static void	iwx_watchdog(void *);
static void	iwx_parent(struct ieee80211com *);
#ifdef IWX_DEBUG
static const char *
		iwx_desc_lookup(uint32_t);
static void	iwx_nic_error(struct iwx_softc *);
static void	iwx_nic_umac_error(struct iwx_softc *);
#endif
static void	iwx_handle_rxb(struct iwx_softc *, struct mbuf *);
static void	iwx_notif_intr(struct iwx_softc *);
static void	iwx_intr(void *);
static int	iwx_attach(device_t);
static int	iwx_is_valid_ether_addr(uint8_t *);
static void	iwx_preinit(void *);
static int	iwx_detach_local(struct iwx_softc *sc, int);
static void	iwx_init_task(void *);
static void	iwx_radiotap_attach(struct iwx_softc *);
static struct ieee80211vap *
		iwx_vap_create(struct ieee80211com *,
		               const char [IFNAMSIZ], int,
		               enum ieee80211_opmode, int,
		               const uint8_t [IEEE80211_ADDR_LEN],
		               const uint8_t [IEEE80211_ADDR_LEN]);
static void	iwx_vap_delete(struct ieee80211vap *);
static void	iwx_xmit_queue_drain(struct iwx_softc *);
static void	iwx_scan_start(struct ieee80211com *);
static void	iwx_scan_end(struct ieee80211com *);
static void	iwx_update_mcast(struct ieee80211com *);
static void	iwx_set_channel(struct ieee80211com *);
static void	iwx_scan_curchan(struct ieee80211_scan_state *, unsigned long);
static void	iwx_scan_mindwell(struct ieee80211_scan_state *);
static int	iwx_detach(device_t);
/* end copy/paste */

/* prototypes present in openbsd's if_iwx.c */
int	iwx_ctxt_info_init(struct iwx_softc *, const struct iwx_fw_img *);
void	iwx_ctxt_info_free(struct iwx_softc *);
void	iwx_ctxt_info_free_paging(struct iwx_softc *);
int	iwx_enable_txq(struct iwx_softc *, int, int, int, int);
int	iwx_send_dqa_cmd(struct iwx_softc *);
void	iwx_update_rx_desc(struct iwx_softc *, struct iwx_rx_ring *, int);
void	iwx_tx_update_byte_tbl(struct iwx_tx_ring *, uint16_t, uint16_t);
/* end openbsd */

static int	iwx_lar_disable = 0;
TUNABLE_INT("hw.iwx.lar.disable", &iwx_lar_disable);

/*
 * Firmware parser.
 */

static int
iwx_store_cscheme(struct iwx_softc *sc, const uint8_t *data, size_t dlen)
{
	const struct iwx_fw_cscheme_list *l = (const void *)data;

	if (dlen < sizeof(*l) ||
	    dlen < sizeof(l->size) + l->size * sizeof(*l->cs))
		return EINVAL;

	/* we don't actually store anything for now, always use s/w crypto */

	return 0;
}

static int
iwx_firmware_store_section(struct iwx_softc *sc,
    enum iwx_ucode_type type, const uint8_t *data, size_t dlen)
{
	struct iwx_fw_img *fws;
	struct iwx_fw_desc *fwone;

	if (type >= IWX_UCODE_TYPE_MAX)
		return EINVAL;
	if (dlen < sizeof(uint32_t))
		return EINVAL;

	fws = &sc->sc_fw.img[type];
	if (fws->fw_count >= IWX_UCODE_SECTION_MAX)
		return EINVAL;

	fwone = &fws->fw_sect[fws->fw_count];

	/* first 32bit are device load offset */
	memcpy(&fwone->offset, data, sizeof(uint32_t));

	/* rest is data */
	fwone->data = data + sizeof(uint32_t);
	fwone->len = dlen - sizeof(uint32_t);

	fws->fw_count++;

	return 0;
}

#define IWX_DEFAULT_SCAN_CHANNELS 40
/* Newer firmware might support more channels. Raise this value if needed. */
#define IWX_MAX_SCAN_CHANNELS		52 /* as of 8265-34 firmware image */

/* iwlwifi: iwl-drv.c */
struct iwx_tlv_calib_data {
	uint32_t ucode_type;
	struct iwx_tlv_calib_ctrl calib;
} __packed;

static int
iwx_set_default_calib(struct iwx_softc *sc, const void *data)
{
	const struct iwx_tlv_calib_data *def_calib = data;
	uint32_t ucode_type = le32toh(def_calib->ucode_type);

	if (ucode_type >= IWX_UCODE_TYPE_MAX) {
		device_printf(sc->sc_dev,
		    "Wrong ucode_type %u for default "
		    "calibration.\n", ucode_type);
		return EINVAL;
	}

	sc->sc_default_calib[ucode_type].flow_trigger =
	    def_calib->calib.flow_trigger;
	sc->sc_default_calib[ucode_type].event_trigger =
	    def_calib->calib.event_trigger;

	return 0;
}

static int
iwx_set_ucode_api_flags(struct iwx_softc *sc, const uint8_t *data,
		struct iwx_ucode_capabilities *capa)
{
	const struct iwx_ucode_api *ucode_api = (const void *)data;
	uint32_t api_index = le32toh(ucode_api->api_index);
	uint32_t api_flags = le32toh(ucode_api->api_flags);
	int i;

	if (api_index >= howmany(IWX_NUM_UCODE_TLV_API, 32)) {
		device_printf(sc->sc_dev,
		    "api flags index %d larger than supported by driver\n",
		    api_index);
		/* don't return an error so we can load FW that has more bits */
		return 0;
	}

	for (i = 0; i < 32; i++) {
		if (api_flags & (1U << i))
			setbit(capa->enabled_api, i + 32 * api_index);
	}

	return 0;
}

static int
iwx_set_ucode_capabilities(struct iwx_softc *sc, const uint8_t *data,
			   struct iwx_ucode_capabilities *capa)
{
	const struct iwx_ucode_capa *ucode_capa = (const void *)data;
	uint32_t api_index = le32toh(ucode_capa->api_index);
	uint32_t api_flags = le32toh(ucode_capa->api_capa);
	int i;

	if (api_index >= howmany(IWX_NUM_UCODE_TLV_CAPA, 32)) {
		device_printf(sc->sc_dev,
		    "capa flags index %d larger than supported by driver\n",
		    api_index);
		/* don't return an error so we can load FW that has more bits */
		return 0;
	}

	for (i = 0; i < 32; i++) {
		if (api_flags & (1U << i))
			setbit(capa->enabled_capa, i + 32 * api_index);
	}

	return 0;
}

static void
iwx_fw_info_free(struct iwx_fw_info *fw)
{
	firmware_put(fw->fw_fp, FIRMWARE_UNLOAD);
	fw->fw_fp = NULL;
	memset(fw->img, 0, sizeof(fw->img));
}

#define IWX_FW_ADDR_CACHE_CONTROL 0xC0000000

static int
iwx_read_firmware(struct iwx_softc *sc)
{
	struct iwx_fw_info *fw = &sc->sc_fw;
	const struct iwx_tlv_ucode_header *uhdr;
	const struct iwx_ucode_tlv *tlv;
	struct iwx_ucode_capabilities *capa = &sc->sc_fw.ucode_capa;
	enum iwx_ucode_tlv_type tlv_type;
	const struct firmware *fwp;
	const uint8_t *data;
	uint32_t tlv_len;
//	uint32_t usniffer_img;
	const uint8_t *tlv_data;
//	uint32_t paging_mem_size;
	int num_of_cpus;
	int error = 0;
	size_t len;

	/*
	 * Load firmware into driver memory.
	 * fw_fp will be set.
	 */
	fwp = firmware_get(sc->cfg->fw_name);

	if (fwp == NULL) {
		device_printf(sc->sc_dev,
		    "could not read firmware %s (error %d)\n",
		    sc->cfg->fw_name, error);
		goto out;
	}
	fw->fw_fp = fwp;

	/* (Re-)Initialize default values. */
	capa->flags = 0;
	capa->max_probe_length = IWX_DEFAULT_MAX_PROBE_LENGTH;
	capa->n_scan_channels = IWX_DEFAULT_SCAN_CHANNELS;
	memset(capa->enabled_capa, 0, sizeof(capa->enabled_capa));
	memset(capa->enabled_api, 0, sizeof(capa->enabled_api));
//	memset(sc->sc_fw_mcc, 0, sizeof(sc->sc_fw_mcc));

	/*
	 * Parse firmware contents
	 */

	uhdr = (const void *)fw->fw_fp->data;

	if (*(const uint32_t *)fw->fw_fp->data != 0
	    || le32toh(uhdr->magic) != IWX_TLV_UCODE_MAGIC) {
		device_printf(sc->sc_dev, "invalid firmware %s\n",
		    sc->cfg->fw_name);
		error = EINVAL;
		goto out;
	}

	snprintf(sc->sc_fwver, sizeof(sc->sc_fwver), "%u.%u (API ver %u)",
	    IWX_UCODE_MAJOR(le32toh(uhdr->ver)),
	    IWX_UCODE_MINOR(le32toh(uhdr->ver)),
	    IWX_UCODE_API(le32toh(uhdr->ver)));
	data = uhdr->data;
	len = fw->fw_fp->datasize - sizeof(*uhdr);

	while (len >= sizeof(*tlv)) {
		len -= sizeof(*tlv);
		tlv = (const void *)data;

		tlv_len = le32toh(tlv->length);
		tlv_type = le32toh(tlv->type);
		tlv_data = tlv->data;

//		if (len < tlv_len) {
		if (len < roundup2(tlv_len, 4)) {
			device_printf(sc->sc_dev,
			    "firmware too short: %zu bytes\n",
			    len);
			error = EINVAL;
			goto parse_out;
		}
		len -= roundup2(tlv_len, 4);
		data += sizeof(*tlv) + roundup2(tlv_len, 4);

#if 0
		device_printf(sc->sc_dev, "%s: %s %#0jx %#0jx %#0jx %#0zx %p %p %#0x %#0x %p\n",
				__func__,
				sc->sc_fwver,
				(uintmax_t)(const void *)fwp->data,
				((uintmax_t)(const void *)fwp->data + fwp->datasize),
				(uintmax_t)fwp->datasize,
				len, data, tlv, tlv_len, tlv_type, tlv_data); /* todo if_iwx: remove before submitting for review */
#endif
		switch ((int)tlv_type) {
		case IWX_UCODE_TLV_PROBE_MAX_LEN:
			if (tlv_len != sizeof(uint32_t)) {
				device_printf(sc->sc_dev,
				    "%s: PROBE_MAX_LEN (%u) != sizeof(uint32_t)\n",
				    __func__, tlv_len);

				error = EINVAL;
				goto parse_out;
			}
			capa->max_probe_length =
			    le32_to_cpup((const uint32_t *)tlv_data);
			/* limit it to something sensible */
			if (capa->max_probe_length >
			    IWX_SCAN_OFFLOAD_PROBE_REQ_SIZE) {
				IWX_DPRINTF(sc, IWX_DEBUG_FIRMWARE_TLV,
				    "%s: IWX_UCODE_TLV_PROBE_MAX_LEN "
				    "ridiculous\n", __func__);

				error = EINVAL;
				goto parse_out;
			}
			break;
		case IWX_UCODE_TLV_PAN:
			if (tlv_len) {
				device_printf(sc->sc_dev,
				    "%s: IWX_UCODE_TLV_PAN: tlv_len (%u) > 0\n",
				    __func__, tlv_len);

				error = EINVAL;
				goto parse_out;
			}
			capa->flags |= IWX_UCODE_TLV_FLAGS_PAN;
			break;
		case IWX_UCODE_TLV_FLAGS:
			if (tlv_len < sizeof(uint32_t)) {
				device_printf(sc->sc_dev,
				    "%s: IWX_UCODE_TLV_FLAGS: tlv_len (%u) < sizeof(uint32_t)\n",
				    __func__, tlv_len);

				error = EINVAL;
				goto parse_out;
			}
			if (tlv_len % sizeof(uint32_t)) {
				device_printf(sc->sc_dev,
				    "%s: IWX_UCODE_TLV_FLAGS: tlv_len (%u) %% sizeof(uint32_t)\n",
				    __func__, tlv_len);

				error = EINVAL;
				goto parse_out;
			}
			/*
			 * Apparently there can be many flags, but Linux driver
			 * parses only the first one, and so do we.
			 *
			 * XXX: why does this override IWM_UCODE_TLV_PAN?
			 * Intentional or a bug?  Observations from
			 * current firmware file:
			 *  1) TLV_PAN is parsed first
			 *  2) TLV_FLAGS contains TLV_FLAGS_PAN
			 * ==> this resets TLV_PAN to itself... hnnnk
			 */
			capa->flags = le32_to_cpup((const uint32_t *)tlv_data);
			break;
		case IWX_UCODE_TLV_CSCHEME:
			if ((error = iwx_store_cscheme(sc,
			    tlv_data, tlv_len)) != 0) {
				device_printf(sc->sc_dev,
				    "%s: iwx_store_cscheme(): returned %d\n",
				    __func__, error);

				goto parse_out;
			}
			break;
		case IWX_UCODE_TLV_NUM_OF_CPU:
			if (tlv_len != sizeof(uint32_t)) {
				device_printf(sc->sc_dev,
				    "%s: IWX_UCODE_TLV_NUM_OF_CPU: tlv_len (%u) != sizeof(uint32_t)\n",
				    __func__, tlv_len);

				error = EINVAL;
				goto parse_out;
			}
			num_of_cpus = le32_to_cpup((const uint32_t *)tlv_data);
			if (num_of_cpus == 2) {
				fw->img[IWX_UCODE_REGULAR].is_dual_cpus =
					TRUE;
				fw->img[IWX_UCODE_INIT].is_dual_cpus =
					TRUE;
				fw->img[IWX_UCODE_WOWLAN].is_dual_cpus =
					TRUE;
			} else if ((num_of_cpus > 2) || (num_of_cpus < 1)) {
				device_printf(sc->sc_dev,
				    "%s: Driver supports only 1 or 2 CPUs\n",
				    __func__);
				error = EINVAL;
				goto parse_out;
			}
			break;
		case IWX_UCODE_TLV_SEC_RT:
			if ((error = iwx_firmware_store_section(sc,
			    IWX_UCODE_REGULAR, tlv_data, tlv_len)) != 0) {
				device_printf(sc->sc_dev,
				    "%s: IWX_UCODE_REGULAR: iwx_firmware_store_section() failed; %d\n",
				    __func__, error);

				goto parse_out;
			}
			break;
		case IWX_UCODE_TLV_SEC_INIT:
			if ((error = iwx_firmware_store_section(sc,
			    IWX_UCODE_INIT, tlv_data, tlv_len)) != 0) {
				device_printf(sc->sc_dev,
				    "%s: IWX_UCODE_INIT: iwx_firmware_store_section() failed; %d\n",
				    __func__, error);

				goto parse_out;
			}
			break;
		case IWX_UCODE_TLV_SEC_WOWLAN:
			if ((error = iwx_firmware_store_section(sc,
			    IWX_UCODE_WOWLAN, tlv_data, tlv_len)) != 0) {
				device_printf(sc->sc_dev,
				    "%s: IWX_UCODE_WOWLAN: iwx_firmware_store_section() failed; %d\n",
				    __func__, error);/* todo if_iwx: remove before submitting for review */

				goto parse_out;
			}
			break;
		case IWX_UCODE_TLV_DEF_CALIB:
			if (tlv_len != sizeof(struct iwx_tlv_calib_data)) {
				device_printf(sc->sc_dev,
				    "%s: IWX_UCODE_TLV_DEV_CALIB: tlv_len (%u) < sizeof(iwm_tlv_calib_data) (%zu)\n",
				    __func__, tlv_len,
				    sizeof(struct iwx_tlv_calib_data));

				error = EINVAL;
				goto parse_out;
			}
			if ((error = iwx_set_default_calib(sc, tlv_data)) != 0) {
				device_printf(sc->sc_dev,
				    "%s: iwx_set_default_calib() failed: %d\n",
				    __func__, error);

				goto parse_out;
			}
			break;
		case IWX_UCODE_TLV_PHY_SKU:
			if (tlv_len != sizeof(uint32_t)) {
				error = EINVAL;
				device_printf(sc->sc_dev,
				    "%s: IWX_UCODE_TLV_PHY_SKU: tlv_len (%u) < sizeof(uint32_t)\n",
				    __func__, tlv_len);

				goto parse_out;
			}
			sc->sc_fw.phy_config =
			    le32_to_cpup((const uint32_t *)tlv_data);
			sc->sc_fw.valid_tx_ant = (sc->sc_fw.phy_config &
						  IWX_FW_PHY_CFG_TX_CHAIN) >>
						  IWX_FW_PHY_CFG_TX_CHAIN_POS;
			sc->sc_fw.valid_rx_ant = (sc->sc_fw.phy_config &
						  IWX_FW_PHY_CFG_RX_CHAIN) >>
						  IWX_FW_PHY_CFG_RX_CHAIN_POS;
			break;

		case IWX_UCODE_TLV_API_CHANGES_SET: {
			if (tlv_len != sizeof(struct iwx_ucode_api)) {
				error = EINVAL;
				goto parse_out;
			}
			if (iwx_set_ucode_api_flags(sc, tlv_data, capa)) {
				error = EINVAL;
				goto parse_out;
			}
			break;
		}

		case IWX_UCODE_TLV_ENABLED_CAPABILITIES: {
			if (tlv_len != sizeof(struct iwx_ucode_capa)) {
				error = EINVAL;
				goto parse_out;
			}
			if (iwx_set_ucode_capabilities(sc, tlv_data, capa)) {
				error = EINVAL;
				goto parse_out;
			}
			break;
		}

		case IWX_UCODE_TLV_SDIO_ADMA_ADDR:
		case IWX_UCODE_TLV_FW_GSCAN_CAPA:
			/* ignore, not used by current driver */
			break;

		case IWX_UCODE_TLV_SEC_RT_USNIFFER:
			if ((error = iwx_firmware_store_section(sc,
			    IWX_UCODE_REGULAR_USNIFFER, tlv_data,
			    tlv_len)) != 0)
				goto parse_out;
			break;

		case IWX_UCODE_TLV_PAGING:
			if (tlv_len != sizeof(uint32_t)) {
				error = EINVAL;
				goto parse_out;
			}
			break;

		case IWX_UCODE_TLV_N_SCAN_CHANNELS:
			if (tlv_len != sizeof(uint32_t)) {
				error = EINVAL;
				goto parse_out;
			}
			capa->n_scan_channels =
			    le32_to_cpup((const uint32_t *)tlv_data);
			break;

		case IWX_UCODE_TLV_FW_VERSION:
			if (tlv_len != sizeof(uint32_t) * 3) {
				error = EINVAL;
				goto parse_out;
			}
			snprintf(sc->sc_fwver, sizeof(sc->sc_fwver),
			    "%u.%u.%u",
			    le32toh(((const uint32_t *)tlv_data)[0]),
			    le32toh(((const uint32_t *)tlv_data)[1]),
			    le32toh(((const uint32_t *)tlv_data)[2]));
			break;

		case IWX_UCODE_TLV_FW_DBG_DEST: {
#if 0
			struct iwx_fw_dbg_dest_tlv_v1 *dest_v1 = NULL;

			fw->dbg.dbg_dest_ver = (uint8_t *)tlv_data;
			if (*fw->dbg.dbg_dest_ver != 0) {
				error = EINVAL;
				goto parse_out;
			}

			if (fw->dbg.dbg_dest_tlv_init)
				break;
			fw->dbg.dbg_dest_tlv_init = true;

			dest_v1 = (void *)tlv_data;
			fw->dbg.dbg_dest_tlv_v1 = dest_v1;
			fw->dbg.n_dest_reg = tlv_len -
			    offsetof(struct iwx_fw_dbg_dest_tlv_v1, reg_ops);
			fw->dbg.n_dest_reg /= sizeof(dest_v1->reg_ops[0]);
			IWX_DPRINTF(sc, IWX_DEBUG_XMIT, "%s: found debug dest; n_dest_reg=%d\n", __func__, fw->dbg.n_dest_reg);
#endif
			break;
		}

		case IWX_UCODE_TLV_FW_DBG_CONF: {
#if 0
			struct iwx_fw_dbg_conf_tlv *conf = (void *)tlv_data;

			if (!fw->dbg.dbg_dest_tlv_init ||
			    conf->id >= nitems(fw->dbg.dbg_conf_tlv) ||
			    fw->dbg_conf_tlv[conf->id] != NULL)
				break;

			IWX_DPRINTF(sc, IWX_DEBUG_TRACE, "Found debug configuration: %d\n", conf->id);
			fw->dbg_conf_tlv[conf->id] = conf;
			fw->dbg_conf_tlv_len[conf->id] = tlv_len;
#endif
			break;
		}

		case IWX_UCODE_TLV_UMAC_DEBUG_ADDRS: {
#if 0
			struct iwx_umac_debug_addrs *dbg_ptrs =
				(void *)tlv_data;

			if (tlv_len != sizeof(*dbg_ptrs)) {
				err = EINVAL;
				goto parse_out;
			}
			sc->uc_umac_error_event_table =
				le32toh(dbg_ptrs->error_info_addr) &
				~IWX_FW_ADDR_CACHE_CONTROL;
			sc->error_event_table_tlv_status |=
				IWX_ERROR_EVENT_TABLE_UMAC;
#endif
			break;
		}

		case IWX_UCODE_TLV_LMAC_DEBUG_ADDRS: {
#if 0
			struct iwx_lmac_debug_addrs *dbg_ptrs =
				(void *)tlv_data;

			if (tlv_len != sizeof(*dbg_ptrs)) {
				err = EINVAL;
				goto parse_out;
			}
			if (sc->sc_device_family < IWX_DEVICE_FAMILY_22000)
				break;
			sc->sc_uc.uc_lmac_error_event_table[0] =
				le32toh(dbg_ptrs->error_event_table_ptr) &
				~IWX_FW_ADDR_CACHE_CONTROL;
			sc->sc_uc.error_event_table_tlv_status |=
				IWX_ERROR_EVENT_TABLE_LMAC1;
#endif
			break;
		}

		case IWX_UCODE_TLV_FW_MEM_SEG:
			break;

		case IWX_UCODE_TLV_CMD_VERSIONS:
		if (tlv_len % sizeof(struct iwx_fw_cmd_version)) {
				tlv_len /= sizeof(struct iwx_fw_cmd_version);
				tlv_len *= sizeof(struct iwx_fw_cmd_version);
			}
			if (sc->n_cmd_versions != 0) {
				error = EINVAL;
				goto parse_out;
			}
			if (tlv_len > sizeof(sc->cmd_versions)) {
				error = EINVAL;
				goto parse_out;
			}
			memcpy(&sc->cmd_versions[0], tlv_data, tlv_len);
			sc->n_cmd_versions = tlv_len / sizeof(struct iwx_fw_cmd_version);
			break;

		case IWX_UCODE_TLV_FW_RECOVERY_INFO:
			break;

		/* undocumented TLVs found in ax200-cc-a0-46 image */
		case 58:
		case 0x1000003:
		case 0x1000004:
			break;
		default:
			device_printf(sc->sc_dev,
			    "%s: unknown firmware section %d, abort\n",
			    __func__, tlv_type);
			error = EINVAL;
			goto parse_out;
		}
	}

	KASSERT(error == 0, ("unhandled error"));

 parse_out:
	if (error) {
		device_printf(sc->sc_dev, "firmware parse error %d, "
		    "section type %d\n", error, tlv_type);
	}

 out:
	if (error) {
		if (fw->fw_fp != NULL)
			iwx_fw_info_free(fw);
	}

	return error;
}

/*
 * DMA resource routines
 */

/* fwmem is used to load firmware onto the card */
/* assume iwx_alloc_fwmem() is identical to iwx_ctxt_info_alloc_dma() */
static int
iwx_alloc_fwmem(struct iwx_softc *sc)
{
	/* Must be aligned on a 16-byte boundary. */
	return iwx_dma_contig_alloc(sc->sc_dmat, &sc->fw_dma,
	    IWX_FH_MEM_TB_MAX_LENGTH, 16);
}

/* tx scheduler rings.  not used? */
static int
iwx_alloc_sched(struct iwx_softc *sc)
{
	/* TX scheduler rings must be aligned on a 1KB boundary. */
	return iwx_dma_contig_alloc(sc->sc_dmat, &sc->sched_dma,
	    nitems(sc->txq) * sizeof(struct iwx_agn_scd_bc_tbl), 1024);
}

/* interrupt cause table */
static int
iwx_alloc_ict(struct iwx_softc *sc)
{
	return iwx_dma_contig_alloc(sc->sc_dmat, &sc->ict_dma,
	    IWX_ICT_SIZE, 1<<IWX_ICT_PADDR_SHIFT);
}

static int
iwx_alloc_rx_ring(struct iwx_softc *sc, struct iwx_rx_ring *ring)
{
	bus_size_t size;
	int i, error;

	ring->cur = 0;

	/* Allocate RX descriptors (256-byte aligned). */
	size = IWX_RX_MQ_RING_COUNT * sizeof(uint64_t);
	error = iwx_dma_contig_alloc(sc->sc_dmat, &ring->free_desc_dma, size,
	    256);
//	device_printf(sc->sc_dev, "%s: AYBABTU size=%zu error=%d\n", __func__, size, error); /* todo if_iwx: remove before submitting for review */
	if (error != 0) {
		device_printf(sc->sc_dev,
		    "could not allocate RX ring DMA memory\n");
		goto fail;
	}
	ring->desc = ring->free_desc_dma.vaddr;

	/* Allocate RX status area (16-byte aligned). */
	error = iwx_dma_contig_alloc(sc->sc_dmat, &ring->stat_dma,
	    sizeof(*ring->stat), 16);
//	device_printf(sc->sc_dev, "%s: AYBABTU size=%zu error=%d\n", __func__, size, error); /* todo if_iwx: remove before submitting for review */
	if (error != 0) {
		device_printf(sc->sc_dev,
		    "could not allocate RX status DMA memory\n");
		goto fail;
	}
	ring->stat = ring->stat_dma.vaddr;

	size = IWX_RX_MQ_RING_COUNT * sizeof(uint32_t);
	error = iwx_dma_contig_alloc(sc->sc_dmat, &ring->used_desc_dma,
	    size, 256);
//	device_printf(sc->sc_dev, "%s: AYBABTU size=%zu error=%d\n", __func__, size, error); /* todo if_iwx: remove before submitting for review */
	if (error) {
		device_printf(sc->sc_dev, "could not allocate RX ring DMA memory\n");
		goto fail;
	}

        /* Create RX buffer DMA tag. */
        error = bus_dma_tag_create(sc->sc_dmat, 1, 0,
            BUS_SPACE_MAXADDR_32BIT, BUS_SPACE_MAXADDR, NULL, NULL,
            IWX_RBUF_SIZE, 1, IWX_RBUF_SIZE, 0, NULL, NULL, &ring->data_dmat);
        if (error != 0) {
                device_printf(sc->sc_dev,
                    "%s: could not create RX buf DMA tag, error %d\n",
                    __func__, error);
                goto fail;
        }

	/* Allocate spare bus_dmamap_t for iwm_rx_addbuf() */
	error = bus_dmamap_create(ring->data_dmat, 0, &ring->spare_map);
	if (error != 0) {
		device_printf(sc->sc_dev,
		    "%s: could not create RX buf DMA map, error %d\n",
		    __func__, error);
		goto fail;
	}

	/*
	 * Allocate and map RX buffers.
	 */
	for (i = 0; i < IWX_RX_MQ_RING_COUNT; i++) {
		struct iwx_rx_data *data = &ring->data[i];
		error = bus_dmamap_create(ring->data_dmat, 0, &data->map);
		if (error != 0) {
			device_printf(sc->sc_dev,
			    "%s: could not create RX buf DMA map, error %d\n",
			    __func__, error);
			goto fail;
		}
		data->m = NULL;

		if ((error = iwx_rx_addbuf(sc, IWX_RBUF_SIZE, i)) != 0) {
			goto fail;
		}
	}
	return 0;

fail:	iwx_free_rx_ring(sc, ring);
	return error;
}

static void
iwx_reset_rx_ring(struct iwx_softc *sc, struct iwx_rx_ring *ring)
{
	/* Reset the ring state */
	ring->cur = 0;

	/*
	 * The hw rx ring index in shared memory must also be cleared,
	 * otherwise the discrepancy can cause reprocessing chaos.
	 */
	if (sc->rxq.stat)
		memset(sc->rxq.stat, 0, sizeof(*sc->rxq.stat));
}

static void
iwx_free_rx_ring(struct iwx_softc *sc, struct iwx_rx_ring *ring)
{
	int i;

	iwx_dma_contig_free(&ring->free_desc_dma);
	iwx_dma_contig_free(&ring->stat_dma);
	iwx_dma_contig_free(&ring->used_desc_dma);

	for (i = 0; i < IWX_RX_MQ_RING_COUNT; i++) {
		struct iwx_rx_data *data = &ring->data[i];

		if (data->m != NULL) {
			bus_dmamap_sync(ring->data_dmat, data->map,
			    BUS_DMASYNC_POSTREAD);
			bus_dmamap_unload(ring->data_dmat, data->map);
			m_freem(data->m);
			data->m = NULL;
		}
		if (data->map != NULL) {
			bus_dmamap_destroy(ring->data_dmat, data->map);
			data->map = NULL;
		}
	}
	if (ring->spare_map != NULL) {
		bus_dmamap_destroy(ring->data_dmat, ring->spare_map);
		ring->spare_map = NULL;
	}
	if (ring->data_dmat != NULL) {
		bus_dma_tag_destroy(ring->data_dmat);
		ring->data_dmat = NULL;
	}
}

static int
iwx_alloc_tx_ring(struct iwx_softc *sc, struct iwx_tx_ring *ring, int qid)
{
	bus_addr_t paddr;
	bus_size_t size;
	size_t maxsize;
	int nsegments;
	int i, error, qlen;

	ring->qid = qid;
	ring->queued = 0;
	ring->cur = 0;
//	ring->tail = 0;

	if (qid == IWX_DQA_CMD_QUEUE)
		qlen = IWX_CMD_QUEUE_SIZE;
	else
		qlen = IWX_TX_RING_COUNT;

	/* Allocate TX descriptors (256-byte aligned). */
	size = qlen * sizeof (struct iwx_tfh_tfd);
	error = iwx_dma_contig_alloc(sc->sc_dmat, &ring->desc_dma, size, 256);
//	device_printf(sc->sc_dev, "%s: AYBABTU size=%zu error=%d\n", __func__, size, error); /* todo if_iwx: remove before submitting for review */
	if (error != 0) {
		device_printf(sc->sc_dev,
		    "could not allocate TX ring DMA memory\n");
		goto fail;
	}
	ring->desc = ring->desc_dma.vaddr;

	/*
	 * We only use rings 0 through 9 (4 EDCA + cmd) so there is no need
	 * to allocate commands space for other rings.
	 */
	if (qid > IWX_DQA_MAX_MGMT_QUEUE)
		return 0;

	error = iwx_dma_contig_alloc(sc->sc_dmat, &ring->bc_tbl,
	    sizeof(struct iwx_agn_scd_bc_tbl), 1);
//	device_printf(sc->sc_dev, "%s: AYBABTU size=%zu error=%d\n", __func__, size, error); /* todo if_iwx: remove before submitting for review */
	if (error != 0) {
		device_printf(sc->sc_dev,
		    "could not allocate byte count table DMA memory\n");
		goto fail;
	}

	size = qlen * sizeof(struct iwx_device_cmd);
	error = iwx_dma_contig_alloc(sc->sc_dmat, &ring->cmd_dma, size,
	    IWX_FIRST_TB_SIZE_ALIGN);
//	device_printf(sc->sc_dev, "%s: AYBABTU size=%zu error=%d\n", __func__, size, error); /* todo if_iwx: remove before submitting for review */
	if (error != 0) {
		device_printf(sc->sc_dev, "could not allocate cmd DMA memory\n");
		goto fail;
	}
	ring->cmd = ring->cmd_dma.vaddr;

	/* FW commands may require more mapped space than packets. */
	if (qid == IWX_DQA_CMD_QUEUE) {
		maxsize = IWX_RBUF_SIZE;
		nsegments = 1;
	} else {
		maxsize = MCLBYTES;
		nsegments = IWX_MAX_SCATTER - 2;
	}

	error = bus_dma_tag_create(sc->sc_dmat, 1, 0,
	    BUS_SPACE_MAXADDR_32BIT, BUS_SPACE_MAXADDR, NULL, NULL, maxsize,
            nsegments, maxsize, 0, NULL, NULL, &ring->data_dmat);
	if (error != 0) {
		device_printf(sc->sc_dev, "could not create TX buf DMA tag\n");
		goto fail;
	}

	paddr = ring->cmd_dma.paddr;
	for (i = 0; i < qlen; i++) {
		struct iwx_tx_data *data = &ring->data[i];

		data->cmd_paddr = paddr;
		paddr += sizeof(struct iwx_device_cmd);

		error = bus_dmamap_create(ring->data_dmat, 0, &data->map);
		if (error != 0) {
			device_printf(sc->sc_dev,
			    "could not create TX buf DMA map\n");
			goto fail;
		}
	}
	KASSERT(paddr == ring->cmd_dma.paddr + size,
	    ("invalid physical address"));
	return 0;

fail:	iwx_free_tx_ring(sc, ring);
	return error;
}

static void
iwx_reset_tx_ring(struct iwx_softc *sc, struct iwx_tx_ring *ring)
{
	int i, qlen;

	if (ring->qid == IWX_DQA_CMD_QUEUE)
		qlen = IWX_CMD_QUEUE_SIZE;
	else
		qlen = IWX_TX_RING_COUNT;

	for (i = 0; i < qlen; i++) {
		struct iwx_tx_data *data = &ring->data[i];

		if (data->m != NULL) {
			bus_dmamap_sync(ring->data_dmat, data->map,
			    BUS_DMASYNC_POSTWRITE);
			bus_dmamap_unload(ring->data_dmat, data->map);
			m_freem(data->m);
			data->m = NULL;
		}
	}
	/* Clear TX descriptors. */
	memset(ring->desc, 0, ring->desc_dma.size);
	bus_dmamap_sync(ring->desc_dma.tag, ring->desc_dma.map,
	    BUS_DMASYNC_PREWRITE);
	sc->qfullmsk &= ~(1 << ring->qid);
	ring->queued = 0;
	ring->cur = 0;
//	ring->tali = 0;

//	if (ring->qid == IWX_DQA_CMD_QUEUE && sc->cmd_hold_nic_awake)
	if (ring->qid == IWX_CMD_QUEUE && sc->cmd_hold_nic_awake)
		iwx_pcie_clear_cmd_in_flight(sc);
}

static void
iwx_free_tx_ring(struct iwx_softc *sc, struct iwx_tx_ring *ring)
{
	int i, qlen;

	iwx_dma_contig_free(&ring->desc_dma);
	iwx_dma_contig_free(&ring->cmd_dma);
	iwx_dma_contig_free(&ring->bc_tbl);

	if (ring->qid == IWX_DQA_CMD_QUEUE)
		qlen = IWX_CMD_QUEUE_SIZE;
	else
		qlen = IWX_TX_RING_COUNT;

	for (i = 0; i < qlen; i++) {
		struct iwx_tx_data *data = &ring->data[i];

		if (data->m != NULL) {
			bus_dmamap_sync(ring->data_dmat, data->map,
			    BUS_DMASYNC_POSTWRITE);
			bus_dmamap_unload(ring->data_dmat, data->map);
			m_freem(data->m);
			data->m = NULL;
		}
		if (data->map != NULL) {
			bus_dmamap_destroy(ring->data_dmat, data->map);
			data->map = NULL;
		}
	}
	if (ring->data_dmat != NULL) {
		bus_dma_tag_destroy(ring->data_dmat);
		ring->data_dmat = NULL;
	}
}

/*
 * High-level hardware frobbing routines
 */

static void
iwx_enable_interrupts(struct iwx_softc *sc)
{
	sc->sc_intmask = IWX_CSR_INI_SET_MASK;
	IWX_WRITE(sc, IWX_CSR_INT_MASK, sc->sc_intmask);
}

static void
iwx_restore_interrupts(struct iwx_softc *sc)
{
	IWX_WRITE(sc, IWX_CSR_INT_MASK, sc->sc_intmask);
}

static void
iwx_disable_interrupts(struct iwx_softc *sc)
{
	/* disable interrupts */
	IWX_WRITE(sc, IWX_CSR_INT_MASK, 0);

	/* acknowledge all interrupts */
	IWX_WRITE(sc, IWX_CSR_INT, ~0);
	IWX_WRITE(sc, IWX_CSR_FH_INT_STATUS, ~0);
}

static void
iwx_ict_reset(struct iwx_softc *sc)
{
	iwx_disable_interrupts(sc);

	/* Reset ICT table. */
	memset(sc->ict_dma.vaddr, 0, IWX_ICT_SIZE);
	sc->ict_cur = 0;

	/* Set physical address of ICT table (4KB aligned). */
	IWX_WRITE(sc, IWX_CSR_DRAM_INT_TBL_REG,
	    IWX_CSR_DRAM_INT_TBL_ENABLE
	    | IWX_CSR_DRAM_INIT_TBL_WRITE_POINTER
	    | IWX_CSR_DRAM_INIT_TBL_WRAP_CHECK
	    | sc->ict_dma.paddr >> IWX_ICT_PADDR_SHIFT);

	/* Switch to ICT interrupt mode in driver. */
	sc->sc_flags |= IWX_FLAG_USE_ICT;

	/* Re-enable interrupts. */
	IWX_WRITE(sc, IWX_CSR_INT, ~0);
	iwx_enable_interrupts(sc);
}

/* iwlwifi pcie/trans-gen2.c */

/*
 * Since this .. hard-resets things, it's time to actually
 * mark the first vap (if any) as having no mac context.
 * It's annoying, but since the driver is potentially being
 * stop/start'ed whilst active (thanks openbsd port!) we
 * have to correctly track this.
 */
static void
iwx_stop_device(struct iwx_softc *sc)
{
	struct ieee80211com *ic = &sc->sc_ic;
	struct ieee80211vap *vap = TAILQ_FIRST(&ic->ic_vaps);
//	int chnl, qid;
//	uint32_t mask = 0;
	int qid;

	/* tell the device to stop sending interrupts */
	iwx_disable_interrupts(sc);

	/*
	 * FreeBSD-local: mark the first vap as not-uploaded,
	 * so the next transition through auth/assoc
	 * will correctly populate the MAC context.
	 */
	if (vap) {
		struct iwx_vap *iv = IWX_VAP(vap);
		iv->phy_ctxt = NULL;
		iv->is_uploaded = 0;
	}
	sc->sc_firmware_state = 0;
	sc->sc_flags &= ~IWX_FLAG_TE_ACTIVE;

	/* device going down, Stop using ICT table */
	sc->sc_flags &= ~IWX_FLAG_USE_ICT;

	// iwx_disable_rx_dma() in OpenBSD?
//	iwx_pcie_rx_stop(sc);

	/* Stop RX ring. */
	iwx_reset_rx_ring(sc, &sc->rxq);

	/* Reset all TX rings. */
	/* todo: sync with iwl_pcie_gen2_tx_stop()? */
	for (qid = 0; qid < nitems(sc->txq); qid++)
		iwx_reset_tx_ring(sc, &sc->txq[qid]);

	iwx_ctxt_info_free_paging(sc);

	/* todo: add support for AX210,
	 * see _iwl_trans_pcie_gen2_stop_device() */
	iwx_ctxt_info_free(sc);

	/* Make sure (redundant) we've released our request to stay awake */
	IWX_CLRBITS(sc, IWX_CSR_GP_CNTRL,
	    IWX_CSR_GP_CNTRL_REG_FLAG_MAC_ACCESS_REQ);

	/* Stop the device, and put it in low power state */
	iwx_apm_stop(sc);

	/* stop and reset the on-board processor */
	IWX_SETBITS(sc, IWX_CSR_RESET, IWX_CSR_RESET_REG_FLAG_SW_RESET);
	DELAY(5000);

	/*
	 * Upon stop, the APM issues an interrupt if HW RF kill is set.
	 */
	iwx_disable_interrupts(sc);

	/*
	 * Even if we stop the HW, we still want the RF kill
	 * interrupt
	 */
	iwx_enable_rfkill_int(sc);
	iwx_check_rfkill(sc);

	iwx_prepare_card_hw(sc);
}

/* iwlwifi: mvm/ops.c */
static void
iwx_nic_config(struct iwx_softc *sc)
{
	uint8_t radio_cfg_type, radio_cfg_step, radio_cfg_dash;
	uint32_t reg_val = 0;
	uint32_t phy_config = iwx_get_phy_config(sc);

	radio_cfg_type = (phy_config & IWX_FW_PHY_CFG_RADIO_TYPE) >>
	    IWX_FW_PHY_CFG_RADIO_TYPE_POS;
	radio_cfg_step = (phy_config & IWX_FW_PHY_CFG_RADIO_STEP) >>
	    IWX_FW_PHY_CFG_RADIO_STEP_POS;
	radio_cfg_dash = (phy_config & IWX_FW_PHY_CFG_RADIO_DASH) >>
	    IWX_FW_PHY_CFG_RADIO_DASH_POS;

	/* SKU control */
	reg_val |= IWX_CSR_HW_REV_STEP(sc->sc_hw_rev) <<
	    IWX_CSR_HW_IF_CONFIG_REG_POS_MAC_STEP;
	reg_val |= IWX_CSR_HW_REV_DASH(sc->sc_hw_rev) <<
	    IWX_CSR_HW_IF_CONFIG_REG_POS_MAC_DASH;

	/* radio configuration */
	reg_val |= radio_cfg_type << IWM_CSR_HW_IF_CONFIG_REG_POS_PHY_TYPE;
	reg_val |= radio_cfg_step << IWM_CSR_HW_IF_CONFIG_REG_POS_PHY_STEP;
	reg_val |= radio_cfg_dash << IWM_CSR_HW_IF_CONFIG_REG_POS_PHY_DASH;

	IWX_WRITE(sc, IWX_CSR_HW_IF_CONFIG_REG,
			IWX_CSR_HW_IF_CONFIG_REG_MSK_MAC_DASH |
			IWX_CSR_HW_IF_CONFIG_REG_MSK_MAC_STEP |
			IWX_CSR_HW_IF_CONFIG_REG_MSK_PHY_STEP |
			IWX_CSR_HW_IF_CONFIG_REG_MSK_PHY_DASH |
			IWX_CSR_HW_IF_CONFIG_REG_MSK_PHY_TYPE |
			IWX_CSR_HW_IF_CONFIG_REG_BIT_RADIO_SI |
			IWX_CSR_HW_IF_CONFIG_REG_BIT_MAC_SI |
	    reg_val);

	IWX_DPRINTF(sc, IWX_DEBUG_RESET,
	    "Radio type=0x%x-0x%x-0x%x\n", radio_cfg_type,
	    radio_cfg_step, radio_cfg_dash);
}

int
iwx_nic_rx_init(struct iwx_softc *sc)
{
	IWX_WRITE_1(sc, IWX_CSR_INT_COALESCING, IWX_HOST_INT_TIMEOUT_DEF);

	/*
	 * We don't configure the RFH; the firmware will do that.
	 * Rx descriptors are set when firmware sends an ALIVE interrupt.
	 */
	return 0;
}

/* pcie/tx-gen2.c */
static int
iwx_nic_tx_init(struct iwx_softc *sc, int txq_id, int queue_size)
{
//	struct iwx_txq *txq;
//	int ret;

	/* todo: this function could be lacking features */

	return 0;
}

static int
iwx_nic_init(struct iwx_softc *sc)
{
	int error;

//	int queue_size = max_t(u32, IWX_CMD_QUEUE_SIZE,
//			sc->cfg->min_txq_size);

	int queue_size = IWX_CMD_QUEUE_SIZE;

	iwx_apm_init(sc);

	/* not used in pcie/trans-gen2.c */
//	iwx_set_pwr(sc);

	iwx_nic_config(sc);

	if ((error = iwx_nic_rx_init(sc)) != 0)
		return error;

	if ((error = iwx_nic_tx_init(sc, sc->cmd_queue, queue_size)) != 0)
		return error;

	IWX_DPRINTF(sc, IWX_DEBUG_RESET,
	    "%s: shadow registers enabled\n", __func__);
	IWX_SETBITS(sc, IWX_CSR_MAC_SHADOW_REG_CTRL, 0x800fffff);

	return 0;
}

/* todo if_iwx: port this function, instead of copying code over from if_iwm */
/* OpenBSD changed the amount and type of parameters */
int
iwx_enable_txq(struct iwx_softc *sc, int sta_id, int qid, int tid,
    int num_slots)
{
	int qmsk;

	qmsk = 1 << qid;

	if (!iwx_nic_lock(sc)) {
		device_printf(sc->sc_dev, "%s: cannot enable txq %d\n",
				__func__, qid);
		return EBUSY;
	}

	IWX_WRITE(sc, IWX_HBUS_TARG_WRPTR, qid << 8 | 0);
	
	if (qid == IWX_CMD_QUEUE) {
		/* Disable the scheduler. */
		iwx_write_prph(sc, IWX_SCD_EN_CTRL, 0);

		/* Stop the TX queue prior to configuration. */
		iwx_write_prph(sc, IWX_SCD_QUEUE_STATUS_BITS(qid),
		    (0 << IWX_SCD_QUEUE_STTS_REG_POS_ACTIVE) |
		    (1 << IWX_SCD_QUEUE_STTS_REG_POS_SCD_ACT_EN));

		iwx_nic_unlock(sc);

		/* Disable aggregations for this queue. */
		iwx_clear_bits_prph(sc, IWX_SCD_AGGR_SEL, qmsk);

		if (!iwx_nic_lock(sc)) {
			device_printf(sc->sc_dev,
			    "%s: cannot enable txq %d\n", __func__, qid);
			return EBUSY;
		}
		iwx_write_prph(sc, IWX_SCD_QUEUE_RDPTR(qid), 0);
		iwx_nic_unlock(sc);

		iwx_write_mem32(sc,
		    sc->scd_base_addr + IWX_SCD_CONTEXT_QUEUE_OFFSET(qid), 0);
		/* Set scheduler window size and frame limit. */
		iwx_write_mem32(sc,
		    sc->scd_base_addr + IWX_SCD_CONTEXT_QUEUE_OFFSET(qid) +
		    sizeof(uint32_t),
		    ((IWX_FRAME_LIMIT << IWX_SCD_QUEUE_CTX_REG2_WIN_SIZE_POS) &
		    IWX_SCD_QUEUE_CTX_REG2_WIN_SIZE_MSK) |
		    ((IWX_FRAME_LIMIT << IWX_SCD_QUEUE_CTX_REG2_FRAME_LIMIT_POS) &
		    IWX_SCD_QUEUE_CTX_REG2_FRAME_LIMIT_MSK));

		if (!iwx_nic_lock(sc)) {
			device_printf(sc->sc_dev,
			    "%s: cannot enable txq %d\n", __func__, qid);
			return EBUSY;
		}
#ifdef tbd
	/* todo: openbsd drops the 'fifo' parameter */
		iwx_write_prph(sc, IWX_SCD_QUEUE_STATUS_BITS(qid),
		    (1 << IWX_SCD_QUEUE_STTS_REG_POS_ACTIVE) |
		    (fifo << IWX_SCD_QUEUE_STTS_REG_POS_TXF) |
		    (1 << IWX_SCD_QUEUE_STTS_REG_POS_WSL) |
		    IWX_SCD_QUEUE_STTS_REG_MSK);
#endif

		/* Enable the scheduler for this queue. */
		iwx_write_prph(sc, IWX_SCD_EN_CTRL, qmsk);
	} else {
		/* this comes from OpenBSD */
		struct iwx_tx_queue_cfg_cmd cmd;
		struct iwx_rx_packet *pkt;
		struct iwx_tx_queue_cfg_rsp *resp;
		struct iwx_host_cmd hcmd = {
			.id = IWX_SCD_QUEUE_CFG,
			.flags = IWX_CMD_WANT_SKB,
	//		.resp_pkt_len = sizeof(*pkt) + sizeof(*resp),
		};
		struct iwx_tx_ring *ring = &sc->txq[qid];
		int err, fwqid;
		uint32_t wr_idx;
		size_t resp_len;
	
		iwx_reset_tx_ring(sc, ring);
	
		memset(&cmd, 0, sizeof(cmd));
		cmd.sta_id = sta_id;
		cmd.tid = tid;
		cmd.flags = htole16(IWX_TX_QUEUE_CFG_ENABLE_QUEUE);
		cmd.cb_size = htole32(IWX_TFD_QUEUE_CB_SIZE(num_slots));
		cmd.byte_cnt_addr = htole64(ring->bc_tbl.paddr);
		cmd.tfdq_addr = htole64(ring->desc_dma.paddr);
	
		hcmd.data[0] = &cmd;
		hcmd.len[0] = sizeof(cmd);
	
		err = iwx_send_cmd(sc, &hcmd);
		if (err)
			return err;
	
		pkt = hcmd.resp_pkt;
		if (!pkt || (pkt->hdr.flags & IWX_CMD_FAILED_MSK)) {
			IWX_DPRINTF(sc, IWX_DEBUG_CMD, "SCD_QUEUE_CFG command failed\n");
			err = EIO;
//			goto out;
			return err;
		}
	
		resp_len = iwx_rx_packet_payload_len(pkt);
		if (resp_len != sizeof(*resp)) {
			IWX_DPRINTF(sc, IWX_DEBUG_CMD, "SCD_QUEUE_CFG returned %zu bytes, expected %zu bytes\n", resp_len, sizeof(*resp));
			err = EIO;
//			goto out;
			return err;
		}
	
		resp = (void *)pkt->data;
		fwqid = le16toh(resp->queue_number);
		wr_idx = le16toh(resp->write_pointer);
	
		/* Unlike iwlwifi, we do not support dynamic queue ID assignment. */
		if (fwqid != qid) {
			IWX_DPRINTF(sc, IWX_DEBUG_CMD, "requested qid %d but %d was assigned\n", qid, fwqid);
			err = EIO;
//			goto out;
			return err;
		}
	
		if (wr_idx != ring->cur) {
			IWX_DPRINTF(sc, IWX_DEBUG_CMD, "fw write index is %d but ring is %d\n", wr_idx, ring->cur);
			err = EIO;
//			goto out;
			iwx_free_resp(sc, &hcmd);
			return err;
		}
	} // else
//out:
//	iwx_free_resp(sc, &hcmd);
//	} // else

	iwx_nic_unlock(sc);

//	IWX_DPRINTF(sc, IWX_DEBUG_XMIT, "%s: enabled txq %d FIFO %d\n",
//			__func__, qid, fifo);

	return 0;
}

/* pcie/trans-gen2.c */
/* iwx_post_alive() in openbsd */
static void
iwx_trans_pcie_fw_alive(struct iwx_softc *sc, uint32_t scd_base_addr)
{
	device_printf(sc->sc_dev, "%s: hoi\n", __func__);/* todo if_iwx: remove before submitting for review */

	memset(sc->queue_stopped, 0, sizeof(sc->queue_stopped));
	memset(sc->queue_used, 0, sizeof(sc->queue_used));

	iwx_ict_reset(sc);
	iwx_ctxt_info_free(sc);
}

/*
 * NVM read access and content parsing.  We do not support
 * external NVM or writing NVM.
 * iwlwifi/mvm/nvm.c
 */

/* Default NVM size to read */
#define IWX_NVM_DEFAULT_CHUNK_SIZE	(2*1024)

#define IWX_NVM_WRITE_OPCODE 1
#define IWX_NVM_READ_OPCODE 0

/* load nvm chunk response */
enum {
	IWM_READ_NVM_CHUNK_SUCCEED = 0,
	IWX_READ_NVM_CHUNK_NOT_VALID_ADDRESS = 1
};

static int
iwx_nvm_read_chunk(struct iwx_softc *sc, uint16_t section,
	uint16_t offset, uint16_t length, uint8_t *data, uint16_t *len)
{
	struct iwx_nvm_access_cmd nvm_access_cmd = {
		.offset = htole16(offset),
		.length = htole16(length),
		.type = htole16(section),
		.op_code = IWX_NVM_READ_OPCODE,
	};
	struct iwx_nvm_access_resp *nvm_resp;
	struct iwx_rx_packet *pkt;
	struct iwx_host_cmd cmd = {
		.id = IWX_NVM_ACCESS_CMD,
		.flags = IWX_CMD_WANT_SKB | IWX_CMD_SEND_IN_RFKILL,
//		.resp_pkt_len = IWX_CMD_RESP_MAX,
		.data = { &nvm_access_cmd, },
	};
	int ret, bytes_read, offset_read;
	uint8_t *resp_data;

	cmd.len[0] = sizeof(struct iwx_nvm_access_cmd);

	ret = iwx_send_cmd(sc, &cmd);
	if (ret) {
		device_printf(sc->sc_dev,
		    "Could not send NVM_ACCESS command (error=%d)\n", ret);
		return ret;
	}

	pkt = cmd.resp_pkt;

	/* Extract NVM response */
	nvm_resp = (void *)pkt->data;
	ret = le16toh(nvm_resp->status);
	bytes_read = le16toh(nvm_resp->length);
	offset_read = le16toh(nvm_resp->offset);
	resp_data = nvm_resp->data;
	if (ret) {
		if ((offset != 0) &&
		    (ret == IWX_READ_NVM_CHUNK_NOT_VALID_ADDRESS)) {
			/*
			 * meaning of NOT_VALID_ADDRESS:
			 * driver try to read chunk from address that is
			 * multiple of 2K and got an error since addr is empty.
			 * meaning of (offset != 0): driver already
			 * read valid data from another chunk so this case
			 * is not an error.
			 */
			IWX_DPRINTF(sc, IWX_DEBUG_EEPROM | IWX_DEBUG_RESET,
				    "NVM access command failed on offset 0x%x since that section size is multiple 2K\n",
				    offset);
			*len = 0;
			ret = 0;
		} else {
			IWX_DPRINTF(sc, IWX_DEBUG_EEPROM | IWX_DEBUG_RESET,
				    "NVM access command failed with status %d\n", ret);
			ret = EIO;
		}
		goto exit;
	}

	if (offset_read != offset) {
		device_printf(sc->sc_dev,
		    "NVM ACCESS response with invalid offset %d\n",
		    offset_read);
		ret = EINVAL;
		goto exit;
	}

	if (bytes_read > length) {
		device_printf(sc->sc_dev,
		    "NVM ACCESS response with too much data "
		    "(%d bytes requested, %d bytes received)\n",
		    length, bytes_read);
		ret = EINVAL;
		goto exit;
	}

	/* Write data to NVM */
	memcpy(data + offset, resp_data, bytes_read);
	*len = bytes_read;

 exit:
	iwx_free_resp(sc, &cmd);
	return ret;
}


/*
 * Reads an NVM section completely.
 * NICs prior to 7000 family don't have a real NVM, but just read
 * section 0 which is the EEPROM. Because the EEPROM reading is unlimited
 * by uCode, we need to manually check in this case that we don't
 * overflow and try to read more than the EEPROM size.
 * For 7000 family NICs, we supply the maximal size we can read, and
 * the uCode fills the response with as much data as we can,
 * without overflowing, so no check is needed.
 */
static int
iwx_nvm_read_section(struct iwx_softc *sc,
	uint16_t section, uint8_t *data, uint16_t *len, uint32_t size_read)
{
	uint16_t seglen, length, offset = 0;
	int ret;

	/* Set nvm section read length */
	length = IWX_NVM_DEFAULT_CHUNK_SIZE;

	seglen = length;

	/* Read the NVM until exhausted (reading less than requested) */
	while (seglen == length) {
		/* Check no memory assumptions fail and cause an overflow */
		if ((size_read + offset + length) >
		    sc->cfg->eeprom_size) {
			device_printf(sc->sc_dev,
			    "EEPROM size is too small for NVM\n");
			return ENOBUFS;
		}

		ret = iwx_nvm_read_chunk(sc, section, offset, length, data, &seglen);
		if (ret) {
			IWX_DPRINTF(sc, IWX_DEBUG_EEPROM | IWX_DEBUG_RESET,
				    "Cannot read NVM from section %d offset %d, length %d\n",
				    section, offset, length);
			return ret;
		}
		offset += seglen;
	}

	IWX_DPRINTF(sc, IWX_DEBUG_EEPROM | IWX_DEBUG_RESET,
		    "NVM section %d read completed\n", section);
	*len = offset;
	return 0;
}

/*
 * BEGIN IWM_NVM_PARSE
 */

/* iwlwifi/iwl-nvm-parse.c */

/* NVM offsets (in words) definitions */
enum iwx_nvm_offsets {
	IWX_NVM_VERSION = 0,
//};

//enum iwx_8000_nvm_offsets {
	/* NVM HW-Section offset (in words) definitions */
	IWM_HW_ADDR0_WFPM_8000 = 0x12,
	IWM_HW_ADDR1_WFPM_8000 = 0x16,
	IWM_HW_ADDR0_PCIE_8000 = 0x8A,
	IWM_HW_ADDR1_PCIE_8000 = 0x8E,
	IWX_MAC_ADDRESS_OVERRIDE_8000 = 1,

	/* NVM SW-Section offset (in words) definitions */
//	IWM_NVM_SW_SECTION_8000 = 0x1C0,
//	IWM_NVM_VERSION_8000 = 0,
//	IWX_RADIO_CFG_8000 = 0,
	IWX_RADIO_CFG = 0,
//	IWM_SKU_8000 = 2,
	IWX_SKU = 2,
//	IWX_N_HW_ADDRS_8000 = 3,
	IWX_N_HW_ADDRS = 3,

	/* NVM REGULATORY -Section offset (in words) definitions */
	IWX_NVM_CHANNELS = 0,
	IWX_NVM_LAR_OFFSET_8000_OLD = 0x4C7,
	IWX_NVM_LAR_OFFSET_8000 = 0x507,
	IWX_NVM_LAR_ENABLED_8000 = 0x7,

	/* NVM calibration section offset (in words) definitions */
	IWM_NVM_CALIB_SECTION_8000 = 0x2B8,
	IWM_XTAL_CALIB_8000 = 0x316 - IWM_NVM_CALIB_SECTION_8000
};

/* SKU Capabilities (actual values from NVM definition) */
enum nvm_sku_bits {
	IWX_NVM_SKU_CAP_BAND_24GHZ	= (1 << 0),
	IWX_NVM_SKU_CAP_BAND_52GHZ	= (1 << 1),
	IWM_NVM_SKU_CAP_11N_ENABLE	= (1 << 2),
	IWM_NVM_SKU_CAP_11AC_ENABLE	= (1 << 3),
};

/* radio config bits (actual values from NVM definition) */
#define IWX_NVM_RF_CFG_FLAVOR_MSK(x) (x & 0xF)
#define IWX_NVM_RF_CFG_DASH_MSK(x)   ((x >> 4) & 0xF)
#define IWX_NVM_RF_CFG_STEP_MSK(x)   ((x >> 8) & 0xF)
#define IWX_NVM_RF_CFG_TYPE_MSK(x)   ((x >> 12) & 0xFFF)
#define IWX_NVM_RF_CFG_TX_ANT_MSK(x) ((x >> 24) & 0xF)
#define IWX_NVM_RF_CFG_RX_ANT_MSK(x) ((x >> 28) & 0xF)

/**
 * enum iwm_nvm_channel_flags - channel flags in NVM
 * @IWM_NVM_CHANNEL_VALID: channel is usable for this SKU/geo
 * @IWM_NVM_CHANNEL_IBSS: usable as an IBSS channel
 * @IWM_NVM_CHANNEL_ACTIVE: active scanning allowed
 * @IWM_NVM_CHANNEL_RADAR: radar detection required
 * XXX cannot find this (DFS) flag in iwm-nvm-parse.c
 * @IWM_NVM_CHANNEL_DFS: dynamic freq selection candidate
 * @IWM_NVM_CHANNEL_WIDE: 20 MHz channel okay (?)
 * @IWM_NVM_CHANNEL_40MHZ: 40 MHz channel okay (?)
 * @IWM_NVM_CHANNEL_80MHZ: 80 MHz channel okay (?)
 * @IWM_NVM_CHANNEL_160MHZ: 160 MHz channel okay (?)
 */
enum iwx_nvm_channel_flags {
	IWX_NVM_CHANNEL_VALID = (1 << 0),
	IWX_NVM_CHANNEL_IBSS = (1 << 1),
	IWX_NVM_CHANNEL_ACTIVE = (1 << 3),
	IWX_NVM_CHANNEL_RADAR = (1 << 4),
	IWM_NVM_CHANNEL_DFS = (1 << 7),
	IWM_NVM_CHANNEL_WIDE = (1 << 8),
	IWM_NVM_CHANNEL_40MHZ = (1 << 9),
	IWM_NVM_CHANNEL_80MHZ = (1 << 10),
	IWM_NVM_CHANNEL_160MHZ = (1 << 11),
};

/*
 * Translate EEPROM flags to net80211.
 */
static uint32_t
iwx_eeprom_channel_flags(uint16_t ch_flags)
{
	uint32_t nflags;

	nflags = 0;
	if ((ch_flags & IWX_NVM_CHANNEL_ACTIVE) == 0)
		nflags |= IEEE80211_CHAN_PASSIVE;
	if ((ch_flags & IWX_NVM_CHANNEL_IBSS) == 0)
		nflags |= IEEE80211_CHAN_NOADHOC;
	if (ch_flags & IWX_NVM_CHANNEL_RADAR) {
		nflags |= IEEE80211_CHAN_DFS;
		/* Just in case. */
		nflags |= IEEE80211_CHAN_NOADHOC;
	}

	return (nflags);
}

static void
iwx_add_channel_band(struct iwx_softc *sc, struct ieee80211_channel chans[],
    int maxchans, int *nchans, int ch_idx, size_t ch_num,
    const uint8_t bands[])
{
	const uint16_t * const nvm_ch_flags = sc->nvm_data->nvm_ch_flags;
	uint32_t nflags;
	uint16_t ch_flags;
	uint8_t ieee;
	int error;

	for (; ch_idx < ch_num; ch_idx++) {
		ch_flags = le16_to_cpup(nvm_ch_flags + ch_idx);
		ieee = iwx_nvm_channels[ch_idx];

		if (!(ch_flags & IWX_NVM_CHANNEL_VALID)) {
			IWX_DPRINTF(sc, IWX_DEBUG_EEPROM,
			    "Ch. %d Flags %x [%sGHz] - No traffic\n",
			    ieee, ch_flags,
			    (ch_idx >= IWX_NUM_2GHZ_CHANNELS) ?
			    "5.2" : "2.4");
			continue;
		}

		nflags = iwx_eeprom_channel_flags(ch_flags);
		error = ieee80211_add_channel(chans, maxchans, nchans,
		    ieee, 0, 0, nflags, bands);
		if (error != 0)
			break;

		IWX_DPRINTF(sc, IWX_DEBUG_EEPROM,
		    "Ch. %d Flags %x [%sGHz] - Added\n",
		    ieee, ch_flags,
		    (ch_idx >= IWX_NUM_2GHZ_CHANNELS) ?
		    "5.2" : "2.4");
	}
}

static void
iwx_init_channel_map(struct ieee80211com *ic, int maxchans, int *nchans,
    struct ieee80211_channel chans[])
{
	struct iwx_softc *sc = ic->ic_softc;
	struct iwx_nvm_data *data = sc->nvm_data;
	uint8_t bands[IEEE80211_MODE_BYTES];
	size_t ch_num;

	memset(bands, 0, sizeof(bands));
	/* 1-13: 11b/g channels. */
	setbit(bands, IEEE80211_MODE_11B);
	setbit(bands, IEEE80211_MODE_11G);
	iwx_add_channel_band(sc, chans, maxchans, nchans, 0,
	    IWX_NUM_2GHZ_CHANNELS - 1, bands);

	/* 14: 11b channel only. */
	clrbit(bands, IEEE80211_MODE_11G);
	iwx_add_channel_band(sc, chans, maxchans, nchans,
	    IWX_NUM_2GHZ_CHANNELS - 1, IWX_NUM_2GHZ_CHANNELS, bands);

	if (data->sku_cap_band_52GHz_enable) {
		ch_num = nitems(iwx_nvm_channels);
		memset(bands, 0, sizeof(bands));
		setbit(bands, IEEE80211_MODE_11A);
		iwx_add_channel_band(sc, chans, maxchans, nchans,
		    IWX_NUM_2GHZ_CHANNELS, ch_num, bands);
	}
}

static void
iwx_set_hw_address_family_8000(struct iwx_softc *sc, struct iwx_nvm_data *data,
	const uint16_t *mac_override, const uint16_t *nvm_hw)
{
	const uint8_t *hw_addr;

	if (mac_override) {
		static const uint8_t reserved_mac[] = {
			0x02, 0xcc, 0xaa, 0xff, 0xee, 0x00
		};

		hw_addr = (const uint8_t *)(mac_override +
				 IWX_MAC_ADDRESS_OVERRIDE_8000);

		/*
		 * Store the MAC address from MAO section.
		 * No byte swapping is required in MAO section
		 */
		IEEE80211_ADDR_COPY(data->hw_addr, hw_addr);

		/*
		 * Force the use of the OTP MAC address in case of reserved MAC
		 * address in the NVM, or if address is given but invalid.
		 */
		if (!IEEE80211_ADDR_EQ(reserved_mac, hw_addr) &&
		    !IEEE80211_ADDR_EQ(ieee80211broadcastaddr, data->hw_addr) &&
		    iwx_is_valid_ether_addr(data->hw_addr) &&
		    !IEEE80211_IS_MULTICAST(data->hw_addr))
			return;

		IWX_DPRINTF(sc, IWX_DEBUG_RESET,
		    "%s: mac address from nvm override section invalid\n",
		    __func__);
	}

	if (nvm_hw) {
		/* read the mac address from WFMP registers */
		uint32_t mac_addr0 =
		    htole32(iwx_read_prph(sc, IWX_WFMP_MAC_ADDR_0));
		uint32_t mac_addr1 =
		    htole32(iwx_read_prph(sc, IWX_WFMP_MAC_ADDR_1));

		hw_addr = (const uint8_t *)&mac_addr0;
		data->hw_addr[0] = hw_addr[3];
		data->hw_addr[1] = hw_addr[2];
		data->hw_addr[2] = hw_addr[1];
		data->hw_addr[3] = hw_addr[0];

		hw_addr = (const uint8_t *)&mac_addr1;
		data->hw_addr[4] = hw_addr[1];
		data->hw_addr[5] = hw_addr[0];

		return;
	}

	device_printf(sc->sc_dev, "%s: mac address not found\n", __func__);
	memset(data->hw_addr, 0, sizeof(data->hw_addr));
}

static int
iwx_get_sku(const struct iwx_softc *sc, const uint16_t *nvm_sw,
	    const uint16_t *phy_sku)
{
	/* todo if_iwx: tune to match/support family 22000 */
	return le32_to_cpup((const uint32_t *)(phy_sku + IWX_SKU));
}

static int
iwx_get_nvm_version(const struct iwx_softc *sc, const uint16_t *nvm_sw)
{
	/* todo if_iwx: tune to match/support family 22000 */
	return le32_to_cpup((const uint32_t *)(nvm_sw +
				IWX_NVM_VERSION));
}

static int
iwx_get_radio_cfg(const struct iwx_softc *sc, const uint16_t *nvm_sw,
		  const uint16_t *phy_sku)
{
	/* todo if_iwx: tune to match/support family 22000 */
        return le32_to_cpup((const uint32_t *)(phy_sku + IWX_RADIO_CFG));
}

static int
iwx_get_n_hw_addrs(const struct iwx_softc *sc, const uint16_t *nvm_sw)
{
	int n_hw_addr;

	/* todo if_iwx: tune to match/support family 22000 */
	// n_hw_addr = le32_to_cpup((const uint32_t *)(nvm_sw + IWX_N_HW_ADDRS_8000));
	n_hw_addr = le32_to_cpup((const uint32_t *)(nvm_sw + IWX_N_HW_ADDRS));
        return n_hw_addr & IWX_N_HW_ADDR_MASK;
}

static void
iwx_set_radio_cfg(const struct iwx_softc *sc, struct iwx_nvm_data *data,
		  uint32_t radio_cfg)
{
	/* todo if_iwx: tune to match/support family 22000 */
	/* set the radio configuration for family 8000 */
//	data->radio_cfg_type = IWX_NVM_RF_CFG_TYPE_MSK_8000(radio_cfg);
//	data->radio_cfg_step = IWX_NVM_RF_CFG_STEP_MSK_8000(radio_cfg);
//	data->radio_cfg_dash = IWX_NVM_RF_CFG_DASH_MSK_8000(radio_cfg);
//	data->radio_cfg_pnum = IWX_NVM_RF_CFG_FLAVOR_MSK_8000(radio_cfg);
//	data->valid_tx_ant = IWX_NVM_RF_CFG_TX_ANT_MSK_8000(radio_cfg);
//	data->valid_rx_ant = IWX_NVM_RF_CFG_RX_ANT_MSK_8000(radio_cfg);
	data->radio_cfg_type = IWX_NVM_RF_CFG_TYPE_MSK(radio_cfg);
	data->radio_cfg_step = IWX_NVM_RF_CFG_STEP_MSK(radio_cfg);
	data->radio_cfg_dash = IWX_NVM_RF_CFG_DASH_MSK(radio_cfg);
	data->radio_cfg_pnum = IWX_NVM_RF_CFG_FLAVOR_MSK(radio_cfg);
	data->valid_tx_ant = IWX_NVM_RF_CFG_TX_ANT_MSK(radio_cfg);
	data->valid_rx_ant = IWX_NVM_RF_CFG_RX_ANT_MSK(radio_cfg);
}

static int
iwx_set_hw_address(struct iwx_softc *sc, struct iwx_nvm_data *data,
		   const uint16_t *nvm_hw, const uint16_t *mac_override)
{
#ifdef notyet /* for FAMILY 9000 */
	if (cfg->mac_addr_from_csr) {
		iwm_set_hw_address_from_csr(sc, data);
	} else
#endif
		/* todo if_iwx: merge the function below into this function  */
		iwx_set_hw_address_family_8000(sc, data, mac_override, nvm_hw);
//	}

	if (!iwx_is_valid_ether_addr(data->hw_addr)) {
		device_printf(sc->sc_dev, "no valid mac address was found\n");
		return EINVAL;
	}

	return 0;
}

static struct iwx_nvm_data *
iwx_parse_nvm_data(struct iwx_softc *sc,
		   const uint16_t *nvm_hw, const uint16_t *nvm_sw,
		   const uint16_t *nvm_calib, const uint16_t *mac_override,
		   const uint16_t *phy_sku, const uint16_t *regulatory)
{
	struct iwx_nvm_data *data;
	uint32_t sku, radio_cfg;
//	uint16_t lar_config;

	data = malloc(sizeof(*data) +
	    IWX_NUM_CHANNELS * sizeof(uint16_t),
	    M_DEVBUF, M_NOWAIT | M_ZERO);
	if (!data)
		return NULL;

	data->nvm_version = iwx_get_nvm_version(sc, nvm_sw);

	radio_cfg = iwx_get_radio_cfg(sc, nvm_sw, phy_sku);
	iwx_set_radio_cfg(sc, data, radio_cfg);

	sku = iwx_get_sku(sc, nvm_sw, phy_sku);
	data->sku_cap_band_24GHz_enable = sku & IWX_NVM_SKU_CAP_BAND_24GHZ;
	data->sku_cap_band_52GHz_enable = sku & IWX_NVM_SKU_CAP_BAND_52GHZ;
	data->sku_cap_11n_enable = 0;

	data->n_hw_addrs = iwx_get_n_hw_addrs(sc, nvm_sw);

	/* todo if_iwx: tune to match/support family 22000 */
#if 0
	if (sc->cfg->device_family >= IWX_DEVICE_FAMILY_8000) {
		/* TODO: use IWL_NVM_EXT */
		uint16_t lar_offset = data->nvm_version < 0xE39 ?
				       IWX_NVM_LAR_OFFSET_8000_OLD :
				       IWX_NVM_LAR_OFFSET_8000;

		lar_config = le16_to_cpup(regulatory + lar_offset);
		data->lar_enabled = !!(lar_config &
				       IWX_NVM_LAR_ENABLED_8000);
	}
#endif

	/* If no valid mac address was found - bail out */
	if (iwx_set_hw_address(sc, data, nvm_hw, mac_override)) {
		free(data, M_DEVBUF);
		return NULL;
	}

	memcpy(data->nvm_ch_flags, &regulatory[IWX_NVM_CHANNELS],
	    IWX_NUM_CHANNELS * sizeof(uint16_t));

	return data;
}


static void
iwx_free_nvm_data(struct iwx_nvm_data *data)
{
	if (data != NULL)
		free(data, M_DEVBUF);
}

static struct iwx_nvm_data *
iwx_parse_nvm_sections(struct iwx_softc *sc, struct iwx_nvm_section *sections)
{
	const uint16_t *hw, *sw, *calib, *regulatory, *mac_override, *phy_sku;

	/* todo if_iwx: tune to match/support family 22000 */
	/* Checking for required sections */
	/* SW and REGULATORY sections are mandatory */
	if (!sections[IWX_NVM_SECTION_TYPE_SW].data ||
	    !sections[IWX_NVM_SECTION_TYPE_REGULATORY].data) {
		device_printf(sc->sc_dev,
		    "Can't parse empty OTP/NVM sections\n");
		return NULL;
	}
	/* MAC_OVERRIDE or at least HW section must exist */
	if (!sections[sc->cfg->nvm_hw_section_num].data &&
	    !sections[IWX_NVM_SECTION_TYPE_MAC_OVERRIDE].data) {
		device_printf(sc->sc_dev,
		    "Can't parse mac_address, empty sections\n");
		return NULL;
	}

	/* PHY_SKU section is mandatory in B0 */
	if (!sections[IWX_NVM_SECTION_TYPE_PHY_SKU].data) {
		device_printf(sc->sc_dev,
		    "Can't parse phy_sku in B0, empty sections\n");
		return NULL;
	}

	hw = (const uint16_t *) sections[sc->cfg->nvm_hw_section_num].data;
	sw = (const uint16_t *)sections[IWX_NVM_SECTION_TYPE_SW].data;
	calib = (const uint16_t *)
	    sections[IWX_NVM_SECTION_TYPE_CALIBRATION].data;
	regulatory =
	    (const uint16_t *)sections[IWX_NVM_SECTION_TYPE_REGULATORY].data;
	mac_override = (const uint16_t *)
	    sections[IWX_NVM_SECTION_TYPE_MAC_OVERRIDE].data;
	phy_sku = (const uint16_t *)sections[IWX_NVM_SECTION_TYPE_PHY_SKU].data;

	return iwx_parse_nvm_data(sc, hw, sw, calib, mac_override,
	    phy_sku, regulatory);
}

static int
iwx_nvm_init(struct iwx_softc *sc)
{
	struct iwx_nvm_section nvm_sections[IWX_NVM_MAX_NUM_SECTIONS];
	int i, ret, section;
	uint32_t size_read = 0;
	uint8_t *nvm_buffer, *temp;
	uint16_t len;

	memset(nvm_sections, 0, sizeof(nvm_sections));

	if (sc->cfg->nvm_hw_section_num >= IWX_NVM_MAX_NUM_SECTIONS)
		return EINVAL;

	/* load NVM values from nic */
	/* Read From FW NVM */
	IWX_DPRINTF(sc, IWX_DEBUG_EEPROM, "Read from NVM\n");

	nvm_buffer = malloc(sc->cfg->eeprom_size, M_DEVBUF, M_NOWAIT | M_ZERO);
	if (!nvm_buffer)
		return ENOMEM;
	for (section = 0; section < IWX_NVM_MAX_NUM_SECTIONS; section++) {
		/* we override the constness for initial read */
		ret = iwx_nvm_read_section(sc, section, nvm_buffer,
					   &len, size_read);
		if (ret)
			continue;
		size_read += len;
		temp = malloc(len, M_DEVBUF, M_NOWAIT);
		if (!temp) {
			ret = ENOMEM;
			break;
		}
		memcpy(temp, nvm_buffer, len);

		nvm_sections[section].data = temp;
		nvm_sections[section].length = len;
	}
	if (!size_read)
		device_printf(sc->sc_dev, "OTP is blank\n");
	free(nvm_buffer, M_DEVBUF);

	sc->nvm_data = iwx_parse_nvm_sections(sc, nvm_sections);
	if (!sc->nvm_data)
		return EINVAL;
	IWX_DPRINTF(sc, IWX_DEBUG_EEPROM | IWX_DEBUG_RESET,
		    "nvm version = %x\n", sc->nvm_data->nvm_version);

	for (i = 0; i < IWX_NVM_MAX_NUM_SECTIONS; i++) {
		if (nvm_sections[i].data != NULL)
			free(nvm_sections[i].data, M_DEVBUF);
	}

	return 0;
}

/*
 * ucode
 */

/* iwx_pcie_load_given_ucode() follows FreeBSD's parameters (like if_iwm),
 * the implementation comes from OpenBSD */
static int
iwx_pcie_load_given_ucode(struct iwx_softc *sc, const struct iwx_fw_img *fw)
{
	device_printf(sc->sc_dev, "%s: hoi\n", __func__);/* todo if_iwx: remove before submitting for review */

	int ret = 0;

	if ((ret = iwx_ctxt_info_init(sc, fw)) != 0) {
			device_printf(sc->sc_dev,
					"%s: could not init context info\n",
					__func__);
			return ret;
	}

	return 0;
}

/* XXX Get rid of this definition */
static inline void
iwx_enable_fw_load_int(struct iwx_softc *sc)
{
	device_printf(sc->sc_dev, "%s: hoi\n", __func__);/* todo if_iwx: remove before submitting for review */

	IWX_DPRINTF(sc, IWX_DEBUG_INTR, "Enabling FW load interrupt\n");
	sc->sc_intmask = IWX_CSR_INT_BIT_ALIVE | IWX_CSR_INT_BIT_FH_RX;
	IWX_WRITE(sc, IWX_CSR_INT_MASK, sc->sc_intmask);
}

/* pcie/trans-gen2.c */
/* XXX Add proper rfkill support code */
static int
iwx_start_fw(struct iwx_softc *sc, const struct iwx_fw_img *fw)
{
	device_printf(sc->sc_dev, "%s: hoi\n", __func__);/* todo if_iwx: remove before submitting for review */

	int ret;

	/* This may fail if AMT took ownership of the device */
	if (iwx_prepare_card_hw(sc)) {
		device_printf(sc->sc_dev,
		    "%s: Exit HW not ready\n", __func__);
		ret = EIO;
		goto out;
	}

	IWX_WRITE(sc, IWX_CSR_INT, 0xFFFFFFFF);

	iwx_disable_interrupts(sc);

	/* make sure rfkill handshake bits are cleared */
	IWX_WRITE(sc, IWX_CSR_UCODE_DRV_GP1_CLR, IWX_CSR_UCODE_SW_BIT_RFKILL);
	IWX_WRITE(sc, IWX_CSR_UCODE_DRV_GP1_CLR,
	    IWX_CSR_UCODE_DRV_GP1_BIT_CMD_BLOCKED);

	/* clear (again), then enable host interrupts */
	IWX_WRITE(sc, IWX_CSR_INT, 0xFFFFFFFF);

	ret = iwx_nic_init(sc);
	if (ret) {
		device_printf(sc->sc_dev, "%s: Unable to init nic\n", __func__);
		goto out;
	}

	/*
	 * Now, we load the firmware and don't want to be interrupted, even
	 * by the RF-Kill interrupt (hence mask all the interrupt besides the
	 * FH_TX interrupt which is needed to load the firmware). If the
	 * RF-Kill switch is toggled, we will find out after having loaded
	 * the firmware and return the proper value to the caller.
	 */
	iwx_enable_fw_load_int(sc);

	/* todo: add support for AX210,
	 * see iwl_trans_pcie_gen2_start_fw() */

	/* XXX re-check RF-Kill state */

	ret = iwx_pcie_load_given_ucode(sc, fw);

out:
	return ret;
}

static int
iwx_send_tx_ant_cfg(struct iwx_softc *sc, uint8_t valid_tx_ant)
{
	struct iwx_tx_ant_cfg_cmd tx_ant_cmd = {
		.valid = htole32(valid_tx_ant),
	};

	return iwx_send_cmd_pdu(sc, IWX_TX_ANT_CONFIGURATION_CMD,
	    IWX_CMD_SYNC, sizeof(tx_ant_cmd), &tx_ant_cmd);
}

/* iwlwifi: mvm/fw.c */
static int
iwx_send_phy_cfg_cmd(struct iwx_softc *sc)
{
	struct iwx_phy_cfg_cmd phy_cfg_cmd;

	/* Set parameters */
	phy_cfg_cmd.phy_cfg = htole32(iwx_get_phy_config(sc));
	phy_cfg_cmd.calib_control.event_trigger =
	    sc->sc_default_calib[IWX_UCODE_REGULAR].event_trigger;
	phy_cfg_cmd.calib_control.flow_trigger =
	    sc->sc_default_calib[IWX_UCODE_REGULAR].flow_trigger;

	IWX_DPRINTF(sc, IWX_DEBUG_CMD | IWX_DEBUG_RESET,
	    "Sending Phy CFG command: 0x%x\n", phy_cfg_cmd.phy_cfg);
	return iwx_send_cmd_pdu(sc, IWX_PHY_CONFIGURATION_CMD, IWX_CMD_SYNC,
	    sizeof(phy_cfg_cmd), &phy_cfg_cmd);
}

/* source: OpenBSD */
int
iwx_send_dqa_cmd(struct iwx_softc *sc)
{
	struct iwx_dqa_enable_cmd dqa_cmd = {
		.cmd_queue = htole32(IWX_DQA_CMD_QUEUE),
	};
	uint32_t cmd_id;

	cmd_id = iwx_cmd_id(IWX_DQA_ENABLE_CMD, IWX_DATA_PATH_GROUP, 0);
	return iwx_send_cmd_pdu(sc, cmd_id, 0, sizeof(dqa_cmd), &dqa_cmd);
}

static int
iwx_alive_fn(struct iwx_softc *sc, struct iwx_rx_packet *pkt, void *data)
{
	struct iwx_alive_data *alive_data = data;
	struct iwx_alive_resp_v4 *palive;
	struct iwx_umac_alive *umac;
	struct iwx_lmac_alive *lmac1 = NULL;
	struct iwx_lmac_alive *lmac2 = NULL;
	uint16_t status = 0;

	if (iwx_rx_packet_payload_len(pkt) == sizeof(*palive)) {
		palive = (void *)pkt->data;
		umac = &palive->umac_data;
		lmac1 = &palive->lmac_data[0];
		lmac2 = &palive->lmac_data[1];
		status = le16toh(palive->status);
	}

	sc->lmac_error_event_table[0] = le32toh(lmac1->dbg_ptrs.error_event_table_ptr);
	if (lmac2)
		sc->lmac_error_event_table[1] =
			le32toh(lmac2->dbg_ptrs.error_event_table_ptr);
	sc->log_event_table = le32toh(lmac1->dbg_ptrs.log_event_table_ptr);
	alive_data->scd_base_addr = le32toh(lmac1->dbg_ptrs.scd_base_ptr);
	alive_data->valid = status == IWX_ALIVE_STATUS_OK;
	if (sc->umac_error_event_table)
		sc->support_umac_log = TRUE;

	IWX_DPRINTF(sc, IWX_DEBUG_FW,
		    "Alive ucode status 0x%04x revision 0x%01X 0x%01X\n",
		    status, lmac1->ver_type, lmac1->ver_subtype);

	if (lmac2)
		IWX_DPRINTF(sc, IWX_DEBUG_FW, "Alive ucode CDB\n");

	IWX_DPRINTF(sc, IWX_DEBUG_FW,
		    "UMAC version: Major - 0x%x, Minor - 0x%x\n",
		    le32toh(umac->umac_major),
		    le32toh(umac->umac_minor));

	return TRUE;
}

static int
iwx_wait_phy_db_entry(struct iwx_softc *sc,
	struct iwx_rx_packet *pkt, void *data)
{
//	struct iwx_phy_db *phy_db = data;

	if (pkt->hdr.code != IWX_CALIB_RES_NOTIF_PHY_DB) {
		if(pkt->hdr.code != IWX_INIT_COMPLETE_NOTIF) {
			device_printf(sc->sc_dev, "%s: Unexpected cmd: %d\n",
			    __func__, pkt->hdr.code);
		}
		return TRUE;
	}

#ifdef not_in_iwx
//	if (iwx_phy_db_set_section(phy_db, pkt)) {
//		device_printf(sc->sc_dev,
//		    "%s: iwx_phy_db_set_section failed\n", __func__);
//	}
#endif

	return FALSE;
}

static int
iwx_load_ucode_wait_alive(struct iwx_softc *sc)
{
	struct iwx_notification_wait alive_wait;
	struct iwx_alive_data alive_data;
	struct iwx_fw_img *fw = &sc->sc_fw.img[IWX_UCODE_REGULAR]; /* img, not fw_sects */
	int error;
	static const uint16_t alive_cmd[] = { IWX_ALIVE };

	sc->ucode_loaded = FALSE;

	memset(&alive_data, 0, sizeof(alive_data));
	iwx_init_notification_wait(sc->sc_notif_wait, &alive_wait,
				   alive_cmd, nitems(alive_cmd),
				   iwx_alive_fn, &alive_data);

	if ((error = iwx_start_fw(sc, fw)) != 0) {
		device_printf(sc->sc_dev, "%s: iwx_start_fw failed; %d\n",
				__func__, error);
		iwx_remove_notification(sc->sc_notif_wait, &alive_wait);
		return error;
	}

	/*
	 * Some things may run in the background now, but we
	 * just wait for the ALIVE notification here.
	 */
	IWX_UNLOCK(sc);
	error = iwx_wait_notification(sc->sc_notif_wait, &alive_wait,
				      IWX_UCODE_ALIVE_TIMEOUT);
	IWX_LOCK(sc);
	if (error) {
		uint32_t a = 0x5a5a5a5a, b = 0x5a5a5a5a;
		uint32_t c = 0x5a5a5a5a, d = 0x5a5a5a5a;
		uint32_t e = 0x5a5a5a5a;
		uint32_t f = 0x5a5a5a5a, g = 0x5a5a5a5a;
		if (iwx_nic_lock(sc)) {
			a = iwx_read_umac_prph(sc, IWX_UMAG_SB_CPU_1_STATUS);
			b = iwx_read_umac_prph(sc, IWX_UMAG_SB_CPU_2_STATUS);
			c = iwx_read_umac_prph(sc, IWX_UREG_UMAC_CURRENT_PC);
			d = iwx_read_umac_prph(sc, IWX_UREG_LMAC1_CURRENT_PC);
			if (iwx_fw_has_capa(sc, IWX_UCODE_TLV_CAPA_CDB_SUPPORT)) {
				e = iwx_read_umac_prph(sc, IWX_UREG_LMAC2_CURRENT_PC);
			}
			f = iwx_read_prph(sc, IWX_SB_CPU_1_STATUS);
			g = iwx_read_prph(sc, IWX_SB_CPU_2_STATUS);

			iwx_nic_unlock(sc);
		}
		device_printf(sc->sc_dev,
		    "SecBoot (UMAG) CPU1 Status: 0x%x, CPU2 Status: 0x%x\n",
		    a, b);
		device_printf(sc->sc_dev,
		    "SecBoot CPU1 Status: 0x%x, CPU2 Status: 0x%x\n",
		    f, g);

		return error;
	}

	if (!alive_data.valid) {
		device_printf(sc->sc_dev, "%s: Loaded ucode is not valid\n",
		    __func__);
		return EIO;
	}

	/* aka iwx_post_alive() */
	iwx_trans_pcie_fw_alive(sc, alive_data.scd_base_addr);

	if (!error)
		/* how does openbsd signal this? */
		sc->ucode_loaded = TRUE;

	return error;
}

/*
 * mvm misc bits
 */

/*
 * follows iwlwifi/fw.c
 */

/* source: iwlwifi, mvm/fw.c */
static int
iwx_wait_init_complete(struct iwx_softc *sc,
	struct iwx_rx_packet *pkt, void *data)
{
	return 1;
}

/* iwl_run_unified_mvm_ucode() */
static int
iwx_run_init_unified_ucode(struct iwx_softc *sc, bool read_nvm)
{
	struct iwx_notification_wait init_wait;
	static const uint16_t init_complete[] = {
		IWX_INIT_COMPLETE_NOTIF,
	};
	struct iwx_nvm_access_complete_cmd nvm_complete = {};
	struct iwx_init_extended_cfg_cmd init_cfg = {
		.init_flags = htole32(IWX_INIT_NVM)};
	int ret;

	/* do not operate with rfkill switch turned on */
	if ((sc->sc_flags & IWX_FLAG_RFKILL) && !read_nvm) {
		device_printf(sc->sc_dev,
		    "radio is disabled by hardware switch\n");
		return EPERM;
	}

	iwx_init_notification_wait(sc->sc_notif_wait,
				   &init_wait,
				   init_complete,
				   nitems(init_complete),
				   iwx_wait_init_complete,
				   NULL);

	/* Will also start the device */
	ret = iwx_load_ucode_wait_alive(sc);
	if (ret) {
		device_printf(sc->sc_dev, "Failed to start INIT ucode: %d\n",
		    ret);
		goto error;
	}

	/*
	 * Send init config command to mark that we are sending NVM
	 * access commands
	 */
	ret = iwx_send_cmd_pdu(sc, IWX_WIDE_ID(IWX_SYSTEM_GROUP,
	    IWX_INIT_EXTENDED_CFG_CMD), 0, sizeof(init_cfg), &init_cfg);
	device_printf(sc->sc_dev, "%s: [1] ret=%d\n",
			__func__, ret);/* todo if_iwx: remove before submitting for review */

	if (ret)
		return ret;

	if (read_nvm) {
		/* Read nvm */
		ret = iwx_nvm_init(sc);
		if (ret) {
			device_printf(sc->sc_dev, "failed to read nvm\n");
			goto error;
		}
//		IEEE80211_ADDR_COPY(sc->sc_ic.ic_macaddr, sc->nvm_data->hw_addr);
//		goto error;
	}

	ret = iwx_send_cmd_pdu(sc, IWX_WIDE_ID(IWX_REGULATORY_AND_NVM_GROUP,
				IWX_NVM_ACCESS_COMPLETE), 0, sizeof(nvm_complete), &nvm_complete);

	/* Wait for the init complete notification from the firmware. */
	IWX_UNLOCK(sc);
	ret = iwx_wait_notification(sc->sc_notif_wait, &init_wait,
	    IWX_UCODE_ALIVE_TIMEOUT);
	IWX_LOCK(sc);


	goto out;

error:
	iwx_remove_notification(sc->sc_notif_wait, &init_wait);
out:
	device_printf(sc->sc_dev, "%s: [2] ret=%d\n",
			__func__, ret);/* todo if_iwx: remove before submitting for review */

	return ret;
}

static int
iwx_config_ltr(struct iwx_softc *sc)
{
	struct iwx_ltr_config_cmd cmd = {
		.flags = htole32(IWX_LTR_CFG_FLAG_FEATURE_ENABLE),
	};

	if (!sc->sc_ltr_enabled)
		return 0;

	return iwx_send_cmd_pdu(sc, IWX_LTR_CONFIG, 0, sizeof(cmd), &cmd);
}

/*
 * receive side
 */

/* source: OpenBSD */
void
iwx_update_rx_desc(struct iwx_softc *sc, struct iwx_rx_ring *ring, int idx)
{
	// silence the compiler: build error for struct bus_dmamap_t
//	struct iwx_rx_data *data = &ring->data[idx];

//	((uint64_t *)ring->desc)[idx] =
//	    htole64(data->map->dm_segs[0].ds_addr | (idx & 0x0fff));
	bus_dmamap_sync(sc->sc_dmat, ring->free_desc_dma.map,
//	    idx * sizeof(uint64_t), sizeof(uint64_t),
	    BUS_DMASYNC_PREWRITE);
}

/* (re)stock rx ring, called at init-time and at runtime */
static int
iwx_rx_addbuf(struct iwx_softc *sc, int size, int idx)
{
	struct iwx_rx_ring *ring = &sc->rxq;
	struct iwx_rx_data *data = &ring->data[idx];
	struct mbuf *m;
	bus_dmamap_t dmamap;
	bus_dma_segment_t seg;
	int nsegs, error;

	m = m_getjcl(M_NOWAIT, MT_DATA, M_PKTHDR, IWX_RBUF_SIZE);
	if (m == NULL)
		return ENOBUFS;

	m->m_len = m->m_pkthdr.len = m->m_ext.ext_size;
	error = bus_dmamap_load_mbuf_sg(ring->data_dmat, ring->spare_map, m,
	    &seg, &nsegs, BUS_DMA_NOWAIT);
	if (error != 0) {
		device_printf(sc->sc_dev,
		    "%s: can't map mbuf, error %d\n", __func__, error);
		m_freem(m);
		return error;
	}

	if (data->m != NULL)
		bus_dmamap_unload(ring->data_dmat, data->map);

	/* Swap ring->spare_map with data->map */
	dmamap = data->map;
	data->map = ring->spare_map;
	ring->spare_map = dmamap;

	bus_dmamap_sync(ring->data_dmat, data->map, BUS_DMASYNC_PREREAD);
	data->m = m;

	/* Update RX descriptor. */
	KASSERT((seg.ds_addr & 255) == 0, ("seg.ds_addr not aligned"));
	iwx_update_rx_desc(sc, ring, idx);

	return 0;
}

static void
iwx_rx_rx_phy_cmd(struct iwx_softc *sc, struct iwx_rx_packet *pkt)
{
	struct iwx_rx_phy_info *phy_info = (void *)pkt->data;

	IWX_DPRINTF(sc, IWX_DEBUG_RECV, "received PHY stats\n");

	memcpy(&sc->sc_last_phy_info, phy_info, sizeof(sc->sc_last_phy_info));
}

/*
 * Retrieve the average noise (in dBm) among receivers.
 */
static int
iwx_get_noise(struct iwx_softc *sc,
    const struct iwx_statistics_rx_non_phy *stats)
{
	int i, total, nbant, noise;

	total = nbant = noise = 0;
	for (i = 0; i < 3; i++) {
		noise = le32toh(stats->beacon_silence_rssi[i]) & 0xff;
		IWX_DPRINTF(sc, IWX_DEBUG_RECV, "%s: i=%d, noise=%d\n",
		    __func__,
		    i,
		    noise);

		if (noise) {
			total += noise;
			nbant++;
		}
	}

	IWX_DPRINTF(sc, IWX_DEBUG_RECV, "%s: nbant=%d, total=%d\n",
	    __func__, nbant, total);
#if 0
	/* There should be at least one antenna but check anyway. */
	return (nbant == 0) ? -127 : (total / nbant) - 107;
#else
	/* For now, just hard-code it to -96 to be safe */
	return (-96);
#endif
}

static void
iwx_handle_rx_statistics(struct iwx_softc *sc, struct iwx_rx_packet *pkt)
{
	struct iwx_notif_statistics_v13 *stats = (void *)&pkt->data;

	memcpy(&sc->sc_stats, stats, sizeof(sc->sc_stats));
	sc->sc_noise = iwx_get_noise(sc, &stats->rx.general);
}

/* iwlwifi: mvm/rx.c */
static int
iwx_rxmq_get_signal_strength(struct iwx_softc *sc,
    struct iwx_rx_mpdu_desc *desc)
{
	int energy_a, energy_b;

	energy_a = desc->v1.energy_a;
	energy_b = desc->v1.energy_b;
	energy_a = energy_a ? -energy_a : -256;
	energy_b = energy_b ? -energy_b : -256;
	return MAX(energy_a, energy_b);
}

/*
 * iwm_rx_rx_mpdu - IWM_REPLY_RX_MPDU_CMD handler
 *
 * Handles the actual data of the Rx packet from the fw
 */
static bool
iwx_rx_rx_mpdu(struct iwx_softc *sc, struct mbuf *m, uint32_t offset,
    bool stolen)
{
	struct ieee80211com *ic = &sc->sc_ic;
	struct ieee80211vap *vap = TAILQ_FIRST(&ic->ic_vaps);
	struct ieee80211_frame *wh;
	struct ieee80211_rx_stats rxs;
	struct iwx_rx_phy_info *phy_info;
	struct iwx_rx_mpdu_res_start *rx_res;
	struct iwx_rx_packet *pkt = mtodoff(m, struct iwx_rx_packet *, offset);
	uint32_t len;
	uint32_t rx_pkt_status;
	int rssi;

	phy_info = &sc->sc_last_phy_info;
	rx_res = (struct iwx_rx_mpdu_res_start *)pkt->data;
	wh = (struct ieee80211_frame *)(pkt->data + sizeof(*rx_res));
	len = le16toh(rx_res->byte_count);
	rx_pkt_status = le32toh(*(uint32_t *)(pkt->data + sizeof(*rx_res) + len));

	if (__predict_false(phy_info->cfg_phy_cnt > 20)) {
		device_printf(sc->sc_dev,
		    "dsp size out of range [0,20]: %d\n",
		    phy_info->cfg_phy_cnt);
		return false;
	}

	if (!(rx_pkt_status & IWX_RX_MPDU_RES_STATUS_CRC_OK) ||
	    !(rx_pkt_status & IWX_RX_MPDU_RES_STATUS_OVERRUN_OK)) {
		IWX_DPRINTF(sc, IWX_DEBUG_RECV,
		    "Bad CRC or FIFO: 0x%08X.\n", rx_pkt_status);
		return false;
	}

	/* todo if_iwx: sync with openbsd */
	rssi = 0;
#if 0
	rssi = iwx_rx_get_signal_strength(sc, phy_info);
#endif

	/* Map it to relative value */
	rssi = rssi - sc->sc_noise;

	/* replenish ring for the buffer we're going to feed to the sharks */
	if (!stolen && iwx_rx_addbuf(sc, IWX_RBUF_SIZE, sc->rxq.cur) != 0) {
		device_printf(sc->sc_dev, "%s: unable to add more buffers\n",
		    __func__);
		return false;
	}

	m->m_data = pkt->data + sizeof(*rx_res);
	m->m_pkthdr.len = m->m_len = len;

	IWX_DPRINTF(sc, IWX_DEBUG_RECV,
	    "%s: rssi=%d, noise=%d\n", __func__, rssi, sc->sc_noise);

	IWX_DPRINTF(sc, IWX_DEBUG_RECV,
	    "%s: phy_info: channel=%d, flags=0x%08x\n",
	    __func__,
	    le16toh(phy_info->channel),
	    le16toh(phy_info->phy_flags));

	/*
	 * Populate an RX state struct with the provided information.
	 */
	bzero(&rxs, sizeof(rxs));
	rxs.r_flags |= IEEE80211_R_IEEE | IEEE80211_R_FREQ;
	rxs.r_flags |= IEEE80211_R_NF | IEEE80211_R_RSSI;
	rxs.c_ieee = le16toh(phy_info->channel);
	if (le16toh(phy_info->phy_flags & IWX_RX_RES_PHY_FLAGS_BAND_24)) {
		rxs.c_freq = ieee80211_ieee2mhz(rxs.c_ieee, IEEE80211_CHAN_2GHZ);
	} else {
		rxs.c_freq = ieee80211_ieee2mhz(rxs.c_ieee, IEEE80211_CHAN_5GHZ);
	}

	/* rssi is in 1/2db units */
	rxs.c_rssi = rssi * 2;
	rxs.c_nf = sc->sc_noise;
	if (ieee80211_add_rx_params(m, &rxs) == 0)
		return false;

	if (ieee80211_radiotap_active_vap(vap)) {
		struct iwx_rx_radiotap_header *tap = &sc->sc_rxtap;

		tap->wr_flags = 0;
		if (phy_info->phy_flags & htole16(IWX_PHY_INFO_FLAG_SHPREAMBLE))
			tap->wr_flags |= IEEE80211_RADIOTAP_F_SHORTPRE;
		tap->wr_chan_freq = htole16(rxs.c_freq);
		/* XXX only if ic->ic_curchan->ic_ieee == rxs.c_ieee */
		tap->wr_chan_flags = htole16(ic->ic_curchan->ic_flags);
		tap->wr_dbm_antsignal = (int8_t)rssi;
		tap->wr_dbm_antnoise = (int8_t)sc->sc_noise;
		tap->wr_tsft = phy_info->system_timestamp;
		switch (phy_info->rate) {
		/* CCK rates. */
		case  10: tap->wr_rate =   2; break;
		case  20: tap->wr_rate =   4; break;
		case  55: tap->wr_rate =  11; break;
		case 110: tap->wr_rate =  22; break;
		/* OFDM rates. */
		case 0xd: tap->wr_rate =  12; break;
		case 0xf: tap->wr_rate =  18; break;
		case 0x5: tap->wr_rate =  24; break;
		case 0x7: tap->wr_rate =  36; break;
		case 0x9: tap->wr_rate =  48; break;
		case 0xb: tap->wr_rate =  72; break;
		case 0x1: tap->wr_rate =  96; break;
		case 0x3: tap->wr_rate = 108; break;
		/* Unknown rate: should not happen. */
		default:  tap->wr_rate =   0;
		}
	}

	return true;
}

static bool
iwx_rx_mpdu_mq(struct iwx_softc *sc, struct mbuf *m, uint32_t offset,
    bool stolen)
{
	struct ieee80211com *ic = &sc->sc_ic;
	struct ieee80211vap *vap = TAILQ_FIRST(&ic->ic_vaps);
	struct ieee80211_frame *wh;
	struct ieee80211_rx_stats rxs;
	struct iwx_rx_mpdu_desc *desc;
	struct iwx_rx_packet *pkt;
	int rssi;
	uint32_t hdrlen, len, rate_n_flags;
	uint16_t phy_info;
	uint8_t channel;

	pkt = mtodo(m, offset);
	desc = (void *)pkt->data;

	if (!(desc->status & htole16(IWX_RX_MPDU_RES_STATUS_CRC_OK)) ||
	    !(desc->status & htole16(IWX_RX_MPDU_RES_STATUS_OVERRUN_OK))) {
		IWX_DPRINTF(sc, IWX_DEBUG_RECV,
		    "Bad CRC or FIFO: 0x%08X.\n", desc->status);
		return false;
	}

	channel = desc->v1.channel;
	len = le16toh(desc->mpdu_len);
	phy_info = le16toh(desc->phy_info);
	rate_n_flags = desc->v1.rate_n_flags;

	wh = mtodo(m, sizeof(*desc));
	m->m_data = pkt->data + sizeof(*desc);
	m->m_pkthdr.len = m->m_len = len;
	m->m_len = len;

	/* Account for padding following the frame header. */
	if ((desc->mac_flags2 & IWX_RX_MPDU_MFLG2_PAD)) {
		hdrlen = ieee80211_anyhdrsize(wh);
		memmove(mtodo(m, 2), mtodo(m, 0), hdrlen);
		m->m_data = mtodo(m, 2);
		wh = mtod(m, struct ieee80211_frame *);
	}

	/* Map it to relative value */
	rssi = iwx_rxmq_get_signal_strength(sc, desc);
	rssi = rssi - sc->sc_noise;

	/* replenish ring for the buffer we're going to feed to the sharks */
	if (!stolen && iwx_rx_addbuf(sc, IWX_RBUF_SIZE, sc->rxq.cur) != 0) {
		device_printf(sc->sc_dev, "%s: unable to add more buffers\n",
		    __func__);
		return false;
	}

	IWX_DPRINTF(sc, IWX_DEBUG_RECV,
	    "%s: rssi=%d, noise=%d\n", __func__, rssi, sc->sc_noise);

	/*
	 * Populate an RX state struct with the provided information.
	 */
	bzero(&rxs, sizeof(rxs));
	rxs.r_flags |= IEEE80211_R_IEEE | IEEE80211_R_FREQ;
	rxs.r_flags |= IEEE80211_R_NF | IEEE80211_R_RSSI;
	rxs.c_ieee = channel;
	rxs.c_freq = ieee80211_ieee2mhz(rxs.c_ieee,
	    channel <= 14 ? IEEE80211_CHAN_2GHZ : IEEE80211_CHAN_5GHZ);

	/* rssi is in 1/2db units */
	rxs.c_rssi = rssi * 2;
	rxs.c_nf = sc->sc_noise;
	if (ieee80211_add_rx_params(m, &rxs) == 0)
		return false;

	if (ieee80211_radiotap_active_vap(vap)) {
		struct iwx_rx_radiotap_header *tap = &sc->sc_rxtap;

		tap->wr_flags = 0;
		if ((phy_info & IWX_RX_MPDU_PHY_SHORT_PREAMBLE) != 0)
			tap->wr_flags |= IEEE80211_RADIOTAP_F_SHORTPRE;
		tap->wr_chan_freq = htole16(rxs.c_freq);
		/* XXX only if ic->ic_curchan->ic_ieee == rxs.c_ieee */
		tap->wr_chan_flags = htole16(ic->ic_curchan->ic_flags);
		tap->wr_dbm_antsignal = (int8_t)rssi;
		tap->wr_dbm_antnoise = (int8_t)sc->sc_noise;
		tap->wr_tsft = desc->v1.gp2_on_air_rise;
		switch ((rate_n_flags & 0xff)) {
		/* CCK rates. */
		case  10: tap->wr_rate =   2; break;
		case  20: tap->wr_rate =   4; break;
		case  55: tap->wr_rate =  11; break;
		case 110: tap->wr_rate =  22; break;
		/* OFDM rates. */
		case 0xd: tap->wr_rate =  12; break;
		case 0xf: tap->wr_rate =  18; break;
		case 0x5: tap->wr_rate =  24; break;
		case 0x7: tap->wr_rate =  36; break;
		case 0x9: tap->wr_rate =  48; break;
		case 0xb: tap->wr_rate =  72; break;
		case 0x1: tap->wr_rate =  96; break;
		case 0x3: tap->wr_rate = 108; break;
		/* Unknown rate: should not happen. */
		default:  tap->wr_rate =   0;
		}
	}

	return true;
}

static bool
iwx_rx_mpdu(struct iwx_softc *sc, struct mbuf *m, uint32_t offset,
    bool stolen)
{
#if (__FreeBSD_version >= 1300077)
	struct epoch_tracker et;
#endif
	struct ieee80211com *ic;
	struct ieee80211_frame *wh;
	struct ieee80211_node *ni;
	bool ret;

	ic = &sc->sc_ic;

	ret = sc->cfg->mqrx_supported ?
	    iwx_rx_mpdu_mq(sc, m, offset, stolen) :
	    iwx_rx_rx_mpdu(sc, m, offset, stolen);
	if (!ret) {
		counter_u64_add(ic->ic_ierrors, 1);
		return (ret);
	}

	wh = mtod(m, struct ieee80211_frame *);
	ni = ieee80211_find_rxnode(ic, (struct ieee80211_frame_min *)wh);

	IWX_UNLOCK(sc);

#if (__FreeBSD_version >= 1300077)
	NET_EPOCH_ENTER(et);
#endif
	if (ni != NULL) {
		IWX_DPRINTF(sc, IWX_DEBUG_RECV, "input m %p\n", m);
		ieee80211_input_mimo(ni, m);
		ieee80211_free_node(ni);
	} else {
		IWX_DPRINTF(sc, IWX_DEBUG_RECV, "inputall m %p\n", m);
		ieee80211_input_mimo_all(ic, m);
	}
#if (__FreeBSD_version >= 1300077)
	NET_EPOCH_EXIT(et);
#endif

	IWX_LOCK(sc);

	return true;
}

static int
iwx_rx_tx_cmd_single(struct iwx_softc *sc, struct iwx_rx_packet *pkt,
	struct iwx_node *in)
{
	struct iwx_tx_resp *tx_resp = (void *)pkt->data;
	struct ieee80211_ratectl_tx_status *txs = &sc->sc_txs;
	struct ieee80211_node *ni = &in->in_ni;
	struct ieee80211vap *vap = ni->ni_vap;
	int status = le16toh(tx_resp->status.status) & IWX_TX_STATUS_MSK;
	int new_rate, cur_rate = vap->iv_bss->ni_txrate;
	boolean_t rate_matched;
	uint8_t tx_resp_rate;

	KASSERT(tx_resp->frame_count == 1, ("too many frames"));

	/* Update rate control statistics. */
	IWX_DPRINTF(sc, IWX_DEBUG_XMIT, "%s: status=0x%04x, seq=%d, fc=%d, btc=%d, frts=%d, ff=%d, irate=%08x, wmt=%d\n",
	    __func__,
	    (int) le16toh(tx_resp->status.status),
	    (int) le16toh(tx_resp->status.sequence),
	    tx_resp->frame_count,
	    tx_resp->bt_kill_count,
	    tx_resp->failure_rts,
	    tx_resp->failure_frame,
	    le32toh(tx_resp->initial_rate),
	    (int) le16toh(tx_resp->wireless_media_time));

	tx_resp_rate = iwx_rate_from_ucode_rate(le32toh(tx_resp->initial_rate));

	/* For rate control, ignore frames sent at different initial rate */
	rate_matched = (tx_resp_rate != 0 && tx_resp_rate == cur_rate);

	if (tx_resp_rate != 0 && cur_rate != 0 && !rate_matched) {
		IWX_DPRINTF(sc, IWX_DEBUG_TXRATE,
		    "tx_resp_rate doesn't match ni_txrate (tx_resp_rate=%u "
		    "ni_txrate=%d)\n", tx_resp_rate, cur_rate);
	}

	txs->flags = IEEE80211_RATECTL_STATUS_SHORT_RETRY |
		     IEEE80211_RATECTL_STATUS_LONG_RETRY;
	txs->short_retries = tx_resp->failure_rts;
	txs->long_retries = tx_resp->failure_frame;
	if (status != IWX_TX_STATUS_SUCCESS &&
	    status != IWX_TX_STATUS_DIRECT_DONE) {
		switch (status) {
		case IWX_TX_STATUS_FAIL_SHORT_LIMIT:
			txs->status = IEEE80211_RATECTL_TX_FAIL_SHORT;
			break;
		case IWX_TX_STATUS_FAIL_LONG_LIMIT:
			txs->status = IEEE80211_RATECTL_TX_FAIL_LONG;
			break;
		case IWX_TX_STATUS_FAIL_LIFE_EXPIRE:
			txs->status = IEEE80211_RATECTL_TX_FAIL_EXPIRED;
			break;
		default:
			txs->status = IEEE80211_RATECTL_TX_FAIL_UNSPECIFIED;
			break;
		}
	} else {
		txs->status = IEEE80211_RATECTL_TX_SUCCESS;
	}

	if (rate_matched) {
		ieee80211_ratectl_tx_complete(ni, txs);

		int rix = ieee80211_ratectl_rate(vap->iv_bss, NULL, 0);
		new_rate = vap->iv_bss->ni_txrate;
		if (new_rate != 0 && new_rate != cur_rate) {
			struct iwx_node *in = IWX_NODE(vap->iv_bss);
			iwx_setrates(sc, in, rix);
//			iwx_send_lq_cmd(sc, &in->in_lq, FALSE);
		}
 	}

	return (txs->status != IEEE80211_RATECTL_TX_SUCCESS);
}

static void
iwx_rx_tx_cmd(struct iwx_softc *sc, struct iwx_rx_packet *pkt)
{
	struct iwx_cmd_header *cmd_hdr;
	struct iwx_tx_ring *ring;
	struct iwx_tx_data *txd;
	struct iwx_node *in;
	struct mbuf *m;
	int idx, qid, qmsk, status;

	cmd_hdr = &pkt->hdr;
	idx = cmd_hdr->idx;
	qid = cmd_hdr->qid;

	ring = &sc->txq[qid];
	txd = &ring->data[idx];
	in = txd->in;
	m = txd->m;

	KASSERT(txd->done == 0, ("txd not done"));
	KASSERT(txd->in != NULL, ("txd without node"));
	KASSERT(txd->m != NULL, ("txd without mbuf"));

	sc->sc_tx_timer = 0;

	status = iwx_rx_tx_cmd_single(sc, pkt, in);

	/* Unmap and free mbuf. */
	bus_dmamap_sync(ring->data_dmat, txd->map, BUS_DMASYNC_POSTWRITE);
	bus_dmamap_unload(ring->data_dmat, txd->map);

	IWX_DPRINTF(sc, IWX_DEBUG_XMIT,
	    "free txd %p, in %p\n", txd, txd->in);
	txd->done = 1;
	txd->m = NULL;
	txd->in = NULL;

	ieee80211_tx_complete(&in->in_ni, m, status);

	qmsk = 1 << qid;
	if (--ring->queued < IWX_TX_RING_LOMARK && (sc->qfullmsk & qmsk) != 0) {
		sc->qfullmsk &= ~qmsk;
		if (sc->qfullmsk == 0)
			iwx_start(sc);
	}
}

/*
 * transmit side
 */

/*
 * Process a "command done" firmware notification.  This is where we wakeup
 * processes waiting for a synchronous command completion.
 * from if_iwn
 */
/* OpenBSD changed the amount and type of parameters */
static void
iwx_cmd_done(struct iwx_softc *sc, struct iwx_rx_packet *pkt, int code)
{
	struct iwx_tx_ring *ring = &sc->txq[IWX_DQA_CMD_QUEUE];
	struct iwx_tx_data *data;

	if (pkt->hdr.qid != IWX_DQA_CMD_QUEUE) {
		return;	/* Not a command ack. */
	}

	/* XXX wide commands? */
	IWX_DPRINTF(sc, IWX_DEBUG_CMD,
	    "cmd notification type 0x%x qid %d idx %d\n",
	    pkt->hdr.code, pkt->hdr.qid, pkt->hdr.idx);

	data = &ring->data[pkt->hdr.idx];

	/* If the command was mapped in an mbuf, free it. */
	if (data->m != NULL) {
		bus_dmamap_sync(ring->data_dmat, data->map,
		    BUS_DMASYNC_POSTWRITE);
		bus_dmamap_unload(ring->data_dmat, data->map);
		m_freem(data->m);
		data->m = NULL;
	}
	wakeup(&ring->desc[pkt->hdr.idx]);

	if (((pkt->hdr.idx + ring->queued) % IWX_TX_RING_COUNT) != ring->cur) {
		if (code != IWX_NVM_ACCESS_CMD) {
			device_printf(sc->sc_dev,
		    "%s: Some HCMDs skipped?: idx=%d queued=%d cur=%d\n",
		    __func__, pkt->hdr.idx, ring->queued, ring->cur);
		/* XXX call iwm_force_nmi() */
		}
		else {
			KASSERT(ring->queued > 0, ("ring->queued is empty?"));
			ring->queued--;
		}
		if (ring->queued == 0)
		iwx_pcie_clear_cmd_in_flight(sc);
	}
}

static int
iwx_tx_rateidx_global_lookup(struct iwx_softc *sc, uint8_t rate)
{
	int i;

	for (i = 0; i < nitems(iwx_rates); i++) {
		if (iwx_rates[i].rate == rate)
			return (i);
	}
	/* XXX error? */
	IWX_DPRINTF(sc, IWX_DEBUG_XMIT | IWX_DEBUG_TXRATE,
	    "%s: couldn't find an entry for rate=%d\n",
	    __func__,
	    rate);
	return (0);
}

/*
 * Fill in the rate related information for a transmit command.
 */
static const struct iwx_rate *
iwx_tx_fill_cmd(struct iwx_softc *sc, struct iwx_node *in,
	struct mbuf *m, struct iwx_tx_cmd_gen2 *tx)
{
	struct ieee80211_node *ni = &in->in_ni;
	struct ieee80211_frame *wh;
	const struct ieee80211_txparam *tp = ni->ni_txparms;
	const struct iwx_rate *rinfo;
	int type;
	int ridx, rate_flags;
	uint32_t flags;

	wh = mtod(m, struct ieee80211_frame *);
	type = wh->i_fc[0] & IEEE80211_FC0_TYPE_MASK;

	if (type == IEEE80211_FC0_TYPE_MGT ||
	    type == IEEE80211_FC0_TYPE_CTL ||
	    (m->m_flags & M_EAPOL) != 0) {
		ridx = iwx_tx_rateidx_global_lookup(sc, tp->mgmtrate);
		IWX_DPRINTF(sc, IWX_DEBUG_TXRATE,
		    "%s: MGT (%d)\n", __func__, tp->mgmtrate);
	} else if (IEEE80211_IS_MULTICAST(wh->i_addr1)) {
		ridx = iwx_tx_rateidx_global_lookup(sc, tp->mcastrate);
		IWX_DPRINTF(sc, IWX_DEBUG_TXRATE,
		    "%s: MCAST (%d)\n", __func__, tp->mcastrate);
	} else if (tp->ucastrate != IEEE80211_FIXED_RATE_NONE) {
		ridx = iwx_tx_rateidx_global_lookup(sc, tp->ucastrate);
		IWX_DPRINTF(sc, IWX_DEBUG_TXRATE,
		    "%s: FIXED_RATE (%d)\n", __func__, tp->ucastrate);
	} else {
		/* for data frames, use RS table */
		IWX_DPRINTF(sc, IWX_DEBUG_TXRATE, "%s: DATA\n", __func__);
		ridx = iwx_rate2ridx(sc, ni->ni_txrate);
		if (ridx == -1)
			ridx = 0;

		/* This is the index into the programmed table */
/* openbsd drops this */
//	tx->initial_rate_index = 0;
//		tx->tx_flags |= htole32(IWX_TX_CMD_FLG_STA_RATE);
	}

	IWX_DPRINTF(sc, IWX_DEBUG_XMIT | IWX_DEBUG_TXRATE,
	    "%s: frame type=%d txrate %d\n",
	        __func__, type, iwx_rates[ridx].rate);

	flags = (IWX_TX_FLAGS_CMD_RATE | IWX_TX_FLAGS_ENCRYPT_DIS);
#if 0
	if ((ic->ic_flags & IEEE80211_F_RSNON) &&
	    ni->ni_rsn_supp_state == RSNA_SUPP_PTKNEGOTIATING)
		flags |= IWX_TX_FLAGS_HIGH_PRI;
#endif
	tx->flags = htole32(flags);

	rinfo = &iwx_rates[ridx];

	IWX_DPRINTF(sc, IWX_DEBUG_TXRATE, "%s: ridx=%d; rate=%d, CCK=%d\n",
	    __func__, ridx,
	    rinfo->rate,
	    !! (IWX_RIDX_IS_CCK(ridx))
	    );

	rate_flags = IWX_RATE_MCS_ANT_A_MSK;
	if (IWX_RIDX_IS_CCK(ridx))
		rate_flags |= IWX_RATE_MCS_CCK_MSK;
	tx->rate_n_flags = htole32(rate_flags | rinfo->plcp);

	return rinfo;
}

/*
 * necessary only for block ack mode
 */
/* source: OpenBSD */
void
iwx_tx_update_byte_tbl(struct iwx_tx_ring *txq, uint16_t byte_cnt,
    uint16_t num_tbs)
{
	uint8_t filled_tfd_size, num_fetch_chunks;
	uint16_t len = byte_cnt;
	uint16_t bc_ent;
	struct iwx_agn_scd_bc_tbl *scd_bc_tbl = txq->bc_tbl.vaddr;

	filled_tfd_size = offsetof(struct iwx_tfh_tfd, tbs) +
			  num_tbs * sizeof(struct iwx_tfh_tb);
	/*
	 * filled_tfd_size contains the number of filled bytes in the TFD.
	 * Dividing it by 64 will give the number of chunks to fetch
	 * to SRAM- 0 for one chunk, 1 for 2 and so on.
	 * If, for example, TFD contains only 3 TBs then 32 bytes
	 * of the TFD are used, and only one chunk of 64 bytes should
	 * be fetched
	 */
	num_fetch_chunks = howmany(filled_tfd_size, 64) - 1;

	/* Before AX210, the HW expects DW */
	len = howmany(len, 4);
	bc_ent = htole16(len | (num_fetch_chunks << 12));
	scd_bc_tbl->tfd_offset[txq->cur] = bc_ent;
}

static int
iwx_tx(struct iwx_softc *sc, struct mbuf *m, struct ieee80211_node *ni, int ac)
{
	struct ieee80211com *ic = &sc->sc_ic;
	struct ieee80211vap *vap = TAILQ_FIRST(&ic->ic_vaps);
	struct iwx_node *in = IWX_NODE(ni);
	struct iwx_tx_ring *ring;
	struct iwx_tx_data *data;
	struct iwx_tfh_tfd *desc;
	struct iwx_device_cmd *cmd;
	struct iwx_tx_cmd_gen2 *tx;
	struct ieee80211_frame *wh;
	struct ieee80211_key *k = NULL;
	struct mbuf *m1;
	const struct iwx_rate *rinfo;
	uint64_t paddr;
	u_int hdrlen;
	bus_dma_segment_t *seg, segs[IWX_MAX_SCATTER];
	int nsegs;
//	uint8_t tid, type;
	uint16_t num_tbs;
	uint8_t type;
	uint8_t tid;
	int i, totlen, error, pad;

	wh = mtod(m, struct ieee80211_frame *);
	hdrlen = ieee80211_anyhdrsize(wh);
	type = wh->i_fc[0] & IEEE80211_FC0_TYPE_MASK;
	tid = 0;
	ring = &sc->txq[IWX_DQA_MIN_MGMT_QUEUE + ac];
	desc = &ring->desc[ring->cur];
	data = &ring->data[ring->cur];

	/* Fill out iwm_tx_cmd to send to the firmware */
	cmd = &ring->cmd[ring->cur];
	cmd->hdr.code = IWX_TX_CMD;
	cmd->hdr.flags = 0;
	cmd->hdr.qid = ring->qid;
	cmd->hdr.idx = ring->cur;

	tx = (void *)cmd->data;
	memset(tx, 0, sizeof(*tx));

	rinfo = iwx_tx_fill_cmd(sc, in, m, tx);

	/* Encrypt the frame if need be. */
	if (wh->i_fc[1] & IEEE80211_FC1_PROTECTED) {
		/* Retrieve key for TX && do software encryption. */
		k = ieee80211_crypto_encap(ni, m);
		if (k == NULL) {
			m_freem(m);
			return (ENOBUFS);
		}
		/* 802.11 header may have moved. */
		wh = mtod(m, struct ieee80211_frame *);
	}

	if (ieee80211_radiotap_active_vap(vap)) {
		struct iwx_tx_radiotap_header *tap = &sc->sc_txtap;

		tap->wt_flags = 0;
		tap->wt_chan_freq = htole16(ni->ni_chan->ic_freq);
		tap->wt_chan_flags = htole16(ni->ni_chan->ic_flags);
		tap->wt_rate = rinfo->rate;
		if (k != NULL)
			tap->wt_flags |= IEEE80211_RADIOTAP_F_WEP;
		ieee80211_radiotap_tx(vap, m);
	}
	totlen = m->m_pkthdr.len;

	if (hdrlen & 3) {
		/* First segment length must be a multiple of 4. */
		tx->offload_assist |= htole16(1 << IWX_TX_CMD_OFFLD_PAD);
		pad = 4 - (hdrlen & 3);
	} else {
		tx->offload_assist = 0;
		pad = 0;
	}

	tx->len = htole16(totlen);

	/* Copy 802.11 header in TX command. */
	memcpy((uint8_t *)tx + sizeof(*tx), wh, hdrlen);

	/* Trim 802.11 header. */
	m_adj(m, hdrlen);

	error = bus_dmamap_load_mbuf_sg(ring->data_dmat, data->map, m,
	    segs, &nsegs, BUS_DMA_NOWAIT);
	if (error != 0) {
		if (error != EFBIG) {
			device_printf(sc->sc_dev, "can't map mbuf (error %d)\n",
			    error);
			m_freem(m);
			return error;
		}
		/* Too many DMA segments, linearize mbuf. */
		m1 = m_collapse(m, M_NOWAIT, IWX_MAX_SCATTER - 2);
		if (m1 == NULL) {
			device_printf(sc->sc_dev,
			    "%s: could not defrag mbuf\n", __func__);
			m_freem(m);
			return (ENOBUFS);
		}
		m = m1;

		error = bus_dmamap_load_mbuf_sg(ring->data_dmat, data->map, m,
		    segs, &nsegs, BUS_DMA_NOWAIT);
		if (error != 0) {
			device_printf(sc->sc_dev, "can't map mbuf (error %d)\n",
			    error);
			m_freem(m);
			return error;
		}
	}
	data->m = m;
	data->in = in;
	data->done = 0;

	IWX_DPRINTF(sc, IWX_DEBUG_XMIT,
	    "sending txd %p, in %p\n", data, data->in);
	KASSERT(data->in != NULL, ("node is NULL"));

	IWX_DPRINTF(sc, IWX_DEBUG_XMIT,
//	    "sending data: qid=%d idx=%d len=%d nsegs=%d txflags=0x%08x rate_n_flags=0x%08x rateidx=%u\n",
	    "sending data: qid=%d idx=%d len=%d nsegs=%d rate_n_flags=0x%08x\n",
	    ring->qid, ring->cur, totlen, nsegs,
//	    le32toh(tx->tx_flags),
	    le32toh(tx->rate_n_flags)
//	    tx->initial_rate_index
	    );

	/* Fill TX descriptor. */
	memset(desc, 0, sizeof(*desc));
	num_tbs = 2 + nsegs;
	desc->num_tbs = htole16(num_tbs);

	desc->tbs[0].tb_len = htole16(IWX_FIRST_TB_SIZE);
	paddr = htole64(data->cmd_paddr);
	memcpy(&desc->tbs[0].addr, &paddr, sizeof(paddr));
#ifdef tbd
	if (data->cmd_paddr >> 32 != (data->cmd_paddr + le32toh(desc->tbs[0].tb_len)) >> 32)
		IWX_DPRINTF(sc, IWX_DEBUG_XMIT, "%s: TB0 crosses 32bit boundary\n", __func__);
#endif
	desc->tbs[1].tb_len = htole16(sizeof(struct iwx_cmd_header) +
	    sizeof(*tx) + hdrlen + pad - IWX_FIRST_TB_SIZE);
	paddr = htole64(data->cmd_paddr + IWX_FIRST_TB_SIZE);
	memcpy(&desc->tbs[1].addr, &paddr, sizeof(paddr));

#ifdef tbd
	if (data->cmd_paddr >> 32 != (data->cmd_paddr + le32toh(desc->tbs[1].tb_len)) >> 32)
		IWX_DPRINTF(sc, IWX_DEBUG_XMIT, "%s: TB1 crosses 32bit boundary\n", __func__);
#endif

	/* Other DMA segments are for data payload. */
	for (i = 0; i < nsegs; i++) {
		seg = &segs[i];
		desc->tbs[i + 2].tb_len = htole16(seg->ds_len);
		paddr = htole64(seg->ds_addr);
		memcpy(&desc->tbs[i + 2].addr, &paddr, sizeof(paddr));
#ifdef tbd
		if (data->cmd_paddr >> 32 != (data->cmd_paddr + le32toh(desc->tbs[i + 2].tb_len)) >> 32)
			IWX_DPRINTF(sc, IWX_DEBUG_XMIT, "%s: TB%d crosses 32bit boundary\n", __func__, i + 2);
#endif
	}

	bus_dmamap_sync(ring->data_dmat, data->map,
	    BUS_DMASYNC_PREWRITE);
	bus_dmamap_sync(ring->cmd_dma.tag, ring->cmd_dma.map,
	    BUS_DMASYNC_PREWRITE);
	bus_dmamap_sync(ring->desc_dma.tag, ring->desc_dma.map,
	    BUS_DMASYNC_PREWRITE);

#if 0
	iwx_tx_update_byte_tbl(ring, totlen, num_tbs);
#endif

	/* Kick TX ring. */
	ring->cur = (ring->cur + 1) % IWX_TX_RING_COUNT;
	IWX_WRITE(sc, IWX_HBUS_TARG_WRPTR, ring->qid << 16 | ring->cur);

	/* Mark TX ring as full if we reach a certain threshold. */
	if (++ring->queued > IWX_TX_RING_HIMARK) {
		sc->qfullmsk |= 1 << ring->qid;
	}

	return 0;
}

static int
iwx_raw_xmit(struct ieee80211_node *ni, struct mbuf *m,
    const struct ieee80211_bpf_params *params)
{
	struct ieee80211com *ic = ni->ni_ic;
	struct iwx_softc *sc = ic->ic_softc;
	int error = 0;

	IWX_DPRINTF(sc, IWX_DEBUG_XMIT,
	    "->%s begin\n", __func__);

	if ((sc->sc_flags & IWX_FLAG_HW_INITED) == 0) {
		m_freem(m);
		IWX_DPRINTF(sc, IWX_DEBUG_XMIT,
		    "<-%s not RUNNING\n", __func__);
		return (ENETDOWN);
        }

	IWX_LOCK(sc);
	/* XXX fix this */
        if (params == NULL) {
		error = iwx_tx(sc, m, ni, 0);
	} else {
		error = iwx_tx(sc, m, ni, 0);
	}
	if (sc->sc_tx_timer == 0)
		callout_reset(&sc->sc_watchdog_to, hz, iwx_watchdog, sc);
	sc->sc_tx_timer = 5;
	IWX_UNLOCK(sc);

        return (error);
}

/*
 * mvm/tx.c
 */

/*
 * Note that there are transports that buffer frames before they reach
 * the firmware. This means that after flush_tx_path is called, the
 * queue might not be empty. The race-free way to handle this is to:
 * 1) set the station as draining
 * 2) flush the Tx path
 * 3) wait for the transport queues to be empty
 */
int
iwx_flush_tx_path(struct iwx_softc *sc, uint32_t flags)
{
	int ret;
	struct iwx_tx_path_flush_cmd flush_cmd = {
		.sta_id = htole32(IWX_STATION_ID),
		.tid_mask = htole16(0xffff),
	};

	ret = iwx_send_cmd_pdu(sc, IWX_TXPATH_FLUSH, flags,
	    sizeof(flush_cmd), &flush_cmd);
	if (ret)
                device_printf(sc->sc_dev,
		    "Flushing tx queue failed: %d\n", ret);
	return ret;
}

/*
 * BEGIN mvm/quota.c
 */

static int
iwx_update_quotas(struct iwx_softc *sc, struct iwx_vap *ivp)
{
	struct iwx_time_quota_cmd cmd;
	int i, idx, ret, num_active_macs, quota, quota_rem;
	int colors[IWX_MAX_BINDINGS] = { -1, -1, -1, -1, };
	int n_ifs[IWX_MAX_BINDINGS] = {0, };
	uint16_t id;

	memset(&cmd, 0, sizeof(cmd));

	/* currently, PHY ID == binding ID */
	if (ivp) {
		id = ivp->phy_ctxt->id;
		KASSERT(id < IWX_MAX_BINDINGS, ("invalid id"));
		colors[id] = ivp->phy_ctxt->color;

		if (1)
			n_ifs[id] = 1;
	}

	/*
	 * The FW's scheduling session consists of
	 * IWM_MAX_QUOTA fragments. Divide these fragments
	 * equally between all the bindings that require quota
	 */
	num_active_macs = 0;
	for (i = 0; i < IWX_MAX_BINDINGS; i++) {
		cmd.quotas[i].id_and_color = htole32(IWX_FW_CTXT_INVALID);
		num_active_macs += n_ifs[i];
	}

	quota = 0;
	quota_rem = 0;
	if (num_active_macs) {
		quota = IWX_MAX_QUOTA / num_active_macs;
		quota_rem = IWX_MAX_QUOTA % num_active_macs;
	}

	for (idx = 0, i = 0; i < IWX_MAX_BINDINGS; i++) {
		if (colors[i] < 0)
			continue;

		cmd.quotas[idx].id_and_color =
			htole32(IWX_FW_CMD_ID_AND_COLOR(i, colors[i]));

		if (n_ifs[i] <= 0) {
			cmd.quotas[idx].quota = htole32(0);
			cmd.quotas[idx].max_duration = htole32(0);
		} else {
			cmd.quotas[idx].quota = htole32(quota * n_ifs[i]);
			cmd.quotas[idx].max_duration = htole32(0);
		}
		idx++;
	}

	/* Give the remainder of the session to the first binding */
	cmd.quotas[0].quota = htole32(le32toh(cmd.quotas[0].quota) + quota_rem);

	ret = iwx_send_cmd_pdu(sc, IWX_TIME_QUOTA_CMD, IWX_CMD_SYNC,
	    sizeof(cmd), &cmd);
	if (ret)
		device_printf(sc->sc_dev,
		    "%s: Failed to send quota: %d\n", __func__, ret);
	return ret;
}

/*
 * END mvm/quota.c
 */

/*
 * ieee80211 routines
 */

/*
 * Change to AUTH state in 80211 state machine.  Roughly matches what
 * Linux does in bss_info_changed().
 */
static int
iwx_auth(struct ieee80211vap *vap, struct iwx_softc *sc)
{
	struct ieee80211_node *ni;
	struct iwx_node *in;
	struct iwx_vap *iv = IWX_VAP(vap);
	uint32_t duration;
	int error;
	int generation = sc->sc_generation;

	/*
	 * XXX i have a feeling that the vap node is being
	 * freed from underneath us. Grr.
	 */
	ni = ieee80211_ref_node(vap->iv_bss);
	in = IWX_NODE(ni);
	IWX_DPRINTF(sc, IWX_DEBUG_RESET | IWX_DEBUG_STATE,
	    "%s: called; vap=%p, bss ni=%p\n",
	    __func__,
	    vap,
	    ni);
	IWX_DPRINTF(sc, IWX_DEBUG_STATE, "%s: Current node bssid: %s\n",
	    __func__, ether_sprintf(ni->ni_bssid));

	in->in_assoc = 0;
	iv->iv_auth = 1;

	/*
	 * Firmware bug - it'll crash if the beacon interval is less
	 * than 16. We can't avoid connecting at all, so refuse the
	 * station state change, this will cause net80211 to abandon
	 * attempts to connect to this AP, and eventually wpa_s will
	 * blacklist the AP...
	 */
	if (ni->ni_intval < 16) {
		device_printf(sc->sc_dev,
		    "AP %s beacon interval is %d, refusing due to firmware bug!\n",
		    ether_sprintf(ni->ni_bssid), ni->ni_intval);
		error = EINVAL;
		goto out;
	}

	error = iwx_allow_mcast(vap, sc);
	if (error) {
		device_printf(sc->sc_dev,
		    "%s: failed to set multicast\n", __func__);
		goto out;
	}

	/*
	 * This is where it deviates from what Linux does.
	 *
	 * Linux iwlwifi doesn't reset the nic each time, nor does it
	 * call ctxt_add() here.  Instead, it adds it during vap creation,
	 * and always does a mac_ctx_changed().
	 *
	 * The openbsd port doesn't attempt to do that - it reset things
	 * at odd states and does the add here.
	 *
	 * So, until the state handling is fixed (ie, we never reset
	 * the NIC except for a firmware failure, which should drag
	 * the NIC back to IDLE, re-setup and re-add all the mac/phy
	 * contexts that are required), let's do a dirty hack here.
	 */
	if (iv->is_uploaded) {
		if ((error = iwx_mac_ctxt_changed(sc, vap)) != 0) {
			device_printf(sc->sc_dev,
			    "%s: failed to update MAC\n", __func__);
			goto out;
		}
	} else {
		if ((error = iwx_mac_ctxt_add(sc, vap)) != 0) {
			device_printf(sc->sc_dev,
			    "%s: failed to add MAC\n", __func__);
			goto out;
		}
	}
	sc->sc_firmware_state = 1;

	if ((error = iwx_phy_ctxt_changed(sc, &sc->sc_phyctxt[0],
	    in->in_ni.ni_chan, 1, 1)) != 0) {
		device_printf(sc->sc_dev,
		    "%s: failed update phy ctxt\n", __func__);
		goto out;
	}
	iv->phy_ctxt = &sc->sc_phyctxt[0];

	if ((error = iwx_binding_add_vif(sc, iv)) != 0) {
		device_printf(sc->sc_dev,
		    "%s: binding update cmd\n", __func__);
		goto out;
	}
	sc->sc_firmware_state = 2;
	/*
	 * Authentication becomes unreliable when powersaving is left enabled
	 * here. Powersaving will be activated again when association has
	 * finished or is aborted.
	 */
	iv->ps_disabled = TRUE;
	error = iwx_power_update_mac(sc);
	iv->ps_disabled = FALSE;
	if (error != 0) {
		device_printf(sc->sc_dev,
		    "%s: failed to update power management\n",
		    __func__);
		goto out;
	}
	if ((error = iwx_add_sta(sc, in)) != 0) {
		device_printf(sc->sc_dev,
		    "%s: failed to add sta\n", __func__);
		goto out;
	}
	sc->sc_firmware_state = 3;

	// openbsd has code that calls iwx_enable_txq()

	/* ported from openbsd */
	error = iwx_enable_data_tx_queues(sc);
	if (error)
		goto rm_sta;

	error = iwx_clear_statistics(sc);
	if (error)
		goto rm_sta;

	/*
	 * Prevent the FW from wandering off channel during association
	 * by "protecting" the session with a time event.
	 */
	/* XXX duration is in units of TU, not MS */
	duration = IWX_TE_SESSION_PROTECTION_MAX_TIME_MS;
	iwx_protect_session(sc, iv, duration, 500 /* XXX magic number */, TRUE);

	error = 0;

rm_sta:
	if (generation == sc->sc_generation) {
#ifdef tbd
		iwx_rm_sta_cmd(sc, in);
#endif
//		sc->sc_flags &= ~IWX_FLAG_STA_ACTIVE;
		sc->sc_flags &= ~0x20;
	}
out:
	if (error != 0)
		iv->iv_auth = 0;
	ieee80211_free_node(ni);
	return (error);
}

static struct ieee80211_node *
iwx_node_alloc(struct ieee80211vap *vap, const uint8_t mac[IEEE80211_ADDR_LEN])
{
	return malloc(sizeof (struct iwx_node), M_80211_NODE,
	    M_NOWAIT | M_ZERO);
}

static uint8_t
iwx_rate_from_ucode_rate(uint32_t rate_n_flags)
{
	uint8_t plcp = rate_n_flags & 0xff;
	int i;

	for (i = 0; i <= IWX_RIDX_MAX; i++) {
		if (iwx_rates[i].plcp == plcp)
			return iwx_rates[i].rate;
	}
	return 0;
}

uint8_t
iwx_ridx2rate(struct ieee80211_rateset *rs, int ridx)
{
	int i;
	uint8_t rval;

	for (i = 0; i < rs->rs_nrates; i++) {
		rval = (rs->rs_rates[i] & IEEE80211_RATE_VAL);
		if (rval == iwx_rates[ridx].rate)
			return rs->rs_rates[i];
	}

	return 0;
}

static int
iwx_rate2ridx(struct iwx_softc *sc, uint8_t rate)
{
	int i;

	for (i = 0; i <= IWX_RIDX_MAX; i++) {
		if (iwx_rates[i].rate == rate)
			return i;
	}

	device_printf(sc->sc_dev,
	    "%s: WARNING: device rate for %u not found!\n",
	    __func__, rate);

	return -1;
}

static void
iwx_setrates(struct iwx_softc *sc, struct iwx_node *in, int rix)
{
	struct ieee80211_node *ni = &in->in_ni;
//	struct iwx_lq_cmd *lq = &in->in_lq;
	struct ieee80211_rateset *rs = &ni->ni_rates;
	int nrates = rs->rs_nrates;
	int i, ridx, tab = 0;
//	int txant = 0;

	KASSERT(rix >= 0 && rix < nrates, ("invalid rix"));

//	if (nrates > nitems(lq->rs_table)) {
//		device_printf(sc->sc_dev,
//		    "%s: node supports %d rates, driver handles "
//		    "only %zu\n", __func__, nrates, nitems(lq->rs_table));
//		return;
//	}
	if (nrates == 0) {
		device_printf(sc->sc_dev,
		    "%s: node supports 0 rates, odd!\n", __func__);
		return;
	}
	nrates = imin(rix + 1, nrates);

	IWX_DPRINTF(sc, IWX_DEBUG_TXRATE,
	    "%s: nrates=%d\n", __func__, nrates);

	/* then construct a lq_cmd based on those */
//	memset(lq, 0, sizeof(*lq));
//	lq->sta_id = IWX_STATION_ID;

	/* For HT, always enable RTS/CTS to avoid excessive retries. */
#ifdef not_in_iwx
	if (ni->ni_flags & IEEE80211_NODE_HT)
		lq->flags |= IWX_LQ_FLAG_USE_RTS_MSK;
#endif

	/*
	 * are these used? (we don't do SISO or MIMO)
	 * need to set them to non-zero, though, or we get an error.
	 */
//	lq->single_stream_ant_msk = 1;
//	lq->dual_stream_ant_msk = 1;

	/*
	 * Build the actual rate selection table.
	 * The lowest bits are the rates.  Additionally,
	 * CCK needs bit 9 to be set.  The rest of the bits
	 * we add to the table select the tx antenna
	 * Note that we add the rates in the highest rate first
	 * (opposite of ni_rates).
	 */
	for (i = 0; i < nrates; i++) {
		int rate = rs->rs_rates[rix - i] & IEEE80211_RATE_VAL;
		int nextant;

		/* Map 802.11 rate to HW rate index. */
		ridx = iwx_rate2ridx(sc, rate);
		if (ridx == -1)
			continue;

#if 0
		if (txant == 0)
			txant = iwm_get_valid_tx_ant(sc);
		nextant = 1<<(ffs(txant)-1);
		txant &= ~nextant;
#else
		nextant = iwx_get_valid_tx_ant(sc);
#endif
		tab = iwx_rates[ridx].plcp;
		tab |= nextant << IWX_RATE_MCS_ANT_POS;
		if (IWX_RIDX_IS_CCK(ridx))
			tab |= IWX_RATE_MCS_CCK_MSK;
		IWX_DPRINTF(sc, IWX_DEBUG_TXRATE,
		    "station rate i=%d, rate=%d, hw=%x\n",
		    i, iwx_rates[ridx].rate, tab);
//		lq->rs_table[i] = htole32(tab);
	}
	/* then fill the rest with the lowest possible rate */
//	for (i = nrates; i < nitems(lq->rs_table); i++) {
//		KASSERT(tab != 0, ("invalid tab"));
//		lq->rs_table[i] = htole32(tab);
//	}
}

static int
iwx_media_change(struct ifnet *ifp)
{
	struct ieee80211vap *vap = ifp->if_softc;
	struct ieee80211com *ic = vap->iv_ic;
	struct iwx_softc *sc = ic->ic_softc;
	int error;

	error = ieee80211_media_change(ifp);
	if (error != ENETRESET)
		return error;

	IWX_LOCK(sc);
	if (ic->ic_nrunning > 0) {
		iwx_stop(sc);
		iwx_init(sc);
	}
	IWX_UNLOCK(sc);
	return error;
}

static void
iwx_bring_down_firmware(struct iwx_softc *sc, struct ieee80211vap *vap)
{
	struct iwx_vap *ivp = IWX_VAP(vap);
	int error;

	/* Avoid Tx watchdog triggering, when transfers get dropped here. */
	sc->sc_tx_timer = 0;

	ivp->iv_auth = 0;
	if (sc->sc_firmware_state == 3) {
		iwx_xmit_queue_drain(sc);
//		iwx_flush_tx_path(sc, 0xf, IWM_CMD_SYNC);
		error = iwx_rm_sta(sc, vap, TRUE);
		if (error) {
			device_printf(sc->sc_dev,
			    "%s: Failed to remove station: %d\n",
			    __func__, error);
		}
	}
	if (sc->sc_firmware_state == 3) {
		error = iwx_mac_ctxt_changed(sc, vap);
		if (error) {
			device_printf(sc->sc_dev,
			    "%s: Failed to change mac context: %d\n",
			    __func__, error);
		}
	}
	if (sc->sc_firmware_state == 3) {
		error = iwx_sf_update(sc, vap, FALSE);
		if (error) {
			device_printf(sc->sc_dev,
			    "%s: Failed to update smart FIFO: %d\n",
			    __func__, error);
		}
	}
	if (sc->sc_firmware_state == 3) {
		error = iwx_rm_sta_id(sc, vap);
		if (error) {
			device_printf(sc->sc_dev,
			    "%s: Failed to remove station id: %d\n",
			    __func__, error);
		}
	}
	if (sc->sc_firmware_state == 3) {
		error = iwx_update_quotas(sc, NULL);
		if (error) {
			device_printf(sc->sc_dev,
			    "%s: Failed to update PHY quota: %d\n",
			    __func__, error);
		}
	}
	if (sc->sc_firmware_state == 3) {
		/* XXX Might need to specify bssid correctly. */
		error = iwx_mac_ctxt_changed(sc, vap);
		if (error) {
			device_printf(sc->sc_dev,
			    "%s: Failed to change mac context: %d\n",
			    __func__, error);
		}
	}
	if (sc->sc_firmware_state == 3) {
		sc->sc_firmware_state = 2;
	}
	if (sc->sc_firmware_state > 1) {
		error = iwx_binding_remove_vif(sc, ivp);
		if (error) {
			device_printf(sc->sc_dev,
			    "%s: Failed to remove channel ctx: %d\n",
			    __func__, error);
		}
	}
	if (sc->sc_firmware_state > 1) {
		sc->sc_firmware_state = 1;
	}
	ivp->phy_ctxt = NULL;
	if (sc->sc_firmware_state > 0) {
		error = iwx_mac_ctxt_changed(sc, vap);
		if (error) {
			device_printf(sc->sc_dev,
			    "%s: Failed to change mac context: %d\n",
			    __func__, error);
		}
	}
	if (sc->sc_firmware_state > 0) {
		error = iwx_power_update_mac(sc);
		if (error != 0) {
			device_printf(sc->sc_dev,
			    "%s: failed to update power management\n",
			    __func__);
		}
	}
	sc->sc_firmware_state = 0;
}

static int
iwx_newstate(struct ieee80211vap *vap, enum ieee80211_state nstate, int arg)
{
	struct iwx_vap *ivp = IWX_VAP(vap);
	struct ieee80211com *ic = vap->iv_ic;
	struct iwx_softc *sc = ic->ic_softc;
	struct iwx_node *in;
	int error;

	IWX_DPRINTF(sc, IWX_DEBUG_STATE,
	    "switching state %s -> %s arg=0x%x\n",
	    ieee80211_state_name[vap->iv_state],
	    ieee80211_state_name[nstate],
	    arg);

	IEEE80211_UNLOCK(ic);
	IWX_LOCK(sc);

	if ((sc->sc_flags & IWX_FLAG_SCAN_RUNNING) &&
	    (nstate == IEEE80211_S_AUTH ||
	     nstate == IEEE80211_S_ASSOC ||
	     nstate == IEEE80211_S_RUN)) {
		/* Stop blinking for a scan, when authenticating. */
//		iwm_led_blink_stop(sc);
	}

	if (vap->iv_state == IEEE80211_S_RUN && nstate != IEEE80211_S_RUN) {
//		iwm_led_disable(sc);
		/* disable beacon filtering if we're hopping out of RUN */
		iwx_disable_beacon_filter(sc);
		if (((in = IWX_NODE(vap->iv_bss)) != NULL))
			in->in_assoc = 0;
	}

	if ((vap->iv_state == IEEE80211_S_AUTH ||
	     vap->iv_state == IEEE80211_S_ASSOC ||
	     vap->iv_state == IEEE80211_S_RUN) &&
	    (nstate == IEEE80211_S_INIT ||
	     nstate == IEEE80211_S_SCAN ||
	     nstate == IEEE80211_S_AUTH)) {
		iwx_stop_session_protection(sc, ivp);
	}

	if ((vap->iv_state == IEEE80211_S_RUN ||
	     vap->iv_state == IEEE80211_S_ASSOC) &&
	    nstate == IEEE80211_S_INIT) {
		/*
		 * In this case, iv_newstate() wants to send an 80211 frame on
		 * the network that we are leaving. So we need to call it,
		 * before tearing down all the firmware state.
		 */
		IWX_UNLOCK(sc);
		IEEE80211_LOCK(ic);
		ivp->iv_newstate(vap, nstate, arg);
		IEEE80211_UNLOCK(ic);
		IWX_LOCK(sc);
		iwx_bring_down_firmware(sc, vap);
		IWX_UNLOCK(sc);
		IEEE80211_LOCK(ic);
		return 0;
	}

	switch (nstate) {
	case IEEE80211_S_INIT:
	case IEEE80211_S_SCAN:
		break;

	case IEEE80211_S_AUTH:
		iwx_bring_down_firmware(sc, vap);
		if ((error = iwx_auth(vap, sc)) != 0) {
			device_printf(sc->sc_dev,
			    "%s: could not move to auth state: %d\n",
			    __func__, error);
			iwx_bring_down_firmware(sc, vap);
			IWX_UNLOCK(sc);
			IEEE80211_LOCK(ic);
			return 1;
		}
		break;

	case IEEE80211_S_ASSOC:
		/*
		 * EBS may be disabled due to previous failures reported by FW.
		 * Reset EBS status here assuming environment has been changed.
		 */
		sc->last_ebs_successful = TRUE;
		break;

	case IEEE80211_S_RUN:
		in = IWX_NODE(vap->iv_bss);
		/* Update the association state, now we have it all */
		/* (eg associd comes in at this point */
		error = iwx_update_sta(sc, in);
		if (error != 0) {
			device_printf(sc->sc_dev,
			    "%s: failed to update STA\n", __func__);
			IWX_UNLOCK(sc);
			IEEE80211_LOCK(ic);
			return error;
		}
		in->in_assoc = 1;
		error = iwx_mac_ctxt_changed(sc, vap);
		if (error != 0) {
			device_printf(sc->sc_dev,
			    "%s: failed to update MAC: %d\n", __func__, error);
		}

		iwx_sf_update(sc, vap, FALSE);
		iwx_enable_beacon_filter(sc, ivp);
		iwx_power_update_mac(sc);
		iwx_update_quotas(sc, ivp);
		int rix = ieee80211_ratectl_rate(&in->in_ni, NULL, 0);
		iwx_setrates(sc, in, rix);

//		if ((error = iwx_send_lq_cmd(sc, &in->in_lq, TRUE)) != 0) {
//			device_printf(sc->sc_dev,
//			    "%s: IWX_LQ_CMD failed: %d\n", __func__, error);
//		}

//		iwm_led_enable(sc);
		break;

	default:
		break;
	}
	IWX_UNLOCK(sc);
	IEEE80211_LOCK(ic);

	return (ivp->iv_newstate(vap, nstate, arg));
}

void
iwx_endscan_cb(void *arg, int pending)
{
	struct iwx_softc *sc = arg;
	struct ieee80211com *ic = &sc->sc_ic;

	IWX_DPRINTF(sc, IWX_DEBUG_SCAN | IWX_DEBUG_TRACE,
	    "%s: scan ended\n",
	    __func__);

	ieee80211_scan_done(TAILQ_FIRST(&ic->ic_vaps));
}

static boolean_t
iwx_is_lar_supported(struct iwx_softc *sc)
{
//	boolean_t nvm_lar = sc->nvm_data->lar_enabled;
	boolean_t tlv_lar = iwx_fw_has_capa(sc, IWX_UCODE_TLV_CAPA_LAR_SUPPORT);

	if (iwx_lar_disable)
		return FALSE;

	/*
	 * Enable LAR only if it is supported by the FW (TLV) &&
	 * enabled in the NVM
	 */
	/* todo if_iwx: compare against upstream whether or not this creates a bug */
	return tlv_lar;
}

static boolean_t
iwx_is_wifi_mcc_supported(struct iwx_softc *sc)
{
	return iwx_fw_has_api(sc, IWX_UCODE_TLV_API_WIFI_MCC_UPDATE) ||
	    iwx_fw_has_capa(sc, IWX_UCODE_TLV_CAPA_LAR_MULTI_MCC);
}

static int
iwx_send_update_mcc_cmd(struct iwx_softc *sc, const char *alpha2)
{
	struct iwx_mcc_update_cmd mcc_cmd;
	struct iwx_host_cmd hcmd = {
		.id = IWX_MCC_UPDATE_CMD,
		.flags = (IWX_CMD_SYNC | IWX_CMD_WANT_SKB),
		.data = { &mcc_cmd },
	};
	int ret;
#ifdef IWX_DEBUG
	struct iwx_rx_packet *pkt;
//	struct iwx_mcc_update_resp_v1 *mcc_resp_v1 = NULL;
	struct iwx_mcc_update_resp *mcc_resp;
	int n_channels;
	uint16_t mcc;
#endif
#ifdef not_in_iwx
	int resp_v2 = iwx_fw_has_capa(sc, IWX_UCODE_TLV_CAPA_LAR_SUPPORT_V2);
#endif
	if (!iwx_is_lar_supported(sc)) {
		IWX_DPRINTF(sc, IWX_DEBUG_LAR, "%s: no LAR support\n",
		    __func__);
		return 0;
	}

	memset(&mcc_cmd, 0, sizeof(mcc_cmd));
	mcc_cmd.mcc = htole16(alpha2[0] << 8 | alpha2[1]);
	if (iwx_is_wifi_mcc_supported(sc))
		mcc_cmd.source_id = IWX_MCC_SOURCE_GET_CURRENT;
	else
		mcc_cmd.source_id = IWX_MCC_SOURCE_OLD_FW;

#ifdef not_in_iwx
	if (resp_v2)
		hcmd.len[0] = sizeof(struct iwx_mcc_update_cmd);
	else
		hcmd.len[0] = sizeof(struct iwx_mcc_update_cmd_v1);
#endif
	hcmd.len[0] = sizeof(struct iwx_mcc_update_cmd);

	IWX_DPRINTF(sc, IWX_DEBUG_LAR,
	    "send MCC update to FW with '%c%c' src = %d\n",
	    alpha2[0], alpha2[1], mcc_cmd.source_id);

	ret = iwx_send_cmd(sc, &hcmd);
	if (ret)
		return ret;

#ifdef IWX_DEBUG
	pkt = hcmd.resp_pkt;

	/* Extract MCC response */
#ifdef not_in_iwx
	if (resp_v2) {
		mcc_resp = (void *)pkt->data;
		mcc = mcc_resp->mcc;
		n_channels =  le32toh(mcc_resp->n_channels);
	} else {
		mcc_resp_v1 = (void *)pkt->data;
		mcc = mcc_resp_v1->mcc;
		n_channels =  le32toh(mcc_resp_v1->n_channels);
	}
#endif
	mcc_resp = (void *)pkt->data;
	mcc = mcc_resp->mcc;
	n_channels =  le32toh(mcc_resp->n_channels);

	/* W/A for a FW/NVM issue - returns 0x00 for the world domain */
	if (mcc == 0)
		mcc = 0x3030;  /* "00" - world */

	IWX_DPRINTF(sc, IWX_DEBUG_LAR,
	    "regulatory domain '%c%c' (%d channels available)\n",
	    mcc >> 8, mcc & 0xff, n_channels);
#endif
	iwx_free_resp(sc, &hcmd);

	return 0;
}

static int
iwx_init_hw(struct iwx_softc *sc)
{
	struct ieee80211com *ic = &sc->sc_ic;
//	int error, i, ac;
	int error, i;

	sc->sf_state = IWX_SF_UNINIT;

	if ((error = iwx_start_hw(sc)) != 0) {
		printf("iwx_start_hw: failed %d\n", error);
		return error;
	}

	if ((error = iwx_run_init_unified_ucode(sc, true)) != 0) {
		printf("iwx_run_init_unified_ucode: failed %d\n", error);
		return error;
	}

	/*
	 * should stop and start HW since that INIT
	 * image just loaded
	 */
	iwx_stop_device(sc);
	sc->sc_ps_disabled = FALSE;
	if ((error = iwx_start_hw(sc)) != 0) {
		device_printf(sc->sc_dev, "could not initialize hardware\n");
		return error;
	}

	/* omstart, this time with the regular firmware */
//	error = iwx_load_ucode_wait_alive(sc, IWX_UCODE_REGULAR);
	error = iwx_load_ucode_wait_alive(sc);
	if (error) {
		device_printf(sc->sc_dev, "could not load firmware\n");
		goto error;
	}

	error = iwx_sf_update(sc, NULL, FALSE);
	if (error)
		device_printf(sc->sc_dev, "Failed to initialize Smart Fifo\n");

#if 0
	if ((error = iwx_send_bt_init_conf(sc)) != 0) {
		device_printf(sc->sc_dev, "bt init conf failed\n");
		goto error;
	}
#endif

	error = iwx_send_tx_ant_cfg(sc, iwx_get_valid_tx_ant(sc));
	if (error != 0) {
		device_printf(sc->sc_dev, "antenna config failed\n");
		goto error;
	}

#ifdef not_in_iwx
	/* Send phy db control command and then phy db calibration */
	if ((error = iwx_send_phy_db_data(sc->sc_phy_db)) != 0)
		goto error;
#endif

	if (sc->sc_tx_with_siso_diversity) {
		if ((error = iwx_send_phy_cfg_cmd(sc)) != 0) {
			device_printf(sc->sc_dev, "phy_cfg_cmd failed\n");
			goto error;
		}
	}

	/* todo if_iwx: openbsd calls iwx_send_dqa_cmd(sc) at this point */
	if ((error = iwx_send_dqa_cmd(sc)) != 0) {
		device_printf(sc->sc_dev, "%s iwx_send_dqa_cmd failed", __func__);/* todo if_iwx: remove before submitting for review */

		goto error;
	}

	/* Add auxiliary station for scanning */
	if ((error = iwx_add_aux_sta(sc)) != 0) {
		device_printf(sc->sc_dev, "add_aux_sta failed\n");
		goto error;
	}

	for (i = 0; i < IWX_NUM_PHY_CTX; i++) {
		/*
		 * The channel used here isn't relevant as it's
		 * going to be overwritten in the other flows.
		 * For now use the first channel we have.
		 */
		if ((error = iwx_phy_ctxt_add(sc,
		    &sc->sc_phyctxt[i], &ic->ic_channels[1], 1, 1)) != 0)
			goto error;
	}

	if (iwx_config_ltr(sc) != 0)
		device_printf(sc->sc_dev, "PCIe LTR configuration failed\n");

	error = iwx_power_update_device(sc);
	if (error)
		goto error;

	if ((error = iwx_send_update_mcc_cmd(sc, "ZZ")) != 0)
		goto error;

//	if (iwx_fw_has_capa(sc, IWX_UCODE_TLV_CAPA_UMAC_SCAN)) {
	if ((error = iwx_config_umac_scan(sc)) != 0)
		goto error;
//	}

	/* Enable Tx queues. */
#ifdef not_in_iwx
	for (ac = 0; ac < WME_NUM_AC; ac++) {
		error = iwx_enable_txq(sc, IWX_STATION_ID, ac,
		    iwx_ac_to_tx_fifo[ac]);
		if (error)
			goto error;
	}
#endif

	if ((error = iwx_disable_beacon_filter(sc)) != 0) {
		device_printf(sc->sc_dev, "failed to disable beacon filter\n");
		goto error;
	}

	return 0;

 error:
	iwx_stop_device(sc);
	return error;
}

/* Allow multicast from our BSSID. */
static int
iwx_allow_mcast(struct ieee80211vap *vap, struct iwx_softc *sc)
{
	struct ieee80211_node *ni = vap->iv_bss;
	struct iwx_mcast_filter_cmd *cmd;
	size_t size;
	int error;

	size = roundup(sizeof(*cmd), 4);
	cmd = malloc(size, M_DEVBUF, M_NOWAIT | M_ZERO);
	if (cmd == NULL)
		return ENOMEM;
	cmd->filter_own = 1;
	cmd->port_id = 0;
	cmd->count = 0;
	cmd->pass_all = 1;
	IEEE80211_ADDR_COPY(cmd->bssid, ni->ni_bssid);

	error = iwx_send_cmd_pdu(sc, IWX_MCAST_FILTER_CMD,
	    IWX_CMD_SYNC, size, cmd);
	free(cmd, M_DEVBUF);

	return (error);
}

/*
 * ifnet interfaces
 */

static void
iwx_init(struct iwx_softc *sc)
{
	int error;

	if (sc->sc_flags & IWX_FLAG_HW_INITED) {
		return;
	}
	sc->sc_generation++;
	sc->sc_flags &= ~IWX_FLAG_STOPPED;

	if ((error = iwx_init_hw(sc)) != 0) {
		printf("iwx_init_hw failed %d\n", error);
		iwx_stop(sc);
		return;
	}

	/*
	 * Ok, firmware loaded and we are jogging
	 */
	sc->sc_flags |= IWX_FLAG_HW_INITED;
}

static int
iwx_transmit(struct ieee80211com *ic, struct mbuf *m)
{
	struct iwx_softc *sc;
	int error;

	sc = ic->ic_softc;

	IWX_LOCK(sc);
	if ((sc->sc_flags & IWX_FLAG_HW_INITED) == 0) {
		IWX_UNLOCK(sc);
		return (ENXIO);
	}
	error = mbufq_enqueue(&sc->sc_snd, m);
	if (error) {
		IWX_UNLOCK(sc);
		return (error);
	}
	iwx_start(sc);
	IWX_UNLOCK(sc);
	return (0);
}

/*
 * Dequeue packets from sendq and call send.
 */
static void
iwx_start(struct iwx_softc *sc)
{
	struct ieee80211_node *ni;
	struct mbuf *m;
	int ac = 0;

	IWX_DPRINTF(sc, IWX_DEBUG_XMIT | IWX_DEBUG_TRACE, "->%s\n", __func__);
	while (sc->qfullmsk == 0 &&
		(m = mbufq_dequeue(&sc->sc_snd)) != NULL) {
		ni = (struct ieee80211_node *)m->m_pkthdr.rcvif;
		if (iwx_tx(sc, m, ni, ac) != 0) {
			if_inc_counter(ni->ni_vap->iv_ifp,
			    IFCOUNTER_OERRORS, 1);
			ieee80211_free_node(ni);
			continue;
		}
		if (sc->sc_tx_timer == 0) {
			callout_reset(&sc->sc_watchdog_to, hz, iwx_watchdog,
			    sc);
		}
		sc->sc_tx_timer = 15;
	}
	IWX_DPRINTF(sc, IWX_DEBUG_XMIT | IWX_DEBUG_TRACE, "<-%s\n", __func__);
}

static void
iwx_stop(struct iwx_softc *sc)
{

	sc->sc_flags &= ~IWX_FLAG_HW_INITED;
	sc->sc_flags |= IWX_FLAG_STOPPED;
	sc->sc_generation++;
	sc->sc_tx_timer = 0;
	iwx_stop_device(sc);
	sc->sc_flags &= ~IWX_FLAG_SCAN_RUNNING;
}

static void
iwx_watchdog(void *arg)
{
	struct iwx_softc *sc = arg;
	struct ieee80211com *ic = &sc->sc_ic;

	if (sc->sc_attached == 0)
		return;

	if (sc->sc_tx_timer > 0) {
		if (--sc->sc_tx_timer == 0) {
			device_printf(sc->sc_dev, "device timeout\n");
#ifdef IWX_DEBUG
			iwx_nic_error(sc);
#endif
			ieee80211_restart_all(ic);
			counter_u64_add(sc->sc_ic.ic_oerrors, 1);
			return;
		}
		callout_reset(&sc->sc_watchdog_to, hz, iwx_watchdog, sc);
	}
}

static void
iwx_parent(struct ieee80211com *ic)
{
	struct iwx_softc *sc = ic->ic_softc;
	int startall = 0;

	IWX_LOCK(sc);
	if (ic->ic_nrunning > 0) {
		if (!(sc->sc_flags & IWX_FLAG_HW_INITED)) {
			iwx_init(sc);
			startall = 1;
		}
	} else if (sc->sc_flags & IWX_FLAG_HW_INITED)
		iwx_stop(sc);
	IWX_UNLOCK(sc);
	if (startall)
		ieee80211_start_all(ic);
}

/*
 * The interrupt side of things
 */

/*
 * error dumping routines are from iwlwifi/mvm/utils.c
 */

/*
 * Note: This structure is read from the device with IO accesses,
 * and the reading already does the endian conversion. As it is
 * read with uint32_t-sized accesses, any members with a different size
 * need to be ordered correctly though!
 */
struct iwx_error_event_table {
	uint32_t valid;		/* (nonzero) valid, (0) log is empty */
	uint32_t error_id;		/* type of error */
	uint32_t trm_hw_status0;	/* TRM HW status */
	uint32_t trm_hw_status1;	/* TRM HW status */
	uint32_t blink2;		/* branch link */
	uint32_t ilink1;		/* interrupt link */
	uint32_t ilink2;		/* interrupt link */
	uint32_t data1;		/* error-specific data */
	uint32_t data2;		/* error-specific data */
	uint32_t data3;		/* error-specific data */
	uint32_t bcon_time;		/* beacon timer */
	uint32_t tsf_low;		/* network timestamp function timer */
	uint32_t tsf_hi;		/* network timestamp function timer */
	uint32_t gp1;		/* GP1 timer register */
	uint32_t gp2;		/* GP2 timer register */
	uint32_t fw_rev_type;	/* firmware revision type */
	uint32_t major;		/* uCode version major */
	uint32_t minor;		/* uCode version minor */
	uint32_t hw_ver;		/* HW Silicon version */
	uint32_t brd_ver;		/* HW board version */
	uint32_t log_pc;		/* log program counter */
	uint32_t frame_ptr;		/* frame pointer */
	uint32_t stack_ptr;		/* stack pointer */
	uint32_t hcmd;		/* last host command header */
	uint32_t isr0;		/* isr status register LMPM_NIC_ISR0:
				 * rxtx_flag */
	uint32_t isr1;		/* isr status register LMPM_NIC_ISR1:
				 * host_flag */
	uint32_t isr2;		/* isr status register LMPM_NIC_ISR2:
				 * enc_flag */
	uint32_t isr3;		/* isr status register LMPM_NIC_ISR3:
				 * time_flag */
	uint32_t isr4;		/* isr status register LMPM_NIC_ISR4:
				 * wico interrupt */
	uint32_t last_cmd_id;	/* last HCMD id handled by the firmware */
	uint32_t wait_event;		/* wait event() caller address */
	uint32_t l2p_control;	/* L2pControlField */
	uint32_t l2p_duration;	/* L2pDurationField */
	uint32_t l2p_mhvalid;	/* L2pMhValidBits */
	uint32_t l2p_addr_match;	/* L2pAddrMatchStat */
	uint32_t lmpm_pmg_sel;	/* indicate which clocks are turned on
				 * (LMPM_PMG_SEL) */
	uint32_t u_timestamp;	/* indicate when the date and time of the
				 * compilation */
	uint32_t flow_handler;	/* FH read/write pointers, RX credit */
} __packed /* LOG_ERROR_TABLE_API_S_VER_3 */;

/*
 * UMAC error struct - relevant starting from family 8000 chip.
 * Note: This structure is read from the device with IO accesses,
 * and the reading already does the endian conversion. As it is
 * read with u32-sized accesses, any members with a different size
 * need to be ordered correctly though!
 */
struct iwx_umac_error_event_table {
	uint32_t valid;		/* (nonzero) valid, (0) log is empty */
	uint32_t error_id;	/* type of error */
	uint32_t blink1;	/* branch link */
	uint32_t blink2;	/* branch link */
	uint32_t ilink1;	/* interrupt link */
	uint32_t ilink2;	/* interrupt link */
	uint32_t data1;		/* error-specific data */
	uint32_t data2;		/* error-specific data */
	uint32_t data3;		/* error-specific data */
	uint32_t umac_major;
	uint32_t umac_minor;
	uint32_t frame_pointer;	/* core register 27*/
	uint32_t stack_pointer;	/* core register 28 */
	uint32_t cmd_header;	/* latest host cmd sent to UMAC */
	uint32_t nic_isr_pref;	/* ISR status register */
} __packed;

#define ERROR_START_OFFSET  (1 * sizeof(uint32_t))
#define ERROR_ELEM_SIZE     (7 * sizeof(uint32_t))

#ifdef IWX_DEBUG
struct {
	const char *name;
	uint8_t num;
} advanced_lookup[] = {
	{ "NMI_INTERRUPT_WDG", 0x34 },
	{ "SYSASSERT", 0x35 },
	{ "UCODE_VERSION_MISMATCH", 0x37 },
	{ "BAD_COMMAND", 0x38 },
	{ "NMI_INTERRUPT_DATA_ACTION_PT", 0x3C },
	{ "FATAL_ERROR", 0x3D },
	{ "NMI_TRM_HW_ERR", 0x46 },
	{ "NMI_INTERRUPT_TRM", 0x4C },
	{ "NMI_INTERRUPT_BREAK_POINT", 0x54 },
	{ "NMI_INTERRUPT_WDG_RXF_FULL", 0x5C },
	{ "NMI_INTERRUPT_WDG_NO_RBD_RXF_FULL", 0x64 },
	{ "NMI_INTERRUPT_HOST", 0x66 },
	{ "NMI_INTERRUPT_ACTION_PT", 0x7C },
	{ "NMI_INTERRUPT_UNKNOWN", 0x84 },
	{ "NMI_INTERRUPT_INST_ACTION_PT", 0x86 },
	{ "ADVANCED_SYSASSERT", 0 },
};

static const char *
iwx_desc_lookup(uint32_t num)
{
	int i;

	for (i = 0; i < nitems(advanced_lookup) - 1; i++)
		if (advanced_lookup[i].num == num)
			return advanced_lookup[i].name;

	/* No entry matches 'num', so it is the last: ADVANCED_SYSASSERT */
	return advanced_lookup[i].name;
}
static void
iwx_nic_umac_error(struct iwx_softc *sc)
{
	struct iwx_umac_error_event_table table;
	uint32_t base;

	base = sc->umac_error_event_table;

	if (base < 0x800000) {
		device_printf(sc->sc_dev, "Invalid error log pointer 0x%08x\n",
		    base);
		return;
	}

	if (iwx_read_mem(sc, base, &table, sizeof(table)/sizeof(uint32_t))) {
		device_printf(sc->sc_dev, "reading errlog failed\n");
		return;
	}

	if (ERROR_START_OFFSET <= table.valid * ERROR_ELEM_SIZE) {
		device_printf(sc->sc_dev, "Start UMAC Error Log Dump:\n");
		device_printf(sc->sc_dev, "Status: 0x%x, count: %d\n",
		    sc->sc_flags, table.valid);
	}

	device_printf(sc->sc_dev, "0x%08X | %s\n", table.error_id,
		iwx_desc_lookup(table.error_id));
	device_printf(sc->sc_dev, "0x%08X | umac branchlink1\n", table.blink1);
	device_printf(sc->sc_dev, "0x%08X | umac branchlink2\n", table.blink2);
	device_printf(sc->sc_dev, "0x%08X | umac interruptlink1\n",
	    table.ilink1);
	device_printf(sc->sc_dev, "0x%08X | umac interruptlink2\n",
	    table.ilink2);
	device_printf(sc->sc_dev, "0x%08X | umac data1\n", table.data1);
	device_printf(sc->sc_dev, "0x%08X | umac data2\n", table.data2);
	device_printf(sc->sc_dev, "0x%08X | umac data3\n", table.data3);
	device_printf(sc->sc_dev, "0x%08X | umac major\n", table.umac_major);
	device_printf(sc->sc_dev, "0x%08X | umac minor\n", table.umac_minor);
	device_printf(sc->sc_dev, "0x%08X | frame pointer\n",
	    table.frame_pointer);
	device_printf(sc->sc_dev, "0x%08X | stack pointer\n",
	    table.stack_pointer);
	device_printf(sc->sc_dev, "0x%08X | last host cmd\n", table.cmd_header);
	device_printf(sc->sc_dev, "0x%08X | isr status reg\n",
	    table.nic_isr_pref);
}

/*
 * Support for dumping the error log seemed like a good idea ...
 * but it's mostly hex junk and the only sensible thing is the
 * hw/ucode revision (which we know anyway).  Since it's here,
 * I'll just leave it in, just in case e.g. the Intel guys want to
 * help us decipher some "ADVANCED_SYSASSERT" later.
 */
static void
iwx_nic_error(struct iwx_softc *sc)
{
	struct iwx_error_event_table table;
	uint32_t base;

	device_printf(sc->sc_dev, "dumping device error log\n");
//	base = sc->error_event_table[0];
	base = sc->lmac_error_event_table[0];
	if (base < 0x800000) {
		device_printf(sc->sc_dev,
		    "Invalid error log pointer 0x%08x\n", base);
		return;
	}

	if (iwx_read_mem(sc, base, &table, sizeof(table)/sizeof(uint32_t))) {
		device_printf(sc->sc_dev, "reading errlog failed\n");
		return;
	}

	if (!table.valid) {
		device_printf(sc->sc_dev, "errlog not found, skipping\n");
		return;
	}

	if (ERROR_START_OFFSET <= table.valid * ERROR_ELEM_SIZE) {
		device_printf(sc->sc_dev, "Start Error Log Dump:\n");
		device_printf(sc->sc_dev, "Status: 0x%x, count: %d\n",
		    sc->sc_flags, table.valid);
	}

	device_printf(sc->sc_dev, "0x%08X | %-28s\n", table.error_id,
	    iwx_desc_lookup(table.error_id));
	device_printf(sc->sc_dev, "%08X | trm_hw_status0\n",
	    table.trm_hw_status0);
	device_printf(sc->sc_dev, "%08X | trm_hw_status1\n",
	    table.trm_hw_status1);
	device_printf(sc->sc_dev, "%08X | branchlink2\n", table.blink2);
	device_printf(sc->sc_dev, "%08X | interruptlink1\n", table.ilink1);
	device_printf(sc->sc_dev, "%08X | interruptlink2\n", table.ilink2);
	device_printf(sc->sc_dev, "%08X | data1\n", table.data1);
	device_printf(sc->sc_dev, "%08X | data2\n", table.data2);
	device_printf(sc->sc_dev, "%08X | data3\n", table.data3);
	device_printf(sc->sc_dev, "%08X | beacon time\n", table.bcon_time);
	device_printf(sc->sc_dev, "%08X | tsf low\n", table.tsf_low);
	device_printf(sc->sc_dev, "%08X | tsf hi\n", table.tsf_hi);
	device_printf(sc->sc_dev, "%08X | time gp1\n", table.gp1);
	device_printf(sc->sc_dev, "%08X | time gp2\n", table.gp2);
	device_printf(sc->sc_dev, "%08X | uCode revision type\n",
	    table.fw_rev_type);
	device_printf(sc->sc_dev, "%08X | uCode version major\n", table.major);
	device_printf(sc->sc_dev, "%08X | uCode version minor\n", table.minor);
	device_printf(sc->sc_dev, "%08X | hw version\n", table.hw_ver);
	device_printf(sc->sc_dev, "%08X | board version\n", table.brd_ver);
	device_printf(sc->sc_dev, "%08X | hcmd\n", table.hcmd);
	device_printf(sc->sc_dev, "%08X | isr0\n", table.isr0);
	device_printf(sc->sc_dev, "%08X | isr1\n", table.isr1);
	device_printf(sc->sc_dev, "%08X | isr2\n", table.isr2);
	device_printf(sc->sc_dev, "%08X | isr3\n", table.isr3);
	device_printf(sc->sc_dev, "%08X | isr4\n", table.isr4);
	device_printf(sc->sc_dev, "%08X | last cmd Id\n", table.last_cmd_id);
	device_printf(sc->sc_dev, "%08X | wait_event\n", table.wait_event);
	device_printf(sc->sc_dev, "%08X | l2p_control\n", table.l2p_control);
	device_printf(sc->sc_dev, "%08X | l2p_duration\n", table.l2p_duration);
	device_printf(sc->sc_dev, "%08X | l2p_mhvalid\n", table.l2p_mhvalid);
	device_printf(sc->sc_dev, "%08X | l2p_addr_match\n", table.l2p_addr_match);
	device_printf(sc->sc_dev, "%08X | lmpm_pmg_sel\n", table.lmpm_pmg_sel);
	device_printf(sc->sc_dev, "%08X | timestamp\n", table.u_timestamp);
	device_printf(sc->sc_dev, "%08X | flow_handler\n", table.flow_handler);

	if (sc->umac_error_event_table)
		iwx_nic_umac_error(sc);
}
#endif

/* iwx_rx_pkt() in openbsd */
static void
iwx_handle_rxb(struct iwx_softc *sc, struct mbuf *m)
{
	struct ieee80211com *ic = &sc->sc_ic;
//	struct iwx_cmd_response *cresp;
	struct mbuf *m1;
	uint32_t offset = 0;
	uint32_t maxoff = IWX_RBUF_SIZE;
	uint32_t nextoff;
	boolean_t stolen = FALSE;

#define HAVEROOM(a)	\
    ((a) + sizeof(uint32_t) + sizeof(struct iwx_cmd_header) < maxoff)

	while (HAVEROOM(offset)) {
		struct iwx_rx_packet *pkt = mtodoff(m, struct iwx_rx_packet *,
		    offset);
		int qid, idx, code, len;

		qid = pkt->hdr.qid;
		idx = pkt->hdr.idx;

		code = IWX_WIDE_ID(pkt->hdr.flags, pkt->hdr.code);

		/*
		 * randomly get these from the firmware, no idea why.
		 * they at least seem harmless, so just ignore them for now
		 */
		if ((pkt->hdr.code == 0 && (qid & ~0x80) == 0 && idx == 0) ||
		    pkt->len_n_flags == htole32(IWX_FH_RSCSR_FRAME_INVALID)) {
			break;
		}

		IWX_DPRINTF(sc, IWX_DEBUG_INTR,
		    "rx packet qid=%d idx=%d type=%x\n",
		    qid & ~0x80, pkt->hdr.idx, code);

		len = iwx_rx_packet_len(pkt);
		len += sizeof(uint32_t); /* account for status word */
		nextoff = offset + roundup2(len, IWX_FH_RSCSR_FRAME_ALIGN);

		iwx_notification_wait_notify(sc->sc_notif_wait, code, pkt);

		switch (code) {
		case IWX_REPLY_RX_PHY_CMD:
			iwx_rx_rx_phy_cmd(sc, pkt);
			break;

		case IWX_REPLY_RX_MPDU_CMD: {
			/*
			 * If this is the last frame in the RX buffer, we
			 * can directly feed the mbuf to the sharks here.
			 */
			struct iwx_rx_packet *nextpkt = mtodoff(m,
			    struct iwx_rx_packet *, nextoff);
			if (!HAVEROOM(nextoff) ||
			    (nextpkt->hdr.code == 0 &&
			     (nextpkt->hdr.qid & ~0x80) == 0 &&
			     nextpkt->hdr.idx == 0) ||
			    (nextpkt->len_n_flags ==
			     htole32(IWX_FH_RSCSR_FRAME_INVALID))) {
				if (iwx_rx_mpdu(sc, m, offset, stolen)) {
//				if (iwx_rx_mpdu_mq(sc, m, offset, stolen)) {
					stolen = FALSE;
					/* Make sure we abort the loop */
					nextoff = maxoff;
				}
				break;
			}

			/*
			 * Use m_copym instead of m_split, because that
			 * makes it easier to keep a valid rx buffer in
			 * the ring, when iwm_rx_mpdu() fails.
			 *
			 * We need to start m_copym() at offset 0, to get the
			 * M_PKTHDR flag preserved.
			 */
			m1 = m_copym(m, 0, M_COPYALL, M_NOWAIT);
			if (m1) {
			if (iwx_rx_mpdu(sc, m1, offset, stolen))
//			if (iwx_rx_mpdu_mq(sc, m1, offset, stolen))
					stolen = TRUE;
				else
					m_freem(m1);
			}
			break;
		}

		case IWX_TX_CMD:
			iwx_rx_tx_cmd(sc, pkt);
			break;

		case IWX_MISSED_BEACONS_NOTIFICATION: {
			struct iwx_missed_beacons_notif *resp;
			int missed;

			/* XXX look at mac_id to determine interface ID */
			struct ieee80211vap *vap = TAILQ_FIRST(&ic->ic_vaps);

			resp = (void *)pkt->data;
			missed = le32toh(resp->consec_missed_beacons);

			IWX_DPRINTF(sc, IWX_DEBUG_BEACON | IWX_DEBUG_STATE,
			    "%s: MISSED_BEACON: mac_id=%d, "
			    "consec_since_last_rx=%d, consec=%d, num_expect=%d "
			    "num_rx=%d\n",
			    __func__,
			    le32toh(resp->mac_id),
			    le32toh(resp->consec_missed_beacons_since_last_rx),
			    le32toh(resp->consec_missed_beacons),
			    le32toh(resp->num_expected_beacons),
			    le32toh(resp->num_recvd_beacons));

			/* Be paranoid */
			if (vap == NULL)
				break;

			/* XXX no net80211 locking? */
			if (vap->iv_state == IEEE80211_S_RUN &&
			    (ic->ic_flags & IEEE80211_F_SCAN) == 0) {
				if (missed > vap->iv_bmissthreshold) {
					/* XXX bad locking; turn into task */
					IWX_UNLOCK(sc);
					ieee80211_beacon_miss(ic);
					IWX_LOCK(sc);
				}
			}

			break;
		}

		case IWX_MFUART_LOAD_NOTIFICATION:
			break;

		case IWX_ALIVE:
			break;

#ifdef not_in_iwx
		case IWX_CALIB_RES_NOTIF_PHY_DB:
			break;
#endif

		case IWX_STATISTICS_NOTIFICATION:
			iwx_handle_rx_statistics(sc, pkt);
			break;

		case IWX_NVM_ACCESS_CMD:
		case IWX_MCC_UPDATE_CMD:
			if (sc->sc_wantresp == (((qid & ~0x80) << 16) | idx)) {
				memcpy(sc->sc_cmd_resp,
				    pkt, sizeof(sc->sc_cmd_resp));
			}
			break;

#ifdef not_in_iwx
		case IWX_MCC_CHUB_UPDATE_CMD: {
			struct iwx_mcc_chub_notif *notif;
			notif = (void *)pkt->data;

			sc->sc_fw_mcc[0] = (notif->mcc & 0xff00) >> 8;
			sc->sc_fw_mcc[1] = notif->mcc & 0xff;
			sc->sc_fw_mcc[2] = '\0';
			IWX_DPRINTF(sc, IWX_DEBUG_LAR,
			    "fw source %d sent CC '%s'\n",
			    notif->source_id, sc->sc_fw_mcc);
			break;
		}
#endif

		case IWX_DTS_MEASUREMENT_NOTIFICATION:
		case IWX_WIDE_ID(IWX_PHY_OPS_GROUP,
				 IWX_DTS_MEASUREMENT_NOTIF_WIDE): {
			struct iwx_dts_measurement_notif_v1 *notif;

			if (iwx_rx_packet_payload_len(pkt) < sizeof(*notif)) {
				device_printf(sc->sc_dev,
				    "Invalid DTS_MEASUREMENT_NOTIFICATION\n");
				break;
			}
			notif = (void *)pkt->data;
			IWX_DPRINTF(sc, IWX_DEBUG_TEMP,
			    "IWX_DTS_MEASUREMENT_NOTIFICATION - %d\n",
			    notif->temp);
			break;
		}

		case IWX_PHY_CONFIGURATION_CMD:
		case IWX_TX_ANT_CONFIGURATION_CMD:
		case IWX_ADD_STA:
		case IWX_MAC_CONTEXT_CMD:
		case IWX_REPLY_SF_CFG_CMD:
		case IWX_POWER_TABLE_CMD:
		case IWX_LTR_CONFIG:
		case IWX_PHY_CONTEXT_CMD:
		case IWX_BINDING_CONTEXT_CMD:
		case IWX_TIME_EVENT_CMD:
		case IWX_WIDE_ID(IWX_ALWAYS_LONG_GROUP, IWX_SCAN_CFG_CMD):
		case IWX_WIDE_ID(IWX_ALWAYS_LONG_GROUP, IWX_SCAN_REQ_UMAC):
		case IWX_WIDE_ID(IWX_ALWAYS_LONG_GROUP, IWX_SCAN_ABORT_UMAC):
#ifdef not_in_iwx
		case IWX_SCAN_OFFLOAD_REQUEST_CMD:
		case IWX_SCAN_OFFLOAD_ABORT_CMD:
#endif
		case IWX_REPLY_BEACON_FILTERING_CMD:
		case IWX_MAC_PM_POWER_TABLE:
		case IWX_TIME_QUOTA_CMD:
		case IWX_REMOVE_STA:
		case IWX_TXPATH_FLUSH:
#ifdef not_in_iwx
		case IWX_LQ_CMD:
		case IWX_WIDE_ID(IWX_ALWAYS_LONG_GROUP,
				 IWX_FW_PAGING_BLOCK_CMD):
#endif
		case IWX_BT_CONFIG:
#ifdef not_in_iwx
		case IWX_REPLY_THERMAL_MNG_BACKOFF:
			cresp = (void *)pkt->data;
			if (sc->sc_wantresp == (((qid & ~0x80) << 16) | idx)) {
				memcpy(sc->sc_cmd_resp,
				    pkt, sizeof(*pkt)+sizeof(*cresp));
			}
			break;

		/* ignore */
		case IWX_PHY_DB_CMD:
			break;
#endif

		case IWX_INIT_COMPLETE_NOTIF:
			break;

#ifdef not_in_iwx
		case IWX_SCAN_OFFLOAD_COMPLETE:
			iwx_rx_lmac_scan_complete_notif(sc, pkt);
			if (sc->sc_flags & IWX_FLAG_SCAN_RUNNING) {
				sc->sc_flags &= ~IWX_FLAG_SCAN_RUNNING;
				ieee80211_runtask(ic, &sc->sc_es_task);
			}
			break;

		case IWX_SCAN_ITERATION_COMPLETE: {
			struct iwx_lmac_scan_complete_notif *notif;
			notif = (void *)pkt->data;
			break;
		}
#endif

		case IWX_SCAN_COMPLETE_UMAC:
			iwx_rx_umac_scan_complete_notif(sc, pkt);
			if (sc->sc_flags & IWX_FLAG_SCAN_RUNNING) {
				sc->sc_flags &= ~IWX_FLAG_SCAN_RUNNING;
				ieee80211_runtask(ic, &sc->sc_es_task);
			}
			break;

		case IWX_SCAN_ITERATION_COMPLETE_UMAC: {
			struct iwx_umac_scan_iter_complete_notif *notif;
			notif = (void *)pkt->data;

			IWX_DPRINTF(sc, IWX_DEBUG_SCAN, "UMAC scan iteration "
			    "complete, status=0x%x, %d channels scanned\n",
			    notif->status, notif->scanned_channels);
			break;
		}

		case IWX_REPLY_ERROR: {
			struct iwx_error_resp *resp;
			resp = (void *)pkt->data;

			device_printf(sc->sc_dev,
			    "firmware error 0x%x, cmd 0x%x\n",
			    le32toh(resp->error_type),
			    resp->cmd_id);
			break;
		}

		case IWX_TIME_EVENT_NOTIFICATION:
			iwx_rx_time_event_notif(sc, pkt);
			break;

#if 0
		/*
		 * Firmware versions 21 and 22 generate some DEBUG_LOG_MSG
		 * messages. Just ignore them for now.
		 */
		case IWX_DEBUG_LOG_MSG:
			break;
#endif

		case IWX_MCAST_FILTER_CMD:
			break;

#ifdef not_in_iwx
		case IWX_SCD_QUEUE_CFG: {
			struct iwx_scd_txq_cfg_rsp *rsp;
			rsp = (void *)pkt->data;

			IWX_DPRINTF(sc, IWX_DEBUG_CMD,
			    "queue cfg token=0x%x sta_id=%d "
			    "tid=%d scd_queue=%d\n",
			    rsp->token, rsp->sta_id, rsp->tid,
			    rsp->scd_queue);
			break;
		}
#endif
		case IWX_WIDE_ID(IWX_DATA_PATH_GROUP, IWX_DQA_ENABLE_CMD):
			break;

		case IWX_WIDE_ID(IWX_SYSTEM_GROUP, IWX_INIT_EXTENDED_CFG_CMD):
			break;

		case IWX_WIDE_ID(IWX_REGULATORY_AND_NVM_GROUP,
			IWX_NVM_ACCESS_COMPLETE):
			break;

		case IWX_WIDE_ID(IWX_DATA_PATH_GROUP, IWX_RX_NO_DATA_NOTIF):
			break;

		default:
			device_printf(sc->sc_dev,
			    "frame %d/%d %x UNHANDLED (this should "
			    "not happen)\n", qid & ~0x80, idx,
			    pkt->len_n_flags);
			break;
		}

		/*
		 * Why test bit 0x80?  The Linux driver:
		 *
		 * There is one exception:  uCode sets bit 15 when it
		 * originates the response/notification, i.e. when the
		 * response/notification is not a direct response to a
		 * command sent by the driver.  For example, uCode issues
		 * IWM_REPLY_RX when it sends a received frame to the driver;
		 * it is not a direct response to any driver command.
		 *
		 * Ok, so since when is 7 == 15?  Well, the Linux driver
		 * uses a slightly different format for pkt->hdr, and "qid"
		 * is actually the upper byte of a two-byte field.
		 */
		if (!(qid & (1 << 7)))
			iwx_cmd_done(sc, pkt, 0);

		offset = nextoff;
	}
	if (stolen)
		m_freem(m);
#undef HAVEROOM
}

/*
 * Process an IWM_CSR_INT_BIT_FH_RX or IWM_CSR_INT_BIT_SW_RX interrupt.
 * Basic structure from if_iwn
 */
static void
iwx_notif_intr(struct iwx_softc *sc)
{
//	uint32_t wreg;
	uint16_t hw;

	bus_dmamap_sync(sc->rxq.stat_dma.tag, sc->rxq.stat_dma.map,
	    BUS_DMASYNC_POSTREAD);

#ifdef not_in_iwx
	if (sc->cfg->mqrx_supported) {
		count = IWX_RX_MQ_RING_COUNT;
		wreg = IWX_RFH_Q0_FRBDCB_WIDX_TRG;
	} else {
		count = IWX_RX_LEGACY_RING_COUNT;
		wreg = IWX_FH_RSCSR_CHNL0_WPTR;
	}
#endif

	hw = le16toh(sc->rxq.stat->closed_rb_num) & 0xfff;

	/*
	 * Process responses
	 */
	while (sc->rxq.cur != hw) {
		struct iwx_rx_ring *ring = &sc->rxq;
		struct iwx_rx_data *data = &ring->data[ring->cur];

		bus_dmamap_sync(ring->data_dmat, data->map,
		    BUS_DMASYNC_POSTREAD);

		IWX_DPRINTF(sc, IWX_DEBUG_INTR,
		    "%s: hw = %d cur = %d\n", __func__, hw, ring->cur);
		iwx_handle_rxb(sc, data->m);

		ring->cur = (ring->cur + 1) % IWX_RX_MQ_RING_COUNT;
	}

	/*
	 * Tell the firmware that it can reuse the ring entries that
	 * we have just processed.
	 * Seems like the hardware gets upset unless we align
	 * the write by 8??
	 */
	hw = (hw == 0) ? IWX_RX_MQ_RING_COUNT - 1 : hw - 1;
	IWX_WRITE(sc, IWX_RFH_Q0_FRBDCB_WIDX_TRG, rounddown2(hw, 8));
}

static void
iwx_intr(void *arg)
{
	struct iwx_softc *sc = arg;
	int handled = 0;
	int r1, r2, rv = 0;
	int isperiodic = 0;

	IWX_LOCK(sc);
	IWX_WRITE(sc, IWX_CSR_INT_MASK, 0);

	if (sc->sc_flags & IWX_FLAG_USE_ICT) {
		uint32_t *ict = sc->ict_dma.vaddr;
		int tmp;

		tmp = htole32(ict[sc->ict_cur]);
		if (!tmp)
			goto out_ena;

		/*
		 * ok, there was something.  keep plowing until we have all.
		 */
		device_printf(sc->sc_dev, "%s: we be plowin'\n", __func__);/* todo if_iwx: remove before submitting for review */

		r1 = r2 = 0;
		while (tmp) {
			r1 |= tmp;
			ict[sc->ict_cur] = 0;
			sc->ict_cur = (sc->ict_cur+1) % IWX_ICT_COUNT;
			tmp = htole32(ict[sc->ict_cur]);
		}

		/* this is where the fun begins.  don't ask */
		if (r1 == 0xffffffff)
			r1 = 0;

		/* i am not expected to understand this */
		if (r1 & 0xc0000)
			r1 |= 0x8000;
		r1 = (0xff & r1) | ((0xff00 & r1) << 16);
	} else {
		r1 = IWX_READ(sc, IWX_CSR_INT);
		/* "hardware gone" (where, fishing?) */
		if (r1 == 0xffffffff || (r1 & 0xfffffff0) == 0xa5a5a5a0) {
			device_printf(sc->sc_dev, "%s: r1 could be 0xa5a5a5a0...: r1=0x%x\n", __func__, r1);/* todo if_iwx: remove before submitting for review */

			goto out;
		}
		r2 = IWX_READ(sc, IWX_CSR_FH_INT_STATUS);
	}
	if (r1 == 0 && r2 == 0) {
		goto out_ena;
	}

	IWX_WRITE(sc, IWX_CSR_INT, r1 | ~sc->sc_intmask);

	/* copied from openbsd */
	if (r1 & IWX_CSR_INT_BIT_ALIVE) {
		int i;

		/* Firmware has now configured the RFH. */
		for (i = 0; i < IWX_RX_MQ_RING_COUNT; i++)
			iwx_update_rx_desc(sc, &sc->rxq, i);
		IWX_WRITE(sc, IWX_RFH_Q0_FRBDCB_WIDX_TRG, 8);
	}

	/* from OpenBSD */
	handled |= (r1 & (IWX_CSR_INT_BIT_ALIVE /* | IWX_CSR_INT_BIT_SCD */));

	if (r1 & IWX_CSR_INT_BIT_SW_ERR) {
		int i;
		struct ieee80211com *ic = &sc->sc_ic;
		struct ieee80211vap *vap = TAILQ_FIRST(&ic->ic_vaps);

#ifdef IWX_DEBUG
		iwx_nic_error(sc);
#endif
		/* Dump driver status (TX and RX rings) while we're here. */
		device_printf(sc->sc_dev, "driver status:\n");
		for (i = 0; i < IWX_MAX_QUEUES; i++) {
			struct iwx_tx_ring *ring = &sc->txq[i];
			device_printf(sc->sc_dev,
			    "  tx ring %2d: qid=%-2d cur=%-3d "
			    "queued=%-3d\n",
			    i, ring->qid, ring->cur, ring->queued);
		}
		device_printf(sc->sc_dev,
		    "  rx ring: cur=%d\n", sc->rxq.cur);
		device_printf(sc->sc_dev,
		    "  802.11 state %d\n", (vap == NULL) ? -1 : vap->iv_state);

		/* Reset our firmware state tracking. */
		sc->sc_firmware_state = 0;
		/* Don't stop the device; just do a VAP restart */
		IWX_UNLOCK(sc);

		if (vap == NULL) {
			printf("%s: null vap\n", __func__);
			return;
		}

		device_printf(sc->sc_dev, "%s: controller panicked, iv_state = %d; "
		    "restarting\n", __func__, vap->iv_state);

		ieee80211_restart_all(ic);
		return;
	}

	if (r1 & IWX_CSR_INT_BIT_HW_ERR) {
		handled |= IWX_CSR_INT_BIT_HW_ERR;
		device_printf(sc->sc_dev, "hardware error, stopping device\n");
		iwx_stop(sc);
		rv = 1;
		goto out;
	}

	/* firmware chunk loaded */
	if (r1 & IWX_CSR_INT_BIT_FH_TX) {
		IWX_WRITE(sc, IWX_CSR_FH_INT_STATUS, IWX_CSR_FH_INT_TX_MASK);
		handled |= IWX_CSR_INT_BIT_FH_TX;
		sc->sc_fw_chunk_done = 1;
		wakeup(&sc->sc_fw);
	}

	if (r1 & IWX_CSR_INT_BIT_RF_KILL) {
		handled |= IWX_CSR_INT_BIT_RF_KILL;
		if (iwx_check_rfkill(sc)) {
			device_printf(sc->sc_dev,
			    "%s: rfkill switch, disabling interface\n",
			    __func__);
			iwx_stop(sc);
		}
	}

	/*
	 * The Linux driver uses periodic interrupts to avoid races.
	 * We cargo-cult like it's going out of fashion.
	 */
	if (r1 & IWX_CSR_INT_BIT_RX_PERIODIC) {
		handled |= IWX_CSR_INT_BIT_RX_PERIODIC;
		IWX_WRITE(sc, IWX_CSR_INT, IWX_CSR_INT_BIT_RX_PERIODIC);
		if ((r1 & (IWX_CSR_INT_BIT_FH_RX | IWX_CSR_INT_BIT_SW_RX)) == 0)
			IWX_WRITE_1(sc,
			    IWX_CSR_INT_PERIODIC_REG, IWX_CSR_INT_PERIODIC_DIS);
		isperiodic = 1;
	}

	if ((r1 & (IWX_CSR_INT_BIT_FH_RX | IWX_CSR_INT_BIT_SW_RX)) || isperiodic) {
		handled |= (IWX_CSR_INT_BIT_FH_RX | IWX_CSR_INT_BIT_SW_RX);
		IWX_WRITE(sc, IWX_CSR_FH_INT_STATUS, IWX_CSR_FH_INT_RX_MASK);

		iwx_notif_intr(sc);

		/* enable periodic interrupt, see above */
		if (r1 & (IWX_CSR_INT_BIT_FH_RX | IWX_CSR_INT_BIT_SW_RX) && !isperiodic)
			IWX_WRITE_1(sc, IWX_CSR_INT_PERIODIC_REG,
			    IWX_CSR_INT_PERIODIC_ENA);
	}

	if (__predict_false(r1 & ~handled))
		IWX_DPRINTF(sc, IWX_DEBUG_INTR,
		    "%s: unhandled interrupts: %x\n", __func__, r1);
	rv = 1;

 out_ena:
	iwx_restore_interrupts(sc);
 out:
	IWX_UNLOCK(sc);
	return;
}

/*
 * Autoconf glue-sniffing
 */
#define	PCI_VENDOR_INTEL		0x8086
#define	PCI_PRODUCT_INTEL_WL_22500_1	0x2723

static const struct iwx_devices {
	uint16_t		device;
	const struct iwx_cfg	*cfg;
} iwx_devices[] = {
	{ PCI_PRODUCT_INTEL_WL_22500_1, &iwx22000_cfg },
};

static int
iwx_probe(device_t dev)
{
	int i;

	for (i = 0; i < nitems(iwx_devices); i++) {
		if (pci_get_vendor(dev) == PCI_VENDOR_INTEL &&
		    pci_get_device(dev) == iwx_devices[i].device) {
			device_set_desc(dev, iwx_devices[i].cfg->name);
			return (BUS_PROBE_DEFAULT);
		}
	}

	return (ENXIO);
}

static int
iwx_dev_check(device_t dev)
{
	struct iwx_softc *sc;
	uint16_t devid;
	int i;

	sc = device_get_softc(dev);

	devid = pci_get_device(dev);
	for (i = 0; i < nitems(iwx_devices); i++) {
		if (iwx_devices[i].device == devid) {
			sc->cfg = iwx_devices[i].cfg;
			return (0);
		}
	}
	device_printf(dev, "unknown adapter type\n");
	return ENXIO;
}

/* PCI registers */
#define PCI_CFG_RETRY_TIMEOUT	0x041 /* shared with iwm */

static int
iwx_pci_attach(device_t dev)
{
	struct iwx_softc *sc;
	int count, error, rid;
	uint16_t reg;

	sc = device_get_softc(dev);

	/* We disable the RETRY_TIMEOUT register (0x41) to keep
	 * PCI Tx retries from interfering with C3 CPU state */
	pci_write_config(dev, PCI_CFG_RETRY_TIMEOUT, 0x00, 1);

	/* Enable bus-mastering and hardware bug workaround. */
	pci_enable_busmaster(dev);
	reg = pci_read_config(dev, PCIR_STATUS, sizeof(reg));
	/* if !MSI */
	if (reg & PCIM_STATUS_INTxSTATE) {
		reg &= ~PCIM_STATUS_INTxSTATE;
	}
	pci_write_config(dev, PCIR_STATUS, reg, sizeof(reg));

	rid = PCIR_BAR(0);
	sc->sc_mem = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid,
	    RF_ACTIVE);
	if (sc->sc_mem == NULL) {
		device_printf(sc->sc_dev, "can't map mem space\n");
		return (ENXIO);
	}
	sc->sc_st = rman_get_bustag(sc->sc_mem);
	sc->sc_sh = rman_get_bushandle(sc->sc_mem);

	/* Install interrupt handler. */
	count = 1;
	rid = 0;
	if (pci_alloc_msi(dev, &count) == 0)
		rid = 1;
	sc->sc_irq = bus_alloc_resource_any(dev, SYS_RES_IRQ, &rid, RF_ACTIVE |
	    (rid != 0 ? 0 : RF_SHAREABLE));
	if (sc->sc_irq == NULL) {
		device_printf(dev, "can't map interrupt\n");
			return (ENXIO);
	}
	error = bus_setup_intr(dev, sc->sc_irq, INTR_TYPE_NET | INTR_MPSAFE,
	    NULL, iwx_intr, sc, &sc->sc_ih);
	if (sc->sc_ih == NULL) {
		device_printf(dev, "can't establish interrupt");
			return (ENXIO);
	}
	sc->sc_dmat = bus_get_dma_tag(sc->sc_dev);

	return (0);
}

static void
iwx_pci_detach(device_t dev)
{
	struct iwx_softc *sc = device_get_softc(dev);

	if (sc->sc_irq != NULL) {
		bus_teardown_intr(dev, sc->sc_irq, sc->sc_ih);
		bus_release_resource(dev, SYS_RES_IRQ,
		    rman_get_rid(sc->sc_irq), sc->sc_irq);
		pci_release_msi(dev);
        }
	if (sc->sc_mem != NULL)
		bus_release_resource(dev, SYS_RES_MEMORY,
		    rman_get_rid(sc->sc_mem), sc->sc_mem);
}

static int
iwx_attach(device_t dev)
{
	struct iwx_softc *sc = device_get_softc(dev);
	struct ieee80211com *ic = &sc->sc_ic;
	int error;
	int txq_i, i;
	uint32_t hw_step;

	sc->sc_dev = dev;
	sc->sc_attached = 1;
	sc->sc_debug = IWX_DEBUG_ANY;
	IWX_LOCK_INIT(sc);
	mbufq_init(&sc->sc_snd, ifqmaxlen);
	callout_init_mtx(&sc->sc_watchdog_to, &sc->sc_mtx, 0);
#ifdef HAS_LED
	callout_init_mtx(&sc->sc_led_blink_to, &sc->sc_mtx, 0);
#endif
	TASK_INIT(&sc->sc_es_task, 0, iwx_endscan_cb, sc);

	error = iwx_dev_check(dev);
	if (error != 0)
		goto fail;

	sc->sc_notif_wait = iwx_notification_wait_init(sc);
	if (sc->sc_notif_wait == NULL) {
		device_printf(dev, "failed to init notification wait struct\n");
		goto fail;
	}

	sc->sf_state = IWX_SF_UNINIT;

	/* Init phy db */
	sc->sc_phy_db = iwx_phy_db_init(sc);
	if (!sc->sc_phy_db) {
		device_printf(dev, "Cannot init phy_db\n");
		goto fail;
	}

	/* Set EBS as successful as long as not stated otherwise by the FW. */
	sc->last_ebs_successful = TRUE;

	/* PCI attach */
	error = iwx_pci_attach(dev);
	if (error != 0)
		goto fail;

	sc->sc_wantresp = -1;
	
	/* source: OpenBSD */
	iwx_disable_interrupts(sc);

	sc->sc_hw_rev = IWX_READ(sc, IWX_CSR_HW_REV);
	/*
	 * In the 8000 HW family the format of the 4 bytes of CSR_HW_REV have
	 * changed, and now the revision step also includes bit 0-1 (no more
	 * "dash" value). To keep hw_rev backwards compatible - we'll store it
	 * in the old format.
	 */
	sc->sc_hw_rev = (sc->sc_hw_rev & 0xfff0) |
			(IWX_CSR_HW_REV_STEP(sc->sc_hw_rev << 2) << 2);

	if (iwx_prepare_card_hw(sc) != 0) {
		device_printf(dev, "could not initialize hardware\n");
		goto fail;
	}

	/*
	 * In order to recognize C step the driver should read the
	 * chip version id located at the AUX bus MISC address.
	 */
	IWX_SETBITS(sc, IWX_CSR_GP_CNTRL,
		    IWX_CSR_GP_CNTRL_REG_FLAG_INIT_DONE);
	DELAY(2);

	error = iwx_poll_bit(sc, IWX_CSR_GP_CNTRL,
			   IWX_CSR_GP_CNTRL_REG_FLAG_MAC_CLOCK_READY,
			   IWX_CSR_GP_CNTRL_REG_FLAG_MAC_CLOCK_READY,
			   25000);
	if (!error) {
		device_printf(sc->sc_dev,
		    "Failed to wake up the nic\n");
		goto fail;
	}

	if (iwx_nic_lock(sc)) {
		hw_step = iwx_read_prph(sc, IWX_WFPM_CTRL_REG);
		hw_step |= IWX_ENABLE_WFPM;
		iwx_write_prph(sc, IWX_WFPM_CTRL_REG, hw_step);
		hw_step = iwx_read_prph(sc, IWX_AUX_MISC_REG);
		hw_step = (hw_step >> IWX_HW_STEP_LOCATION_BITS) & 0xF;

		if (hw_step == 0x3)
			sc->sc_hw_rev = (sc->sc_hw_rev & 0xFFFFFFF3) |
					(IWX_SILICON_C_STEP << 2);
		iwx_nic_unlock(sc);
	} else {
		device_printf(sc->sc_dev, "Failed to lock the nic\n");
		goto fail;
	}

	/* Allocate DMA memory for firmware transfers. */
	if ((error = iwx_alloc_fwmem(sc)) != 0) {
		device_printf(dev, "could not allocate memory for firmware\n");
		goto fail;
	}

	/* We use ICT interrupts */
	if ((error = iwx_alloc_ict(sc)) != 0) {
		device_printf(dev, "could not allocate ICT table\n");
		goto fail;
	}

	/* Allocate TX scheduler "rings". */
	if ((error = iwx_alloc_sched(sc)) != 0) {
		device_printf(dev, "could not allocate TX scheduler rings\n");
		goto fail;
	}

	/* Allocate TX rings */
	for (txq_i = 0; txq_i < nitems(sc->txq); txq_i++) {
		if ((error = iwx_alloc_tx_ring(sc,
		    &sc->txq[txq_i], txq_i)) != 0) {
			device_printf(dev,
			    "could not allocate TX ring %d\n",
			    txq_i);
			goto fail;
		}
	}

	/* Allocate RX ring. */
	if ((error = iwx_alloc_rx_ring(sc, &sc->rxq)) != 0) {
		device_printf(dev, "could not allocate RX ring\n");
		goto fail;
	}

	/* Clear pending interrupts. */
	IWX_WRITE(sc, IWX_CSR_INT, 0xffffffff);

	ic->ic_softc = sc;
	ic->ic_name = device_get_nameunit(sc->sc_dev);
	ic->ic_phytype = IEEE80211_T_OFDM;	/* not only, but not used */
	ic->ic_opmode = IEEE80211_M_STA;	/* default to BSS mode */

	/* Set device capabilities. */
	ic->ic_caps =
	    IEEE80211_C_STA |
	    IEEE80211_C_WPA |		/* WPA/RSN */
	    IEEE80211_C_WME |
	    IEEE80211_C_PMGT |
	    IEEE80211_C_SHSLOT |	/* short slot time supported */
	    IEEE80211_C_SHPREAMBLE	/* short preamble supported */
//	    IEEE80211_C_BGSCAN		/* capable of bg scanning */
	    ;
	/* Advertise full-offload scanning */
	ic->ic_flags_ext = IEEE80211_FEXT_SCAN_OFFLOAD;
	for (i = 0; i < nitems(sc->sc_phyctxt); i++) {
		sc->sc_phyctxt[i].id = i;
		sc->sc_phyctxt[i].color = 0;
		sc->sc_phyctxt[i].ref = 0;
		sc->sc_phyctxt[i].channel = NULL;
	}

	/* Default noise floor */
	sc->sc_noise = -96;

	/* Max RSSI */
	sc->sc_max_rssi = IWX_MAX_DBM - IWX_MIN_DBM;

#ifdef IWX_DEBUG
	SYSCTL_ADD_INT(device_get_sysctl_ctx(dev),
	    SYSCTL_CHILDREN(device_get_sysctl_tree(dev)), OID_AUTO, "debug",
	    CTLFLAG_RW, &sc->sc_debug, IWX_DEBUG_ANY, "control debugging");
#endif

	error = iwx_read_firmware(sc);
	device_printf(sc->sc_dev, "%s: iwx_read_firmware() returned %d\n",
			__func__, error); /* todo if_iwx: remove before submitting for review */
	if (error) {
		goto fail;
	} else if (sc->sc_fw.fw_fp == NULL) {
		/*
		 * XXX Add a solution for properly deferring firmware load
		 *     during bootup.
		 */
		goto fail;
	} else {

		sc->sc_preinit_hook.ich_func = iwx_preinit;
		sc->sc_preinit_hook.ich_arg = sc;
		if (config_intrhook_establish(&sc->sc_preinit_hook) != 0) {
			device_printf(dev,
			    "config_intrhook_establish failed\n");
			goto fail;
		}
	}

	IWX_DPRINTF(sc, IWX_DEBUG_RESET | IWX_DEBUG_TRACE,
	    "<-%s\n", __func__);

	device_printf(sc->sc_dev, "%s: outside fail: label, returning\n",
			__func__); /* todo if_iwx: remove before submitting for review */
	return 0;

	/* Free allocated memory if something failed during attachment. */
fail:
	device_printf(sc->sc_dev, "%s: inside fail: label\n", __func__); /* todo if_iwx: remove before submitting for review */
	iwx_detach_local(sc, 0);
	device_printf(sc->sc_dev, "%s: inside fail: label, returning\n",
			__func__); /* todo if_iwx: remove before submitting for review */

	return ENXIO;
}

static int
iwx_is_valid_ether_addr(uint8_t *addr)
{
	char zero_addr[IEEE80211_ADDR_LEN] = { 0, 0, 0, 0, 0, 0 };

	if ((addr[0] & 1) || IEEE80211_ADDR_EQ(zero_addr, addr))
		return (FALSE);

	return (TRUE);
}

static int
iwx_wme_update(struct ieee80211com *ic)
{
#define IWX_EXP2(x)	((1 << (x)) - 1)	/* CWmin = 2^ECWmin - 1 */
	struct iwx_softc *sc = ic->ic_softc;
	struct chanAccParams chp;
	struct ieee80211vap *vap = TAILQ_FIRST(&ic->ic_vaps);
	struct iwx_vap *ivp = IWX_VAP(vap);
	struct iwx_node *in;
	struct wmeParams tmp[WME_NUM_AC];
	int aci, error;

	if (vap == NULL)
		return (0);

	ieee80211_wme_ic_getparams(ic, &chp);

	IEEE80211_LOCK(ic);
	for (aci = 0; aci < WME_NUM_AC; aci++)
		tmp[aci] = chp.cap_wmeParams[aci];
	IEEE80211_UNLOCK(ic);

	IWX_LOCK(sc);
	for (aci = 0; aci < WME_NUM_AC; aci++) {
		const struct wmeParams *ac = &tmp[aci];
		ivp->queue_params[aci].aifsn = ac->wmep_aifsn;
		ivp->queue_params[aci].cw_min = IWX_EXP2(ac->wmep_logcwmin);
		ivp->queue_params[aci].cw_max = IWX_EXP2(ac->wmep_logcwmax);
		ivp->queue_params[aci].edca_txop =
		    IEEE80211_TXOP_TO_US(ac->wmep_txopLimit);
	}
	ivp->have_wme = TRUE;
	if (ivp->is_uploaded && vap->iv_bss != NULL) {
		in = IWX_NODE(vap->iv_bss);
		if (in->in_assoc) {
			if ((error = iwx_mac_ctxt_changed(sc, vap)) != 0) {
				device_printf(sc->sc_dev,
				    "%s: failed to update MAC\n", __func__);
			}
		}
	}
	IWX_UNLOCK(sc);

	return (0);
#undef IWX_EXP2
}

static void
iwx_preinit(void *arg)
{
	struct iwx_softc *sc = arg;
	device_t dev = sc->sc_dev;
	struct ieee80211com *ic = &sc->sc_ic;
	int error;

	IWX_DPRINTF(sc, IWX_DEBUG_RESET | IWX_DEBUG_TRACE,
	    "->%s\n", __func__);

	IWX_LOCK(sc);

	if ((error = iwx_start_hw(sc)) != 0) {
		device_printf(dev, "could not initialize hardware\n");
		IWX_UNLOCK(sc);
		goto fail;
	}

	error = iwx_run_init_unified_ucode(sc, true);
	device_printf(sc->sc_dev, "%s: before iwx_stop_device\n",
			__func__);/* todo if_iwx: remove before submitting for review */

	iwx_stop_device(sc);
	device_printf(sc->sc_dev, "%s: after iwx_stop_device\n",
			__func__);/* todo if_iwx: remove before submitting for review */

	if (error) {
		IWX_UNLOCK(sc);
		goto fail;
	}
	device_printf(dev,
	    "hw rev 0x%x, fw ver %s, address %s\n",
	    sc->sc_hw_rev & IWX_CSR_HW_REV_TYPE_MSK,
	    sc->sc_fwver, ether_sprintf(sc->nvm_data->hw_addr));

	/* not all hardware can do 5GHz band */
	if (!sc->nvm_data->sku_cap_band_52GHz_enable)
		memset(&ic->ic_sup_rates[IEEE80211_MODE_11A], 0,
		    sizeof(ic->ic_sup_rates[IEEE80211_MODE_11A]));
	IWX_UNLOCK(sc);

	iwx_init_channel_map(ic, IEEE80211_CHAN_MAX, &ic->ic_nchans,
	    ic->ic_channels);

	/*
	 * At this point we've committed - if we fail to do setup,
	 * we now also have to tear down the net80211 state.
	 */
	ieee80211_ifattach(ic);
	ic->ic_vap_create = iwx_vap_create;
	ic->ic_vap_delete = iwx_vap_delete;
	ic->ic_raw_xmit = iwx_raw_xmit;
	ic->ic_node_alloc = iwx_node_alloc;
	ic->ic_scan_start = iwx_scan_start;
	ic->ic_scan_end = iwx_scan_end;
	ic->ic_update_mcast = iwx_update_mcast;
	ic->ic_getradiocaps = iwx_init_channel_map;
	ic->ic_set_channel = iwx_set_channel;
	ic->ic_scan_curchan = iwx_scan_curchan;
	ic->ic_scan_mindwell = iwx_scan_mindwell;
	ic->ic_wme.wme_update = iwx_wme_update;
	ic->ic_parent = iwx_parent;
	ic->ic_transmit = iwx_transmit;
	iwx_radiotap_attach(sc);
	if (bootverbose)
		ieee80211_announce(ic);

	IWX_DPRINTF(sc, IWX_DEBUG_RESET | IWX_DEBUG_TRACE,
	    "<-%s\n", __func__);
	config_intrhook_disestablish(&sc->sc_preinit_hook);

	device_printf(sc->sc_dev, "return from %s\n",
			__func__);/* todo if_iwx: remove before submitting for review */

	return;
fail:
	config_intrhook_disestablish(&sc->sc_preinit_hook);
	device_printf(sc->sc_dev, "%s: inside fail: label, before iwx_detach_local()\n",
			__func__);/* todo if_iwx: remove before submitting for review */

	iwx_detach_local(sc, 0);
	device_printf(sc->sc_dev, "%s: inside fail: label, after iwx_detach_local()\n",
			__func__);/* todo if_iwx: remove before submitting for review */

}

/*
 * Attach the interface to 802.11 radiotap.
 */
static void
iwx_radiotap_attach(struct iwx_softc *sc)
{
        struct ieee80211com *ic = &sc->sc_ic;

	IWX_DPRINTF(sc, IWX_DEBUG_RESET | IWX_DEBUG_TRACE,
	    "->%s begin\n", __func__);
        ieee80211_radiotap_attach(ic,
            &sc->sc_txtap.wt_ihdr, sizeof(sc->sc_txtap),
                IWX_TX_RADIOTAP_PRESENT,
            &sc->sc_rxtap.wr_ihdr, sizeof(sc->sc_rxtap),
                IWX_RX_RADIOTAP_PRESENT);
	IWX_DPRINTF(sc, IWX_DEBUG_RESET | IWX_DEBUG_TRACE,
	    "->%s end\n", __func__);
}

static struct ieee80211vap *
iwx_vap_create(struct ieee80211com *ic, const char name[IFNAMSIZ], int unit,
    enum ieee80211_opmode opmode, int flags,
    const uint8_t bssid[IEEE80211_ADDR_LEN],
    const uint8_t mac[IEEE80211_ADDR_LEN])
{
	struct iwx_vap *ivp;
	struct ieee80211vap *vap;

	if (!TAILQ_EMPTY(&ic->ic_vaps))         /* only one at a time */
		return NULL;
	ivp = malloc(sizeof(struct iwx_vap), M_80211_VAP, M_WAITOK | M_ZERO);
	vap = &ivp->iv_vap;
	ieee80211_vap_setup(ic, vap, name, unit, opmode, flags, bssid);
	vap->iv_bmissthreshold = 10;            /* override default */
	/* Override with driver methods. */
	ivp->iv_newstate = vap->iv_newstate;
	vap->iv_newstate = iwx_newstate;

	ivp->id = IWX_DEFAULT_MACID;
	ivp->color = IWX_DEFAULT_COLOR;

	ivp->have_wme = FALSE;
	ivp->ps_disabled = FALSE;

	ieee80211_ratectl_init(vap);
	/* Complete setup. */
	ieee80211_vap_attach(vap, iwx_media_change, ieee80211_media_status,
	    mac);
	ic->ic_opmode = opmode;

	return vap;
}

static void
iwx_vap_delete(struct ieee80211vap *vap)
{
	struct iwx_vap *ivp = IWX_VAP(vap);

	ieee80211_ratectl_deinit(vap);
	ieee80211_vap_detach(vap);
	free(ivp, M_80211_VAP);
}

static void
iwx_xmit_queue_drain(struct iwx_softc *sc)
{
	struct mbuf *m;
	struct ieee80211_node *ni;

	while ((m = mbufq_dequeue(&sc->sc_snd)) != NULL) {
		ni = (struct ieee80211_node *)m->m_pkthdr.rcvif;
		ieee80211_free_node(ni);
		m_freem(m);
	}
}

static void
iwx_scan_start(struct ieee80211com *ic)
{
	struct ieee80211vap *vap = TAILQ_FIRST(&ic->ic_vaps);
	struct iwx_softc *sc = ic->ic_softc;
	int error;

	IWX_LOCK(sc);
	if (sc->sc_flags & IWX_FLAG_SCAN_RUNNING) {
		/* This should not be possible */
		device_printf(sc->sc_dev,
		    "%s: Previous scan not completed yet\n", __func__);
	}
#ifdef not_in_iwx
	if (iwx_fw_has_capa(sc, IWX_UCODE_TLV_CAPA_UMAC_SCAN))
		error = iwx_umac_scan(sc);
	else
		error = iwx_lmac_scan(sc);
#endif
	error = iwx_umac_scan(sc);

	if (error != 0) {
		device_printf(sc->sc_dev, "could not initiate scan\n");
		IWX_UNLOCK(sc);
		ieee80211_cancel_scan(vap);
	} else {
		sc->sc_flags |= IWX_FLAG_SCAN_RUNNING;
#ifdef HAS_LED
		iwx_led_blink_start(sc);
#endif
		IWX_UNLOCK(sc);
	}
}

static void
iwx_scan_end(struct ieee80211com *ic)
{
#ifdef HAS_LED
	struct ieee80211vap *vap = TAILQ_FIRST(&ic->ic_vaps);
#endif
	struct iwx_softc *sc = ic->ic_softc;

	IWX_LOCK(sc);
#ifdef HAS_LED
	iwm_led_blink_stop(sc);
	if (vap->iv_state == IEEE80211_S_RUN)
		iwm_led_enable(sc);
#endif
	if (sc->sc_flags & IWX_FLAG_SCAN_RUNNING) {
		/*
		 * Removing IWM_FLAG_SCAN_RUNNING now, is fine because
		 * both iwm_scan_end and iwm_scan_start run in the ic->ic_tq
		 * taskqueue.
		 */
		sc->sc_flags &= ~IWX_FLAG_SCAN_RUNNING;
		iwx_scan_stop_wait(sc);
	}
	IWX_UNLOCK(sc);

	/*
	 * Make sure we don't race, if sc_es_task is still enqueued here.
	 * This is to make sure that it won't call ieee80211_scan_done
	 * when we have already started the next scan.
	 */
	taskqueue_cancel(ic->ic_tq, &sc->sc_es_task, NULL);
}

static void
iwx_update_mcast(struct ieee80211com *ic)
{
}

static void
iwx_set_channel(struct ieee80211com *ic)
{
}

static void
iwx_scan_curchan(struct ieee80211_scan_state *ss, unsigned long maxdwell)
{
}

static void
iwx_scan_mindwell(struct ieee80211_scan_state *ss)
{
}

void
iwx_init_task(void *arg1)
{
	struct iwx_softc *sc = arg1;

	IWX_LOCK(sc);
	while (sc->sc_flags & IWX_FLAG_BUSY)
		msleep(&sc->sc_flags, &sc->sc_mtx, 0, "iwxpwr", 0);
	sc->sc_flags |= IWX_FLAG_BUSY;
	iwx_stop(sc);
	if (sc->sc_ic.ic_nrunning > 0)
		iwx_init(sc);
	sc->sc_flags &= ~IWX_FLAG_BUSY;
	wakeup(&sc->sc_flags);
	IWX_UNLOCK(sc);
}

static int
iwx_resume(device_t dev)
{
	struct iwx_softc *sc = device_get_softc(dev);
	int do_reinit = 0;

	/*
	 * We disable the RETRY_TIMEOUT register (0x41) to keep
	 * PCI Tx retries from interfering with C3 CPU state.
	 */
	pci_write_config(dev, PCI_CFG_RETRY_TIMEOUT, 0x00, 1);

	if (!sc->sc_attached)
		return 0;

	iwx_init_task(device_get_softc(dev));

	IWX_LOCK(sc);
	if (sc->sc_flags & IWX_FLAG_SCANNING) {
		sc->sc_flags &= ~IWX_FLAG_SCANNING;
		do_reinit = 1;
	}
	IWX_UNLOCK(sc);

	if (do_reinit)
		ieee80211_resume_all(&sc->sc_ic);

	return 0;
}

static int
iwx_suspend(device_t dev)
{
	int do_stop = 0;
	struct iwx_softc *sc = device_get_softc(dev);

	do_stop = !! (sc->sc_ic.ic_nrunning > 0);

	if (!sc->sc_attached)
		return (0);

	ieee80211_suspend_all(&sc->sc_ic);

	if (do_stop) {
		IWX_LOCK(sc);
		iwx_stop(sc);
		sc->sc_flags |= IWX_FLAG_SCANNING;
		IWX_UNLOCK(sc);
	}

	return (0);
}


static int
iwx_detach_local(struct iwx_softc *sc, int do_net80211)
{
	struct iwx_fw_info *fw = &sc->sc_fw;
	device_t dev = sc->sc_dev;
	int i;

	if (!sc->sc_attached)
		return 0;
	sc->sc_attached = 0;
	if (do_net80211) {
		ieee80211_draintask(&sc->sc_ic, &sc->sc_es_task);
	}
	iwx_stop_device(sc);
	if (do_net80211) {
		IWX_LOCK(sc);
		iwx_xmit_queue_drain(sc);
		IWX_UNLOCK(sc);
		ieee80211_ifdetach(&sc->sc_ic);
	}
#ifdef HAS_LED
	callout_drain(&sc->sc_led_blink_to);
#endif
	callout_drain(&sc->sc_watchdog_to);

	iwx_phy_db_free(sc->sc_phy_db);
	sc->sc_phy_db = NULL;

	iwx_free_nvm_data(sc->nvm_data);

	/* Free descriptor rings */
	iwx_free_rx_ring(sc, &sc->rxq);
	for (i = 0; i < nitems(sc->txq); i++)
		iwx_free_tx_ring(sc, &sc->txq[i]);

	/* Free firmware */
	if (fw->fw_fp != NULL)
		iwx_fw_info_free(fw);

	/* Free scheduler */
	iwx_dma_contig_free(&sc->sched_dma);
	iwx_dma_contig_free(&sc->ict_dma);
	iwx_dma_contig_free(&sc->fw_dma);

	/* Finished with the hardware - detach things */
	iwx_pci_detach(dev);

	if (sc->sc_notif_wait != NULL) {
		iwx_notification_wait_free(sc->sc_notif_wait);
		sc->sc_notif_wait = NULL;
	}

	IWX_LOCK_DESTROY(sc);

	return (0);
}

static int
iwx_detach(device_t dev)
{
	struct iwx_softc *sc = device_get_softc(dev);

	return (iwx_detach_local(sc, 1));
}

static device_method_t iwx_pci_methods[] = {
        /* Device interface */
	DEVMETHOD(device_probe,         iwx_probe),
        DEVMETHOD(device_attach,        iwx_attach),
        DEVMETHOD(device_detach,        iwx_detach),
        DEVMETHOD(device_suspend,       iwx_suspend),
        DEVMETHOD(device_resume,        iwx_resume),

        DEVMETHOD_END
};

static driver_t iwx_pci_driver = {
        "iwx",
        iwx_pci_methods,
        sizeof (struct iwx_softc)
};

static devclass_t iwx_devclass;

DRIVER_MODULE(iwx, pci, iwx_pci_driver, iwx_devclass, NULL, NULL);
MODULE_PNP_INFO("U16:device;P:#;T:vendor=0x8086", pci, iwx_pci_driver,
    iwx_devices, nitems(iwx_devices));
MODULE_DEPEND(iwx, firmware, 1, 1, 1);
MODULE_DEPEND(iwx, pci, 1, 1, 1);
MODULE_DEPEND(iwx, wlan, 1, 1, 1);
