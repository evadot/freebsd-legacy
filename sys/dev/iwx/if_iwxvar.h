/* copied from FreeBSD sys/dev/iwm/if_iwmvar.h */

/*	$OpenBSD: if_iwmvar.h,v 1.7 2015/03/02 13:51:10 jsg Exp $	*/
/*	$FreeBSD$ */

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

struct iwx_rx_radiotap_header {
	struct ieee80211_radiotap_header wr_ihdr;
	uint64_t	wr_tsft;
	uint8_t		wr_flags;
	uint8_t		wr_rate;
	uint16_t	wr_chan_freq;
	uint16_t	wr_chan_flags;
	int8_t		wr_dbm_antsignal;
	int8_t		wr_dbm_antnoise;
} __packed __aligned(8);

#define IWX_RX_RADIOTAP_PRESENT						\
	((1 << IEEE80211_RADIOTAP_TSFT) |				\
	 (1 << IEEE80211_RADIOTAP_FLAGS) |				\
	 (1 << IEEE80211_RADIOTAP_RATE) |				\
	 (1 << IEEE80211_RADIOTAP_CHANNEL) |				\
	 (1 << IEEE80211_RADIOTAP_DBM_ANTSIGNAL) |			\
	 (1 << IEEE80211_RADIOTAP_DBM_ANTNOISE))

struct iwx_tx_radiotap_header {
	struct ieee80211_radiotap_header wt_ihdr;
	uint8_t		wt_flags;
	uint8_t		wt_rate;
	uint16_t	wt_chan_freq;
	uint16_t	wt_chan_flags;
} __packed;

#define IWX_TX_RADIOTAP_PRESENT						\
	((1 << IEEE80211_RADIOTAP_FLAGS) |				\
	 (1 << IEEE80211_RADIOTAP_RATE) |				\
	 (1 << IEEE80211_RADIOTAP_CHANNEL))

#define IWX_UCODE_SECTION_MAX 39

/**
 * enum iwm_ucode_type
 *
 * The type of ucode.
 *
 * @IWX_UCODE_REGULAR: Normal runtime ucode
 * @IWM_UCODE_INIT: Initial ucode
 * @IWM_UCODE_WOWLAN: Wake on Wireless enabled ucode
 * @IWM_UCODE_REGULAR_USNIFFER: Normal runtime ucode when using usniffer image
 */
enum iwx_ucode_type {
	IWX_UCODE_REGULAR,
	IWX_UCODE_INIT,
	IWX_UCODE_WOWLAN,
	IWX_UCODE_REGULAR_USNIFFER,
	IWX_UCODE_TYPE_MAX
};

struct iwx_ucode_capabilities {
	// sc_capa_max_probe_len in iwx_softc @ openbsd
	uint32_t max_probe_length;
	// sc_capa_n_scan_channels @ openbsd
	uint32_t n_scan_channels;
	// sc_capa_flags @ openbsd
	uint32_t flags;
	// sc_ucode_api @ openbsd
	uint8_t enabled_api[howmany(IWX_NUM_UCODE_TLV_API, NBBY)];
	// sc_enabled_capa in iwx_softc 
	uint8_t enabled_capa[howmany(IWX_NUM_UCODE_TLV_CAPA, NBBY)];
};

/* one for each uCode image (inst/data, init/runtime/wowlan) */
struct iwx_fw_desc { // iwx_fw_onesect in openbsd
	const void *data;	/* vmalloc'ed data */
	uint32_t len;		/* size in bytes */
	uint32_t offset;	/* offset in the device */
};

struct iwx_fw_img { // iwx_fw_sects in openbsd
	struct iwx_fw_desc fw_sect[IWX_UCODE_SECTION_MAX];
	int fw_count;
	int is_dual_cpus;
//	uint32_t paging_mem_size;
};

//#define IWX_FW_DBG_CONF_MAX 32

//enum iwx_fw_dbg_trigger {
	/* todo if_iwx: import other entries from openbsd */
	/* must be last */
//	IWX_FW_DBG_TRIGGER_MAX,
//};

/* source: OpenBSD */
struct iwx_fw_dbg {
	/* FW debug data parsed for driver usage */
	int dbg_dest_tlv_init;
	uint8_t *dbg_dest_ver;
	uint8_t n_dest_reg;
	struct iwx_fw_dbg_dest_tlv_v1 *dbg_dest_tlv_v1;

	struct iwx_fw_dbg_conf_tlv *dbg_conf_tlv[IWX_FW_DBG_CONF_MAX];
	size_t dbg_conf_tlv_len[IWX_FW_DBG_CONF_MAX];
	struct iwx_fw_dbg_trigger_tlv *dbg_trigger_tlv[IWX_FW_DBG_TRIGGER_MAX];
	size_t dbg_trigger_tlv_len[IWX_FW_DBG_TRIGGER_MAX];
	struct iwx_fw_dbg_mem_seg_tlv *dbg_mem_tlv;
	size_t n_mem_tlv;
};

struct iwx_fw_info {
	const struct firmware *fw_fp;

	/* ucode images */
	struct iwx_fw_img img[IWX_UCODE_TYPE_MAX];

	struct iwx_ucode_capabilities ucode_capa;

	uint32_t phy_config;
	uint8_t valid_tx_ant;
	uint8_t valid_rx_ant;
	struct iwx_fw_dbg dbg;
};

struct iwx_nvm_data {
	int n_hw_addrs;
	uint8_t hw_addr[IEEE80211_ADDR_LEN];

	int sku_cap_band_24GHz_enable;
	int sku_cap_band_52GHz_enable;
	int sku_cap_11n_enable;
	int sku_cap_amt_enable;
	int sku_cap_ipan_enable;

	uint8_t radio_cfg_type;
	uint8_t radio_cfg_step;
	uint8_t radio_cfg_dash;
	uint8_t radio_cfg_pnum;
	uint8_t valid_tx_ant, valid_rx_ant;
/* todo if_iwx: sync with openbsd */
#define IWX_NUM_CHANNELS	51
	uint16_t nvm_version;
	uint8_t max_tx_pwr_half_dbm;

	uint16_t nvm_ch_flags[];
};

/* max bufs per tfd the driver will use */
#define IWX_MAX_CMD_TBS_PER_TFD 2

struct iwx_rx_packet;
struct iwx_host_cmd {
	const void *data[IWX_MAX_CMD_TBS_PER_TFD];
	struct iwx_rx_packet *resp_pkt;
	unsigned long _rx_page_addr;
	uint32_t _rx_page_order;
	int handler_status;

	uint32_t flags;
	uint32_t id;
	uint16_t len[IWX_MAX_CMD_TBS_PER_TFD];
	uint8_t dataflags[IWX_MAX_CMD_TBS_PER_TFD];
};

/*
 * DMA glue is from iwn
 */

typedef caddr_t iwm_caddr_t;
typedef void *iwm_hookarg_t;

struct iwx_dma_info {
	bus_dma_tag_t		tag;
	bus_dmamap_t		map;
	bus_dma_segment_t	seg;
	bus_addr_t		paddr;
	void 			*vaddr;
	bus_size_t		size;
};

struct iwx_txq {
	// todo
};

#define	IWX_DEFAULT_QUEUE_SIZE	IWX_TFD_QUEUE_SIZE_MAX

#define IWX_TX_RING_COUNT	IWX_DEFAULT_QUEUE_SIZE
#define IWX_TX_RING_LOMARK	192
#define IWX_TX_RING_HIMARK	224

struct iwx_tx_data {
	bus_dmamap_t		map;
	bus_addr_t		cmd_paddr;
//	bus_addr_t		scratch_paddr;
	struct mbuf		*m;
	struct iwx_node 	*in;
	int			done;
};

struct iwx_tx_ring {
	struct iwx_dma_info	desc_dma;
	struct iwx_dma_info	cmd_dma;
	struct iwx_dma_info	bc_tbl;
	struct iwx_tfh_tfd	*desc;
	struct iwx_device_cmd	*cmd;
	bus_dma_tag_t		data_dmat;
	struct iwx_tx_data	data[IWX_TX_RING_COUNT];
	int			qid;
	int			queued;
	int			cur;
	// openbsd adds: int			tail;
};

#define IWX_RX_MQ_RING_COUNT		512

#define IWX_RBUF_SIZE		4096

#define IWX_MAX_SCATTER		20

struct iwx_rx_data {
	struct mbuf	*m;
	bus_dmamap_t	map;
};

struct iwx_rx_ring {
	struct iwx_dma_info	free_desc_dma;
	struct iwx_dma_info	used_desc_dma;
	struct iwx_dma_info	stat_dma;
	struct iwx_dma_info	buf_dma;
	void			*desc;
	struct iwx_rb_status	*stat;
	struct iwx_rx_data	data[IWX_RX_MQ_RING_COUNT];
	bus_dmamap_t		spare_map;	/* for iwm_rx_addbuf() */
	bus_dma_tag_t           data_dmat;
	int			cur;
};

#define IWX_ERROR_EVENT_TABLE_LMAC1	(1 << 0)
#define IWX_ERROR_EVENT_TABLE_LMAC2	(1 << 1)
#define IWX_ERROR_EVENT_TABLE_UMAC	(1 << 2)

#define IWM_CMD_RESP_MAX PAGE_SIZE

#define IWX_TE_SESSION_PROTECTION_MAX_TIME_MS 500
#define IWM_TE_SESSION_PROTECTION_MIN_TIME_MS 400

/*
 * Command headers are in iwl-trans.h, which is full of all
 * kinds of other junk, so we just replicate the structures here.
 * First the software bits:
 */
enum IWX_CMD_MODE {
	IWX_CMD_SYNC		= 0,
	IWX_CMD_ASYNC		= (1 << 0),
	IWX_CMD_WANT_SKB	= (1 << 1), // IWX_CMD_WANT_RESP @ openbsd
	IWX_CMD_SEND_IN_RFKILL	= (1 << 2),
};
enum iwm_hcmd_dataflag {
	IWM_HCMD_DFL_NOCOPY     = (1 << 0),
	IWM_HCMD_DFL_DUP        = (1 << 1),
};

struct iwx_int_sta {
	uint32_t sta_id;
	uint32_t tfd_queue_msk;
};

struct iwx_phy_ctxt {
	uint16_t id;
	uint16_t color;
	uint32_t ref;
	struct ieee80211_channel *channel;
};

struct iwm_bf_data {
	int bf_enabled;		/* filtering	*/
	int ba_enabled;		/* abort	*/
	int ave_beacon_signal;
	int last_cqm_event;
};

/* source: OpenBSD */
/**
 * struct iwx_self_init_dram - dram data used by self init process
 * @fw: lmac and umac dram data
 * @fw_cnt: total number of items in array
 * @paging: paging dram data
 * @paging_cnt: total number of items in array
 */
struct iwx_self_init_dram {
	struct iwx_dma_info *fw;
	int fw_cnt;
	struct iwx_dma_info *paging;
	int paging_cnt;
};

struct iwx_vap {
	struct ieee80211vap	iv_vap;
	int			is_uploaded;
	int			iv_auth;

	int			(*iv_newstate)(struct ieee80211vap *,
				    enum ieee80211_state, int);

	struct iwx_phy_ctxt	*phy_ctxt;

	uint16_t		id;
	uint16_t		color;

	boolean_t		have_wme;
	/*
	 * QoS data from net80211, need to store this here
	 * as net80211 has a separate callback but we need
	 * to have the data for the MAC context
	 */
        struct {
		uint16_t cw_min;
		uint16_t cw_max;
		uint16_t edca_txop;
		uint8_t aifsn;
	} queue_params[WME_NUM_AC];

	/* indicates that this interface requires PS to be disabled */
	boolean_t		ps_disabled;
};
#define IWX_VAP(_vap)		((struct iwx_vap *)(_vap))

struct iwx_node {
	struct ieee80211_node	in_ni;

	/* status "bits" */
	int			in_assoc;

	struct iwx_lq_cmd	in_lq;
};
#define IWX_NODE(_ni)		((struct iwx_node *)(_ni))

#define IWX_STATION_ID 0
#define IWX_AUX_STA_ID 1
// #define IWX_MONITOR_STA_ID 2

#define	IWX_DEFAULT_MACID	0
#define	IWX_DEFAULT_COLOR	0
#define	IWX_DEFAULT_TSFID	0

#define IWX_ICT_SIZE		4096
#define IWX_ICT_COUNT		(IWX_ICT_SIZE / sizeof (uint32_t))
#define IWX_ICT_PADDR_SHIFT	12

struct iwx_cfg;

struct iwx_softc {
	device_t		sc_dev;
	uint32_t		sc_debug;
	int			sc_attached;

	struct mtx		sc_mtx;
	struct mbufq		sc_snd;
	struct ieee80211com	sc_ic;
	struct ieee80211_ratectl_tx_status sc_txs;

	int			sc_flags;
/* these #defines are shared with iwm */
#define IWX_FLAG_USE_ICT	(1 << 0)
#define IWX_FLAG_HW_INITED	(1 << 1)
#define IWX_FLAG_STOPPED	(1 << 2)
#define IWX_FLAG_RFKILL		(1 << 3)
#define IWX_FLAG_BUSY		(1 << 4) // IWX_FLAG_MAC_ACTIVE
#define IWX_FLAG_SCANNING	(1 << 5) // IWX_FLAG_BINDING_ACTIVE
#define IWX_FLAG_SCAN_RUNNING	(1 << 6) // IWX_FLAG_STA_ACTIVE
#define IWX_FLAG_TE_ACTIVE	(1 << 7)
// new
#define IWX_FLAG_HW_ERR		(1 << 8)
#define IWX_FLAG_SHUTDOWN	(1 << 9)
#define IWX_FLAG_BGSCAN		(1 << 10)

	struct intr_config_hook sc_preinit_hook;
	struct callout		sc_watchdog_to;
#ifdef HAS_LED
	struct callout		sc_led_blink_to;
#endif

	struct task		init_task;

	struct resource		*sc_irq;
	struct resource		*sc_mem;
	bus_space_tag_t		sc_st;
	bus_space_handle_t	sc_sh;
	bus_size_t		sc_sz;
	bus_dma_tag_t		sc_dmat;
	void			*sc_ih;

	/* TX scheduler rings. */
	struct iwx_dma_info	sched_dma;
	uint32_t		scd_base_addr;

	/* TX/RX rings. */
	struct iwx_tx_ring	txq[IWX_MAX_QUEUES];
	struct iwx_rx_ring	rxq;
	int			qfullmsk;

	/* ICT table. */
	struct iwx_dma_info	ict_dma;
	int			ict_cur;

	int			sc_hw_rev;
	int			sc_hw_id;

	struct iwx_dma_info	fw_dma;

	unsigned long queue_stopped[IWX_MAX_TVQM_QUEUES];
	unsigned long queue_used[IWX_MAX_TVQM_QUEUES];

	uint8_t			cmd_queue;

	struct iwx_dma_info	ctxt_info_dma;
	struct iwx_self_init_dram init_dram;

	int			sc_fw_chunk_done;

//	enum iwx_ucode_type	cur_ucode;
	int			ucode_loaded;
	char			sc_fwver[32];

//	char			sc_fw_mcc[3];
#define IWX_MAX_FW_CMD_VERSIONS	64
	struct iwx_fw_cmd_version cmd_versions[IWX_MAX_FW_CMD_VERSIONS];
	int n_cmd_versions;

	int			sc_intmask;

	/*
	 * So why do we need a separate stopped flag and a generation?
	 * the former protects the device from issuing commands when it's
	 * stopped (duh).  The latter protects against race from a very
	 * fast stop/unstop cycle where threads waiting for responses do
	 * not have a chance to run in between.  Notably: we want to stop
	 * the device from interrupt context when it craps out, so we
	 * don't have the luxury of waiting for quiescense.
	 */
	int			sc_generation;

	struct iwx_fw_info	sc_fw;
	struct iwx_dma_info	fw_mon;
	struct iwx_tlv_calib_ctrl sc_default_calib[IWX_UCODE_TYPE_MAX];

	const struct iwx_cfg	*cfg;
	struct iwx_nvm_data	*nvm_data; // sc_nvm?
	struct iwx_phy_db	*sc_phy_db;

	struct iwm_bf_data	sc_bf; // does the openbsd driver actually use iwx_bf_data?

	int			sc_tx_timer;

	int			sc_scan_last_antenna;

	int			sc_fixed_ridx;

	int			sc_staid;
	int			sc_nodecolor;

	uint8_t			sc_cmd_resp[IWM_CMD_RESP_MAX];
	int			sc_wantresp;

	struct task		sc_es_task;

	struct iwx_rx_phy_info	sc_last_phy_info;
	int			sc_ampdu_ref;

	struct iwx_int_sta	sc_aux_sta;

	/* phy contexts.  we only use the first one */
	struct iwx_phy_ctxt	sc_phyctxt[IWX_NUM_PHY_CTX];

	/* todo if_iwx: update struct to v10 */
	struct iwx_notif_statistics_v13 sc_stats;
	int			sc_noise;

	struct iwx_rx_radiotap_header sc_rxtap;
	struct iwx_tx_radiotap_header sc_txtap;

	int			sc_max_rssi;

	struct iwx_notif_wait_data *sc_notif_wait;

	int			cmd_hold_nic_awake;

	/* Firmware status */
	// uc_lmac_error_event_table in iwx_softc @ openbsd
	uint32_t		lmac_error_event_table[2];
	// uc_log_event_table in iwx_softc @ openbsd
	uint32_t		log_event_table;
	// uc_umac_error_event_table in iwx_softc @ openbsd
	uint32_t		umac_error_event_table;
	int			support_umac_log;
	int			error_event_table_tlv_status;

	boolean_t		last_ebs_successful;

	/* last smart fifo state that was successfully sent to firmware */
	enum iwx_sf_state	sf_state;

	/* Indicate if device power save is allowed */
	boolean_t		sc_ps_disabled;

	int			sc_ltr_enabled;

	int			sc_tx_with_siso_diversity;

	/* Track firmware state for STA association. */
	int			sc_firmware_state;

	/* Unique ID (assigned by the firmware) of the current Time Event. */
	uint32_t		sc_time_event_uid;

	/* Duration of the Time Event in TU. */
	uint32_t		sc_time_event_duration;

	/* Expected end of the Time Event in HZ ticks. */
	int			sc_time_event_end_ticks;
};

#define IWX_LOCK_INIT(_sc) \
	mtx_init(&(_sc)->sc_mtx, device_get_nameunit((_sc)->sc_dev), \
	    MTX_NETWORK_LOCK, MTX_DEF);
#define	IWX_LOCK(_sc)		mtx_lock(&(_sc)->sc_mtx)
#define	IWX_UNLOCK(_sc)		mtx_unlock(&(_sc)->sc_mtx)
#define IWX_LOCK_DESTROY(_sc)	mtx_destroy(&(_sc)->sc_mtx)

static inline bool
iwx_fw_has_api(struct iwx_softc *sc, unsigned int api)
{
	return isset(sc->sc_fw.ucode_capa.enabled_api, api);
}

static inline bool
iwx_fw_has_capa(struct iwx_softc *sc, unsigned int capa)
{
	return isset(sc->sc_fw.ucode_capa.enabled_capa, capa);
}
