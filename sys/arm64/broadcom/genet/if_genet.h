
#define	TX_DESC_COUNT		GENET_DMA_DESC_COUNT
#define	RX_DESC_COUNT		GENET_DMA_DESC_COUNT

#define	TX_NEXT(n, count)		(((n) + 1) & ((count) - 1))
#define	RX_NEXT(n, count)		(((n) + 1) & ((count) - 1))

#define	TX_MAX_SEGS		20

enum {
	_RES_MAC,		/* what to call this? */
	_RES_IRQ1,
	_RES_IRQ2,
	_RES_NITEMS
};

/* structure per ring entry */
struct gen_ring_ent {
	bus_dmamap_t		map;
	struct mbuf		*mbuf;
};

struct tx_queue {
	int			hwindex;		/* hardware index */
	int			nentries;
	u_int			queued;			/* or avail? */
	u_int			cur;
	u_int			next;
	u_int			prod_idx;
	u_int			cons_idx;
	struct gen_ring_ent	*entries;
};

struct rx_queue {
	int			hwindex;		/* hardware index */
	int			nentries;
	u_int			cur;
	u_int			prod_idx;
	u_int			cons_idx;
	struct gen_ring_ent	*entries;
};

struct gen_softc {
	struct resource		*res[_RES_NITEMS];
	struct mtx		mtx;
	if_t			ifp;
	device_t		dev;
	device_t		miibus;
	mii_contype_t		phy_mode;
	struct ether_addr	eaddr;
	bool			eaddr_found;

	struct callout		stat_ch;
	struct task		link_task;
	void			*ih;
	void			*ih2;
	int			type;
	int			if_flags;
	int			link;
	bus_dma_tag_t		tx_buf_tag;
	/*
	 * The genet chip has multiple queues for transmit and receive.
	 * This driver uses only one (queue 16, the default), but is cast
	 * with multiple rings.  The additional rings are used for different
	 * priorities.
	 */
#define DEF_TXQUEUE	0
#define NTXQUEUE	1
	struct tx_queue		tx_queue[NTXQUEUE];
	struct gen_ring_ent	tx_ring_ent[TX_DESC_COUNT];  /* ring entries */

	bus_dma_tag_t		rx_buf_tag;
#define DEF_RXQUEUE	0
#define NRXQUEUE	1
	struct rx_queue		rx_queue[NRXQUEUE];
	struct gen_ring_ent	rx_ring_ent[RX_DESC_COUNT];  /* ring entries */
};

int genet_miibus_readreg(device_t dev, int phy, int reg);
int genet_miibus_writereg(device_t dev, int phy, int reg, int val);
void genet_miibus_statchg(device_t dev);
int genet_attach(struct gen_softc *sc);
void gen_destroy(struct gen_softc *sc);
