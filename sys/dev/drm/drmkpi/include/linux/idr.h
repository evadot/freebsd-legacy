/*-
 * Copyright (c) 2010 Isilon Systems, Inc.
 * Copyright (c) 2010 iX Systems, Inc.
 * Copyright (c) 2010 Panasas, Inc.
 * Copyright (c) 2013-2016 Mellanox Technologies, Ltd.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice unmodified, this list of conditions, and the following
 *    disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * $FreeBSD$
 */
#ifndef	_LINUX_IDR_H_
#define	_LINUX_IDR_H_

#include <sys/param.h>
#include <sys/lock.h>
#include <sys/mutex.h>

#include <linux/types.h>

#define	IDR_BITS	5
#define	IDR_SIZE	(1 << IDR_BITS)
#define	IDR_MASK	(IDR_SIZE - 1)

#define	MAX_ID_SHIFT	((sizeof(int) * NBBY) - 1)
#define	MAX_ID_BIT	(1U << MAX_ID_SHIFT)
#define	MAX_ID_MASK	(MAX_ID_BIT - 1)
#define	MAX_LEVEL	(MAX_ID_SHIFT + IDR_BITS - 1) / IDR_BITS

#define MAX_IDR_SHIFT (sizeof(int)*8 - 1)
#define MAX_IDR_BIT (1U << MAX_IDR_SHIFT)
#define MAX_IDR_MASK (MAX_IDR_BIT - 1)

struct idr_layer {
	unsigned long		bitmap;
	struct idr_layer	*ary[IDR_SIZE];
};

struct idr {
	struct mtx		lock;
	struct idr_layer	*top;
	struct idr_layer	*free;
	int			layers;
	int			next_cyclic_id;
};

/* NOTE: It is the applications responsibility to destroy the IDR */
#define	DEFINE_IDR(name)						\
	struct idr name;						\
	SYSINIT(name##_idr_sysinit, SI_SUB_DRIVERS, SI_ORDER_FIRST,	\
	    idr_init, &(name))

/* NOTE: It is the applications responsibility to destroy the IDA */
#define	DEFINE_IDA(name)						\
	struct ida name;						\
	SYSINIT(name##_ida_sysinit, SI_SUB_DRIVERS, SI_ORDER_FIRST,	\
	    ida_init, &(name))

#define	idr_preload	drmkpi_idr_preload
void	drmkpi_idr_preload(gfp_t gfp_mask);
#define	idr_preload_end	drmkpi_idr_preload_end
void	drmkpi_idr_preload_end(void);
#define	idr_find	drmkpi_idr_find
void	*drmkpi_idr_find(struct idr *idp, int id);
#define	idr_get_next	drmkpi_idr_get_next
void	*drmkpi_idr_get_next(struct idr *idp, int *nextid);
#define	idr_is_empty	drmkpi_idr_is_empty
bool	drmkpi_idr_is_empty(struct idr *idp);
#define	idr_pre_get	drmkpi_idr_pre_get
int	drmkpi_idr_pre_get(struct idr *idp, gfp_t gfp_mask);
#define	idr_get_new	drmkpi_idr_get_new
int	drmkpi_idr_get_new(struct idr *idp, void *ptr, int *id);
#define	idr_get_new_above	drmkpi_idr_get_new_above
int	drmkpi_idr_get_new_above(struct idr *idp, void *ptr, int starting_id, int *id);
#define	idr_replace	drmkpi_idr_replace
void	*drmkpi_idr_replace(struct idr *idp, void *ptr, int id);
#define	idr_remove	drmkpi_idr_remove
void	*drmkpi_idr_remove(struct idr *idp, int id);
#define	idr_remove_all	drmkpi_idr_remove_all
void	drmkpi_idr_remove_all(struct idr *idp);
#define	idr_destroy	drmkpi_idr_destroy
void	drmkpi_idr_destroy(struct idr *idp);
#define	idr_init	drmkpi_idr_init
void	drmkpi_idr_init(struct idr *idp);
#define	idr_alloc drmkpi_idr_alloc
int	drmkpi_idr_alloc(struct idr *idp, void *ptr, int start, int end, gfp_t);
#define	idr_alloc_cyclic drmkpi_idr_alloc_cyclic
int	drmkpi_idr_alloc_cyclic(struct idr *idp, void *ptr, int start, int end, gfp_t);
#define	idr_for_each	drmkpi_idr_for_each
int	drmkpi_idr_for_each(struct idr *idp, int (*fn)(int id, void *p, void *data), void *data);

#define	idr_for_each_entry(idp, entry, id)	\
	for ((id) = 0; ((entry) = idr_get_next(idp, &(id))) != NULL; ++(id))

#define	IDA_CHUNK_SIZE		128	/* 128 bytes per chunk */
#define	IDA_BITMAP_LONGS	(IDA_CHUNK_SIZE / sizeof(long) - 1)
#define	IDA_BITMAP_BITS		(IDA_BITMAP_LONGS * sizeof(long) * 8)

struct ida_bitmap {
	long			nr_busy;
	unsigned long		bitmap[IDA_BITMAP_LONGS];
};

struct ida {
	struct idr		idr;
	struct ida_bitmap	*free_bitmap;
};

#define	ida_pre_get	drmkpi_ida_pre_get
int	drmkpi_ida_pre_get(struct ida *ida, gfp_t gfp_mask);
#define	ida_pre_get_new_above	drmkpi_ida_get_new_above
int	drmkpi_ida_get_new_above(struct ida *ida, int starting_id, int *p_id);
#define	ida_remove	drmkpi_ida_remove
void	drmkpi_ida_remove(struct ida *ida, int id);
#define	ida_destroy	drmkpi_ida_destroy
void	drmkpi_ida_destroy(struct ida *ida);
#define	ida_init	drmkpi_ida_init
void	drmkpi_ida_init(struct ida *ida);

#define	ida_simple_get	drmkpi_ida_simple_get
int	drmkpi_ida_simple_get(struct ida *ida, unsigned int start, unsigned int end,
    gfp_t gfp_mask);
#define	ida_simple_remove	drmkpi_ida_simple_remove
void	drmkpi_ida_simple_remove(struct ida *ida, unsigned int id);

static inline int
ida_get_new(struct ida *ida, int *p_id)
{
	return (drmkpi_ida_get_new_above(ida, 0, p_id));
}

#endif	/* _LINUX_IDR_H_ */
