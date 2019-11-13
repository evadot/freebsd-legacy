/*-
 * Copyright (c) 2010 Isilon Systems, Inc.
 * Copyright (c) 2010 iX Systems, Inc.
 * Copyright (c) 2010 Panasas, Inc.
 * Copyright (c) 2013-2018 Mellanox Technologies, Ltd.
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
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/malloc.h>
#include <sys/kernel.h>
#include <sys/sysctl.h>
#include <sys/proc.h>
#include <sys/sglist.h>
#include <sys/sleepqueue.h>
#include <sys/lock.h>
#include <sys/mutex.h>
#include <sys/bus.h>
#include <sys/fcntl.h>
#include <sys/file.h>
#include <sys/filio.h>
#include <sys/rwlock.h>
#include <sys/mman.h>

#include <vm/vm.h>
#include <vm/pmap.h>
#include <vm/vm_object.h>
#include <vm/vm_page.h>
#include <vm/vm_pager.h>

#include <machine/stdarg.h>

#if defined(__i386__) || defined(__amd64__)
#include <machine/md_var.h>
#endif

#include <linux/kobject.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/cdev.h>
#include <linux/file.h>
#include <linux/sysfs.h>
#include <linux/mm.h>
#include <linux/io.h>
#include <linux/vmalloc.h>
#include <linux/timer.h>
#include <linux/interrupt.h>
#include <linux/uaccess.h>
#include <linux/list.h>
#include <linux/kthread.h>
#include <linux/kernel.h>
#include <linux/compat.h>
#include <linux/poll.h>
#include <linux/smp.h>
#include <linux/preempt.h>

#if defined(__i386__) || defined(__amd64__)
#include <asm/smp.h>
#endif

SYSCTL_NODE(_compat, OID_AUTO, linuxkpi, CTLFLAG_RW, 0, "LinuxKPI parameters");

MALLOC_DEFINE(M_KMALLOC, "linux", "Linux kmalloc compat");

#if 0
#include <linux/rbtree.h>
#undef RB_ROOT
#define	RB_ROOT(head)	(head)->rbh_root
#endif

/* Undo Linux compat changes. */


unsigned long linux_timer_hz_mask;

#if 0
int
panic_cmp(struct rb_node *one, struct rb_node *two)
{
	panic("no cmp");
}

RB_GENERATE(linux_root, rb_node, __entry, panic_cmp);
#endif

int
kobject_set_name_vargs(struct kobject *kobj, const char *fmt, va_list args)
{
	va_list tmp_va;
	int len;
	char *old;
	char *name;
	char dummy;

	old = kobj->name;

	if (old && fmt == NULL)
		return (0);

	/* compute length of string */
	va_copy(tmp_va, args);
	len = vsnprintf(&dummy, 0, fmt, tmp_va);
	va_end(tmp_va);

	/* account for zero termination */
	len++;

	/* check for error */
	if (len < 1)
		return (-EINVAL);

	/* allocate memory for string */
	name = kzalloc(len, GFP_KERNEL);
	if (name == NULL)
		return (-ENOMEM);
	vsnprintf(name, len, fmt, args);
	kobj->name = name;

	/* free old string */
	kfree(old);

	/* filter new string */
	for (; *name != '\0'; name++)
		if (*name == '/')
			*name = '!';
	return (0);
}

int
kobject_set_name(struct kobject *kobj, const char *fmt, ...)
{
	va_list args;
	int error;

	va_start(args, fmt);
	error = kobject_set_name_vargs(kobj, fmt, args);
	va_end(args);

	return (error);
}

static int
kobject_add_complete(struct kobject *kobj, struct kobject *parent)
{
	const struct kobj_type *t;
	int error;

	kobj->parent = parent;
	error = sysfs_create_dir(kobj);
	if (error == 0 && kobj->ktype && kobj->ktype->default_attrs) {
		struct attribute **attr;
		t = kobj->ktype;

		for (attr = t->default_attrs; *attr != NULL; attr++) {
			error = sysfs_create_file(kobj, *attr);
			if (error)
				break;
		}
		if (error)
			sysfs_remove_dir(kobj);

	}
	return (error);
}

int
kobject_add(struct kobject *kobj, struct kobject *parent, const char *fmt, ...)
{
	va_list args;
	int error;

	va_start(args, fmt);
	error = kobject_set_name_vargs(kobj, fmt, args);
	va_end(args);
	if (error)
		return (error);

	return kobject_add_complete(kobj, parent);
}

void
linux_kobject_release(struct kref *kref)
{
	struct kobject *kobj;
	char *name;

	kobj = container_of(kref, struct kobject, kref);
	sysfs_remove_dir(kobj);
	name = kobj->name;
	if (kobj->ktype && kobj->ktype->release)
		kobj->ktype->release(kobj);
	kfree(name);
}

int
kobject_init_and_add(struct kobject *kobj, const struct kobj_type *ktype,
    struct kobject *parent, const char *fmt, ...)
{
	va_list args;
	int error;

	kobject_init(kobj, ktype);
	kobj->ktype = ktype;
	kobj->parent = parent;
	kobj->name = NULL;

	va_start(args, fmt);
	error = kobject_set_name_vargs(kobj, fmt, args);
	va_end(args);
	if (error)
		return (error);
	return kobject_add_complete(kobj, parent);
}

static struct rwlock linux_vma_lock;
static TAILQ_HEAD(, vm_area_struct) linux_vma_head =
    TAILQ_HEAD_INITIALIZER(linux_vma_head);

#define	LINUX_IOCTL_MIN_PTR 0x10000UL
#define	LINUX_IOCTL_MAX_PTR (LINUX_IOCTL_MIN_PTR + IOCPARM_MAX)

static inline int
linux_remap_address(void **uaddr, size_t len)
{
	uintptr_t uaddr_val = (uintptr_t)(*uaddr);

	if (unlikely(uaddr_val >= LINUX_IOCTL_MIN_PTR &&
	    uaddr_val < LINUX_IOCTL_MAX_PTR)) {
		struct task_struct *pts = current;
		if (pts == NULL) {
			*uaddr = NULL;
			return (1);
		}

		/* compute data offset */
		uaddr_val -= LINUX_IOCTL_MIN_PTR;

		/* check that length is within bounds */
		if ((len > IOCPARM_MAX) ||
		    (uaddr_val + len) > pts->bsd_ioctl_len) {
			*uaddr = NULL;
			return (1);
		}

		/* re-add kernel buffer address */
		uaddr_val += (uintptr_t)pts->bsd_ioctl_data;

		/* update address location */
		*uaddr = (void *)uaddr_val;
		return (1);
	}
	return (0);
}


int
linux_copyin(const void *uaddr, void *kaddr, size_t len)
{
	if (linux_remap_address(__DECONST(void **, &uaddr), len)) {
		if (uaddr == NULL)
			return (-EFAULT);
		memcpy(kaddr, uaddr, len);
		return (0);
	}
	return (-copyin(uaddr, kaddr, len));
}

int
linux_copyout(const void *kaddr, void *uaddr, size_t len)
{
	if (linux_remap_address(&uaddr, len)) {
		if (uaddr == NULL)
			return (-EFAULT);
		memcpy(uaddr, kaddr, len);
		return (0);
	}
	return (-copyout(kaddr, uaddr, len));
}

size_t
linux_clear_user(void *_uaddr, size_t _len)
{
	uint8_t *uaddr = _uaddr;
	size_t len = _len;

	/* make sure uaddr is aligned before going into the fast loop */
	while (((uintptr_t)uaddr & 7) != 0 && len > 7) {
		if (subyte(uaddr, 0))
			return (_len);
		uaddr++;
		len--;
	}

	/* zero 8 bytes at a time */
	while (len > 7) {
#ifdef __LP64__
		if (suword64(uaddr, 0))
			return (_len);
#else
		if (suword32(uaddr, 0))
			return (_len);
		if (suword32(uaddr + 4, 0))
			return (_len);
#endif
		uaddr += 8;
		len -= 8;
	}

	/* zero fill end, if any */
	while (len > 0) {
		if (subyte(uaddr, 0))
			return (_len);
		uaddr++;
		len--;
	}
	return (0);
}

int
linux_access_ok(const void *uaddr, size_t len)
{
	uintptr_t saddr;
	uintptr_t eaddr;

	/* get start and end address */
	saddr = (uintptr_t)uaddr;
	eaddr = (uintptr_t)uaddr + len;

	/* verify addresses are valid for userspace */
	return ((saddr == eaddr) ||
	    (eaddr > saddr && eaddr <= VM_MAXUSER_ADDRESS));
}

unsigned int
iminor(struct inode *inode)
{
	struct vnode *vnode;

	vnode = (struct vnode *)inode;
	if (vnode == NULL || vnode->v_rdev == NULL)
		return (-1U);

	return (vnode->v_rdev->si_drv0);
}

/*
 * Hash of vmmap addresses.  This is infrequently accessed and does not
 * need to be particularly large.  This is done because we must store the
 * caller's idea of the map size to properly unmap.
 */
struct vmmap {
	LIST_ENTRY(vmmap)	vm_next;
	void 			*vm_addr;
	unsigned long		vm_size;
};

struct vmmaphd {
	struct vmmap *lh_first;
};
#define	VMMAP_HASH_SIZE	64
#define	VMMAP_HASH_MASK	(VMMAP_HASH_SIZE - 1)
#define	VM_HASH(addr)	((uintptr_t)(addr) >> PAGE_SHIFT) & VMMAP_HASH_MASK
static struct vmmaphd vmmaphead[VMMAP_HASH_SIZE];
static struct mtx vmmaplock;

static void
vmmap_add(void *addr, unsigned long size)
{
	struct vmmap *vmmap;

	vmmap = kmalloc(sizeof(*vmmap), GFP_KERNEL);
	mtx_lock(&vmmaplock);
	vmmap->vm_size = size;
	vmmap->vm_addr = addr;
	LIST_INSERT_HEAD(&vmmaphead[VM_HASH(addr)], vmmap, vm_next);
	mtx_unlock(&vmmaplock);
}

static struct vmmap *
vmmap_remove(void *addr)
{
	struct vmmap *vmmap;

	mtx_lock(&vmmaplock);
	LIST_FOREACH(vmmap, &vmmaphead[VM_HASH(addr)], vm_next)
		if (vmmap->vm_addr == addr)
			break;
	if (vmmap)
		LIST_REMOVE(vmmap, vm_next);
	mtx_unlock(&vmmaplock);

	return (vmmap);
}

#if defined(__i386__) || defined(__amd64__) || defined(__powerpc__)
void *
_ioremap_attr(vm_paddr_t phys_addr, unsigned long size, int attr)
{
	void *addr;

	addr = pmap_mapdev_attr(phys_addr, size, attr);
	if (addr == NULL)
		return (NULL);
	vmmap_add(addr, size);

	return (addr);
}
#endif

void
iounmap(void *addr)
{
	struct vmmap *vmmap;

	vmmap = vmmap_remove(addr);
	if (vmmap == NULL)
		return;
#if defined(__i386__) || defined(__amd64__) || defined(__powerpc__)
	pmap_unmapdev((vm_offset_t)addr, vmmap->vm_size);
#endif
	kfree(vmmap);
}


void *
vmap(struct page **pages, unsigned int count, unsigned long flags, int prot)
{
	vm_offset_t off;
	size_t size;

	size = count * PAGE_SIZE;
	off = kva_alloc(size);
	if (off == 0)
		return (NULL);
	vmmap_add((void *)off, size);
	pmap_qenter(off, pages, count);

	return ((void *)off);
}

void
vunmap(void *addr)
{
	struct vmmap *vmmap;

	vmmap = vmmap_remove(addr);
	if (vmmap == NULL)
		return;
	pmap_qremove((vm_offset_t)addr, vmmap->vm_size / PAGE_SIZE);
	kva_free((vm_offset_t)addr, vmmap->vm_size);
	kfree(vmmap);
}

char *
kvasprintf(gfp_t gfp, const char *fmt, va_list ap)
{
	unsigned int len;
	char *p;
	va_list aq;

	va_copy(aq, ap);
	len = vsnprintf(NULL, 0, fmt, aq);
	va_end(aq);

	p = kmalloc(len + 1, gfp);
	if (p != NULL)
		vsnprintf(p, len + 1, fmt, ap);

	return (p);
}

char *
kasprintf(gfp_t gfp, const char *fmt, ...)
{
	va_list ap;
	char *p;

	va_start(ap, fmt);
	p = kvasprintf(gfp, fmt, ap);
	va_end(ap);

	return (p);
}

static void
linux_timer_callback_wrapper(void *context)
{
	struct timer_list *timer;

	linux_set_current(curthread);

	timer = context;
	timer->function(timer->data);
}

void
mod_timer(struct timer_list *timer, int expires)
{

	timer->expires = expires;
	callout_reset(&timer->callout,
	    linux_timer_jiffies_until(expires),
	    &linux_timer_callback_wrapper, timer);
}

void
add_timer(struct timer_list *timer)
{

	callout_reset(&timer->callout,
	    linux_timer_jiffies_until(timer->expires),
	    &linux_timer_callback_wrapper, timer);
}

void
add_timer_on(struct timer_list *timer, int cpu)
{

	callout_reset_on(&timer->callout,
	    linux_timer_jiffies_until(timer->expires),
	    &linux_timer_callback_wrapper, timer, cpu);
}

static void
linux_timer_init(void *arg)
{

	/*
	 * Compute an internal HZ value which can divide 2**32 to
	 * avoid timer rounding problems when the tick value wraps
	 * around 2**32:
	 */
	linux_timer_hz_mask = 1;
	while (linux_timer_hz_mask < (unsigned long)hz)
		linux_timer_hz_mask *= 2;
	linux_timer_hz_mask--;
}
SYSINIT(linux_timer, SI_SUB_DRIVERS, SI_ORDER_FIRST, linux_timer_init, NULL);

void
linux_complete_common(struct completion *c, int all)
{
	int wakeup_swapper;

	sleepq_lock(c);
	if (all) {
		c->done = UINT_MAX;
		wakeup_swapper = sleepq_broadcast(c, SLEEPQ_SLEEP, 0, 0);
	} else {
		if (c->done != UINT_MAX)
			c->done++;
		wakeup_swapper = sleepq_signal(c, SLEEPQ_SLEEP, 0, 0);
	}
	sleepq_release(c);
	if (wakeup_swapper)
		kick_proc0();
}

/*
 * Indefinite wait for done != 0 with or without signals.
 */
int
linux_wait_for_common(struct completion *c, int flags)
{
	struct task_struct *task;
	int error;

	if (SCHEDULER_STOPPED())
		return (0);

	task = current;

	if (flags != 0)
		flags = SLEEPQ_INTERRUPTIBLE | SLEEPQ_SLEEP;
	else
		flags = SLEEPQ_SLEEP;
	error = 0;
	for (;;) {
		sleepq_lock(c);
		if (c->done)
			break;
		sleepq_add(c, NULL, "completion", flags, 0);
		if (flags & SLEEPQ_INTERRUPTIBLE) {
			DROP_GIANT();
			error = -sleepq_wait_sig(c, 0);
			PICKUP_GIANT();
			if (error != 0) {
				linux_schedule_save_interrupt_value(task, error);
				error = -ERESTARTSYS;
				goto intr;
			}
		} else {
			DROP_GIANT();
			sleepq_wait(c, 0);
			PICKUP_GIANT();
		}
	}
	if (c->done != UINT_MAX)
		c->done--;
	sleepq_release(c);

intr:
	return (error);
}

/*
 * Time limited wait for done != 0 with or without signals.
 */
int
linux_wait_for_timeout_common(struct completion *c, int timeout, int flags)
{
	struct task_struct *task;
	int end = jiffies + timeout;
	int error;

	if (SCHEDULER_STOPPED())
		return (0);

	task = current;

	if (flags != 0)
		flags = SLEEPQ_INTERRUPTIBLE | SLEEPQ_SLEEP;
	else
		flags = SLEEPQ_SLEEP;

	for (;;) {
		sleepq_lock(c);
		if (c->done)
			break;
		sleepq_add(c, NULL, "completion", flags, 0);
		sleepq_set_timeout(c, linux_timer_jiffies_until(end));

		DROP_GIANT();
		if (flags & SLEEPQ_INTERRUPTIBLE)
			error = -sleepq_timedwait_sig(c, 0);
		else
			error = -sleepq_timedwait(c, 0);
		PICKUP_GIANT();

		if (error != 0) {
			/* check for timeout */
			if (error == -EWOULDBLOCK) {
				error = 0;	/* timeout */
			} else {
				/* signal happened */
				linux_schedule_save_interrupt_value(task, error);
				error = -ERESTARTSYS;
			}
			goto done;
		}
	}
	if (c->done != UINT_MAX)
		c->done--;
	sleepq_release(c);

	/* return how many jiffies are left */
	error = linux_timer_jiffies_until(end);
done:
	return (error);
}

int
linux_try_wait_for_completion(struct completion *c)
{
	int isdone;

	sleepq_lock(c);
	isdone = (c->done != 0);
	if (c->done != 0 && c->done != UINT_MAX)
		c->done--;
	sleepq_release(c);
	return (isdone);
}

int
linux_completion_done(struct completion *c)
{
	int isdone;

	sleepq_lock(c);
	isdone = (c->done != 0);
	sleepq_release(c);
	return (isdone);
}

struct list_sort_thunk {
	int (*cmp)(void *, struct list_head *, struct list_head *);
	void *priv;
};

static inline int
linux_le_cmp(void *priv, const void *d1, const void *d2)
{
	struct list_head *le1, *le2;
	struct list_sort_thunk *thunk;

	thunk = priv;
	le1 = *(__DECONST(struct list_head **, d1));
	le2 = *(__DECONST(struct list_head **, d2));
	return ((thunk->cmp)(thunk->priv, le1, le2));
}

void
list_sort(void *priv, struct list_head *head, int (*cmp)(void *priv,
    struct list_head *a, struct list_head *b))
{
	struct list_sort_thunk thunk;
	struct list_head **ar, *le;
	size_t count, i;

	count = 0;
	list_for_each(le, head)
		count++;
	ar = malloc(sizeof(struct list_head *) * count, M_KMALLOC, M_WAITOK);
	i = 0;
	list_for_each(le, head)
		ar[i++] = le;
	thunk.cmp = cmp;
	thunk.priv = priv;
	qsort_r(ar, count, sizeof(struct list_head *), &thunk, linux_le_cmp);
	INIT_LIST_HEAD(head);
	for (i = 0; i < count; i++)
		list_add_tail(ar[i], head);
	free(ar, M_KMALLOC);
}


#if defined(__i386__) || defined(__amd64__)
int
linux_wbinvd_on_all_cpus(void)
{

	pmap_invalidate_cache();
	return (0);
}
#endif

int
linux_on_each_cpu(void callback(void *), void *data)
{

	smp_rendezvous(smp_no_rendezvous_barrier, callback,
	    smp_no_rendezvous_barrier, data);
	return (0);
}

int
linux_in_atomic(void)
{

	return ((curthread->td_pflags & TDP_NOFAULTING) != 0);
}


#if defined(__i386__) || defined(__amd64__)
bool linux_cpu_has_clflush;
#endif

static void
linux_compat_init(void *arg)
{
	int i;

#if defined(__i386__) || defined(__amd64__)
	linux_cpu_has_clflush = (cpu_feature & CPUID_CLFSH);
#endif
	rw_init(&linux_vma_lock, "lkpi-vma-lock");

	mtx_init(&vmmaplock, "IO Map lock", NULL, MTX_DEF);
	for (i = 0; i < VMMAP_HASH_SIZE; i++)
		LIST_INIT(&vmmaphead[i]);
}
SYSINIT(linux_compat, SI_SUB_DRIVERS, SI_ORDER_SECOND, linux_compat_init, NULL);

static void
linux_compat_uninit(void *arg)
{

	mtx_destroy(&vmmaplock);
}
SYSUNINIT(linux_compat, SI_SUB_DRIVERS, SI_ORDER_SECOND, linux_compat_uninit, NULL);

/*
 * NOTE: Linux frequently uses "unsigned long" for pointer to integer
 * conversion and vice versa, where in FreeBSD "uintptr_t" would be
 * used. Assert these types have the same size, else some parts of the
 * LinuxKPI may not work like expected:
 */
CTASSERT(sizeof(unsigned long) == sizeof(uintptr_t));
