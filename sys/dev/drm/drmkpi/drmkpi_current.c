/*-
 * Copyright (c) 2017 Hans Petter Selasky
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

#include <sys/kernel.h>
#include <sys/eventhandler.h>
#include <sys/malloc.h>

#include <linux/compat.h>
#include <linux/mm.h>

static MALLOC_DEFINE(M_DRMKPI_CURRENT, "drmkpcurrent", "LinuxKPI task structure");

int
drmkpi_alloc_current(struct thread *td, int flags)
{
	struct proc *proc;
	struct thread *td_other;
	struct task_struct *ts;
	struct task_struct *ts_other;
	struct mm_struct *mm;
	struct mm_struct *mm_other;

	MPASS(td->td_lkpi_task == NULL);

	ts = malloc(sizeof(*ts), M_DRMKPI_CURRENT, flags | M_ZERO);
	if (ts == NULL)
		return (ENOMEM);

	mm = malloc(sizeof(*mm), M_DRMKPI_CURRENT, flags | M_ZERO);
	if (mm == NULL) {
		free(ts, M_DRMKPI_CURRENT);
		return (ENOMEM);
	}

	/* setup new task structure */
	atomic_set(&ts->kthread_flags, 0);
	ts->task_thread = td;
	ts->comm = td->td_name;
	ts->pid = td->td_tid;
	atomic_set(&ts->state, TASK_RUNNING);
	init_completion(&ts->parked);
	init_completion(&ts->exited);

	proc = td->td_proc;

	/* check if another thread already has a mm_struct */
	PROC_LOCK(proc);
	FOREACH_THREAD_IN_PROC(proc, td_other) {
		ts_other = td_other->td_lkpi_task;
		if (ts_other == NULL)
			continue;

		mm_other = ts_other->mm;
		if (mm_other == NULL)
			continue;

		/* try to share other mm_struct */
		if (atomic_inc_not_zero(&mm_other->mm_users)) {
			/* set mm_struct pointer */
			ts->mm = mm_other;
			break;
		}
	}

	/* use allocated mm_struct as a fallback */
	if (ts->mm == NULL) {
		/* setup new mm_struct */
		atomic_set(&mm->mm_count, 1);
		atomic_set(&mm->mm_users, 1);
		/* set mm_struct pointer */
		ts->mm = mm;
		/* clear pointer to not free memory */
		mm = NULL;
	}

	/* store pointer to task struct */
	td->td_lkpi_task = ts;
	PROC_UNLOCK(proc);

	/* free mm_struct pointer, if any */
	free(mm, M_DRMKPI_CURRENT);

	return (0);
}
