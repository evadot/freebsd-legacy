#!/bin/sh

#-
# SPDX-License-Identifier: BSD-2-Clause-FreeBSD
#
# Copyright 2019 Emmanuel Vadot <manu@FreeBSD.Org>
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted providing that the following conditions 
# are met:
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
# IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
# OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
# HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
# STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
# IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# $FreeBSD$

usage() {
	cat <<EOF
usage: `basename $0` command [arg]

Options:
  -f            -- Force the command to be executed
  -k KERNDIR    -- Use KERNDIR instead of /boot/
  -p            -- Use the package name instead of the KERNCONF one
  -v            -- Verbose output

Commands:
  backup        -- Backup the kernel as kernel.old if it is the current one
                   (or always backup if -f is used)
  status        -- Show which is the current kernel
  switch        -- Switch the kernel to use base on the KERNCONF name
  list          -- List the available(s) kernel(s)
EOF
	exit 1
}

USE_PKG_NAME=
VERBOSE=
FORCE=
KERNDIR=/boot
QUIET=

parse_cmd() {
	# TODO Convert to getopt
	while [ $# -gt 0 ]; do
		case "$1" in
			# Options
			-p)
				USE_PKG_NAME=yes
				;;
			-v)
				VERBOSE=yes
				;;
			-f)
				FORCE=yes
				;;
			-k)
				shift
				KERNDIR=$1
				;;
			-q)
				QUIET=yes
				;;

			# Commands
			status | list | cleanup)
				if [ ! -z "${COMMAND}" ]; then
					usage
				fi
				COMMAND="$1"
				;;
			switch | backup)
				if [ ! -z "${COMMAND}" ]; then
					usage
				fi
				COMMAND="$1"
				shift
				if [ $# -eq 0 ]; then
					usage
				fi
				CMDARG="$1"
				;;
			*)
				usage
				;;
		esac
		shift
	done

	if [ -z "${COMMAND}" ]; then
		usage
	fi

	if [ ! -d "${KERNDIR}" ]; then
		verbose_print "${KERNDIR} is not a directory" ""
		exit 1
	fi
}

verbose_print() {
	if [ "${QUIET}" == "yes" ]; then
		return
	fi
	if [ "${VERBOSE}" == "yes" ]; then
		if [ -z "${2}" ]; then
			echo $1
		else
			echo $2
		fi
	else
		if [ ! -z "$1" ]; then
			echo $1
		fi
	fi
}

verbose_error() {
	verbose_print "$1" "$2"
	exit 1
}

cmd_backup() {
	curkernel=$(realpath ${KERNDIR}/kernel)
	curkernconf=$(realpath ${KERNDIR}/kernel/ | awk -F. '{print $2}')
	curpkgkernel=$(pkg which -q ${curkernel}/kernel)

	if [ "${USE_PKG_NAME}" == "yes" ]; then
		backkernel=$(pkg query '%Fp' ${CMDARG} | head -n1 | sed 's/\/boot\/kernel\.\(.*\)\/.*/\/boot\/kernel\.\1/')
	else
		backkernel=$(realpath ${KERNDIR}/kernel.${CMDARG})
	fi

	if [ "${backkernel}" != "${curkernel}" ] && [ "${FORCE}" != "yes" ]; then
		verbose_print "${CMDARG} is not the current kernel, aborting" ""
		exit 0
	fi

	rm -rf ${KERNDIR}/kernel.old || verbose_error "Cannot remove ${KERNDIR}/kernel.old directory" ""
	cp -R "${backkernel}" "${KERNDIR}/kernel.old"
}

cmd_cleanup() {
	for kerndir in ${KERNDIR}/kernel.*; do
		if [ "${kerndir}" == "${KERNDIR}/kernel.old" ]; then
			continue
		fi
		if [ ! -f "${kerndir}/kernel" ]; then
			verbose_print "Removing ${kerndir}" ""
			rm -rf ${kerndir}
		fi
	done
}

cmd_status() {
	kernel=$(realpath ${KERNDIR}/kernel/kernel 2>/dev/null)
	if [ -z "${kernel}" ]; then
		verbose_error "No current kernel"
		exit 1
	fi
	kernconf=$(realpath ${KERNDIR}/kernel/ | awk -F. '{print $2}')
	pkgkernel=$(pkg which -q ${kernel})
	if [ "${USE_PKG_NAME}" = "yes" ]; then
		kernconf=${pkgkernel}
	fi
	verbose_print "${kernconf}" "Current kernel was installed from ${pkgkernel} with kernel config ${kernconf}"
}

cmd_list() {
	for kerndir in $(ls -d ${KERNDIR}/kernel.*); do
		pkg=$(pkg which -q ${kerndir}/kernel 2>/dev/null)
		if [ ! -z "${pkg}" ]; then
			kernconf=$(echo ${kerndir} | awk -F. '{print $2}')
			if [ "${USE_PKG_NAME}" == "yes" ]; then
				verbose_print "${pkg}" "${kernconf} from ${pkg}"
			else
				verbose_print "${kernconf}" "${kernconf} from ${pkg}"
			fi
		fi
	done
}

cmd_switch() {
	if [ "${USE_PKG_NAME}" == "yes" ]; then
		kernconf=$(pkg query '%Fp' ${CMDARG} | grep -E 'kernel$' | sed 's/\/boot\/kernel\.\(.*\)\/.*/\1/')
		if [ -z "${kernconf}" ]; then
			# This happens with -debug packages in post-install
			echo "${CMDARG} doesn't seems to be a kernel package"
			exit 1
		fi
		CMDARG=${kernconf}
	fi
	if [ ! -d ${KERNDIR}/kernel.$CMDARG ]; then
		echo "${KERNDIR}/kernel.${CMDARG} doesn't exits"
		exit 1
	fi
	curkernel=$(realpath ${KERNDIR}/kernel/kernel | awk -F. '{print $2}' >/dev/null 2>&1)
	if [ "${curkernel}" == "${CMDARG}" ]; then
		verbose_print "" "${CMDARG} is already the current kernel"
		exit 0
	fi
	rm ${KERNDIR}/kernel >/dev/null 2>&1 || rmdir ${KERNDIR}/kernel >/dev/null 2>&1
	verbose_print "" "Switching to kernel $CMDARG"
	(cd ${KERNDIR} && ln -s kernel.${CMDARG} kernel && cd -)
	verbose_print "" "Running kldxref"
	/usr/sbin/kldxref ${KERNDIR}/kernel/
}

parse_cmd $@
cmd_${COMMAND}
