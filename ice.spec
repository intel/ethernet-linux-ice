Name: ice
Summary: Intel(R) Ethernet Connection E800 Series Linux Driver
Version: 2.7.6
Release: 1
Source: %{name}-%{version}.tar.gz
Vendor: Intel Corporation
License: GPLv2 and Redistributable, no modification permitted
ExclusiveOS: linux
Group: System Environment/Kernel
Provides: %{name}
URL: http://support.intel.com
BuildRoot: %{_tmppath}/%{name}-%{version}-root
%global debug_package %{nil}
# candidate locations of the system files to update at install time
# (pci.ids, pcitable); the first one found on the target system is used
%define _pciids   /usr/share/pci.ids        /usr/share/hwdata/pci.ids
%define _pcitable /usr/share/kudzu/pcitable /usr/share/hwdata/pcitable
# data needed by the scriptlets; must not be %doc (rpm --excludedocs)
%global ice_datadir %{_datadir}/%{name}
%global ice_file_list %{ice_datadir}/%{name}-%{version}-%{release}.list
Requires: kernel, findutils, gawk, bash

%global __strip /bin/true

%if 0%{?BUILD_KERNEL:1}
%define kernel_ver %{BUILD_KERNEL}
%define check_aux_args_kernel -b %{BUILD_KERNEL} 
%else
%define kernel_ver %(uname -r)
%endif

%if 0%{?KSRC:1}
%define check_aux_args_ksrc -k %{KSRC}
%endif

%define check_aux_args %check_aux_args_kernel %check_aux_args_ksrc

# kernels to refresh: the one the module was built for and the running one
%define ice_kernels $( (echo %{kernel_ver}; uname -r) | sort -u)

# the driver may be needed at boot, so a failed initramfs update is an error
%define ice_update_initramfs \
if command -v dracut >/dev/null 2>&1; then \
	for k in %{ice_kernels}; do \
		[ -d /lib/modules/$k ] || continue \
		echo "Updating initramfs for $k with dracut..." \
		if dracut --force --kver $k; then \
			echo "Successfully updated initramfs." \
		else \
			echo "Failed to update initramfs for $k." >&2 \
			echo "You must update your initramfs image for changes to take place." >&2 \
			exit -1 \
		fi \
	done \
elif command -v mkinitrd >/dev/null 2>&1; then \
	echo "Updating initrd with mkinitrd..." \
	if mkinitrd; then \
		echo "Successfully updated initrd." \
	else \
		echo "Failed to update initrd." >&2 \
		echo "You must update your initrd image for changes to take place." >&2 \
		exit -1 \
	fi \
else \
	echo "Unable to determine utility to update initrd image." >&2 \
	echo "You must update your initrd manually for changes to take place." >&2 \
	exit -1 \
fi \
%{nil}

%define ice_depmod \
for k in %{ice_kernels}; do \
	[ -d /lib/modules/$k ] || continue \
	echo $k | grep -q BOOT || /sbin/depmod -a $k > /dev/null 2>&1 || true \
done \
%{nil}

%define need_aux_rpm %( [ -L /lib/modules/%kernel_ver/source ] && (rpm -q --whatprovides /lib/modules/%kernel_ver/source/include/linux/auxiliary_bus.h > /dev/null 2>&1 && echo 0 || echo 2) || (rpm -q --whatprovides /lib/modules/%kernel_ver/build/include/linux/auxiliary_bus.h > /dev/null 2>&1 && echo 0 || echo 2) )

%if (%need_aux_rpm == 2)
Requires: intel_auxiliary
%endif

# Check for existence of variable kernel_module_package_buildreqs ...
%if 0%{?!kernel_module_package_buildreqs:1}
# ... and provide a suitable definition if it is not defined
%define kernel_module_package_buildreqs kernel-devel
%endif

%define kernel_module_package_buildreqs_fixed %(/bin/bash -fc 'if [[ %{kernel_ver} == *uek* ]]; 
	then echo %kernel_module_package_buildreqs | sed 's/kernel-devel/kernel-uek-devel/g' ; else echo %kernel_module_package_buildreqs ; fi')

BuildRequires: %kernel_module_package_buildreqs_fixed

%description
This package contains the Intel(R) Ethernet Connection E800 Series Linux Driver.

%prep
%setup

%build
make -C src clean
make -C src

%install
make -C src INSTALL_MOD_PATH=%{buildroot} MANDIR=%{_mandir} modules_install_no_aux mandocs_install
# Remove modules files that we do not want to include
if [ -d %{buildroot}/lib/modules ]; then
	find %{buildroot}/lib/modules/ -name 'modules.*' -exec rm -f {} \;
fi

cd %{buildroot}
find . -name "ice.ko" -printf "/%p\n" \
	>%{_builddir}/%{name}-%{version}/file.list

find . -path '*/ddp/*' -printf "/%p\n" \
	>>%{_builddir}/%{name}-%{version}/file.list
find . -name "ice-vfio-pci.ko" -printf "/%p\n" \
	>>%{_builddir}/%{name}-%{version}/file.list
%if (%need_aux_rpm == 2)
make -C %{_builddir}/%{name}-%{version}/src INSTALL_MOD_PATH=%{buildroot} auxiliary_install

find . -path "*extern-symvers/intel_auxiliary.symvers" -printf "/%p\n" \
	>%{_builddir}/%{name}-%{version}/aux.list
find . -name "auxiliary_bus.h" -printf "/%p\n" \
	>>%{_builddir}/%{name}-%{version}/aux.list
find . -name "auxiliary_compat.h" -printf "/%p\n" \
	>>%{_builddir}/%{name}-%{version}/aux.list
find . -name "kcompat_generated_defs.h" -printf "/%p\n" \
	>>%{_builddir}/%{name}-%{version}/aux.list
find . -name "intel_auxiliary.ko" -printf "/%p\n" \
	>>%{_builddir}/%{name}-%{version}/aux.list
%else
find . -name "intel_auxiliary.ko" -type f -delete
%endif

export _ksrc=%{_usrsrc}/kernels/%{kernel_ver}
cd %{buildroot}
# Sign the modules(s)
%if %{?_with_modsign:1}%{!?_with_modsign:0}
%define __strip /bin/true
%{!?privkey: %define privkey %{_sysconfdir}/pki/SECURE-BOOT-KEY.priv}
%{!?pubkey: %define pubkey %{_sysconfdir}/pki/SECURE-BOOT-KEY.der}
%{!?_signfile: %define _signfile ${_ksrc}/scripts/sign-file}
systemctl --no-ask-password restart kerberos || echo kerberos not restarted
for module in `find . -type f -name \*.ko`; do
	strip --strip-debug ${module}
	KSRC=${_ksrc} %{_signfile} sha512 %{privkey} %{pubkey} ${module}
done
%endif

# data used by the scriptlets
install -D -m 644 %{_builddir}/%{name}-%{version}/file.list %{buildroot}%{ice_file_list}
install -D -m 644 %{_builddir}/%{name}-%{version}/pci.updates %{buildroot}%{ice_datadir}/pci.updates



%clean
rm -rf %{buildroot}

%files -f file.list

%defattr(-,root,root)
%{_mandir}/man7/ice.7.gz
%dir %{ice_datadir}
%{ice_file_list}
%{ice_datadir}/pci.updates
%doc COPYING
%doc README

%post
# find the system files on the system being installed to
pciids=
for f in %{_pciids}; do
	if [ -e "$f" ]; then pciids="$f"; break; fi
done
pcitable=/dev/null
for f in %{_pcitable}; do
	if [ -e "$f" ]; then pcitable="$f"; break; fi
done

# only touch pci.ids if there is something to merge and something to merge into
if [ -s "$pciids" ] && [ -r %{ice_datadir}/pci.updates ]; then

	rm -rf /usr/local/share/%{name}
	mkdir -p /usr/local/share/%{name}
	cp --parents "$pciids" /usr/local/share/%{name}/
	echo "original pci.ids saved in /usr/local/share/%{name}";
	ids_new=$(mktemp "$pciids.XXXXXX") || ids_new=
	table_new=/dev/null
	if [ "$pcitable" != "/dev/null" ]; then
		cp --parents "$pcitable" /usr/local/share/%{name}/
		echo "original pcitable saved in /usr/local/share/%{name}";
		table_new=$(mktemp "$pcitable.XXXXXX") || table_new=
	fi

	if [ -z "$ids_new" ] || [ -z "$table_new" ]; then
		echo "WARNING: cannot create a temporary file, pci.ids was not updated" >&2
	else

		#Yes, this really needs bash
		bash -s "$pciids" \
			"$pcitable" \
			%{ice_datadir}/pci.updates \
			"$ids_new" \
			"$table_new" \
			%{name} \
		<<"END"
#! /bin/bash
# Copyright (C) 2017 - 2023 Intel Corporation
# For licensing information, see the file 'LICENSE' in the root folder
# $1 = system pci.ids file to update
# $2 = system pcitable file to update
# $3 = file with new entries in pci.ids file format
# $4 = pci.ids output file
# $5 = pcitable output file
# $6 = driver name for use in pcitable file

exec 3<$1
exec 4<$2
exec 5<$3
exec 6>$4
exec 7>$5
driver=$6
IFS=

# pattern matching strings
ID="[[:xdigit:]][[:xdigit:]][[:xdigit:]][[:xdigit:]]"
VEN="${ID}*"
DEV="	${ID}*"
SUB="		${ID}*"
TABLE_DEV="0x${ID}	0x${ID}	\"*"
TABLE_SUB="0x${ID}	0x${ID}	0x${ID}	0x${ID}	\"*"

line=
table_line=
ids_in=
table_in=
vendor=
device=
ids_device=
table_device=
subven=
ids_subven=
table_subven=
subdev=
ids_subdev=
table_subdev=
ven_str=
dev_str=
sub_str=

# force a sub-shell to fork with a new stdin
# this is needed if the shell is reading these instructions from stdin
while true
do
	# get the first line of each data file to jump start things
	exec 0<&3
	read -r ids_in
	if [ "$2" != "/dev/null" ];then
	exec 0<&4
	read -r table_in
	fi

	# outer loop reads lines from the updates file
	exec 0<&5
	while read -r line || [ -n "$line" ]
	do
		# vendor entry
		if [[ $line == $VEN ]]
		then
			vendor=0x${line:0:4}
			ven_str=${line#${line:0:6}}
			# add entry to pci.ids
			exec 0<&3
			exec 1>&6
			while [[ $ids_in != $VEN ||
				 0x${ids_in:0:4} < $vendor ]]
			do
				echo "$ids_in"
				read -r ids_in || [ -n "$ids_in" ] || break
			done
			echo "$line"
			if [[ 0x${ids_in:0:4} == $vendor ]]
			then
				read -r ids_in
			fi

		# device entry
		elif [[ $line == $DEV ]]
		then
			device=`echo ${line:1:4} | tr "[:upper:]" "[:lower:]"`
			table_device=0x${line:1:4}
			dev_str=${line#${line:0:7}}
			ids_device=`echo ${ids_in:1:4} | tr "[:upper:]" "[:lower:]"`
			table_line="$vendor	$table_device	\"$driver\"	\"$ven_str|$dev_str\""
			# add entry to pci.ids
			exec 0<&3
			exec 1>&6
			while [[ $ids_in != $DEV ||
				 $ids_device < $device ]]
			do
				if [[ $ids_in == $VEN ]]
				then
					break
				fi
				if [[ $ids_device != ${ids_in:1:4} ]]
				then
					echo "${ids_in:0:1}$ids_device${ids_in#${ids_in:0:5}}"
				else
					echo "$ids_in"
				fi
				read -r ids_in || [ -n "$ids_in" ] || break
				ids_device=`echo ${ids_in:1:4} | tr "[:upper:]" "[:lower:]"`
			done
			if [[ $device != ${line:1:4} ]]
			then
				echo "${line:0:1}$device${line#${line:0:5}}"
			else
				echo "$line"
			fi
			if [[ $ids_device == $device ]]
			then
				read -r ids_in
			fi
			# add entry to pcitable
			if [ "$2" != "/dev/null" ];then
			exec 0<&4
			exec 1>&7
			while [[ $table_in != $TABLE_DEV ||
				 ${table_in:0:6} < $vendor ||
				 ( ${table_in:0:6} == $vendor &&
				   ${table_in:7:6} < $table_device ) ]]
			do
				echo "$table_in"
				read -r table_in || [ -n "$table_in" ] || break
			done
			echo "$table_line"
			if [[ ${table_in:0:6} == $vendor &&
			      ${table_in:7:6} == $table_device ]]
			then
				read -r table_in
			fi
			fi
		# subsystem entry
		elif [[ $line == $SUB ]]
		then
			subven=`echo ${line:2:4} | tr "[:upper:]" "[:lower:]"`
			subdev=`echo ${line:7:4} | tr "[:upper:]" "[:lower:]"`
			table_subven=0x${line:2:4}
			table_subdev=0x${line:7:4}
			sub_str=${line#${line:0:13}}
			ids_subven=`echo ${ids_in:2:4} | tr "[:upper:]" "[:lower:]"`
			ids_subdev=`echo ${ids_in:7:4} | tr "[:upper:]" "[:lower:]"`
			table_line="$vendor	$table_device	$table_subven	$table_subdev	\"$driver\"	\"$ven_str|$sub_str\""
			# add entry to pci.ids
			exec 0<&3
			exec 1>&6
			while [[ $ids_in != $SUB ||
				 $ids_subven < $subven ||
				 ( $ids_subven == $subven && 
				   $ids_subdev < $subdev ) ]]
			do
				if [[ $ids_in == $VEN ||
				      $ids_in == $DEV ]]
				then
					break
				fi
				if [[ ! (${ids_in:2:4} == "1014" &&
					 ${ids_in:7:4} == "052C") ]]
				then
					if [[ $ids_subven != ${ids_in:2:4} || $ids_subdev != ${ids_in:7:4} ]]
					then
						echo "${ids_in:0:2}$ids_subven $ids_subdev${ids_in#${ids_in:0:11}}"
					else
						echo "$ids_in"
					fi
				fi
				read -r ids_in || [ -n "$ids_in" ] || break
				ids_subven=`echo ${ids_in:2:4} | tr "[:upper:]" "[:lower:]"`
				ids_subdev=`echo ${ids_in:7:4} | tr "[:upper:]" "[:lower:]"`
			done
			if [[ $subven != ${line:2:4} || $subdev != ${line:7:4} ]]
			then
				echo "${line:0:2}$subven $subdev${line#${line:0:11}}"
			else
				echo "$line"
			fi
			if [[ $ids_subven == $subven  &&
			      $ids_subdev == $subdev ]]
			then
				read -r ids_in
			fi
			# add entry to pcitable
			if [ "$2" != "/dev/null" ];then
			exec 0<&4
			exec 1>&7
			while [[ $table_in != $TABLE_SUB ||
				 ${table_in:14:6} < $table_subven ||
				 ( ${table_in:14:6} == $table_subven &&
				   ${table_in:21:6} < $table_subdev ) ]]
			do
				if [[ $table_in == $TABLE_DEV ]]
				then
					break
				fi
				if [[ ! (${table_in:14:6} == "0x1014" &&
					 ${table_in:21:6} == "0x052C") ]]
				then
					echo "$table_in"
				fi
				read -r table_in || [ -n "$table_in" ] || break
			done
			echo "$table_line"
			if [[ ${table_in:14:6} == $table_subven &&
			      ${table_in:21:6} == $table_subdev ]]
			then
				read -r table_in
			fi
			fi
		fi

		exec 0<&5
	done

	# print the remainder of the original files
	exec 0<&3
	exec 1>&6
	echo "$ids_in"
	while read -r ids_in || [ -n "$ids_in" ]
	do
		echo "$ids_in"
	done

	if [ "$2" != "/dev/null" ];then
	exec 0>&4
	exec 1>&7
	echo "$table_in"
	while read -r table_in || [ -n "$table_in" ]
	do
		echo "$table_in"
	done
	fi

	break
done <&5

exec 3<&-
exec 4<&-
exec 5<&-
exec 6>&-
exec 7>&-

END

		# never replace the system files with an empty or missing result
		if [ $? -eq 0 ] && [ -s "$ids_new" ]; then
			chmod --reference="$pciids" "$ids_new"
			mv -f "$ids_new" "$pciids"
			if [ "$pcitable" != "/dev/null" ] && [ -s "$table_new" ]; then
				chmod --reference="$pcitable" "$table_new"
				mv -f "$table_new" "$pcitable"
			fi
		else
			echo "WARNING: pci.ids was not updated" >&2
		fi

	fi

	# remove whatever is left of the temporary files
	if [ -n "$ids_new" ]; then
		rm -f "$ids_new"
	fi
	if [ -n "$table_new" ] && [ "$table_new" != "/dev/null" ]; then
		rm -f "$table_new"
	fi

else
	echo "pci.ids or pci.updates is missing or empty, pci.ids not updated"
fi

%{ice_depmod}

%posttrans
# Runs after the scriptlets of the package being replaced on upgrade, so the
# weak-modules links and the initramfs are refreshed once, at the very end.
if [ -x "/usr/sbin/weak-modules" ]; then
    grep '\.ko$' %{ice_file_list} | xargs realpath | /usr/sbin/weak-modules --no-initramfs --add-modules
fi

%{ice_update_initramfs}

%preun
# save tmp list of installed kernel modules for weak-modules
grep '\.ko$' %{ice_file_list} | xargs realpath > /var/run/rpm-%{name}-modules.list

# $1 == 0: package is being erased, not upgraded
if [ $1 -eq 0 ]; then
	rm -rf /usr/local/share/%{name}
fi

%postun
%{ice_depmod}

if [ -x "/usr/sbin/weak-modules" ] && [ -f /var/run/rpm-%{name}-modules.list ]; then
    /usr/sbin/weak-modules --no-initramfs --remove-modules < /var/run/rpm-%{name}-modules.list
fi

# On upgrade the new package's %%posttrans updates the initramfs, unless the
# modules of this package are gone (the new package is for another kernel).
gone=0
if [ -f /var/run/rpm-%{name}-modules.list ]; then
	for m in $(cat /var/run/rpm-%{name}-modules.list); do
		[ -e "$m" ] || gone=1
	done
fi
rm -f /var/run/rpm-%{name}-modules.list

if [ $1 -eq 0 ] || [ $gone -eq 1 ]; then
%{ice_update_initramfs}
fi

%if (%need_aux_rpm == 2)
%package -n intel_auxiliary
Summary: Auxiliary bus driver (backport)
Version: 1.0.2

%description -n intel_auxiliary
The Auxiliary bus driver (intel_auxiliary.ko), backported from upstream, for use by kernels that don't have auxiliary bus.

%files -n intel_auxiliary -f aux.list
%doc aux.list
%endif

