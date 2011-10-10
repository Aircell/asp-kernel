#!/bin/bash -eu

[ -f ~/.bash-android ] && source ~/.bash-android
[ -f ./bash-android ] && source [ ./bash-android ]	# FIX: remove when done
[ -f $CLOUDSURFER_ROOT/bin/bash-android ] && source $CLOUDSURFER_ROOT/bin/bash-android

source parseargs.sh

export ARCH=arm
export CROSS_COMPILE=arm-eabi-
usage="usage: $0 [-s|-v|-c]"

die() {
  echo $* >&2
  exit 1
}

status() {
  # announce something with a time stamp
  stamp=$(date +%T)
  echo [$stamp] $* >&3
}

start() {
  # announce the start of the build
  kversion=$(fromMk VERSION)
  kpatchlevel=$(fromMk PATCHLEVEL)
  ksublevel=$(fromMk SUBLEVEL)
  kextraversion=$(fromMk EXTRAVERSION)
  
  status starting build, kernel $kversion.$kpatchlevel.$ksublevel${kextraversion:+-$kextraversion}
}

finish() {
  status done
  exit 0
}

fromMk() {
  grep "^$1" Makefile | awk '{print $3}'
}

kconfig() {
  status configuring kernel

  [ -e .config ] ||
      make omap3_aircell_android_defconfig
}

kclean() {
  status cleaning kernel

  make distclean 
}

kbuild() {
  status building kernel

  nparallel=4
  make -j$nparallel uImage
}

build_wifi() {
  status building TI wifi driver

  export KERNEL_DIR=$PWD
  export HOST_PLATFORM=omap3logic
  wifidrv=drivers/wl1271
  export SDIOKOPATH=$wifidrv/external_drivers/omap3logic/Linux/proprietary_sdio
  pushd $wifidrv/platforms/os/linux &> /dev/null
  rm -f tiwlan_drv.ko
  make
}

parseargs $*

start

if [ "$clean" ]; then
 kclean && kconfig
else
  kbuild
  build_wifi
fi

finish



    #if [ $? -eq 0 ] && [ -e tiwlan_drv.ko ]; then
    #    for file in firmware.bin tiwlan_drv.ko ../../../config/tiwlan.ini; do
    #        cp $file $TOPDIR/$ANDROID_DIR/out/target/product/omap3logic/system/etc/wifi
    #    done
