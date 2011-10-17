#!/bin/bash -eu

[ -f ~/.bash-android ] && source ~/.bash-android
[ -f $CLOUDSURFER_ROOT/bin/bash-android ] && source $CLOUDSURFER_ROOT/bin/bash-android

source ./parseargs.sh

export ARCH=arm
export CROSS_COMPILE=arm-eabi-
usage="usage: $0 [-s|-v|-c]"
wifidrv=drivers/wl1271

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
  ( cd $wifidrv; rm -rf $(< cleanfiles) )
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
  export SDIOKOPATH=$wifidrv/external_drivers/omap3logic/Linux/proprietary_sdio
  pushd $wifidrv/platforms/os/linux &> /dev/null
  rm -f tiwlan_drv.ko
  make
  # this next should, I think, be handled in packaging
  [ -e tiwlan_drv.ko ] || die "no ti_wlan_drv.ko"
  for file in firmware.bin tiwlan_drv.ko ../../../config/tiwlan.ini; do
    cp $file $ANDROID_BUILD_DIR/out/target/product/omap3logic/system/etc/wifi
  done
}

parseargs $*

start

if [ "$clean" ]; then
  kclean
else
#  kconfig
#  kbuild
  build_wifi
fi

finish



    #if [ $? -eq 0 ] && [ -e tiwlan_drv.ko ]; then
    #    for file in firmware.bin tiwlan_drv.ko ../../../config/tiwlan.ini; do
    #        cp $file $TOPDIR/$ANDROID_DIR/out/target/product/omap3logic/system/etc/wifi
    #    done
