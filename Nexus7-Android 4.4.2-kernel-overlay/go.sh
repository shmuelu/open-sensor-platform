#!/bin/sh

make ARCH=arm osp_sensor_hub_tegra3_android_defconfig 2>&1 | tee logs/build.log
make -j8 VERBOSE=1 CROSS_COMPILE=/android/Nexus7/kitkat/android/prebuilts/gcc/linux-x86/arm/arm-linux-androideabi-4.6/bin/arm-linux-androideabi-  ARCH=arm  2>&1 | tee -a logs/build.log


