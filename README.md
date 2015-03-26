Snowcap version of OpenOCD
==========================

* Fix support for ChibiOS 3.0
* Add support for Snowcap boards.

Install
-------

```
./configure --prefix=/usr/local/openocd/ --disable-werror --enable-usbprog --enable-vsllink --enable-buspirate --enable-ftdi --enable-stlink --enable-arm-jtag-ew --enable-remote-bitbang --enable-dummy
make -j4
make install
```

