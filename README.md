# osr_usb_fx2

Introduction
------------
A educational experiment in writing a Linux USB Driver for the
OSR USB FX2 device.

LED bar and 7-segment LED are read and written to via sysfs attributes.

Toggle Switch state is read via a sysfs attribute as well.  An interrupt
handler monitors changes in the Toggle Switch.

Toggle Switch debounce is handled by scheduling a delayed work on a the
shared work queue.


Building
--------
This code has been built on Ubuntu 14.04 with kernel 3.16.0-x.  I got
build errors on newer 4.x kernels, and I'll fix that when I get around
to it.


Credit
------
This is an attempt to teach myself how to write a Linux USB driver.
I learned a lot from reading these two articles and shamelessly copied
some code:

http://www.linuxjournal.com/article/7353

http://matthias.vallentin.net/blog/2007/04/writing-a-linux-kernel-driver-for-an-unknown-usb-device/


