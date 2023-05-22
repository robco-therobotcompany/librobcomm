# librobcomm

C++ library for interacting with Robco robots.

## Build and Install

Librobcomm can be built from source, then installed as a Debian package:

```bash
~$ git clone https://github.com/robco-therobotcompany/librobcomm && cd librobcomm
~/librobcomm$ rm -rf ./build
~/librobcomm$ mkdir -p build
~/librobcomm$ cd build
~/librobcomm/build$ cmake ..
~/librobcomm/build$ make
~/librobcomm/build$ cpack
~/librobcomm/build$ sudo apt install ./librobcomm*.deb
```
