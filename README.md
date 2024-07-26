# librobcomm

C++ library for interacting with RobCo robots.

## Build and Install

To build the library from source:

```bash
~$ git clone https://github.com/robco-therobotcompany/librobcomm && cd librobcomm
~/librobcomm$ rm -rf ./build
~/librobcomm$ mkdir -p build
~/librobcomm$ cd build
~/librobcomm/build$ cmake ..
~/librobcomm/build$ make
```

To create and install a debian package:

```bash
~/librobcomm/build$ cpack
~/librobcomm/build$ sudo apt install ./librobcomm*.deb
```

To also build examples:

```bash
~/librobcomm$ cd build
~/librobcomm/build$ cmake -DBUILD_EXAMPLES=1 ..
~/librobcomm/build$ make
```

**WARNING**: Some examples will move the robot. Ensure that the parameters are correct for the current situation, and
always have your hand on the emergency stop button!

