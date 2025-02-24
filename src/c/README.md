# Library Serial Port POSIX (libserialposix) C

## Building

Clone the repository (if not already done):

```bash
$ git clone https://github.com/FDanielPacheco/libserialposix.git

$ cd libserialposix
```

If it's only necessary to install the library jump to section install, otherwise to build the library:

```bash
$ make build/c
```

In case it's necessary to remove the build artifacts and build again:

```bash
$ make clean

$ make build/c
```
---

## Installing

After cloning the repository, inside the LibSerialPOSIX/, the command will install the library on the system, to be able to include the library in any project:

```bash
$ make install/c
```
