# Ubuntu system packages

This instruction is based on Ubuntu 22.04 LTS. Newer versions of Ubuntu likely package newer versions of GCC and Clang.

## General

First install the basics:

```shell
sudo apt update
sudo apt upgrade
sudo apt install build-essential git python3-pip
```

## GCC

To build with the GNU C Compiler compiler toolchain, also install:

```shell
sudo apt install g++-13 gcc-13
```

Make symlinks from `g++-13` and `gcc-13` to `g++` and `gcc`, respectively.

## Clang

To build with the LLVM C compiler toolchain, also install:

```shell
sudo apt install clang-17 clang-tidy-17 clang-format-17 libc++-17-dev libc++abi-17-dev libclang-rt-17-dev llvm-17
```

Make symlinks from `clang++-17`, `clang-17` and `clang-tidy-17` to `clang++`, `clang` and `clang-tidy`, respectively.
