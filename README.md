# sssp-fpga
> Bellman-Ford implementations for shared memory systems accelerated using std::thread and FPGAs via AWS EC2 F1 

[![Build Status](https://travis-ci.org/aaron-zou/sssp-fpga.svg?branch=master)](https://travis-ci.org/aaron-zou/sssp-fpga)

This repository contains multiple implementations of the Bellman-Ford algorithm for solving single-source shortest paths (SSSP). In addition to CPU-only sequential and multithreaded implementations, we also accelerate via FPGA an SDAccel Bellman-Ford implementation written in OpenCL for deployment on AWS F1 FPGA instances.

## Installation

Note: this repository is only tested on Ubuntu 16.04 and Ubuntu 17.10. While CMake introduces portability for the CPU-only implementations, the FPGA requires Linux (see https://github.com/aws/aws-fpga). 

```sh
mkdir build && cd build
cmake ..
make
```

### CUDA

### SDAccel
export XILINX_SDX

### Testing

Run all tests using:
```sh
make test
```

## Authors
- [Aaron Zou](https://github.com/aaron-zou/)
- [Shawn Wu](https://github.com/chudooder)

## License

[![License](http://img.shields.io/:license-mit-blue.svg?style=flat-square)](http://badges.mit-license.org) This project is licensed under the MIT License.

