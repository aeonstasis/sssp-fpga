/**********
Copyright (c) 2017, Xilinx, Inc.
All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice,
this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
this list of conditions and the following disclaimer in the documentation
and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors
may be used to endorse or promote products derived from this software
without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
**********/

#pragma OPENCL EXTENSION cl_khr_fp64: enable
#pragma OPENCL EXTENSION cl_khr_int64_base_atomics: enable

ulong atom_cmpxchg(volatile __global ulong *p, ulong cmp, ulong val);

double __attribute__((overloadable)) atomic_min(global double *valq, double val) {

    double oldVal;
    double newVal; 

    unsigned long* oldValPtr = (unsigned long*) &oldVal;
    unsigned long* newValPtr = (unsigned long*) &newVal;

    do {
        oldVal = *valq;
        newVal = val < oldVal ? val : oldVal;
    } while (atom_cmpxchg((volatile global unsigned long*) valq, *oldValPtr, *newValPtr) != *oldValPtr);
    return oldVal;
}

// This function represents an OpenCL kernel. The kernel will be call from
// host application using the xcl_run_kernels call. The pointers in kernel
// parameters with the global keyword represents cl_mem objects on the FPGA
// DDR memory.
//
kernel __attribute__((reqd_work_group_size(1, 1, 1)))
void bellman_ford(global double* distsRead,
                global double* distsWrite,
                global const ulong* sources,
                global const ulong* destinations,
                global const double* costs,
                const ulong numVertices,
                const ulong numEdges)
{
    for(ulong iter = 0; iter < numVertices; iter++) {
        __attribute__((opencl_unroll_hint))
        for(int i=0; i<numEdges; i++) {
            ulong source = sources[i];
            ulong dest = destinations[i];
            double cost = costs[i];

            if(distsRead[source] + cost < distsRead[dest]) {
                distsWrite[dest] = distsRead[source] + cost;
            }
            // atomic_min(&distsWrite[dest], distsRead[source] + cost);
        }

        __attribute__((opencl_unroll_hint))
        for(ulong i=0; i<numVertices; i++) {
            distsRead[i] = distsWrite[i];
        }
    }
}
