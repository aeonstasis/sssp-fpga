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
#include "xcl2.hpp"
#include "graph.hpp"
#include <vector>
#include <stdexcept>
#include <string>
#include <limits>

using std::vector;
using std::string;
using graph::Graph;
using graph::Edge;

static constexpr double kInfinity = std::numeric_limits<double>::max();
static const std::string error_message =
    "Error: Result mismatch:\n"
    "i = %d CPU result = %d Device result = %d\n";

// This example illustrates the very simple OpenCL example that performs
// an addition on two vectors
int main(int argc, char **argv) {

    /*
    if(argc != 3) {
        throw std::invalid_argument("Usage: ./bellmanford <input-file> <source-id>");
    }
    string inputFile = argv[1];
    string sourceStr = argv[2];
    int source = std::stoi(sourceStr); 
    */
    string inputFile = "../data/v10-e20.graph";
    int source = 0;


    Graph graph = Graph(inputFile);

    // compute the size of array in bytes
    // size_t size_in_bytes = DATA_SIZE * sizeof(int);

    // Creates a vector of DATA_SIZE elements with an initial value of 10 and 32
    vector<double, aligned_allocator<double>> distsRead(graph.num_vertices, kInfinity);
    vector<double, aligned_allocator<double>> distsWrite(graph.num_vertices, kInfinity);
    distsRead[source] = 0.0;
    distsWrite[source] = 0.0;
    vector<int, aligned_allocator<int>> sources(graph.getNumEdges());
    vector<int, aligned_allocator<int>> destinations(graph.getNumEdges());
    vector<double, aligned_allocator<double>> costs(graph.getNumEdges());

    vector<Edge> allEdges = graph.getAllEdges();
    for(int i=0; i<allEdges.size(); i++) {
        sources[i] = allEdges[i].src;
        destinations[i] = allEdges[i].dest;
        costs[i] = allEdges[i].cost;
    }
    


    // The get_xil_devices will return vector of Xilinx Devices 
    std::vector<cl::Device> devices = xcl::get_xil_devices();
    cl::Device device = devices[0];

    //Creating Context and Command Queue for selected Device 
    cl::Context context(device);
    cl::CommandQueue q(context, device, CL_QUEUE_PROFILING_ENABLE);
    std::string device_name = device.getInfo<CL_DEVICE_NAME>(); 
    std::cout << "Found Device=" << device_name.c_str() << std::endl;

    // import_binary() command will find the OpenCL binary file created using the 
    // xocc compiler load into OpenCL Binary and return as Binaries
    // OpenCL and it can contain many functions which can be executed on the
    // device.
    std::string binaryFile = xcl::find_binary_file(device_name,"bf_kernel");
    cl::Program::Binaries bins = xcl::import_binary_file(binaryFile);
    devices.resize(1);
    cl::Program program(context, devices, bins);

    // These commands will allocate memory on the FPGA. The cl::Buffer objects can
    // be used to reference the memory locations on the device. The cl::Buffer
    // object cannot be referenced directly and must be passed to other OpenCL
    // functions.
    cl::Buffer bufferDistsRead(context, CL_MEM_USE_HOST_PTR | CL_MEM_READ_WRITE,  
            graph.num_vertices * sizeof(double), distsRead.data());
    cl::Buffer bufferDistsWrite(context, CL_MEM_USE_HOST_PTR | CL_MEM_READ_WRITE,  
            graph.num_vertices * sizeof(double), distsWrite.data());
    cl::Buffer bufferSources(context, CL_MEM_USE_HOST_PTR | CL_MEM_READ_ONLY,  
            graph.getNumEdges() * sizeof(int), sources.data());
    cl::Buffer bufferDestinations(context, CL_MEM_USE_HOST_PTR | CL_MEM_READ_ONLY,  
            graph.getNumEdges() * sizeof(int), destinations.data());
    cl::Buffer bufferCosts(context, CL_MEM_USE_HOST_PTR | CL_MEM_READ_ONLY,  
            graph.getNumEdges() * sizeof(double), costs.data());

    //Separate Read/write Buffer vector is needed to migrate data between host/device
    std::vector<cl::Memory> inBufVec, outBufVec;
    inBufVec.push_back(bufferDistsRead);
    inBufVec.push_back(bufferDistsWrite);
    inBufVec.push_back(bufferSources);
    inBufVec.push_back(bufferDestinations);
    inBufVec.push_back(bufferCosts);
    outBufVec.push_back(bufferDistsRead);


    // These commands will load the source_a and source_b vectors from the host
    // application and into the buffer_a and buffer_b cl::Buffer objects. The data
    // will be be transferred from system memory over PCIe to the FPGA on-board
    // DDR memory.
    q.enqueueMigrateMemObjects(inBufVec,0/* TN: 0 means from host*/);

    // This call will extract a kernel out of the program we loaded in the
    // previous line. A kernel is an OpenCL function that is executed on the
    // FPGA. This function is defined in the src/vetor_addition.cl file.
    cl::Kernel krnl_bellman_ford(program,"bellman_ford");

    //set the kernel Arguments
    int narg=0;
    krnl_bellman_ford.setArg(narg++, graph.num_vertices);
    krnl_bellman_ford.setArg(narg++, graph.getNumEdges());
    krnl_bellman_ford.setArg(narg++, bufferDistsRead);
    krnl_bellman_ford.setArg(narg++, bufferDistsWrite);
    krnl_bellman_ford.setArg(narg++, bufferSources);
    krnl_bellman_ford.setArg(narg++, bufferDestinations);
    krnl_bellman_ford.setArg(narg++, bufferCosts);

    std::cout << "Max work group size: " << krnl_bellman_ford.getWorkGroupInfo<CL_KERNEL_WORK_GROUP_SIZE>(device) << std::endl;

    //Launch the Kernel
    q.enqueueTask(krnl_bellman_ford);

    // The result of the previous kernel execution will need to be retrieved in
    // order to view the results. This call will write the data from the
    // buffer_result cl_mem object to the source_results vector
    q.enqueueMigrateMemObjects(outBufVec,CL_MIGRATE_MEM_OBJECT_HOST);
    q.finish();


    for (int i = 0; i < graph.num_vertices; i++) {
        printf("%d: %f\n", i, distsRead[i]);
    }

    return 0;
}
