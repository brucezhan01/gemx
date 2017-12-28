/**********
 * Copyright (c) 2017, Xilinx, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software
 * without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * **********/
/**
 *  @brief Main executable for SDX flow
 *
 *  $DateTime: 2017/11/07 13:14:14 $
 */

// Fast Csim compile
//   make host

// Fast run on board
//  ( gdb --args ./gemx.exe k app.bin app_out.bin )
 
// Fast CPU emu
// ( setenv XCL_EMULATION_MODE true ; ./out_host/gemx_host.exe out_cpu_emu/gemx.xclbin out_host/app.bin out_cpu_emu/app_out.bin )

 
#include <stdio.h>
//#include <stdlib.h>
#include <string>
#include <fstream>
#include <iostream>
#include <vector>
#include <iomanip>

#include <CL/opencl.h>
#include "gemx_kernel.h"
#if TEST_SDX
  #include "gemx_fpga.h"
#endif

std::ifstream::pos_type getFileSize(std::string p_FileName)
{
  std::ifstream in(p_FileName.c_str(), std::ifstream::ate | std::ifstream::binary);
  return in.tellg(); 
}

std::vector<DdrType>
loadBinFile(std::string p_BinFileName)
{
  std::vector<DdrType> l_memVec;
  // Bin file existence
  std::ifstream l_if(p_BinFileName.c_str(), std::ios::binary);
  if (l_if.is_open()) {
    // Bin file size
    size_t l_binFileSize = getFileSize(p_BinFileName);
    std::cout << "INFO: loading " + p_BinFileName + " of size " << l_binFileSize << "\n";
    assert(l_binFileSize > 0);
    size_t l_binFileSizeInDdrWords = l_binFileSize / sizeof(DdrType);
    assert(l_binFileSize % sizeof(DdrType) == 0);

    // Bin file storage
    //l_memVec.reserve(l_binFileSizeInDdrWords);
    l_memVec.resize(l_binFileSizeInDdrWords);
    DdrType *l_mem = &l_memVec[0];

    // Read the bin file
    l_if.read((char*)l_mem, l_binFileSize);
    if (l_if) {
      std::cout << "INFO: loaded " << l_binFileSize << " bytes from " << p_BinFileName << "\n";
    } else {
      l_memVec.clear();
      std::cout << "ERROR: loaded only " << l_if.gcount() << " bytes from " << p_BinFileName << "\n";
    }
    l_if.close();

    // Debug print the file content
  } else {
    std::cout << "ERROR: failed to open file " + p_BinFileName + "\n";
  }

  return(l_memVec);
}

bool
writeBinFile(std::string p_BinFileName, std::vector<DdrType> &p_MemVec)
{
  bool ok = false;  
  std::ofstream l_of(p_BinFileName.c_str(), std::ios::binary);
  if (l_of.is_open()) {
    size_t l_sizeBytes =  sizeof(DdrType) * p_MemVec.size();
    l_of.write((char*)&p_MemVec[0], l_sizeBytes);
    if (l_of.tellp() == l_sizeBytes) {
      std::cout << "INFO: wrote " << l_sizeBytes << " bytes to " << p_BinFileName << "\n";
      ok = true;
    } else {
      std::cout << "ERROR: wrote only " << l_of.tellp() << " bytes to " << p_BinFileName << "\n";
    }
    l_of.close();
  }
  return(ok);
}

#if TEST_SDX
  typedef std::chrono::time_point<std::chrono::high_resolution_clock> TimePointType;

  void
  showTimeData(std::string p_Task, TimePointType &t1, TimePointType &t2)
  {
    t2 = std::chrono::high_resolution_clock::now();    
    std::chrono::duration<double> l_durationSec = t2 - t1;
    std::cout << "  DATA: time " << p_Task
              << "  " << std::fixed << std::setprecision(6)
              << l_durationSec.count() * 1e3 << " msec\n";
  }
#endif

template <typename T>
T convert(const char* data) {
    return *reinterpret_cast<const T*>(data);
}

#define ENUM_CASE(ENUM)     \
    case ENUM:              \
        printf(#ENUM "\n"); \
        break

#define BITFIELD_SETUP(type)                  \
    auto value = convert<type>(field.data()); \
    printf("[ ")

#define BITFIELD_PRINT(FIELD) \
    if (value & FIELD) {      \
        printf(#FIELD " ");   \
    }

#define BITFIELD_END() printf("]\n")

std::pair<int, const char*> platform_info[] = {
    {CL_PLATFORM_PROFILE, "profile"},
    {CL_PLATFORM_VERSION, "version"},
    {CL_PLATFORM_NAME, "name"},
    {CL_PLATFORM_VENDOR, "vendor"},
    {CL_PLATFORM_EXTENSIONS, "extensions"}};

std::pair<int, const char*> device_info[] = {
    {CL_DEVICE_TYPE, "type"},
    {CL_DEVICE_VENDOR_ID, "vendor id"},
    {CL_DEVICE_MAX_COMPUTE_UNITS, "max compute units"},
    {CL_DEVICE_MAX_WORK_ITEM_DIMENSIONS, "max work item dimensions"},
    {CL_DEVICE_MAX_WORK_GROUP_SIZE, "max work group size"},
    {CL_DEVICE_MAX_WORK_ITEM_SIZES, "max work item sizes"},
    {CL_DEVICE_PREFERRED_VECTOR_WIDTH_CHAR, "preferred vector width char"},
    {CL_DEVICE_PREFERRED_VECTOR_WIDTH_SHORT, "preferred vector width short"},
    {CL_DEVICE_PREFERRED_VECTOR_WIDTH_INT, "preferred vector width int"},
    {CL_DEVICE_PREFERRED_VECTOR_WIDTH_LONG, "preferred vector width long"},
    {CL_DEVICE_PREFERRED_VECTOR_WIDTH_FLOAT, "preferred vector width float"},
    {CL_DEVICE_PREFERRED_VECTOR_WIDTH_DOUBLE, "preferred vector width double"},
    {CL_DEVICE_MAX_CLOCK_FREQUENCY, "max clock frequency"},
    {CL_DEVICE_ADDRESS_BITS, "address bits"},
    {CL_DEVICE_MAX_READ_IMAGE_ARGS, "max read image args"},
    {CL_DEVICE_MAX_WRITE_IMAGE_ARGS, "max write image args"},
    {CL_DEVICE_MAX_MEM_ALLOC_SIZE, "max mem alloc size"},
    {CL_DEVICE_IMAGE2D_MAX_WIDTH, "image2d max width"},
    {CL_DEVICE_IMAGE2D_MAX_HEIGHT, "image2d max height"},
    {CL_DEVICE_IMAGE3D_MAX_WIDTH, "image3d max width"},
    {CL_DEVICE_IMAGE3D_MAX_HEIGHT, "image3d max height"},
    {CL_DEVICE_IMAGE3D_MAX_DEPTH, "image3d max depth"},
    {CL_DEVICE_IMAGE_SUPPORT, "image support"},
    {CL_DEVICE_MAX_PARAMETER_SIZE, "max parameter size"},
    {CL_DEVICE_MAX_SAMPLERS, "max samplers"},
    {CL_DEVICE_MEM_BASE_ADDR_ALIGN, "mem base addr align"},
    {CL_DEVICE_MIN_DATA_TYPE_ALIGN_SIZE, "min data type align size"},
    {CL_DEVICE_SINGLE_FP_CONFIG, "single fp config"},
    {CL_DEVICE_GLOBAL_MEM_CACHE_TYPE, "global mem cache type"},
    {CL_DEVICE_GLOBAL_MEM_CACHELINE_SIZE, "global mem cacheline size"},
    {CL_DEVICE_GLOBAL_MEM_CACHE_SIZE, "global mem cache size"},
    {CL_DEVICE_GLOBAL_MEM_SIZE, "global mem size"},
    {CL_DEVICE_MAX_CONSTANT_BUFFER_SIZE, "max constant buffer size"},
    {CL_DEVICE_MAX_CONSTANT_ARGS, "max constant args"},
    {CL_DEVICE_LOCAL_MEM_TYPE, "local mem type"},
    {CL_DEVICE_LOCAL_MEM_SIZE, "local mem size"},
    {CL_DEVICE_ERROR_CORRECTION_SUPPORT, "error correction support"},
    {CL_DEVICE_PROFILING_TIMER_RESOLUTION, "profiling timer resolution"},
    {CL_DEVICE_ENDIAN_LITTLE, "endian little"},
    {CL_DEVICE_AVAILABLE, "available"},
    {CL_DEVICE_COMPILER_AVAILABLE, "compiler available"},
    {CL_DEVICE_EXECUTION_CAPABILITIES, "execution capabilities"},
    {CL_DEVICE_QUEUE_PROPERTIES, "queue properties"},
    {CL_DEVICE_NAME, "name"},
    {CL_DEVICE_VENDOR, "vendor"},
    {CL_DRIVER_VERSION, "version"},
    {CL_DEVICE_PROFILE, "profile"},
    {CL_DEVICE_VERSION, "version"},
    {CL_DEVICE_EXTENSIONS, "extensions"},
    {CL_DEVICE_PLATFORM, "platform"},
    {CL_DEVICE_DOUBLE_FP_CONFIG, "double fp config"},
    {CL_DEVICE_PREFERRED_VECTOR_WIDTH_HALF, "preferred vector width half"},
    {CL_DEVICE_HOST_UNIFIED_MEMORY, "host unified memory"},
    {CL_DEVICE_NATIVE_VECTOR_WIDTH_CHAR, "native vector width char"},
    {CL_DEVICE_NATIVE_VECTOR_WIDTH_SHORT, "native vector width short"},
    {CL_DEVICE_NATIVE_VECTOR_WIDTH_INT, "native vector width int"},
    {CL_DEVICE_NATIVE_VECTOR_WIDTH_LONG, "native vector width long"},
    {CL_DEVICE_NATIVE_VECTOR_WIDTH_FLOAT, "native vector width float"},
    {CL_DEVICE_NATIVE_VECTOR_WIDTH_DOUBLE, "native vector width double"},
    {CL_DEVICE_NATIVE_VECTOR_WIDTH_HALF, "native vector width half"},
    {CL_DEVICE_OPENCL_C_VERSION, "opencl c version"},
    {CL_DEVICE_LINKER_AVAILABLE, "linker available"},
    {CL_DEVICE_BUILT_IN_KERNELS, "built in kernels"},
    {CL_DEVICE_IMAGE_MAX_BUFFER_SIZE, "image max buffer size"},
    {CL_DEVICE_IMAGE_MAX_ARRAY_SIZE, "image max array size"},
    {CL_DEVICE_PARENT_DEVICE, "parent device"},
    {CL_DEVICE_PARTITION_MAX_SUB_DEVICES, "partition max sub devices"},
    {CL_DEVICE_PARTITION_PROPERTIES, "partition properties"},
    {CL_DEVICE_PARTITION_AFFINITY_DOMAIN, "partition affinity domain"},
    {CL_DEVICE_PARTITION_TYPE, "partition type"},
    {CL_DEVICE_REFERENCE_COUNT, "reference count"},
    {CL_DEVICE_PREFERRED_INTEROP_USER_SYNC, "preferred interop user sync"},
    {CL_DEVICE_PRINTF_BUFFER_SIZE, "printf buffer size"}};

template <typename T, size_t N>
int sizeof_array(T (&)[N]) {
    return N;
}

void print_platform_info(boost::compute::platform const &platform) {
    for (int i = 0; i < sizeof_array(platform_info); ++i) {
        std::string str = std::move(platform.get_info<std::string>(platform_info[i].first));
        printf("platform %-11s: %s\n", platform_info[i].second, str.c_str());
    }
}

void print_device_info(boost::compute::device const &device) {
    for (int i = 0; i < sizeof_array(device_info); i++) {
        std::string field = std::move(device.get_info<std::string>(device_info[i].first));

        printf("  device %-32s: ", device_info[i].second);
        switch (device_info[i].first) {
            case CL_DEVICE_ADDRESS_BITS:
            case CL_DEVICE_GLOBAL_MEM_CACHELINE_SIZE:
            case CL_DEVICE_MAX_CLOCK_FREQUENCY:
            case CL_DEVICE_MAX_COMPUTE_UNITS:
            case CL_DEVICE_MAX_CONSTANT_ARGS:
            case CL_DEVICE_MAX_READ_IMAGE_ARGS:
            case CL_DEVICE_MAX_SAMPLERS:
            case CL_DEVICE_MAX_WORK_ITEM_DIMENSIONS:
            case CL_DEVICE_MAX_WRITE_IMAGE_ARGS:
            case CL_DEVICE_MEM_BASE_ADDR_ALIGN:
            case CL_DEVICE_MIN_DATA_TYPE_ALIGN_SIZE:
            case CL_DEVICE_PREFERRED_VECTOR_WIDTH_CHAR:
            case CL_DEVICE_PREFERRED_VECTOR_WIDTH_SHORT:
            case CL_DEVICE_PREFERRED_VECTOR_WIDTH_INT:
            case CL_DEVICE_PREFERRED_VECTOR_WIDTH_LONG:
            case CL_DEVICE_PREFERRED_VECTOR_WIDTH_FLOAT:
            case CL_DEVICE_PREFERRED_VECTOR_WIDTH_DOUBLE:
            case CL_DEVICE_PREFERRED_VECTOR_WIDTH_HALF:
            case CL_DEVICE_NATIVE_VECTOR_WIDTH_CHAR:
            case CL_DEVICE_NATIVE_VECTOR_WIDTH_SHORT:
            case CL_DEVICE_NATIVE_VECTOR_WIDTH_INT:
            case CL_DEVICE_NATIVE_VECTOR_WIDTH_LONG:
            case CL_DEVICE_NATIVE_VECTOR_WIDTH_FLOAT:
            case CL_DEVICE_NATIVE_VECTOR_WIDTH_DOUBLE:
            case CL_DEVICE_NATIVE_VECTOR_WIDTH_HALF:
            case CL_DEVICE_VENDOR_ID:
            case CL_DEVICE_PARTITION_MAX_SUB_DEVICES:
            case CL_DEVICE_REFERENCE_COUNT:
                printf("%d\n", convert<cl_uint>(field.data()));
                break;

            case CL_DEVICE_AVAILABLE:
            case CL_DEVICE_COMPILER_AVAILABLE:
            case CL_DEVICE_ENDIAN_LITTLE:
            case CL_DEVICE_ERROR_CORRECTION_SUPPORT:
            case CL_DEVICE_IMAGE_SUPPORT:
            case CL_DEVICE_HOST_UNIFIED_MEMORY:
            case CL_DEVICE_LINKER_AVAILABLE:
            case CL_DEVICE_PREFERRED_INTEROP_USER_SYNC:
                printf("%s\n",
                       convert<cl_bool>(field.data()) ? "true" : "false");
                break;

            case CL_DEVICE_GLOBAL_MEM_CACHE_SIZE:
            case CL_DEVICE_GLOBAL_MEM_SIZE:
            case CL_DEVICE_LOCAL_MEM_SIZE:
            case CL_DEVICE_MAX_CONSTANT_BUFFER_SIZE:
            case CL_DEVICE_MAX_MEM_ALLOC_SIZE:
                printf("%lu\n", convert<cl_ulong>(field.data()));
                break;

            case CL_DEVICE_IMAGE2D_MAX_HEIGHT:
            case CL_DEVICE_IMAGE2D_MAX_WIDTH:
            case CL_DEVICE_IMAGE3D_MAX_DEPTH:
            case CL_DEVICE_IMAGE3D_MAX_HEIGHT:
            case CL_DEVICE_IMAGE3D_MAX_WIDTH:
            case CL_DEVICE_MAX_PARAMETER_SIZE:
            case CL_DEVICE_MAX_WORK_GROUP_SIZE:
            case CL_DEVICE_PROFILING_TIMER_RESOLUTION:
            case CL_DEVICE_PRINTF_BUFFER_SIZE:
            case CL_DEVICE_IMAGE_MAX_BUFFER_SIZE:
            case CL_DEVICE_IMAGE_MAX_ARRAY_SIZE:
                printf("%zu\n", convert<size_t>(field.data()));
                break;

            case CL_DEVICE_MAX_WORK_ITEM_SIZES: {
                size_t max_dim = 0;
                max_dim = device.get_info<size_t>(CL_DEVICE_MAX_WORK_ITEM_DIMENSIONS);
                printf("[  ");
                for (int i = 0; i < (int)max_dim; i++) {
                    printf("\b%zu  ",
                           convert<size_t>(&field[i * sizeof(size_t)]));
                }
                printf("\b]\n");
            } break;

            case CL_DEVICE_EXTENSIONS:
            case CL_DEVICE_NAME:
            case CL_DEVICE_PROFILE:
            case CL_DEVICE_VENDOR:
            case CL_DEVICE_VERSION:
            case CL_DRIVER_VERSION:
            case CL_DEVICE_BUILT_IN_KERNELS:
            case CL_DEVICE_OPENCL_C_VERSION:
                printf("%s\n", field.data());
                break;

            case CL_DEVICE_TYPE:
                switch (convert<cl_device_type>(field.data())) {
                    ENUM_CASE(CL_DEVICE_TYPE_ACCELERATOR);
                    ENUM_CASE(CL_DEVICE_TYPE_CPU);
                    ENUM_CASE(CL_DEVICE_TYPE_GPU);
                    ENUM_CASE(CL_DEVICE_TYPE_DEFAULT);
                    default:
                        printf("UNKNOWN\n");
                }
                break;
            case CL_DEVICE_GLOBAL_MEM_CACHE_TYPE:
                switch (convert<cl_device_mem_cache_type>(field.data())) {
                    ENUM_CASE(CL_NONE);
                    ENUM_CASE(CL_READ_ONLY_CACHE);
                    ENUM_CASE(CL_READ_WRITE_CACHE);
                    default:
                        printf("UNKNOWN\n");
                }
                break;
            case CL_DEVICE_LOCAL_MEM_TYPE:
                switch (convert<cl_device_mem_cache_type>(field.data())) {
                    ENUM_CASE(CL_LOCAL);
                    ENUM_CASE(CL_GLOBAL);
                    default:
                        printf("UNKNOWN\n");
                }
                break;
            case CL_DEVICE_PARTITION_PROPERTIES:
                switch (convert<cl_device_mem_cache_type>(field.data())) {
                    case 0:
                        printf("NONE\n");
                        break;
                        ENUM_CASE(CL_DEVICE_PARTITION_EQUALLY);
                        ENUM_CASE(CL_DEVICE_PARTITION_BY_COUNTS);
                        ENUM_CASE(CL_DEVICE_PARTITION_BY_AFFINITY_DOMAIN);
                    default:
                        printf("UNKNOWN\n");
                }
                break;
            case CL_DEVICE_QUEUE_PROPERTIES: {
                BITFIELD_SETUP(cl_command_queue_properties);
                BITFIELD_PRINT(CL_QUEUE_OUT_OF_ORDER_EXEC_MODE_ENABLE);
                BITFIELD_PRINT(CL_QUEUE_PROFILING_ENABLE);
                BITFIELD_END();
            } break;
            case CL_DEVICE_DOUBLE_FP_CONFIG:
            case CL_DEVICE_HALF_FP_CONFIG:
            case CL_DEVICE_SINGLE_FP_CONFIG: {
                BITFIELD_SETUP(cl_device_fp_config);
                BITFIELD_PRINT(CL_FP_DENORM);
                BITFIELD_PRINT(CL_FP_INF_NAN);
                BITFIELD_PRINT(CL_FP_ROUND_TO_ZERO);
                BITFIELD_PRINT(CL_FP_ROUND_TO_INF);
                BITFIELD_PRINT(CL_FP_FMA);
                BITFIELD_PRINT(CL_FP_SOFT_FLOAT);
                BITFIELD_END();
            } break;
            case CL_DEVICE_EXECUTION_CAPABILITIES: {
                BITFIELD_SETUP(cl_device_exec_capabilities);
                BITFIELD_PRINT(CL_EXEC_KERNEL);
                BITFIELD_PRINT(CL_EXEC_NATIVE_KERNEL);
                BITFIELD_END();
            } break;
            case CL_DEVICE_PARTITION_AFFINITY_DOMAIN: {
                BITFIELD_SETUP(cl_device_affinity_domain);
                BITFIELD_PRINT(CL_DEVICE_AFFINITY_DOMAIN_NUMA);
                BITFIELD_PRINT(CL_DEVICE_AFFINITY_DOMAIN_L4_CACHE);
                BITFIELD_PRINT(CL_DEVICE_AFFINITY_DOMAIN_L3_CACHE);
                BITFIELD_PRINT(CL_DEVICE_AFFINITY_DOMAIN_L2_CACHE);
                BITFIELD_PRINT(CL_DEVICE_AFFINITY_DOMAIN_L1_CACHE);
                BITFIELD_PRINT(CL_DEVICE_AFFINITY_DOMAIN_NEXT_PARTITIONABLE);
                BITFIELD_END();
            }
            break;
            case CL_DEVICE_PLATFORM: {
                std::string field = device.platform().name();
                printf("%s\n", field.c_str());
            } break;
            default:
                printf("N/A \n");
                continue;
        }
        field[0] = '\0';
    }
}

int main(int argc, char** argv)
{
  if (argc < 4){
    printf("ERROR: passed %d arguments instead of %d, exiting\n",
           argc, 4);
    printf("  Usage:\n    gemx_host.exe  gemx.xclbin  app.bin  app_out.bin\n");
    return EXIT_FAILURE;
  }
  
  std::string l_xclbinFile(argv[1]);
  std::string l_binFile(argv[2]);
  std::string l_binFileOut(argv[3]);
  unsigned int l_kernelId = 0;
  unsigned int l_kernelNameId = 0;
  /*if (argc > 4) {
    l_kernelId = atoi(argv[4]);
    assert(l_kernelId < 4);
  }
  if (argc > 5) {
	l_kernelNameId = atoi(argv[5]);
	assert(l_kernelNameId < 4);
  }
  else {
	l_kernelNameId = l_kernelId;
  }*/
  boost::compute::device device = boost::compute::system::default_device();
  //std::cout << "Device: " << device.name() << std::endl;
  //std::cout << "Platform: " << device.platform().name() << std::endl;
  print_platform_info(device.platform());
  print_device_info(device);
  printf("GEMX:   %s  %s  %s %s\n",
         argv[0], l_xclbinFile.c_str(), l_binFile.c_str(), l_binFileOut.c_str());
  
  // Load the bin file
  std::vector<DdrType> l_memVec = loadBinFile(l_binFile);
  if (l_memVec.empty()) {
    return EXIT_FAILURE;
  }
  DdrType *l_mem = &l_memVec[0];
  
  
  std::vector<DdrType> l_memVecOut[GEMX_numKernels];
  #if TEST_SDX
    #include <chrono>
    TimePointType l_tp[10];
    unsigned int l_tpIdx = 0;
    l_tp[l_tpIdx] = std::chrono::high_resolution_clock::now(); 
    
    // ################# HW run through SDX #################
    // Init FPGA
    gemx::Fpga l_fpga;
    std::string kernelNames[GEMX_numKernels];
    for (int i=0; i<GEMX_numKernels; ++i){
	kernelNames[i] = "gemxKernel_" + std::to_string(i);
    }
    //std::string l_kernelName("gemxKernel_"+std::to_string(l_kernelNameId));
    if (l_fpga.loadXclbinWithoutEvent(l_xclbinFile, kernelNames)) {
      std::cout << "INFO: created kernels" << std::endl;
    } else {
      std::cerr << "ERROR: failed to load " + l_xclbinFile + "\n";
      return EXIT_FAILURE;
    }
    showTimeData("loadXclbin", l_tp[l_tpIdx], l_tp[l_tpIdx+1]); l_tpIdx++;

    // Transfer data to FPGA
    gemx::MemDesc l_memDesc(l_memVec.size() * sizeof(DdrType) / GEMX_pageSizeBytes, l_memVec.data());
    assert(l_memVec.size() * sizeof(DdrType) % GEMX_pageSizeBytes == 0);
    if (l_fpga.copyToFpgaWithoutEvent(l_memDesc)) {
      std::cout << "INFO: transferred data to FPGA" << std::endl;
    } else {
      std::cerr << "ERROR: failed to copy data to FPGA DDR\n";
      return EXIT_FAILURE;
    }
    showTimeData("copyToFpga", l_tp[l_tpIdx], l_tp[l_tpIdx+1]); l_tpIdx++;

    // Gemx kernel ops
    if (l_fpga.callKernelWithoutEvent()) {
      std::cout << "INFO: Executed kernel" << std::endl;
    } else {
      std::cerr << "ERROR: failed to call kernel \n";
      return EXIT_FAILURE;
    }
    showTimeData("callKernel", l_tp[l_tpIdx], l_tp[l_tpIdx+1]); l_tpIdx++;
  
    gemx::MemDesc l_memDescOuts[GEMX_numKernels];
    for (unsigned int i=0; i<GEMX_numKernels; ++i) {
      l_memVecOut[i].resize(l_memVec.size());
      gemx::MemDesc l_memDescOut(l_memDesc.sizePages(), l_memVecOut[i].data());
      l_memDescOuts[i]= l_memDescOut;
    }
    // Transfer data back to host
    if (l_fpga.copyFromFpgaWithoutEvent(l_memDescOuts)) {
      std::cout << "INFO: Transferred data from FPGA" << std::endl;
    } else {
      std::cerr << "ERROR: failed to copy data from FPGA DDR\n";
      return EXIT_FAILURE;
    }    
    showTimeData("copyFromFpga", l_tp[l_tpIdx], l_tp[l_tpIdx+1]); l_tpIdx++;
    showTimeData("total", l_tp[0], l_tp[l_tpIdx]); l_tpIdx++;
    showTimeData("subtotalFpga", l_tp[1], l_tp[l_tpIdx]); l_tpIdx++; // Host->DDR, kernel, DDR->host
    
  #else
    // ################# SW run through HLS #################
    // Gemx kernel ops
    gemxKernel_0(l_mem, l_mem);
    
    l_memVecOut[0] = l_memVec;
  
  #endif
  
  // Write out the received data
 for (int i=0; i<GEMX_numKernels; ++i) {
    std::size_t pos0 = l_binFileOut.find("/");
    std::size_t pos1 = l_binFileOut.find("app_out");
    std::size_t pos2 = l_binFileOut.find(".bin");
    //std::string binFileOutName = l_binFileOut.substr(0,10) + std::to_string(i) + l_binFileOut.substr(10,4);
    std::string binFileOutName =l_binFileOut.substr(0,pos0+1)+l_binFileOut.substr(pos1,7) + std::to_string(i) + l_binFileOut.substr(pos2,4);
   if (!writeBinFile(binFileOutName, l_memVecOut[i])) {
      std::cerr << "ERROR: failed to write output file " + binFileOutName + "\n";
      return EXIT_FAILURE;
    }
 }

  return EXIT_SUCCESS;
}

  
