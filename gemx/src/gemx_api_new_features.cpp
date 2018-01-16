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
 *  @brief Simple GEMM example of C++ API client interaction with GEMMX linear algebra accelerator on Xilinx FPGA 
 *
 *  $DateTime: 2017/08/18 08:31:34 $
 *  $Author: jzejda $
 */

// Prerequisites:
//  - Boost installation (edit the Makefile with your boost path)
//  - Compiled C++ to bitstream accelerator kernel
//     - use "make run_hw"
//     - or get a pre-compiled copy of the out_hw/gemx.xclbin)
// Compile and run this API example:
//   make out_host/gemx_api_gemm.exe
//   out_host/gemx_api_gemm.exe
//     # No argumens will show help message
// You can also test it with a cpu emulation accelerator kernel (faster to combile, make run_cpu_em)
//   ( setenv XCL_EMULATION_MODE true ; out_host/gemx_api_gemm.exe out_cpu_emu/gemx.xclbin )
 
#include <stdio.h>
#include <string>
#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>
#include <iomanip>
#include <chrono>
#include <stdio.h>  // fgets for popen

#include "gemx_kernel.h"
#if TEST_SDX
#include "gemx_fpga.h"
#endif
#include "gemx_gen_bin.h"

//#define VERBOSE 0 

bool checkDim(unsigned int p_Val, unsigned int p_Mod, unsigned int p_Min) {
  bool l_ok = true;
  if (p_Val % p_Mod != 0) {
    std::cerr << "ERROR: value " << p_Val << " must be multiple of " << p_Mod << "\n";
    l_ok = false;
  }
  if (p_Val < p_Min) {
    std::cerr << "ERROR: value " << p_Val << " must be at least " << p_Min << "\n";
    l_ok = false;
  }
  return(l_ok);
}

float getBoardFreqMHz(unsigned int p_BoardId) {
  std::string l_freqCmd = "$XILINX_OPENCL/runtime/bin/xbsak query -d" + std::to_string(p_BoardId);;
  std::cout <<"DEBUG: query board info with command - " << l_freqCmd << std::endl;
  float l_freq = -1;
  char l_lineBuf[256];
  std::shared_ptr<FILE> l_pipe(popen(l_freqCmd.c_str(), "r"), pclose);
  //if (!l_pipe) throw std::runtime_error("ERROR: popen(" + l_freqCmd + ") failed");
  if (!l_pipe) std::cout << ("ERROR: popen(" + l_freqCmd + ") failed");
  bool l_nextLine_isFreq = false;
  while (l_pipe && fgets(l_lineBuf, 256, l_pipe.get()) ) {
    std::string l_line(l_lineBuf);
    std::cout << "DEBUG: read line " << l_line << std::endl;
    if (l_nextLine_isFreq) {
      std::string l_prefix, l_val, l_mhz;
      std::stringstream l_ss(l_line);
      l_ss >> l_prefix >> l_val >> l_mhz;
      l_freq = std::stof(l_val);
      assert(l_mhz == "MHz");
      break;
    } else if (l_line.find("OCL Frequency:") != std::string::npos) {
      l_nextLine_isFreq = true;
    }
  }
  if (l_freq == -1) {
	//if xbsak does not work, as happens on F1, put the XOCC achieved kernel frequcy here
	l_freq = 246.6;
    std::cout << "INFO: Failed to get board frequency by xbsak. This is normal for cpu and hw emulation, using -1 MHz for reporting.\n";
  }
  return(l_freq);
}

int main(int argc, char **argv)
{
#if 0
  //############  UI and GEMM problem size  ############
  if (argc < 2) {
    std::cerr << "Usage:\n"
              <<  "  gemx_api_gemm.exe <path/gemx.xclbin> [M K N  [LdA LdB LdC] ]\n"
              <<  "  Examples:\n"
              <<  "    gemx_api_gemm.exe   out_hw/gemx.xclbin\n"
              <<  "    gemx_api_gemm.exe   out_hw/gemx.xclbin  32 64 32\n"
              <<  "    gemx_api_gemm.exe   out_hw/gemx.xclbin  32 128 256  256 128 128\n"
              <<  "    gemx_api_gemm.exe   out_hw/gemx.xclbin  32 128 256  288 160 160\n";
    exit(2);
  }
#endif

  unsigned int l_argIdx = 1;
  std::string l_xclbinFile(argv[l_argIdx]);
  unsigned int l_kernelId = 0;

#if 0
  // Row major  C  M rows N cols  =  A  M rows K cols  *  B  K rows N cols
  //   MatType - tensor like type to allocate/store/align memory; you can use your own type instead
  //   Min size is the array edge (e.g., 32 on ku115), see GenGemm::check() for example of arg checking functions
  unsigned int l_ddrW = GEMX_ddrWidth;
  unsigned int l_M = l_ddrW,  l_K = l_ddrW,  l_N = l_ddrW;  // the smallest matrices for flow testing
  if (argc > ++l_argIdx) {l_M = atoi(argv[l_argIdx]);}  
  if (argc > ++l_argIdx) {l_K = atoi(argv[l_argIdx]);}  
  if (argc > ++l_argIdx) {l_N = atoi(argv[l_argIdx]);}  
  unsigned int l_LdA = l_K,  l_LdB= l_N,  l_LdC = l_N;
  if (argc > ++l_argIdx) {l_LdA = atoi(argv[l_argIdx]);}
  if (argc > ++l_argIdx) {l_LdB = atoi(argv[l_argIdx]);}
  if (argc > ++l_argIdx) {l_LdC = atoi(argv[l_argIdx]);}
  
  if (! (
      checkDim(l_M, l_ddrW*GEMX_gemmMBlocks, l_ddrW*GEMX_gemmMBlocks) &&
      checkDim(l_K, l_ddrW*GEMX_gemmKBlocks, l_ddrW*GEMX_gemmKBlocks) &&
      checkDim(l_N, l_ddrW*GEMX_gemmNBlocks, l_ddrW*GEMX_gemmNBlocks) &&
      checkDim(l_LdA, l_ddrW, l_K) &&
      checkDim(l_LdB, l_ddrW, l_N) &&
      checkDim(l_LdC, l_ddrW, l_N)
    )) {
    return EXIT_FAILURE;
  }  
  
  printf("GEMX-gemm C++ API example using accelerator image \n",
         l_xclbinFile.c_str());
    
  //############  Client code - prepare the gemm problem input  ############
  ProgramType l_program[GEMX_numKernels];  // Holds instructions and controls memory allocation
  std::string l_handleA[GEMX_numKernels];
  std::string l_handleB[GEMX_numKernels];
  std::string l_handleC[GEMX_numKernels];

  bool l_newAllocA[GEMX_numKernels];
  bool l_newAllocB[GEMX_numKernels];
  bool l_newAllocC[GEMX_numKernels];

  unsigned int l_pageA[GEMX_numKernels];
  unsigned int l_pageB[GEMX_numKernels];
  unsigned int l_pageC[GEMX_numKernels];

  MatType l_matA[GEMX_numKernels];
  MatType l_matB[GEMX_numKernels];
  MatType l_matC[GEMX_numKernels];
  
  GemmArgsType l_gemmArgs[GEMX_numKernels];
  KargsType l_kargs[GEMX_numKernels];

  for (int i=0; i<GEMX_numKernels; ++i) {
	l_handleA[i] = "A"+std::to_string(i);
	l_handleB[i] = "B"+std::to_string(i);
	l_handleC[i] = "C"+std::to_string(i);
  	// Allocate all pages before getting any address
  	l_pageA[i] = l_program[i].allocPages(l_handleA[i], l_newAllocA[i], l_M * l_LdA);
  	l_pageB[i] = l_program[i].allocPages(l_handleB[i], l_newAllocB[i], l_K * l_LdB);
  	l_pageC[i] = l_program[i].allocPages(l_handleC[i], l_newAllocC[i], l_M * l_LdC);
  	
	// Get addresses where matrices are stored
  	l_matA[i].init(l_M, l_K, l_LdA, l_program[i].getPageAddr(l_pageA[i]));
  	l_matB[i].init(l_K, l_N, l_LdB, l_program[i].getPageAddr(l_pageB[i]));
  	l_matC[i].init(l_M, l_N, l_LdC, l_program[i].getPageAddr(l_pageC[i]));
  	
	// Fill inputs with random data
	if (l_newAllocA[i]) {
    	l_matA[i].fillMod(67, 1);
  	}
  	if (l_newAllocB[i]) {
    	l_matB[i].fillMod(129, 65);
  	}
  	
	// You can fill the input matrices with your own data by e.g.,
  	//for (int row = 0; row < l_matA.rows();  ++row) {
  	//  for (int col = 0; col < l_matA.cols();  ++col) {
  	//    GEMX_dataType l_val = 17 + 2 * row + 3 * col
  	//    l_matA.getVal(row, col) = l_val;
  	//  }
  	//}
  
	//############  Compile program for the accelerator with a single GEMM instruction  ############
  	// GemmArgsType, KargsType - helper types to compile and check the accelerator program
    l_gemmArgs[i].init(
      l_pageA[i], l_pageB[i], l_pageC[i],
      l_M, l_K, l_N,
      l_LdA, l_LdB, l_LdC
    );
    l_kargs[i].setGemmArgs(l_gemmArgs[i]);
    l_kargs[i].store(l_program[i].addInstr(), 0);
    std::cout << "Added instruction GEMM " << l_M << "x" << l_K << "x" << l_N << "  ";
  }

  std::string kernelNames[GEMX_numKernels];
  gemx::MemDesc l_memDesc[GEMX_numKernels];
 
  for (int i=0; i<GEMX_numKernels; ++i) { 
  	l_memDesc[i] = l_program[i].getMemDesc();
  }
  
  //############  Runtime reporting Infra  ############
  TimePointType l_tp[10];
  unsigned int l_tpIdx = 0;
  l_tp[l_tpIdx] = std::chrono::high_resolution_clock::now(); 

  //############  Run FPGA accelerator  ############
  // Init FPGA
  gemx::Fpga l_fpga;

  for (int i=0; i<GEMX_numKernels; ++i){
	kernelNames[i] = "gemxKernel_" + std::to_string(i);
  }
  if (l_fpga.loadXclbin(l_xclbinFile, kernelNames)) {
    std::cout << "INFO: created kernels" << std::endl;
  } else {
    std::cerr << "ERROR: failed to load " + l_xclbinFile + "\n";
    return EXIT_FAILURE;
  }
  showTimeData("loadXclbin", l_tp[l_tpIdx], l_tp[l_tpIdx+1]); l_tpIdx++;

  //create buffers for transferring data to FPGA
  if (!l_fpga.createBuffers(l_memDesc)) {
    std::cerr << "ERROR: failed to create buffers for transffering data to FPGA DDR\n";
    return EXIT_FAILURE;
  }
  showTimeData("created buffers", l_tp[l_tpIdx], l_tp[l_tpIdx+1]); l_tpIdx++;
  
  // Transfer data to FPGA
  if (l_fpga.copyToFpga()) {
    (VERBOSE > 0) && std::cout << "INFO: transferred data to FPGA" << std::endl;
  } else {
    std::cerr << "ERROR: failed to copy data to FPGA DDR\n";
    return EXIT_FAILURE;
  }
  showTimeData("copyToFpga", l_tp[l_tpIdx], l_tp[l_tpIdx+1]); l_tpIdx++;

  // Gemx kernel ops
  if (l_fpga.callKernels()) {
    (VERBOSE > 0) && std::cout << "INFO: Executed kernel" << std::endl;
  } else {
    std::cerr << "ERROR: failed to call kernels ";
	for (int i=0; i<GEMX_numKernels; ++i) {
		std::cerr << kernelNames[i] << " ";
	}
	std::cerr << "\n";
    return EXIT_FAILURE;
  }
  showTimeData("callKernel", l_tp[l_tpIdx], l_tp[l_tpIdx+1]); l_tpIdx++;

  // Transfer data back to host - due to lazy evaluation this is generally wheer the accelerator performs the work
  if (l_fpga.copyFromFpga()) {
    (VERBOSE > 0) && std::cout << "INFO: Transferred data from FPGA" << std::endl;
  } else {
    std::cerr << "ERROR: failed to copy data from FPGA DDR\n";
    return EXIT_FAILURE;
  }
  showTimeData("copyFromFpga", l_tp[l_tpIdx], l_tp[l_tpIdx+1]); l_tpIdx++;
  showTimeData("total", l_tp[0], l_tp[l_tpIdx]); l_tpIdx++;
  double l_timeApiInMs = -1;
  showTimeData("subtotalFpga", l_tp[2], l_tp[l_tpIdx], &l_timeApiInMs); l_tpIdx++; // Host->DDR, kernel, DDR->host
  
  //############  Get the exact kernel time from HW cycle counters on the accelerator  ############
  float l_boardFreqMHz = getBoardFreqMHz(0);
  unsigned long int l_Ops = 2ull * l_M * l_N * l_K;
  KargsType l_kargsRes[GEMX_numKernels];
  KargsOpType l_op[GEMX_numKernels];
  gemx::InstrResArgs l_instrRes[GEMX_numKernels];
  unsigned long int l_cycleCount[GEMX_numKernels];
  unsigned long int l_maxCycleCount=0;
  double l_timeKernelInMs[GEMX_numKernels];
  double l_maxTimeKernelInMs=0;
  double l_perfKernelInTops[GEMX_numKernels];
  double l_totalPerfKernelInTops=0;
  double l_perfApiInTops;
  double l_timeMsAt100pctEff;
  double l_effKernelPct;
  double l_effApiPct;

  for (int i=0; i<GEMX_numKernels; ++i) {
  	l_op[i] = l_kargsRes[i].load(l_program[i].getBaseResAddr(), 0);
  	assert(l_op[i] == KargsType::OpResult);
  	l_instrRes[i] = l_kargsRes[i].getInstrResArgs();
  	l_cycleCount[i] = l_instrRes[i].getDuration();
    l_maxCycleCount = (l_cycleCount[i] > l_maxCycleCount)? l_cycleCount[i]: l_maxCycleCount;
  	l_timeKernelInMs[i] = l_cycleCount[i] / (l_boardFreqMHz * 1e6) * 1e3;
    l_maxTimeKernelInMs = (l_timeKernelInMs[i] > l_maxTimeKernelInMs)? l_timeKernelInMs[i]: l_maxTimeKernelInMs;
	l_perfKernelInTops[i] = l_Ops / (l_timeKernelInMs[i] * 1e-3) / 1e12;
    l_totalPerfKernelInTops += l_perfKernelInTops[i];
  }
  l_perfApiInTops = (l_Ops*GEMX_numKernels) / (l_timeApiInMs * 1e-3) / 1e12;
  l_timeMsAt100pctEff = l_Ops / 2 / GEMX_ddrWidth / GEMX_ddrWidth / (l_boardFreqMHz * 1e6) * 1e3;
  l_effKernelPct = 100 * l_timeMsAt100pctEff / l_maxTimeKernelInMs;
  l_effApiPct = 100 * l_timeMsAt100pctEff / l_timeApiInMs;
  // Show time, Tops in csv format
  std::cout << std::string("DATA_CSV:,DdrWidth,Freq,M,K,N,")
             + "Ops,KernelCycles,"
             + "TimeKernelMs,TimeApiMs,"
             + "EffKernelPct,EffApiPct,"
             + "PerfKernelTops,PerfApiTops\n"
            << "DATA_CSV:," <<  GEMX_ddrWidth << "," << l_boardFreqMHz << ","
            << l_M << "," << l_K << "," << l_N << ","
            << l_Ops*GEMX_numKernels << "," << l_maxCycleCount << ","
            << l_maxTimeKernelInMs << "," << l_timeApiInMs << ","
            << l_effKernelPct << "," << l_effApiPct << ","
            << l_totalPerfKernelInTops << "," << l_perfApiInTops
            << std::endl;

  //############  Compare tha FPGA results with the reference results  ############
  // Calculate reference C = A * B
  // Since the reference is not needed on the acclerator allocate memory in any way
  std::cout << "INFO: Calculating gold values on host ..." << std::endl;
  // C matrix reference calculated on the host - l_matCref
  std::vector<GEMX_dataType> l_CmatAlloc;
  l_CmatAlloc.resize(l_M * l_LdC);
  for (int i=0; i<GEMX_numKernels; ++i) {
  	assert((l_matC[i].rows() == l_M) && (l_matC[i].cols() == l_N) && (l_matC[i].ld() == l_LdC));
  }
  MatType l_matCref(l_M, l_N, l_LdC, l_CmatAlloc.data());
  bool l_calcGold = (l_Ops <= 2ull * 2048*2048*2048);
  if (l_calcGold) {
    l_matCref.multiply(l_matA[0], l_matB[0]);
  } else {
    std::cout << "INFO: skipped gold calculation on host since it may take too long\n";
  }
  // C matrix in the DDR image received from the accelerator - l_matCfpga
  MatType l_matCfpga[GEMX_numKernels];
  for (int i=0; i<GEMX_numKernels; ++i) {
	l_matCfpga[i].init(l_M, l_N, l_LdC, l_program[i].getPageAddr(l_pageC[i]));  
  }
  
  // Verbose reporting
  bool l_verbose = ((unsigned long)l_M * l_N * l_K <= 4096);
  l_verbose && std::cout << "INFO: GEMM details C = A * B\n"
    << "A\n" <<  l_matA << "\n"
    << "B\n" <<  l_matB << "\n"
    << "Cref\n" <<  l_matCref << "\n"
    << "Cfpga\n" <<  l_matCfpga << "\n";

  // Compare FPGA and the reference
  if (l_calcGold) {
    float  l_TolRel = 1e-3,  l_TolAbs = 1e-5;
    bool l_ok;
    bool l_pass = true;
    for (int i=0; i<GEMX_numKernels; ++i) {
	 	l_ok = l_matCfpga[i].cmp(l_TolRel, l_TolAbs, l_matCref);
    	std::cout << "INFO: accelerator kernel " << i << " result " << (l_ok ? "MATCHES" : "does NOT match") << " the reference\n";
    	std::cout << "INFO: status " << (l_ok ? "PASS" : "FAIL") << "\n";
        if (!l_ok){
        	l_pass = false;
        }
    }
    return l_pass ? EXIT_SUCCESS : EXIT_FAILURE;
  } else {
    return EXIT_SUCCESS;
  }
#endif

    return EXIT_SUCCESS;
}

  
