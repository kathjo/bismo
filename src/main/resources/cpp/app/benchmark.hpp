// Copyright (c) 2019 Xilinx
//
// BSD v3 License
//
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// * Neither the name of BISMO nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include <iostream>
#include <string>
using namespace std;
#include "gemmbitserial/test/testhelpers.hpp"
#include "bismo_rt.hpp"
#include <math.h>
#include <fstream>

const char * delimiter = ", ";

void printInstrumentationHeaders(bismo_rt::InstrumentationData & data) {
  bismo_rt::InstrumentationData::iterator it;
  for(it = data.begin(); it != data.end(); it++) {
    cout << it->first << delimiter;
  }
  cout << endl;
}

void printInstrumentationData(bismo_rt::InstrumentationData & data) {
  bismo_rt::InstrumentationData::iterator it;
  for(it = data.begin(); it != data.end(); it++) {
    cout << it->second << delimiter;
  }
  cout << endl;
}

bismo_rt::InstrumentationData run_benchmark_matmul(
  size_t nrows_lhs, size_t nrows_rhs, size_t ncols, size_t nbits_lhs,
  size_t nbits_rhs
) {
  uint8_t * lhs = new uint8_t[nrows_lhs * ncols];
  uint8_t * rhs = new uint8_t[nrows_rhs * ncols];
  gemmbitserial::generateRandomVector(nbits_lhs, nrows_lhs*ncols, lhs);
  gemmbitserial::generateRandomVector(nbits_rhs, nrows_rhs*ncols, rhs);
  bismo_rt::MatMulDescriptor dscr;
  dscr.wbits = nbits_lhs;
  dscr.ibits = nbits_rhs;
  dscr.wsigned = false;
  dscr.isigned = false;
  dscr.M = nrows_lhs;
  dscr.K = ncols;
  dscr.N = nrows_rhs;
  bismo_rt::init();
  bismo_rt::InstrumentationData ret;
  try {
    bismo_rt::LayerHandle id = bismo_rt::initMatMul(dscr);
    uint8_t * accel_lhs = bismo_rt::getLayerLHSBuffer(id);
    uint8_t * accel_rhs = bismo_rt::getLayerRHSBuffer(id);
    int32_t * accel_res = bismo_rt::getLayerResBuffer(id);
    memcpy(accel_lhs, lhs, nrows_lhs * ncols);
    bismo_rt::syncLayerLHSBuffer(id);
    memcpy(accel_rhs, rhs, nrows_rhs * ncols);
    bismo_rt::syncLayerRHSBuffer(id);
    bismo_rt::execMatMul(id);
    bismo_rt::syncLayerResBuffer(id);
    ret = bismo_rt::getInstrumentationData(id);
  } catch(const char * e) {
    cout << "Exception: " << e << endl;
  }

  bismo_rt::deinit();

  delete [] lhs;
  delete [] rhs;
  return ret;
}

void benchmark_gemm_interactive() {
  while(1) {
    int rows, depth, cols, lhsbits, rhsbits;
    cout << "Enter rows depth cols, 0 to exit " << endl;
    cin >> rows;
    if(rows == 0) {
      return;
    }
    cin >> depth >> cols;
    cout << "Enter lhs and rhs bits: " << endl;
    cin >> lhsbits >> rhsbits;
    bismo_rt::InstrumentationData ret = run_benchmark_matmul(rows, cols, depth, lhsbits, rhsbits);
    printInstrumentationHeaders(ret);
    printInstrumentationData(ret);
  }
}

bismo_rt::InstrumentationData benchmark_vs_cpu(
  size_t rows, size_t cols, size_t depth, size_t lhsbits, size_t rhsbits
) {
    bismo_rt::InstrumentationData ret = run_benchmark_matmul(rows, cols, depth, lhsbits, rhsbits);
    // bismo has quite a few components
    float total_bismo = ret["run_cycles"];
    // convert exec cycles to microseconds
    total_bismo /= ret["hw_fclk_mhz"];
    // add un/padding costs
    total_bismo += ret["mat_lhs_pad_us"];
    total_bismo += ret["mat_rhs_pad_us"];
    total_bismo += ret["mat_res_unpad_us"];
    // add p2s costs
    // total_bismo += ret["mat_lhs_p2s_us"];
    total_bismo += ret["mat_rhs_p2s_us"];
    // add data movement costs
    total_bismo += ret["mat_lhs_host2accel_us"];
    total_bismo += ret["mat_res_accel2host_us"];
    total_bismo += ret["mat_rhs_host2accel_us"];
    //calculating runs vs run+wait in percent
    float fetch_utilize = ret["stg_fetch_run"];
    fetch_utilize = (fetch_utilize / (fetch_utilize + ret["stg_fetch_rcv"]))*100;
    float exec_utilize = ret["stg_exec_run"];
    exec_utilize = (exec_utilize / (exec_utilize + ret["stg_exec_rcv"]))*100;
    float result_utilize = ret["stg_result_run"];
    result_utilize = (result_utilize / (result_utilize + ret["stg_result_rcv"]))*100;
    // gemmlowp only has a single component
    float total_gemmlowp = ret["cpu_gemmlowp_exec_us"];

    bismo_rt::InstrumentationData new_ret;
    new_ret["1_rows_M"] = rows;
    new_ret["2_depth_K"] = depth;
    new_ret["3_cols_N"] = cols;
    new_ret["4_lhsbits"] = lhsbits;
    new_ret["5_rhsbits"] = rhsbits;
    new_ret["fetch_utilize"] = fetch_utilize;
    new_ret["exec_utilize"] = exec_utilize;
    new_ret["result_utilize"] = result_utilize;
    new_ret["6_total_bismo_us"] = total_bismo;
    new_ret["8_total_gemmlowp_us"] = total_gemmlowp;
    
    return new_ret;
  }

void benchmark_gemm_cpuvsaccel(string & input) {
  cout << "IMPORTANT: Remember to uncomment the following lines in bismo_rt_options.hpp: " << endl;
  cout << "#define BISMORT_MATMUL_VERIFY_AGAINST_CPU" << endl;
  cout << "#define BISMORT_BENCHMARK_GEMMLOWP" << endl;
  cout << "The CPU comparative benchmark will not work until then." << endl;
  bool headers_printed = false;
  cout << "reading wl from file " << input << endl;	
  while(1) {
    int rows, depth, cols, lhsbits, rhsbits;
    for(int bits = 2; bits <= 4; bits+=2){
    	for(int log_batch = 0; log_batch <= 9; log_batch++){
    		int batch = pow(2 ,log_batch);
  			std::ifstream infile(input);
  			//vector<bismo_rt::InstrumentationData> ret_vector;
  			int bismo_sum = 0, gemmlowp_sum = 0;
  			while(infile >> rows >> depth >> cols >> depth){
  				depth = depth * batch;
					bismo_rt::InstrumentationData ret = run_benchmark_matmul(rows, cols, depth, bits, bits);
					printInstrumentationData(ret);
					bismo_sum += ret["6_total_bismo_us"];
					gemmlowp_sum += ret["8_total_gemmlowp_us"];
				}
				//vector<bismo_rt::InstrumentationData>::iterator ptr;
				//for(ptr = ret_vector.begin(); ptr < ret_vector.end(); ptr++){
					//printInstrumentationData(*ptr);
				//}
				cout << bits << delimiter << batch << delimiter << bismo_sum << delimiter << gemmlowp_sum << endl;
			}
  	}
  }
}

void benchmark_gemm_batch() {
  bool headers_printed = false;
  while(1) {
    int rows, depth, cols, lhsbits, rhsbits;
    cin >> rows;
    if(rows == 0) {
      return;
    }
    cin >> depth >> cols;
    cin >> lhsbits >> rhsbits;
    bismo_rt::InstrumentationData ret = run_benchmark_matmul(rows, cols, depth, lhsbits, rhsbits);
    if(!headers_printed) {
      printInstrumentationHeaders(ret);
      headers_printed = true;
    }
    printInstrumentationData(ret);
  }
}
