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

#include <ap_int.h>
#include <hls_stream.h>
#include "BISMOInstruction.hpp"
#include <iostream>

using namespace std;

void FetchInstrGen(
  hls::stream<ap_uint<BISMO_MMDESCR_BITS>> & in,
  hls::stream<ap_uint<BISMO_INSTR_BITS>> & out
);


void make_first(hls::stream<ap_uint<BISMO_INSTR_BITS>> & out) {
  out.write(ap_uint<BISMO_INSTR_BITS>("0", 16));
}

void make_second(hls::stream<ap_uint<BISMO_INSTR_BITS>> & out) {
  out.write(ap_uint<BISMO_INSTR_BITS>("08", 16));
}

bool TestFetchInstrGen() {
  cout << "Now running HLS Test for FetchInstrGen" << endl;
  hls::stream<ap_uint<BISMO_MMDESCR_BITS>> in;
  hls::stream<ap_uint<BISMO_INSTR_BITS>> out;
  hls::stream<ap_uint<BISMO_INSTR_BITS>> first;
  hls::stream<ap_uint<BISMO_INSTR_BITS>> second;
  SingleMMDescriptor desc;
  BISMOInstruction ins, golden_ins, first_ins, second_ins;
  desc.tiles_m = 2;
  desc.tiles_k = 4;
  desc.tiles_n = 2;
  desc.bits_l = 2;
  desc.bits_r = 2;
  desc.base_l = 0;
  desc.base_r = 0;
  desc.nbufs_fetch_exec_log2 = 2;
  desc.dram_lhs = 0;
  desc.dram_rhs = 1000;
  in.write(desc.asRaw());
  make_first(first);
  make_second(second);
  FetchInstrGen(in, out);

  int expected_size = (desc.tiles_m * desc.tiles_k * desc.tiles_n + desc.tiles_n) + (desc.tiles_m * desc.tiles_n + desc.tiles_n) * 2;

  bool all_OK = true;
  if(out.size() != expected_size) {
    cout << "ERROR: Incorrect number of fetch instructions produced!" << endl;
    cout << "Expected: " << expected_size << "Found: " << out.size() << endl;
    all_OK = false;
  }else{
    cout << "Correct number of fetch instructions produced: " << out.size() << endl;
  }

  first_ins = first.read();
  second_ins = second.read();
  int idx = 0;
  int m_loops;
  while(!out.empty()) {
    ins = out.read();

    //check for sync receive instr before fetch rhs
    if(idx % (desc.tiles_m * (desc.tiles_k + 2) + 3) == 0 ) {
      if(ins != first_ins) {
        cout << "Index: " << idx << endl;
        cout << "Modulo: " << idx % (desc.tiles_m * (desc.tiles_k + 2) + 3) << endl;
        cout << "ERROR: Mismatch found. Expected: " << first_ins << endl;
        cout << "Found: " << ins << endl;
        all_OK = false;
      } 
    }
    //check for sync send instr after fetch rhs
    if(idx % (desc.tiles_m * (desc.tiles_k + 2) + 3) == 2){
      if(ins != second_ins) {
        cout << "Index: " << idx << endl;
        cout << "Modulo: " << idx % (desc.tiles_m * (desc.tiles_k + 2) + 3) << endl;
        cout << "ERROR: Mismatch found. Expected: " << second_ins << endl;
        cout << "Found: " << ins << endl;
        all_OK = false;
      } 
    }
    // calculate index range for lhs fetch instructions
    m_loops = idx % (desc.tiles_m * (desc.tiles_k + 2) + 3);
    m_loops = m_loops - 3;
    //check for sync receive instr before fetch lhs
    if(m_loops % (desc.tiles_k + 2) == 0) {
      if(ins != first_ins) {
        cout << "Index: " << idx << endl;
        cout << "M loops: " << m_loops << endl;
        cout << "Modulo: " << m_loops % (desc.tiles_k + 2) << endl;
        cout << "ERROR: Mismatch found. Expected: " << second_ins << endl;
        cout << "Found: " << ins << endl;
        all_OK = false;      
      }
    }
    //check for sync send instr after fetch lhs
    if(m_loops % (desc.tiles_k + 2) == desc.tiles_k + 1) {
      if(ins != second_ins) {
        cout << "Index: " << idx << endl;
        cout << "M loops: " << m_loops << endl;
        cout << "Modulo: " << m_loops % (desc.tiles_k + 2) << endl;
        cout << "ERROR: Mismatch found. Expected: " << second_ins << endl;
        cout << "Found: " << ins << endl;
        all_OK = false;      
      }
    }
    cout << "Found: " << ins << endl;
    idx++;
  }
  return all_OK;
}

int main(int argc, char *argv[]) {
  if(TestFetchInstrGen()) {
    cout << "Test passed: FetchInstrGen" << endl;
    return 0;
  } else {
    cout << "Test failed: FetchInstrGen" << endl;
    return -1;
  }
}
