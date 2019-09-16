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
#include <ap_utils.h>
#include <hls_stream.h>
#include <stdint.h>
#include "BISMOInstruction.hpp"

template <
  // matmul array dimensions: rows, common, cols
  unsigned int M, unsigned int K, unsigned int N,
  // exec-to-fetch left-shift ratio: log2(K / fetch width)
  unsigned int ETF_S,
  // capacity of LHS and RHS memories (in elements)
  unsigned int LMEM, unsigned int RMEM
>
void FetchInstrGen_RHSLHSTiling_Templated(
  hls::stream<ap_uint<BISMO_MMDESCR_BITS>> & in,
  hls::stream<ap_uint<BISMO_INSTR_BITS>> & out
) {
  #pragma HLS INTERFACE ap_ctrl_none port=return
  #pragma HLS INTERFACE axis port=out
  #pragma HLS INTERFACE axis port=in

  BISMOFetchRunInstruction fetch;
  BISMOSyncInstruction sync_rec;
  BISMOSyncInstruction sync_send;

  // set the invariants (values do not depend on loop iter)
  sync_rec.targetStage = stgFetch;
  sync_rec.isRunCfg = 0;
  sync_rec.isSendToken = 0;
  sync_rec.chanID = 0;

  sync_send.targetStage = stgFetch;
  sync_send.isRunCfg = 0;
  sync_send.isSendToken = 1;
  sync_send.chanID = 0;

  fetch.targetStage = stgFetch;
  fetch.isRunCfg = 1;
  // read the descriptor
  SingleMMDescriptor ins_in;
  ins_in.fromRaw(in.read());
  ap_wait();

  // mems are divided into regions to provide fetch-exec concurrency
  uint8_t lmem_num_regions = (1 << ins_in.nbufs_fetch_exec_log2);
  uint16_t lmem_region_size = (LMEM >> ins_in.nbufs_fetch_exec_log2);
  const uint8_t lmem_num_regions_new = (ins_in.tiles_m + 1 ) / 2 + 1;
  const uint16_t lmem_region_size_new = (LMEM / lmem_num_regions_new);
  uint8_t lmem_region = 0;
  uint16_t lmem_region_offset = 0;

  const uint8_t rmem_num_regions = (1 << ins_in.nbufs_fetch_exec_log2);
  const uint16_t rmem_region_size = (RMEM >> ins_in.nbufs_fetch_exec_log2);
  uint8_t rmem_region = 0;
  uint16_t rmem_region_offset = 0;

  const int first_lhs_id = 0;
  const int first_rhs_id = M;
  const int bytes_per_rhs_tile = (N * K) / 8;
  const int bytes_per_lhs_tile = (M * K) / 8;

  const bool lhs_tiles_fit = lmem_region_size_new >= ins_in.tiles_k * ins_in.bits_l;

  if(lhs_tiles_fit){
    lmem_num_regions = lmem_num_regions_new;
    lmem_region_size = lmem_region_size_new;
    lmem_region_offset = lmem_region_size;
  }

  // compute the size of the iteration space
  const unsigned int total_iters = ins_in.tiles_m * ins_in.tiles_n;
  uint16_t n = 0, m = 0;

  for(uint16_t i = 0; i < total_iters; i++) {
    const bool m_is_even = (m + 2) % 2 == 0;
    if(m == 0) {
      // fill RHS buffer
      // each bit position is one block
      fetch.dram_block_count = ins_in.bits_r;
      // each block is a group of Dn rows' worth of bits
      // to save space and since the smallest unit of data handled by any part of the hardware is a octet (8 bytes)
      // this variable is right-shiftet by three to encode it as an octet
      fetch.dram_block_size_bytes = ins_in.tiles_k * bytes_per_rhs_tile >> 3;
      // block stride/skip is one bit position worth of bits
      fetch.dram_block_offset_bytes = ins_in.tiles_n * ins_in.tiles_k * bytes_per_rhs_tile;
      // IMPORTANT TODO: put in SW assertions around sizes of these, especially
      // dram_block_offset_bytes! other option is to generate one
      // fetch instruction per bit position...
      // DRAM base address for LHS
      fetch.dram_base = ins_in.dram_rhs + n * ins_in.tiles_k * bytes_per_rhs_tile;
      // note: no buffer regions for RHS tiles
      fetch.bram_addr_base = (ins_in.base_r + rmem_region_offset) << ETF_S;
      fetch.bram_id_start = first_rhs_id;
      // ID range of BRAM: 0 for LHS, 1 for RHS
      fetch.bram_id_range = 1;
      // how many DRAM data words are copied before the
      // fetch interconnect starts targeting the next BRAM
      fetch.tiles_per_row = ins_in.tiles_k << ETF_S;
      io_section_0:{
        #pragma HLS protocol fixed
        // receive token from execute stage representing RHS buf
        out.write(sync_rec.asRaw());
        ap_wait();
        // emit fetch instruction for RHS matrix
        out.write(fetch.asRaw());
        ap_wait();
        // signal that RHS buffer now filled
        // send token to execute stage
        out.write(sync_send.asRaw());
      }
      // use the next rmem region for following fetch
      rmem_region++;
      rmem_region_offset += rmem_region_size;
      if(rmem_region == rmem_num_regions) {
        rmem_region = 0;
        rmem_region_offset = 0;
      }
    }
    if(n == 0 || !m_is_even || !lhs_tiles_fit){
      //since index starts from 0 even tiles are beginn
      if(m_is_even || !lhs_tiles_fit){
        fetch.bram_addr_base = (ins_in.base_l + lmem_region_offset) << ETF_S;
      }
      else{
        fetch.bram_addr_base = (ins_in.base_l) << ETF_S;
      }
    // fill LHS buffer
    // each bit position is one block
    fetch.dram_block_count = ins_in.bits_l;
    // each block is a group of Dm rows' worth of bits
    // this variable is right-shiftet by three to encode it as an octet
    fetch.dram_block_size_bytes = ins_in.tiles_k * bytes_per_lhs_tile >> 3;
    // block stride/skip is one bit position worth of bits
    fetch.dram_block_offset_bytes = ins_in.tiles_m * ins_in.tiles_k * bytes_per_lhs_tile;
    // IMPORTANT TODO: put in SW assertions around sizes of these, especially
    // dram_block_offset_bytes! other option is to generate one
    // fetch instruction per bit position...
    // DRAM base address for LHS
    fetch.dram_base = ins_in.dram_lhs + m * ins_in.tiles_k * bytes_per_lhs_tile;
    //fetch.bram_addr_base = (ins_in.base_l + lmem_region_offset) << ETF_S;
    fetch.bram_id_start = first_lhs_id;
    // ID range of BRAM: 0 for LHS, 1 for RHS
    fetch.bram_id_range = 0;
    // how many DRAM data words are copied before the
    // fetch interconnect starts targeting the next BRAM
    fetch.tiles_per_row = ins_in.tiles_k << ETF_S;
    // emit fetch instruction for RHS matrix
    io_section_1:{
      #pragma HLS protocol fixed
      // receive token from execute stage representing LHS buf
      out.write(sync_rec.asRaw());
      ap_wait();
      out.write(fetch.asRaw());
      ap_wait();
      // signal that RHS buffer now filled
      // send token to execute stage
      out.write(sync_send.asRaw());
      }
    }else{
      io_section_2:{
      #pragma HLS protocol fixed
      // receive token from execute stage representing LHS buf
      out.write(sync_rec.asRaw());
      ap_wait();
      // signal that RHS buffer now filled
      // send token to execute stage
      out.write(sync_send.asRaw());
      }
    }
    // use the next lmem region for following fetch
    if(m_is_even){
      lmem_region++;
      lmem_region_offset += lmem_region_size;
    }
    if(lhs_tiles_fit){
      if(lmem_region == lmem_num_regions - 1) {
        lmem_region = 0;
        lmem_region_offset = lmem_region_size;
      }
    }else{
      if(lmem_region == lmem_num_regions) {
        lmem_region = 0;
        lmem_region_offset = 0;
      }
    }
    // iteration tracking logic: nested loops over tiles
    m++;
    if(m == ins_in.tiles_m) {
      m = 0;
      n++;
      if(n == ins_in.tiles_n) {
        n = 0;
      }
    }
  }
}


#include "FetchInstrGen_TemplateDefs.hpp"
void FetchInstrGen(
  hls::stream<ap_uint<BISMO_MMDESCR_BITS>> & in,
  hls::stream<ap_uint<BISMO_INSTR_BITS>> & out
) {
  #pragma HLS INTERFACE ap_ctrl_none port=return
  #pragma HLS INTERFACE axis port=out
  #pragma HLS INTERFACE axis port=in

  FetchInstrGen_RHSLHSTiling_Templated<
    TEMPLATE_PARAM_M, TEMPLATE_PARAM_K, TEMPLATE_PARAM_N,
    TEMPLATE_PARAM_ETF_S, TEMPLATE_PARAM_LMEM, TEMPLATE_PARAM_RMEM
  >(
    in, out
  );
}
