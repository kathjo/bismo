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

#include "bismo_rt_matmul.hpp"
#include <iostream>
#include "gemmbitserial/test/testhelpers.hpp"
#ifdef BISMORT_BENCHMARK_GEMMLOWP
#include "gemmlowp/public/gemmlowp.h"
#endif

namespace bismo_rt {
// note that the MatMul calls here simply implement wrappers around the
// MatrixMultiply class. this is to allow the BISMO RT to be compiled as a
// shared library, also removing the need for the template classes implemented
// as header files to be included with the rtlib

LayerHandle initMatMul(MatMulDescriptor & dsc) {
  bool is_coherent = platform->is_coherent();
  Matrix<uint8_t> * lhs = new Matrix<uint8_t>(
    dsc.M, dsc.K, dsc.wbits, dsc.wsigned, false, matTypeLHS, "mat_lhs", is_coherent
  );
  Matrix<uint8_t> * rhs = new Matrix<uint8_t>(
    dsc.K, dsc.N, dsc.ibits, dsc.isigned, true, matTypeRHS, "mat_rhs", is_coherent
  );
  Matrix<int32_t> * res = new Matrix<int32_t>(
    dsc.M, dsc.N, 32, true, true, matTypeRes, "mat_res", is_coherent
  );
#ifdef BISMORT_MATMUL_VERIFY_AGAINST_CPU
  bool allow_gemmbitserial = true;
#else
  bool allow_gemmbitserial = false;
#endif
  MatrixMultiply * mm = new MatrixMultiply(lhs, rhs, res, allow_gemmbitserial);//pass pointer to list of custom instr

  return (LayerHandle) mm;
}

void execMatMul(LayerHandle id) {
  MatrixMultiply * mm = (MatrixMultiply *) id;
  mm->exec();
  if(mm->has_cpu_ctx()) {
    gemmbitserial::GEMMContext ctx = mm->getCPUContext();
    TIMER_SAMPLE();
    gemmbitserial::gemmBitSerial(ctx);
    TIMER_SAMPLE();
    TIMER_REPORT("cpu_gemmbitserial_exec");
#ifdef BISMORT_MATMUL_VERIFY_AGAINST_CPU
    mm->m_res->accel2host();
    size_t nbytes_res = mm->M()*mm->N()*sizeof(int32_t);
    int verify_res = memcmp(ctx.res, mm->m_res->hostbuf(), nbytes_res);
    if(verify_res != 0) {
      std::cout << "CPU vs accel verification result = " << verify_res << std::endl;
    }
#endif
  }
#ifdef BISMORT_BENCHMARK_GEMMLOWP
  const gemmlowp::MatrixMap<const std::uint8_t, gemmlowp::MapOrder::RowMajor> lhs(
    mm->m_lhs->hostbuf(), mm->m_lhs->outer(), mm->m_lhs->inner()
  );
  const gemmlowp::MatrixMap<const std::uint8_t, gemmlowp::MapOrder::ColMajor> rhs(
    mm->m_rhs->hostbuf(), mm->m_rhs->inner(), mm->m_rhs->outer()
  );
  gemmlowp::MatrixMap<std::int32_t, gemmlowp::MapOrder::ColMajor> resmap(
    mm->m_res->hostbuf(), mm->m_lhs->outer(), mm->m_rhs->outer()
  );
  std::tuple<> output_pipeline;
  gemmlowp::GemmContext gemm_context;
  TIMER_SAMPLE();
  gemmlowp::GemmWithOutputPipeline<
    std::uint8_t, std::int32_t, gemmlowp::DefaultL8R8BitDepthParams
  >(
    &gemm_context, lhs, rhs, &resmap, 0, 0, output_pipeline
  );
  TIMER_SAMPLE();
  TIMER_REPORT("cpu_gemmlowp_exec");
#endif
}

void genInstructions(LayerHandle id) {
  MatrixMultiply * mm = (MatrixMultiply *) id;
  mm -> printInstrGen();
}

InstrumentationData getInstrumentationData(LayerHandle id) {
  MatrixMultiply * mm = (MatrixMultiply *) id;
  acc->updateStateBreakdown();
  mm->perfSummary();
  mm->perfDetails();
  mm->descrDetails();
  return instrumentationData;
}

uint8_t * getLayerLHSBuffer(LayerHandle id) {
  MatrixMultiply * mm = (MatrixMultiply *) id;
  return mm->m_lhs->hostbuf();
}

uint8_t * getLayerRHSBuffer(LayerHandle id) {
  MatrixMultiply * mm = (MatrixMultiply *) id;
  return mm->m_rhs->hostbuf();
}

int32_t * getLayerResBuffer(LayerHandle id) {
  MatrixMultiply * mm = (MatrixMultiply *) id;
  return mm->m_res->hostbuf();
}

void syncLayerLHSBuffer(LayerHandle id) {
  MatrixMultiply * mm = (MatrixMultiply *) id;
  mm->m_lhs->host2accel();
  if(mm->has_cpu_ctx()) {
    gemmbitserial::GEMMContext ctx = mm->getCPUContext();
    TIMER_SAMPLE();
    ctx.lhs.importRegular(mm->m_lhs->hostbuf());
    TIMER_SAMPLE();
    TIMER_REPORT("cpu_gemmbitserial_lhs_p2s");
  }
}

void syncLayerRHSBuffer(LayerHandle id) {
  MatrixMultiply * mm = (MatrixMultiply *) id;
  mm->m_rhs->host2accel();
  if(mm->has_cpu_ctx()) {
    gemmbitserial::GEMMContext ctx = mm->getCPUContext();
    TIMER_SAMPLE();
    ctx.rhs.importRegular(mm->m_rhs->hostbuf());
    TIMER_SAMPLE();
    TIMER_REPORT("cpu_gemmbitserial_rhs_p2s");
  }
}

void syncLayerResBuffer(LayerHandle id) {
  MatrixMultiply * mm = (MatrixMultiply *) id;
  mm->m_res->accel2host();
}

void deinitMatMul(LayerHandle id) {
  MatrixMultiply * mm = (MatrixMultiply *) id;
  delete mm->m_lhs;
  delete mm->m_rhs;
  delete mm->m_res;
  delete mm;
}

}
