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

// enable to print debug info
//#define DEBUG
// enable instrumentation for detailed measurements
#define BISMORT_INSTRUMENTATION
//#define BISMORT_INSTRUMENTATION_VERBOSE
// enable to compare hw-produced results against sw-produced ones
// this produces additional instrumentation data:
// cpu_gemmbitserial_lhs_p2s, cpu_gemmbitserial_rhs_p2s, cpu_gemmbitserial_exec
#define BISMORT_MATMUL_VERIFY_AGAINST_CPU
// enable to benchmark against a gemmlowp implementation with each matmul call
// this produces additional instrumentation data:
// cpu_gemmlowp_exec
//#define BISMORT_BENCHMARK_GEMMLOWP
// number of bytes for the p2s bit-parallel buffer on the accelerator side
#define BISMORT_P2S_BITPAR_BYTES  (1024*1024)
//#define BISMORT_USE_SW_P2S
