`timescale 1ns/1ps

// =============================================================================
//  Program : alu.v
//  Author  : Jin-you Wu
//  Date    : Dec/19/2018
// -----------------------------------------------------------------------------
//  Description:
//  This is the Arithmetic Logic Unit of the Aquila core (A RISC-V core).
// -----------------------------------------------------------------------------
//  Revision information:
//
//  Nov/26/2019, by Chun-Jen Tsai:
//    Modify the arithmetic shift code and parameterize XLEN.
//
//  Aug/26/2025, by Jun-Kai Chen:
//    Add support for word-size instructions.
// -----------------------------------------------------------------------------
//  License information:
//
//  This software is released under the BSD-3-Clause Licence,
//  see https://opensource.org/licenses/BSD-3-Clause for details.
//  In the following license statements, "software" refers to the
//  "source code" of the complete hardware/software system.
//
//  Copyright 2019,
//                    Embedded Intelligent Systems Lab (EISL)
//                    Deparment of Computer Science
//                    National Chiao Tung Uniersity
//                    Hsinchu, Taiwan.
//
//  All rights reserved.
//
//  Redistribution and use in source and binary forms, with or without
//  modification, are permitted provided that the following conditions are met:
//
//  1. Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//
//  2. Redistributions in binary form must reproduce the above copyright notice,
//     this list of conditions and the following disclaimer in the documentation
//     and/or other materials provided with the distribution.
//
//  3. Neither the name of the copyright holder nor the names of its contributors
//     may be used to endorse or promote products derived from this software
//     without specific prior written permission.
//
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
//  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
//  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
//  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
//  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
//  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
//  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
//  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
//  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
//  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
//  POSSIBILITY OF SUCH DAMAGE.
// =============================================================================
`include "aquila_config.vh"

module alu #(
    parameter XLEN = 64
)(
    input  [XLEN-1 : 0] a_i,
    input  [XLEN-1 : 0] b_i,
    input  [2 : 0]      operation_sel_i,
    input               word_sel_i, // {0: dword}, {1: words}
    input               shift_sel_i,// {0: logic}, {1: arithmetic}
    output [XLEN-1 : 0] alu_result_o
);

// result wires declaration
wire [XLEN-1 : 0] result_add, result_sll, result_slt, result_sltu;
wire [XLEN-1 : 0] result_xor, result_sr,  result_or,  result_and;
wire [XLEN-1 : 0] result_srd, result_slld;
wire [31 : 0] result_sllw;
wire [31 : 0] result_srw;

// operation wires declaration
wire op_add, op_sll, op_slt, op_sltu, op_xor, op_sr, op_or, op_and;

// input and output for the alu
wire [XLEN-1 : 0] a, b, alu_result;


// input reassignment
assign a = (word_sel_i) ? { {32{a_i[31]}}, a_i[31: 0]} : a_i;
assign b = (word_sel_i) ? { {32{b_i[31]}}, b_i[31: 0]} : b_i;

// operation wires assignment
assign op_add  = (operation_sel_i == 3'b000);
assign op_sll  = (operation_sel_i == 3'b001);
assign op_slt  = (operation_sel_i == 3'b010);
assign op_sltu = (operation_sel_i == 3'b011);
assign op_xor  = (operation_sel_i == 3'b100);
assign op_sr   = (operation_sel_i == 3'b101);
assign op_or   = (operation_sel_i == 3'b110);
assign op_and  = (operation_sel_i == 3'b111);

assign result_add = a + b;                                    // add
assign result_slld = a << b[5:0];    // sll
assign result_sllw = a << b[4:0];
assign result_sll = word_sel_i ? { {32{result_sllw[31]}}, result_sllw[31: 0]}
                                 : result_slld;

assign result_slt = ($signed(a) < $signed(b)) ? 1 : 0;        // slt
assign result_sltu = (a < b) ? 1 : 0;                         // sltu
assign result_xor = a ^ b;                                    // xor
assign result_srd = shift_sel_i ? ($signed(a) >>> b_i[5:0])
                                : ($signed(a) >>  b_i[5:0]);  // sra, srl
assign result_srw = shift_sel_i ? ($signed(a[31: 0]) >>> b[4:0])
                                : ($signed(a[31: 0]) >>  b[4:0]);
assign result_sr = word_sel_i ? { {32{result_srw[31]}}, result_srw[31: 0] } 
                              : result_srd;
assign result_or = a | b;                                     // or
assign result_and = a & b;                                    // and

assign alu_result =
       (    {XLEN{op_add }} & result_add  )
       | (  {XLEN{op_sll }} & result_sll  )
       | (  {XLEN{op_slt }} & result_slt  )
       | (  {XLEN{op_sltu}} & result_sltu )
       | (  {XLEN{op_xor }} & result_xor  )
       | (  {XLEN{op_sr  }} & result_sr   )
       | (  {XLEN{op_or  }} & result_or   )
       | (  {XLEN{op_and }} & result_and  );

assign alu_result_o = 
    (word_sel_i) ? { {32{alu_result[31]}}, alu_result[31: 0] }
                 : { alu_result };

endmodule   // alu