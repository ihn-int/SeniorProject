`timescale 1ns / 1ps
// =============================================================================
//  Program : muldiv.v
//  Author  : Jin-you Wu
//  Date    : Jan/17/2019
// -----------------------------------------------------------------------------
//  Description:
//  This is the Multiplication-Division Unit of the Aquila core (A RISC-V core).
//  It implements the following RISC-V muldiv operations:
//      (0) mul
//      (1) mulh
//      (2) mulhsu
//      (3) mulhu
//      (4) div
//      (5) divu
//      (6) rem
//      (7) remu
// -----------------------------------------------------------------------------
//  Revision information:
//
//  Sep/8/2020, by Chun-Jen Tsai:
//    Added an option to infer a fast Xilinx 1- or 3-cycle 32-bit x 32-bit
//    multiplier.
//
//  Aug/26/2025, by Jun-Kai Chen:
//    Modify the Multiplication-Division Unit to 64-bit, and add support for
//    word-size instructions.
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

module muldiv #(
    parameter XLEN = 64
)(
    input                   clk_i,
    input                   rst_i,
    input                   stall_i,
    input  [XLEN-1 : 0]     a_i,
    input  [XLEN-1 : 0]     b_i,
    input                   req_i,
    input  [2 : 0]          operation_sel_i,
    input                   word_sel_i,
    output reg [XLEN-1 : 0] muldiv_result_o,
    output reg              ready_o
);

function [XLEN-1 : 0] abs;
    input [XLEN-1 : 0] num;

    abs = num[XLEN-1] ? -num : num;
endfunction

//==========================================================
// Operation signals
//
wire op_mul, op_mulh, op_mulhsu, op_mulhu;
wire op_div, op_divu, op_rem, op_remu;

assign op_mul    = (operation_sel_i == 3'b000);
assign op_mulh   = (operation_sel_i == 3'b001);
assign op_mulhsu = (operation_sel_i == 3'b010);
assign op_mulhu  = (operation_sel_i == 3'b011);
assign op_div    = (operation_sel_i == 3'b100);
assign op_divu   = (operation_sel_i == 3'b101);
assign op_rem    = (operation_sel_i == 3'b110);
assign op_remu   = (operation_sel_i == 3'b111);

wire is_divider, is_a_zero, is_b_zero, is_a_neg, is_b_neg,
     signed_adjust, mul_overflow, is_usext;

assign is_divider = operation_sel_i[2];
assign is_a_zero  = (a_i == 64'b0);
assign is_b_zero  = (b_i == 64'b0);
assign is_a_neg   = a_i[63] & (op_mulh | op_mulhsu | op_div | op_rem);
assign is_b_neg   = b_i[63] & (op_mulh | op_div | op_rem);
assign signed_adjust = (is_a_neg ^ is_b_neg);
assign mul_overflow  = (is_a_neg & (a_i[62: 0] == 63'b0));

// Only 32-bit unsigned change to unsigned extension
assign is_usext = word_sel_i & (op_divu | op_remu);

wire [XLEN-1 : 0] op_a, op_b, op_a_tmp, op_b_tmp;

assign op_a_tmp = (is_a_neg) ? abs(a_i) : a_i;
assign op_b_tmp = (is_b_neg) ? abs(b_i) : b_i;
assign op_a = (is_usext) ? { 32'b0, op_a_tmp[31:0] } : op_a_tmp;
assign op_b = (is_usext) ? { 32'b0, op_b_tmp[31:0] } : op_b_tmp;

wire is_calc_done;
reg  [XLEN-1 : 0] reg64; // For dword
reg  [XLEN*2 : 0] result; // For dword
reg  [6 : 0]      cnt;

`ifdef ENABLE_FAST_MULTIPLY
// .W and .D share the same fast multiplier
reg  [XLEN-1 : 0]   op_a_r, op_b_r;
reg  [XLEN*2-1 : 0] fast_result, mul0, mul1;
reg  [1 : 0]        fast_mul_counter;

assign is_calc_done = (op_mul | op_mulh | op_mulhsu | op_mulhu) ?
                        (&fast_mul_counter) : (~|cnt);
`else

// For the slow shift-add multiplier
assign is_calc_done = (~|cnt);
`endif 

wire [XLEN-1 : 0] mull, mulh, quotient, remainder;

assign mull = result[63: 0];
assign mulh = result[127:64];
assign quotient  = result[63: 0];
assign remainder = result[128:65];

//==========================================================
// Finite State Machine for shift-add registers
//
localparam S_IDLE        = 3'b000,
           S_CALC        = 3'b001,
           S_SIGN_ADJUST = 3'b010,
           S_DONE        = 3'b011,
           S_STALL       = 3'b100;
reg  [2 : 0] S, S_nxt;

// state FF logic
always @(posedge clk_i)
begin
    if (rst_i) S <= S_IDLE;
    else S <= S_nxt;
end

// state transition logic
always @(*)
begin
    case (S)
        S_IDLE:
            // correspond zero flag and word selection
            S_nxt = (req_i) ? 
                (is_a_zero | is_b_zero) ?
                S_DONE : S_CALC : S_IDLE;
        S_CALC:
            S_nxt = (is_calc_done) ? S_SIGN_ADJUST : S_CALC;
        S_SIGN_ADJUST:
            S_nxt = S_DONE;
        S_DONE:
            S_nxt = (stall_i) ? S_STALL : S_IDLE;
        S_STALL:
            S_nxt = (stall_i) ? S_STALL : S_IDLE;
        default:
            S_nxt = S_IDLE;
    endcase
end

//==========================================================
// Computation
//
// For dword
wire mul_add = result[0];
wire div_sub = (result[127:64] >= reg64);

wire [64: 0] adder_opa, adder_opb;
assign adder_opa = is_divider ? -reg64 : reg64;
assign adder_opb = result[127:64];

wire [64: 0] adder_tmp = adder_opa + adder_opb;
wire [128: 0] result_tmp = {adder_tmp, result[63: 0]};


`ifdef  ENABLE_FAST_MULTIPLY
// Here, we use Xilinx synthesizer coding style to infer a
// 3-cycel multiplier. On KC-705@100MHz, you can even infer
// a single-cycle 32-bit x 32-bit multipler.
//
always @(posedge clk_i)
begin
    if (rst_i | ((S == S_IDLE) & req_i))
    begin
        fast_result <= 64'b0;
        fast_mul_counter <= 2'b00;
    end
    else
    begin
        op_a_r <= op_a;
        op_b_r <= op_b;
        mul0 <= op_a_r * op_b_r;
        mul1 <= mul0;
        fast_result <= mul1;
        fast_mul_counter <= (& fast_mul_counter) ? fast_mul_counter : fast_mul_counter + 1;
    end
end
`endif 

always @(posedge clk_i)
begin
    if ( (S == S_IDLE) & req_i)
    begin
        if (is_divider)
        begin
            if (is_b_zero)
            begin
                result[128:65] <= a_i;
                result[64] <= 1'b0;
                result[63: 0] <= {64{1'b1}};
            end
            else
            begin
                cnt <= 'd64;
                reg64 <= op_b;
                result <= {1'b0, 64'b0, op_a};
            end
        end
        else
        begin
            if (is_a_zero | is_b_zero)
            begin
                result <= 129'b0;
            end
            else
            begin
                cnt <= 'd63;
                reg64 <= op_a;
                result <= {1'b0, 64'b0, op_b};
            end
        end
    end
    else if (S == S_CALC)
    begin
        cnt <= cnt - 'd1;
        if (is_divider)
        begin
            result <= (div_sub) ? {result_tmp[127: 0], 1'b1} :
                                  {result[127: 0], 1'b0};
        end
        else
        begin
            result <= (mul_add) ? {1'b0, result_tmp[128: 1]} :
                                  {1'b0, result[128: 1]};
        end
    end
    else if (S == S_SIGN_ADJUST)
    begin
        if (is_divider)
        begin
            result[128:65] <= (is_a_neg) ? -remainder : remainder;
            result[63: 0]  <= (signed_adjust) ? -quotient : quotient;
        end
        else
        begin
`ifdef ENABLE_FAST_MULTIPLY
            result <= (op_mulh | op_mulhu)? (signed_adjust)? -fast_result : fast_result
                    : (op_mulhsu)? (is_a_neg)? -fast_result : fast_result : fast_result;
`else
            result <= (op_mulh | op_mulhu)? (signed_adjust)? -result : result
                    : (op_mulhsu)? (is_a_neg)? -result : result : result;
`endif
        end
    end
end

//==========================================================
// Output signals
//
wire [XLEN-1 : 0] muldiv_result;
assign muldiv_result = 
     ( { {64{op_mul}} & mull } )
    |( { {64{op_mulh | op_mulhsu | op_mulhu}} & mulh } )
    |( { {64{op_div | op_divu}} & quotient } )
    |( { {64{op_rem | op_remu}} & remainder } );

always @(posedge clk_i)
begin
    if (S == S_DONE || (S == S_STALL && S_nxt != S_IDLE))
        ready_o <= 1;
    else
        ready_o <= 0;
end

always @(posedge clk_i)
begin
    if (S == S_DONE)
        muldiv_result_o <= muldiv_result;
    else
        muldiv_result_o <= muldiv_result_o;
end

endmodule