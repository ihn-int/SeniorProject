`timescale 1ns / 1ps
// =============================================================================
//  Program : memory.v
//  Author  : Jin-you Wu
//  Date    : Dec/19/2018
// -----------------------------------------------------------------------------
//  Description:
//  This is the Memory Access Unit of the Aquila core (A RISC-V core).
// -----------------------------------------------------------------------------
//  Revision information:
//
//  Aug/17/2020, by Chun-Jen Tsai:
//    Rename the old module 'memory_alignment' to 'memory' and rename the
//    port signals.
//
//  Aug/26/2025, by Jun-Kai Chen:
//    Modify data rearrangement to fit 64-bit bus.
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

module memory #(
    parameter XLEN = 64
)(//  Processor clock and reset signals.
    input                   clk_i,
    input                   rst_i,

    // Pipeline stall signal.
    input                   stall_i,

    // Pipeline flush signal.
    input                   flush_i,
    
    // Passing signals from Execute to Writeback
    input  [ 2 : 0]         regfile_input_sel_i,
    output reg [ 2 : 0]     regfile_input_sel_o,

    // From Execute
    input  [XLEN-1 : 0]     mem_addr_i,
    input  [1 : 0]          dsize_sel_i, // (8, 16, 32, 64 bits)
    input  [XLEN-1 : 0]     unaligned_data_i, // from rs2
    input                   we_i,
    input                   re_i,

    input                   regfile_we_i,
    input  [ 4 : 0]         rd_addr_i,
    input                   signex_sel_i,
    input  [XLEN-1 : 0]     p_data_i,

    input                   csr_we_i,
    input  [11: 0]          csr_we_addr_i,
    input  [XLEN-1 : 0]     csr_we_data_i,

    // from D-memory
    input  [XLEN-1 : 0]     m_data_i,

    // To D-memory
    output reg [XLEN-1 : 0] data_o, // data to write
    output reg [7 : 0]      byte_sel_o, // use parameter XLEN instead of 8

    // Indicating memory mis-alignment exception
    output reg              mem_align_exception_o,

    // To Writeback stage
    output reg              regfile_we_o,
    output reg [ 4 : 0]     rd_addr_o,
    output reg              signex_sel_o,

    output reg [XLEN-1 : 0] aligned_data_o,
    output reg [XLEN-1 : 0] p_data_o,

    output reg              csr_we_o,
    output reg [11 : 0]     csr_we_addr_o,
    output reg [XLEN-1 : 0] csr_we_data_o,

    // PC of the current instruction
    input  [XLEN-1 : 0]     pc_i,
    output reg [XLEN-1 : 0] pc_o,

    // System jump operation
    input                   sys_jump_i,
    input  [1 : 0]          sys_jump_csr_addr_i,
    output                  sys_jump_o,
    output [1 : 0]          sys_jump_csr_addr_o,

    // Has instruction fetch being successiful?
    input                   fetch_valid_i,
    output reg              fetch_valid_o,

    // Execption info passed from Execute to Wrtieback
    input                   xcpt_valid_i,
    input  [3 : 0]          xcpt_cause_i,
    input  [XLEN-1 : 0]     xcpt_tval_i,
    output reg              xcpt_valid_o,
    output reg [3 : 0]      xcpt_cause_o,
    output reg [XLEN-1 : 0] xcpt_tval_o
);

assign sys_jump_o = sys_jump_i;
assign sys_jump_csr_addr_o = sys_jump_csr_addr_i;

always @(*)
begin
    if (mem_align_exception_o && (we_i || re_i))
    begin
        xcpt_valid_o = 1'b1;
        xcpt_cause_o = (we_i) ? 'd6 : 'd4;
        xcpt_tval_o  = mem_addr_i;
    end
    else
    begin
        xcpt_valid_o = xcpt_valid_i;
        xcpt_cause_o = xcpt_cause_i;
        xcpt_tval_o  = xcpt_tval_i;
    end
end

// store
// For legal store operation:
// +---+---+---+---+---+---+---+---+
// | 7 | 6 | 5 | 4 | 3 | 2 | 1 | 0 |
// +---+---+---+---+---+---+---+---+
// byte could be placed at any place
// half-word could be placed at 0xXX0
// word could be placed at 0xX00
// double word could be placed at 0x000
//
wire [2 : 0] mem_addr = mem_addr_i[2 : 0];
always @(*)
begin
    case (mem_addr) // 64 bits -> 8 bytes
        3'b000: // all size
        begin
            case(dsize_sel_i)
                2'b00:  // byte
                begin
                    data_o = {56'b0, unaligned_data_i[7 : 0]};
                    byte_sel_o = 8'b0000_0001;
                    mem_align_exception_o = 0;
                end
                2'b01:  // half-word
                begin
                    data_o = {48'b0, unaligned_data_i[15: 0]};
                    byte_sel_o = 8'b0000_0011;
                    mem_align_exception_o = 0;
                end
                2'b10:  // word
                begin
                    data_o = {32'b0, unaligned_data_i[31: 0]};
                    byte_sel_o = 8'b0000_1111;
                    mem_align_exception_o = 0;
                end
                2'b11:  // double word
                begin
                    data_o = unaligned_data_i;
                    byte_sel_o = 8'b1111_1111;
                    mem_align_exception_o = 0;
                end
                default:
                begin
                    data_o = 64'b0;
                    byte_sel_o = 0;
                    mem_align_exception_o = 1;
                end
            endcase
        end
        3'b001: // byte
        begin
            case(dsize_sel_i)
                2'b00:  // byte
                begin
                    data_o = {48'b0, unaligned_data_i[7 : 0], 8'b0};
                    byte_sel_o = 8'b0000_0010;
                    mem_align_exception_o = 0;
                end
                default:
                begin
                    data_o = 64'b0;
                    byte_sel_o = 0;
                    mem_align_exception_o = 1;
                end
            endcase
        end
        3'b010: // byte, half-word
        begin
            case(dsize_sel_i)
                2'b00:  // byte
                begin
                    data_o = {40'b0, unaligned_data_i[7 : 0], 16'b0};
                    byte_sel_o = 8'b0000_0100;
                    mem_align_exception_o = 0;
                end
                2'b01:  // half-word
                begin
                    data_o = {32'b0, unaligned_data_i[15: 0], 16'b0};
                    byte_sel_o = 8'b0000_1100;
                    mem_align_exception_o = 0;
                end
                default:
                begin
                    data_o = 64'b0;
                    byte_sel_o = 0;
                    mem_align_exception_o = 1;
                end
            endcase
        end
        3'b011: // byte
        begin
            case(dsize_sel_i)
                2'b00:  // byte
                begin
                    data_o = {32'b0, unaligned_data_i[7 : 0], 24'b0};
                    byte_sel_o = 8'b0000_1000;
                    mem_align_exception_o = 0;
                end
                default:
                begin
                    data_o = 64'b0;
                    byte_sel_o = 0;
                    mem_align_exception_o = 1;
                end
            endcase
        end
        3'b100: // byte, half-word, word
        begin
            case(dsize_sel_i)
                2'b00:  // byte
                begin
                    data_o = {24'b0, unaligned_data_i[7 : 0], 32'b0};
                    byte_sel_o = 8'b0001_0000;
                    mem_align_exception_o = 0;
                end
                2'b01:  // half-word
                begin
                    data_o = {16'b0, unaligned_data_i[15: 0], 32'b0};
                    byte_sel_o = 8'b0011_0000;
                    mem_align_exception_o = 0;
                end
                2'b10:  // word
                begin
                    data_o = {unaligned_data_i[31: 0], 32'b0};
                    byte_sel_o = 8'b1111_0000;
                    mem_align_exception_o = 0;
                end
                default:
                begin
                    data_o = 64'b0;
                    byte_sel_o = 0;
                    mem_align_exception_o = 1;
                end
            endcase
        end
        3'b101: // byte
        begin
            case (dsize_sel_i)
                2'b00:  // byte
                begin
                    data_o = {16'b0, unaligned_data_i[7 : 0], 40'b0};
                    byte_sel_o = 8'b0010_0000;
                    mem_align_exception_o = 0;
                end
                default:
                begin
                    data_o = 64'b0;
                    byte_sel_o = 0;
                    mem_align_exception_o = 1;
                end
            endcase
        end
        3'b110: // byte, half-word
        begin
            case (dsize_sel_i)
                2'b00:  // byte
                begin
                    data_o = {8'b0, unaligned_data_i[7 : 0], 48'b0};
                    byte_sel_o = 8'b0100_0000;
                    mem_align_exception_o = 0;
                end
                2'b01:  // half-word
                begin
                    data_o = {unaligned_data_i[15: 0], 48'b0};
                    byte_sel_o = 8'b1100_0000;
                    mem_align_exception_o = 0;
                end
                default:
                begin
                    data_o = 64'b0;
                    byte_sel_o = 0;
                    mem_align_exception_o = 1;
                end
            endcase
        end
        3'b111: // byte
        begin
            case (dsize_sel_i)
                2'b00:  // byte
                begin
                    data_o = {unaligned_data_i[7 : 0], 56'b0};
                    byte_sel_o = 8'b1000_0000;
                    mem_align_exception_o = 0;
                end
                default:
                begin
                    data_o = 64'b0;
                    byte_sel_o = 0;
                    mem_align_exception_o = 1;
                end
            endcase
        end
        default:
        begin
            data_o = 64'b0;
            byte_sel_o = 0;
            mem_align_exception_o = 1;
        end
    endcase
end

// ===============================================================================
//  Output registers to the Writeback stage
//
always @(posedge clk_i)
begin
    if (rst_i || (flush_i && !stall_i)) // stall has higher priority than flush.
    begin
        pc_o <= (flush_i)? pc_i : 0;
        fetch_valid_o <= 0;
        regfile_we_o <= 0;
        regfile_input_sel_o <= 5;
        rd_addr_o <= 0;
        signex_sel_o <= 0;

        aligned_data_o <= 0;
        p_data_o <= 0;

        csr_we_o <= 0;
        csr_we_addr_o <= 0;
        csr_we_data_o <= 0;
    end
    else if (stall_i)
    begin
        pc_o <= pc_o;
        fetch_valid_o <= fetch_valid_o;
        regfile_we_o <= regfile_we_o;
        regfile_input_sel_o <= regfile_input_sel_o;
        rd_addr_o <= rd_addr_o;
        signex_sel_o <= signex_sel_o;

        aligned_data_o <= aligned_data_o;
        p_data_o <= p_data_o;

        csr_we_o <= csr_we_o;
        csr_we_addr_o <= csr_we_addr_o;
        csr_we_data_o <= csr_we_data_o;
    end
    else if (xcpt_valid_o)
    begin
        pc_o <= pc_i;
        fetch_valid_o <= 1;
        regfile_we_o <= 0;
        regfile_input_sel_o <= 5;
        rd_addr_o <= 0;
        signex_sel_o <= 0;

        aligned_data_o <= 0;
        p_data_o <= 0;

        csr_we_o <= 0;
        csr_we_addr_o <= 0;
        csr_we_data_o <= 0;
    end
    else
    begin
        pc_o <= pc_i;
        fetch_valid_o <= fetch_valid_i;
        regfile_we_o <= regfile_we_i;
        regfile_input_sel_o <= regfile_input_sel_i;
        rd_addr_o <= rd_addr_i;
        signex_sel_o <= signex_sel_i;

        case (mem_addr)
            3'b000: aligned_data_o <= m_data_i;
            3'b001: aligned_data_o <= {m_data_i[7 : 0], m_data_i[63: 8]};
            3'b010: aligned_data_o <= {m_data_i[15: 0], m_data_i[63:16]};
            3'b011: aligned_data_o <= {m_data_i[23: 0], m_data_i[63:24]};
            3'b100: aligned_data_o <= {m_data_i[31: 0], m_data_i[63:32]};
            3'b101: aligned_data_o <= {m_data_i[39: 0], m_data_i[63:40]};
            3'b110: aligned_data_o <= {m_data_i[47: 0], m_data_i[63:48]};
            3'b111: aligned_data_o <= {m_data_i[55: 0], m_data_i[63:56]};
        endcase
        p_data_o <= p_data_i;

        csr_we_o <= csr_we_i;
        csr_we_addr_o <= csr_we_addr_i;
        csr_we_data_o <= csr_we_data_i;
    end
end

endmodule