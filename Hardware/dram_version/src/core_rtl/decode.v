`timescale 1ns / 1ps
// =============================================================================
//  Program : decode.v
//  Author  : Jin-you Wu
//  Date    : Dec/19/2018
// -----------------------------------------------------------------------------
//  Description:
//  This is the Decoding Unit of the Aquila core (A RISC-V core).
// -----------------------------------------------------------------------------
//  Revision information:
//
//  Nov/29/2019, by Chun-Jen Tsai:
//    Merges the pipeline register moduel 'decode_execute' into the 'decode'
//    module.
//
//  Feb/10/2022, by Che-Yu Wu:
//    Add load-hazard detection for amo instructions.
// 
//  Aug/26/2025, by Jun-Kai Chen:
//    Add support for RV64 instructions.
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

module decode #(
    parameter XLEN = 64
)(
    // System signals
    input                   clk_i,
    input                   rst_i,

    // Pipeline stall signal
    input                   stall_i,

    // Pipeline flush signal
    input                   flush_i,

    // From Fetch
    input  [XLEN-1 : 0]     pc_i,
    input  [31: 0]          instruction_i,
    input                   branch_hit_i,
    input                   branch_decision_i,

    // From CSR
    input  [XLEN-1 : 0]     csr_data_i,
    input  [1 : 0]          privilege_lvl_i,

    // Instruction operands from the Register File. To be forwarded.
    input  [XLEN-1 : 0]     rs1_data_i,
    input  [XLEN-1 : 0]     rs2_data_i,

    // to Pipeline Control
    output                  is_load_hazard_o,

    // Operand register IDs to the RFU
    output [4 : 0]          rs1_addr_o,
    output [4 : 0]          rs2_addr_o,

    // illegal
    output                  unsupported_instr_o,

    // to Execute
    output reg [XLEN-1 : 0] imm_o,
    output reg              csr_we_o,
    output reg [1 : 0]      inputA_sel_o,
    output reg [1 : 0]      inputB_sel_o,
    output reg [2 : 0]      operation_sel_o,
    output reg              word_sel_o, // new signal in RV64
    output reg              alu_muldiv_sel_o,
    output reg              shift_sel_o,
    output reg              branch_hit_o,
    output reg              branch_decision_o,
    output reg              is_jalr_o,
    output reg              is_fencei_o,

    // to Execute and BPU
    output reg [XLEN-1 : 0] pc_o, // also to CSR
    output reg              is_branch_o,
    output reg              is_jal_o,

    // to CSR
    output     [11: 0]      csr_addr_o, // Addr and Imm are defined in instruction, not XLEN
    output reg [4 : 0]      csr_imm_o,

    // to Execute
    output reg [4 : 0]      rd_addr_o,
    output reg              regfile_we_o,
    output reg [2 : 0]      regfile_input_sel_o,
    output reg              we_o,
    output reg              re_o,
    output reg [1 : 0]      dsize_sel_o,  // data size select
    output reg              signex_sel_o, // sign-extension select
    output reg              is_amo_o,
    output reg              amo_word_sel_o, // new signal in RV64
    output reg [4 : 0]      amo_type_o,

    // to Forwarding_Unit
    output reg [4 : 0]      rs1_addr2fwd_o,
    output reg [4 : 0]      rs2_addr2fwd_o,
    output reg [XLEN-1 : 0] rs1_data2fwd_o,
    output reg [XLEN-1 : 0] rs2_data2fwd_o,
    output reg [11: 0]      csr_addr2fwd_o,
    output reg [XLEN-1 : 0] csr_data2fwd_o,

    // System Jump operation
    output reg              sys_jump_o,
    output reg [1 : 0]      sys_jump_csr_addr_o,

    // Has instruction fetch being successiful?
    input                   fetch_valid_i,
    output reg              fetch_valid_o,

    // Exception info passed from Fetch to Execute.
    input                   xcpt_valid_i,
    input  [3 : 0]          xcpt_cause_i,
    input  [XLEN-1 : 0]     xcpt_tval_i,
    output reg              xcpt_valid_o,
    output reg [3 : 0]      xcpt_cause_o,
    output reg [XLEN-1 : 0] xcpt_tval_o
);

// Interal signals of the Decode Stage.
wire [XLEN-1 : 0] imm;
wire [4 : 0]      rd_addr;
wire              we;
wire              re;
wire              regfile_we;
reg  [1 : 0]      inputA_sel;
reg  [1 : 0]      inputB_sel;
reg  [2 : 0]      regfile_input_sel;
wire [2 : 0]      operation_sel;
wire              word_sel;
wire [1 : 0]      dsize_sel;      // 2'b11 for dword
wire              signex_sel;     // for lb, lbu, lh and lhu.
wire              alu_muldiv_sel; // for rv64m operation.
wire              shift_sel;      // for shift right operation.
wire [4 : 0]      csr_imm;

// Instruction field declaration
wire [31: 0] rv64_instr = instruction_i;
wire [6 : 0] opcode = rv64_instr[6 : 0];
wire [5 : 0] rv64_shamt = rv64_instr[25:20];
wire [2 : 0] rv64_funct3 = rv64_instr[14:12];
wire [6 : 0] rv64_funct7 = rv64_instr[31:25];
wire [4 : 0] amo_type = rv64_instr[31:27];

// Immediate value field, with
wire [XLEN-1 : 0] immI, immS, immB, immU, immJ;
assign immI = { {53{rv64_instr[31]}}, rv64_instr[30:20] };
assign immS = { {53{rv64_instr[31]}}, rv64_instr[30:25],
                    rv64_instr[11: 7] };
assign immB = { {52{rv64_instr[31]}}, rv64_instr[7],
                    rv64_instr[30:25], rv64_instr[11: 8], 1'b0 };
assign immU = { {33{rv64_instr[31]}}, rv64_instr[30:12], 12'h000 };
assign immJ = { {44{rv64_instr[31]}}, rv64_instr[19:12],
                    rv64_instr[20], rv64_instr[30:21] , 1'b0};

// ================================================================================
//  We generate the signals and reused them as much as possible to save gate counts
//
// wire opcode_1_0_00 = (opcode[1:0] == 2'b00);  // rvc
// wire opcode_1_0_01 = (opcode[1:0] == 2'b01);  // rvc
// wire opcode_1_0_10 = (opcode[1:0] == 2'b10);  // rvc
wire opcode_1_0_11 = (opcode[1: 0] == 2'b11);    // rv64

wire opcode_4_2_000 = (opcode[4: 2] == 3'b000);
wire opcode_4_2_001 = (opcode[4: 2] == 3'b001);
wire opcode_4_2_010 = (opcode[4: 2] == 3'b010);
wire opcode_4_2_011 = (opcode[4: 2] == 3'b011);
wire opcode_4_2_100 = (opcode[4: 2] == 3'b100);
wire opcode_4_2_101 = (opcode[4: 2] == 3'b101);
wire opcode_4_2_110 = (opcode[4: 2] == 3'b110);
wire opcode_4_2_111 = (opcode[4: 2] == 3'b111);

wire opcode_6_5_00 = (opcode[6: 5] == 2'b00);
wire opcode_6_5_01 = (opcode[6: 5] == 2'b01);
wire opcode_6_5_10 = (opcode[6: 5] == 2'b10);
wire opcode_6_5_11 = (opcode[6: 5] == 2'b11);

wire rv64_funct3_000 = (rv64_funct3 == 3'b000);
wire rv64_funct3_001 = (rv64_funct3 == 3'b001);
wire rv64_funct3_010 = (rv64_funct3 == 3'b010);
wire rv64_funct3_011 = (rv64_funct3 == 3'b011);
wire rv64_funct3_100 = (rv64_funct3 == 3'b100);
wire rv64_funct3_101 = (rv64_funct3 == 3'b101);
wire rv64_funct3_110 = (rv64_funct3 == 3'b110);
wire rv64_funct3_111 = (rv64_funct3 == 3'b111);

wire rv64_funct7_0000000 = (rv64_funct7 == 7'b0000000);
wire rv64_funct7_0100000 = (rv64_funct7 == 7'b0100000);
wire rv64_funct7_0000001 = (rv64_funct7 == 7'b0000001);
wire rv64_funct7_0000101 = (rv64_funct7 == 7'b0000101);
wire rv64_funct7_0001001 = (rv64_funct7 == 7'b0001001);
wire rv64_funct7_0001101 = (rv64_funct7 == 7'b0001101);
wire rv64_funct7_0010101 = (rv64_funct7 == 7'b0010101);
wire rv64_funct7_0100001 = (rv64_funct7 == 7'b0100001);
wire rv64_funct7_0010001 = (rv64_funct7 == 7'b0010001);
wire rv64_funct7_0101101 = (rv64_funct7 == 7'b0101101);
wire rv64_funct7_1111111 = (rv64_funct7 == 7'b1111111);
wire rv64_funct7_0000100 = (rv64_funct7 == 7'b0000100);
wire rv64_funct7_0001000 = (rv64_funct7 == 7'b0001000);
wire rv64_funct7_0001100 = (rv64_funct7 == 7'b0001100);
wire rv64_funct7_0101100 = (rv64_funct7 == 7'b0101100);
wire rv64_funct7_0010000 = (rv64_funct7 == 7'b0010000);
wire rv64_funct7_0010100 = (rv64_funct7 == 7'b0010100);
wire rv64_funct7_1100000 = (rv64_funct7 == 7'b1100000);
wire rv64_funct7_1110000 = (rv64_funct7 == 7'b1110000);
wire rv64_funct7_1010000 = (rv64_funct7 == 7'b1010000);
wire rv64_funct7_1101000 = (rv64_funct7 == 7'b1101000);
wire rv64_funct7_1111000 = (rv64_funct7 == 7'b1111000);
wire rv64_funct7_1010001 = (rv64_funct7 == 7'b1010001);
wire rv64_funct7_1110001 = (rv64_funct7 == 7'b1110001);
wire rv64_funct7_1100001 = (rv64_funct7 == 7'b1100001);
wire rv64_funct7_1101001 = (rv64_funct7 == 7'b1101001);

// RV64I Opcode Classification, with word instruction
// I base instructions need extra signal, op and op_imm for ALU.
// A extension instructions share the same opcode.
wire rv32_op        = opcode_6_5_01 & opcode_4_2_110;
wire rv32_op_imm    = opcode_6_5_00 & opcode_4_2_110;
wire rv64_op        = opcode_6_5_01 & opcode_4_2_100;   // OP opcode
wire rv64_op_imm    = opcode_6_5_00 & opcode_4_2_100;   // OP-IMM opcode
wire rv64_jal       = opcode_6_5_11 & opcode_4_2_011;   // JAL opcode
wire rv64_jalr      = opcode_6_5_11 & opcode_4_2_001;   // JALR opcode
wire rv64_load      = opcode_6_5_00 & opcode_4_2_000;   // LOAD opcode
wire rv64_store     = opcode_6_5_01 & opcode_4_2_000;   // STORE opcode
wire rv64_branch    = opcode_6_5_11 & opcode_4_2_000;   // BRANCH opcode
wire rv64_lui       = opcode_6_5_01 & opcode_4_2_101;   // LUI opcode
wire rv64_auipc     = opcode_6_5_00 & opcode_4_2_101;   // AUIPC opcode
wire rv64_miscmem   = opcode_6_5_00 & opcode_4_2_011;   // MISC-MEM opcode
wire rv64_system    = opcode_6_5_11 & opcode_4_2_100;   // SYSTEM opcode
wire rv64_amo       = opcode_6_5_01 & opcode_4_2_011;   // AMO opcode

// M extension instructions
// M extension instructions need extra signal, with no imm.
// I base word instructions and M ext. word instruction share the same
// opcode, OP-32.
wire rv32m = rv32_op & rv64_funct7_0000001;
wire rv64m = rv64_op & rv64_funct7_0000001;

// Immediate value selection
wire rv64_imm_seli = rv64_op_imm | rv64_jalr | rv64_load | rv32_op_imm;
wire rv64_imm_sels = rv64_store;
wire rv64_imm_selb = rv64_branch;
wire rv64_imm_selu = rv64_lui | rv64_auipc;
wire rv64_imm_selj = rv64_jal;

// SUB opcode
wire rv_sub   = rv64_funct3_000 & rv64_funct7_0100000;
wire rv32_sub = rv32_op & rv_sub;
wire rv64_sub = rv64_op & rv_sub;

//==========================================================
// Conditional branch instructions
wire rv64_beq  = rv64_branch & rv64_funct3_000;
wire rv64_bne  = rv64_branch & rv64_funct3_001;
wire rv64_blt  = rv64_branch & rv64_funct3_100;
wire rv64_bgt  = rv64_branch & rv64_funct3_101;
wire rv64_bltu = rv64_branch & rv64_funct3_110;
wire rv64_bgtu = rv64_branch & rv64_funct3_111;

//==========================================================
// MISC-MEM
wire rv64_fence  = rv64_miscmem & rv64_funct3_000;
wire rv64_fencei = rv64_miscmem & rv64_funct3_001;

//==========================================================
// System Instructions
wire rv64_csrrw  = rv64_system & rv64_funct3_001;
wire rv64_csrrs  = rv64_system & rv64_funct3_010;
wire rv64_csrrc  = rv64_system & rv64_funct3_011;
wire rv64_csrrwi = rv64_system & rv64_funct3_101;
wire rv64_csrrsi = rv64_system & rv64_funct3_110;
wire rv64_csrrci = rv64_system & rv64_funct3_111;
wire rv64_csr    = rv64_system & (~rv64_funct3_000);

wire rv64_sys_op = rv64_system & rv64_funct3_000;
wire rv64_ecall  = rv64_sys_op & (rv64_instr[31:20] == 12'h000);
wire rv64_ebreak = rv64_sys_op & (rv64_instr[31:20] == 12'h001);
wire rv64_mret   = rv64_sys_op & (rv64_instr[31:20] == 12'h302);
wire rv64_sret   = rv64_sys_op & (rv64_instr[31:20] == 12'h102);

//==========================================================
// Load / Store Instructions
wire rv64_lb    = rv64_load & rv64_funct3_000;
wire rv64_lh    = rv64_load & rv64_funct3_001;
wire rv64_lw    = rv64_load & rv64_funct3_010;
wire rv64_ld    = rv64_load & rv64_funct3_011;
wire rv64_lbu   = rv64_load & rv64_funct3_100;
wire rv64_lhu   = rv64_load & rv64_funct3_101;
wire rv64_lwu   = rv64_load & rv64_funct3_110;

wire rv64_sb    = rv64_store & rv64_funct3_000;
wire rv64_sh    = rv64_store & rv64_funct3_001;
wire rv64_sw    = rv64_store & rv64_funct3_010;
wire rv64_sd    = rv64_store & rv64_funct3_011;

//==========================================================
// Exception Signals
wire xcpt_valid = rv64_ecall;
wire [3 : 0] xcpt_cause = (privilege_lvl_i == 2'b11) ? 4'd11 :
                          (privilege_lvl_i == 2'b01) ? 4'd9  : 4'd8;
wire [3 : 0] xcpt_tval  = 0;

//==========================================================
// Output Signals
assign imm = 
    ( {64{rv64_imm_seli}} & immI)
    |({64{rv64_imm_sels}} & immS)
    |({64{rv64_imm_selb}} & immB)
    |({64{rv64_imm_selu}} & immU)
    |({64{rv64_imm_selj}} & immJ);

// All the RV64IMA need rd except the
// # BRANCH, STORE, FENCE, FENCE.I, ECALL, EBREAK
assign regfile_we = rv64_lui | rv64_auipc | rv64_load | rv64_op_imm | rv64_op  | rv64_csr   |
                    rv64_amo | rv64_jal   | rv64_jalr | rv32_op     | rv32_op_imm;
assign re = rv64_load  | rv64_amo;
assign we = rv64_store;


assign rd_addr = rv64_instr[11: 7];
assign rs1_addr_o = rv64_instr[19:15];
assign rs2_addr_o = rv64_instr[24:20];

// Only R and I types needs word select signal
assign word_sel   = (rv32_op | rv32_op_imm);
assign dsize_sel  = rv64_funct3[1 : 0];
assign signex_sel = rv64_funct3[2]; // {0: unsigned}
                                    // {1: signed}
assign alu_muldiv_sel = rv64m | rv32m;
assign amo_word_sel   = ~rv64_funct3[0];
assign operation_sel  = (rv64_lui | rv64_auipc) ? 3'b000 : rv64_funct3;
assign shift_sel = rv64_funct7_0100000;

assign csr_addr_o = rv64_instr[31:20];
assign csr_imm    = rv64_instr[19:15];

// Detect load structure hazard.
wire is_r_type   = (rv64_op | rv64_system | rv32_op) & opcode_1_0_11;
wire is_i_type   = (rv64_load | rv64_op_imm | rv64_jalr | rv32_op_imm) & opcode_1_0_11;
wire is_s_type   = rv64_store & opcode_1_0_11;
wire is_b_type   = rv64_branch & opcode_1_0_11;
wire is_fence    = rv64_fence & opcode_1_0_11;
wire is_csr_type = (rv64_csrrw | rv64_csrrs | rv64_csrrc) & opcode_1_0_11;
wire is_amo_type = rv64_amo & opcode_1_0_11;


// Forwarding
wire is_rs1_rd_same = (rs1_addr_o == rd_addr_o) & 
                      (is_r_type | is_s_type | is_b_type | is_i_type|
                       is_fence  | is_csr_type | is_amo_type);
wire is_rs2_rd_same = (rs2_addr_o == rd_addr_o) &
                      (is_r_type | is_s_type | is_b_type);
assign is_load_hazard_o = (is_rs1_rd_same | is_rs2_rd_same) & re_o;

// Input A selection
always @(*)
begin
    if (rv64_auipc | rv64_jal | rv64_branch)
        inputA_sel = 1; // pc
    else if (rv64_lui | rv64_store)
        inputA_sel = 0; // 0
    else
        inputA_sel = 2; // rs1
end

// Input B selection
always @(*)
begin
    if (rv32_sub | rv64_sub)
        inputB_sel = 2; // -rs2
    else if (rv64_store | rv64_op | rv64_amo | rv32_op)
        inputB_sel = 1; // rs2
    else
        inputB_sel = 0; // immX
end

// Register File Input
always @(*)
begin
    if (rv64_lb | rv64_lbu)
        regfile_input_sel = 0;  // load byte
    else if (rv64_lh | rv64_lhu)
        regfile_input_sel = 1;  // load half word
    else if (rv64_lw | rv64_lwu)
        regfile_input_sel = 2;  // load word
    else if (rv64_ld | rv64_amo)
        regfile_input_sel = 3;  // load double word
    else if (rv64_jal | rv64_jalr)
        regfile_input_sel = 4;  // pc+4
    else if (rv64_csr)
        regfile_input_sel = 6;  // csr
    else
        regfile_input_sel = 5;  // execute result
end

// The instruction that are not supported currently
assign unsupported_instr_o = rv64_fence | rv64_ebreak;

// Pipeline output to other stage.
always @(posedge clk_i)
begin
    if (rst_i || (flush_i & !stall_i))
    begin
        pc_o <= (flush_i)? pc_i : 0;
        fetch_valid_o <= 0;
        rs1_data2fwd_o <= 0;
        rs2_data2fwd_o <= 0;
        imm_o <= 0;
        inputA_sel_o <= 2;
        inputB_sel_o <= 0;
        operation_sel_o <= 0;
        signex_sel_o <= 0;
        dsize_sel_o <= 0;
        alu_muldiv_sel_o <= 0;
        shift_sel_o <= 0;
        is_branch_o <= 0;
        is_jal_o <= 0;
        is_jalr_o <= 0;
        regfile_we_o <= 1;
        regfile_input_sel_o <= 5; // send Execute result into the RFU.
        we_o <= 0;
        re_o <= 0;
        word_sel_o <= 0;
        dsize_sel_o <= 0;
        rd_addr_o <= 0;
        rs1_addr2fwd_o <= 0;
        rs2_addr2fwd_o <= 0;
        csr_we_o <= 0;
        csr_imm_o <= 0;
        branch_hit_o <= 0;
        branch_decision_o <= 0;
        is_fencei_o <= 0;
        amo_word_sel_o <= 0;
        amo_type_o <= 0;
        is_amo_o <= 0;

        sys_jump_o <= 0;
        sys_jump_csr_addr_o <= 0;
        xcpt_valid_o <= 0;
        xcpt_cause_o <= 0;
        xcpt_tval_o <= 0;
        csr_data2fwd_o <= 0;
        csr_addr2fwd_o <= 0;
    end
    else if (stall_i)
    begin
        pc_o <= pc_o;
        fetch_valid_o <= fetch_valid_o;
        rs1_data2fwd_o <= rs1_data2fwd_o;
        rs2_data2fwd_o <= rs2_data2fwd_o;
        imm_o <= imm_o;
        inputA_sel_o <= inputA_sel_o;
        inputB_sel_o <= inputB_sel_o;
        operation_sel_o <= operation_sel_o;
        signex_sel_o <= signex_sel_o;
        alu_muldiv_sel_o <= alu_muldiv_sel_o;
        shift_sel_o <= shift_sel_o;
        is_branch_o <= is_branch_o;
        is_jal_o <= is_jal_o;
        is_jalr_o <= is_jalr_o;
        regfile_we_o <= regfile_we_o;
        regfile_input_sel_o <= regfile_input_sel_o;
        we_o <= we_o;
        re_o <= re_o;
        word_sel_o <= word_sel_o;
        dsize_sel_o <= dsize_sel_o;
        rd_addr_o <= rd_addr_o;
        rs1_addr2fwd_o <= rs1_addr2fwd_o;
        rs2_addr2fwd_o <= rs2_addr2fwd_o;
        csr_we_o <= csr_we_o;
        csr_imm_o <= csr_imm_o;
        branch_hit_o <= branch_hit_o;
        branch_decision_o <= branch_decision_o;
        is_fencei_o <= is_fencei_o;
        amo_word_sel_o <= amo_word_sel_o;
        amo_type_o <= amo_type_o;
        is_amo_o <= is_amo_o;

        sys_jump_o <= sys_jump_o;
        sys_jump_csr_addr_o <= sys_jump_csr_addr_o;
        xcpt_valid_o <= xcpt_valid_o;
        xcpt_cause_o <= xcpt_cause_o;
        xcpt_tval_o <= xcpt_tval_o;
        csr_data2fwd_o <= csr_data2fwd_o;
        csr_addr2fwd_o <= csr_addr2fwd_o;
    end
    else if (xcpt_valid)
    begin
        pc_o <= pc_i;
        fetch_valid_o <= 1;
        rs1_data2fwd_o <= 0;
        rs2_data2fwd_o <= 0;
        imm_o <= 0;
        inputA_sel_o <= 2;
        inputB_sel_o <= 0;
        operation_sel_o <= 0;
        signex_sel_o <= 0;
        dsize_sel_o <= 0;
        alu_muldiv_sel_o <= 0;
        shift_sel_o <= 0;
        is_branch_o <= 0;
        is_jal_o <= 0;
        is_jalr_o <= 0;
        regfile_we_o <= 1;
        regfile_input_sel_o <= 5;  // send Execute result into the RFU.
        we_o <= 0;
        re_o <= 0;
        word_sel_o <= 0;
        dsize_sel_o <= 0;
        rd_addr_o <= 0;
        rs1_addr2fwd_o <= 0;
        rs2_addr2fwd_o <= 0;
        csr_we_o <= 0;
        csr_imm_o <= 0;
        branch_hit_o <= 0;
        branch_decision_o <= 0;
        is_fencei_o <= 0;
        amo_word_sel_o <= 0;
        amo_type_o <= 0;
        is_amo_o <= 0;

        sys_jump_o <= 0;
        sys_jump_csr_addr_o <= 0;
        xcpt_valid_o <= xcpt_valid;
        xcpt_cause_o <= xcpt_cause;
        xcpt_tval_o <= xcpt_tval;
        csr_data2fwd_o <= 0;
        csr_addr2fwd_o <= 0;
    end
    else
    begin
        pc_o <= pc_i;
        fetch_valid_o <= fetch_valid_i;
        rs1_data2fwd_o <= rs1_data_i;
        rs2_data2fwd_o <= rs2_data_i;
        imm_o <= imm;
        inputA_sel_o <= inputA_sel;
        inputB_sel_o <= inputB_sel;
        operation_sel_o <= operation_sel;
        signex_sel_o <= signex_sel;
        alu_muldiv_sel_o <= alu_muldiv_sel;
        shift_sel_o <= shift_sel;
        is_branch_o <= rv64_branch;
        is_jal_o <= rv64_jal;
        is_jalr_o <= rv64_jalr;
        regfile_we_o <= regfile_we;
        regfile_input_sel_o <= regfile_input_sel;
        we_o <= we;
        re_o <= re;
        word_sel_o <= word_sel;
        dsize_sel_o <= dsize_sel;
        rd_addr_o <= rd_addr;
        rs1_addr2fwd_o <= rs1_addr_o;
        rs2_addr2fwd_o <= rs2_addr_o;
        csr_we_o <= rv64_csr & !((rv64_csrrs | rv64_csrrc) & rv64_instr[19: 15] == 5'b00000);
        csr_imm_o <= csr_imm;
        branch_hit_o <= branch_hit_i;
        branch_decision_o <= branch_decision_i;
        is_fencei_o <= rv64_fencei;
        amo_word_sel_o <= amo_word_sel;
        amo_type_o <= amo_type;
        is_amo_o <= rv64_amo;

        sys_jump_o <= rv64_mret | rv64_sret;
        sys_jump_csr_addr_o <= ({2{rv64_mret}} & 2'b11) | ({2{rv64_sret}} & 2'b01);
        xcpt_valid_o <= xcpt_valid_i;
        xcpt_cause_o <= xcpt_cause_i;
        xcpt_tval_o <= xcpt_tval_i;
        csr_data2fwd_o <= csr_data_i;
        csr_addr2fwd_o <= csr_addr_o;
    end
end

endmodule
