/* ********************************************
 *	COSE222 Lab #4
 *
 *	Module: top design of the single-cycle CPU (single_cycle_cpu.sv)
 *  - Top design of the single-cycle CPU
 *
 *  Author: Gunjae Koo (gunjaekoo@korea.ac.kr)
 *
 * ********************************************
 */

`timescale 1ns/1ps

module single_cycle_cpu
#(  parameter IMEM_DEPTH = 1024,    // imem depth (default: 1024 entries = 4 KB)
              IMEM_ADDR_WIDTH = 10,
              REG_WIDTH = 32,
              DMEM_DEPTH = 1024,    // dmem depth (default: 1024 entries = 8 KB)
              DMEM_ADDR_WIDTH = 10 )
(
    input           clk,            // System clock
    input           reset_b         // Asychronous negative reset
);

    // Wires for datapath elements
    logic   [IMEM_ADDR_WIDTH-1:0]   imem_addr;
    logic   [31:0]  inst;   // instructions = an output of ????

    logic   [4:0]   rs1, rs2, rd;    // register numbers
    logic   [REG_WIDTH-1:0] rd_din;
    logic           reg_write;
    logic   [REG_WIDTH-1:0] rs1_dout, rs2_dout;

    logic   [REG_WIDTH-1:0] alu_in1, alu_in2;
    logic   [3:0]   alu_control;    // ALU control signal
    logic   [REG_WIDTH-1:0] alu_result;
    logic           alu_zero;
    logic           alu_sign;

    logic   [DMEM_ADDR_WIDTH-1:0]    dmem_addr;
    logic   [31:0]  dmem_din, dmem_dout;
    logic           mem_read, mem_write;

    // -------------------------------------------------------------------
    /* Main control unit:
     * Main control unit generates control signals for datapath elements
     * The control signals are determined by decoding instructions
     * Generating control signals using opcode = inst[6:0]
     */
    logic   [6:0]   opcode;
    logic   [3:0]   branch;
    logic           alu_src, mem_to_reg;
    logic   [1:0]   alu_op;
    logic   [2:0]   funct3;
    //logic         mem_read, mem_write, reg_write; // declared above
    // Note for Lab #5
    // The branch control signal has 4-bits since this processor supports beq, bne, blt, and bge
    // Each bit of the branch control signal represents the corresponding branch instruction
    // i.e., branch[0] = beq, branch[1] = bne, branch[2] = blt, branch[3] = bge

    // COMPLETE THE MAIN CONTROL UNIT HERE
    assign opcode = ;
    assign branch[0] = ;
    assign branch[1] = ;
    assign branch[2] = ;
    assign branch[3] = ;

    assign mem_read = ;    // ld
    assign mem_write = ;   // sd
    assign mem_to_reg = ;
    assign reg_write = ; // ld, r-type, or i-type
    assign alu_src = ;   // ld, sd, or i-type

    assign alu_op[0] = ;
    assign alu_op[1] = ;    // r-type or i-type


    // --------------------------------------------------------------------

    // --------------------------------------------------------------------
    /* ALU control unit:
     * ALU control unit generate alu_control signal which selects ALU operations
     * Generating control signals using alu_op, funct7, and funct3 fileds
     */
    logic   [6:0]   funct7;
    //logic   [2:0]   funct3;   // declared above

    // COMPLETE THE ALU CONTROL UNIT HERE



    // ---------------------------------------------------------------------


    // ---------------------------------------------------------------------
    /* Immediate generator:
     * Generating immediate value from inst[31:0]
     * We require immediate type data for load, store, i-type, and branch instructions
     */
    logic   [REG_WIDTH-1:0]  imm32;
    logic   [REG_WIDTH-1:0]  imm32_branch;  // imm32 left shifted by 1
    logic   [11:0]  imm12;  // 12-bit immediate value extracted from inst

    // COMPLETE IMMEDIATE GENERATOR HERE



    // ----------------------------------------------------------------------

    // Program counter
    logic   [31:0]  pc_curr, pc_next;
    logic           pc_next_sel;    // selection signal for pc_next
    logic   [31:0]  pc_next_plus4, pc_next_branch;


    assign pc_next_plus4 = ;    // FILL THIS

    always_ff @ (posedge clk or negedge reset_b) begin
        if (~reset_b) begin
            pc_curr <= 'b0;
        end else begin
            pc_curr <= ;        // FILL THIS
        end
    end


    // MUXes:
    // COMPLETE MUXES HERE
    // PC_NEXT
    assign pc_next_sel = ;      // FILL THIS
    assign pc_next = (pc_next_sel) ? pc_next_branch: pc_next_plus4; // if branch is taken, pc_next_sel=1'b1
    assign pc_next_branch = ;   // FILL THIS

    // ALU inputs
    assign alu_in1 = ;
    assign alu_in2 = ;

    // RF din
    assign rd_din = ;

    // COMPLETE CONNECTIONS HERE
    // imem
    assign imem_addr = ;

    // regfile
    assign rs1 = ;
    assign rs2 = ;
    assign rd = ;

    // dmem
    assign dmem_addr = ;
    assign dmem_din = ;
 

    // -----------------------------------------------------------------------
    /* Instantiation of datapath elements
     * All input/output ports should be connected
     */
    
    // IMEM
    imem #(
        .IMEM_DEPTH         (IMEM_DEPTH),
        .IMEM_ADDR_WIDTH    (IMEM_ADDR_WIDTH)
    ) u_imem_0 (
        .addr               ( imem_addr     ),
        .dout               (               )
    );

    // REGFILE

    // ALU

    // DMEM

endmodule
