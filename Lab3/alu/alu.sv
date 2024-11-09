/* ********************************************
 *	COSE222 Lab #3
 *
 *	Module: ALU (alu.sv)
 *  - 32-bit 2 input and 1 output ports
 *
 *  Author: Gunjae Koo (gunjaekoo@korea.ac.kr)
 *
 * ********************************************
 */

`timescale 1ns/1ps

module alu
#(  parameter REG_WIDTH = 32 )  // ALU input data width is equal to the width of register file
(
    input   [REG_WIDTH-1:0] in1,    // Operand 1
    input   [REG_WIDTH-1:0] in2,    // Operand 2
    input   [3:0]   alu_control,    // ALU control signal
    output  logic [REG_WIDTH-1:0] result, // ALU output
    output          zero,           // Zero detection
    output          sign            // Sign bit
);

    always_comb begin
        case (alu_control)
            4'b0010: result = in1 + in2;    // lw, sw, add, addi
            4'b0110: result = in1 - in2;    // sub, beq, bne, blt, bge
            4'b0000: result = in1 & in2;    // and, andi
            4'b0001: result = in1 | in2;    // or, ori
            4'b0011: result = in1 ^ in2;    // xor, xori
            default: result = 0;            // default (to prevent latches)
		endcase
    end

    assign zero = (result == 0);
    assign sign = result[REG_WIDTH-1];

endmodule
