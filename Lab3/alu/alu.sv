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
            
		endcase
    end

    assign zero = ;
    assign sign = ;

endmodule