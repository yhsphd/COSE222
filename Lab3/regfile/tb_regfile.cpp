#include <stdlib.h>
#include <iostream>
#include <verilated.h>
#include <verilated_vcd_c.h>
#include "Vregfile.h"

#define CLK_T 10	// clock period = 10 ns
#define ITER 32
#define CLK_NUM ITER*2+4
#define RST_OFF 2	// reset is released after this clock counts

int main(int argc, char** argv, char** env) {
	Vregfile *dut = new Vregfile;

	// initializing waveform file
	Verilated::traceEverOn(true);
	VerilatedVcdC *m_trace = new VerilatedVcdC;
	dut->trace(m_trace, 5);
	m_trace->open("wave.vcd");

	FILE *fp = fopen("regfile.rpt", "w");

	// test vector
	unsigned int cc = 0;	// clock count
	unsigned int tick = 0;	// half clock
	unsigned int clk = 1;
	unsigned int rst_b = 0;

	dut->rs1 = 0;
	dut->rs2 = 0;
	dut->rd = 0;
	dut->rd_din = 0;
	dut->reg_write = 0;

	while (cc < CLK_NUM) {
		dut->clk = clk;

		// input stimulus
		if (rst_b==0) {
			dut->rs1 = 0;
			dut->rs2 = 0;
			dut->rd = 0;
			dut->rd_din = 0;
			dut->reg_write = 0;
		} else {
			if (clk==0) {	// at negedge
				if ((cc-RST_OFF) < ITER) {
					dut->reg_write = 1;
				} else {
					dut->reg_write = 0;
				}
				dut->rs1++;
				dut->rs2 = dut->rs1+1;
				dut->rd = dut->rs1;
				dut->rd_din += 4;

				dut->rs1 %= 32;
				dut->rs2 %= 32;
				dut->rd %= 32;
			}
		}
		
		dut->eval();
		m_trace->dump(tick*CLK_T/2);
		
		// monitoring
		if (clk==1) {
			fprintf(fp, "CC:%4d reg_write: %1d ", cc, dut->reg_write);
			fprintf(fp, "rs1: %02x rs2: %02x, rd: %02x ", dut->rs1, dut->rs2, dut->rd);
			fprintf(fp, "rd_din: %016lx ", dut->rd_din);
			fprintf(fp, "rs1_dout=%016lx, rs2_dout=%016lx\n", dut->rs1_dout, dut->rs2_dout);
		}
		
		// clock & reset
		clk ^= 1;
		if (cc==RST_OFF) rst_b = 1;
		
		tick++;
		if (clk==1) cc++;
	}

	dut->eval();
	m_trace->dump(tick*CLK_T/2);

	for (unsigned int i = 0; i < 32; i++) {
		fprintf(fp, "RF[%2d]: %016lx\n", i, dut->regfile__DOT__rf_data[i]);
	}

	fclose(fp);
	m_trace->close();
	delete dut;
	exit(EXIT_SUCCESS);
}
