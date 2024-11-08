#include <stdlib.h>
#include <iostream>
#include <verilated.h>
#include <verilated_vcd_c.h>
#include "Valu.h"

#define CLK_T 10	// clock period = 10 ns
#define CLK_NUM 115
#define RST_OFF 2	// reset is released after this clock counts

int main(int argc, char** argv, char** env) {
	Valu *dut = new Valu;

	// initializing waveform file
	Verilated::traceEverOn(true);
	VerilatedVcdC *m_trace = new VerilatedVcdC;
	dut->trace(m_trace, 5);
	m_trace->open("wave.vcd");

	// report file
	FILE* fp = fopen("alu.rpt", "w");

	// test vector
	unsigned int cc = 0;	// clock count
	unsigned int tick = 0;	// half clock
	unsigned int clk = 1;
	unsigned int rst_b = 0;

	dut->in1 = 0x07;
	dut->in2 = 0x00;
	dut->alu_control = 0;

	while (cc < CLK_NUM) {
		// input stimulus
		if (rst_b==0) {
			dut->in1 = 7;
			dut->in2 = 0;
			dut->alu_control = 0;
		} else {
			if (clk==0) {	// at negedge
				if ((cc-RST_OFF)%16==0) {
					dut->alu_control++;
					dut->in1 = 7;
					dut->in2 = 0;
				} else {
					dut->in2++;
				}
			}
		}
		
		dut->eval();
		m_trace->dump(tick*CLK_T/2);
		
		// monitoring
		if (clk==1) {
			fprintf(fp, "CC:%4d alu_control: %04x ", cc, dut->alu_control);
			fprintf(fp, "in1: %04lx in2: %04lx ", dut->in1, dut->in2);
			fprintf(fp, "result=%016lx, zero=%1d, sign=%1d\n", dut->result, dut->zero, dut->sign);
		}

		// clock & reset
		clk ^= 1;
		if (cc==RST_OFF) rst_b = 1;

		tick++;
		if (clk==1) cc++;
	}

	dut->eval();
	m_trace->dump(tick*CLK_T/2);

	fclose(fp);
	m_trace->close();
	delete dut;
	exit(EXIT_SUCCESS);
}
