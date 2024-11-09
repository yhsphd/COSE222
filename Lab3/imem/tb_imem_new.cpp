#include <stdlib.h>
#include <iostream>
#include <verilated.h>
#include <verilated_vcd_c.h>
#include "Vimem.h"
#include "Vimem___024root.h"

#define CLK_T 10	// clock period = 10 ns
#define ITER 16
#define CLK_NUM ITER+4
#define RST_OFF 2	// reset is released after this clock counts

int main(int argc, char** argv, char** env) {
	Vimem *dut = new Vimem;

	// initializing waveform file
	Verilated::traceEverOn(true);
	VerilatedVcdC *m_trace = new VerilatedVcdC;
	dut->trace(m_trace, 5);
	m_trace->open("wave.vcd");

	// report file
	FILE* fp = fopen("imem.rpt", "w");

	// test vector
	unsigned int cc = 0;	// clock count
	unsigned int tick = 0;	// half clock
	unsigned int clk = 1;
	unsigned int rst_b = 0;

	dut->addr = 0;

	// initialize memory
	dut->eval();
	for (unsigned int i = 0; i < ITER; i++) {
		dut->rootp->imem__DOT__data[i] = i << 1;
	}

	while (cc < CLK_NUM) {
		// input stimulus
		if (rst_b==0) {
			dut->addr = 0;
		} else {
			if (clk==0) {	// at negedge
				dut->addr++;
			}
		}
		
		
		dut->eval();
		m_trace->dump(tick*CLK_T/2);
		
		// monitoring
		if (clk==1) {
			fprintf(fp, "CC:%4d addr: %02x ", cc, dut->addr);
			fprintf(fp, "dout=%08x\n", dut->dout);
		}

		// clock & reset
		clk ^= 1;
		if (cc==RST_OFF) rst_b = 1;

		tick++;
		if (clk==1) cc++;
	}

	dut->eval();
	m_trace->dump(tick*CLK_T/2);

	for (unsigned int i = 0; i < ITER; i++) {
		fprintf(fp, "data[%2d]: %016x\n", i, dut->rootp->imem__DOT__data[i]);
	}

	fclose(fp);
	m_trace->close();
	delete dut;
	exit(EXIT_SUCCESS);
}
