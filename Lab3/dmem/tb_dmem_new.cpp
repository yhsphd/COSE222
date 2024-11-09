#include <stdlib.h>
#include <iostream>
#include <verilated.h>
#include <verilated_vcd_c.h>
#include "Vdmem.h"
#include "Vdmem___024root.h"

#define CLK_T 10	// clock period = 10 ns
#define ITER 16
#define CLK_NUM ITER*2+4
#define RST_OFF 2	// reset is released after this clock counts

int main(int argc, char** argv, char** env) {
	Vdmem *dut = new Vdmem;

	// initializing waveform file
	Verilated::traceEverOn(true);
	VerilatedVcdC *m_trace = new VerilatedVcdC;
	dut->trace(m_trace, 5);
	m_trace->open("wave.vcd");

	// report file
	FILE* fp = fopen("dmem.rpt", "w");

	// test vector
	unsigned int cc = 0;	// clock count
	unsigned int tick = 0;	// half clock
	unsigned int clk = 1;
	unsigned int rst_b = 0;

	dut->addr = 0;
	dut->din = 0;
	dut->mem_read = 0;
	dut->mem_write = 0;

	// initialize memory
	dut->eval();
	for (unsigned int i = 0; i < ITER; i++) {
		dut->rootp->dmem__DOT__data[i] = i*4;
	}

	while (cc < CLK_NUM) {
		dut->clk = clk;

		// input stimulus
		if (rst_b==0) {
			dut->addr = 0;
			dut->din = 0;
			dut->mem_read = 0;
			dut->mem_write = 0;
		} else {
			if (clk==0) {	// at negedge
				if ((cc-RST_OFF) < ITER) {
					dut->mem_read = 0;
					dut->mem_write = 1;
				} else if ((cc-RST_OFF) < ITER*2) {
					dut->mem_read = 1;
					dut->mem_write = 0;
				} else {
					dut->mem_read = 0;
					dut->mem_write = 0;
				}
				dut->addr++;
				dut->addr %= ITER;
				dut->din += 2;
			}
		}
		
		dut->eval();
		m_trace->dump(tick*CLK_T/2);
		
		// monitoring
		if (clk==1) {
			fprintf(fp, "CC:%4d mem_read: %1d mem_write: %1d ", cc, dut->mem_read, dut->mem_write);
			fprintf(fp, "addr: %02x din: %016lx ", dut->addr, dut->din);
			fprintf(fp, "dout=%016lx\n", dut->dout);
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
		fprintf(fp, "data[%2d]: %016lX\n", i, dut->rootp->dmem__DOT__data[i]);
	}

	fclose(fp);
	m_trace->close();
	delete dut;
	exit(EXIT_SUCCESS);
}
