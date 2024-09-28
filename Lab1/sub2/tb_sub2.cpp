#include <stdlib.h>
#include <iostream>
#include <verilated.h>
#include <verilated_vcd_c.h> // required for generating waveforms
#include "Vsub2.h"           // “sub2” is the module name

#define ITER 16

int main(int argc, char **argv, char **env)
{
    Vsub2 *dut = new Vsub2;

    // initializing waveform dump
    Verilated::traceEverOn(true);
    VerilatedVcdC *m_trace = new VerilatedVcdC;
    dut->trace(m_trace, 5);
    m_trace->open("wave.vcd");

    // test vectors
    for (int i = 0; i < ITER; i++)
    {
        dut->a = i;
        dut->b = ITER - 1 - i;
        dut->eval();
        m_trace->dump(i);
        printf("a: 0x%X, b: 0x%X, c: 0x%X\n", dut->a, dut->b, dut->c);
    }
    dut->eval();
    m_trace->dump(ITER);

    // closing
    m_trace->close();
    delete dut;
    exit(EXIT_SUCCESS);
}
