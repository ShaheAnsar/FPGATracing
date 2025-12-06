#include <exception>
#include <iostream>
#include <string>
#include <fstream>
#include <filesystem>
#include <memory>
#include <verilated.h>
#include <verilated_vcd_c.h>
#include "Vnewton_raphson_divider.h"
#include "Vnewton_raphson_divider___024root.h"


size_t clk_rate = 100000000; // Clk is 12MHz


uint64_t sim_time = 0;
int enable_dumping = 1;
Vnewton_raphson_divider* dut;
VerilatedVcdC* m_trace;


void init_sim() {
	dut = new Vnewton_raphson_divider;
	Verilated::traceEverOn(true);
	m_trace =  new VerilatedVcdC;
	dut->trace(m_trace, 5);
	m_trace->open("waveform.vcd");
}

void finish_sim() {
	m_trace->close();
	delete dut;
}

void sim_clock_cycle() {
	dut->clk ^= 1; // Rising edge
	dut->eval();
	if(enable_dumping)
		m_trace->dump(sim_time);
	sim_time++;
	dut->clk ^= 1; // Falling edge
	dut->eval();
	if(enable_dumping)
		m_trace->dump(sim_time);
	sim_time++;
}

void half_clock() {
	dut->clk ^= 1; // Falling edge
	dut->eval();
	if(enable_dumping)
		m_trace->dump(sim_time);
	sim_time++;
}

void sim_reset() {
	dut->nRST = 0; // Set reset to 0
	for(int i = 0; i < 5; i++) { // Stay reset for the next 5 clocks
		sim_clock_cycle();
	} // Exiting on a falling edge
	dut->nRST = 1;
}


void sim_run_division() {
	dut->divider = -10 * (1 << 16);
	dut->dividend = 3 * ( 1 << 16);
	dut->request = 1;
	int time_start = sim_time;
	for(int i = 0; i < 20; i++) {
		if(dut->ready) {
			std::cout << "Cycles taken: " << (sim_time - time_start)/2 << std::endl;
			break;
		}
		sim_clock_cycle();
		dut->request = 0;
	}
	std::cout << "Result: " << dut->quotient << "; Binary 0b" << std::bitset<32>(dut->quotient) <<
		"; Float conv: " << (float)( dut->quotient)/(1 << 16) << std::endl;
}



int main(void) {
	init_sim();
	sim_reset(); // Reset
	sim_run_division();
	finish_sim();
}
