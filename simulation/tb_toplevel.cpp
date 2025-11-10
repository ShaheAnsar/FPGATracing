#include <exception>
#include <iostream>
#include <string>
#include <fstream>
#include <filesystem>
#include <memory>
#include <verilated.h>
#include <verilated_vcd_c.h>
#include "Vpicorv_learn.h"
#include "Vpicorv_learn___024root.h"


size_t clk_rate = 12000000; // Clk is 12MHz

struct UART_Tx{ // Simulated UART Tx
	std::unique_ptr<uint8_t[]> data; // Data array
	std::size_t data_size;
	int tx; // tx pin
	size_t _data_bit_i; // Bit index of the current byte
	size_t _data_i; // Index of current byte in buffer
	size_t prediv_top;
	size_t prediv_counter; 
	int emit_start;
	int emit_stop;
	UART_Tx(std::string filepath) : 
		tx(1),
		_data_i(0),
		_data_bit_i(0),
		prediv_top(clk_rate/9600 - 1),
		prediv_counter(0),
		emit_start(1),
		emit_stop(0)
	{
		try{
			std::size_t filesize = std::filesystem::file_size(filepath);
			data_size = filesize;
			data = std::make_unique<uint8_t[]>(filesize);
			std::cout << "File size: " << filesize << std::endl;
			std::fstream bin_file(filepath, std::ios::in | std::ios::binary);
			std::cout << "File opned" << std::endl;
			bin_file.read(reinterpret_cast<char*>(data.get()), data_size);
			std::cout << "File read" << std::endl;
			bin_file.close();
		} catch(std::exception e) {
			std::cerr << "Uh oh" << std::endl;
		}
	}

	void step() {
		if(prediv_counter == prediv_top){
			prediv_counter = 0;
			if(emit_start){
				tx = 0; // Start bit
				emit_start = 0;
			}
			else if(emit_stop){
				tx = 1; // Stop bit
				emit_stop = 0;
				emit_start = 1;
			}
			else{
				tx = (data[_data_i] & (1 << _data_bit_i)) != 0;
				_data_bit_i++;
				if(_data_bit_i >= 8){
					emit_stop = 1;
					_data_bit_i = 0;
					_data_i++;
					if(_data_i >= data_size) {
						_data_i = 0;
					}
				}
			}
		}
		else
			prediv_counter++;
	}
};

uint64_t sim_time = 0;
int enable_dumping = 1;
Vpicorv_learn* dut;
VerilatedVcdC* m_trace;


void init_sim() {
	dut = new Vpicorv_learn;
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

void sim_reset() {
	dut->nRST = 0; // Set reset to 0
	dut->uart_rx_pin = 1; // Set uart input to idle
	for(int i = 0; i < 5; i++) { // Stay reset for the next 5 clocks
		sim_clock_cycle();
	} // Exiting on a falling edge
	dut->nRST = 1;
}


void sim_load_fw() { // Load firmware using simulated UART
	UART_Tx utx("/home/shahe/development/FPGA/10CL25/picorv_learn/firm/main.bin");
	int mem_size = 1024 * 4; // 4KB memory
	
	for(int i = 0; i < mem_size * 12500 + 10; i++) { // 1250 ticks per baudclock, and 10 baudclocks per byte, and 10 latency
		if(utx._data_i < utx.data_size) { // If data is still avaialable
			utx.step();
			dut->uart_rx_pin = utx.tx;
		} else { // No data
			dut->uart_rx_pin = 1; // Idle
			break;
		}
		sim_clock_cycle();
	} // Data is now successfully loaded
}

void sim_run_prog() { // Run program on core
	int num_clocks = 1000; // Run for 1000 clocks
	for(int i = 0; i < num_clocks; i++) {
		sim_clock_cycle();
	}
}

void dump_ram() {
	for(int i = 0; i < 1024; i++) {
		std::cout << std::hex << "[" << i*4 << "]: " << "0x" <<
			dut->rootp->picorv_learn__DOT__sram0__DOT__sram0__DOT__memory[i] << std::endl;
	}
}


int main(void) {
	init_sim();
	sim_reset(); // Reset
	sim_load_fw();
	sim_run_prog();
	dump_ram();
	finish_sim();
}
