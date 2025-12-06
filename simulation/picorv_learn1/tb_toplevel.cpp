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


size_t clk_rate = 100000000; // Clk is 12MHz

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
		prediv_top(clk_rate/115200 - 1),
		prediv_counter(0),
		emit_start(1),
		emit_stop(0)
	{
		try{
			std::size_t filesize = std::filesystem::file_size(filepath);
			data_size = filesize;
			data = std::make_unique<uint8_t[]>(filesize + 1);
			std::cout << "File size: " << filesize << std::endl;
			std::fstream bin_file(filepath, std::ios::in | std::ios::binary);
			std::cout << "File opned" << std::endl;
			bin_file.read(reinterpret_cast<char*>(data.get()) + 1, data_size);
			std::cout << "File read" << std::endl;
			bin_file.close();
			data[0] = (uint8_t)'S';
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
	dut->inclk ^= 1; // Rising edge
	dut->eval();
	if(enable_dumping)
		m_trace->dump(sim_time);
	sim_time++;
	dut->inclk ^= 1; // Falling edge
	dut->eval();
	if(enable_dumping)
		m_trace->dump(sim_time);
	sim_time++;
}

void half_clock() {
	dut->inclk ^= 1; // Falling edge
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
	UART_Tx utx("firm/final.bin");
	int mem_size = 20 * 1024 * 4; // 20KB memory
	enable_dumping = 0;
	
	for(int i = 0; i < mem_size * 8680 + 10; i++) { // 868 ticks per baudclock, and 10 baudclocks per byte, and 10 latency
		half_clock();
		if(dut->rootp->picorv_learn__DOT__state == 8) {
			std::cout << "FW loaded" << std::endl;
			break;
		}
		if(i >= 100000)
			enable_dumping = 0;
		if(!(i % 1000000)) {
			std::cout << "Loaded [" << dut->rootp->picorv_learn__DOT__read_counter/(float)1024/5 << "]" << std::endl;
		}
		if(utx._data_i < utx.data_size) { // If data is still avaialable
			utx.step();
			dut->uart_rx_pin = utx.tx;
		} else { // No data
			dut->uart_rx_pin = 1; // Idle
			break;
		}
		//sim_clock_cycle();
		half_clock();
	} // Data is now successfully loaded
	m_trace->close();
	m_trace->open("waveform2.vcd"); // Dump first 1000 cycles of program here
}

void sim_run_prog() { // Run program on core
	enable_dumping = 1;
	int num_clocks = 100000000; // Run for 1000 clocks
	int tx_dump_count = 0;
	for(int i = 0; i < num_clocks; i++) {
		if(i % 100 == 0)
			std::cout << "Tx count" << tx_dump_count << std::endl;
		if(dut->rootp->picorv_learn__DOT__state == 9)
			break;
		if(tx_dump_count >= 1000) {
			std::cout << "Tx dumped!" << std::endl;
			enable_dumping = 0;
			m_trace->close();
			break;
		} else
		if(dut->rootp->picorv_learn__DOT__state == 5) {
			tx_dump_count++;
			m_trace->open("Txwave.vcd");
			enable_dumping = 1;
		}
		if(i == 10000){
			m_trace->close();
			enable_dumping = 0;
		}
		sim_clock_cycle();
	}
}

//void dump_ram() {
//	for(int i = 0; i < 1024; i++) {
//		std::cout << std::hex << "[" << i*4 << "]: ";
//		for(int j = 0; j < 4; j++)
//			std::cout << unsigned(reinterpret_cast<uint8_t*>
//				(&dut->rootp->picorv_learn__DOT__sram0__DOT__memory[i])[j]) << " ";
//		std::cout << ";\t" <<
//			dut->rootp->picorv_learn__DOT__sram0__DOT__memory[i] << std::endl;
//	}
//}

void dump_fb() {
	std::ofstream fb_dump("fb.dump", std::ios::binary);
	if(fb_dump.is_open()){
		fb_dump.write(reinterpret_cast<char*>( &dut->rootp->picorv_learn__DOT__fb__DOT__sram_fb__DOT__memory[0] ), 2048 * 4);
		fb_dump.close();
	} else {
		std::cout << "Error occurred during fb dump!" << std::endl;
	}
}

void dump_instructions() {
	std::ofstream fb_dump("instructions.dump", std::ios::binary);
	if(fb_dump.is_open()){
		fb_dump.write(reinterpret_cast<char*>( &dut->rootp->picorv_learn__DOT__wv0__DOT__wave_local_sram__DOT__memory[0] ), 1024 * 4);
		fb_dump.close();
	} else {
		std::cout << "Error occurred during fb dump!" << std::endl;
	}
}


int main(void) {
	init_sim();
	sim_reset(); // Reset
	sim_load_fw();
	sim_run_prog();
	dump_fb();
	dump_instructions();
	finish_sim();
}
