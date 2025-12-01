module core_unit
(
	input clk,
	input nRST,
	// Main interface
	output reg mem_instr,
	output reg mem_valid,
	input mem_ready,
	output reg [31:0] mem_addr,
	output reg[ 3:0] wstrb,
	input [31:0] data_in,
	output reg [31:0] data_out,
	output reg [7:0] fault,
	output reg [7:0] flags,
	// FW upload/dump interface
	input nFW_mode,
	input [31:0] fw_mem_addr,
	input [31:0] fw_data_in,
	output reg [31:0] fw_data_out,
	input [3:0] fw_byte_en,
	input fw_rd_en,
	input fw_wr_en
);

reg sram_wr_en;
reg sram_rd_en;
reg[3:0] sram_byte_en;
reg[9:0] sram_addr;
wire[31:0] sram_data_out;
reg[31:0] sram_data_in;

sram sram_private(
	.clk(clk), .rd_en(sram_rd_en), .wr_en(sram_wr_en),
	.byte_en(sram_byte_en), .data_out(sram_data_out),
	.data_in(sram_data_in), .address(sram_addr)
); // Attached to core if address is between 0x2000 and 0x3000
/*
picorv32 core0(.clk(clk), .resetn(core_reset), .mem_valid(core0_mem_valid), .mem_ready(core0_mem_ready),
					.mem_addr(core0_addr_bus), .mem_rdata(core0_rdata_bus), .mem_wdata(core0_wdata_bus),
					.mem_wstrb(core0_wstrb));
*/
wire core_mem_valid;
reg core_mem_ready;
reg core_mem_ready_internal; // Used when access_private is true
wire core_mem_instr;
wire [31:0] core_mem_addr;
reg [31:0] core_rdata;
wire[31:0] core_wdata;
wire[3:0] core_wstrb;
picorv32 core(
	.clk(clk), .resetn(nRST), .mem_valid(core_mem_valid),
	.mem_ready(core_mem_ready), .mem_addr(core_mem_addr),
	.mem_rdata(core_rdata), .mem_wdata(core_wdata),
	.mem_wstrb(core_wstrb), .mem_instr(core_mem_instr)
);


wire access_private; // Is 1 if private memory is being accessed
assign access_private = (core_mem_addr >= 32'h2000) && (core_mem_addr < 32'h3000);

// Core-SRAM connections
always @* begin /* Combinational Logic */
	if(nFW_mode) begin /* In normal running mode */
		if(access_private) begin /* private mem is between 0x2000-0x3000 */
			sram_rd_en = core_mem_valid & (core_wstrb == 0);
			sram_wr_en = (core_wstrb != 0) & (core_mem_valid);
			sram_byte_en = core_wstrb;
			sram_addr = core_mem_addr[11:2]; // Ignore the first two bits, all accesses are 4 aligned
											 // Ignore bit 12 onwards, since
											 // 0x2000 must be subtracted,
											 // which is essentially the same
											 // operation in this context
			sram_data_in = core_wdata;
			core_rdata = sram_data_out;
			// Disable external interface
			wstrb = 0;
			mem_addr = 0;
			data_out = 0;
			mem_instr = 0;
			mem_valid = 0;
			core_mem_ready = core_mem_ready_internal;
		end else begin
			// Disable SRAM since the mem req is outside of its space
			sram_rd_en = 0;
			sram_wr_en = 0;
			sram_byte_en = 0;
			sram_addr = 0;
			sram_data_in = 0;
			// Send the data out onto the external mem bus
			wstrb = core_wstrb;
			mem_addr = core_mem_addr;
			core_rdata = data_in;
			data_out = core_wdata;
			mem_instr = core_mem_instr;
			mem_valid = core_mem_valid;
			core_mem_ready = mem_ready;
		end
		// Disable FW interface
		fw_data_out = 0;
	end else begin /*Bypass core and give access to memory */
		// Connect private SRAM to fw interface
		sram_wr_en = fw_wr_en;
		sram_rd_en = fw_rd_en;
		sram_data_in = fw_data_in;
		fw_data_out = sram_data_out;
		sram_byte_en = fw_byte_en;
		sram_addr = fw_mem_addr;
		// Disable main interface
		wstrb = 0;
		mem_instr = 0;
		mem_valid = 0;
		mem_addr = 0;
		data_out = 0;
		// Disconnect core from memory
		core_rdata = 0;
		core_mem_ready = 0;
	end
end


always @(posedge clk) begin
	if(nFW_mode) begin // Normal running mode
		if(access_private) begin // If the internal SRAM is used,
								 //1 cycle latency is guaranteed
								 // So, assert mem_ready with a one cycle
								 // delay.
								 // Otherwise, the external interface will
								 // handle it
			if(core_mem_ready_internal) // Clear existing mem_ready
				core_mem_ready_internal <= 0;
			else if(core_mem_valid) // If a mem_valid is asserted while mem_ready_internal is not,
									// activate mem_ready_internal
				core_mem_ready_internal <= 1;
			else // For all other conditions keep it at 0
				core_mem_ready_internal <= 0;
		end
	end
	if(!nRST) begin
		core_mem_ready_internal <= 0;
	end
end

endmodule


module wavefront // Collection of 4 core units
#(parameter CORE_COUNT=4)
( 
	input clk, input nRST,
	// Main Interface
	output reg [CORE_COUNT - 1: 0] mem_instr,
	output reg [CORE_COUNT - 1: 0] mem_valid,
	input [CORE_COUNT - 1: 0] mem_ready,
	output reg [CORE_COUNT*32 - 1:0] mem_addr,
	output reg [CORE_COUNT*4 - 1:0] wstrb,
	input [CORE_COUNT*32 - 1:0] data_in,
	output reg [CORE_COUNT*32 - 1:0] data_out,
	output reg [7:0] fault,
	output reg [7:0] flags,
	// FW upload/dump interface
	input nFW_mode,
	input [$clog2(CORE_COUNT) - 1:0] fw_core_select,
	input fw_wave_select,
	input [31:0] fw_mem_addr,
	input [31:0] fw_data_in,
	output reg [31:0] fw_data_out,
	input [3:0] fw_byte_en,
	input fw_rd_en,
	input fw_wr_en
);

reg sram_wr_en;
reg sram_rd_en;
reg[3:0] sram_byte_en;
reg[9:0] sram_addr;
wire[31:0] sram_data_out;
reg[31:0] sram_data_in;

sram wave_local_sram(
	.clk(clk), .rd_en(sram_rd_en),
	.wr_en(sram_wr_en), .byte_en(sram_byte_en),
	.data_out(sram_data_out), .data_in(sram_data_in),
	.address(sram_addr)
);

reg hw_fault; // Fault in the HDL
reg core_fault; // Fault active on any of the cores
reg [CORE_COUNT - 1:0] mem_instr_different; // Set to one if mem_instr is not the same
reg [CORE_COUNT - 1:0] mem_addr_different; // Set to one if the memory address across all cores is not the same
reg instr_lockstep_fault; // Instruction fetch is out of lockstep
reg mem_fault; // All other memory faults
reg access_private; // Denotes whether private memory is accessed or not
wire wave_local_mem_req; // Set when a valid memory read has been placed during wave-local access
reg wave_local_mem_ready;

assign wave_local_mem_req = sram_rd_en && access_private;

generate 
	genvar i;
	for(i = 0; i < CORE_COUNT; i = i + 1) begin : cores
		wire mem_instr;
		wire mem_valid;
		reg mem_ready;
		wire [31:0] mem_addr;
		wire [3:0] wstrb;
		reg [31:0] data_in;
		wire [31:0] data_out;
		wire [7:0] fault;
		wire [7:0] flags;
		reg [31:0] fw_mem_addr;
		reg [31:0] fw_data_in;
		wire [31:0] fw_data_out;
		reg [3:0] fw_byte_en;
		reg fw_rd_en;
		reg fw_wr_en;
		core_unit cu
		(
			.clk(clk), .nRST(nRST),
			.mem_instr(mem_instr), .mem_valid(mem_valid),
			.mem_ready(mem_ready), .mem_addr(mem_addr),
			.wstrb(wstrb), .data_in(data_in), .data_out(data_out),
			.fault(fault), .flags(flags),
			.nFW_mode(nFW_mode), .fw_mem_addr(fw_mem_addr),
			.fw_data_in(fw_data_in), .fw_data_out(fw_data_out),
			.fw_byte_en(fw_byte_en), .fw_rd_en(fw_rd_en),
			.fw_wr_en(fw_wr_en)
		);
	end
	genvar _i;
	/* Check that mem_instr is the same across all cores */
	for(_i = 1; _i < CORE_COUNT; _i = _i + 1) begin : mid_block
		always @* begin
			mem_instr_different[_i] = (cores[_i].mem_instr != cores[_i - 1].mem_instr);
		end
	end

	/* Check the memory addresses are the same across all cores */
	for(_i = 1; _i < CORE_COUNT; _i = _i + 1) begin : mad_block
		always @* begin
			for(j = 0; j < 31; j = j + 1) begin
				mem_addr_different[_i] = (cores[_i].mem_addr[j] != cores[_i - 1].mem_addr[j]);
			end
		end
	end
endgenerate





integer j;
always @* begin /* Comb */
	// Error Checking
	/* Construct flags register */
	hw_fault = 0;
	core_fault = 0; // TODO: Actually create this
	flags = 0;
	instr_lockstep_fault = 0;
	/* Private instruction memory is mapped to the first 1000 words */
	access_private = (cores[0].mem_instr) && (cores[0].mem_addr < 32'h1000);
	/* If data access is tried on the private space, we flag a mem fault */
	mem_fault = (!cores[0].mem_instr) && (cores[0].mem_addr < 32'h1000);

	instr_lockstep_fault = | mem_instr_different;
	if(cores[0].mem_instr) begin
		instr_lockstep_fault = instr_lockstep_fault | (| mem_addr_different);
	end 
	fault = {instr_lockstep_fault, mem_fault, core_fault};
	// Memory wiring
	mem_instr = 0;
	mem_valid = 0;
	mem_addr = 0;
	wstrb = 0;
	data_out = 0;
	fw_data_out = 0;
	cores[0].data_in = 0;
	cores[1].data_in = 0;
	cores[2].data_in = 0;
	cores[3].data_in = 0;
	// Disable core fw interface
	// Manually unrolling loop at the moment because Verilator doesn't
	// support for loops well
	cores[0].fw_mem_addr = 0;
	cores[0].fw_data_in = 0;
	cores[0].fw_byte_en = 0;
	cores[0].fw_wr_en = 0;
	cores[0].fw_rd_en = 0;
	cores[1].fw_mem_addr = 0;
	cores[1].fw_data_in = 0;
	cores[1].fw_byte_en = 0;
	cores[1].fw_wr_en = 0;
	cores[1].fw_rd_en = 0;
	cores[2].fw_mem_addr = 0;
	cores[2].fw_data_in = 0;
	cores[2].fw_byte_en = 0;
	cores[2].fw_wr_en = 0;
	cores[2].fw_rd_en = 0;
	cores[3].fw_mem_addr = 0;
	cores[3].fw_data_in = 0;
	cores[3].fw_byte_en = 0;
	cores[3].fw_wr_en = 0;
	cores[3].fw_rd_en = 0;
	// We use core 0 as the reference when instructions are accessed
	if(!nFW_mode) begin /* FW mode */
		if(fw_wave_select) begin /* Wave local memory is selected */
			// Enable SRAM fw interface
			sram_addr = fw_mem_addr;
			sram_data_in = fw_data_in;
			fw_data_out = sram_data_out;
			sram_byte_en = fw_byte_en;
			sram_wr_en = fw_wr_en;
			sram_rd_en = fw_rd_en;
		end else begin /* Core memory is selected */
			// Enable core fw interface
			case(fw_core_select) 
				0: begin
					cores[0].fw_mem_addr = fw_mem_addr;
					cores[0].fw_data_in = fw_data_in;
					fw_data_out = cores[0].fw_data_out;
					cores[0].fw_byte_en = fw_byte_en;
					cores[0].fw_wr_en = fw_wr_en;
					cores[0].fw_rd_en = fw_rd_en;
				end
				1: begin
					cores[1].fw_mem_addr = fw_mem_addr;
					cores[1].fw_data_in = fw_data_in;
					fw_data_out = cores[1].fw_data_out;
					cores[1].fw_byte_en = fw_byte_en;
					cores[1].fw_wr_en = fw_wr_en;
					cores[1].fw_rd_en = fw_rd_en;
				end
				2: begin
					cores[2].fw_mem_addr = fw_mem_addr;
					cores[2].fw_data_in = fw_data_in;
					fw_data_out = cores[2].fw_data_out;
					cores[2].fw_byte_en = fw_byte_en;
					cores[2].fw_wr_en = fw_wr_en;
					cores[2].fw_rd_en = fw_rd_en;
				end
				3: begin
					cores[3].fw_mem_addr = fw_mem_addr;
					cores[3].fw_data_in = fw_data_in;
					fw_data_out = cores[3].fw_data_out;
					cores[3].fw_byte_en = fw_byte_en;
					cores[3].fw_wr_en = fw_wr_en;
					cores[3].fw_rd_en = fw_rd_en;
				end
				default: begin
					hw_fault = 1;
				end
			endcase
			// Diable SRAM fw interface
			sram_addr = 0;
			sram_data_in = 0;
			sram_byte_en = 0;
			sram_wr_en = 0;
			sram_rd_en = 0;
		end
	end else begin 
		if(access_private) begin /* Access private mem */
			// SRAM interface, read only
			sram_addr = cores[0].mem_addr[11:2];
			sram_data_in = 0; // Reads only!
			cores[0].data_in = sram_data_out;
			cores[0].mem_ready = wave_local_mem_ready;
			cores[1].data_in = sram_data_out;
			cores[1].mem_ready = wave_local_mem_ready;
			cores[2].data_in = sram_data_out;
			cores[2].mem_ready = wave_local_mem_ready;
			cores[3].data_in = sram_data_out;
			cores[3].mem_ready = wave_local_mem_ready;
			sram_rd_en = cores[0].mem_valid && cores[0].mem_instr;
			sram_wr_en = 0;
			sram_byte_en = 0;
		end else begin /* Cores are doing normal accesses, so pass interface to outside */
			mem_instr[0] = cores[0].mem_instr;
			mem_valid[0] = cores[0].mem_valid;
			cores[0].mem_ready = mem_ready[0];
			wstrb[0] = cores[0].wstrb;
			mem_addr[31:0] = cores[0].mem_addr;
			data_out[31:0] = cores[0].data_out;
			cores[0].data_in = data_in[31:0];
			mem_instr[1] = cores[1].mem_instr;
			mem_valid[1] = cores[1].mem_valid;
			cores[1].mem_ready = mem_ready[1];
			wstrb[1] = cores[1].wstrb;
			mem_addr[63:32] = cores[1].mem_addr;
			data_out[63:32] = cores[1].data_out;
			cores[1].data_in = data_in[63:32];
			mem_instr[2] = cores[2].mem_instr;
			mem_valid[2] = cores[2].mem_valid;
			cores[2].mem_ready = mem_ready[2];
			wstrb[2] = cores[2].wstrb;
			mem_addr[95:64] = cores[2].mem_addr;
			data_out[95:64] = cores[2].data_out;
			cores[2].data_in = data_in[95:64];
			mem_instr[3] = cores[3].mem_instr;
			mem_valid[3] = cores[3].mem_valid;
			cores[3].mem_ready = mem_ready[3];
			wstrb[3] = cores[3].wstrb;
			mem_addr[127:96] = cores[3].mem_addr;
			data_out[127:96] = cores[3].data_out;
			cores[3].data_in = data_in[127:96];

			// Disable SRAM interface
			sram_addr = 0;
			sram_data_in = 0;
			sram_rd_en = 0;
			sram_wr_en = 0;
			sram_byte_en = 0;
		end
	end
end

always @(posedge clk) begin /* Seq */
	if(access_private && (nFW_mode)) begin
		// Wave-local memory access
		if(wave_local_mem_req) begin
			wave_local_mem_ready <= 1;
		end else begin
			wave_local_mem_ready <= 0;
		end
	end 
	

	if(!nRST) begin
		wave_local_mem_ready <= 0;
	end
end
	// Fill in later
endmodule

module picorv_learn(input inclk, input nRST,
output reg[7:0] leds, output uart_tx_pin, input uart_rx_pin, output reg[7:0] dbg_sram);
wire clk;
pll_100 pll0(.inclk0(inclk), .c0(clk));

reg uart_start_read;
wire uart_rd_avl;
wire uart_rx_busy;
wire [7:0] read_data;

reg core_reset;
wire[31:0] core0_addr_bus;
wire[31:0] core0_wdata_bus;
reg[31:0] core0_rdata_bus;
wire[3:0] core0_wstrb; // Write strobe
localparam WSTRB_READ = 4'd0;
localparam WSTRB_BYTE0 = 4'd1;
localparam WSTRB_BYTE1 = 4'd2;
localparam WSTRB_BYTE2 = 4'd4;
localparam WSTRB_BYTE3 = 4'd8;
localparam WSTRB_LOWER_HWORD = 4'd3;
localparam WSTRB_HIGHER_HWORD = 4'd12;
localparam WSTRB_WORD = 4'd15;
wire core0_mem_valid; // core asserts 1 when a request is placed on the bus
reg core0_mem_ready; // peripheral asserts 1 when memory is ready to respond

reg[7:0] state;
reg[7:0] mem_loader_state;
reg[2:0] mem_loader_counter;
localparam STATE_IDLE = 8'b0;
localparam STATE_LOAD_MEMORY = 8'd1;
localparam STATE_TRANSMIT_MEMORY_DEBUG = 8'd3;
localparam STATE_RUN_PROGRAM = 8'd2;
localparam STATE_MEMLOADER_START = 8'd1;
localparam STATE_MEMLOADER_LATCH = 8'd2;
localparam STATE_MEMLOADER_WAIT = 8'd3;
localparam STATE_MEMLOADER_WRITE = 8'd4;

reg sram0_wr_en;
reg sram0_rd_en;
reg[9:0] sram0_addr_bus;
reg[31:0] sram0_data_in;
reg[3:0] sram0_data_be;
wire[31:0] sram0_data_out;
sram sram0(
	.clk(clk), .rd_en(sram0_rd_en), .wr_en(sram0_wr_en),
	.data_in(sram0_data_in), .data_out(sram0_data_out),
	.address(sram0_addr_bus), .byte_en(sram0_data_be)
); //1K*4B SRAM
reg[9:0] sram_rd_ptr;
reg[9:0] sram_wr_ptr;

always @* begin
	if(state == STATE_LOAD_MEMORY) begin
		sram0_rd_en = 0;
		case(mem_loader_counter)
			0:
				sram0_data_be = 4'b1;
			1:
				sram0_data_be = 4'b10;
			2:
				sram0_data_be = 4'b100;
			3:
				sram0_data_be = 4'b1000;
			default: begin
				sram0_data_be = 4'b1111;
			end
		endcase
		sram0_addr_bus = sram_wr_ptr;
		sram0_data_in = {4{read_data}}; // Since we use a mask, replicate the read byte
		sram0_wr_en = uart_rd_avl & ~(uart_start_read); // Latch data in when output is valid
		core0_rdata_bus = 0;
	end
	else if (state == STATE_RUN_PROGRAM) begin
		sram0_rd_en = (core0_wstrb == WSTRB_READ) && core0_mem_valid;
		sram0_data_be = core0_wstrb;
		sram0_addr_bus = core0_addr_bus[11:2]; // Ignore the first two bits because SRAM has a 1024*4K struct
		sram0_data_in = core0_wdata_bus;
		sram0_wr_en = (core0_wstrb != 0) && core0_mem_valid;
		core0_rdata_bus = sram0_data_out;
	end
	else if (state == STATE_TRANSMIT_MEMORY_DEBUG) begin
		sram0_rd_en = 1;
		sram0_addr_bus = sram_rd_ptr;
		sram0_data_in = 0;
		sram0_data_be = 4'b1111;
		sram0_wr_en = 0;
		core0_rdata_bus = 0;
	end
	else begin
		sram0_addr_bus = 0;
		sram0_data_in = 0;
		sram0_data_be = 4'b1111;
		sram0_wr_en = 0;
		sram0_rd_en = 0;
		core0_rdata_bus = 0;
	end
end



uart_rx urx(.clk(clk), .n_reset(nRST), .rx_pin(uart_rx_pin), .start_read(uart_start_read),
				.read_avl(uart_rd_avl), .read_data(read_data), .busy(uart_rx_busy));

picorv32 core0(.clk(clk), .resetn(core_reset), .mem_valid(core0_mem_valid), .mem_ready(core0_mem_ready),
					.mem_addr(core0_addr_bus), .mem_rdata(core0_rdata_bus), .mem_wdata(core0_wdata_bus),
					.mem_wstrb(core0_wstrb));

always @(posedge clk) begin
	if(!nRST)begin
		state <= STATE_IDLE;
		mem_loader_state <= STATE_IDLE;
		mem_loader_counter <= 0;
		core_reset <= 0; // Keep all cores reset till data is available;
		core0_mem_ready <= 0;
		sram_rd_ptr <= 0;
		sram_wr_ptr <= 0;
		uart_start_read <= 0;
		leds <= 0;
	end
	else begin
		case(state)
			STATE_IDLE: begin
				// Reset all necessary signals
				mem_loader_counter <= 0;
				uart_start_read <= 0;
				sram_rd_ptr <= 0;
				sram_wr_ptr <= 0;
				state <= STATE_LOAD_MEMORY;
				leds[0] <= 1;
			end
			STATE_LOAD_MEMORY: begin
				case(mem_loader_state)
					STATE_IDLE: begin // Wait till data is given
						if(uart_rd_avl) begin
							uart_start_read <= 0;
							if(read_data == "S") begin
								mem_loader_state <= STATE_MEMLOADER_START;
							end else begin
								mem_loader_state <= STATE_IDLE;
							end
						end else if(!uart_rx_busy) begin
							uart_start_read <= 1;
						end else begin
							uart_start_read <= 0;
						end
					end
					STATE_MEMLOADER_START: begin // Wait till busy goes away. Then assert a read start
						if(!uart_rx_busy) begin
							uart_start_read <= 1;
							mem_loader_state <= STATE_MEMLOADER_LATCH;
						end else
							uart_start_read <= 0;
					end
					STATE_MEMLOADER_LATCH:
						mem_loader_state <= STATE_MEMLOADER_WAIT;
					STATE_MEMLOADER_WAIT: begin
						uart_start_read <= 0;
						if(uart_rd_avl) begin // New data is available
							if(mem_loader_counter == 3) begin
								sram_wr_ptr <= sram_wr_ptr + 1;
								mem_loader_counter <= 0;
							end
							else
								mem_loader_counter <= mem_loader_counter + 1;
							mem_loader_state <= STATE_MEMLOADER_START; // Return back to idle state
							if(sram_wr_ptr == 10'd1023) begin // If all 1024 bytes have been loaded, start running the core
								state <= STATE_TRANSMIT_MEMORY_DEBUG;
								mem_loader_state <= STATE_IDLE;
							end
						end else begin
						end
					end
					default: begin
							mem_loader_state <= STATE_IDLE;
							state <= STATE_IDLE;
					end
				endcase
			end
			STATE_TRANSMIT_MEMORY_DEBUG: begin
				dbg_sram <= sram0_data_out;
				sram_rd_ptr <= sram_rd_ptr + 1;
				if(sram_rd_ptr == 10'd1023) begin
					state <= STATE_RUN_PROGRAM;
					sram_rd_ptr <= 0;
				end
			end
			STATE_RUN_PROGRAM: begin
				core_reset <= 1; // Release reset on cores
				if(core0_mem_valid) begin // Memory request has been placed, gets reset when mem_ready is set
					core0_mem_ready <= 1;
				end else begin
					core0_mem_ready <= 0; // If no request has been placed, deassert mem_ready
				end
			end
			default:
				state <= STATE_IDLE;
		endcase
	end
end
endmodule
