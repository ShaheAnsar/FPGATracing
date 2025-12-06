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
	cores[0].mem_ready = 0;
	cores[1].mem_ready = 0;
	cores[2].mem_ready = 0;
	cores[3].mem_ready = 0;
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
			wstrb[3:0] = cores[0].wstrb;
			mem_addr[31:0] = cores[0].mem_addr;
			data_out[31:0] = cores[0].data_out;
			cores[0].data_in = data_in[31:0];
			mem_instr[1] = cores[1].mem_instr;
			mem_valid[1] = cores[1].mem_valid;
			cores[1].mem_ready = mem_ready[1];
			wstrb[7:4] = cores[1].wstrb;
			mem_addr[63:32] = cores[1].mem_addr;
			data_out[63:32] = cores[1].data_out;
			cores[1].data_in = data_in[63:32];
			mem_instr[2] = cores[2].mem_instr;
			mem_valid[2] = cores[2].mem_valid;
			cores[2].mem_ready = mem_ready[2];
			wstrb[11:8] = cores[2].wstrb;
			mem_addr[95:64] = cores[2].mem_addr;
			data_out[95:64] = cores[2].data_out;
			cores[2].data_in = data_in[95:64];
			mem_instr[3] = cores[3].mem_instr;
			mem_valid[3] = cores[3].mem_valid;
			cores[3].mem_ready = mem_ready[3];
			wstrb[15:12] = cores[3].wstrb;
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

module debug_fb// Use sram during debug
#
(
	parameter PORTS=8,
	parameter DEPTH=2048
)
(
	input clk, input nRST,
	// Main Interface
	input [$clog2(DEPTH) * (PORTS) - 1 : 0] addr,
	input [32 * (PORTS) - 1 : 0] data_in,
	input [4 * PORTS - 1 : 0] byte_en,
	input [PORTS - 1 : 0] wr_en,
	input [PORTS - 1 : 0] rd_en,
	input [PORTS - 1 : 0] request,
	output reg [32 * PORTS - 1 : 0] data_out,
	output reg [PORTS - 1 : 0] grant,
	output reg [PORTS - 1 : 0] ready,
	output fault,
	// FW interface
	input nFW_mode,
	input [$clog2(DEPTH) - 1 : 0] fw_mem_addr,
	input [31:0] fw_data_in,
	input [3:0] fw_byte_en,
	input fw_rd_en,
	input fw_wr_en,
	output reg [31:0] fw_data_out
);

localparam SIZE = $clog2(DEPTH);


reg [SIZE - 1 : 0] fb_mem_addr;
reg [31 : 0] fb_data_in;
wire [31 : 0] fb_data_out;
reg fb_rd_en;
reg fb_wr_en;
reg [3:0] fb_byte_en;

sram #(.DEPTH(DEPTH)) sram_fb
(
	.clk(clk),  .wr_en(fb_wr_en),
	.rd_en(fb_rd_en), .address(fb_mem_addr),
	.data_out(fb_data_out), .data_in(fb_data_in),
	.byte_en(fb_byte_en)
);

reg prio_enc_fault;
reg pipeline_fault;
reg req_fault;

assign fault = prio_enc_fault | pipeline_fault | req_fault;

reg [2:0] current_req;
reg current_req_valid;

/* Prio Enc */
always @* begin
	current_req_valid = 1;
	prio_enc_fault = 0;
	if(request == 0) begin /* No request */
		current_req_valid = 0;
		current_req = 3'd0;
	end else if(request & 1) begin /* LSB request is given priority */
		current_req = 3'd0;
	end else if (request & (1 << 1)) begin
		current_req = 3'd1;
	end else if (request & (1 << 2)) begin
		current_req = 3'd2;
	end else if (request & (1 << 3)) begin
		current_req = 3'd3;
	end else if (request & (1 << 4)) begin
		current_req = 3'd4;
	end else if (request & (1 << 5)) begin
		current_req = 3'd5;
	end else if (request & (1 << 6)) begin
		current_req = 3'd6;
	end else if (request & (1 << 7)) begin
		current_req = 3'd7;
	end else begin
		current_req_valid = 0;
		current_req = 3'd0;
		prio_enc_fault = 1;
	end
end

localparam STATE_IDLE = 3'd0;
localparam STATE_CHOOSE = 3'd1;
localparam STATE_GRANT = 3'd2;
localparam STATE_PROCESS = 3'd3;
localparam STATE_OUTPUT = 3'd4;

reg[2:0] state;
/* Stage A */
reg[SIZE - 1:0] req_addr;
reg[31:0] req_data_in;
reg[3:0] req_byte_en;
reg[2:0] req_id;
reg req_rd_en;
reg req_wr_en;
reg pipeline_stage_A_valid;
/* Stage B */
reg[2:0] req_id_B;
reg req_wr_en_B;
reg req_rd_en_B;
reg pipeline_stage_B_valid;
/* Stage C */
reg[2:0] req_id_C;
reg req_wr_en_C;
reg req_rd_en_C;
reg[31:0] req_data_out;
reg pipeline_stage_C_valid;

/* SRAM pipeline connections */
always @* begin
	grant = (current_req_valid)?(1 << current_req):0;
	if(pipeline_stage_A_valid) begin /* Request is valid */
		fb_mem_addr = req_addr;
		fb_data_in = req_data_in;
		fb_rd_en = req_rd_en;
		fb_wr_en = req_wr_en;
		fb_byte_en = req_byte_en;
	end else begin /* Request invalid, disconnect */
		fb_mem_addr = 0;
		fb_data_in = 0;
		fb_rd_en = 0;
		fb_wr_en = 0;
		fb_byte_en = 0;
		data_out = 0;
	end

	if(pipeline_stage_C_valid) begin /* Output valid */
		ready = (1 << req_id_C); // Assert mem_ready for the relevant bus
		data_out = 0;
		data_out[(req_id_C + 1) * 32 - 1 -: 32] = req_data_out;
	end else begin /* Output invalid, disconnect */
		ready = 0;
		data_out = 0;
	end
end

always @(posedge clk) begin
	/* Acquire state */
	if( | request) begin /* Request Available */
		req_addr <= addr[(current_req + 1) * SIZE - 1 -: SIZE];
		req_data_in <= data_in[(current_req + 1) * 32 - 1 -: 32];
		req_byte_en <= byte_en[(current_req + 1) * 4 - 1 -: 4];
		req_rd_en <= rd_en[current_req];
		req_wr_en <= wr_en[current_req];
		req_id <= current_req;
		pipeline_stage_A_valid <= 1;
	end else begin
		pipeline_stage_A_valid <= 0;
	end

	if(pipeline_stage_A_valid) begin /* Commit to RAM */
		req_id_B <= req_id;
		req_wr_en_B <= req_wr_en;
		req_rd_en_B <= req_rd_en;
		pipeline_stage_B_valid <= 1;
	end else begin
		pipeline_stage_B_valid <= 0;
	end

	if(pipeline_stage_B_valid) begin /* Output to BUS */
		req_id_C <= req_id_B;
		if(req_rd_en_B) begin /* Read request */
			req_data_out <= fb_data_out;
		end
		pipeline_stage_C_valid <= 1;
	end else begin
		pipeline_stage_C_valid <= 0;
	end

	if(!nRST) begin
		req_addr <= 0;
		req_data_in <= 0;
		req_byte_en <= 0;
		req_wr_en <= 0;
		req_rd_en <= 0;
		req_id <= 0;
		pipeline_stage_A_valid <= 0;
		pipeline_stage_B_valid <= 0;
	end
end

endmodule

module mem_sync
#(
	parameter DEPTH=2048 /* Depth of the FB in words */
)
(
	input clk, input nRST,
	// Connects to wavefront
	input [3:0] mem_instr,
	input [3:0] mem_valid,
	output reg [3:0] mem_ready,
	input [127:0] mem_addr,
	input [15:0] wstrb,
	output reg [127:0] data_in, /* Connects to data_in */
	input [127:0] data_out, /* Connects to data_out */
	// Connects to arbiter
	output reg [$clog2(DEPTH)*4 - 1:0] arbiter_addr,
	input [127:0] arbiter_data_out, /* Connects to data_out */
	output reg [127:0] arbiter_data_in,  /* Connects to data_in */
	output reg [3:0] arbiter_wr_en,
	output reg [3:0] arbiter_rd_en,
	output reg [15:0] arbiter_byte_en,
	output reg [3:0] arbiter_request,
	input [3:0] arbiter_grant,
	input [3:0] arbiter_ready
);

localparam SIZE = $clog2(DEPTH);


reg [127:0] latched_arbiter_data_out;

reg [3:0] received_grant;
reg [3:0] serviced_requests;

reg [3:0] wave_ready_reg;

localparam IDLE = 3'd0;
localparam PENDING = 3'd1;
localparam FINISH = 3'd2;
reg [2:0] state;

always @* begin
	mem_ready = wave_ready_reg;
end

generate
genvar i;
for(i = 0; i < 4; i = i + 1) begin : bit_i_wiring
	always @* begin
		data_in[(i + 1) * 32 - 1 -: 32]  = 0;
		arbiter_addr[(i + 1 ) * SIZE - 1 -: SIZE] = 0;
		arbiter_data_in[(i + 1 ) *32 - 1 -: 32] = 0;
		arbiter_wr_en[i] = 0;
		arbiter_rd_en[i] = 0;
		arbiter_request[i] = 0;
		arbiter_byte_en[(i + 1) * 4 - 1 -: 4] = 0;
		if(state == PENDING) begin
			arbiter_addr[(i + 1 ) * SIZE - 1 -: SIZE] = mem_addr[i * 32 +: SIZE];
			arbiter_data_in[(i + 1) * 32 - 1 -: 32] = data_out[(i + 1) * 32 - 1 -: 32 ];
			arbiter_wr_en[i] = (mem_valid[i]) && (wstrb[(i + 1) * 4 - 1 -: 4] != 0);
			arbiter_rd_en[i] = (mem_valid[i]) && (wstrb[(i + 1) * 4 - 1 -: 4] == 0);
			arbiter_byte_en[(i + 1) * 4 - 1 -: 4] = wstrb[(i + 1) * 4 - 1 -: 4];
			data_in[(i + 1) * 32 - 1 -: 32] = latched_arbiter_data_out[(i + 1) * 32 - 1 -: 32];
			arbiter_request[i] = mem_valid[i] && (~received_grant[i]);
		end 
	end
end
endgenerate

always @(posedge clk) begin
	case(state)
		IDLE : begin
			received_grant <= 0;
			serviced_requests <= 0;
			latched_arbiter_data_out <= 0;
			wave_ready_reg <= 0;
			if(| mem_valid) begin // Latch in memory requests
				state <= PENDING;
			end
		end
		PENDING: begin
			latched_arbiter_data_out <= arbiter_data_out | latched_arbiter_data_out; // Compound data_outs
			received_grant <= received_grant | arbiter_grant; // Update grants that have been received;
			serviced_requests <= serviced_requests | arbiter_ready; // Update all the ready requests
			if(serviced_requests == 4'b1111) begin
				wave_ready_reg <= serviced_requests;
				state <= IDLE;
			end
		end
	endcase


	if(!nRST) begin
		state <= IDLE;
		received_grant  <= 0;
		serviced_requests <= 0;
		latched_arbiter_data_out <= 0;
		wave_ready_reg <= 0;
	end
end
endmodule

// Meant for 4 cores atm
module simple_gpu
(
	input clk, input nRST,
	// FW interface
	input nFW_mode,
	input [3:0] fw_core_select,
	input fw_wave_select,
	input [31:0] fw_mem_addr,
	input [31:0] fw_data_in,
	output reg [31:0] fw_data_out,
	input [3:0] fw_byte_en,
	input fw_rd_en,
	input fw_wr_en
);

reg [3:0] wv0_mem_sync_mem_instr;
reg [3:0] wv0_mem_sync_mem_valid;
wire [3:0] wv0_mem_sync_mem_ready;
reg [127:0] wv0_mem_sync_mem_addr;
reg [15:0] wv0_mem_sync_wstrb;
wire [127:0] wv0_mem_sync_data_in;
reg [127:0] wv0_mem_sync_data_out;

wire [43:0] wv0_mem_sync_arbiter_addr;
reg [127:0] wv0_mem_sync_arbiter_data_out;
wire [127:0] wv0_mem_sync_arbiter_data_in;
wire [3:0] wv0_mem_sync_arbiter_wr_en;
wire [3:0] wv0_mem_sync_arbiter_rd_en;
wire [15:0] wv0_mem_sync_arbiter_byte_en;
wire [3:0] wv0_mem_sync_arbiter_request;
reg [3:0] wv0_mem_sync_arbiter_grant;
reg [3:0] wv0_mem_sync_arbiter_ready;


mem_sync wv0_mem_sync
(
	.clk(clk), .nRST(nRST),
	//WV
	.mem_instr(wv0_mem_sync_mem_instr),
	.mem_valid(wv0_mem_sync_mem_valid),
	.mem_ready(wv0_mem_sync_mem_ready),
	.mem_addr(wv0_mem_sync_mem_addr),
	.wstrb(wv0_mem_sync_wstrb),
	.data_in(wv0_mem_sync_data_in),
	.data_out(wv0_mem_sync_data_out),
	//MEM
	.arbiter_addr(wv0_mem_sync_arbiter_addr),
	.arbiter_data_out(wv0_mem_sync_arbiter_data_out),
	.arbiter_data_in(wv0_mem_sync_arbiter_data_in),
	.arbiter_wr_en(wv0_mem_sync_arbiter_wr_en),
	.arbiter_rd_en(wv0_mem_sync_arbiter_rd_en),
	.arbiter_byte_en(wv0_mem_sync_arbiter_byte_en),
	.arbiter_request(wv0_mem_sync_arbiter_request),
	.arbiter_grant(wv0_mem_sync_arbiter_grant),
	.arbiter_ready(wv0_mem_sync_arbiter_ready)
);


reg [7:0] fb_wr_en;
reg [7:0] fb_rd_en;
reg [255:0] fb_byte_en;
reg [255:0] fb_data_in;
reg [87:0] fb_mem_addr;
reg [7:0] fb_request;
wire [7:0] fb_grant;
wire [7:0] fb_ready;
wire [31:0] fb_data_out;

debug_fb fb 
(
	.clk(clk), .nRST(nRST),
	.wr_en(fb_wr_en), .rd_en(fb_rd_en),
	.addr(fb_mem_addr),
	.data_out(fb_data_out), .data_in(fb_data_in),
	.byte_en(fb_byte_en), .grant(fb_grant),
	.request(fb_request), .ready(fb_ready)
	// FW mode stuff
	
);// Mapped from

wire [3:0] wave_mem_instr;
wire [3:0] wave_mem_valid;
reg [3:0] wave_mem_ready;
wire [127:0] wave_mem_addr;
wire [15:0] wave_wstrb;
reg [127:0] wave_data_in;
wire [127:0] wave_data_out;
wavefront wv0
(
	.clk(clk), .nRST(nRST),
	.mem_addr(wave_mem_addr),
	.data_in(wave_data_in), .data_out(wave_data_out),
	.wstrb(wave_wstrb), .mem_ready(wave_mem_ready),
	.mem_valid(wave_mem_valid), .mem_instr(wave_mem_instr),
	//FW intfc
	.nFW_mode(nFW_mode), .fw_core_select(fw_core_select),
	.fw_wave_select(fw_wave_select), .fw_mem_addr(fw_mem_addr),
	.fw_data_in(fw_data_in), .fw_data_out(fw_data_out),
	.fw_byte_en(fw_byte_en), .fw_rd_en(fw_rd_en),
	.fw_wr_en(fw_wr_en)
);

reg [3:0] access_fb /* verilator lint_off UNOPTFLAT */;
// Wavefront - FB connection

//always @*begin
//	if(access_fb) begin
//		fb_byte_en = wave_wstrb;
//		fb_data_in = wave_data_out;
//		wave_data_in = fb_data_out;
//	end else begin
//		fb_byte_en = 0;
//		fb_data_in = 0;
//		wave_data_in = 0;
//	end
//end

always @*begin
	fb_byte_en = wv0_mem_sync_arbiter_byte_en;
	fb_data_in = wv0_mem_sync_arbiter_data_in;
	wv0_mem_sync_arbiter_data_out = fb_data_out;
	wv0_mem_sync_arbiter_grant = fb_grant;
	if(access_fb) begin
		wave_data_in = wv0_mem_sync_data_in;
		wv0_mem_sync_wstrb = wave_wstrb;
		wv0_mem_sync_data_out = wave_data_out;
		//wv0_mem_sync_mem_addr = (wave_mem_addr - 32'h3000);
		//wv0_mem_sync_mem_addr = wv0_mem_sync_mem_addr[12:2];
		wv0_mem_sync_mem_valid = wave_mem_valid;
		wv0_mem_sync_mem_instr = wave_mem_instr;
		wave_mem_ready = wv0_mem_sync_mem_ready;
	end else begin
		wave_data_in = 0;
		wv0_mem_sync_wstrb = 0;
		wv0_mem_sync_data_out = 0;
		//wv0_mem_sync_mem_addr = 0;
		wv0_mem_sync_mem_valid = 0;
		wv0_mem_sync_mem_instr = 0;
		wave_mem_ready = 0;
	end
end

generate
genvar i;
for(i = 0; i < 4; i = i + 1) begin : blk_control_signals
	always @* begin
		access_fb[i] = (wave_mem_addr[(i + 1) * 32 - 1 -: 32] >= 32'h3000) && ( wave_mem_addr[(i + 1) * 32 - 1 -: 32] < 32'h5000);
		fb_wr_en[i] = wv0_mem_sync_arbiter_wr_en[i] && access_fb[i];
		fb_rd_en[i] = wv0_mem_sync_arbiter_rd_en[i] && access_fb[i];
		fb_request[i] = wv0_mem_sync_arbiter_request[i] && access_fb[i];
		wv0_mem_sync_arbiter_ready[i] = fb_ready[i];
		fb_mem_addr[(i + 1) * 11 - 1 -: 11 ] = wv0_mem_sync_arbiter_addr[(i + 1) * 11 - 1 -: 11 ];
		if(access_fb) begin
			wv0_mem_sync_mem_addr[i * 32 +: 32] = (wave_mem_addr[i*32 +: 32] - 32'h3000);
			wv0_mem_sync_mem_addr[i * 32 +: 32] = wv0_mem_sync_mem_addr[i * 32 + 2 +: 30] ;
		end else begin
			wv0_mem_sync_mem_addr[i * 32 +: 32] = 0;
		end
		if(!nFW_mode) begin
			fb_wr_en[i] = 0;
			fb_rd_en[i] = 0;
			fb_request[i] = 0;
			fb_mem_addr[(i + 1) * 11 - 1 -: 11 ] = 0;
		end
	end

end
endgenerate

endmodule

module picorv_learn
(
	input inclk, input nRST,
	output reg [7:0] leds, output uart_tx_pin,
	input uart_rx_pin
);


wire clk;
pll_100 pll0(.inclk0(inclk), .c0(clk));
//assign clk = inclk;
reg urx_start_read;
wire urx_read_avl;
wire[7:0] urx_read_data;
wire urx_busy;
uart_rx urx(
	.clk(clk), .n_reset(nRST),
	.rx_pin(uart_rx_pin), .start_read(urx_start_read),
	.read_avl(urx_read_avl), .read_data(urx_read_data),
	.busy(urx_busy)
);

reg utx_start_write;
reg [7:0] utx_write_data;
wire utx_write_avl;
uart_tx utx(
	.clk(clk), .n_reset(nRST),
	.tx_pin(uart_tx_pin), .start_write(utx_start_write),
	.write_avl(utx_write_avl), .write_data(utx_write_data)
);

reg [15:0] read_counter;
reg [15:0] write_counter;
reg [31:0] word;
reg [2:0] word_counter;
reg [63:0] cycle_counter;

localparam STATE_START_SYNC_READ = 4'd0;
localparam STATE_START_READ = 4'd1; // Read from UART
localparam STATE_READ_BYTE = 4'd2;
localparam STATE_STORE_WORD = 4'd3;
localparam STATE_START_WRITE = 4'd4; // Write to UART
localparam STATE_LOAD_WORD = 4'd5;
localparam STATE_WRITE_BYTE = 4'd6;
localparam STATE_CHECK_SYNC_READ = 4'hf;
localparam STATE_RUN_PROGRAM = 4'd8;
localparam STATE_HANG = 4'd9;
localparam STATE_READ_DELAY = 4'd10;
reg [3:0] state;
always @* begin
	//leds[6] = !urx_busy;
	//leds[5] = urx_read_avl;
	//leds[4] = urx_start_read;
	leds[3:0] = state;
//	leds = read_counter[15:8];
end
/* Heartbeat */
//reg [25:0] heartbeat_red;
//always @(posedge clk) begin
//	heartbeat_red <= heartbeat_red + 1;
//	if(heartbeat_red[25])
//		leds[7] <= ~leds[7];
//	if(!nRST) begin
//		leds[7] <= 0;
//		heartbeat_red <= 0;
//	end
//end

/* Load FW */
always @(posedge clk) begin
	if(!nRST) begin
		state <= STATE_START_SYNC_READ;
		read_counter <= 0;
		word <= 0;
		word_counter <= 0;
		write_counter <= 0;
		cycle_counter <= 0;
		nFW_mode <= 0;
		fw_core_select <= 0;
		fw_mem_addr <= 0;
		fw_rd_en <= 0;
		fw_wr_en <= 0;
		fw_byte_en <= 0;
		fw_data_in <= 0;
		fw_wave_select <= 0;
	end else begin
	case(state)
		STATE_START_SYNC_READ: begin
			if(!urx_busy) begin
				urx_start_read <= 1;
				state <= STATE_CHECK_SYNC_READ;
			end
		end
		STATE_CHECK_SYNC_READ: begin
			urx_start_read <= 0;
			if(urx_read_avl) begin /* Read has completed */
				if(urx_read_data == 8'h53) begin /* Sync byte received */
					state <= STATE_START_READ; /* Start loading fw */
				end else begin
					state <= STATE_START_SYNC_READ; /* Read again */
				end
			end
		end
		STATE_START_READ: begin
			fw_wr_en <= 0;
			if(!urx_busy) begin
				urx_start_read <= 1;
				state <= STATE_READ_DELAY;
			end
		end
		STATE_READ_DELAY: begin
			state <= STATE_READ_BYTE;
		end
		STATE_READ_BYTE: begin
			nFW_mode <= 0;
			urx_start_read <= 0;
			if(urx_read_avl) begin
				word <= {urx_read_data, word[31:8]};
				if(word_counter < 3) begin
					word_counter <= word_counter + 1;
					state <= STATE_START_READ;
				end else begin
					word_counter <= 0;
					state <= STATE_STORE_WORD;
				end
			end
		end
		STATE_STORE_WORD: begin
			if(read_counter < 16'h1400) begin
				if(read_counter < 1024) begin /* Instructions */
					fw_wave_select <= 1;
					fw_core_select <= 0;
					fw_mem_addr <= read_counter;
				end else if (read_counter < 2048) begin /* Private data Core 0 */
					fw_wave_select <= 0;
					fw_core_select <= 0;
					fw_mem_addr <= read_counter - 1024;
				end else if (read_counter < 3072) begin /* Private data Core 1 */
					fw_wave_select <= 0;
					fw_core_select <= 1;
					fw_mem_addr <= read_counter - 2048;
				end else if (read_counter < 4096) begin /* Private data Core 2 */
					fw_wave_select <= 0;
					fw_core_select <= 2;
					fw_mem_addr <= read_counter - 3072;
				end else begin /* Private data Core 3 */
					fw_wave_select <= 0;
					fw_core_select <= 3;
					fw_mem_addr <= read_counter - 4096;
				end
				//fw_mem_addr <= read_counter;
				fw_data_in <= word;
				fw_byte_en <= 4'b1111;
				fw_wr_en <= 1;
				read_counter <= read_counter + 1;
				
				if(read_counter < 16'h13ff)
					state <= STATE_START_READ;
			end else begin
				fw_wr_en <= 0;
				read_counter <= 0;
				cycle_counter <= 0;
				state <= STATE_RUN_PROGRAM;
			end
		end
		STATE_RUN_PROGRAM: begin
			nFW_mode <= 1;
//			if(cycle_counter == 64'h3b9aca00) begin
			if(cycle_counter == 64'd50000000) begin
				cycle_counter <= 0;
				write_counter <= 0;
				word_counter <= 0;
				state <= STATE_LOAD_WORD;
			end else begin
				cycle_counter <= cycle_counter + 1;
			end
		end
		STATE_LOAD_WORD: begin
			if(write_counter >= 2048) begin // All data has been sent out
				state <= STATE_HANG;
			end else if(fb_ready) begin // Data is ready now
				word <= fb_data_out;
				write_counter <= write_counter + 1;
				state <= STATE_START_WRITE;
			end
		end

		STATE_HANG: begin
			state <= STATE_HANG;
		end
		STATE_START_WRITE: begin
			utx_start_write <= 0;
			if(word_counter >= 4) begin
				word_counter <= 0;
				state <= STATE_LOAD_WORD;
			end else if(utx_write_avl) begin
				utx_write_data <= word[word_counter*8 +: 8];
				utx_start_write <= 1;
				word_counter <= word_counter + 1;
			end	 
		end
		default: begin
		end
	endcase
end
end




reg [3:0] wv0_mem_sync_mem_instr;
reg [3:0] wv0_mem_sync_mem_valid;
wire [3:0] wv0_mem_sync_mem_ready;
reg [127:0] wv0_mem_sync_mem_addr;
reg [15:0] wv0_mem_sync_wstrb;
wire [127:0] wv0_mem_sync_data_in;
reg [127:0] wv0_mem_sync_data_out;

wire [43:0] wv0_mem_sync_arbiter_addr;
reg [127:0] wv0_mem_sync_arbiter_data_out;
wire [127:0] wv0_mem_sync_arbiter_data_in;
wire [3:0] wv0_mem_sync_arbiter_wr_en;
wire [3:0] wv0_mem_sync_arbiter_rd_en;
wire [15:0] wv0_mem_sync_arbiter_byte_en;
wire [3:0] wv0_mem_sync_arbiter_request;
reg [3:0] wv0_mem_sync_arbiter_grant;
reg [3:0] wv0_mem_sync_arbiter_ready;


mem_sync wv0_mem_sync
(
	.clk(clk), .nRST(nRST),
	//WV
	.mem_instr(wv0_mem_sync_mem_instr),
	.mem_valid(wv0_mem_sync_mem_valid),
	.mem_ready(wv0_mem_sync_mem_ready),
	.mem_addr(wv0_mem_sync_mem_addr),
	.wstrb(wv0_mem_sync_wstrb),
	.data_in(wv0_mem_sync_data_in),
	.data_out(wv0_mem_sync_data_out),
	//MEM
	.arbiter_addr(wv0_mem_sync_arbiter_addr),
	.arbiter_data_out(wv0_mem_sync_arbiter_data_out),
	.arbiter_data_in(wv0_mem_sync_arbiter_data_in),
	.arbiter_wr_en(wv0_mem_sync_arbiter_wr_en),
	.arbiter_rd_en(wv0_mem_sync_arbiter_rd_en),
	.arbiter_byte_en(wv0_mem_sync_arbiter_byte_en),
	.arbiter_request(wv0_mem_sync_arbiter_request),
	.arbiter_grant(wv0_mem_sync_arbiter_grant),
	.arbiter_ready(wv0_mem_sync_arbiter_ready)
);


reg [7:0] fb_wr_en;
reg [7:0] fb_rd_en;
reg [255:0] fb_byte_en;
reg [255:0] fb_data_in;
reg [87:0] fb_mem_addr;
reg [7:0] fb_request;
wire [7:0] fb_grant;
wire [7:0] fb_ready;
wire [31:0] fb_data_out;

debug_fb fb 
(
	.clk(clk), .nRST(nRST),
	.wr_en(fb_wr_en), .rd_en(fb_rd_en),
	.addr(fb_mem_addr),
	.data_out(fb_data_out), .data_in(fb_data_in),
	.byte_en(fb_byte_en), .grant(fb_grant),
	.request(fb_request), .ready(fb_ready)
);// Mapped from

reg nFW_mode;
reg [3:0] fw_core_select;
reg [31:0] fw_mem_addr;
wire [31:0] fw_data_out;
reg fw_rd_en;
reg fw_wr_en;
reg [3:0] fw_byte_en;
reg [31:0] fw_data_in;
reg fw_wave_select;
wire [3:0] wave_mem_instr;
wire [3:0] wave_mem_valid;
reg [3:0] wave_mem_ready;
wire [127:0] wave_mem_addr;
wire [15:0] wave_wstrb;
reg [127:0] wave_data_in;
wire [127:0] wave_data_out;
wavefront wv0
(
	.clk(clk), .nRST(nRST),
	.mem_addr(wave_mem_addr),
	.data_in(wave_data_in), .data_out(wave_data_out),
	.wstrb(wave_wstrb), .mem_ready(wave_mem_ready),
	.mem_valid(wave_mem_valid), .mem_instr(wave_mem_instr),
	//FW intfc
	.nFW_mode(nFW_mode), .fw_core_select(fw_core_select),
	.fw_wave_select(fw_wave_select), .fw_mem_addr(fw_mem_addr),
	.fw_data_in(fw_data_in), .fw_data_out(fw_data_out),
	.fw_byte_en(fw_byte_en), .fw_rd_en(fw_rd_en),
	.fw_wr_en(fw_wr_en)
);

reg [3:0] access_fb /* verilator lint_off UNOPTFLAT */;

always @*begin
	fb_byte_en = 0;
	fb_data_in = 0;
	wv0_mem_sync_arbiter_data_out =0;
	wv0_mem_sync_arbiter_grant =0;
	wave_data_in = 0;
	wv0_mem_sync_wstrb = 0;
	wv0_mem_sync_data_out = 0;
	wv0_mem_sync_mem_valid = 0;
	wv0_mem_sync_mem_instr = 0;
	wave_mem_ready = 0;
	if(state == STATE_RUN_PROGRAM) begin
		fb_byte_en = wv0_mem_sync_arbiter_byte_en;
		fb_data_in = wv0_mem_sync_arbiter_data_in;
		wv0_mem_sync_arbiter_data_out = fb_data_out;
		wv0_mem_sync_arbiter_grant = fb_grant;
		if(access_fb) begin
			wave_data_in = wv0_mem_sync_data_in;
			wv0_mem_sync_wstrb = wave_wstrb;
			wv0_mem_sync_data_out = wave_data_out;
			wv0_mem_sync_mem_valid = wave_mem_valid;
			wv0_mem_sync_mem_instr = wave_mem_instr;
			wave_mem_ready = wv0_mem_sync_mem_ready;
		end else begin
			wave_data_in = 0;
			wv0_mem_sync_wstrb = 0;
			wv0_mem_sync_data_out = 0;
			wv0_mem_sync_mem_valid = 0;
			wv0_mem_sync_mem_instr = 0;
			wave_mem_ready = 0;
		end
	end else if (state == STATE_LOAD_WORD) begin /* Connect FB to FSM */
		fb_byte_en = 0;
		fb_data_in = 0;
	end
end

generate
genvar i;
for(i = 0; i < 4; i = i + 1) begin : blk_control_signals
	always @* begin
		access_fb[i] = (wave_mem_addr[(i + 1) * 32 - 1 -: 32] >= 32'h3000) && ( wave_mem_addr[(i + 1) * 32 - 1 -: 32] < 32'h5000);
		fb_wr_en[i] = 0;
		fb_rd_en[i] = 0;
		fb_request[i] = 0;
		wv0_mem_sync_arbiter_ready[i] = 0;
		fb_mem_addr[(i + 1) * 11 - 1 -: 11 ] = 0;
		wv0_mem_sync_mem_addr[i * 32 +: 32] = 0;

		if(state == STATE_RUN_PROGRAM) begin
			fb_wr_en[i] = wv0_mem_sync_arbiter_wr_en[i] && access_fb[i];
			fb_rd_en[i] = wv0_mem_sync_arbiter_rd_en[i] && access_fb[i];
			fb_request[i] = wv0_mem_sync_arbiter_request[i] && access_fb[i];
			wv0_mem_sync_arbiter_ready[i] = fb_ready[i];
			fb_mem_addr[(i + 1) * 11 - 1 -: 11 ] = wv0_mem_sync_arbiter_addr[(i + 1) * 11 - 1 -: 11 ];
			if(access_fb) begin
				wv0_mem_sync_mem_addr[i * 32 +: 32] = (wave_mem_addr[i*32 +: 32] - 32'h3000);
				wv0_mem_sync_mem_addr[i * 32 +: 32] = wv0_mem_sync_mem_addr[i * 32 + 2 +: 30] ;
			end else begin
				wv0_mem_sync_mem_addr[i * 32 +: 32] = 0;
			end
			if(!nFW_mode) begin
				fb_wr_en[i] = 0;
				fb_rd_en[i] = 0;
				fb_request[i] = 0;
				fb_mem_addr[(i + 1) * 11 - 1 -: 11 ] = 0;
			end
		end else if (state == STATE_LOAD_WORD) begin /* Connect FB to FSM */
			fb_rd_en[i] = 1;
			fb_request[i] = 1;
			fb_mem_addr[(i + 1) * 11 - 1 -: 11 ] = write_counter;
		end
	end

end
endgenerate

endmodule

//module picorv_learn(input inclk, input nRST,
//output reg[7:0] leds, output uart_tx_pin, input uart_rx_pin, output reg[7:0] dbg_sram);
//wire clk;
//pll_100 pll0(.inclk0(inclk), .c0(clk));
//
//reg uart_start_read;
//wire uart_rd_avl;
//wire uart_rx_busy;
//wire [7:0] read_data;
//
//reg core_reset;
//wire[31:0] core0_addr_bus;
//wire[31:0] core0_wdata_bus;
//reg[31:0] core0_rdata_bus;
//wire[3:0] core0_wstrb; // Write strobe
//localparam WSTRB_READ = 4'd0;
//localparam WSTRB_BYTE0 = 4'd1;
//localparam WSTRB_BYTE1 = 4'd2;
//localparam WSTRB_BYTE2 = 4'd4;
//localparam WSTRB_BYTE3 = 4'd8;
//localparam WSTRB_LOWER_HWORD = 4'd3;
//localparam WSTRB_HIGHER_HWORD = 4'd12;
//localparam WSTRB_WORD = 4'd15;
//wire core0_mem_valid; // core asserts 1 when a request is placed on the bus
//reg core0_mem_ready; // peripheral asserts 1 when memory is ready to respond
//
//reg[7:0] state;
//reg[7:0] mem_loader_state;
//reg[2:0] mem_loader_counter;
//localparam STATE_IDLE = 8'b0;
//localparam STATE_LOAD_MEMORY = 8'd1;
//localparam STATE_TRANSMIT_MEMORY_DEBUG = 8'd3;
//localparam STATE_RUN_PROGRAM = 8'd2;
//localparam STATE_MEMLOADER_START = 8'd1;
//localparam STATE_MEMLOADER_LATCH = 8'd2;
//localparam STATE_MEMLOADER_WAIT = 8'd3;
//localparam STATE_MEMLOADER_WRITE = 8'd4;
//
//reg sram0_wr_en;
//reg sram0_rd_en;
//reg[9:0] sram0_addr_bus;
//reg[31:0] sram0_data_in;
//reg[3:0] sram0_data_be;
//wire[31:0] sram0_data_out;
//sram sram0(
//	.clk(clk), .rd_en(sram0_rd_en), .wr_en(sram0_wr_en),
//	.data_in(sram0_data_in), .data_out(sram0_data_out),
//	.address(sram0_addr_bus), .byte_en(sram0_data_be)
//); //1K*4B SRAM
//reg[9:0] sram_rd_ptr;
//reg[9:0] sram_wr_ptr;
//
//always @* begin
//	if(state == STATE_LOAD_MEMORY) begin
//		sram0_rd_en = 0;
//		case(mem_loader_counter)
//			0:
//				sram0_data_be = 4'b1;
//			1:
//				sram0_data_be = 4'b10;
//			2:
//				sram0_data_be = 4'b100;
//			3:
//				sram0_data_be = 4'b1000;
//			default: begin
//				sram0_data_be = 4'b1111;
//			end
//		endcase
//		sram0_addr_bus = sram_wr_ptr;
//		sram0_data_in = {4{read_data}}; // Since we use a mask, replicate the read byte
//		sram0_wr_en = uart_rd_avl & ~(uart_start_read); // Latch data in when output is valid
//		core0_rdata_bus = 0;
//	end
//	else if (state == STATE_RUN_PROGRAM) begin
//		sram0_rd_en = (core0_wstrb == WSTRB_READ) && core0_mem_valid;
//		sram0_data_be = core0_wstrb;
//		sram0_addr_bus = core0_addr_bus[11:2]; // Ignore the first two bits because SRAM has a 1024*4K struct
//		sram0_data_in = core0_wdata_bus;
//		sram0_wr_en = (core0_wstrb != 0) && core0_mem_valid;
//		core0_rdata_bus = sram0_data_out;
//	end
//	else if (state == STATE_TRANSMIT_MEMORY_DEBUG) begin
//		sram0_rd_en = 1;
//		sram0_addr_bus = sram_rd_ptr;
//		sram0_data_in = 0;
//		sram0_data_be = 4'b1111;
//		sram0_wr_en = 0;
//		core0_rdata_bus = 0;
//	end
//	else begin
//		sram0_addr_bus = 0;
//		sram0_data_in = 0;
//		sram0_data_be = 4'b1111;
//		sram0_wr_en = 0;
//		sram0_rd_en = 0;
//		core0_rdata_bus = 0;
//	end
//end
//
//
//
//uart_rx urx(.clk(clk), .n_reset(nRST), .rx_pin(uart_rx_pin), .start_read(uart_start_read),
//				.read_avl(uart_rd_avl), .read_data(read_data), .busy(uart_rx_busy));
//
//picorv32 core0(.clk(clk), .resetn(core_reset), .mem_valid(core0_mem_valid), .mem_ready(core0_mem_ready),
//					.mem_addr(core0_addr_bus), .mem_rdata(core0_rdata_bus), .mem_wdata(core0_wdata_bus),
//					.mem_wstrb(core0_wstrb));
//
//always @(posedge clk) begin
//	if(!nRST)begin
//		state <= STATE_IDLE;
//		mem_loader_state <= STATE_IDLE;
//		mem_loader_counter <= 0;
//		core_reset <= 0; // Keep all cores reset till data is available;
//		core0_mem_ready <= 0;
//		sram_rd_ptr <= 0;
//		sram_wr_ptr <= 0;
//		uart_start_read <= 0;
//		leds <= 0;
//	end
//	else begin
//		case(state)
//			STATE_IDLE: begin
//				// Reset all necessary signals
//				mem_loader_counter <= 0;
//				uart_start_read <= 0;
//				sram_rd_ptr <= 0;
//				sram_wr_ptr <= 0;
//				state <= STATE_LOAD_MEMORY;
//				leds[0] <= 1;
//			end
//			STATE_LOAD_MEMORY: begin
//				case(mem_loader_state)
//					STATE_IDLE: begin // Wait till data is given
//						if(uart_rd_avl) begin
//							uart_start_read <= 0;
//							if(read_data == "S") begin
//								mem_loader_state <= STATE_MEMLOADER_START;
//							end else begin
//								mem_loader_state <= STATE_IDLE;
//							end
//						end else if(!uart_rx_busy) begin
//							uart_start_read <= 1;
//						end else begin
//							uart_start_read <= 0;
//						end
//					end
//					STATE_MEMLOADER_START: begin // Wait till busy goes away. Then assert a read start
//						if(!uart_rx_busy) begin
//							uart_start_read <= 1;
//							mem_loader_state <= STATE_MEMLOADER_LATCH;
//						end else
//							uart_start_read <= 0;
//					end
//					STATE_MEMLOADER_LATCH:
//						mem_loader_state <= STATE_MEMLOADER_WAIT;
//					STATE_MEMLOADER_WAIT: begin
//						uart_start_read <= 0;
//						if(uart_rd_avl) begin // New data is available
//							if(mem_loader_counter == 3) begin
//								sram_wr_ptr <= sram_wr_ptr + 1;
//								mem_loader_counter <= 0;
//							end
//							else
//								mem_loader_counter <= mem_loader_counter + 1;
//							mem_loader_state <= STATE_MEMLOADER_START; // Return back to idle state
//							if(sram_wr_ptr == 10'd1023) begin // If all 1024 bytes have been loaded, start running the core
//								state <= STATE_TRANSMIT_MEMORY_DEBUG;
//								mem_loader_state <= STATE_IDLE;
//							end
//						end else begin
//						end
//					end
//					default: begin
//							mem_loader_state <= STATE_IDLE;
//							state <= STATE_IDLE;
//					end
//				endcase
//			end
//			STATE_TRANSMIT_MEMORY_DEBUG: begin
//				dbg_sram <= sram0_data_out;
//				sram_rd_ptr <= sram_rd_ptr + 1;
//				if(sram_rd_ptr == 10'd1023) begin
//					state <= STATE_RUN_PROGRAM;
//					sram_rd_ptr <= 0;
//				end
//			end
//			STATE_RUN_PROGRAM: begin
//				core_reset <= 1; // Release reset on cores
//				if(core0_mem_valid) begin // Memory request has been placed, gets reset when mem_ready is set
//					core0_mem_ready <= 1;
//				end else begin
//					core0_mem_ready <= 0; // If no request has been placed, deassert mem_ready
//				end
//			end
//			default:
//				state <= STATE_IDLE;
//		endcase
//	end
//end
//endmodule
