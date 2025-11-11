module picorv_learn(input clk, input nRST,
output reg[7:0] leds, output uart_tx_pin, input uart_rx_pin, output reg[7:0] dbg_sram);


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
	end
	else begin
		sram0_addr_bus = 0;
		sram0_data_in = 0;
		sram0_data_be = 4'b1111;
		sram0_wr_en = 0;
		sram0_rd_en = 0;
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
								state <= STATE_RUN_PROGRAM;
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
