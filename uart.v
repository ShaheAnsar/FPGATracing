

module uart_tx#(parameter CLOCK=12000000, parameter BAUDRATE=9600)(output tx_pin,
						input clk, input[32:0] baud_ctr_top, input n_reset,
						input start_write, output reg write_avl, input [7:0] write_data);
localparam baudgen_top_val = CLOCK/BAUDRATE;
reg[32:0] baudgen_ctr;
reg[1:0] write_state; // Writer state
reg[9:0] write_reg; // Write shift register
reg[3:0] shifted_counter; // Used to keep track of how many we have shifted out
reg n_out_en = 0; // Used to enable the output. Set to 0 to enable the output
wire baud_clock = (baudgen_top_val == baudgen_ctr);
localparam idle_state = 2'd0; // Common to both read and write
localparam read_busy = 2'd1; // Command has been given to start reading data
localparam read_fin_state = 2'd2; // Read finished, data available in reg
localparam write_busy = 2'd1; // Writing is in progress

assign tx_pin = write_reg[0] | n_out_en; // If write state is not write_busy, tx_pin should always be high
always @(posedge clk) begin
	if(!n_reset) begin
		baudgen_ctr <= 0;
		shifted_counter <= 0;
		write_reg <= 0;
		write_avl <= 1;
		write_state <= idle_state;
		n_out_en <= 1; // Disable the output
	end else begin
		baudgen_ctr <= baudgen_ctr + 1;
		if(baudgen_ctr == baudgen_top_val)
			baudgen_ctr <= 0;
//		case(read_state)
//			idle_state:
//		endcase
		
		case(write_state)
			idle_state: begin
			 if(start_write) begin
				write_reg <= {1'b1, write_data, 1'b0};
				write_state <= write_busy;
				write_avl <= 0;
				shifted_counter <= 0;
			 end else begin
				write_avl <= 1;
			end
			end
			write_busy: begin //shift out data
				if(shifted_counter == 4'd10) begin
					write_state <= idle_state;
					write_avl <= 1;
					n_out_en <= 1;
				end
				if(n_out_en == 1'b1) begin
					if(baud_clock) n_out_en <= 0;
				end
				else if(baud_clock) begin
				write_reg <= {1'b1, write_reg[9:1]};
				shifted_counter <= shifted_counter + 1;
				end
			end
			default: begin
			// Do nothing
			end
		endcase
	end
end					
endmodule

module uart_rx#(parameter CLOCK=12000000, parameter BAUDRATE=9600)
(input rx_pin, input clk, input start_read, output reg read_avl, output reg busy, input n_reset, output[7:0] read_data, output[1:0] dbg_leds);
localparam baudgen_top = CLOCK/BAUDRATE - 1;
localparam oversample_top = CLOCK/BAUDRATE/8 - 1;
reg[32:0] baudgen_ctr;
wire baud_ov_clock = (baudgen_ctr == oversample_top);
//FSM
localparam state_idle = 4'd0;
localparam state_waiting_for_start = 4'd1;
localparam state_shifting_data = 4'd2;
localparam state_data_in_reg = 4'd3;
reg[3:0] read_state;
//CDC sync
reg[1:0] rx_sync;
//RX Window for start detection
reg[2:0] rx_win;
//Add all oversampled bits to this reg. If this is greater than 4, we have a 1, otherwise we have a 0
reg[3:0] rx_bit;
wire bit_val = (rx_bit > 4);
//Keeps track of how many oversampled bits we have processed
reg[3:0] rx_bit_ctr;
//Keeps track of how many actual bits we have processed
reg[3:0] rx_bit_ctr2;
//Debug shit
reg reset_happened = 0;
always @(posedge clk) begin
	if(!n_reset) begin
	read_state <= state_idle;
	reset_happened <= 1;
	baudgen_ctr <= 0;
	read_data <= 0;
	read_avl <= 0;
	rx_sync <= 2'b11;
	rx_win <= 3'b111;
	rx_bit <= 0;
	rx_bit_ctr <= 0;
	rx_bit_ctr2 <= 0;
	busy <= 0;
	end
	else begin
		//Baudrate stuff
		if(baudgen_ctr == oversample_top) baudgen_ctr <= 0;
		else baudgen_ctr <= baudgen_ctr + 1;
		//RX Sync
		if(baud_ov_clock) rx_sync <= {rx_pin, rx_sync[1]};
		//RX Win
		if(baud_ov_clock) rx_win <= {rx_sync[0], rx_win[2:1]}; // This will be all 0s if a start bit is detected
		//FSM
		case(read_state)
			state_idle: begin
				// start_read is asserted when we want to receive data
				if(start_read) begin
					read_state <= state_waiting_for_start;
					read_avl <= 0; // Deny read_avl till shifting is done
					read_data <= 0; // Reset read_data
					busy <= 1; // Signal that the unit is busy
				end
			end
			state_waiting_for_start: begin
				if(rx_win == 0) begin // Found start bit
					read_state <= state_shifting_data;
//					rx_bit <= rx_bit + rx_win[0];
//					rx_bit_ctr <= rx_bit_ctr + 1;
				end 
			end
			state_shifting_data: begin
				if(rx_bit_ctr2 < 9) begin // Acquite 9 bits since the start bit is included too. Since the older bits get discarded, it works out
					if(rx_bit_ctr == 8) begin
						read_data <= {bit_val, read_data[7:1]};
						rx_bit_ctr <= 0;
						rx_bit <= 0; // Reset our bit comparison
						rx_bit_ctr2 <= rx_bit_ctr2 + 1;
					end else if(baud_ov_clock) begin
						rx_bit <= rx_bit + rx_win[0];
						rx_bit_ctr <= rx_bit_ctr + 1;
					end
				end else begin
					// Clean up and move to reg
					read_state <= state_idle;
					read_avl <= 1;
					rx_bit <= 0;
					rx_bit_ctr <= 0;
					rx_bit_ctr2 <= 0;
					busy <= 0; // Signal that the unit is available again
					rx_win <= 3'b111; // Reset the start bit detector
				end
			end
		endcase
	end
end
assign dbg_leds[0] = reset_happened;
assign dbg_leds[1] = (read_state == state_idle);

endmodule
