module sram #(parameter BYTES=4, parameter DEPTH=1024)
(
	input clk, input wr_en, input rd_en,
	input[$clog2(DEPTH) - 1 : 0] address,
	output reg[BYTES*8 - 1:0] data_out,
	input [BYTES*8 - 1:0] data_in,
	input [BYTES - 1:0] byte_en
);

logic [BYTES-1:0][7:0] memory[DEPTH] /* verilator public_flat */;

always_ff@(posedge clk) begin
	if(wr_en) begin
		if(byte_en[0])
			memory[address][0] <= data_in[7:0];
		if(byte_en[1])
			memory[address][1] <= data_in[15:8];
		if(byte_en[2])
			memory[address][2] <= data_in[23:16];
		if(byte_en[3])
			memory[address][3] <= data_in[31:24];
	end
	if(rd_en)
		data_out <= memory[address];
end

endmodule
