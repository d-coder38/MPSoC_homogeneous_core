module spi_slave (
    input wire clk,
    input wire rst,
    input wire spi_clk,
    input wire spi_mosi,
    output reg spi_miso,
    input wire spi_cs,
    output reg [31:0] data_out,
    input wire [31:0] data_in
);
    
    reg [5:0] bit_count;
    reg [31:0] shift_reg;
    
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            bit_count <= 0;
            shift_reg <= data_in;
            data_out <= 0;
        end else if (!spi_cs) begin
            if (spi_clk) begin // Capture data on rising edge
                shift_reg <= {shift_reg[30:0], spi_mosi};
                bit_count <= bit_count + 1;
            end else begin // Shift out data on falling edge
                spi_miso <= shift_reg[31];
                shift_reg <= {shift_reg[30:0], 1'b0};
            end
            if (bit_count == 32) begin
                data_out <= shift_reg;
            end
        end
    end
endmodule
