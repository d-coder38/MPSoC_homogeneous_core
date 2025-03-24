module spi_master (
    input wire clk,
    input wire rst,
    input wire start,
    input wire [31:0] data_in,
    output reg spi_clk,
    output reg spi_mosi,
    input wire spi_miso,
    output reg spi_cs,
    output reg done,
    output reg [31:0] data_out
);
    
    parameter CLK_DIV = 4;  // Clock divider for SPI speed control
    reg [5:0] bit_count;
    reg [31:0] shift_reg;
    reg [3:0] clk_counter;
    reg active;
    
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            spi_clk <= 1'b0;
            spi_mosi <= 1'b0;
            spi_cs <= 1'b1;
            done <= 1'b0;
            bit_count <= 0;
            shift_reg <= 0;
            clk_counter <= 0;
            active <= 0;
        end else begin
            if (start && !active) begin
                active <= 1;
                spi_cs <= 0;
                shift_reg <= data_in;
                bit_count <= 0;
                done <= 0;
            end 
            
            if (active) begin
                if (clk_counter < CLK_DIV) begin
                    clk_counter <= clk_counter + 1;
                end else begin
                    clk_counter <= 0;
                    spi_clk <= ~spi_clk;
                    
                    if (spi_clk) begin // Rising edge: Shift out data
                        spi_mosi <= shift_reg[31];
                        shift_reg <= {shift_reg[30:0], spi_miso};
                        bit_count <= bit_count + 1;
                    end
                    
                    if (bit_count == 32) begin
                        active <= 0;
                        spi_cs <= 1;
                        data_out <= shift_reg;
                        done <= 1;
                    end
                end
            end
        end
    end
endmodule
