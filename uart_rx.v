module uart_rx(
    input wire clk,
    input wire rst,
    input wire rx,
    output reg rx_done,
    output reg [31:0] rx_data
);
    
    parameter BAUD_RATE = 9600;
    parameter CLK_FREQ = 50000000;
    parameter BIT_PERIOD = CLK_FREQ / BAUD_RATE;
    
    reg [5:0] bit_index;
    reg [33:0] shift_reg;
    reg [31:0] baud_counter;
    reg receiving;
    
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            rx_done <= 0;
            receiving <= 0;
            baud_counter <= 0;
            bit_index <= 0;
        end else begin
            if (!receiving && !rx) begin
                receiving <= 1;
                baud_counter <= BIT_PERIOD / 2;
                bit_index <= 0;
            end else if (receiving) begin
                if (baud_counter < BIT_PERIOD) begin
                    baud_counter <= baud_counter + 1;
                end else begin
                    baud_counter <= 0;
                    shift_reg <= {rx, shift_reg[33:1]};
                    bit_index <= bit_index + 1;
                    if (bit_index == 32) begin
                        rx_data <= shift_reg[31:0];
                        rx_done <= 1;
                        receiving <= 0;
                    end
                end
            end
        end
    end
endmodule
