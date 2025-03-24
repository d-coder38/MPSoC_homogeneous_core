module uart_tx(
    input wire clk,
    input wire rst,
    input wire tx_start,
    input wire [31:0] tx_data,
    output reg tx,
    output reg tx_busy
);
    
    parameter BAUD_RATE = 9600;
    parameter CLK_FREQ = 50000000;
    parameter BIT_PERIOD = CLK_FREQ / BAUD_RATE;
    
    reg [5:0] bit_index;
    reg [33:0] shift_reg;
    reg [31:0] baud_counter;
    reg transmitting;
    
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            tx <= 1'b1;
            tx_busy <= 1'b0;
            transmitting <= 1'b0;
            baud_counter <= 0;
            bit_index <= 0;
        end else begin
            if (tx_start && !transmitting) begin
                shift_reg <= {1'b1, tx_data, 1'b0}; // Start bit (0), 32 data bits, Stop bit (1)
                transmitting <= 1'b1;
                tx_busy <= 1'b1;
                bit_index <= 0;
                baud_counter <= 0;
            end else if (transmitting) begin
                if (baud_counter < BIT_PERIOD) begin
                    baud_counter <= baud_counter + 1;
                end else begin
                    baud_counter <= 0;
                    tx <= shift_reg[0];
                    shift_reg <= shift_reg >> 1;
                    bit_index <= bit_index + 1;
                    if (bit_index == 33) begin
                        transmitting <= 1'b0;
                        tx_busy <= 1'b0;
                    end
                end
            end
        end
    end
endmodule
