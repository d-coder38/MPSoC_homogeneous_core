module gpio #(parameter WIDTH = 32) (
    input wire clk,
    input wire rst,
    input wire [WIDTH-1:0] gpio_in,   // Input signals from external devices
    output reg [WIDTH-1:0] gpio_out,  // Output signals to external devices
    input wire [WIDTH-1:0] dir,       // Direction control: 1 = output, 0 = input
    input wire [WIDTH-1:0] write_data,// Data to be written to GPIO
    input wire write_enable           // Write enable signal
);

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            gpio_out <= 0;
        end else begin
            if (write_enable) begin
                gpio_out <= (dir & write_data) | (~dir & gpio_out); // Update only output pins
            end
        end
    end

endmodule
