module timer_counter (
    input wire clk,           // System clock
    input wire rst_n,         // Active-low reset
    input wire pclk,          // APB clock
    input wire penable,       // APB enable signal
    input wire pwrite,        // APB write enable
    input wire [3:0]  paddr,         // APB address
    input wire [31:0] pwdata,        // APB write data
    output reg [31:0] prdata,        // APB read data
    input wire enable,        // Timer enable
    output reg irq,           // Interrupt request
    output reg pwm_out        // PWM output
);

    // Register map
    reg [31:0] counter;             // Counter register
    reg [31:0] compare;             // Compare register
    reg [31:0] reload_value;        // Reload register (for auto-reload)
    reg [15:0] prescaler;           // Prescaler register
    reg auto_reload;         // Auto-reload enable

    reg [15:0] prescaler_counter;   // Prescaler counter

    // APB register read/write handling
    always @(posedge pclk or negedge rst_n) begin
        if (!rst_n) begin
            compare       <= 32'd1000; // Default compare value
            reload_value  <= 32'd1000; // Default reload value
            prescaler     <= 16'd1;    // Default prescaler value
            auto_reload   <= 1'b0;
        end else if (penable && pwrite) begin
            case (paddr)
                4'h4: compare       <= pwdata;
                4'h8: reload_value  <= pwdata;
                4'hC: prescaler     <= pwdata[15:0];
                4'h10: auto_reload  <= pwdata[0];
            endcase
        end
    end

    always @(*) begin
        case (paddr)
            4'h0: prdata = counter;
            4'h4: prdata = compare;
            4'h8: prdata = reload_value;
            4'hC: prdata = {16'b0, prescaler};
            4'h10: prdata = {31'b0, auto_reload};
            default: prdata = 32'b0;
        endcase
    end

    // Timer Counter Logic
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            counter            <= 32'd0;
            prescaler_counter  <= 16'd0;
            irq                <= 1'b0;
            pwm_out            <= 1'b0;
        end else if (enable) begin
            // Prescaler Logic
            if (prescaler_counter >= prescaler) begin
                prescaler_counter <= 16'd0; // Reset prescaler
                counter <= counter + 1; // Increment main counter
                
                // Compare match logic
                if (counter >= compare) begin
                    irq <= 1'b1; // Generate an interrupt
                    pwm_out <= ~pwm_out; // Toggle PWM output

                    if (auto_reload)
                        counter <= reload_value; // Reload counter if enabled
                    else
                        counter <= 32'd0;
                end else begin
                    irq <= 1'b0; // Clear interrupt flag
                end
            end else begin
                prescaler_counter <= prescaler_counter + 1;
            end
        end else begin
            counter <= 32'd0; // Reset counter when disabled
        end
    end

endmodule
