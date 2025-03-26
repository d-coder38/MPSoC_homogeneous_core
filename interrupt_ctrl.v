module interrupt_ctrl #(
    parameter NUM_IRQS = 16  // Number of interrupt sources
)(
    input  wire                   clk,       // System clock
    input  wire                   rst_n,     // Active-low reset
    input  wire [NUM_IRQS-1:0]     irq,       // Interrupt request lines
    input  wire [NUM_IRQS-1:0]     mask,      // Interrupt mask register (1 = disabled)
    input  wire                   ack,       // CPU acknowledges interrupt
    output reg                    int_req,   // Interrupt request to CPU
    output reg [$clog2(NUM_IRQS)-1:0] int_id, // Interrupt ID
    output reg [7:0]               int_vector // Interrupt vector (ISR address)
);

    reg [NUM_IRQS-1:0] pending;   // Stores pending interrupts
    reg [$clog2(NUM_IRQS)-1:0] priority;  // Holds highest priority interrupt ID
    reg servicing; // Indicates if an interrupt is being serviced

    // Interrupt Vector Table (example ISR addresses)
    reg [7:0] vector_table [NUM_IRQS-1:0];

    integer i;

    // Initialize interrupt vector table
    initial begin
        for (i = 0; i < NUM_IRQS; i = i + 1) begin
            vector_table[i] = i * 8; // Example vector offsets (modify as needed)
        end
    end

    always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        pending    <= {NUM_IRQS{1'b0}};
        int_req    <= 1'b0;
        servicing  <= 1'b0;
    end else begin
        // Update pending interrupts (only unmasked IRQs)
        pending <= (irq & ~mask) | (pending & ~ack);

        // Determine highest priority interrupt (Fixed - No `break`)
        priority = NUM_IRQS - 1; // Default: no interrupt
        for (i = 0; i < NUM_IRQS; i = i + 1) begin
            if (pending[i] && priority == (NUM_IRQS - 1)) begin
                priority = i;
            end
        end

        // Generate interrupt request to CPU
        if (priority != (NUM_IRQS - 1) && !servicing) begin
            int_req    <= 1'b1;
            int_id     <= priority;
            int_vector <= vector_table[priority];
            servicing  <= 1'b1;
        end

        // CPU acknowledges the interrupt
        if (ack) begin
            pending[priority] <= 1'b0;
            int_req           <= 1'b0;
            servicing         <= 1'b0;
        end
    end
end

endmodule
