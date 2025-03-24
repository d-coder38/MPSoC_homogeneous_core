// AGB Master module
module AHB_Master (
    input  wire clk,
    input  wire rst_n,
    output reg [31:0] haddr,    // Address
    output reg [31:0] hwdata,   // Write Data
    input  wire [31:0] hrdata,  // Read Data
    output reg hwrite,          // Write Enable
    output reg htrans,          // Transfer Type
    output reg hready,          // Ready Signal
    input  wire hresp           // Response
);

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        haddr  <= 0;
        hwdata <= 0;
        hwrite <= 0;
        htrans <= 0;
        hready <= 0;
    end else begin
        haddr  <= 32'h40000000;  // Sample address
        hwdata <= 32'h12345678;  // Sample data
        hwrite <= 1;             // Write transaction
        htrans <= 1;             // Valid transfer
        hready <= 1;             // Ready to transfer
    end
end

endmodule

//AHB Slave module
module AHB_Slave (
    input  wire clk,
    input  wire rst_n,
    input  wire [31:0] haddr,
    input  wire [31:0] hwdata,
    output reg [31:0] hrdata,
    input  wire hwrite,
    input  wire htrans,
    input  wire hready,
    output reg hresp
);

reg [31:0] memory [0:255];

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        hresp  <= 0;
        hrdata <= 0;
    end else if (hready && htrans) begin
        if (hwrite) begin
            memory[haddr[7:0]] <= hwdata;  // Write to memory
            hresp <= 0;  // Success
        end else begin
            hrdata <= memory[haddr[7:0]];  // Read from memory
            hresp  <= 0;  // Success
        end
    end
end

endmodule

// AHB to APB bridge module
module AHB_to_APB_Bridge (
    input  wire clk,
    input  wire rst_n,
    
    // AHB Interface
    input  wire [31:0] haddr,
    input  wire [31:0] hwdata,
    output reg  [31:0] hrdata,
    input  wire hwrite,
    input  wire htrans,
    input  wire hready,
    output reg  hresp,
    
    // APB Interface
    output reg [31:0] paddr,
    output reg [31:0] pwdata,
    input  wire [31:0] prdata,
    output reg pwrite,
    output reg penable,
    output reg psel
);

reg [1:0] state;
parameter IDLE = 2'b00, SETUP = 2'b01, ACCESS = 2'b10;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        state  <= IDLE;
        psel   <= 0;
        penable <= 0;
        hresp  <= 0;
    end else begin
        case (state)
            IDLE: begin
                if (htrans && hready) begin
                    state  <= SETUP;
                    psel   <= 1;
                    paddr  <= haddr;
                    pwrite <= hwrite;
                    pwdata <= hwdata;
                end
            end
            SETUP: begin
                penable <= 1;
                state   <= ACCESS;
            end
            ACCESS: begin
                penable <= 0;
                state   <= IDLE;
                if (!pwrite)
                    hrdata <= prdata;  // Read data from APB
            end
        endcase
    end
end

endmodule

//APB slave module
module APB_Slave (
    input  wire clk,
    input  wire rst_n,
    input  wire psel,
    input  wire penable,
    input  wire [31:0] paddr,
    input  wire [31:0] pwdata,
    output reg [31:0] prdata,
    input  wire pwrite
);

reg [31:0] apb_memory [0:255];

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        prdata <= 0;
    end else if (psel && penable) begin
        if (pwrite) begin
            apb_memory[paddr[7:0]] <= pwdata; // Write to memory
        end else begin
            prdata <= apb_memory[paddr[7:0]]; // Read from memory
        end
    end
end

endmodule

//AMBA bus protocol
module AMBA (
    input wire clk,
    input wire rst_n
);

// AHB Master Signals
wire [31:0] haddr, hwdata, hrdata;
wire hwrite, htrans, hready, hresp;

// APB Bridge Signals
wire [31:0] paddr, pwdata, prdata;
wire pwrite, penable, psel;

// Instantiate AHB Master
AHB_Master ahb_master (
    .clk(clk), .rst_n(rst_n),
    .haddr(haddr), .hwdata(hwdata), .hrdata(hrdata),
    .hwrite(hwrite), .htrans(htrans), .hready(hready), .hresp(hresp)
);

// Instantiate AHB-to-APB Bridge
AHB_to_APB_Bridge bridge (
    .clk(clk), .rst_n(rst_n),
    .haddr(haddr), .hwdata(hwdata), .hrdata(hrdata),
    .hwrite(hwrite), .htrans(htrans), .hready(hready), .hresp(hresp),
    .paddr(paddr), .pwdata(pwdata), .prdata(prdata),
    .pwrite(pwrite), .penable(penable), .psel(psel)
);

// Instantiate APB Slave
APB_Slave apb_slave (
    .clk(clk), .rst_n(rst_n),
    .psel(psel), .penable(penable),
    .paddr(paddr), .pwdata(pwdata), .prdata(prdata),
    .pwrite(pwrite)
);

endmodule
