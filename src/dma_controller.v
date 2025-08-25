module dma #(
    parameter DATA_WIDTH = 32,
    parameter AXI_ADDR_WIDTH = 32
)(
    input  wire clk,
    input  wire rst_n,

    input  wire [3:0]  burst_size_i,
    input  wire        dma_dir_i,
    input  wire        incr_addr_i,
    input  wire [AXI_ADDR_WIDTH-1:0] dma_addr_i,
    input  wire [31:0] dma_len_i,

    input  wire dma_start_i,
    output wire dma_done_o,

    // RX FIFO Interface (for DMA read operations)
    input  wire [DATA_WIDTH-1:0] rx_data_dma,
    input  wire rx_empty,
    output wire rx_ren,

    // TX FIFO Interface (for DMA write operations) - NEWLY ADDED
    output wire tx_wen,
    output wire [DATA_WIDTH-1:0] tx_data_dma,
    input  wire tx_full,

    // AXI Master Write Address Channel
    output wire [3:0]  m_awid,
    output wire [AXI_ADDR_WIDTH-1:0] m_awaddr,
    output wire [7:0]  m_awlen,
    output wire [2:0]  m_awsize,
    output wire [1:0]  m_awburst,
    output wire        m_awvalid,
    input  wire        m_awready,

    // AXI Master Write Data Channel
    output wire [DATA_WIDTH-1:0] m_wdata,
    output wire [DATA_WIDTH/8-1:0] m_wstrb,
    output wire        m_wlast,
    output wire        m_wvalid,
    input  wire        m_wready,

    // AXI Master Write Response Channel
    input  wire [3:0]  m_bid,
    input  wire [1:0]  m_bresp,
    input  wire        m_bvalid,
    output wire        m_bready,

    // AXI Master Read Address Channel
    output wire [3:0]  m_arid,
    output wire [AXI_ADDR_WIDTH-1:0] m_araddr,
    output wire [7:0]  m_arlen,
    output wire [2:0]  m_arsize,
    output wire [1:0]  m_arburst,
    output wire        m_arvalid,
    input  wire        m_arready,

    // AXI Master Read Data Channel
    input  wire [3:0]  m_rid,
    input  wire [DATA_WIDTH-1:0] m_rdata,
    input  wire [1:0]  m_rresp,
    input  wire        m_rlast,
    input  wire        m_rvalid,
    output wire        m_rready
);

    reg [31:0] data_counter;
    reg [AXI_ADDR_WIDTH-1:0] current_addr;
    reg [31:0] bytes_remaining;
    reg [7:0]  burst_count;
    reg dma_active;
    reg dma_done_reg;

    reg awvalid_reg;
    reg wvalid_reg;
    reg wlast_reg;
    reg bready_reg;
    reg arvalid_reg;
    reg rready_reg;

    // TX FIFO interface registers - NEWLY ADDED
    reg tx_wen_reg;
    reg [DATA_WIDTH-1:0] tx_data_reg;

    localparam [2:0] 
        IDLE = 0,
        READ_START = 1,
        READ_WAIT = 2,
        WRITE_START = 3,
        WRITE_DATA = 4,
        WRITE_WAIT_RESP = 5,
        DONE = 6;

    reg [2:0] current_state, next_state;

    wire [7:0] burst_length = (burst_size_i == 0) ? 8'd0 :
                              (burst_size_i == 1) ? 8'd1 :
                              (burst_size_i == 2) ? 8'd3 :
                              (burst_size_i == 3) ? 8'd7 : 8'd15;

    wire [31:0] burst_bytes = (burst_length + 1) * (DATA_WIDTH/8);

    wire [AXI_ADDR_WIDTH-1:0] next_addr = incr_addr_i ?
                                          (current_addr + burst_bytes) :
                                           current_addr;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            current_state   <= IDLE;
            current_addr    <= 0;
            bytes_remaining <= 0;
            burst_count     <= 0;
            dma_active      <= 0;
            dma_done_reg    <= 0;

            awvalid_reg <= 0;
            wvalid_reg  <= 0;
            wlast_reg   <= 0;
            bready_reg  <= 0;
            arvalid_reg <= 0;
            rready_reg  <= 0;
            data_counter<= 32'hA5A50000;

            // TX FIFO interface initialization - NEWLY ADDED
            tx_wen_reg  <= 0;
            tx_data_reg <= 0;
        end else begin
            current_state <= next_state;
        end
    end

    // FSM Main Control
    always @(posedge clk) begin
        case (current_state)
            IDLE: begin
                if (dma_start_i) begin
                    current_addr    <= dma_addr_i;
                    bytes_remaining <= dma_len_i;
                    dma_active      <= 1;
                    dma_done_reg    <= 0;
                end
            end

            READ_START: begin
                if (m_arready && m_arvalid) begin
                    arvalid_reg    <= 0;
                    current_addr   <= next_addr;
                    bytes_remaining<= bytes_remaining - burst_bytes;
                end
            end

            READ_WAIT: begin
                // When receiving data from AXI, write to TX FIFO for DMA write operations
                // NEWLY ADDED TX FIFO write logic
                if (m_rvalid && m_rready && !dma_dir_i) begin // DMA write mode (dir=0)
                    if (!tx_full) begin
                        tx_wen_reg  <= 1;
                        tx_data_reg <= m_rdata;
                    end else begin
                        tx_wen_reg  <= 0; // Wait for FIFO space
                    end
                end else begin
                    tx_wen_reg <= 0;
                end
            end

            WRITE_START: begin
                if (m_awready && m_awvalid) begin
                    burst_count <= burst_length;
                    wvalid_reg  <= 1;   // Start data phase immediately
                    wlast_reg   <= (burst_length == 0); // Single beat case
                end
            end

            WRITE_DATA: begin
                if (m_wready && m_wvalid) begin
                    if (m_wlast) begin
                        wvalid_reg <= 0;
                        wlast_reg  <= 1;
                        bready_reg <= 1; // Ready for write response
                    end else begin
                        burst_count <= burst_count - 1;
                        wlast_reg   <= (burst_count <= 1);
                        // Increment data counter on each write
                        if (rx_ren) data_counter <= data_counter + 1;
                    end
                end
            end

            WRITE_WAIT_RESP: begin
                if (m_bvalid && m_bready) begin
                    bready_reg <= 0;
                    bytes_remaining <= bytes_remaining - burst_bytes;
                    current_addr    <= next_addr;

                    // Check if transfer is complete
                    if (bytes_remaining <= burst_bytes) begin
                        dma_done_reg <= 1;
                        dma_active   <= 0;
                    end
                end
            end

            DONE: begin
                dma_active   <= 0;
                dma_done_reg <= 1;
                tx_wen_reg   <= 0;  // NEWLY ADDED - Clear TX FIFO write enable
            end

            default: begin
                tx_wen_reg   <= 0;  // NEWLY ADDED - Default clear
            end
        endcase
    end

    always @(*) begin
        next_state = current_state;
        case (current_state)
            IDLE: begin
                if (dma_start_i && dma_len_i != 0)
                    next_state = dma_dir_i ? READ_START : WRITE_START;
            end
            READ_START: begin
                if (m_arready && m_arvalid)
                    next_state = READ_WAIT;
            end
            READ_WAIT: begin
                if (m_rvalid && m_rlast) begin
                    if (bytes_remaining <= burst_bytes)
                        next_state = DONE;
                    else
                        next_state = READ_START;
                end
            end
            WRITE_START: begin
                if (m_awready && m_awvalid)
                    next_state = WRITE_DATA;
            end
            WRITE_DATA: begin
                if (m_wready && m_wvalid && m_wlast)
                    next_state = WRITE_WAIT_RESP;
            end
            WRITE_WAIT_RESP: begin
                if (m_bvalid && m_bready) begin
                    if (bytes_remaining <= burst_bytes)
                        next_state = DONE;
                    else
                        next_state = WRITE_START;
                end
            end
            
            DONE: begin
              next_state = IDLE;
            end
        endcase
    end
    
    // Output assignments
    assign m_awid    = 4'b0;
    assign m_awaddr  = current_addr;
    assign m_awlen   = burst_length;
    assign m_awsize  = (DATA_WIDTH == 32) ? 3'b010 : 3'b011;
    assign m_awburst = 2'b01;
    assign m_awvalid = (current_state == WRITE_START);

    assign m_wdata   = rx_data_dma;
    assign m_wstrb   = {DATA_WIDTH/8{1'b1}};
    assign m_wlast   = wlast_reg;
    assign m_wvalid  = wvalid_reg && (current_state == WRITE_DATA);

    assign m_bready  = bready_reg;

    assign m_arid    = 4'b0;
    assign m_araddr  = current_addr;
    assign m_arlen   = burst_length;
    assign m_arsize  = (DATA_WIDTH == 32) ? 3'b010 : 3'b011;
    assign m_arburst = 2'b01;
    assign m_arvalid = (current_state == READ_START);

    assign m_rready  = (current_state == READ_WAIT);

    assign dma_done_o = dma_done_reg;
    assign rx_ren = (current_state == WRITE_DATA) && wvalid_reg && m_wready && !rx_empty;

    // TX FIFO interface assignments - NEWLY ADDED
    assign tx_wen = tx_wen_reg;
    assign tx_data_dma = tx_data_reg;

endmodule