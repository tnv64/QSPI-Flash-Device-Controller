module rx_fifo #(
    parameter FIFO_DEPTH = 16,   // Default depth in bytes (power of 2)
    parameter DATA_WIDTH = 32    // Data width (32 bits)
) (
    // System signals
    input  wire                  clk,
    input  wire                  rst_n,

    // QSPI FSM Interface
    input  wire                  rx_wen,         // From QSPI FSM when it has received data
    input  wire [DATA_WIDTH-1:0] rx_data_fifo,   // From QSPI FSM

    // CSR Interface (APB read)
    input  wire                  fifo_rx_re_o,   // From APB when reading FIFO_RX_ADDR (0x048)
    output wire [DATA_WIDTH-1:0] fifo_rx_data_i, // To APB prdata

    // DMA Interface
    input  wire                  dma_rd_en,      // From DMA when it needs data
    output wire [DATA_WIDTH-1:0] dma_rd_data,    // To DMA
    output wire                  dma_empty,      // To DMA

    // Status outputs
    output wire [3:0]            rx_level,       // To FIFO_STAT register
    output wire                  rx_full,        // To FIFO_STAT and interrupt logic
    output wire                  rx_empty,       // For CSR and DMA
    output wire                  overrun         // To ERR_STAT register (bit 1)
);

    // Memory array and pointers
    reg [DATA_WIDTH-1:0] mem [0:FIFO_DEPTH-1];
    reg [$clog2(FIFO_DEPTH)-1:0] wr_ptr, rd_ptr;
    reg [$clog2(FIFO_DEPTH):0] count;
    reg                         overrun_reg;

    // Read data register to ensure correct FIFO order
    reg [DATA_WIDTH-1:0] read_data;

    // Read data mux (CSR or DMA)
    wire actual_rd_en = fifo_rx_re_o | dma_rd_en;

    always @(posedge clk) begin
        if (actual_rd_en && !rx_empty)
            read_data <= mem[rd_ptr];
    end

    assign fifo_rx_data_i = read_data;
    assign dma_rd_data    = mem[rd_ptr];

    // Continuous assignments
    assign rx_full   = (count == FIFO_DEPTH);
    assign rx_empty  = (count == 0);
    assign dma_empty = rx_empty;
    assign rx_level  = count[3:0];  // Assuming FIFO_DEPTH <= 16
    assign overrun   = overrun_reg;

    // FIFO control logic
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wr_ptr      <= 0;
            rd_ptr      <= 0;
            count       <= 0;
            overrun_reg <= 0;
        end else begin
            // Write operation (from QSPI FSM)
            if (rx_wen) begin
                if (!rx_full) begin
                    mem[wr_ptr] <= rx_data_fifo;
                    wr_ptr      <= wr_ptr + 1;
                    overrun_reg <= 0;
                end else begin
                    // FIFO is full but we're trying to write - overrun condition
                    overrun_reg <= 1;
                end
            end

            // Read operation (by CSR or DMA)
            if (actual_rd_en && !rx_empty) begin
                rd_ptr <= rd_ptr + 1;
            end

            // Update count
            case ({rx_wen && !rx_full, actual_rd_en && !rx_empty})
                2'b10: count <= count + 1; // Only write
                2'b01: count <= count - 1; // Only read
                default: ;                 // No change
            endcase
        end
    end

endmodule

