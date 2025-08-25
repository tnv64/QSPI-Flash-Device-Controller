module tx_fifo #(
    parameter FIFO_DEPTH = 16,   // Default depth in bytes (power of 2)
    parameter DATA_WIDTH = 32    // Data width (32 bits)
) (
    // System signals
    input  wire                  clk,
    input  wire                  rst_n,

    // CSR Interface (APB write)
    input  wire                  fifo_tx_we_o,   // From APB when writing to FIFO_TX_ADDR (0x044)
    input  wire [DATA_WIDTH-1:0] fifo_tx_data_o, // From APB pwdata

    // QSPI FSM Interface
    input  wire                  tx_ren,         // From QSPI FSM when it needs data
    output wire [DATA_WIDTH-1:0] tx_data_fifo,   // To QSPI FSM
    output wire                  tx_empty,       // To QSPI FSM

    // Status outputs
    output wire [3:0]            tx_level,       // To FIFO_STAT register
    output wire                  tx_full,        // For potential error detection
    output wire                  underrun        // To ERR_STAT register (bit 2)
);

    // Memory array and pointers
    reg [DATA_WIDTH-1:0] mem [0:FIFO_DEPTH-1];
    reg [$clog2(FIFO_DEPTH)-1:0] wr_ptr, rd_ptr;
    reg [$clog2(FIFO_DEPTH):0] count;
    reg                         underrun_reg;

    // Read data register to ensure correct FIFO order
    reg [DATA_WIDTH-1:0] read_data;

    // Continuous assignments
    assign tx_full     = (count == FIFO_DEPTH);
    assign tx_empty    = (count == 0);
    assign tx_data_fifo= read_data;
    assign tx_level    = count[3:0];  // Assuming FIFO_DEPTH <= 16
    assign underrun    = underrun_reg;

    // FIFO control logic
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wr_ptr      <= 0;
            rd_ptr      <= 0;
            count       <= 0;
            underrun_reg<= 0;
            read_data   <= 0;
        end else begin
            // Write operation (from CSR)
            if (fifo_tx_we_o && !tx_full) begin
                mem[wr_ptr] <= fifo_tx_data_o;
                wr_ptr      <= wr_ptr + 1;
            end

            // Read operation (by QSPI FSM)
            if (tx_ren) begin
                if (!tx_empty) begin
                    read_data   <= mem[rd_ptr];   // Read before updating pointer
                    rd_ptr      <= rd_ptr + 1;
                    underrun_reg<= 0;
                end else begin
                    // FIFO is empty but QSPI FSM is trying to read - underrun condition
                    underrun_reg<= 1;
                end
            end

            // Update count
            case ({fifo_tx_we_o && !tx_full, tx_ren && !tx_empty})
                2'b10: count <= count + 1; // Only write
                2'b01: count <= count - 1; // Only read
                default: ;                 // No change
            endcase
        end
    end

endmodule

