module qspi_device (
  input  wire qspi_sclk,
  input  wire qspi_cs_n,
  inout  wire qspi_io0,
  inout  wire qspi_io1,
  inout  wire qspi_io2,
  inout  wire qspi_io3
);

  // ---------------------------------------------------------------------------
  // Parameters (from screenshots)
  // ---------------------------------------------------------------------------
  parameter MEM_SIZE   = 1024 * 1024;
  parameter ADDR_BITS  = 24;
  parameter PAGE_SIZE  = 256;
  parameter SECTOR_SIZE= 4096;
  parameter ERASE_TIME = 100;

  // Memory
  reg [7:0] memory [0:MEM_SIZE-1];

  // I/O tristsate
  reg  [3:0] io_oe = 0;   // 1=drive
  reg  [3:0] io_do = 0;   // data out
  wire [3:0] io_di = {qspi_io3, qspi_io2, qspi_io1, qspi_io0};

  assign qspi_io0 = io_oe[0] ? io_do[0] : 1'bz;
  assign qspi_io1 = io_oe[1] ? io_do[1] : 1'bz;
  assign qspi_io2 = io_oe[2] ? io_do[2] : 1'bz;
  assign qspi_io3 = io_oe[3] ? io_do[3] : 1'bz;

  // Registers per screenshots
  reg [7:0]  status_reg = 8'h00;          // WIP in bit0
  reg [31:0] id_reg     = 32'h00C22017;   // example manufacturer/device id
  reg [7:0]  cmd_reg    = 8'h00;
  reg [7:0]  nxt_cmd_reg= 8'h00;
  reg [ADDR_BITS-0:0] addr_reg = 0;       // local address (allow extra for shifts)
  reg [7:0]  mode_bits  = 8'h00;
  reg [7:0]  shift_in   = 8'h00;
  reg [7:0]  shift_out  = 8'h00;
  reg [3:0]  lanes      = 0;
  reg [7:0]  dummy_cycles = 0;
  reg        continuous_read = 0;

  // FSM states (values from screenshots)
  localparam ST_IDLE       = 4'h0;
  localparam ST_CMD        = 4'h1;
  localparam ST_ADDR       = 4'h2;
  localparam ST_MODE       = 4'h3;
  localparam ST_DUMMY      = 4'h4;
  localparam ST_DATA_READ  = 4'h5;
  localparam ST_DATA_WRITE = 4'h6;
  localparam ST_ERASE      = 4'h7;
  localparam ST_STATUS     = 4'h8;
  localparam ST_ID_READ    = 4'h9;

  reg [3:0] state = ST_IDLE;

  // Counters
  reg [31:0] bit_cnt      = 0;
  reg [31:0] nxt_bit_cnt  = 0;
  reg [31:0] nxt_addr_reg = 0;
  reg [31:0] byte_cnt     = 0;
  reg        wip          = 0;
  reg [31:0] erase_counter= 0;

  // ---------------------------------------------------------------------------
  // Init memory to 0xFF
  // ---------------------------------------------------------------------------
  integer i;
  integer j;
  initial begin
    for (i = 0; i < MEM_SIZE; i = i + 1) begin
      memory[i] = 8'hFF;
    end
    wip = 1'b0;
  end

  // ---------------------------------------------------------------------------
  // Fake erase timer when WIP set
  // ---------------------------------------------------------------------------
  always @(posedge qspi_sclk) begin
    if (wip) begin
      erase_counter <= erase_counter + 1;
      if (erase_counter >= ERASE_TIME) begin
        wip <= 1'b0;
        erase_counter <= 0;
      end
    end
  end

  // ---------------------------------------------------------------------------
  // Main protocol state machine
  // - shift on SCLK posedge
  // - reset on CS# rising edge (device deselect)
// ---------------------------------------------------------------------------
  always @(posedge qspi_sclk or posedge qspi_cs_n) begin
    if (qspi_cs_n) begin
      state          <= ST_IDLE;
      io_oe          <= 4'b0000;
      continuous_read<= 1'b0;
      shift_in       <= 8'h00;
    end else begin
      case (state)
        // ------------- Idle -> wait first byte (opcode) ----------------------
        ST_IDLE: begin
          state   <= ST_CMD;
          bit_cnt <= 0;
          lanes   <= 1;
          shift_in<= {shift_in[6:0], io_di[0]};
        end

        // ------------- Shift in OPCODE --------------------------------------
        ST_CMD: begin
          // shift in by 1 lane (always command on 1 lane in this simple model)
          shift_in <= {shift_in[6:0], io_di[0]};
          bit_cnt  <= bit_cnt + 1;
          if (nxt_bit_cnt == 7) begin
            cmd_reg  <= {shift_in[6:0], io_di[0]};
            bit_cnt  <= 0;
            byte_cnt <= 0;
            case (nxt_cmd_reg)
              8'h9F: begin // READ ID
                state        <= ST_ID_READ;
                lanes        <= 1;
                dummy_cycles <= 0;
                shift_out    <= id_reg[23:16];
                io_oe        <= 4'b0010; // drive IO1 only in this simple model (DO on IO1)
              end
              8'h05: begin // READ STATUS
                state     <= ST_STATUS;
                lanes     <= 1;
                shift_out <= status_reg;
                io_oe     <= 4'b0010;
              end
              8'h06: begin // WRITE ENABLE (set WIP=1)
                status_reg[1] <= 1'b1;
                state         <= ST_IDLE;
              end
              8'h04: begin // WRITE DISABLE (clear WIP)
                status_reg[1] <= 1'b0;
                state         <= ST_IDLE;
              end
              8'h03, 8'h0B, 8'h3B, 8'h6B, 8'hEB: begin // READ variants
                state    <= ST_ADDR;
                addr_reg <= 0;
                shift_in <= 0;
                // lane selection: 0x03->1, 0x0B->1 (fast), 0x3B->2, 0x6B/0xEB->4
                lanes        <= (nxt_cmd_reg == 8'h03 || nxt_cmd_reg == 8'h0B) ? 1 :
                                (nxt_cmd_reg == 8'h3B) ? 2 : 4;
                dummy_cycles <= (nxt_cmd_reg == 8'h03) ? 0 :
                                (nxt_cmd_reg == 8'hEB) ? 8 : 6;
              end
              8'h02, 8'h32: begin // PAGE PROGRAM variants
                dummy_cycles <= 0;
                if (status_reg[1]) begin
                  state <= ST_ADDR;
                  lanes <= (nxt_cmd_reg == 8'h02) ? 1 : 4;
                end else begin
                  state <= ST_IDLE;
                end
              end
              8'h20, 8'hD8: begin // ERASE sector/block
                if (status_reg[1]) begin
                  state <= ST_ADDR;
                end else begin
                  state <= ST_IDLE;
                end
              end
              default: state <= ST_IDLE;
            endcase
          end
        end

        // ------------- Shift in ADDRESS (1/2/4-lane) ------------------------
        ST_ADDR: begin
          
          // accumulate address bits according to lane count
          if (lanes == 1)       shift_in <= {shift_in[6:0],          io_di[0]};
          else if (lanes == 2)  shift_in <= {shift_in[5:0], io_di[1], io_di[0]};
          else                   shift_in <= {shift_in[3:0], io_di[3], io_di[2], io_di[1], io_di[0]};
          bit_cnt <= bit_cnt + lanes;

          if ((bit_cnt == 7  && lanes==1) ||
              (bit_cnt == 6  && lanes==2) ||
              (bit_cnt == 4  && lanes==4)) begin
            // latch next address byte
            addr_reg <= {addr_reg[ADDR_BITS-9:0], (lanes==1) ? {shift_in[6:0], io_di[0]} :
                        (lanes==2) ? {shift_in[5:0], io_di[1], io_di[0]} :
                                     {shift_in[3:0], io_di[3], io_di[2], io_di[1], io_di[0]}};
            bit_cnt  <= 0;
            shift_in <= 0;
            byte_cnt <= byte_cnt + 1;

            if (byte_cnt == (ADDR_BITS/8) - 1) begin
              if (cmd_reg == 8'hEB) begin
                state <= ST_MODE;
              end else if (cmd_reg == 8'h20 || cmd_reg == 8'hD8) begin
                // emulate erase (fill with 0xFF)
                
                if (cmd_reg == 8'h20) begin
                  for (j = nxt_addr_reg; j < nxt_addr_reg + SECTOR_SIZE; j = j + 1)
                    memory[j] <= 8'hFF;
                end else begin
                  for (j = nxt_addr_reg; j < nxt_addr_reg + 65536; j = j + 1)
                    memory[j] <= 8'hFF;
                end
                wip   <= 1'b1;
                state <= ST_ERASE;
              end else if (dummy_cycles > 0) begin
                state <= ST_DUMMY;
              end else begin
                state   <= (cmd_reg==8'h02 || cmd_reg==8'h32) ? ST_DATA_WRITE : ST_DATA_READ;
                bit_cnt <= 0;
                byte_cnt<= 0;
                if (cmd_reg==8'h02 || cmd_reg==8'h32)       io_oe <= 4'b0000;
                else if (lanes==1)                         io_oe <= 4'b0010;
                else if (lanes==2)                         io_oe <= 4'b0011;
                else                                        io_oe <= 4'b1111;
                shift_out <= memory[addr_reg];
              end
            end
          end
        end

        // ------------- MODE (only for 0xEB) ----------------------------------
        ST_MODE: begin
          if (lanes == 1)       shift_in <= {shift_in[6:0], io_di[0]};
          else if (lanes == 2)  shift_in <= {shift_in[5:0], io_di[1], io_di[0]};
          else                   shift_in <= {shift_in[3:0], io_di[3], io_di[2], io_di[1], io_di[0]};
          bit_cnt <= bit_cnt + lanes;
          if (bit_cnt == 8) begin
            mode_bits       <= shift_in[7:0];
            continuous_read <= (shift_in == 8'hA0);
            state           <= ST_DUMMY;
            bit_cnt         <= 0;
          end
        end

        // ------------- DUMMY cycles -----------------------------------------
        ST_DUMMY: begin
          bit_cnt <= bit_cnt + 1;
          if (bit_cnt == (dummy_cycles - 1)) begin
            state    <= (cmd_reg==8'h02 || cmd_reg==8'h32) ? ST_DATA_WRITE : ST_DATA_READ;
            bit_cnt  <= 0;
            byte_cnt <= 0;
            if (cmd_reg==8'h02 || cmd_reg==8'h32)       io_oe <= 4'b0000;
            else if (lanes==1)                         io_oe <= 4'b0010;
            else if (lanes==2)                         io_oe <= 4'b0011;
            else                                        io_oe <= 4'b1111;
            shift_out <= memory[addr_reg];
          end
        end

        // ------------- DATA READ --------------------------------------------
        ST_DATA_READ: begin
          // drive data on outputs by lane count
          if (lanes == 1) begin
            io_do[0] <= shift_out[7];
          end else if (lanes == 2) begin
            io_do[0] <= shift_out[6];
            io_do[1] <= shift_out[7];
          end else begin
            io_do[0] <= shift_out[4];
            io_do[1] <= shift_out[5];
            io_do[2] <= shift_out[6];
            io_do[3] <= shift_out[7];
          end
          shift_out <= {shift_out[6:0], 1'b0};
          bit_cnt   <= bit_cnt + lanes;

          if ((bit_cnt == 7 && lanes==1) ||
              (bit_cnt == 6 && lanes==2) ||
              (bit_cnt == 4 && lanes==4)) begin
            addr_reg  <= addr_reg + 1;
            shift_out <= memory[addr_reg + 1];
            bit_cnt   <= 0;
            if (!continuous_read && byte_cnt == MEM_SIZE - addr_reg) state <= ST_IDLE;
            byte_cnt <= byte_cnt + 1;
          end
        end

        // ------------- DATA WRITE -------------------------------------------
        ST_DATA_WRITE: begin
          if (lanes == 1)       shift_in <= {shift_in[6:0], io_di[0]};
          else if (lanes == 2)  shift_in <= {shift_in[5:0], io_di[1], io_di[0]};
          else                   shift_in <= {shift_in[3:0], io_di[3], io_di[2], io_di[1], io_di[0]};
          bit_cnt <= bit_cnt + lanes;

          if ((bit_cnt == 7 && lanes==1) ||
              (bit_cnt == 6 && lanes==2) ||
              (bit_cnt == 4 && lanes==4)) begin
            if (addr_reg < MEM_SIZE && (addr_reg % PAGE_SIZE != PAGE_SIZE-1)) begin
              memory[addr_reg] <= (lanes==1) ? {shift_in[6:0], io_di[0]} :
                                  (lanes==2) ? {shift_in[5:0], io_di[1], io_di[0]} :
                                               {shift_in[3:0], io_di[3], io_di[2], io_di[1], io_di[0]};
              addr_reg <= addr_reg + 1;
            end
            bit_cnt  <= 0;
            byte_cnt <= byte_cnt + 1;
            wip      <= 1'b1;
          end
        end

        // ------------- ERASE (CS# will be deasserted next) ------------------
        ST_ERASE: begin
          state <= ST_IDLE;
        end

        // ------------- STATUS / ID read serially on IO1 ---------------------
        ST_STATUS: begin
          io_do[1] <= shift_out[7];
          shift_out<= {shift_out[6:0], 1'b0};
          bit_cnt  <= bit_cnt + 1;
          if (bit_cnt == 8) state <= ST_IDLE;
        end
        ST_ID_READ: begin
          // only manufacturer id in screenshot
          io_do[1] <= shift_out[7];
          shift_out<= {shift_out[6:0], 1'b0};
          bit_cnt  <= bit_cnt + 1;
          if (bit_cnt == 8) state <= ST_IDLE;
        end

        default: ;
      endcase
    end
  end

  // ---------------------------------------------------------------------------
  // Combinational helpers as in screenshots
  // ---------------------------------------------------------------------------
  always @* begin
    status_reg[0] = wip;
    nxt_cmd_reg   = {shift_in[6:0], io_di[0]};
    nxt_bit_cnt   = bit_cnt + 1;
    nxt_addr_reg  = {addr_reg[ADDR_BITS-9:0],
                     (lanes==1) ? {shift_in[6:0], io_di[0]} :
                     (lanes==2) ? {shift_in[5:0], io_di[1], io_di[0]} :
                                   {shift_in[3:0], io_di[3], io_di[2], io_di[1], io_di[0]}};
  end

endmodule
