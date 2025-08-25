module qspi_fsm #( 
  parameter ADDR_W = 32 )
(
  input               clk,
  input               rst_n,

  // Timing / CS / CTRL (split)
  input      [2:0]    clk_div,    // SCLK = clk / 2^clk_div while active
  input               cs_auto,    // 1: FSM controls CS#, 0: use cs_level
  input               cs_level,   // CS# level when cs_auto==0
  input               quad_en,    // force ADDR/DATA lanes to Quad (ignored for 0x04/0x05/0x9F)
  input               cpol,       // SPI CPOL
  input               cpha,       // SPI CPHA
  input               lsb_first,  // 1: LSB-first (will be latched to lsb_first_q)

  // Command bundle
  input               start,
  input      [15:0]   cmd_cfg,    // format fields (see header)
  input      [15:0]   cmd_op,     // [7:0]=opcode, [15:8]=mode bits
  input      [ADDR_W-1:0] cmd_addr,
  input      [7:0]    cmd_dummy,
  input      [31:0]   cmd_len,
  output              done,

  // TX FIFO (write -> flash)
  input               tx_ren,          // external pulse: tx_data_fifo valid now
  input      [7:0]    tx_data_fifo,
  output reg          tx_empty,

  // RX FIFO (read <- flash)
  output reg          rx_wen,
  output reg [7:0]    rx_data_fifo,
  input               rx_full,

  // QSPI pins
  output              sclk,
  output reg          cs_n,
  inout               io0,
  inout               io1,
  inout               io2,
  inout               io3
);

  // ------------------ decode CMD/CTRL ------------------
  wire [7:0] opcode    = cmd_op[7:0];
  wire [7:0] mode_bits = cmd_op[15:8];

  // special opcodes
  wire is_cmd_wrdi = (opcode == 8'h04); // WRDI: opcode-only
  wire is_cmd_rdsr = (opcode == 8'h05); // RDSR: opcode-only + read N
  wire is_cmd_rdid = (opcode == 8'h9F); // RDID: opcode-only + read 3

  // raw lanes from CSR
  wire [1:0] cmd_lanes_raw  = cmd_cfg[1:0];
  wire [1:0] addr_lanes_raw = cmd_cfg[3:2];
  wire [1:0] data_lanes_raw = cmd_cfg[5:4];

  // for special commands, force single I/O regardless of quad_en
  wire [1:0] eff_cmd_lanes  = (is_cmd_wrdi|is_cmd_rdsr|is_cmd_rdid) ? 2'd0 : cmd_lanes_raw;
  wire [1:0] eff_addr_lanes = (is_cmd_wrdi|is_cmd_rdsr|is_cmd_rdid) ? 2'd0
                           : (quad_en ? 2'd2 : addr_lanes_raw);
  wire [1:0] eff_data_lanes = (is_cmd_wrdi|is_cmd_rdsr|is_cmd_rdid) ? 2'd0
                           : (quad_en ? 2'd2 : data_lanes_raw);

  function [2:0] lane_w; input [1:0] code; begin
    case (code)
      2'd0: lane_w = 3'd1;
      2'd1: lane_w = 3'd2;
      default: lane_w = 3'd4;
    endcase
  end endfunction

  wire [2:0] w_cmd  = lane_w(eff_cmd_lanes);
  wire [2:0] w_addr = lane_w(eff_addr_lanes);
  wire [2:0] w_data = lane_w(eff_data_lanes);
  wire [2:0] w_mode = w_addr; // MODE bits use same lanes as ADDR

  // address bytes (CSR encoding: 0->0B, 1->3B, 2->4B)
  wire [1:0] addr_bytes_sel = cmd_cfg[7:6];
  reg  [2:0] addr_bytes_raw;
  always @(*) begin
    case (addr_bytes_sel)
      2'd1: addr_bytes_raw = 3'd3;
      2'd2: addr_bytes_raw = 3'd4;
      default: addr_bytes_raw = 3'd0;
    endcase
  end

  // effective address bytes (force 0 for special commands)
  wire [2:0] addr_bytes = (is_cmd_wrdi|is_cmd_rdsr|is_cmd_rdid) ? 3'd0 : addr_bytes_raw;

  // mode/dummy/dir
  wire       mode_en_raw   = cmd_cfg[8];
  wire [3:0] dummy_cfg     = cmd_cfg[12:9];
  wire       dir_read_raw  = cmd_cfg[13];

  // special: disable MODE, DUMMY for these commands
  wire       mode_en   = (is_cmd_wrdi|is_cmd_rdsr|is_cmd_rdid) ? 1'b0 : mode_en_raw;
  wire [7:0] total_dummy_raw = {4'b0, dummy_cfg} + cmd_dummy;
  wire [7:0] total_dummy = (is_cmd_wrdi|is_cmd_rdsr|is_cmd_rdid) ? 8'd0 : total_dummy_raw;

  // special: data direction (RDID/RDSR=read, WRDI=no data)
  wire       dir_read = is_cmd_rdid ? 1'b1 :
                        is_cmd_rdsr ? 1'b1 :
                        dir_read_raw;

  // effective length (if CMD_LEN==0 for RDID -> default 3, RDSR -> default 1, WRDI -> 0)
  wire [31:0] cmd_len_eff =
      is_cmd_wrdi ? 32'd0 :
      is_cmd_rdid ? ((cmd_len==32'd0) ? 32'd3 : cmd_len) :
      is_cmd_rdsr ? ((cmd_len==32'd0) ? 32'd1 : cmd_len) :
                    cmd_len;

  // ------------------ SCLK generator ------------------
  reg [7:0] div_cnt;
  reg       sclk_int, sclk_en;
  wire [7:0] div_mask = (clk_div==3'd0) ? 8'd1 : (8'd1 << clk_div);

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      div_cnt  <= 8'd0;
      sclk_int <= 1'b0;
    end else if (sclk_en) begin
      div_cnt <= div_cnt + 8'd1;
      if (div_cnt == (div_mask - 1)) begin
        div_cnt  <= 8'd0;
        sclk_int <= ~sclk_int;
      end
    end else begin
      div_cnt  <= 8'd0;
      sclk_int <= 1'b0;
    end
  end

  assign sclk = sclk_en ? (cpol ^ sclk_int) : cpol;

  reg sclk_int_d;
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) sclk_int_d <= 1'b0;
    else        sclk_int_d <= sclk_int;
  end

  wire sclk_rise =  sclk_int & ~sclk_int_d;
  wire sclk_fall = ~sclk_int &  sclk_int_d;

  // shift/sample strobes by CPHA
  wire strobe_shift  = cpha ? sclk_rise : sclk_fall;
  wire strobe_sample = cpha ? sclk_fall : sclk_rise;

  // ------------------ IO tri-state ------------------
  reg  [3:0] io_o, io_oe; wire [3:0] io_i;
  assign io0 = io_oe[0] ? io_o[0] : 1'bz;
  assign io1 = io_oe[1] ? io_o[1] : 1'bz;
  assign io2 = io_oe[2] ? io_o[2] : 1'bz;
  assign io3 = io_oe[3] ? io_o[3] : 1'bz;
  assign io_i[0]=io0; assign io_i[1]=io1; assign io_i[2]=io2; assign io_i[3]=io3;

  // ------------------ FSM ------------------
  localparam ST_IDLE   = 4'd0,
             ST_CS_ON  = 4'd1,
             ST_OPCODE = 4'd2,
             ST_ADDR   = 4'd3,
             ST_MODE   = 4'd4,
             ST_DUMMY  = 4'd5,
             ST_DATA_W = 4'd6,
             ST_DATA_R = 4'd7,
             ST_CS_OFF = 4'd8,
             ST_DONE   = 4'd9;

  reg [3:0] state, nstate;

  reg [7:0] out_byte, in_byte;
  reg [2:0] group_ptr;           // 0..7 (advances by lane width)
  reg [31:0] bytes_left;
  reg [2:0] addr_idx;
  reg [7:0] dummy_left;
  reg       done_r; assign done = done_r;

  // LSB-first latch per command
  reg       lsb_first_q;

  // TX latch (since tx_ren is an input)
  reg       have_tx;
  reg [7:0] tx_lat;

  // start pulse
  reg start_d; always @(posedge clk or negedge rst_n) if (!rst_n) start_d<=1'b0; else start_d<=start;
  wire start_pulse = start & ~start_d;

  // group_done helper (no 3-bit overflow)
  function group_done; input [2:0] w; reg [2:0] thr; begin
    if (w==3'd1) thr = 3'd7;     // after 8 shifts
    else if (w==3'd2) thr = 3'd6; // after 4*2
    else thr = 3'd4;             // after 2*4
    group_done = (group_ptr >= thr);
  end endfunction

  // state reg
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) state <= ST_IDLE;
    else        state <= nstate;
  end

  // next-state
  always @(*) begin
    nstate = state;
    case (state)
      ST_IDLE   : nstate = start_pulse ? ST_CS_ON : ST_IDLE;
      ST_CS_ON  : nstate = ST_OPCODE;

      ST_OPCODE : if (strobe_shift && group_done(w_cmd)) begin
                    if (addr_bytes!=0)                    nstate = ST_ADDR;
                    else if (mode_en)                     nstate = ST_MODE;
                    else if (total_dummy!=0)              nstate = ST_DUMMY;
                    else if (cmd_len_eff!=0 && !dir_read) nstate = ST_DATA_W;
                    else if (cmd_len_eff!=0 &&  dir_read) nstate = ST_DATA_R;
                    else                                  nstate = ST_CS_OFF;
                  end

      ST_ADDR   : if (strobe_shift && group_done(w_addr)) begin
                    if (addr_idx == (addr_bytes-1)) begin
                      if (mode_en)                         nstate = ST_MODE;
                      else if (total_dummy!=0)             nstate = ST_DUMMY;
                      else if (cmd_len_eff!=0 && !dir_read)nstate = ST_DATA_W;
                      else if (cmd_len_eff!=0 &&  dir_read)nstate = ST_DATA_R;
                      else                                 nstate = ST_CS_OFF;
                    end
                  end

      ST_MODE   : if (strobe_shift && group_done(w_mode)) begin
                    if (total_dummy!=0)                    nstate = ST_DUMMY;
                    else if (cmd_len_eff!=0 && !dir_read)  nstate = ST_DATA_W;
                    else if (cmd_len_eff!=0 &&  dir_read)  nstate = ST_DATA_R;
                    else                                   nstate = ST_CS_OFF;
                  end

      ST_DUMMY  : if (strobe_sample && (dummy_left==8'd0))
                    nstate = (cmd_len_eff!=0) ? (dir_read ? ST_DATA_R : ST_DATA_W) : ST_CS_OFF;

      ST_DATA_W : if (strobe_shift && group_done(w_data)) begin
                    if (bytes_left==32'd1) nstate = ST_CS_OFF;
                  end
      ST_DATA_R : if (strobe_shift && group_done(w_data)) begin
                    if (bytes_left==32'd1) nstate = ST_CS_OFF;
                  end

      ST_CS_OFF : nstate = ST_DONE;
      ST_DONE   : nstate = ST_IDLE;
      default   : nstate = ST_IDLE;
    endcase
  end

  // datapath / outputs
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      sclk_en<=1'b0; cs_n<=1'b1; io_oe<=4'b0000; io_o<=4'b0000;
      out_byte<=8'h00; in_byte<=8'h00; group_ptr<=3'd0; bytes_left<=32'd0;
      addr_idx<=3'd0; dummy_left<=8'd0; have_tx<=1'b0; tx_lat<=8'h00;
      tx_empty<=1'b1; rx_wen<=1'b0; rx_data_fifo<=8'h00; done_r<=1'b0;
      lsb_first_q<=1'b0;
    end else begin
      rx_wen<=1'b0; done_r<=1'b0;

      // CS/SCLK
      case (state)
        ST_IDLE:   begin sclk_en<=1'b0; cs_n <= (cs_auto ? 1'b1 : cs_level); end
        ST_CS_ON,
        ST_OPCODE, ST_ADDR, ST_MODE, ST_DUMMY, ST_DATA_W, ST_DATA_R:
                   begin sclk_en<=1'b1; cs_n<=1'b0; end
        ST_CS_OFF: begin sclk_en<=1'b1; cs_n <= (cs_auto ? 1'b1 : cs_level); end
        ST_DONE:   begin sclk_en<=1'b0; cs_n <= (cs_auto ? 1'b1 : cs_level); done_r<=1'b1; end
      endcase

      // Start of command: latch per-command state
      if (state==ST_IDLE && start_pulse) begin
        lsb_first_q <= lsb_first;               // <== latch LSB-first
        group_ptr   <= 3'd0;
        out_byte    <= opcode;
        bytes_left  <= cmd_len_eff;
        addr_idx    <= 3'd0;
        dummy_left  <= total_dummy;
        tx_empty    <= (cmd_len_eff==0) || dir_read;
        have_tx     <= 1'b0;
      end

      // Load next outgoing byte at byte boundaries
      if (state==ST_CS_ON && nstate==ST_OPCODE) begin
        out_byte  <= opcode; group_ptr <= 3'd0;
      end
      else if (state==ST_OPCODE && strobe_shift && group_done(w_cmd)) begin
        group_ptr <= 3'd0;
        if (addr_bytes!=0)        out_byte <= pick_addr_byte(cmd_addr, addr_bytes, 3'd0);
        else if (mode_en)         out_byte <= mode_bits;   // MODE if no ADDR
      end
      else if (state==ST_ADDR && strobe_shift && group_done(w_addr)) begin
        group_ptr <= 3'd0;
        if (addr_idx < addr_bytes-1) out_byte <= pick_addr_byte(cmd_addr, addr_bytes, addr_idx+1);
        else if (mode_en)            out_byte <= mode_bits; // MODE after last ADDR
      end
      else if (state==ST_MODE && strobe_shift && group_done(w_mode)) begin
        group_ptr <= 3'd0;
      end
      else if (state==ST_DATA_W && strobe_shift && group_done(w_data)) begin
        group_ptr <= 3'd0;
      end
      else if (state==ST_DATA_R && strobe_shift && group_done(w_data)) begin
        group_ptr <= 3'd0;
      end

      // addr index
      if (state==ST_ADDR && strobe_shift && group_done(w_addr))
        addr_idx <= addr_idx + 3'd1;

      // dummy
      if ((state==ST_OPCODE && nstate==ST_DUMMY) ||
          (state==ST_ADDR   && nstate==ST_DUMMY) ||
          (state==ST_MODE   && nstate==ST_DUMMY)) begin
        dummy_left <= total_dummy;
      end else if (state==ST_DUMMY && strobe_sample && dummy_left!=8'd0) begin
        dummy_left <= dummy_left - 8'd1;
      end

      // TX latch / tx_empty
      if (tx_ren) begin tx_lat <= tx_data_fifo; have_tx <= 1'b0; have_tx <= 1'b1; end
      if (state==ST_DATA_W && strobe_shift && group_done(w_data)) begin
        if (bytes_left!=0) bytes_left <= bytes_left - 32'd1;
        tx_empty <= (bytes_left<=32'd1);
        have_tx  <= 1'b0;
      end
      if (state==ST_DATA_R && strobe_shift && group_done(w_data))
        if (bytes_left!=0) bytes_left <= bytes_left - 32'd1;

      // ------------ IO drive (case by width; no variable part-selects) ------------
      io_oe <= 4'b0000; io_o <= 4'b0000;

      if (state==ST_OPCODE)      drive_group(w_cmd , out_byte, group_ptr, lsb_first_q, io_oe, io_o);
      else if (state==ST_ADDR)   drive_group(w_addr, out_byte, group_ptr, lsb_first_q, io_oe, io_o);
      else if (state==ST_MODE)   drive_group(w_mode, out_byte, group_ptr, lsb_first_q, io_oe, io_o);
      else if (state==ST_DATA_W) drive_group(w_data, (have_tx ? tx_lat : 8'h00), group_ptr, lsb_first_q, io_oe, io_o);
      // reads: Hi?Z, sample only

      // advance group_ptr each shift
      if (strobe_shift) begin
        if (state==ST_OPCODE)      group_ptr <= group_ptr + w_cmd;
        else if (state==ST_ADDR)   group_ptr <= group_ptr + w_addr;
        else if (state==ST_MODE)   group_ptr <= group_ptr + w_mode;
        else if (state==ST_DATA_W || state==ST_DATA_R)
                                   group_ptr <= group_ptr + w_data;
      end

      // receive path
      if (state==ST_DATA_R && strobe_sample)
        sample_group(w_data, in_byte, lsb_first_q, io_i); // NOTE: w=1 samples IO1 (MISO)

      if (state==ST_DATA_R && strobe_shift && group_done(w_data)) begin
        if (!rx_full) begin rx_data_fifo <= in_byte; rx_wen <= 1'b1; end
        in_byte <= 8'h00;
      end

    end
  end

  // ------------------ helpers ------------------
  function [7:0] pick_addr_byte;
    input [31:0] a; input [2:0] nbytes; input [2:0] idx;
    begin
      if (nbytes==3) begin
        case (idx)
          3'd0: pick_addr_byte = a[23:16];
          3'd1: pick_addr_byte = a[15:8];
          default: pick_addr_byte = a[7:0];
        endcase
      end else begin
        case (idx)
          3'd0: pick_addr_byte = a[31:24];
          3'd1: pick_addr_byte = a[23:16];
          3'd2: pick_addr_byte = a[15:8];
          default: pick_addr_byte = a[7:0];
        endcase
      end
    end
  endfunction

  // Drive w bits (w=1/2/4) to lanes IO[3:0] without variable part-selects
  task drive_group;
    input  [2:0] w;              // 1,2,4
    input  [7:0] byte_val;
    input  [2:0] ptr;            // 0..7
    input        lsb_first_t;
    output [3:0] oe;
    output [3:0] o;
    reg [3:0] oe_r, o_r;
    reg b0,b1,b2,b3;
    integer idx;
    begin
      oe_r = 4'b0000; o_r = 4'b0000;
      case (w)
        3'd1: begin
          // single I/O output on IO0 (MOSI)
          oe_r = 4'b0001;
          idx  = lsb_first_t ? ptr : (7 - ptr);
          b0   = byte_val[idx];
          o_r  = {3'b000, b0};                 // IO0
        end
        3'd2: begin
          oe_r = 4'b0011;
          if (lsb_first_t) begin
            b1 = byte_val[ptr+1]; b0 = byte_val[ptr];
          end else begin
            b1 = byte_val[7 - ptr    ];
            b0 = byte_val[7 - (ptr+1)];
          end
          o_r = {2'b00, b1, b0};               // IO1, IO0
        end
        default: begin // 4
          oe_r = 4'b1111;
          if (lsb_first_t) begin
            b3 = byte_val[ptr+3];
            b2 = byte_val[ptr+2];
            b1 = byte_val[ptr+1];
            b0 = byte_val[ptr  ];
          end else begin
            b3 = byte_val[7 - ptr    ];
            b2 = byte_val[7 - (ptr+1)];
            b1 = byte_val[7 - (ptr+2)];
            b0 = byte_val[7 - (ptr+3)];
          end
          o_r = {b3,b2,b1,b0};                 // IO3..IO0
        end
      endcase
      oe = oe_r; o = o_r;
    end
  endtask

  // Sample w bits (w=1/2/4) from lanes into in_byte, no variable slices
  // NOTE: w=1 (single-I/O read) samples IO1 (MISO) per SPI datasheet.
  task sample_group;
    input  [2:0] w;
    inout  [7:0] byte_accum;
    input        lsb_first_t;
    input  [3:0] lanes_in; // {IO3,IO2,IO1,IO0}
    reg b0,b1,b2,b3;
    begin
      b0 = lanes_in[0]; b1 = lanes_in[1]; b2 = lanes_in[2]; b3 = lanes_in[3];
      case (w)
        3'd1: begin
          // single-I/O read comes from IO1
          if (!lsb_first_t)  byte_accum <= {byte_accum[6:0], b1};
          else               byte_accum <= {b1, byte_accum[7:1]};
        end
        3'd2: begin
          if (!lsb_first_t)  byte_accum <= {byte_accum[5:0], b1, b0};
          else               byte_accum <= {b1, b0, byte_accum[7:2]};
        end
        default: begin // 4
          if (!lsb_first_t)  byte_accum <= {byte_accum[3:0], b3, b2, b1, b0};
          else               byte_accum <= {b3, b2, b1, b0, byte_accum[7:4]};
        end
      endcase
    end
  endtask

endmodule
