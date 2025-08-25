`timescale 1ns/1ps

module tb_xip;

  // ---------------------------------------------------------------------------
  // Parameters
  // ---------------------------------------------------------------------------
  localparam DATA_WIDTH     = 32;
  localparam AXI_ADDR_WIDTH = 32;
  localparam integer BYTES_PER_BEAT = (DATA_WIDTH/8);

  // Verilog-2001 compatible clog2
  function integer CLOG2;
    input integer v;
    integer i;
    begin
      CLOG2 = 0;
      for (i = v-1; i > 0; i = i >> 1) CLOG2 = CLOG2 + 1;
    end
  endfunction
  localparam integer LG_BYTES = CLOG2(BYTES_PER_BEAT);

  // ---------------------------------------------------------------------------
  // Clocks / Reset (assumed synchronous)
  // ---------------------------------------------------------------------------
  reg clk, rst_n;

  initial begin
    clk = 0;
    forever #5 clk = ~clk; // 100 MHz
  end

  initial begin
    rst_n = 0;
    repeat (5) @(posedge clk);
    rst_n = 1;
  end

  // AXI clock/reset (n?i cùng h? th?ng)
  wire s_aclk    = clk;
  wire s_aresetn = rst_n;

  // ---------------------------------------------------------------------------
  // AXI signals
  // ---------------------------------------------------------------------------
  reg  [3:0]                 s_awid;
  reg  [AXI_ADDR_WIDTH-1:0]  s_awaddr;
  reg  [7:0]                 s_awlen;
  reg  [2:0]                 s_awsize;
  reg  [1:0]                 s_awburst;
  reg                        s_awvalid;
  wire                       s_awready;

  reg  [DATA_WIDTH-1:0]      s_wdata;
  reg  [DATA_WIDTH/8-1:0]    s_wstrb;
  reg                        s_wlast;
  reg                        s_wuser;
  reg                        s_wvalid;
  wire                       s_wready;

  wire [3:0]                 s_bid;
  wire [1:0]                 s_bresp;
  wire                       s_buser;
  wire                       s_bvalid;
  reg                        s_bready;

  reg  [3:0]                 s_arid;
  reg  [AXI_ADDR_WIDTH-1:0]  s_araddr;
  reg  [7:0]                 s_arlen;
  reg  [2:0]                 s_arsize;
  reg  [1:0]                 s_arburst;
  reg                        s_arvalid;
  wire                       s_arready;

  wire [3:0]                 s_rid;
  wire [DATA_WIDTH-1:0]      s_rdata;
  wire [1:0]                 s_rresp;
  wire                       s_rlast;
  wire                       s_ruser;
  wire                       s_rvalid;
  reg                        s_rready;

  // ---------------------------------------------------------------------------
  // CTRL / XIP_CFG / XIP_CMD
  // ---------------------------------------------------------------------------
  reg                        xip_en;
  wire                       xip_active;

  reg  [1:0]                 xip_cmd_lanes;
  reg  [1:0]                 xip_addr_lanes;
  reg  [1:0]                 xip_data_lanes;
  reg  [1:0]                 xip_addr_bytes;
  reg                        xip_mode_en;
  reg  [3:0]                 xip_dummy_cycles;
  reg                        xip_cont_read;
  reg                        xip_write_en;

  reg  [7:0]                 xip_read_op;
  reg  [7:0]                 xip_write_op;
  reg  [7:0]                 xip_mode_bits;

  // ---------------------------------------------------------------------------
  // RX FIFO (mô ph?ng 8-bit) + DONE
  // ---------------------------------------------------------------------------
  wire                       rx_empty;
  wire [7:0]                 rx_data;
  wire                       rx_ren;

  // Data generator theo cmd_len/cmd_addr c?a DUT
  reg  [31:0]                gen_total_bytes;
  reg  [31:0]                gen_sent_bytes;
  reg  [AXI_ADDR_WIDTH-1:0]  gen_base_addr;

  // >>> FIX 1: Không c?t bit tr?c ti?p trên bi?u th?c
  wire [AXI_ADDR_WIDTH-1:0]  rx_calc = gen_base_addr + gen_sent_bytes;

  assign rx_empty = (gen_sent_bytes >= gen_total_bytes);
  assign rx_data  = rx_calc[7:0];

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      gen_total_bytes <= 32'd0;
      gen_sent_bytes  <= 32'd0;
      gen_base_addr   <= {AXI_ADDR_WIDTH{1'b0}};
    end else begin
      if (dut.start) begin
        gen_total_bytes <= dut.cmd_len;
        gen_sent_bytes  <= 32'd0;
        gen_base_addr   <= dut.cmd_addr;
      end else if (rx_ren && !rx_empty) begin
        gen_sent_bytes <= gen_sent_bytes + 32'd1;
      end
    end
  end

  // DONE pulse khi ?ã xu?t ?? byte và RLAST v?a ???c ch?p nh?n
  reg done_int;
  wire done_w = done_int;
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      done_int <= 1'b0;
    end else begin
      done_int <= 1'b0;
      if ((gen_sent_bytes == gen_total_bytes) && s_rvalid && s_rready && s_rlast) begin
        done_int <= 1'b1;
      end
    end
  end

  // ---------------------------------------------------------------------------
  // Quan sát c?ng l?nh c?a DUT
  // ---------------------------------------------------------------------------
  wire        start_w    = dut.start;
  wire [15:0] cmd_cfg_w  = dut.cmd_cfg;
  wire [15:0] cmd_op_w   = dut.cmd_op;
  wire [31:0] cmd_len_w  = dut.cmd_len;
  wire [AXI_ADDR_WIDTH-1:0] cmd_addr_w = dut.cmd_addr;

  // ---------------------------------------------------------------------------
  // DUT Instance
  // ---------------------------------------------------------------------------
  xip #(
    .DATA_WIDTH     (DATA_WIDTH),
    .AXI_ADDR_WIDTH (AXI_ADDR_WIDTH)
  ) dut (
    .s_aclk      (s_aclk),
    .s_aresetn   (s_aresetn),

    .s_awid      (s_awid),
    .s_awaddr    (s_awaddr),
    .s_awlen     (s_awlen),
    .s_awsize    (s_awsize),
    .s_awburst   (s_awburst),
    .s_awvalid   (s_awvalid),
    .s_awready   (s_awready),
    .s_wdata     (s_wdata),
    .s_wstrb     (s_wstrb),
    .s_wlast     (s_wlast),
    .s_wuser     (s_wuser),
    .s_wvalid    (s_wvalid),
    .s_wready    (s_wready),
    .s_bid       (s_bid),
    .s_bresp     (s_bresp),
    .s_buser     (s_buser),
    .s_bvalid    (s_bvalid),
    .s_bready    (s_bready),

    .s_arid      (s_arid),
    .s_araddr    (s_araddr),
    .s_arlen     (s_arlen),
    .s_arsize    (s_arsize),
    .s_arburst   (s_arburst),
    .s_arvalid   (s_arvalid),
    .s_arready   (s_arready),
    .s_rid       (s_rid),
    .s_rdata     (s_rdata),
    .s_rresp     (s_rresp),
    .s_rlast     (s_rlast),
    .s_ruser     (s_ruser),
    .s_rvalid    (s_rvalid),
    .s_rready    (s_rready),

    .clk         (clk),
    .rst_n       (rst_n),

    .xip_en      (xip_en),
    .xip_active  (xip_active),

    .xip_cmd_lanes    (xip_cmd_lanes),
    .xip_addr_lanes   (xip_addr_lanes),
    .xip_data_lanes   (xip_data_lanes),
    .xip_addr_bytes   (xip_addr_bytes),
    .xip_mode_en      (xip_mode_en),
    .xip_dummy_cycles (xip_dummy_cycles),
    .xip_cont_read    (xip_cont_read),
    .xip_write_en     (xip_write_en),

    .xip_read_op      (xip_read_op),
    .xip_write_op     (xip_write_op),
    .xip_mode_bits    (xip_mode_bits),

    .rx_empty    (rx_empty),
    .rx_data     (rx_data),
    .rx_ren      (rx_ren),

    // >>> Dùng cú pháp .port() tr?ng (t??ng thích V2001)
    .start       (),
    .cmd_cfg     (),
    .cmd_op      (),
    .cmd_addr    (),
    .cmd_dummy   (),
    .cmd_len     (),
    .done        (done_w)
  );

  // ---------------------------------------------------------------------------
  // AXI Master BFM (??n gi?n)
  // ---------------------------------------------------------------------------
  task axi_read_burst;
    input [AXI_ADDR_WIDTH-1:0] addr;
    input [7:0] len; // beats-1
    input integer backpressure_gap;
    integer beat, gap;
    begin
      // Setup AR
      s_arid    <= 4'h3;
      s_araddr  <= addr;
      s_arlen   <= len;
      s_arsize  <= LG_BYTES[2:0];  // >>> FIX 2: không dùng $clog2
      s_arburst <= 2'b01; // INCR
      s_arvalid <= 1'b1;
      // Wait for AR handshake
      @(posedge clk);
      while (!s_arready) @(posedge clk);
      s_arvalid <= 1'b0;

      // Nh?n R beats
      for (beat=0; beat< (len+1); beat=beat+1) begin
        // backpressure
        for (gap=0; gap<backpressure_gap; gap=gap+1) begin
          s_rready <= 1'b0;
          @(posedge clk);
        end
        // s?n sàng nh?n
        s_rready <= 1'b1;
        // ch? RVALID
        @(posedge clk);
        while (!s_rvalid) @(posedge clk);
        // ki?m tra d? li?u
        check_rdata(addr, beat);
        // ch?t beat
        @(posedge clk);
        s_rready <= 1'b0;
      end
    end
  endtask

  // >>> FIX 3: Không c?t bit tr?c ti?p trên bi?u th?c trong task
  task check_rdata;
    input [AXI_ADDR_WIDTH-1:0] base;
    input integer beat_idx;
    reg [AXI_ADDR_WIDTH-1:0] calc;
    reg [7:0] b0,b1,b2,b3;
    reg [DATA_WIDTH-1:0] expected;
    begin
      calc = base + beat_idx*BYTES_PER_BEAT + 0; b0 = calc[7:0];
      calc = base + beat_idx*BYTES_PER_BEAT + 1; b1 = calc[7:0];
      calc = base + beat_idx*BYTES_PER_BEAT + 2; b2 = calc[7:0];
      calc = base + beat_idx*BYTES_PER_BEAT + 3; b3 = calc[7:0];
      expected = {b3,b2,b1,b0};
      if (s_rdata !== expected) begin
        $display("[%0t] ERROR: RDATA mismatch. exp=%h, got=%h (beat=%0d)", $time, expected, s_rdata, beat_idx);
        $stop;
      end
    end
  endtask

  // Check command fields khi DUT start (ch? c?nh báo/thông tin)
  always @(posedge clk) begin
    if (start_w) begin
      if (cmd_op_w[7:0] !== xip_read_op)
        $display("[%0t] WARN: CMD_OP opcode != xip_read_op", $time);
      if (cmd_cfg_w[13] !== 1'b1)
        $display("[%0t] ERROR: CMD_CFG.DIR should be 1 (READ)", $time);
      if (cmd_len_w !== ((s_arlen + 8'd1) * BYTES_PER_BEAT)) begin
        $display("[%0t] ERROR: CMD_LEN mismatch exp=%0d got=%0d", $time,
                 (s_arlen + 8'd1) * BYTES_PER_BEAT, cmd_len_w);
        $stop;
      end
    end
  end

  // ---------------------------------------------------------------------------
  // Write channel negative test: khi xip_write_en=0, không nh?n AW/W
  // ---------------------------------------------------------------------------
  task try_axi_write_nop;
    input [AXI_ADDR_WIDTH-1:0] addr;
    integer k;
    begin
      s_awid    <= 4'h1;
      s_awaddr  <= addr;
      s_awlen   <= 8'd0;
      s_awsize  <= LG_BYTES[2:0];
      s_awburst <= 2'b01;
      s_awvalid <= 1'b1;

      s_wdata   <= 32'hDEADBEEF;
      s_wstrb   <= {BYTES_PER_BEAT{1'b1}};
      s_wlast   <= 1'b1;
      s_wuser   <= 1'b0;
      s_wvalid  <= 1'b1;

      s_bready  <= 1'b1;

      for (k=0; k<5; k=k+1) @(posedge clk);

      if (s_awready || s_wready) begin
        $display("[%0t] ERROR: Write handshake happened while xip_write_en=0!", $time);
        $stop;
      end

      s_awvalid <= 1'b0;
      s_wvalid  <= 1'b0;
      s_bready  <= 1'b0;
    end
  endtask

  // ---------------------------------------------------------------------------
  // Stimulus
  // ---------------------------------------------------------------------------
  initial begin
    // default
    s_awid    = 0; s_awaddr=0; s_awlen=0; s_awsize=0; s_awburst=0; s_awvalid=0;
    s_wdata   = 0; s_wstrb ={BYTES_PER_BEAT{1'b0}}; s_wlast=0; s_wuser=0; s_wvalid=0;
    s_bready  = 0;

    s_arid    = 0; s_araddr=0; s_arlen=0; s_arsize=0; s_arburst=0; s_arvalid=0;
    s_rready  = 0;

    // XIP config: Fast Read 0x0B, 1-1-1, 3B, dummy 8
    xip_en           = 0;
    xip_cmd_lanes    = 2'b00;
    xip_addr_lanes   = 2'b00;
    xip_data_lanes   = 2'b00;
    xip_addr_bytes   = 2'b01; // 3 bytes
    xip_mode_en      = 1'b0;
    xip_dummy_cycles = 4'd8;
    xip_cont_read    = 1'b0;
    xip_write_en     = 1'b0; // write gate OFF

    xip_read_op      = 8'h0B;
    xip_write_op     = 8'h02;
    xip_mode_bits    = 8'h00;

    // Reset
    @(posedge rst_n);
    @(posedge clk);

    // B?t XIP
    xip_en = 1'b1;

    // 1) Negative test: write b? ch?n
    $display("\n[TB] WRITE negative test (xip_write_en=0)...");
    try_axi_write_nop(32'h0000_9000);

    // 2) Read 1 beat (len=0)
    $display("\n[TB] READ single-beat (len=0)...");
    axi_read_burst(32'h0000_1000, 8'd0, 0);

    // 3) Read 4 beats (len=3) v?i backpressure = 2 chu k?
    $display("\n[TB] READ burst 4 beats (len=3) with backpressure...");
    axi_read_burst(32'h0000_2000, 8'd3, 2);

    $display("\n[TB] All tests finished OK.");
    #50;
    $finish;
  end

endmodule