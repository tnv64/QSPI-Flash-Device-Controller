`timescale 1ns/1ps

module qspi_fsm_tb;

  // ================= Clock / Reset =================
  reg clk   = 1'b0;
  reg rst_n = 1'b0;
  real TCLK = 10.0; // 100 MHz
  always #(TCLK/2.0) clk = ~clk;

  initial begin
    rst_n = 1'b0;
    repeat (10) @(posedge clk);
    rst_n = 1'b1;
  end

  // ================= Wires QSPI bus =================
  wire sclk;
  wire cs_n;
  wire io0, io1, io2, io3;

  // ================= CSR/CTRL to DUT =================
  reg  [2:0] clk_div;
  reg        cs_auto, cs_level, quad_en, cpol, cpha, lsb_first;

  // Command bundle
  reg        start;
  wire       done;
  reg [15:0] cmd_cfg;       // [1:0]CMD, [3:2]ADDR, [5:4]DATA, [7:6]ADDR_BYTES, [8]MODE_EN, [12:9]DUMMY, [13]DIR
  reg [15:0] cmd_op;        // [7:0]=opcode, [15:8]=mode bits
  reg [31:0] cmd_addr;
  reg [7:0]  cmd_dummy;
  reg [31:0] cmd_len;

  // TX (write -> flash)
  reg        tx_ren;
  reg [7:0]  tx_data_fifo;
  wire       tx_empty;

  // RX (read <- flash)
  wire       rx_wen;
  wire [7:0] rx_data_fifo;
  reg        rx_full;

  // ================= Instantiate DUT =================
  qspi_fsm dut (
    .clk        (clk),
    .rst_n      (rst_n),

    .clk_div    (clk_div),
    .cs_auto    (cs_auto),
    .cs_level   (cs_level),
    .quad_en    (quad_en),
    .cpol       (cpol),
    .cpha       (cpha),
    .lsb_first  (lsb_first),

    .start      (start),
    .cmd_cfg    (cmd_cfg),
    .cmd_op     (cmd_op),
    .cmd_addr   (cmd_addr),
    .cmd_dummy  (cmd_dummy),
    .cmd_len    (cmd_len),
    .done       (done),

    .tx_ren       (tx_ren),
    .tx_data_fifo (tx_data_fifo),
    .tx_empty     (tx_empty),

    .rx_wen       (rx_wen),
    .rx_data_fifo (rx_data_fifo),
    .rx_full      (rx_full),

    .sclk       (sclk),
    .cs_n       (cs_n),
    .io0        (io0),
    .io1        (io1),
    .io2        (io2),
    .io3        (io3)
  );

  // ================= Instantiate Flash model =================
  qspi_device flash (
    .qspi_sclk (sclk),
    .qspi_cs_n (cs_n),
    .qspi_io0  (io0),
    .qspi_io1  (io1),
    .qspi_io2  (io2),
    .qspi_io3  (io3)
  );

  // ================= Defaults =================
  initial begin
    // CTRL
    clk_div    = 3'd1; // sclk = clk/2
    cs_auto    = 1'b1;
    cs_level   = 1'b1; // ch? dùng khi cs_auto=0
    quad_en    = 1'b0;
    cpol       = 1'b0;
    cpha       = 1'b0;
    lsb_first  = 1'b0;

    // CMD
    start      = 1'b0;
    cmd_cfg    = 16'h0000;
    cmd_op     = 16'h0000;
    cmd_addr   = 32'h0000_0000;
    cmd_dummy  = 8'h00;
    cmd_len    = 32'd0;

    // Data bridges
    tx_ren       = 1'b0;
    tx_data_fifo = 8'h00;
    rx_full      = 1'b0;    // testbench không backpressure
  end

  // ================= RX buffer =================
  reg [7:0] rx_buf [0:255];
  integer   rx_wr_ptr;

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) rx_wr_ptr <= 0;
    else if (rx_wen) begin
      if (rx_wr_ptr < 256) rx_buf[rx_wr_ptr] <= rx_data_fifo;
      rx_wr_ptr <= (rx_wr_ptr < 256) ? (rx_wr_ptr + 1) : rx_wr_ptr;
    end
  end

  task clear_rx; begin rx_wr_ptr = 0; end endtask

  // ================= TX pattern for Page Program =================
  reg [7:0] pattern [0:31];
  integer   pidx;

  // C?p byte cho DUT khi DUT còn ?ang c?n (tx_empty==0)
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      tx_ren       <= 1'b0;
      tx_data_fifo <= 8'h00;
      pidx         <= 0;
    end else begin
      tx_ren <= 1'b0;
      if (tx_empty == 1'b0) begin
        // nh? d?n t?ng byte
        tx_data_fifo <= pattern[pidx];
        tx_ren       <= 1'b1;                  // pulse 1 chu k?
        if (pidx < 31) pidx <= pidx + 1;
      end
    end
  end

  // ================= Helpers =================
  task pulse_start; begin
    @(posedge clk); start <= 1'b1;
    @(posedge clk); start <= 1'b0;
  end endtask

  task wait_done; input integer maxcyc; integer t; begin
    t=0;
    while (done !== 1'b1 && t < maxcyc) begin @(posedge clk); t=t+1; end
    if (done !== 1'b1) begin
      $display("[%0t] ERROR: TIMEOUT wait done", $time);
      $finish;
    end
    @(posedge clk);
  end endtask

  // cmd_cfg templates
  // 1-1-1, không ??a ch?, không dummy, READ
  task cfg_single_noaddr_read; begin
    cmd_cfg[15:14]=2'b00;
    cmd_cfg[13]   =1'b1;   // DIR=READ
    cmd_cfg[12:9] =4'd0;   // DUMMY
    cmd_cfg[8]    =1'b0;   // MODE_EN
    cmd_cfg[7:6]  =2'd0;   // ADDR_BYTES=0
    cmd_cfg[5:4]  =2'd0;   // DATA lanes=1
    cmd_cfg[3:2]  =2'd0;   // ADDR lanes=1
    cmd_cfg[1:0]  =2'd0;   // CMD lanes=1
    cmd_dummy     = 8'd0;
    cmd_addr      = 32'h0;
  end endtask

  // 1-1-1, 3B address (??i DIR theo nhu c?u)
  task cfg_1_1_1_3B; begin
    cmd_cfg[15:14]=2'b00;
    cmd_cfg[13]   =1'b1;   // m?c ??nh READ (??i sang 0 ? l?nh WRITE)
    cmd_cfg[12:9] =4'd0;
    cmd_cfg[8]    =1'b0;
    cmd_cfg[7:6]  =2'd1;   // 3 bytes
    cmd_cfg[5:4]  =2'd0;   // DATA 1
    cmd_cfg[3:2]  =2'd0;   // ADDR 1
    cmd_cfg[1:0]  =2'd0;   // CMD  1
    cmd_dummy     = 8'd0;
  end endtask

  // In d? li?u
  task print_rx; input integer n; integer k; begin
    $write("  RX:");
    for (k=0;k<n;k=k+1) $write(" %02x", rx_buf[k]);
    $write("\n");
  end endtask

  // ================= 3 TASK NG?N G?N: WRDI / RDSR / RDID =================

  // --- G?i WRDI (0x04): write, opcode-only ---
  task send_wrdi;
  begin
    // DIR=WRITE, không addr/mode/dummy/data
    cmd_cfg[15:14]=2'b00;
    cmd_cfg[13]   =1'b0;   // WRITE
    cmd_cfg[12:9] =4'd0;
    cmd_cfg[8]    =1'b0;   // MODE_EN=0
    cmd_cfg[7:6]  =2'd0;   // ADDR_BYTES=0
    cmd_cfg[5:4]  =2'd0;   // DATA 1-bit (ignored)
    cmd_cfg[3:2]  =2'd0;   // ADDR 1-bit (ignored)
    cmd_cfg[1:0]  =2'd0;   // CMD  1-bit
    cmd_op[15:8]  = 8'h00; // MODE bits = 0
    cmd_op[7:0]   = 8'h04; // WRDI
    cmd_len       = 32'd0; // opcode-only
    cmd_dummy     = 8'd0;
    pulse_start(); wait_done(100000);
  end
  endtask

  // --- G?i RDSR (0x05): read status, ??c nbytes (m?c ??nh 1) ---
  task send_rdsr; input [31:0] nbytes;
  begin
    cfg_single_noaddr_read(); // READ, 1-1-1, no addr/mode/dummy
    cmd_op[15:8] = 8'h00;
    cmd_op[7:0]  = 8'h05;
    cmd_len      = (nbytes==0) ? 32'd1 : nbytes;
    clear_rx(); pulse_start(); wait_done(100000);
  end
  endtask

  // --- G?i RDID (0x9F): read ID, ??c nbytes (m?c ??nh 3) ---
  task send_rdid; input [31:0] nbytes;
  begin
    cfg_single_noaddr_read(); // READ, 1-1-1, no addr/mode/dummy
    cmd_op[15:8] = 8'h00;
    cmd_op[7:0]  = 8'h9F;
    cmd_len      = (nbytes==0) ? 32'd3 : nbytes;
    clear_rx(); pulse_start(); wait_done(100000);
  end
  endtask

  // ================= Poll RDSR.WIP = 0 =================
  task poll_WIP_clear;
    integer loopc;
    reg [7:0] sr;
  begin
    loopc = 0;
    while (loopc < 50000) begin
      send_rdsr(32'd1); // sau khi return, rx_buf[0] ?ã có SR
      sr = rx_buf[0];
      if (sr[0] == 1'b0) begin
        disable poll_WIP_clear; // thoát task khi WIP=0
      end
      loopc = loopc + 1;
    end
    $display("[%0t] ERROR: WIP not clear", $time);
    $finish;
  end
  endtask

  // ================= Test sequence =================
  integer i;

  initial begin
    $dumpfile("tb_qspi_fsm.vcd");
    $dumpvars(0, qspi_fsm_tb);

    // ch? reset
    wait (rst_n==1'b1);
    repeat (5) @(posedge clk);

    // ----- RDID 0x9F -----
    $display("\n[TEST] RDID 0x9F");
    send_rdid(32'd3);             // ho?c send_rdid(0) m?c ??nh 3 byte
    print_rx(3);

    // ----- RDSR 0x05 -----
    $display("\n[TEST] RDSR 0x05");
    send_rdsr(32'd1);             // ho?c send_rdsr(0) m?c ??nh 1 byte
    print_rx(1);

    // ----- WRDI 0x04 -----
    $display("\n[TEST] WRDI 0x04");
    send_wrdi();
    $display("[OK] WRDI done");

    // ----- WREN 0x06 -----
    $display("\n[TEST] WREN 0x06");
    // L?nh th??ng: WRITE opcode-only
    cmd_cfg[13] = 1'b0;           // WRITE
    cmd_op[15:8]= 8'h00;
    cmd_op[7:0] = 8'h06;          // WREN
    cmd_len     = 32'd0;
    pulse_start(); wait_done(100000);

    // ----- PAGE PROGRAM 0x02 (16 bytes @ 0x001000) -----
    $display("\n[TEST] PAGE PROGRAM 0x02 (16B @ 0x001000)");
    // chu?n b? pattern
    for (i=0;i<32;i=i+1) pattern[i] = 8'hA0 + i[7:0];
    pidx = 0;

    cfg_1_1_1_3B();
    cmd_cfg[13] = 1'b0;           // WRITE
    cmd_op[15:8]= 8'h00;
    cmd_op[7:0] = 8'h02;
    cmd_addr    = 32'h00001000;
    cmd_len     = 32'd16;
    pulse_start(); wait_done(200000);

    // ch? WIP clear
    poll_WIP_clear();

    // ----- READ 0x03 verify 16 bytes -----
    $display("\n[TEST] READ 0x03 verify");
    cfg_1_1_1_3B();
    cmd_cfg[13] = 1'b1;           // READ
    cmd_op[15:8]= 8'h00;
    cmd_op[7:0] = 8'h03;
    cmd_addr    = 32'h00001000;
    cmd_len     = 32'd16;
    clear_rx(); pulse_start(); wait_done(200000);
    print_rx(16);
    for (i=0;i<16;i=i+1) begin
      if (rx_buf[i] !== (8'hA0 + i[7:0])) begin
        $display("[FAIL] Mismatch @%0d: got %02x exp %02x", i, rx_buf[i], (8'hA0 + i[7:0]));
        $finish;
      end
    end
    $display("[PASS] Page Program verified OK");

    $display("\n=== BASIC TESTS (RDID/RDSR/WRDI + PROG/READ) PASSED ===");
    #100;
    $finish;
  end

endmodule
