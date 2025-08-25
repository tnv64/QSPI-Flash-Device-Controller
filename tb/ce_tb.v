`timescale 1ns/1ps

module ce_tb;

  // ===== Clock & Reset =====
  reg clk = 1'b0;
  reg rst_n = 1'b0;
  real TCLK = 10.0; // 100MHz
  always #(TCLK/2.0) clk = ~clk;

  // ===== DUT I/Os =====
  reg  cmd_trigger;
  reg  dma_en;
  reg  done_qspi;
  reg  dma_done;

  wire start_qspi;
  wire dma_start;
  wire cmd_done;
  wire clear_cmd;
  wire busy;

  // ===== Instantiate DUT =====
  // Ch?nh tên/module mapping cho ?úng v?i module CE c?a b?n n?u khác
  ce dut (
    .clk        (clk),
    .rst_n      (rst_n),

    // CSR side
    .cmd_trigger(cmd_trigger),
    .dma_en     (dma_en),
    .clear_cmd  (clear_cmd),
    .cmd_done   (cmd_done),
    .busy       (busy),

    // DMA side
    .dma_done   (dma_done),
    .dma_start  (dma_start),

    // QSPI FSM side
    .done_qspi  (done_qspi),
    .start_qspi (start_qspi)
  );

  // ===== Variables for TB control / check =====
  integer cycles;
  integer errors;
  reg     seen_start_qspi;
  reg     seen_dma_start;
  reg     seen_clear_cmd;
  reg     seen_cmd_done;

  // ===== Simple monitors =====
  // Ghi nh?n các xung ?? d? assert
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      seen_start_qspi <= 1'b0;
      seen_dma_start  <= 1'b0;
      seen_clear_cmd  <= 1'b0;
      seen_cmd_done   <= 1'b0;
    end else begin
      if (start_qspi) seen_start_qspi <= 1'b1;
      if (dma_start ) seen_dma_start  <= 1'b1;
      if (clear_cmd ) seen_clear_cmd  <= 1'b1;
      if (cmd_done  ) seen_cmd_done   <= 1'b1;
    end
  end

  // ===== Tasks =====

  task apply_reset;
    integer i;
    begin
      rst_n = 1'b0;
      cmd_trigger = 1'b0;
      dma_en      = 1'b0;
      done_qspi   = 1'b0;
      dma_done    = 1'b0;
      seen_start_qspi = 1'b0;
      seen_dma_start  = 1'b0;
      seen_clear_cmd  = 1'b0;
      seen_cmd_done   = 1'b0;
      errors = 0;
      for (i=0;i<10;i=i+1) @(posedge clk);
      rst_n = 1'b1;
      @(posedge clk);
    end
  endtask

  task pulse;  // t?o xung 1 chu k?
    output reg sig;
    begin
      sig = 1'b1; @(posedge clk);
      sig = 1'b0; @(posedge clk);
    end
  endtask

  task wait_cycles;
    input integer n;
    integer k;
    begin
      for (k=0;k<n;k=k+1) @(posedge clk);
    end
  endtask

  task wait_until_or_timeout; // ch? m?t ?i?u ki?n x?y ra, có timeout
    input integer max_cyc;
    input       cond;
    integer     t;
    begin
      t = 0;
      while ((cond === 1'b0) && (t < max_cyc)) begin
        @(posedge clk);
        t = t + 1;
      end
      if (cond === 1'b0) begin
        $display("[%0t] TIMEOUT waiting condition", $time);
        errors = errors + 1;
      end
    end
  endtask

  // Ki?m tra rule: cmd_done = done_qspi khi !dma_en
  //                cmd_done = done_qspi & dma_done khi dma_en
  task check_cmd_done_rule_no_dma; // tr??ng h?p không DMA
    begin
      if (cmd_done !== (done_qspi)) begin
        $display("[%0t] ERROR: cmd_done != done_qspi (no-DMA case)", $time);
        errors = errors + 1;
      end
    end
  endtask

  task check_cmd_done_rule_dma; // tr??ng h?p có DMA
    begin
      if (cmd_done !== (done_qspi & dma_done)) begin
        $display("[%0t] ERROR: cmd_done != (done_qspi & dma_done) (DMA case)", $time);
        errors = errors + 1;
      end
    end
  endtask

  // ===== Test Scenarios =====

  // 1) Command without DMA
  task scenario_no_dma_basic;
    begin
      $display("\n[SCENARIO] No-DMA basic");
      // clear flags
      seen_start_qspi = 1'b0;
      seen_dma_start  = 1'b0;
      seen_clear_cmd  = 1'b0;
      seen_cmd_done   = 1'b0;

      // Configure: no DMA
      dma_en    = 1'b0;
      done_qspi = 1'b0;
      dma_done  = 1'b0;

      // Trigger command
      pulse(cmd_trigger);

      // DUT ph?i phát start_qspi (1 chu k?) và clear_cmd (1 chu k?)
      wait_until_or_timeout(50, seen_start_qspi);
      wait_until_or_timeout(50, seen_clear_cmd);

      // Không ???c phát dma_start
      if (seen_dma_start) begin
        $display("[%0t] ERROR: dma_start asserted in no-DMA scenario", $time);
        errors = errors + 1;
      end

      // K?t thúc QSPI sau m?t lúc
      wait_cycles(5);
      pulse(done_qspi);

      // Ki?m tra rule cmd_done
      wait_until_or_timeout(20, seen_cmd_done);
      check_cmd_done_rule_no_dma();
    end
  endtask

  // 2) Command with DMA; QSPI done first, then DMA done (tr?)
  task scenario_dma_qspi_first_dma_later;
    begin
      $display("\n[SCENARIO] DMA enabled: QSPI done first, DMA later");
      // clear flags
      seen_start_qspi = 1'b0;
      seen_dma_start  = 1'b0;
      seen_clear_cmd  = 1'b0;
      seen_cmd_done   = 1'b0;

      dma_en    = 1'b1;
      done_qspi = 1'b0;
      dma_done  = 1'b0;

      pulse(cmd_trigger);

      // c? hai start ph?i xu?t hi?n
      wait_until_or_timeout(50, seen_start_qspi);
      wait_until_or_timeout(50, seen_dma_start);
      wait_until_or_timeout(50, seen_clear_cmd);

      // K?t thúc QSPI tr??c
      wait_cycles(5);
      pulse(done_qspi);

      // Ch?a cmd_done vì còn ch? dma_done
      wait_cycles(10);
      if (seen_cmd_done) begin
        $display("[%0t] ERROR: cmd_done asserted before dma_done in DMA case", $time);
        errors = errors + 1;
      end
      check_cmd_done_rule_dma();

      // DMA done sau
      pulse(dma_done);

      // Bây gi? cmd_done ph?i lên
      wait_until_or_timeout(20, seen_cmd_done);
      check_cmd_done_rule_dma();
    end
  endtask

  // 3) Command with DMA; DMA done first, r?i QSPI done (??o th? t?)
  task scenario_dma_dma_first_qspi_later;
    begin
      $display("\n[SCENARIO] DMA enabled: DMA done first, QSPI later");
      // clear flags
      seen_start_qspi = 1'b0;
      seen_dma_start  = 1'b0;
      seen_clear_cmd  = 1'b0;
      seen_cmd_done   = 1'b0;

      dma_en    = 1'b1;
      done_qspi = 1'b0;
      dma_done  = 1'b0;

      pulse(cmd_trigger);

      wait_until_or_timeout(50, seen_start_qspi);
      wait_until_or_timeout(50, seen_dma_start);
      wait_until_or_timeout(50, seen_clear_cmd);

      // DMA done tr??c
      wait_cycles(5);
      pulse(dma_done);

      // Ch?a cmd_done vì còn ch? done_qspi
      wait_cycles(10);
      if (seen_cmd_done) begin
        $display("[%0t] ERROR: cmd_done asserted before done_qspi in DMA case", $time);
        errors = errors + 1;
      end
      check_cmd_done_rule_dma();

      // QSPI done sau
      pulse(done_qspi);

      // Bây gi? cmd_done ph?i lên
      wait_until_or_timeout(20, seen_cmd_done);
      check_cmd_done_rule_dma();
    end
  endtask

  // ===== Testbench Top Sequence =====
  initial begin
    $dumpfile("ce_tb.vcd");
    $dumpvars(0, ce_tb);

    apply_reset();

    // Ch?y các k?ch b?n
    scenario_no_dma_basic();
    scenario_dma_qspi_first_dma_later();
    scenario_dma_dma_first_qspi_later();

    // T?ng k?t
    if (errors == 0) $display("\n=== CE TB: ALL TESTS PASSED ===");
    else             $display("\n=== CE TB: TESTS FAILED, errors=%0d ===", errors);

    #50;
    $finish;
  end

endmodule
