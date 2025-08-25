module axi4_cpu_master #(
  parameter integer DATA_WIDTH     = 32,
  parameter integer AXI_ADDR_WIDTH = 32,
  parameter [3:0]   FIXED_ARID     = 4'd0
)(
  input                          clk,
  input                          rst_n,

  // --------- Tiny CPU command interface ----------
  input                          cmd_valid,           // request a burst
  output                         cmd_ready,           // master can accept
  input      [AXI_ADDR_WIDTH-1:0] cmd_addr,           // start address (aligned to DATA_WIDTH/8)
  input      [7:0]               cmd_beats,           // number of beats (1..256)

  // Returned data stream
  output reg [DATA_WIDTH-1:0]    rd_data,
  output reg                     rd_valid,
  input                          rd_ready,
  output reg                     rd_last,

  output                         busy,                // 1 while a burst is in-flight
  output reg                     err,                 // pulses if RRESP != OKAY on any beat

  // ================= AXI4 Master (to XIP slave) =================
  // Write Address (unused)
  output [3:0]                   m_awid,
  output [AXI_ADDR_WIDTH-1:0]    m_awaddr,
  output [7:0]                   m_awlen,
  output [2:0]                   m_awsize,
  output [1:0]                   m_awburst,
  output                         m_awvalid,
  input                          m_awready,

  // Write Data (unused)
  output [DATA_WIDTH-1:0]        m_wdata,
  output [DATA_WIDTH/8-1:0]      m_wstrb,
  output                         m_wlast,
  output                         m_wuser,
  output                         m_wvalid,
  input                          m_wready,

  // Write Response (unused, but consumed)
  input  [3:0]                   m_bid,
  input  [1:0]                   m_bresp,
  input                          m_buser,
  input                          m_bvalid,
  output                         m_bready,

  // Read Address
  output reg [3:0]               m_arid,
  output reg [AXI_ADDR_WIDTH-1:0] m_araddr,
  output reg [7:0]               m_arlen,
  output reg [2:0]               m_arsize,
  output reg [1:0]               m_arburst,
  output reg                     m_arvalid,
  input                          m_arready,

  // Read Data
  input  [3:0]                   m_rid,
  input  [DATA_WIDTH-1:0]        m_rdata,
  input  [1:0]                   m_rresp,
  input                          m_rlast,
  input                          m_ruser,
  input                          m_rvalid,
  output                         m_rready
);

  // log2(DATA_WIDTH/8) with a simple function for Verilog-2001
  function [31:0] clog2;
    input [31:0] val;
    integer i;
    begin
      clog2 = 0;
      for (i=0; (1<<i) < val; i=i+1) clog2 = i+1;
    end
  endfunction

  localparam integer BYTES_PER_BEAT = (DATA_WIDTH/8);
  localparam [2:0]   RSIZE = clog2(BYTES_PER_BEAT); // ARSIZE

  // AXI constants
  localparam [1:0] BURST_INCR = 2'b01;
  localparam [1:0] RESP_OKAY  = 2'b00;

  localparam S_IDLE   = 2'd0;
  localparam S_SENDAR = 2'd1;
  localparam S_RECV   = 2'd2;

  reg [1:0]  state, nstate;
  reg [7:0]  beats_left;     // counts down remaining beats to pop to CPU

  // 1-beat skid buffer for R channel
  reg                rbuf_valid;
  reg [DATA_WIDTH-1:0] rbuf_data;
  reg                rbuf_last;

  // Command accept
  assign cmd_ready = (state == S_IDLE);
  assign busy      = (state != S_IDLE);

  // Unused write channel: tie-off safely
  assign m_awid    = 4'd0;
  assign m_awaddr  = {AXI_ADDR_WIDTH{1'b0}};
  assign m_awlen   = 8'd0;
  assign m_awsize  = RSIZE;
  assign m_awburst = BURST_INCR;
  assign m_awvalid = 1'b0;

  assign m_wdata   = {DATA_WIDTH{1'b0}};
  assign m_wstrb   = {DATA_WIDTH/8{1'b0}};
  assign m_wlast   = 1'b0;
  assign m_wuser   = 1'b0;
  assign m_wvalid  = 1'b0;

  assign m_bready  = 1'b1; // consume if any stray response shows up

  // R channel ready: only accept when skid buffer is empty
  assign m_rready = ~rbuf_valid;

  always @(*) begin
    nstate = state;
    case (state)
      S_IDLE:   nstate = (cmd_valid && cmd_ready) ? S_SENDAR : S_IDLE;

      S_SENDAR: nstate = (m_arvalid && m_arready) ? S_RECV : S_SENDAR;

      S_RECV: begin
        // Stay here until we have delivered the last beat to CPU
        // (i.e., beats_left will be decremented on CPU pop)
        if ((beats_left == 8'd0) && ~rbuf_valid) nstate = S_IDLE;
        else                                     nstate = S_RECV;
      end

      default:  nstate = S_IDLE;
    endcase
  end

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      state      <= S_IDLE;

      // AR defaults
      m_arid     <= FIXED_ARID;
      m_araddr   <= {AXI_ADDR_WIDTH{1'b0}};
      m_arlen    <= 8'd0;
      m_arsize   <= RSIZE;
      m_arburst  <= BURST_INCR;
      m_arvalid  <= 1'b0;

      beats_left <= 8'd0;

      rbuf_valid <= 1'b0;
      rbuf_data  <= {DATA_WIDTH{1'b0}};
      rbuf_last  <= 1'b0;

      rd_data    <= {DATA_WIDTH{1'b0}};
      rd_valid   <= 1'b0;
      rd_last    <= 1'b0;

      err        <= 1'b0;
    end else begin
      state    <= nstate;

      // Default output strobes
      rd_valid <= 1'b0;     // will be asserted when we pop skid buffer to CPU
      rd_last  <= 1'b0;
      err      <= 1'b0;

      // ================= IDLE =================
      if (state == S_IDLE) begin
        // Prepare a new AR when command accepted
        if (cmd_valid && cmd_ready) begin
          m_arid    <= FIXED_ARID;
          m_araddr  <= cmd_addr;
          m_arlen   <= (cmd_beats == 8'd0) ? 8'd0 : (cmd_beats - 8'd1); // AXI len = beats-1
          m_arsize  <= RSIZE;
          m_arburst <= BURST_INCR;
          m_arvalid <= 1'b1;

          beats_left <= (cmd_beats == 8'd0) ? 8'd0 : cmd_beats;
        end else begin
          m_arvalid <= 1'b0;
        end

        // buffer empty
        rbuf_valid <= 1'b0;
      end

      // ================= SEND AR =================
      if (state == S_SENDAR) begin
        // keep ARVALID high until handshake
        if (m_arvalid && m_arready)
          m_arvalid <= 1'b0;
      end

      // ================= RECV (R channel) =================
      if (state == S_RECV) begin
        // Accept from slave into 1-beat skid buffer when empty
        if (~rbuf_valid && m_rvalid && m_rready) begin
          rbuf_data  <= m_rdata;
          rbuf_last  <= m_rlast;
          rbuf_valid <= 1'b1;

          // capture error if any beat says not OKAY
          if (m_rresp != RESP_OKAY)
            err <= 1'b1;
        end

        // Pop to CPU when CPU ready and we have data
        if (rbuf_valid && rd_ready) begin
          rd_data    <= rbuf_data;
          rd_valid   <= 1'b1;
          rd_last    <= rbuf_last;

          rbuf_valid <= 1'b0;

          // manage remaining beat counter on successful CPU pop
          if (beats_left != 8'd0)
            beats_left <= beats_left - 8'd1;
        end
      end
    end
  end

endmodule
