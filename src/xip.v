module xip #(
  parameter DATA_WIDTH     = 32,
  parameter AXI_ADDR_WIDTH = 32
)(

  input                   s_aclk,
  input                   s_aresetn,

  // AXI4 Write Address
  input  [3:0]            s_awid,
  input  [AXI_ADDR_WIDTH-1:0] s_awaddr,
  input  [7:0]            s_awlen,
  input  [2:0]            s_awsize,
  input  [1:0]            s_awburst,
  input                   s_awvalid,
  output reg              s_awready,
  // AXI4 Write Data
  input  [DATA_WIDTH-1:0] s_wdata,
  input  [DATA_WIDTH/8-1:0] s_wstrb,
  input                   s_wlast,
  input                   s_wuser,
  input                   s_wvalid,
  output reg              s_wready,
  // AXI4 Write Response
  output reg [3:0]        s_bid,
  output reg [1:0]        s_bresp,
  output reg              s_buser,
  output reg              s_bvalid,
  input                   s_bready,

  // AXI4 Read Address
  input  [3:0]            s_arid,
  input  [AXI_ADDR_WIDTH-1:0] s_araddr,
  input  [7:0]            s_arlen,
  input  [2:0]            s_arsize,
  input  [1:0]            s_arburst,
  input                   s_arvalid,
  output reg              s_arready,
  // AXI4 Read Data
  output reg [3:0]        s_rid,
  output reg [DATA_WIDTH-1:0] s_rdata,
  output reg [1:0]        s_rresp,
  output reg              s_rlast,
  output reg              s_ruser,
  output reg              s_rvalid,
  input                   s_rready,

  // System clock/reset
  input                   clk,
  input                   rst_n,

  // CTRL / STATUS
  input                   xip_en,
  output reg              xip_active,

  // XIP_CFG @0x01C (tách field)
  input  [1:0]            xip_cmd_lanes,
  input  [1:0]            xip_addr_lanes,
  input  [1:0]            xip_data_lanes,
  input  [1:0]            xip_addr_bytes,
  input                   xip_mode_en,
  input  [3:0]            xip_dummy_cycles,
  input                   xip_cont_read,
  input                   xip_write_en,

  // XIP_CMD @0x020 (tách field)
  input  [7:0]            xip_read_op,
  input  [7:0]            xip_write_op,   
  input  [7:0]            xip_mode_bits,

  // RX FIFO t? QSPI FSM (8-bit)
  input                   rx_empty,
  input  [7:0]            rx_data,
  output reg              rx_ren,

  // Giao ti?p QSPI FSM (??c)
  output reg              start,
  output reg [15:0]       cmd_cfg,     // [13]=DIR,[12:9]=DUMMY,[8]=MODE_EN,[7:6]=ADDR_BYTES,[5:4]=DATA,[3:2]=ADDR,[1:0]=CMD
  output reg [15:0]       cmd_op,      // {MODE_BITS, OPCODE}
  output reg [AXI_ADDR_WIDTH-1:0] cmd_addr,
  output reg [7:0]        cmd_dummy,
  output reg [31:0]       cmd_len,
  input                   done
);

  localparam integer BYTES_PER_BEAT = (DATA_WIDTH/8);
  localparam ST_IDLE     = 3'd0,
             ST_STREAM   = 3'd1,
             ST_RETURN   = 3'd2,
             ST_WAITDONE = 3'd3;

  reg [2:0] state, nstate;
  // ========================== WRITE CHANNEL (sink, gated) =====================
  reg       aw_got;
  reg [3:0] awid_q;

  wire write_accept = xip_en && xip_write_en && (state==ST_IDLE); 
  wire aw_fire      = s_awvalid & s_awready;
  wire w_fire       = s_wvalid  & s_wready;

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      s_awready <= 1'b0;
      s_wready  <= 1'b0;
      s_bvalid  <= 1'b0;
      s_bresp   <= 2'b00;
      s_buser   <= 1'b0;
      s_bid     <= 4'd0;
      aw_got    <= 1'b0;
      awid_q    <= 4'd0;
    end else begin

      s_awready <= write_accept && ~aw_got;
      s_wready  <= write_accept && aw_got;

      // Latch AW
      if (aw_fire) begin
        aw_got <= 1'b1;
        awid_q <= s_awid;
      end

      if (!s_bvalid && aw_got && w_fire && s_wlast) begin
        s_bvalid <= 1'b1;
        s_bid    <= awid_q;
        s_bresp  <= 2'b00; // OKAY
        s_buser  <= 1'b0;
        aw_got   <= 1'b0;  
      end

      if (s_bvalid && s_bready) begin
        s_bvalid <= 1'b0;
      end
    end
  end

  // ============================ READ ENGINE (burst) ===========================
  

  reg [3:0]                arid_q;
  reg [AXI_ADDR_WIDTH-1:0] araddr_q;
  reg [7:0]                arlen_q;

  reg [7:0]                beat_total;  // = ARLEN+1
  reg [7:0]                beat_cnt;
  reg [5:0]                need_bytes;  

  reg [DATA_WIDTH-1:0]     data_shift;

  wire ar_fire = s_arvalid & s_arready;

  // Seq
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      state      <= ST_IDLE;

      s_arready  <= 1'b0;
      s_rvalid   <= 1'b0;
      s_rdata    <= {DATA_WIDTH{1'b0}};
      s_rresp    <= 2'b00;
      s_rid      <= 4'd0;
      s_ruser    <= 1'b0;
      s_rlast    <= 1'b0;

      start      <= 1'b0;
      cmd_cfg    <= 16'h0000;
      cmd_op     <= 16'h0000;
      cmd_addr   <= {AXI_ADDR_WIDTH{1'b0}};
      cmd_dummy  <= 8'h00;
      cmd_len    <= 32'd0;

      xip_active <= 1'b0;
      rx_ren     <= 1'b0;

      arid_q     <= 4'd0;
      araddr_q   <= {AXI_ADDR_WIDTH{1'b0}};
      arlen_q    <= 8'd0;

      beat_total <= 8'd0;
      beat_cnt   <= 8'd0;
      need_bytes <= 6'd0;
      data_shift <= {DATA_WIDTH{1'b0}};
    end else begin
      state     <= nstate;
      s_arready <= 1'b0;
      s_rvalid  <= 1'b0;
      s_rresp   <= 2'b00; 
      s_ruser   <= 1'b0;
      s_rlast   <= 1'b0;
      start     <= 1'b0;
      rx_ren    <= 1'b0;

      case (state)
        // -------------------------------------------------
        ST_IDLE: begin
          xip_active <= 1'b0;

          if (xip_en) begin
            s_arready <= 1'b1;
            if (ar_fire) begin
              // Latch AR
              arid_q    <= s_arid;
              araddr_q  <= s_araddr;
              arlen_q   <= s_arlen;

              beat_total <= s_arlen + 8'd1;
              beat_cnt   <= 8'd0;

              // C?u hình QSPI read cho toàn burst
              cmd_cfg   <= { 2'b00,
                             1'b1 /*DIR=READ*/,
                             xip_dummy_cycles,
                             xip_mode_en,
                             xip_addr_bytes,
                             xip_data_lanes,
                             xip_addr_lanes,
                             xip_cmd_lanes };

              cmd_op    <= { xip_mode_bits, xip_read_op };
              cmd_addr  <= s_araddr;
              cmd_dummy <= 8'h00;
              cmd_len   <= (s_arlen + 8'd1) * BYTES_PER_BEAT;

              start      <= 1'b1;
              xip_active <= 1'b1;
              data_shift <= {DATA_WIDTH{1'b0}};
              need_bytes <= BYTES_PER_BEAT[5:0];
            end
          end
        end

        // -------------------------------------------------
        ST_STREAM: begin

          if (!rx_empty && (need_bytes != 6'd0)) begin
            rx_ren     <= 1'b1; // pulse 1 chu k?
            data_shift <= { rx_data, data_shift[DATA_WIDTH-1:8] };
            need_bytes <= need_bytes - 6'd1;
          end
        end

        // -------------------------------------------------
        ST_RETURN: begin
          s_rvalid <= 1'b1;
          s_rdata  <= data_shift;
          s_rid    <= arid_q;
          s_rresp  <= 2'b00; // OKAY
          s_rlast  <= (beat_cnt == (beat_total - 8'd1));
        end

        // -------------------------------------------------
        ST_WAITDONE: begin
 
          if (done) begin
            if (!xip_cont_read) xip_active <= 1'b0;
          end
        end
      endcase
    end
  end

  always @(*) begin
    nstate = state;
    case (state)
      ST_IDLE:     nstate = (xip_en && ar_fire) ? ST_STREAM : ST_IDLE;
      ST_STREAM:   nstate = (need_bytes==6'd0)  ? ST_RETURN : ST_STREAM;
      ST_RETURN:   nstate = (s_rready) ? ((beat_cnt==(beat_total-8'd1)) ? ST_WAITDONE : ST_STREAM)
                                       : ST_RETURN;
      ST_WAITDONE: nstate = done ? ST_IDLE : ST_WAITDONE;
      default:     nstate = ST_IDLE;
    endcase
  end

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      beat_cnt   <= 8'd0;
      need_bytes <= 6'd0;
      data_shift <= {DATA_WIDTH{1'b0}};
    end else begin
      if (state==ST_RETURN && s_rvalid && s_rready) begin
        beat_cnt   <= beat_cnt + 8'd1;
        need_bytes <= BYTES_PER_BEAT[5:0];
        data_shift <= {DATA_WIDTH{1'b0}};
      end
    end
  end

endmodule