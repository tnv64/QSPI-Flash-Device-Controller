module csr #(
  parameter APB_ADDR_WIDTH = 12
)(
  // APB Interface
  input  wire                      pclk,
  input  wire                      presetn,
  input  wire [APB_ADDR_WIDTH-1:0] paddr,
  input  wire                      psel,
  input  wire                      penable,
  input  wire                      pwrite,
  input  wire [31:0]               pwdata,
  output reg  [31:0]               prdata,
  output wire                      pready,
  output wire                      pslverr,

  // CTRL outputs
  output wire enable_o,
  output wire xip_en_o,
  output wire quad_en_o,
  output wire cpol_o,
  output wire cpha_o,
  output wire lsb_first_o,
  output wire cmd_trigger_o, // pulse
  output wire dma_en_o,
  output wire hold_en_o,
  output wire wp_en_o,

  // CLK_DIV outputs
  output wire [2:0] clk_div_o,

  // CS_CTRL outputs
  output wire       cs_auto_o,
  output wire       cs_level_o,
  output wire [1:0] cs_delay_o,

  // XIP_CFG outputs
  output wire [1:0] xip_addr_bytes_o,
  output wire [1:0] xip_data_lanes_o,
  output wire [3:0] xip_dummy_cycles_o,
  output wire       xip_cont_read_o,
  output wire       xip_mode_en_o,
  output wire [7:0] xip_mode_bits_o,
  output wire       xip_write_en_o,

  // XIP_CMD outputs
  output wire [7:0] xip_read_op_o,
  output wire [7:0] xip_write_op_o,

  // CMD_CFG outputs
  output wire [1:0] cmd_lanes_o,     // total lanes for opcode (if used)
  output wire [1:0] addr_lanes_o,
  output wire [1:0] data_lanes_o,
  output wire [1:0] addr_byte_o,
  output wire       mode_en_o,
  output wire [3:0] dummy_cycles_o,
  output wire       dir_o,

  // CMD_OP outputs
  output wire [7:0] opcode_o,
  output wire [7:0] mode_bits_o,

  // CMD_ADDR outputs
  output wire [31:0] cmd_addr_o,

  // CMD_LEN outputs
  output wire [31:0] cmd_len_o,

  // CMD_DUMMY outputs
  output wire [7:0]  extra_dummy_o,

  // DMA_CFG outputs
  output wire [3:0]  burst_size_o,
  output wire        dma_dir_o,
  output wire        incr_addr_o,

  // DMA_ADDR outputs
  output wire [31:0] dma_addr_o,

  // DMA_LEN outputs
  output wire [31:0] dma_len_o,

  // FIFO_TX outputs
  output wire [31:0] fifo_tx_data_o,
  output wire        fifo_tx_we_o,

  // INT_EN outputs
  output wire cmd_done_en_o,
  output wire dma_done_en_o,
  output wire err_en_o,
  output wire fifo_tx_empty_en_o,
  output wire fifo_rx_full_en_o,

  // STATUS inputs
  input  wire busy_i,
  input  wire xip_active_i,
  input  wire cmd_done_i,
  input  wire dma_done_i,

  // INT_STAT
  input  wire cmd_done_set_i,     // pulse to set
  output wire cmd_done_o,         // latched out
  input  wire dma_done_set_i,     // pulse to set
  output wire dma_done_o,         // latched out
  input  wire err_set_i,          // pulse to set
  output wire err_o,              // latched out
  input  wire fifo_tx_empty_set_i,// pulse to set
  output wire fifo_tx_empty_o,    // latched out
  input  wire fifo_rx_full_set_i, // pulse to set
  output wire fifo_rx_full_o,     // latched out

  // FIFO_RX inputs/outputs
  input  wire [31:0] fifo_rx_data_i,
  output wire        fifo_rx_re_o,

  // FIFO_STAT inputs
  input  wire [3:0]  tx_level_i,
  input  wire [3:0]  rx_level_i,
  input  wire        tx_empty_i,
  input  wire        rx_full_i,

  // ERR_STAT inputs
  input  wire timeout_i,
  input  wire overrun_i,
  input  wire underrun_i,
  input  wire axi_err_i,

  // Interrupt output
  output wire irq
);

  // --------------------------------------------------------------------------
  // Address constants with explicit width to match paddr[11:0]
  // (Double?check against your spec. These are placeholders from screenshots.)
  localparam [11:0] ID_ADDR        = 12'h000;
  localparam [11:0] CTRL_ADDR      = 12'h004;
  localparam [11:0] STATUS_ADDR    = 12'h008;
  localparam [11:0] INT_EN_ADDR    = 12'h00C;
  localparam [11:0] INT_STAT_ADDR  = 12'h010;
  localparam [11:0] CLK_DIV_ADDR   = 12'h014;
  localparam [11:0] CS_CTRL_ADDR   = 12'h018;
  localparam [11:0] XIP_CFG_ADDR   = 12'h01C;
  localparam [11:0] XIP_CMD_ADDR   = 12'h020;
  localparam [11:0] CMD_CFG_ADDR   = 12'h024;
  localparam [11:0] CMD_OP_ADDR    = 12'h028;
  localparam [11:0] CMD_ADDR_ADDR  = 12'h02C;
  localparam [11:0] CMD_LEN_ADDR   = 12'h030;
  localparam [11:0] CMD_DUMMY_ADDR = 12'h034;
  localparam [11:0] DMA_CFG_ADDR   = 12'h038;
  localparam [11:0] DMA_ADDR_ADDR  = 12'h03C;
  localparam [11:0] DMA_LEN_ADDR   = 12'h040;
  localparam [11:0] FIFO_TX_ADDR   = 12'h044;
  localparam [11:0] FIFO_RX_ADDR   = 12'h048;
  localparam [11:0] FIFO_STAT_ADDR = 12'h04C;
  localparam [11:0] ERR_STAT_ADDR  = 12'h050;

  // --------------------------------------------------------------------------
  // Registers
  reg  [31:0] ctrl_reg;
  reg  [31:0] int_en_reg;
  reg  [31:0] clk_div_reg;
  reg  [31:0] cs_ctrl_reg;
  reg  [31:0] xip_cfg_reg;
  reg  [31:0] xip_cmd_reg;
  reg  [31:0] cmd_cfg_reg;
  reg  [31:0] cmd_op_reg;
  reg  [31:0] cmd_addr_reg;
  reg  [31:0] cmd_len_reg;
  reg  [31:0] cmd_dummy_reg;
  reg  [31:0] dma_cfg_reg;
  reg  [31:0] dma_addr_reg;
  reg  [31:0] dma_len_reg;

  // INT_STAT RW1C latches
  reg cmd_done_reg;
  reg dma_done_reg;
  reg err_reg;
  reg fifo_tx_empty_reg;
  reg fifo_rx_full_reg;

  // --------------------------------------------------------------------------
  // Hardcoded ID
  wire [31:0] id_reg = {16'h0A10, 8'h01, 8'h01};

  // Status combinational
  wire [31:0] status_reg =
    {28'b0, dma_done_i, cmd_done_i, xip_active_i, busy_i};

  // FIFO_STAT combinational
  wire [31:0] fifo_stat_reg =
    {22'b0, rx_full_i, tx_empty_i, rx_level_i, tx_level_i};

  // ERR_STAT combinational
  wire [31:0] err_stat_reg =
    {28'b0, axi_err_i, underrun_i, overrun_i, timeout_i};

  // --------------------------------------------------------------------------
  // APB decode
  wire apb_transfer = psel & penable;
  wire write  = apb_transfer &  pwrite;
  wire read   = apb_transfer & ~pwrite;

  assign pready  = 1'b1;          // no wait states
  reg    invalid_addr;
  assign pslverr = invalid_addr;

  always @(*) begin
    invalid_addr = 1'b0;
    if (apb_transfer) begin
      case (paddr)
        ID_ADDR       : /* hit */ ;
        CTRL_ADDR     : /* hit */ ;
        STATUS_ADDR   : /* hit */ ;
        INT_EN_ADDR   : /* hit */ ;
        INT_STAT_ADDR : /* hit */ ;
        CLK_DIV_ADDR  : /* hit */ ;
        CS_CTRL_ADDR  : /* hit */ ;
        XIP_CFG_ADDR  : /* hit */ ;
        XIP_CMD_ADDR  : /* hit */ ;
        CMD_CFG_ADDR  : /* hit */ ;
        CMD_OP_ADDR   : /* hit */ ;
        CMD_ADDR_ADDR : /* hit */ ;
        CMD_LEN_ADDR  : /* hit */ ;
        CMD_DUMMY_ADDR: /* hit */ ;
        DMA_CFG_ADDR  : /* hit */ ;
        DMA_ADDR_ADDR : /* hit */ ;
        DMA_LEN_ADDR  : /* hit */ ;
        FIFO_TX_ADDR  : /* hit */ ;
        FIFO_RX_ADDR  : /* hit */ ;
        FIFO_STAT_ADDR: /* hit */ ;
        ERR_STAT_ADDR : /* hit */ ;
        default: begin
          invalid_addr = 1'b1;
        end
      endcase
    end
  end

  // --------------------------------------------------------------------------
  // CMD_TRIGGER pulse (self-clearing, not stored in reg)
  assign cmd_trigger_o = write && (paddr == CTRL_ADDR) && pwdata[8];
  assign fifo_tx_we_o  = write && (paddr == FIFO_TX_ADDR);
  assign fifo_tx_data_o = pwdata;
  assign fifo_rx_re_o  = read  && (paddr == FIFO_RX_ADDR);

  // --------------------------------------------------------------------------
  // INT_STAT RW1C latches (pulse to set, write-1-to-clear)
  always @(posedge pclk or negedge presetn) begin
    if (!presetn) begin
      cmd_done_reg <= 1'b0;
    end else if (cmd_done_set_i) begin
      cmd_done_reg <= 1'b1;
    end else if (write && (paddr == INT_STAT_ADDR) && pwdata[0]) begin
      cmd_done_reg <= 1'b0;
    end
  end

  always @(posedge pclk or negedge presetn) begin
    if (!presetn) begin
      dma_done_reg <= 1'b0;
    end else if (dma_done_set_i) begin
      dma_done_reg <= 1'b1;
    end else if (write && (paddr == INT_STAT_ADDR) && pwdata[1]) begin
      dma_done_reg <= 1'b0;
    end
  end

  always @(posedge pclk or negedge presetn) begin
    if (!presetn) begin
      err_reg <= 1'b0;
    end else if (err_set_i) begin
      err_reg <= 1'b1;
    end else if (write && (paddr == INT_STAT_ADDR) && pwdata[2]) begin
      err_reg <= 1'b0;
    end
  end

  always @(posedge pclk or negedge presetn) begin
    if (!presetn) begin
      fifo_tx_empty_reg <= 1'b0;
    end else if (fifo_tx_empty_set_i) begin
      fifo_tx_empty_reg <= 1'b1;
    end else if (write && (paddr == INT_STAT_ADDR) && pwdata[3]) begin
      fifo_tx_empty_reg <= 1'b0;
    end
  end

  always @(posedge pclk or negedge presetn) begin
    if (!presetn) begin
      fifo_rx_full_reg <= 1'b0;
    end else if (fifo_rx_full_set_i) begin
      fifo_rx_full_reg <= 1'b1;
    end else if (write && (paddr == INT_STAT_ADDR) && pwdata[4]) begin
      fifo_rx_full_reg <= 1'b0;
    end
  end

  assign cmd_done_o      = cmd_done_reg;
  assign dma_done_o      = dma_done_reg;
  assign err_o           = err_reg;
  assign fifo_tx_empty_o = fifo_tx_empty_reg;
  assign fifo_rx_full_o  = fifo_rx_full_reg;

  // --------------------------------------------------------------------------
  // RW registers update
  always @(posedge pclk or negedge presetn) begin
    if (!presetn) begin
      ctrl_reg     <= 32'b0;
      int_en_reg   <= 32'b0;
      clk_div_reg  <= 32'b0;
      cs_ctrl_reg  <= 32'b0;
      xip_cfg_reg  <= 32'b0;
      xip_cmd_reg  <= 32'b0;
      cmd_cfg_reg  <= 32'b0;
      cmd_op_reg   <= 32'b0;
      cmd_addr_reg <= 32'b0;
      cmd_len_reg  <= 32'b0;
      cmd_dummy_reg<= 32'b0;
      dma_cfg_reg  <= 32'b0;
      dma_addr_reg <= 32'b0;
      dma_len_reg  <= 32'b0;
    end else if (write && !invalid_addr) begin
      case (paddr)
        CTRL_ADDR      : ctrl_reg      <= pwdata;
        INT_EN_ADDR    : int_en_reg    <= pwdata;
        CLK_DIV_ADDR   : clk_div_reg   <= pwdata;
        CS_CTRL_ADDR   : cs_ctrl_reg   <= pwdata;
        XIP_CFG_ADDR   : xip_cfg_reg   <= pwdata;
        XIP_CMD_ADDR   : xip_cmd_reg   <= pwdata;
        CMD_CFG_ADDR   : cmd_cfg_reg   <= pwdata;
        CMD_OP_ADDR    : cmd_op_reg    <= pwdata;
        CMD_ADDR_ADDR  : cmd_addr_reg  <= pwdata;
        CMD_LEN_ADDR   : cmd_len_reg   <= pwdata;
        CMD_DUMMY_ADDR : cmd_dummy_reg <= pwdata;
        DMA_CFG_ADDR   : dma_cfg_reg   <= pwdata;
        DMA_ADDR_ADDR  : dma_addr_reg  <= pwdata;
        DMA_LEN_ADDR   : dma_len_reg   <= pwdata;
        default: ;
      endcase
    end
  end

  // --------------------------------------------------------------------------
  // prdata mux
  always @(*) begin
    prdata = 32'b0;
    if (read) begin
      case (paddr)
        ID_ADDR        : prdata = id_reg;
        CTRL_ADDR      : prdata = ctrl_reg;
        STATUS_ADDR    : prdata = status_reg;
        INT_EN_ADDR    : prdata = int_en_reg;
        INT_STAT_ADDR  : prdata = {27'b0, fifo_rx_full_reg, fifo_tx_empty_reg,
                                   err_reg, dma_done_reg, cmd_done_reg};
        CLK_DIV_ADDR   : prdata = clk_div_reg;
        CS_CTRL_ADDR   : prdata = cs_ctrl_reg;
        XIP_CFG_ADDR   : prdata = xip_cfg_reg;
        XIP_CMD_ADDR   : prdata = xip_cmd_reg;
        CMD_CFG_ADDR   : prdata = cmd_cfg_reg;
        CMD_OP_ADDR    : prdata = cmd_op_reg;
        CMD_ADDR_ADDR  : prdata = cmd_addr_reg;
        CMD_LEN_ADDR   : prdata = cmd_len_reg;
        CMD_DUMMY_ADDR : prdata = cmd_dummy_reg;
        DMA_CFG_ADDR   : prdata = dma_cfg_reg;
        DMA_ADDR_ADDR  : prdata = dma_addr_reg;
        DMA_LEN_ADDR   : prdata = dma_len_reg;
        FIFO_TX_ADDR   : prdata = 32'b0;
        FIFO_RX_ADDR   : prdata = fifo_rx_data_i;
        FIFO_STAT_ADDR : prdata = fifo_stat_reg;
        ERR_STAT_ADDR  : prdata = err_stat_reg;
        default: ;
      endcase
    end
  end

  // --------------------------------------------------------------------------
  // Field extracts to outputs (bit mapping should match your spec)
  // CTRL
  assign enable_o      = ctrl_reg[0];
  assign xip_en_o      = ctrl_reg[1];
  assign quad_en_o     = ctrl_reg[2];
  assign cpol_o        = ctrl_reg[3];
  assign cpha_o        = ctrl_reg[4];
  assign lsb_first_o   = ctrl_reg[5];
  // ctrl_reg[8] is used by cmd_trigger_o pulse on write
  assign dma_en_o      = ctrl_reg[9];
  assign hold_en_o     = ctrl_reg[10];
  assign wp_en_o       = ctrl_reg[11];

  // CLK_DIV
  assign clk_div_o     = clk_div_reg[2:0];

  // CS_CTRL
  assign cs_auto_o     = cs_ctrl_reg[0];
  assign cs_level_o    = cs_ctrl_reg[1];
  assign cs_delay_o    = cs_ctrl_reg[3:2];

  // XIP_CFG
  assign xip_addr_bytes_o   = xip_cfg_reg[1:0];
  assign xip_data_lanes_o   = xip_cfg_reg[3:2];
  assign xip_dummy_cycles_o = xip_cfg_reg[7:4];
  assign xip_cont_read_o    = xip_cfg_reg[8];
  assign xip_mode_en_o      = xip_cfg_reg[9];
  assign xip_mode_bits_o    = xip_cfg_reg[17:10];
  assign xip_write_en_o     = xip_cfg_reg[18];

  // XIP_CMD
  assign xip_read_op_o  = xip_cmd_reg[7:0];
  assign xip_write_op_o = xip_cmd_reg[15:8];

  // CMD_CFG
  assign cmd_lanes_o    = cmd_cfg_reg[1:0];
  assign addr_lanes_o   = cmd_cfg_reg[3:2];
  assign data_lanes_o   = cmd_cfg_reg[5:4];
  assign addr_byte_o    = cmd_cfg_reg[7:6];
  assign mode_en_o      = cmd_cfg_reg[8];
  assign dummy_cycles_o = cmd_cfg_reg[12:9];
  assign dir_o          = cmd_cfg_reg[13];

  // CMD_OP
  assign opcode_o       = cmd_op_reg[7:0];
  assign mode_bits_o    = cmd_op_reg[15:8];

  // CMD_ADDR/LEN/DUMMY
  assign cmd_addr_o     = cmd_addr_reg;
  assign cmd_len_o      = cmd_len_reg;
  assign extra_dummy_o  = cmd_dummy_reg[7:0];

  // DMA
  assign burst_size_o   = dma_cfg_reg[3:0];
  assign dma_dir_o      = dma_cfg_reg[4];
  assign incr_addr_o    = dma_cfg_reg[5];
  assign dma_addr_o     = dma_addr_reg;
  assign dma_len_o      = dma_len_reg;

  // INT_EN
  assign cmd_done_en_o      = int_en_reg[0];
  assign dma_done_en_o      = int_en_reg[1];
  assign err_en_o           = int_en_reg[2];
  assign fifo_tx_empty_en_o = int_en_reg[3];
  assign fifo_rx_full_en_o  = int_en_reg[4];

  // IRQ: OR of enabled & latched
  assign irq = (cmd_done_en_o      & cmd_done_reg)     |
               (dma_done_en_o      & dma_done_reg)     |
               (err_en_o           & err_reg)          |
               (fifo_tx_empty_en_o & fifo_tx_empty_reg)|
               (fifo_rx_full_en_o  & fifo_rx_full_reg);

always @(posedge pclk or negedge presetn) begin
  if (!presetn) begin
    fifo_rx_full_reg <= 1'b0;
  end else if (fifo_rx_full_set_i) begin
    ctrl_reg[8] <= 1'b1;
    $display("INT_STAT: fifo_rx_full set");
  end else if (write && (paddr == INT_STAT_ADDR) && pwdata[4]) begin
    fifo_rx_full_reg <= 1'b0;
    $display("INT_STAT: fifo_rx_full cleared");
  end
end

endmodule
