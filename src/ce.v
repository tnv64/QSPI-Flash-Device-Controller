module ce #(
  parameter RESET_SYNC = 1  
) (
  input  wire clk,
  input  wire rst_n,

  // -------- CSR interface --------
  input  wire cmd_trigger_i,  
  input  wire dma_en_i,        
  output reg  clear_cmd_o,     
  output wire cmd_done_o,      
  output reg  busy_o,          
  output wire dma_done_o,      

  // -------- DMA controller --------
  input  wire dma_done_i,      // DMA báo xong (level)
  output reg  dma_start_o,     

  // -------- QSPI FSM --------
  input  wire done_qspi_i,     
  output reg  start_qspi_o     
);

  assign dma_done_o = dma_done_i;

  // ------------------------------------------------------------
  // Tr?ng thái CE
  // ------------------------------------------------------------
  localparam ST_IDLE      = 3'd0,
            ST_START     = 3'd1,  
            ST_WAIT_QSPI = 3'd2,  
            ST_WAIT_DMA  = 3'd3,  
            ST_DONE      = 3'd4 ;  
  
  reg [2:0] state, nstate;

  reg       use_dma_r;

  wire can_accept = (state == ST_IDLE);
  wire trigger_seen = cmd_trigger_i & can_accept;
  assign cmd_done_o = done_qspi_i & (use_dma_r ? dma_done_i : 1'b1);

  always @(*) begin
    nstate = state;
    case (state)
      ST_IDLE: begin
        if (trigger_seen) nstate = ST_START;
      end

      ST_START: begin
       
        nstate = ST_WAIT_QSPI;
      end

      ST_WAIT_QSPI: begin
        if (done_qspi_i) begin
          nstate = (use_dma_r ? ST_WAIT_DMA : ST_DONE);
        end
      end

      ST_WAIT_DMA: begin
        if (dma_done_i) begin
         
          nstate = ST_DONE;
        end
      end

      ST_DONE: begin
       
        nstate = ST_IDLE;
      end

      default: nstate = ST_IDLE;
    endcase
  end

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      state         <= ST_IDLE;
      start_qspi_o  <= 1'b0;
      dma_start_o   <= 1'b0;
      clear_cmd_o   <= 1'b0;
      busy_o        <= 1'b0;
      use_dma_r     <= 1'b0;
    end else begin
      state         <= nstate;

      start_qspi_o  <= 1'b0;
      dma_start_o   <= 1'b0;
      clear_cmd_o   <= 1'b0;

      case (state)
        ST_IDLE: begin
          busy_o <= 1'b0;
          if (trigger_seen) begin            
            use_dma_r    <= dma_en_i;

            start_qspi_o <= 1'b1;
            dma_start_o  <= dma_en_i;   
            clear_cmd_o  <= 1'b1;       
            busy_o       <= 1'b1;
          end
        end

        ST_START: begin
          
          busy_o <= 1'b1;
        
        end

        ST_WAIT_QSPI: begin
          busy_o <= 1'b1;
          // Ch? done_qspi_i == 1
        end

        ST_WAIT_DMA: begin
          busy_o <= 1'b1;
  
        end

        ST_DONE: begin

          busy_o <= 1'b0;
        end

        default: begin
          busy_o <= 1'b0;
        end
      endcase
    end
  end

endmodule
