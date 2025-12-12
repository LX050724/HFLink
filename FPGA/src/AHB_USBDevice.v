module AHB_USBDevice #(
        parameter USB_REG_NUM = 16,
        parameter USB_DESC_RAM_LEN = 1024,
        parameter ADDRWIDTH = 12
    ) (
        input  wire                  hclk,       // clock
        input  wire                  hresetn,    // reset

        // AHB connection to master
        input  wire                  hsels,
        input  wire [ADDRWIDTH-1:0]  haddrs,
        input  wire [1:0]            htranss,
        input  wire [2:0]            hsizes,
        input  wire                  hwrites,
        input  wire                  hreadys,
        input  wire [31:0]           hwdatas,

        output wire                  hreadyouts,
        output wire                  hresps,
        output reg [31:0]           hrdatas,

        output wire intr,

        // USB ULPI接口
        input [7:0] usb_ulpi_data_i,
        output [7:0] usb_ulpi_data_o,
        inout [7:0] usb_ulpi_data,
        input usb_ulpi_nxt,
        input usb_ulpi_dir,
        output usb_ulpi_stp,
        output usb_nrst
    );

    // ----------------------------------------
    // Internal wires declarations
    wire                   trans_req= hreadys & hsels & htranss[1];
    // transfer request issued only in SEQ and NONSEQ status and slave is
    // selected and last transfer finish

    wire                   ahb_read_req  = trans_req & (~hwrites);// AHB read request
    wire                   ahb_write_req = trans_req &  hwrites;  // AHB write request
    wire                   update_read_req;    // To update the read enable register
    wire                   update_write_req;   // To update the write enable register

    reg  [ADDRWIDTH-1:0]   addr_reg;     // address signal, registered
    reg                    read_en_reg;  // read enable signal, registered
    reg                    write_en_reg; // write enable signal, registered

    reg  [3:0]             byte_strobe_reg; // registered output for byte strobe
    reg  [3:0]             byte_strobe_nxt; // next state for byte_strobe_reg
    //-----------------------------------------------------------
    // Module logic start
    //----------------------------------------------------------

    // Address signal registering, to make the address and data active at the same cycle
    always @(posedge hclk or negedge hresetn) begin
        if (~hresetn)
            addr_reg <= {(ADDRWIDTH){1'b0}}; //default address 0 is selected
        else if (trans_req)
            addr_reg <= haddrs[ADDRWIDTH-1:0];
    end


    // register read signal generation
    assign update_read_req = ahb_read_req | (read_en_reg & hreadys); // Update read enable control if
    //  1. When there is a valid read request
    //  2. When there is an active read, update it at the end of transfer (HREADY=1)

    always @(posedge hclk or negedge hresetn) begin
        if (~hresetn) begin
            read_en_reg <= 1'b0;
        end
        else if (update_read_req) begin
            read_en_reg  <= ahb_read_req;
        end
    end

    // register write signal generation
    assign update_write_req = ahb_write_req |( write_en_reg & hreadys);  // Update write enable control if
    //  1. When there is a valid write request
    //  2. When there is an active write, update it at the end of transfer (HREADY=1)

    always @(posedge hclk or negedge hresetn) begin
        if (~hresetn) begin
            write_en_reg <= 1'b0;
        end
        else if (update_write_req) begin
            write_en_reg  <= ahb_write_req;
        end
    end

    // byte strobe signal
    always @(hsizes or haddrs) begin
        if (hsizes == 3'b000)    //byte
        begin
            case(haddrs[1:0])
                2'b00:
                    byte_strobe_nxt = 4'b0001;
                2'b01:
                    byte_strobe_nxt = 4'b0010;
                2'b10:
                    byte_strobe_nxt = 4'b0100;
                2'b11:
                    byte_strobe_nxt = 4'b1000;
                default:
                    byte_strobe_nxt = 4'bxxxx;
            endcase
        end
        else if (hsizes == 3'b001) //half word
        begin
            if(haddrs[1]==1'b1)
                byte_strobe_nxt = 4'b1100;
            else
                byte_strobe_nxt = 4'b0011;
        end
        else // default 32 bits, word
        begin
            byte_strobe_nxt = 4'b1111;
        end
    end

    always @(posedge hclk or negedge hresetn) begin
        if (~hresetn)
            byte_strobe_reg <= {4{1'b0}};
        else if (update_read_req|update_write_req)
            // Update byte strobe registers if
            // 1. if there is a valid read/write transfer request
            // 2. if there is an on going transfer
            byte_strobe_reg  <= byte_strobe_nxt;
    end

    //-----------------------------------------------------------
    // Outputs
    //-----------------------------------------------------------
    // For simplify the timing, the master and slave signals are connected directly, execpt data bus.
    wire [ADDRWIDTH-1:0] addr        = addr_reg[ADDRWIDTH-1:0];
    wire read_en     = read_en_reg;
    wire write_en    = write_en_reg;
    wire [31:0] wdata       = hwdatas;
    wire [3:0] byte_strobe = byte_strobe_reg;
    reg hreadyouts_reg;

    assign hreadyouts  = hreadyouts_reg;  // slave always ready
    assign hresps      = 1'b0;  // OKAY response from slave
    //-----------------------------------------------------------
    //Module logic end
    //----------------------------------------------------------

    reg [31:0] USB_REGS[USB_REG_NUM-1:0];
    // 描述符RAM映射
    // reg [7:0] USB_DESC_RAM[USB_DESC_RAM_LEN-1:0]; /*synthesis syn_ramstyle="block_ram"*/

    // wire USB_SR_HISPEED = USB_REGS[0][0];
    // wire USB_SR_SUSPEND = USB_REGS[0][1];
    // wire USB_SR_ONLINE = USB_REGS[0][2];
    wire USB_CR_EN = USB_REGS[0][16];


    wire [15:0] USB_DESC_DEV_LEN = USB_REGS[1][15:0];
    wire [15:0] USB_DESC_DEV_ADDR = USB_REGS[1][31:16];

    wire [15:0] USB_DESC_QUAL_LEN = USB_REGS[2][15:0];
    wire [15:0] USB_DESC_QUAL_ADDR = USB_REGS[2][31:16];

    wire [15:0] USB_DESC_FSCFG_LEN = USB_REGS[3][15:0];
    wire [15:0] USB_DESC_FSCFG_ADDR = USB_REGS[3][31:16];

    wire [15:0] USB_DESC_HSCFG_LEN = USB_REGS[4][15:0];
    wire [15:0] USB_DESC_HSCFG_ADDR = USB_REGS[4][31:16];

    wire [15:0] USB_DESC_STRLANG_ADDR = USB_REGS[5][15:0];
    wire [15:0] USB_DESC_OSCFG_ADDR = USB_REGS[5][31:16];

    wire [15:0] USB_DESC_HIDRPT_LEN = USB_REGS[6][15:0];
    wire [15:0] USB_DESC_HIDRPT_ADDR = USB_REGS[6][31:16];

    wire [15:0] USB_DESC_BOS_LEN = USB_REGS[7][15:0];
    wire [15:0] USB_DESC_BOS_ADDR = USB_REGS[7][31:16];

    wire [15:0] USB_DESC_STRVENDOR_LEN = USB_REGS[8][15:0];
    wire [15:0] USB_DESC_STRVENDOR_ADDR = USB_REGS[8][31:16];

    wire [15:0] USB_DESC_STRPRODUCT_LEN = USB_REGS[9][15:0];
    wire [15:0] USB_DESC_STRPRODUCT_ADDR = USB_REGS[9][31:16];

    wire [15:0] USB_DESC_STRSERIAL_LEN = USB_REGS[10][15:0];
    wire [15:0] USB_DESC_STRSERIAL_ADDR = USB_REGS[10][31:16];

    wire USB_DESC_HASSTR = USB_REGS[11][0:0];

// `define USB_ACM_CTRL_DR 12
//     `define USB_ACM_CTRL_SR 13
//     `define USB_ACM_CTRL_SR_RXNE USB_REGS[USB_ACM_CTRL_SR][0]

    // wire [7:0] acm_ctrl_tx_data = USB_REGS[`USB_ACM_CTRL_DR][7:0];
    // reg acm_ctrl_tx_dval;

    // reg acm_ctrl_rx_rdy;
    // wire acm_ctrl_rx_dval;
    // wire [7:0] acm_ctrl_rx_data;
    // reg [7:0] acm_ctrl_rx_data_reg;

    // genvar gi;
    // generate
    //     for (gi = 0; gi < (USB_REG_NUM - USB_DESC_RAM_START); gi=gi+1)
    //         assign USB_DESC_RAM[gi] = USB_REGS[gi+USB_DESC_RAM_START];
    // endgenerate
    wire [15:0] USB_DESCROM_RADDR;
    wire [7:0] USB_DESCROM_RDAT;
    wire usb_link_rst;

    integer i;

    wire [7:0] usb_desc_dpb_douta;

    // USB_DESC_DPB usb_desc_dpb(
    //     .clka(hclk), //input clka
    //     .ocea(1'b1), //input ocea
    //     .cea(1'b1), //input cea
    //     .ada(addr[10:0]), //input [10:0] ada
    //     .dina(hwdatas[7:0]), //input [7:0] dina
    //     .douta(usb_desc_dpb_douta), //output [7:0] douta
    //     .wrea(write_en && (addr[ADDRWIDTH-1] == 0) && (USB_CR_EN == 0)), //input wrea
    //     .reseta(~hresetn), //input reseta
        
    //     .clkb(~hclk), //input clkb
    //     .oceb(1'b1), //input oceb
    //     .ceb(USB_CR_EN), //input ceb
    //     .adb(USB_DESCROM_RADDR[10:0]), //input [10:0] adb
    //     .dinb(8'd0), //input [7:0] dinb
    //     .doutb(USB_DESCROM_RDAT), //output [7:0] doutb
    //     .wreb(8'd0), //input wreb
    //     .resetb(~hresetn) //input resetb
    // );

    always @(posedge hclk or negedge hresetn) begin
        if (!hresetn) begin
            hreadyouts_reg = 1;
        end else begin
            hreadyouts_reg <= 1;
            if (ahb_read_req && (haddrs[ADDRWIDTH-1] == 0))
                hreadyouts_reg <= 0;
        end
    end
    
    always @(posedge hclk or negedge hresetn) begin
        if (!hresetn) begin
            for (i = 0; i < USB_REG_NUM; i=i+1)
                USB_REGS[i] <= 0;
        end
        else begin
            if (write_en) begin
                if (addr[ADDRWIDTH-1] == 1) begin
                    // 寄存器域
                    if (byte_strobe_reg[0])
                        USB_REGS[addr[5:2]][0 +: 8] <= hwdatas[0 +: 8];
                    if (byte_strobe_reg[1])
                        USB_REGS[addr[5:2]][8 +: 8] <= hwdatas[8 +: 8];
                    if (byte_strobe_reg[2])
                        USB_REGS[addr[5:2]][16 +: 8] <= hwdatas[16 +: 8];
                    if (byte_strobe_reg[3])
                        USB_REGS[addr[5:2]][24 +: 8] <= hwdatas[24 +: 8];
                end
            end
        end
    end


    always @(*) begin
        if (read_en) begin
            if (addr[ADDRWIDTH-1] == 1) begin
                case (addr[5:2])
                    0:
                        hrdatas = {USB_REGS[addr[5:2]][31:3], usb_status_hispeed, usb_status_suspend, usb_status_online};
                    default:
                        hrdatas = USB_REGS[addr[5:2]];
                endcase
            end else begin
                // 描述符域仅支持8b操作
                hrdatas = {4{usb_desc_dpb_douta}};
            end
        end
        else begin
            hrdatas = {32{1'bx}};
        end
    end

    //-----------------------------------------------------------
    // USB logic
    //----------------------------------------------------------


    localparam ENDPT_UART_CONFIG = 4'd0;

    wire endpt0_send;
    wire [7:0] endpt0_dat;
    wire [3:0] endpt_sel;
    wire usb_txval = endpt0_send & (endpt_sel == ENDPT_UART_CONFIG);
    wire [7:0] usb_txdat;
    wire usb_txcork;
    wire usb_txpop;
    wire usb_txact;
    wire [7:0] usb_rxdat;
    wire usb_rxval;
    wire usb_rxpktval;
    wire usb_rxact;
    wire usb_rxrdy;
    wire usb_sof;
    wire usb_txpktfin;
    wire setup_active;
    wire usb_status_hispeed;
    wire usb_status_suspend;
    wire usb_status_online;
    wire [11:0] ep_usb_txlen;
    wire [11:0] usb_txdat_len;
    wire [11:0] uart_config_txdat_len;
    wire [7:0] ep_usb_txdat;
    wire ep_usb_rxrdy;
    wire ep_usb_txcork;
    

    /* 根据USB端点号选通数据线 */
    assign usb_rxrdy     = ep_usb_rxrdy;
    assign usb_txcork    = (endpt_sel == ENDPT_UART_CONFIG) ? 1'b0 : ep_usb_txcork;
    assign usb_txdat     = (endpt_sel == ENDPT_UART_CONFIG) ? endpt0_dat : ep_usb_txdat;
    assign usb_txdat_len = (endpt_sel == ENDPT_UART_CONFIG) ? uart_config_txdat_len : ep_usb_txlen;

    // /* INF接口选择 */
    reg [7:0] interface_alter[3:0];
    wire [7:0] inf_sel;
    wire inf_set;
    wire [7:0] inf_alter_i;
    wire [7:0] inf_alter_o;

    always @(posedge hclk or negedge USB_CR_EN) begin
        if (!USB_CR_EN) begin
            interface_alter[0] <= 'd0;
            interface_alter[1] <= 'd0;
            interface_alter[2] <= 'd0;
            interface_alter[3] <= 'd0;
        end
        else begin
            if (inf_set & inf_sel < 4) begin
                interface_alter[inf_sel[1:0]] <= inf_alter_o;
            end
        end
    end

    assign inf_alter_i = inf_sel < 4 ? interface_alter[inf_sel[1:0]] : 0;
    
    USB_Device_Controller_Top usb_controller(
                                  .clk_i(hclk),
                                  .reset_i(~USB_CR_EN),

                                  .usbrst_o(usb_link_rst),

                                  .highspeed_o(usb_status_hispeed),
                                  .suspend_o(usb_status_suspend),
                                  .online_o(usb_status_online),

                                  .txdat_i(usb_txdat),
                                  .txval_i(usb_txval),
                                  .txdat_len_i(usb_txdat_len),
                                  .txiso_pid_i(4'b0011),
                                  .txcork_i(usb_txcork),
                                  .txpop_o(usb_txpop),
                                  .txact_o(usb_txact),
                                  .txpktfin_o(usb_txpktfin),
                                  .rxdat_o(usb_rxdat),
                                  .rxval_o(usb_rxval),
                                  .rxrdy_i(usb_rxrdy),
                                  .rxact_o(usb_rxact),
                                  .rxpktval_o(usb_rxpktval),
                                  .setup_o(setup_active),
                                  .endpt_o(endpt_sel),
                                  .sof_o(usb_sof),

                                  .inf_alter_i(inf_alter_i),
                                  .inf_alter_o(inf_alter_o),
                                  .inf_sel_o(inf_sel),
                                  .inf_set_o(inf_set),

                                  .descrom_rdata_i(USB_DESCROM_RDAT),
                                  .descrom_raddr_o(USB_DESCROM_RADDR),

                                  //   .desc_index_o(),
                                  //   .desc_type_o(),

                                  .desc_dev_addr_i(USB_DESC_DEV_ADDR),
                                  .desc_dev_len_i(USB_DESC_DEV_LEN),
                                  .desc_qual_addr_i(USB_DESC_QUAL_ADDR),
                                  .desc_qual_len_i(USB_DESC_QUAL_LEN),
                                  .desc_fscfg_addr_i(USB_DESC_FSCFG_ADDR),
                                  .desc_fscfg_len_i(USB_DESC_FSCFG_LEN),
                                  .desc_hscfg_addr_i(USB_DESC_HSCFG_ADDR),
                                  .desc_hscfg_len_i(USB_DESC_HSCFG_LEN),
                                  .desc_oscfg_addr_i(USB_DESC_OSCFG_ADDR),
                                  .desc_hidrpt_addr_i(USB_DESC_HIDRPT_ADDR),
                                  .desc_hidrpt_len_i(USB_DESC_HIDRPT_LEN),
                                  .desc_bos_addr_i(USB_DESC_BOS_ADDR),
                                  .desc_bos_len_i(USB_DESC_BOS_LEN),
                                  .desc_strlang_addr_i(USB_DESC_STRLANG_ADDR),
                                  .desc_strvendor_addr_i(USB_DESC_STRVENDOR_ADDR),
                                  .desc_strvendor_len_i(USB_DESC_STRVENDOR_LEN),
                                  .desc_strproduct_addr_i(USB_DESC_STRPRODUCT_ADDR),
                                  .desc_strproduct_len_i(USB_DESC_STRPRODUCT_LEN),
                                  .desc_strserial_addr_i(USB_DESC_STRSERIAL_ADDR),
                                  .desc_strserial_len_i(USB_DESC_STRSERIAL_LEN),
                                  .desc_have_strings_i(USB_DESC_HASSTR),

                                  .ulpi_nxt_i(usb_ulpi_nxt),
                                  .ulpi_dir_i(usb_ulpi_dir),
                                  .ulpi_rxdata_i(usb_ulpi_data_i),
                                  .ulpi_txdata_o(usb_ulpi_data_o),
                                  .ulpi_stp_o(usb_ulpi_stp)
                              );

    wire [11:0] TRANS_MAX = usb_status_hispeed ? 12'd512 : 12'd64;

    // usb_fifo usb_fifo_u (
    //              .i_clk(hclk),
    //              .i_reset(usb_link_rst),
    //              .i_usb_endpt(endpt_sel),
    //              .i_usb_rxact(usb_rxact),
    //              .i_usb_rxval(usb_rxval),
    //              .i_usb_rxpktval(usb_rxpktval),
    //              .i_usb_rxdat(usb_rxdat),
    //              .o_usb_rxrdy(ep_usb_rxrdy),
    //              .i_usb_txact(usb_txact),
    //              .i_usb_txpop(usb_txpop),
    //              .i_usb_txpktfin(usb_txpktfin),
    //              .o_usb_txcork(ep_usb_txcork),
    //              .o_usb_txlen(ep_usb_txlen),
    //              .o_usb_txdat(ep_usb_txdat),

    //              // ACM DIN
    //              .i_ep1_tx_clk(hclk),
    //              .i_ep1_tx_max(TRANS_MAX),
    //              // .i_ep1_tx_dval(),
    //              // .i_ep1_tx_data(),
                 
    //              // ACM DOUT
    //              .i_ep2_rx_clk(hclk),
    //              .i_ep2_rx_rdy(TRANS_MAX),
    //              // .o_ep2_rx_dval(),
    //              // .o_ep2_rx_data(),
                 
    //              // WinUSB IN
    //              .i_ep3_tx_clk(hclk),
    //              .i_ep3_tx_max(TRANS_MAX),
    //              .i_ep3_tx_dval(acm_ctrl_tx_dval),
    //              .i_ep3_tx_data(acm_ctrl_tx_data),
                 
    //              // WinUSB OUT
    //              .i_ep4_rx_clk(hclk)
    //              // .i_ep4_rx_rdy(),
    //              // .o_ep4_rx_dval(),
    //              // .o_ep4_rx_data(),
    //          );

    // /* USB串口配置模块 */
    // usb_uart_config u_usb_uart_config(
    //                     .PHY_CLKOUT(hclk),
    //                     .RESET_IN(usb_link_rst),
    //                     .setup_active(setup_active),
    //                     .endpt_sel(endpt_sel),
    //                     .usb_rxval(usb_rxval),
    //                     .usb_rxact(usb_rxact),
    //                     .usb_rxdat(usb_rxdat),
    //                     .usb_txact(usb_txact),
    //                     .usb_txpop(usb_txpop),
    //                     .usb_txdat_len_o(uart_config_txdat_len),
    //                     .endpt0_dat_o(endpt0_dat),
    //                     .endpt0_send_o(endpt0_send),

    //                     .uart1_en_o(uart_cfg_en),
    //                     .uart1_BAUD_RATE_o(uart_cfg_baud_rate),
    //                     .uart1_PARITY_BIT_o(uart_cfg_parity_bit),
    //                     .uart1_STOP_BIT_o(uart_cfg_stop_bit),
    //                     .uart1_DATA_BITS_o(uart_cfg_data_bits)
    //                 );
    // defparam u_usb_uart_config.ENDPT_UART_CONFIG = ENDPT_UART_CONFIG;

    assign usb_nrst = USB_CR_EN & hresetn;
    //assign intr = ~acm_ctrl_rx_rdy;
endmodule
