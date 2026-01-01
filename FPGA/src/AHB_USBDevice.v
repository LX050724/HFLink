module AHB_USBDevice #(
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
        output usb_nrst,

        // 数据AxisStream
        input winusb_in_tvalid,
        input [7:0] winusb_in_tdata,
        input winusb_out_tready,
        output winusb_out_tvalid,
        output [7:0] winusb_out_tdata,
        input cdc_in_tvalid,
        input [7:0] cdc_in_tdata,
        input cdc_out_tready,
        output cdc_out_tvalid,
        output [7:0] cdc_out_tdata
    );

    localparam [ADDRWIDTH-1:0] USB_SR_ADDR               = 12'h000;
    localparam [ADDRWIDTH-1:0] USB_CR_ADDR               = 12'h004;
    localparam [ADDRWIDTH-1:0] USB_FIFO_RX_FLAG_ADDR     = 12'h008;
    localparam [ADDRWIDTH-1:0] USB_FIFO_TX_FLAG_ADDR     = 12'h00C;
    localparam [ADDRWIDTH-1:0] USB_EP0_FIFO_ADDR         = 12'h010;
    localparam [ADDRWIDTH-1:0] USB_EP1_FIFO_ADDR         = 12'h014;
    localparam [ADDRWIDTH-1:0] USB_EP2_FIFO_ADDR         = 12'h018;
    localparam [ADDRWIDTH-1:0] USB_EP3_FIFO_ADDR         = 12'h01C;
    localparam [ADDRWIDTH-1:0] USB_EP4_FIFO_ADDR         = 12'h020;
    localparam [ADDRWIDTH-1:0] USB_EP5_FIFO_ADDR         = 12'h024;
    localparam [ADDRWIDTH-1:0] USB_EP6_FIFO_ADDR         = 12'h028;
    localparam [ADDRWIDTH-1:0] USB_EP7_FIFO_ADDR         = 12'h02C;
    localparam [ADDRWIDTH-1:0] USB_EP8_FIFO_ADDR         = 12'h030;
    localparam [ADDRWIDTH-1:0] USB_EP9_FIFO_ADDR         = 12'h034;
    localparam [ADDRWIDTH-1:0] USB_EP10_FIFO_ADDR        = 12'h038;
    localparam [ADDRWIDTH-1:0] USB_EP11_FIFO_ADDR        = 12'h03C;
    localparam [ADDRWIDTH-1:0] USB_EP12_FIFO_ADDR        = 12'h040;
    localparam [ADDRWIDTH-1:0] USB_EP13_FIFO_ADDR        = 12'h044;
    localparam [ADDRWIDTH-1:0] USB_EP14_FIFO_ADDR        = 12'h048;
    localparam [ADDRWIDTH-1:0] USB_EP15_FIFO_ADDR        = 12'h04C;

    localparam [ADDRWIDTH-1:0] USB_EP0_FIFO_NUM_ADDR     = 12'h050;
    localparam [ADDRWIDTH-1:0] USB_EP1_FIFO_NUM_ADDR     = 12'h054;
    localparam [ADDRWIDTH-1:0] USB_EP2_FIFO_NUM_ADDR     = 12'h058;
    localparam [ADDRWIDTH-1:0] USB_EP3_FIFO_NUM_ADDR     = 12'h05C;
    localparam [ADDRWIDTH-1:0] USB_EP4_FIFO_NUM_ADDR     = 12'h060;
    localparam [ADDRWIDTH-1:0] USB_EP5_FIFO_NUM_ADDR     = 12'h064;
    localparam [ADDRWIDTH-1:0] USB_EP6_FIFO_NUM_ADDR     = 12'h068;
    localparam [ADDRWIDTH-1:0] USB_EP7_FIFO_NUM_ADDR     = 12'h06C;
    localparam [ADDRWIDTH-1:0] USB_EP8_FIFO_NUM_ADDR     = 12'h070;
    localparam [ADDRWIDTH-1:0] USB_EP9_FIFO_NUM_ADDR     = 12'h074;
    localparam [ADDRWIDTH-1:0] USB_EP10_FIFO_NUM_ADDR    = 12'h078;
    localparam [ADDRWIDTH-1:0] USB_EP11_FIFO_NUM_ADDR    = 12'h07C;
    localparam [ADDRWIDTH-1:0] USB_EP12_FIFO_NUM_ADDR    = 12'h080;
    localparam [ADDRWIDTH-1:0] USB_EP13_FIFO_NUM_ADDR    = 12'h084;
    localparam [ADDRWIDTH-1:0] USB_EP14_FIFO_NUM_ADDR    = 12'h088;
    localparam [ADDRWIDTH-1:0] USB_EP15_FIFO_NUM_ADDR    = 12'h08C;

    localparam [ADDRWIDTH-1:0] USB_DESC_TAB_BASE_ADDR    = 12'h100; // size:256
    localparam [ADDRWIDTH-1:0] USB_DESC_ROM_BASE_ADDR    = 12'h800; // size:2048

    function addr_equ;
        input [ADDRWIDTH-1:0] addr;
        input [ADDRWIDTH-1:0] taddr;
        begin
            addr_equ = addr[ADDRWIDTH-1:2] == taddr[ADDRWIDTH-1:2];
        end
    endfunction

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
    wire [ADDRWIDTH-1:0] addr = addr_reg[ADDRWIDTH-1:0];
    wire read_en     = read_en_reg;
    wire write_en    = write_en_reg;
    wire [31:0] wdata       = hwdatas;
    wire [3:0] byte_strobe = byte_strobe_reg;

    assign hreadyouts  = 1'b1;  // slave always ready
    assign hresps      = 1'b0;  // OKAY response from slave
    //-----------------------------------------------------------
    //Module logic end
    //----------------------------------------------------------

    reg [31:0] usb_ctrl_reg;
    reg [31:0] USB_REGS[10:0];

    wire USB_CR_EN = usb_ctrl_reg[0];
    wire USB_SEND_Z = usb_ctrl_reg[1];

    wire USB_EPOUT_INT_EN = usb_ctrl_reg[31];
    wire USB_EPIN_INT_EN = usb_ctrl_reg[30];
    wire USB_SETUP_INT_EN = usb_ctrl_reg[29];

    wire [15:0] USB_DESC_DEV_LEN = USB_REGS[0][15:0];
    wire [15:0] USB_DESC_DEV_ADDR = USB_REGS[0][31:16];

    wire [15:0] USB_DESC_QUAL_LEN = USB_REGS[1][15:0];
    wire [15:0] USB_DESC_QUAL_ADDR = USB_REGS[1][31:16];

    wire [15:0] USB_DESC_FSCFG_LEN = USB_REGS[2][15:0];
    wire [15:0] USB_DESC_FSCFG_ADDR = USB_REGS[2][31:16];

    wire [15:0] USB_DESC_HSCFG_LEN = USB_REGS[3][15:0];
    wire [15:0] USB_DESC_HSCFG_ADDR = USB_REGS[3][31:16];

    wire [15:0] USB_DESC_STRLANG_ADDR = USB_REGS[4][15:0];
    wire [15:0] USB_DESC_OSCFG_ADDR = USB_REGS[4][31:16];

    wire [15:0] USB_DESC_HIDRPT_LEN = USB_REGS[5][15:0];
    wire [15:0] USB_DESC_HIDRPT_ADDR = USB_REGS[5][31:16];

    wire [15:0] USB_DESC_BOS_LEN = USB_REGS[6][15:0];
    wire [15:0] USB_DESC_BOS_ADDR = USB_REGS[6][31:16];

    wire [15:0] USB_DESC_STRVENDOR_LEN = USB_REGS[7][15:0];
    wire [15:0] USB_DESC_STRVENDOR_ADDR = USB_REGS[7][31:16];

    wire [15:0] USB_DESC_STRPRODUCT_LEN = USB_REGS[8][15:0];
    wire [15:0] USB_DESC_STRPRODUCT_ADDR = USB_REGS[8][31:16];

    wire [15:0] USB_DESC_STRSERIAL_LEN = USB_REGS[9][15:0];
    wire [15:0] USB_DESC_STRSERIAL_ADDR = USB_REGS[9][31:16];

    wire USB_DESC_HASSTR = USB_REGS[10][0:0];


    always @(posedge hclk or negedge hresetn) begin
        if (!hresetn) begin
            usb_ctrl_reg <= 0;
        end
        else begin
            if (write_en && addr_equ(addr, USB_CR_ADDR)) begin
                if (byte_strobe_reg[0])
                    usb_ctrl_reg[0 +: 8] <= hwdatas[0 +: 8];
                if (byte_strobe_reg[1])
                    usb_ctrl_reg[8 +: 8] <= hwdatas[8 +: 8];
                if (byte_strobe_reg[2])
                    usb_ctrl_reg[16 +: 8] <= hwdatas[16 +: 8];
                if (byte_strobe_reg[3])
                    usb_ctrl_reg[24 +: 8] <= hwdatas[24 +: 8];
            end
        end
    end

    integer i;

    always @(posedge hclk or negedge hresetn) begin
        if (!hresetn) begin
            for (i = 0; i < 11; i=i+1) begin
                USB_REGS[i] <= 32'd0;
            end
        end
        else begin
            if (write_en && addr[ADDRWIDTH-1:8] == 4'b1) begin
                if (addr[7:2] < 6'd11) begin
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


    // always @(posedge hclk or negedge hresetn) begin
    //     if (!hresetn) begin
    //         hreadyouts_reg = 1;
    //     end else begin
    //         hreadyouts_reg <= 1;
    //         if (ahb_read_req && (haddrs[ADDRWIDTH-1] == 0))
    //             hreadyouts_reg <= 0;
    //     end
    // end

    reg [10:0] desc_rom_ada;
    reg desc_rom_wrea;
    wire [7:0] desc_rom_dina = hwdatas[7:0];
    wire [7:0] desc_rom_douta;
    wire [10:0] desc_rom_adb;
    wire [7:0] desc_rom_doutb;
    Gowin_DPB usb_desc_ram (
                  .clka(hclk), //input clka
                  .cea(1'b1), //input cea
                  .ocea(1'b1), //input ocea
                  .ada(desc_rom_ada), //input [10:0] ada
                  .wrea(desc_rom_wrea), //input wrea
                  .dina(desc_rom_dina), //input [7:0] dina
                  .douta(desc_rom_douta), //output [7:0] douta
                  .reseta(~hresetn), //input reseta


                  .clkb(~hclk), //input clkb
                  .ceb(1'b1), //input ceb
                  .oceb(1'b1), //input oceb
                  .adb(desc_rom_adb), //input [10:0] adb
                  .wreb(1'b0), //input wreb
                  .dinb(8'h00), //input [7:0] dinb
                  .doutb(desc_rom_doutb), //output [7:0] doutb
                  .resetb(~USB_CR_EN) //input resetb
              );

    always @(*) begin
        if (write_en && addr[ADDRWIDTH-1] == 1'b1) begin
            desc_rom_ada = addr[10:0];
            desc_rom_wrea = ~USB_CR_EN;
        end
        else begin
            desc_rom_ada = haddrs[10:0];
            desc_rom_wrea = 1'b0;
        end
    end
    //-----------------------------------------------------------
    // USB logic
    //----------------------------------------------------------


    localparam ENDPT0 = 4'd0;

    wire endpt0_txval;
    wire [7:0] endpt0_dat;
    wire [3:0] endpt_sel;
    wire usb_txval = endpt0_txval & (endpt_sel == ENDPT0);
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
    wire [11:0] endpt0_txdat_len;
    wire [7:0] ep_usb_txdat;
    wire ep_usb_rxrdy;
    wire ep_usb_txcork;
    wire endpt0_rxrdy;


    /* 根据USB端点号选通数据线 */
    assign usb_rxrdy     = (endpt_sel == ENDPT0) ? endpt0_rxrdy : ep_usb_rxrdy;
    assign usb_txcork    = (endpt_sel == ENDPT0) ? 1'd0 : ep_usb_txcork;
    assign usb_txdat     = (endpt_sel == ENDPT0) ? endpt0_dat : ep_usb_txdat;
    assign usb_txdat_len = (endpt_sel == ENDPT0) ? endpt0_txdat_len : ep_usb_txlen;

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

                                  .descrom_rdata_i(desc_rom_doutb),
                                  .descrom_raddr_o(desc_rom_adb),
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

    // endpt0_rxrdy i 接收就绪信号，高电平时表示可接收 RXDAT。
    // endpt0_txval i 控制数据有效指示信号，仅用于传输端点0的控制数据，高电平时表示用户输入数据有效，传输非控制数据时应置0。
    // endpt0_dat i [7:0] 发数据，IP将使用此数据通过USB接口发出。
    // endpt0_txdat_len i [11:0] 发送数据字节数，可用于控制TX数据字节数据。
    // endpt0_txcork i 发数据有效信号，低电平时表示TXDAT不足。
    // usb_rxval o 数据有效指示信号，高电平时表示 USB 配置数据效。
    // usb_txact o 发送工作信号，高电平表示设备进入数据发送状态。
    // usb_txpop o 发送读信号，高电平时表示读取下一个数据。

    // ep0接收数据有效信号
    wire ep0_rx_val = usb_rxval && (endpt_sel == 4'd0);
    wire ep0_rx_fifo_empty;
    wire ep0_rx_fifo_full;
    wire [7:0] ep0_rx_fifo_rdata;
    wire [11:0] ep0_rx_fifo_num;
    assign endpt0_rxrdy = !ep0_rx_fifo_full;

    fifo_sc_top ep0_rx_fifo (
                    .Clk(hclk), //input Clk
                    .Reset(usb_link_rst), //input Reset

                    .WrEn(ep0_rx_val), //input WrEn
                    .Data(usb_rxdat), //input [7:0] Data

                    .RdEn(read_en && addr_equ(addr, USB_EP0_FIFO_ADDR)), //input RdEn
                    .Q(ep0_rx_fifo_rdata), //output [7:0] Q

                    .Wnum(ep0_rx_fifo_num), //output [9:0] Wnum
                    .Empty(ep0_rx_fifo_empty), //output Empty
                    .Full(ep0_rx_fifo_full) //output Full
                );

    reg usb_epout_int_sig;
    reg usb_epin_int_sig;
    reg usb_setup_int_sig;
    reg [3:0] usb_sel_ep_store;
    reg usb_txact_store;
    reg usb_rxact_store;
    reg usb_setup_store;
    always @(posedge hclk or posedge usb_link_rst) begin
        if (usb_link_rst) begin
            usb_setup_int_sig <= 1'd0;
            usb_setup_store <= 1'd0;
            usb_epout_int_sig <= 1'd0;
            usb_txact_store <= 1'd0;
            usb_rxact_store <= 1'd0;
            usb_sel_ep_store <= 4'd0;
        end
        else begin
            // posedge rxact 锁存端点选择寄存器
            if (!usb_rxact_store && usb_rxact)
                usb_sel_ep_store <= endpt_sel;
            // negedge txact 触发IN中断
            if (usb_txact_store && !usb_txact)
                usb_epin_int_sig <= 1'd1;
            // negedge rxact 触发OUT中断
            if (usb_rxact_store && !usb_rxact)
                usb_epout_int_sig <= 1'd1;
            // negedge setup 触发SETUP中断
            if (usb_setup_store && !setup_active)
                usb_setup_int_sig <= 1'd1;

            usb_rxact_store <= usb_rxact;
            usb_txact_store <= usb_txact;
            usb_setup_store <= setup_active;

            // 写入SR清除中断标志
            if (write_en && addr_equ(addr, USB_SR_ADDR) && byte_strobe_reg[3]) begin
                if (hwdatas[31])
                    usb_epout_int_sig <= 1'd0;
                if (hwdatas[30])
                    usb_epin_int_sig <= 1'd0;
                if (hwdatas[29])
                    usb_setup_int_sig <= 1'd0;
            end
        end
    end

    wire ep0_tx_fifo_empty;
    wire ep0_tx_fifo_full;
    wire [11:0] ep0_tx_fifo_num;
    wire [7:0] ep0_tx_fifo_rddata;
    wire ep0_tx_fifo_rready;

    fifo_sc_top ep0_tx_fifo(
                    .Clk(hclk), //input Clk
                    .Reset(usb_link_rst), //input Reset

                    .WrEn(write_en && addr_equ(addr, USB_EP0_FIFO_ADDR)), //input WrEn
                    .Data(hwdatas[7:0]), //input [7:0] Data

                    .Q(endpt0_dat), //output [7:0] Q
                    .RdEn(usb_txpop & (endpt_sel == 4'd0)), //input RdEn

                    .Wnum(ep0_tx_fifo_num), //output [9:0] Wnum
                    .Empty(ep0_tx_fifo_empty), //output Empty
                    .Full(ep0_tx_fifo_full) //output Full
                );

    // usb_txact有效锁定长度
    reg [11:0] endpt0_txdat_len_r;
    always @(posedge hclk or posedge usb_link_rst) begin
        if (usb_link_rst) begin
            endpt0_txdat_len_r <= 12'd0;
        end
        else begin
            if (!usb_txact)
                endpt0_txdat_len_r <= ep0_tx_fifo_num;
        end
    end
    assign endpt0_txdat_len = endpt0_txdat_len_r;
    assign endpt0_txval = !ep0_tx_fifo_empty;


    usb_fifo usb_fifo_u (
                 .i_clk(hclk),
                 .i_reset(usb_link_rst),
                 .i_usb_endpt(endpt_sel),
                 .i_usb_rxact(usb_rxact),
                 .i_usb_rxval(usb_rxval),
                 .i_usb_rxpktval(usb_rxpktval),
                 .i_usb_rxdat(usb_rxdat),
                 .o_usb_rxrdy(ep_usb_rxrdy),
                 .i_usb_txact(usb_txact),
                 .i_usb_txpop(usb_txpop),
                 .i_usb_txpktfin(usb_txpktfin),
                 .o_usb_txcork(ep_usb_txcork),
                 .o_usb_txlen(ep_usb_txlen),
                 .o_usb_txdat(ep_usb_txdat),

                 // WinUSB IN
                 .i_ep1_tx_clk(hclk),
                 .i_ep1_tx_max(TRANS_MAX),
                 .i_ep1_tx_dval(winusb_in_tvalid),
                 .i_ep1_tx_data(winusb_in_tdata),

                 // WinUSB OUT
                 .i_ep2_rx_clk(hclk),
                 .i_ep2_rx_rdy(winusb_out_tready),
                 .o_ep2_rx_dval(winusb_out_tvalid),
                 .o_ep2_rx_data(winusb_out_tdata),

                 // SWO IN
                 .i_ep6_rx_clk(hclk),
                 .i_ep6_rx_rdy(1'd0),
                 .o_ep6_rx_dval(),
                 .o_ep6_rx_data(),

                 // CDC IN
                 .i_ep3_tx_clk(hclk),
                 .i_ep3_tx_max(TRANS_MAX),
                 .i_ep3_tx_dval(cdc_in_tvalid),
                 .i_ep3_tx_data(cdc_in_tdata),

                 // CDC OUT
                 .i_ep4_rx_clk(hclk),
                 .i_ep4_rx_rdy(cdc_out_tready),
                 .o_ep4_rx_dval(cdc_out_tvalid),
                 .o_ep4_rx_data(cdc_out_tdata)

                 //  HID IN
                //  .i_ep8_tx_clk(hclk),
                //  .i_ep8_tx_max(TRANS_MAX),
                //  .i_ep8_tx_dval(),
                //  .i_ep8_tx_data(),

                 //  HID OUT
                //  .i_ep9_rx_clk(hclk),
                //  .i_ep9_rx_rdy(),
                //  .o_ep9_rx_dval(),
                //  .o_ep9_rx_data()
             );

    assign usb_nrst = USB_CR_EN & hresetn;
    //assign intr = ~acm_ctrl_rx_rdy;

    //-----------------------------------------------------------
    // AHB读逻辑
    //----------------------------------------------------------

    //  0  0: SR
    //  1  4: CR
    //  2  8: { EP0-16_RXFF, EP0-16_RXFE }
    //  3 12: { EP0-16_TXFF, EP0-16_TXFE }
    //  4 16: EP0 FIFO
    //  5 20: EP1 FIFO  X
    //  6 24: EP2 FIFO  X
    //  7 28: EP3 FIFO  X
    //  8 32: EP4 FIFO  X
    //  9 36: EP5 FIFO  X
    // 10 40: EP6 FIFO  X
    // 11 44: EP7 FIFO  X
    // 12 48: EP8 FIFO  X
    // 13 52: EP9 FIFO  X
    // 14 56: EP10 FIFO X
    // 15 60: EP11 FIFO X
    // 16 64: EP12 FIFO X
    // 17 68: EP13 FIFO X
    // 18 72: EP14 FIFO X
    // 19 76: EP15 FIFO X
    // 20 80: EP0 TXNUM RXNUM

    always @(*) begin
        if (read_en) begin
            if (addr[ADDRWIDTH-1] == 1'b1) begin
                hrdatas = {4{desc_rom_douta}};
            end
            else if (addr[ADDRWIDTH-1:8] == 4'b1) begin
                if (addr[7:2] < 6'd11)
                    hrdatas = USB_REGS[addr[5:2]];
                else
                    hrdatas = {32{1'bx}};
            end
            else begin
                // REGS
                case (addr[7:2])
                    USB_SR_ADDR[7:2]:
                        hrdatas = {
                            usb_epout_int_sig,
                            usb_epin_int_sig,
                            usb_setup_int_sig,
                            21'd0,
                            usb_sel_ep_store,
                            usb_setup_store,
                            usb_status_hispeed,
                            usb_status_suspend,
                            usb_status_online
                        };
                    USB_CR_ADDR[7:2]:
                        hrdatas = usb_ctrl_reg;
                    USB_FIFO_RX_FLAG_ADDR[7:2]:
                        hrdatas = {
                            15'd0,
                            ep0_rx_fifo_full,
                            15'd0,
                            ep0_rx_fifo_empty
                        };
                    USB_FIFO_TX_FLAG_ADDR[7:2]:
                        hrdatas = {
                            15'd0,
                            ep0_tx_fifo_full,
                            15'd0,
                            ep0_tx_fifo_empty
                        };
                    USB_EP0_FIFO_ADDR[7:2]:
                        hrdatas = ep0_rx_fifo_rdata;
                    USB_EP0_FIFO_NUM_ADDR[7:2]:
                        hrdatas = {4'd0, ep0_tx_fifo_num, 4'd0, ep0_rx_fifo_num};
                    default:

                        hrdatas = {32{1'bx}};
                endcase
            end
        end
        else begin
            hrdatas = {32{1'bx}};
        end
    end

    // 中断信号
    assign intr = (USB_EPOUT_INT_EN & usb_epout_int_sig) ||
           (USB_EPIN_INT_EN & usb_epin_int_sig) ||
           (USB_SETUP_INT_EN & usb_setup_int_sig);
endmodule
