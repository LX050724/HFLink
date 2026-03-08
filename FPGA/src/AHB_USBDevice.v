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

        // USB外部控制端口
        output [3:0] ext_usb_endpt,
        output ext_usb_txact,
        output ext_usb_txpop,
        output ext_usb_txpktfin,
        input ext_usb_txcork,
        input [7:0] ext_usb_txdata,
        input [11:0] ext_usb_txlen,

        output [7:0] ext_usb_rxdat,
        output ext_usb_rxval,
        input ext_usb_rxrdy,
        output ext_usb_rxact,
        output ext_usb_rxpktval,

        // 数据AxisStream
        input cdc_in_tvalid,
        input [7:0] cdc_in_tdata,
        input cdc_out_tready,
        output cdc_out_tvalid,
        output [7:0] cdc_out_tdata
    );
    integer i;

    localparam [ADDRWIDTH-1:0] USB_SR_ADDR      = 12'h000;
    localparam [ADDRWIDTH-1:0] USB_CR_ADDR      = 12'h004;
    localparam [ADDRWIDTH-1:0] USB_DESC_ADDR    = 12'h008;

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

    reg usb_ctrl_reg;

    wire USB_CR_EN = usb_ctrl_reg;

    always @(posedge hclk or negedge hresetn) begin
        if (!hresetn) begin
            usb_ctrl_reg <= 1'd0;
        end
        else begin
            if (write_en && addr_equ(addr, USB_CR_ADDR)) begin
                if (byte_strobe_reg[0]) begin
                    usb_ctrl_reg <= hwdatas[0];
                end
            end
        end
    end

    //-----------------------------------------------------------
    // USB logic
    //----------------------------------------------------------

    localparam ENDPT0 = 4'd0;
    localparam ENDPT1 = 4'd1;
    localparam ENDPT2 = 4'd2;

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

    wire [15:0] descrom_raddr;
    wire [ 7:0] desc_index;
    wire [ 7:0] desc_type;
    wire [ 7:0] descrom_rdata;
    wire [15:0] desc_dev_addr;
    wire [15:0] desc_dev_len;
    wire [15:0] desc_qual_addr;
    wire [15:0] desc_qual_len;
    wire [15:0] desc_fscfg_addr;
    wire [15:0] desc_fscfg_len;
    wire [15:0] desc_hscfg_addr;
    wire [15:0] desc_hscfg_len;
    wire [15:0] desc_oscfg_addr;
    wire [15:0] desc_hidrpt_addr;
    wire [15:0] desc_hidrpt_len;
    wire [15:0] desc_bos_addr;
    wire [15:0] desc_bos_len;
    wire [15:0] desc_strlang_addr;
    wire [15:0] desc_strvendor_addr;
    wire [15:0] desc_strvendor_len;
    wire [15:0] desc_strproduct_addr;
    wire [15:0] desc_strproduct_len;
    wire [15:0] desc_strserial_addr;
    wire [15:0] desc_strserial_len;
    wire desc_have_strings;

    /* 根据USB端点号选通数据线 */
    assign usb_rxrdy     = (endpt_sel == ENDPT0) ? endpt0_rxrdy : 
                           (endpt_sel == ENDPT2) ? ext_usb_rxrdy : 
                                                   ep_usb_rxrdy;

    assign usb_txcork    = (endpt_sel == ENDPT0) ? 1'd0 : 
                           (endpt_sel == ENDPT1) ? ext_usb_txcork :
                                                   ep_usb_txcork;

    assign usb_txdat     = (endpt_sel == ENDPT0) ? endpt0_dat : 
                           (endpt_sel == ENDPT1) ? ext_usb_txdata : 
                                                   ep_usb_txdat;

    assign usb_txdat_len = (endpt_sel == ENDPT0) ? endpt0_txdat_len : 
                           (endpt_sel == ENDPT1) ? ext_usb_txlen : 
                                                   ep_usb_txlen;

    assign ext_usb_endpt = endpt_sel;
    assign ext_usb_txact = usb_txact;
    assign ext_usb_txpop = usb_txpop;
    assign ext_usb_txpktfin = usb_txpktfin;
    assign ext_usb_rxdat = usb_rxdat;
    assign ext_usb_rxval = usb_rxval;
    assign ext_usb_rxact = usb_rxact;
    assign ext_usb_rxpktval = usb_rxpktval;


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
            if (inf_set & inf_sel < 8'd4) begin
                interface_alter[inf_sel[1:0]] <= inf_alter_o;
            end
        end
    end

    assign inf_alter_i = inf_sel < 8'd4 ? interface_alter[inf_sel[1:0]] : 8'd0;

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

                                  .descrom_rdata_i(descrom_rdata),
                                  .descrom_raddr_o(descrom_raddr),
                                  .desc_dev_addr_i(desc_dev_addr),
                                  .desc_dev_len_i(desc_dev_len),
                                  .desc_qual_addr_i(desc_qual_addr),
                                  .desc_qual_len_i(desc_qual_len),
                                  .desc_fscfg_addr_i(desc_fscfg_addr),
                                  .desc_fscfg_len_i(desc_fscfg_len),
                                  .desc_hscfg_addr_i(desc_hscfg_addr),
                                  .desc_hscfg_len_i(desc_hscfg_len),
                                  .desc_oscfg_addr_i(desc_oscfg_addr),
                                  .desc_hidrpt_addr_i(desc_hidrpt_addr),
                                  .desc_hidrpt_len_i(desc_hidrpt_len),
                                  .desc_bos_addr_i(desc_bos_addr),
                                  .desc_bos_len_i(desc_bos_len),
                                  .desc_strlang_addr_i(desc_strlang_addr),
                                  .desc_strvendor_addr_i(desc_strvendor_addr),
                                  .desc_strvendor_len_i(desc_strvendor_len),
                                  .desc_strproduct_addr_i(desc_strproduct_addr),
                                  .desc_strproduct_len_i(desc_strproduct_len),
                                  .desc_strserial_addr_i(desc_strserial_addr),
                                  .desc_strserial_len_i(desc_strserial_len),
                                  .desc_have_strings_i(desc_have_strings),

                                  .ulpi_nxt_i(usb_ulpi_nxt),
                                  .ulpi_dir_i(usb_ulpi_dir),
                                  .ulpi_rxdata_i(usb_ulpi_data_i),
                                  .ulpi_txdata_o(usb_ulpi_data_o),
                                  .ulpi_stp_o(usb_ulpi_stp)
                              );

    wire [31:0] usb_desc_rdata;
    USB_DESC #(
        .BASE_ADDR(USB_DESC_ADDR)
    ) usb_desc_u (
        .clk(hclk),
        .resetn(hresetn),
        .usbrst(usb_link_rst),

        .ahb_write_en(write_en),
        .ahb_read_en(read_en),
        .ahb_addr(addr),
        .ahb_rdata(usb_desc_rdata),
        .ahb_wdata(hwdatas),
        .ahb_byte_strobe(byte_strobe_reg),

        .txdat(endpt0_dat),
        .txval(endpt0_txval),
        .txdat_len(endpt0_txdat_len),
        .txpop(usb_txpop),
        .txact(usb_txact),
        .rxdat(usb_rxdat),
        .rxval(usb_rxval),
        .rxrdy(endpt0_rxrdy),
        .rxact(usb_rxact),
        .setup(setup_active),
        .endpt_sel(endpt_sel),

        .descrom_raddr(descrom_raddr),
        .desc_index(desc_index),
        .desc_type(desc_type),
        .descrom_rdata(descrom_rdata),
        .desc_dev_addr(desc_dev_addr),
        .desc_dev_len(desc_dev_len),
        .desc_qual_addr(desc_qual_addr),
        .desc_qual_len(desc_qual_len),
        .desc_fscfg_addr(desc_fscfg_addr),
        .desc_fscfg_len(desc_fscfg_len),
        .desc_hscfg_addr(desc_hscfg_addr),
        .desc_hscfg_len(desc_hscfg_len),
        .desc_oscfg_addr(desc_oscfg_addr),
        .desc_hidrpt_addr(desc_hidrpt_addr),
        .desc_hidrpt_len(desc_hidrpt_len),
        .desc_bos_addr(desc_bos_addr),
        .desc_bos_len(desc_bos_len),
        .desc_strlang_addr(desc_strlang_addr),
        .desc_strvendor_addr(desc_strvendor_addr),
        .desc_strvendor_len(desc_strvendor_len),
        .desc_strproduct_addr(desc_strproduct_addr),
        .desc_strproduct_len(desc_strproduct_len),
        .desc_strserial_addr(desc_strserial_addr),
        .desc_strserial_len(desc_strserial_len),
        .desc_have_strings(desc_have_strings),
        .intr(intr)
    );

    wire [11:0] TRANS_MAX = usb_status_hispeed ? 12'd512 : 12'd64;

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

    //-----------------------------------------------------------
    // AHB读逻辑
    //----------------------------------------------------------
    always @(*) begin
        if (read_en) begin
            // REGS
            case (addr[ADDRWIDTH-1:2])
                USB_SR_ADDR[ADDRWIDTH-1:2]:
                    hrdatas = {
                        28'd0,
                        usb_status_hispeed,
                        usb_status_suspend,
                        usb_status_online
                    };
                USB_CR_ADDR[ADDRWIDTH-1:2]:
                    hrdatas = {31'd0, usb_ctrl_reg};
                default:
                    hrdatas = usb_desc_rdata;
            endcase
        end
        else begin
            hrdatas = {32{1'bx}};
        end
    end
endmodule
