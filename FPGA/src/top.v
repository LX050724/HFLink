module HFLink_TOP(
        input clk_osc
        ,output ready_led
        ,output done_led
        ,inout swdio
        ,inout swclk
        ,input reset

        ,inout FLASH_SPI_HOLDN_io
        ,inout FLASH_SPI_CSN_io
        ,inout FLASH_SPI_MISO_io
        ,inout FLASH_SPI_MOSI_io
        ,inout FLASH_SPI_WPN_io
        ,inout FLASH_SPI_CLK_io

        ,output UART_TX
        ,input  UART_RX
        ,output UART_DE
        ,output UART_RTS
        ,output UART_DTR

        ,output usb_clk
        ,output usb_rstn
        ,inout [7:0] usb_ulpi_data
        ,input usb_ulpi_nxt
        ,input usb_ulpi_dir
        ,output usb_ulpi_stp
    );

    wire lock;
    wire clkout0;
    wire clkout1;
    wire n_reset = !reset;
    wire m1_halt;
    wire [3:0] intr;
    assign ready_led = !m1_halt;

    wire [31:0] AHB1HRDATA;
    wire        AHB1HREADYOUT;
    wire [ 1:0] AHB1HRESP;
    wire [ 1:0] AHB1HTRANS;
    wire [ 2:0] AHB1HBURST;
    wire [ 3:0] AHB1HPROT;
    wire [ 2:0] AHB1HSIZE;
    wire        AHB1HWRITE;
    wire        AHB1HREADYMUX;
    wire [ 3:0] AHB1HMASTER;
    wire        AHB1HMASTLOCK;
    wire [31:0] AHB1HADDR;
    wire [31:0] AHB1HWDATA;
    wire        AHB1HSEL;
    wire        AHB1HCLK;
    wire        AHB1HRESET;

    wire [31:0] AHB2HRDATA;
    wire        AHB2HREADYOUT;
    wire [ 1:0] AHB2HRESP;
    wire [ 1:0] AHB2HTRANS;
    wire [ 2:0] AHB2HBURST;
    wire [ 3:0] AHB2HPROT;
    wire [ 2:0] AHB2HSIZE;
    wire        AHB2HWRITE;
    wire        AHB2HREADYMUX;
    wire [ 3:0] AHB2HMASTER;
    wire        AHB2HMASTLOCK;
    wire [31:0] AHB2HADDR;
    wire [31:0] AHB2HWDATA;
    wire        AHB2HSEL;
    wire        AHB2HCLK;
    wire        AHB2HRESET;

    wire [31:0] APB1PADDR;
    wire        APB1PENABLE;
    wire        APB1PWRITE;
    wire [ 3:0] APB1PSTRB;
    wire [ 2:0] APB1PPROT;
    wire [31:0] APB1PWDATA;
    wire        APB1PSEL;
    wire [31:0] APB1PRDATA;
    wire        APB1PREADY;
    wire        APB1PSLVERR;
    wire        APB1PCLK;
    wire        APB1PRESET;

    Gowin_PLL main_pll(
                  .lock(lock), //output lock
                  .clkout0(clkout0), //output clkout0
                  .clkout1(clkout1), //output clkout1
                  .clkin(clk_osc), //input clkin
                  .mdclk(clk_osc),
                  .reset(reset) //input reset
              );

    wire cm_nreset = n_reset & lock;

    Gowin_EMPU_M1_Top Cortex_M1 (
                          //.LOCKUP(lockup), //output LOCKUP
                          .HALTED(m1_halt), //output HALTED
                          .JTAG_7(swdio), //inout JTAG_7
                          .JTAG_9(swclk), //inout JTAG_9
                          .HCLK(clkout0), //input HCLK
                          .hwRstn(cm_nreset), //input hwRstn


                          // AHB1 0x80000000
                          .AHB1HRDATA(AHB1HRDATA), //input [31:0] AHB1HRDATA
                          .AHB1HREADYOUT(AHB1HREADYOUT), //input AHB1HREADYOUT
                          .AHB1HRESP(AHB1HRESP), //input [1:0] AHB1HRESP
                          .AHB1HTRANS(AHB1HTRANS), //output [1:0] AHB1HTRANS
                          .AHB1HBURST(AHB1HBURST), //output [2:0] AHB1HBURST
                          .AHB1HPROT(AHB1HPROT), //output [3:0] AHB1HPROT
                          .AHB1HSIZE(AHB1HSIZE), //output [2:0] AHB1HSIZE
                          .AHB1HWRITE(AHB1HWRITE), //output AHB1HWRITE
                          .AHB1HREADYMUX(AHB1HREADYMUX), //output AHB1HREADYMUX
                          .AHB1HMASTER(AHB1HMASTER), //output [3:0] AHB1HMASTER
                          .AHB1HMASTLOCK(AHB1HMASTLOCK), //output AHB1HMASTLOCK
                          .AHB1HADDR(AHB1HADDR), //output [31:0] AHB1HADDR
                          .AHB1HWDATA(AHB1HWDATA), //output [31:0] AHB1HWDATA
                          .AHB1HSEL(AHB1HSEL), //output AHB1HSEL
                          .AHB1HCLK(AHB1HCLK), //output AHB1HCLK
                          .AHB1HRESET(AHB1HRESET), //output AHB1HRESET

                          // AHB2 0x81000000
                          .AHB2HRDATA(AHB2HRDATA), //input [31:0] AHB2HRDATA
                          .AHB2HREADYOUT(AHB2HREADYOUT), //input AHB2HREADYOUT
                          .AHB2HRESP(AHB2HRESP), //input [1:0] AHB2HRESP
                          .AHB2HTRANS(AHB2HTRANS), //output [1:0] AHB2HTRANS
                          .AHB2HBURST(AHB2HBURST), //output [2:0] AHB2HBURST
                          .AHB2HPROT(AHB2HPROT), //output [3:0] AHB2HPROT
                          .AHB2HSIZE(AHB2HSIZE), //output [2:0] AHB2HSIZE
                          .AHB2HWRITE(AHB2HWRITE), //output AHB2HWRITE
                          .AHB2HREADYMUX(AHB2HREADYMUX), //output AHB2HREADYMUX
                          .AHB2HMASTER(AHB2HMASTER), //output [3:0] AHB2HMASTER
                          .AHB2HMASTLOCK(AHB2HMASTLOCK), //output AHB2HMASTLOCK
                          .AHB2HADDR(AHB2HADDR), //output [31:0] AHB2HADDR
                          .AHB2HWDATA(AHB2HWDATA), //output [31:0] AHB2HWDATA
                          .AHB2HSEL(AHB2HSEL), //output AHB2HSEL
                          .AHB2HCLK(AHB2HCLK), //output AHB2HCLK
                          .AHB2HRESET(AHB2HRESET), //output AHB2HRESET

                          // APB1 0x60000000
                          .APB1PADDR(APB1PADDR), //output [31:0] APB1PADDR
                          .APB1PENABLE(APB1PENABLE), //output APB1PENABLE
                          .APB1PWRITE(APB1PWRITE), //output APB1PWRITE
                          .APB1PSTRB(APB1PSTRB), //output [3:0] APB1PSTRB
                          .APB1PPROT(APB1PPROT), //output [2:0] APB1PPROT
                          .APB1PWDATA(APB1PWDATA), //output [31:0] APB1PWDATA
                          .APB1PSEL(APB1PSEL), //output APB1PSEL
                          .APB1PRDATA(APB1PRDATA), //input [31:0] APB1PRDATA
                          .APB1PREADY(APB1PREADY), //input APB1PREADY
                          .APB1PSLVERR(APB1PSLVERR), //input APB1PSLVERR
                          .APB1PCLK(APB1PCLK), //output APB1PCLK
                          .APB1PRESET(APB1PRESET), //output APB1PRESET

                          // EXFLASH
                          .FLASH_SPI_HOLDN(FLASH_SPI_HOLDN_io), //inout FLASH_SPI_HOLDN
                          .FLASH_SPI_CSN(FLASH_SPI_CSN_io), //inout FLASH_SPI_CSN
                          .FLASH_SPI_MISO(FLASH_SPI_MISO_io), //inout FLASH_SPI_MISO
                          .FLASH_SPI_MOSI(FLASH_SPI_MOSI_io), //inout FLASH_SPI_MOSI
                          .FLASH_SPI_WPN(FLASH_SPI_WPN_io), //inout FLASH_SPI_WPN
                          .FLASH_SPI_CLK(FLASH_SPI_CLK_io), //inout FLASH_SPI_CLK

                          .EXTINT(intr) //input [3:0] EXTINT
                      );

    wire [7:0] usb_ulpi_data_i;
    wire [7:0] usb_ulpi_data_o;

    wire winusb_in_tvalid;
    wire [7:0] winusb_in_tdata;
    wire winusb_out_tready;
    wire winusb_out_tvalid;
    wire [7:0] winusb_out_tdata;
    wire cdc_in_tvalid;
    wire [7:0] cdc_in_tdata;
    wire cdc_out_tready;
    wire cdc_out_tvalid;
    wire [7:0] cdc_out_tdata;

    AHB_USBDevice u_usb0 (
                      .hclk(AHB1HCLK),
                      .hresetn(AHB1HRESET),
                      .hsels(AHB1HSEL),
                      .haddrs(AHB1HADDR[11:0]),
                      .htranss(AHB1HTRANS),
                      .hsizes(AHB1HSIZE),
                      .hwrites(AHB1HWRITE),
                      .hreadys(AHB1HREADYMUX),
                      .hwdatas(AHB1HWDATA),
                      .hreadyouts(AHB1HREADYOUT),
                      .hresps(AHB1HRESP[0]),
                      .hrdatas(AHB1HRDATA),
                      .intr(intr[0]),
                      .usb_ulpi_data_i(usb_ulpi_data_i),
                      .usb_ulpi_data_o(usb_ulpi_data_o),
                      .usb_ulpi_data(usb_ulpi_data),
                      .usb_ulpi_nxt(usb_ulpi_nxt),
                      .usb_ulpi_dir(usb_ulpi_dir),
                      .usb_ulpi_stp(usb_ulpi_stp),
                      .usb_nrst(usb_rstn),

                      .winusb_in_tvalid(winusb_in_tvalid),
                      .winusb_in_tdata(winusb_in_tdata),
                      .winusb_out_tready(winusb_out_tready),
                      .winusb_out_tvalid(winusb_out_tvalid),
                      .winusb_out_tdata(winusb_out_tdata),
                      .cdc_in_tvalid(cdc_in_tvalid),
                      .cdc_in_tdata(cdc_in_tdata),
                      .cdc_out_tready(cdc_out_tready),
                      .cdc_out_tvalid(cdc_out_tvalid),
                      .cdc_out_tdata(cdc_out_tdata)
                  );
    assign usb_clk = clkout0;

    genvar i;
    generate
        for(i = 0; i < 8; i = i + 1) begin: io_genfor
            IOBUF usb_data_io_u(
                      .O(usb_ulpi_data_i[i]),
                      .I(usb_ulpi_data_o[i]),
                      .IO(usb_ulpi_data[i]),
                      .OEN(usb_ulpi_dir)
                  );
        end
    endgenerate


    APB_Stream_UART apb_stream_uart(
                        // APB
                        .PCLK(APB1PCLK),
                        .PWRITE(APB1PWRITE),
                        .PSEL(APB1PSEL),
                        .PENABLE(APB1PENABLE),
                        .PADDR(APB1PADDR),
                        .PSTRB(APB1PSTRB),
                        .PWDATA(APB1PWDATA),
                        .PRDATA(APB1PRDATA),
                        .PREADY(APB1PREADY),
                        .PRESETn(APB1PRESET),

                        // Data Stream
                        .tx_tvalid(cdc_out_tvalid),
                        .tx_tready(cdc_out_tready),
                        .tx_tdata(cdc_out_tdata),
                        .rx_tvalid(cdc_in_tvalid),
                        .rx_tdata(cdc_in_tdata),

                        // UART IO
                        .UART_TX(UART_TX),
                        .UART_RX(UART_RX),
                        .UART_DE(UART_DE),
                        .UART_RTS(UART_RTS),
                        .UART_DTR(UART_DTR)
                    );

    DAP_Controller dap_controller_inst(
                       .hclk(AHB2HCLK),
                       .hresetn(AHB2HRESET),
                       .hsels(AHB2HSEL),
                       .haddrs(AHB2HADDR[11:0]),
                       .htranss(AHB2HTRANS),
                       .hsizes(AHB2HSIZE),
                       .hwrites(AHB2HWRITE),
                       .hreadys(AHB2HREADYMUX),
                       .hwdatas(AHB2HWDATA),
                       .hreadyouts(AHB2HREADYOUT),
                       .hresps(AHB2HRESP[0]),
                       .hrdatas(AHB2HRDATA),

                       .dap_in_tvalid(winusb_out_tvalid),
                       .dap_in_tready(winusb_out_tready),
                       .dap_in_tdata(winusb_out_tdata),
                       .dat_out_tvalid(winusb_in_tvalid),
                       .dap_out_tdata(winusb_in_tdata),
                       .intr(intr[1])
                   );
endmodule
