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

        ,output usb_clk
        ,output usb_rstn
        ,inout [7:0] usb_ulpi_data
        ,input usb_ulpi_nxt
        ,input usb_ulpi_dir
        ,output usb_ulpi_stp
    );


    wire lock;
    wire clkout0;
    wire n_reset = !reset;
    wire m1_halt;
    wire [3:0] intr;
    assign ready_led = !m1_halt;

    wire [31:0] AHB1HRDATA;
    wire AHB1HREADYOUT;
    wire [1:0] AHB1HRESP;
    wire [1:0] AHB1HTRANS;
    wire [2:0] AHB1HBURST;
    wire [3:0] AHB1HPROT;
    wire [2:0] AHB1HSIZE;
    wire AHB1HWRITE;
    wire AHB1HREADYMUX;
    wire [3:0] AHB1HMASTER;
    wire AHB1HMASTLOCK;
    wire [31:0] AHB1HADDR;
    wire [31:0] AHB1HWDATA;
    wire AHB1HSEL;
    wire AHB1HCLK;
    wire AHB1HRESET;

    Gowin_PLL main_pll(
                  .lock(lock), //output lock
                  .clkout0(clkout0), //output clkout0
                  .clkin(clk_osc), //input clkin
                  // .mdclk(clk_osc),
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


                          // AHB1 0x60000000
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

    AHB_USBDevice u_usb0 (
                      .hclk(clkout0),
                      .hresetn(AHB1HRESET),
                      .hsels(AHB1HSEL),
                      .haddrs(AHB1HADDR[31:0]),
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
                      .usb_nrst(usb_rstn)
                  );

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

    assign usb_clk = clkout0;

    // wire [31:0] reg_addr;
    // wire reg_read_en;
    // wire reg_write_en;
    // wire [3:0] reg_byte_strobe;
    // wire [31:0] reg_wdata;
    // reg [31:0] reg_rdata;

    // reg [31:0] regs[0:15]; // 16 registers for testing

    // cmsdk_ahb_eg_slave_interface  ahb_test(
    //     .hclk(clkout0),
    //     .hresetn(AHB1HRESET),
    //     .hsels(AHB1HSEL),
    //     .haddrs(AHB1HADDR[31:0]),
    //     .htranss(AHB1HTRANS),
    //     .hsizes(AHB1HSIZE),
    //     .hwrites(AHB1HWRITE),
    //     .hreadys(AHB1HREADYMUX),
    //     .hwdatas(AHB1HWDATA),
    //     .hreadyouts(AHB1HREADYOUT),
    //     .hresps(), // unused
    //     .hrdatas(AHB1HRDATA),


    //     .addr(reg_addr),
    //     .read_en(reg_read_en),
    //     .write_en(reg_write_en),
    //     .byte_strobe(reg_byte_strobe),
    //     .wdata(reg_wdata),
    //     .rdata(reg_rdata)
    // );

    // always @(posedge clkout0 or negedge AHB1HRESET) begin
    //     if (!AHB1HRESET) begin
    //         // reset all registers to 0
    //         regs[0] <= 32'b0;
    //         regs[1] <= 32'b0;
    //         regs[2] <= 32'b0;
    //         regs[3] <= 32'b0;
    //         regs[4] <= 32'b0;
    //         regs[5] <= 32'b0;
    //         regs[6] <= 32'b0;
    //         regs[7] <= 32'b0;
    //         regs[8] <= 32'b0;
    //         regs[9] <= 32'b0;
    //         regs[10] <= 32'b0;
    //         regs[11] <= 32'b0;
    //         regs[12] <= 32'b0;
    //         regs[13] <= 32'b0;
    //         regs[14] <= 32'b0;
    //         regs[15] <= 32'b0;
    //     end else if (reg_write_en) begin
    //         // write to registers with byte strobe
    //         if (reg_byte_strobe[0])
    //             regs[reg_addr[5:2]][0 +: 8] <= reg_wdata[0 +: 8];
    //         if (reg_byte_strobe[1])
    //             regs[reg_addr[5:2]][8 +: 8] <= reg_wdata[8 +: 8];
    //         if (reg_byte_strobe[2])
    //             regs[reg_addr[5:2]][16 +: 8] <= reg_wdata[16 +: 8];
    //         if (reg_byte_strobe[3])
    //             regs[reg_addr[5:2]][24 +: 8] <= reg_wdata[24 +: 8];
    //     end
    // end

    // always @(*) begin
    //     if (reg_read_en) begin
    //         reg_rdata = regs[reg_addr[5:2]];
    //     end else begin
    //         reg_rdata = {32{1'bx}};
    //     end
    // end

    // assign done_led = regs[0][0];
    // assign intr = regs[1];

endmodule
