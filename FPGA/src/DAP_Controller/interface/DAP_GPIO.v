module DAP_GPIO #(
        parameter ADDRWIDTH = 12,
        parameter [ADDRWIDTH-1:0] BASE_ADDR = 0
    )(
        input clk,
        input resetn,

        // AHB MEM接口
        input ahb_write_en,
        input [ADDRWIDTH-1:0] ahb_addr,
        output reg [31:0] ahb_rdata,
        input [31:0] ahb_wdata,
        input [3:0] ahb_byte_strobe,

        // swd模式信号
        input SWD_MODE,

        // 独立串口连接器
        output EXT_UART_TX,
        input EXT_UART_RX,

        // SWD/JTAG连接器端口
        output EXT_SWCLK_TCK_O,
        output EXT_SWDIO_TMS_T,
        output EXT_SWDIO_TMS_O,
        input EXT_SWDIO_TMS_I,
        input EXT_SWO_TDO_I,
        output EXT_TDI_O,
        input EXT_RTCK_I,
        input EXT_SRST_I,
        output EXT_SRST_O,
        input EXT_TRST_I,
        output EXT_TRST_O,

        // 内部SWJ信号
        input LOC_SWCLK_TCK_O,
        input LOC_SWDIO_TMS_T,
        input LOC_SWDIO_TMS_O,
        output LOC_SWDIO_TMS_I,
        output LOC_SWO_TDO_I,
        input LOC_TDI_O,
        output LOC_TRST_I,
        input LOC_TRST_O,
        output LOC_SRST_I,
        input LOC_SRST_O,

        // 内部SWO信号
        output LOC_SWO_I,

        // 内部返回时钟信号
        output LOC_RTCK_I,

        // 内部串口
        input LOC_UART_TX,
        output LOC_UART_RX
    );

    localparam [ADDRWIDTH-1:0] GPIO_CR_ADDR = BASE_ADDR + 0;  // RW
    localparam [ADDRWIDTH-1:0] SWCLK_TCK_O_DELAY_ADDR = BASE_ADDR + 1; // RW
    localparam [ADDRWIDTH-1:0] SWDIO_TMS_T_DELAY_ADDR = BASE_ADDR + 2;  // RW
    localparam [ADDRWIDTH-1:0] SWDIO_TMS_O_DELAY_ADDR = BASE_ADDR + 3; // RW
    localparam [ADDRWIDTH-1:0] SWDIO_TMS_I_DELAY_ADDR = BASE_ADDR + 4; // RW
    localparam [ADDRWIDTH-1:0] TDO_I_DELAY_ADDR = BASE_ADDR + 5; // RW
    localparam [ADDRWIDTH-1:0] TDI_O_DELAY_ADDR = BASE_ADDR + 6; // RW
    localparam [ADDRWIDTH-1:0] GPIO_MODE_ADDR = BASE_ADDR + 8; // R
    localparam [ADDRWIDTH-1:0] GPIO_STATUS_ADDR = BASE_ADDR + 10; // R
    localparam [ADDRWIDTH-1:0] GPIO_DO_ADDR = BASE_ADDR + 12; // RW
    localparam [ADDRWIDTH-1:0] GPIO_DO_SET_ADDR = BASE_ADDR + 13; // W
    localparam [ADDRWIDTH-1:0] GPIO_DO_RESET_ADDR = BASE_ADDR + 14; // W

    reg [7:0] GPIO_REG_CR;
    reg [1:0] GPIO_DO;

    wire ALONE_UART_CON = GPIO_REG_CR[0];
    wire TRST_DIRECT_EN = GPIO_REG_CR[1];
    wire SRST_DIRECT_EN = GPIO_REG_CR[2];
    wire TRST_DO = GPIO_DO[0];
    wire SRST_DO = GPIO_DO[1];

    reg [7:0] SWCLK_TCK_O_DELAY;
    reg [7:0] SWDIO_TMS_T_DELAY;
    reg [7:0] SWDIO_TMS_O_DELAY;
    reg [7:0] SWDIO_TMS_I_DELAY;
    reg [7:0] TDO_I_DELAY;
    reg [7:0] TDI_O_DELAY;


    IODELAY #(.DYN_DLY_EN("TRUE")) SWCLK_TCK_O_delay_inst (
                .DO(EXT_SWCLK_TCK_O),
                .DI(LOC_SWCLK_TCK_O),
                .SDTAP(1'd0),
                .VALUE(1'd0),
                .DLYSTEP(SWCLK_TCK_O_DELAY)
            );

    IODELAY #(.DYN_DLY_EN("TRUE")) SWDIO_TMS_T_delay_inst (
                .DO(EXT_SWDIO_TMS_T),
                .DI(LOC_SWDIO_TMS_T),
                .SDTAP(1'd0),
                .VALUE(1'd0),
                .DLYSTEP(SWDIO_TMS_T_DELAY)
            );

    IODELAY #(.DYN_DLY_EN("TRUE")) SWDIO_TMS_O_delay_inst (
                .DO(EXT_SWDIO_TMS_O),
                .DI(LOC_SWDIO_TMS_O),
                .SDTAP(1'd0),
                .VALUE(1'd0),
                .DLYSTEP(SWDIO_TMS_O_DELAY)
            );

    IODELAY #(.DYN_DLY_EN("TRUE")) SWDIO_TMS_I_delay_inst (
                .DO(LOC_SWDIO_TMS_I),
                .DI(EXT_SWDIO_TMS_I),
                .SDTAP(1'd0),
                .VALUE(1'd0),
                .DLYSTEP(SWDIO_TMS_I_DELAY)
            );

    IODELAY #(.DYN_DLY_EN("TRUE")) TDO_I_delay_inst (
                .DO(LOC_SWO_TDO_I),
                .DI(EXT_SWO_TDO_I),
                .SDTAP(1'd0),
                .VALUE(1'd0),
                .DLYSTEP(TDO_I_DELAY)
            );

    IODELAY #(.DYN_DLY_EN("TRUE")) TDI_O_delay_inst (
                .DO(EXT_TDI_O),
                .DI((!ALONE_UART_CON && SWD_MODE) ? LOC_UART_TX : LOC_TDI_O),
                .SDTAP(1'd0),
                .VALUE(1'd0),
                .DLYSTEP(TDI_O_DELAY)
            );

    assign LOC_TRST_I = EXT_TRST_I;
    assign EXT_TRST_O = TRST_DIRECT_EN ? TRST_DO : LOC_TRST_O;
    assign LOC_SRST_I = EXT_SRST_I;
    assign EXT_SRST_O = SRST_DIRECT_EN ? SRST_DO : LOC_SRST_O;
    assign EXT_UART_TX = ALONE_UART_CON ? LOC_UART_TX : 1'd1;
    assign LOC_UART_RX = EXT_UART_RX; 
    assign LOC_SWO_I = SWD_MODE ? LOC_SWO_TDO_I : 1'd0;

    always @(posedge clk or negedge resetn) begin : ahb_mem_write_ctrl
        if (!resetn) begin
            GPIO_REG_CR <= 8'd0;
            SWCLK_TCK_O_DELAY <= 8'd0;
            SWDIO_TMS_T_DELAY <= 8'd0;
            SWDIO_TMS_O_DELAY <= 8'd0;
            SWDIO_TMS_I_DELAY <= 8'd0;
            TDO_I_DELAY <= 8'd0;
            TDI_O_DELAY <= 8'd0;
            GPIO_DO <= 2'd0;
        end
        else begin
            if (ahb_write_en) begin
                case (ahb_addr[ADDRWIDTH-1:2])
                    GPIO_CR_ADDR[ADDRWIDTH-1:2]: begin
                        if (ahb_byte_strobe[0])
                            GPIO_REG_CR <= ahb_wdata[ 0+:8];
                        if (ahb_byte_strobe[1])
                            SWCLK_TCK_O_DELAY <= ahb_wdata[ 8+:8];
                        if (ahb_byte_strobe[2])
                            SWDIO_TMS_T_DELAY <= ahb_wdata[16+:8];
                        if (ahb_byte_strobe[3])
                            SWDIO_TMS_O_DELAY <= ahb_wdata[24+:8];
                    end
                    SWDIO_TMS_I_DELAY_ADDR[ADDRWIDTH-1:2]: begin
                        if (ahb_byte_strobe[0])
                            SWDIO_TMS_I_DELAY <= ahb_wdata[ 0+:8];
                        if (ahb_byte_strobe[1])
                            TDO_I_DELAY <= ahb_wdata[ 8+:8];
                        if (ahb_byte_strobe[2])
                            TDI_O_DELAY <= ahb_wdata[16+:8];
                    end
                    GPIO_DO_ADDR[ADDRWIDTH-1:2]: begin
                        if (ahb_byte_strobe[0])
                            GPIO_DO <= ahb_wdata[ 0+:8];
                        if (ahb_byte_strobe[1])
                            GPIO_DO <= GPIO_DO |  ahb_wdata[ 8+:8];
                        if (ahb_byte_strobe[2])
                            GPIO_DO <= GPIO_DO & ~ahb_wdata[16+:8];
                    end
                endcase
            end
        end
    end

    reg [9:0] GPIO_SYMPLE;
    always @(posedge clk or negedge resetn) begin
        if (!resetn) begin
            GPIO_SYMPLE <= 10'd0;
        end
        else begin
            GPIO_SYMPLE <= {
                            LOC_SWCLK_TCK_O,
                            LOC_SWDIO_TMS_T,
                            LOC_SWDIO_TMS_O,
                            LOC_SWDIO_TMS_I,
                            LOC_SWO_TDO_I,
                            LOC_TDI_O,
                            LOC_TRST_I,
                            LOC_TRST_O,
                            LOC_SRST_I,
                            LOC_SRST_O
                        };
        end
    end

    always @(*) begin : ahb_mem_read_ctrl
        case (ahb_addr[ADDRWIDTH-1:2])
            GPIO_CR_ADDR[ADDRWIDTH-1:2]:
                ahb_rdata = {SWDIO_TMS_O_DELAY, SWDIO_TMS_T_DELAY, SWCLK_TCK_O_DELAY, GPIO_REG_CR};
            SWDIO_TMS_I_DELAY_ADDR[ADDRWIDTH-1:2]:
                ahb_rdata = {8'd0, TDI_O_DELAY, TDO_I_DELAY, SWDIO_TMS_I_DELAY};
            GPIO_MODE_ADDR[ADDRWIDTH-1:2]:
                ahb_rdata = {15'd0, SWD_MODE, 6'd0, GPIO_SYMPLE};
            GPIO_DO_ADDR[ADDRWIDTH-1:2]:
                ahb_rdata = {24'd0, GPIO_DO};
            default:
                ahb_rdata = {32{1'bx}};
        endcase
    end

endmodule
