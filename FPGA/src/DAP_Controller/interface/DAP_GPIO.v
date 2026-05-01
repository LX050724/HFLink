module DAP_GPIO #(
        parameter ADDRWIDTH = 12,
        parameter [ADDRWIDTH-1:0] BASE_ADDR = 0
    )(
        input clk,
        input resetn,

        // AHB MEM接口
        input ahb_write_en,
        input ahb_read_en,
        input [ADDRWIDTH-1:0] ahb_addr,
        output reg [31:0] ahb_rdata,
        input [31:0] ahb_wdata,
        input [3:0] ahb_byte_strobe,

        // swd模式信号
        input SWD_MODE,

        // 独立串口连接器
        output reg EXT_UART_TX,
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
        output reg EXT_SRST_O,
        input EXT_TRST_I,
        output reg EXT_TRST_O,

        // DAP忙信号
        input DAP_BUSY,

        // LED控制信号
        output [3:0] DAP_GPIO,

        // 内部SPI信号
        input LOC_SPI_CSN,
        output LOC_SPI_MISO,
        input LOC_SPI_MOSI,
        input LOC_SPI_CLK,
        output LOC_SPI_MUX,

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
        output reg LOC_SWO_I,

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
    localparam [ADDRWIDTH-1:0] GPIO_STATUS_ADDR = BASE_ADDR + 8; // R
    localparam [ADDRWIDTH-1:0] GPIO_MODE_ADDR = BASE_ADDR + 10; // R
    localparam [ADDRWIDTH-1:0] GPIO_DO_ADDR = BASE_ADDR + 12; // RW
    localparam [ADDRWIDTH-1:0] GPIO_DO_SET_ADDR = BASE_ADDR + 13; // W
    localparam [ADDRWIDTH-1:0] GPIO_DO_RESET_ADDR = BASE_ADDR + 14; // W
    localparam [ADDRWIDTH-1:0] GPIO_LED_R_PWM_ADDR = BASE_ADDR + 16; // RW
    localparam [ADDRWIDTH-1:0] GPIO_LED_G_PWM_ADDR = BASE_ADDR + 17; // RW
    localparam [ADDRWIDTH-1:0] GPIO_LED_B_PWM_ADDR = BASE_ADDR + 18; // RW

    reg [7:0] GPIO_REG_CR;
    reg [7:0] GPIO_DO;

    wire ALONE_UART_CON = GPIO_REG_CR[0];
    wire DIRECT_IO_EN = GPIO_REG_CR[1];
    wire SPI_MODE_EN = GPIO_REG_CR[2];
    wire LED_MODE = GPIO_REG_CR[3];

    reg [7:0] SWCLK_TCK_O_DELAY;
    reg [7:0] SWDIO_TMS_T_DELAY;
    reg [7:0] SWDIO_TMS_O_DELAY;
    reg [7:0] SWDIO_TMS_I_DELAY;
    reg [7:0] TDO_I_DELAY;
    reg [7:0] TDI_O_DELAY;

    reg [7:0] RGB_PWM_CMP [2:0];

    reg mux_swclk_tck_o;
    reg mux_swdio_tms_t;
    reg mux_swdio_tms_o;
    reg mux_tdi_o;

    assign LOC_SPI_MUX = SPI_MODE_EN;
    assign LOC_SPI_MISO = LOC_SWO_TDO_I;

    always @(*) begin
        if (SPI_MODE_EN) begin
            mux_swclk_tck_o = LOC_SPI_CLK;
            mux_swdio_tms_t = 1'd0;
            mux_swdio_tms_o = LOC_SPI_CSN;
            mux_tdi_o = LOC_SPI_MOSI;
            EXT_SRST_O = GPIO_DO[4];
            EXT_TRST_O = GPIO_DO[5];
        end
        else if (DIRECT_IO_EN) begin
            mux_swclk_tck_o = GPIO_DO[0];
            mux_swdio_tms_t = GPIO_DO[1];
            mux_swdio_tms_o = GPIO_DO[2];
            mux_tdi_o = GPIO_DO[3];
            EXT_SRST_O = GPIO_DO[4];
            EXT_TRST_O = GPIO_DO[5];
        end
        else begin
            mux_swclk_tck_o = LOC_SWCLK_TCK_O;
            mux_swdio_tms_t = LOC_SWDIO_TMS_T;
            mux_swdio_tms_o = LOC_SWDIO_TMS_O;
            mux_tdi_o = (!ALONE_UART_CON && SWD_MODE) ? LOC_UART_TX : LOC_TDI_O;
            EXT_SRST_O = LOC_TRST_O;
            EXT_TRST_O = LOC_SRST_O;
        end

        EXT_UART_TX = ALONE_UART_CON ? LOC_UART_TX : 1'd1;
        LOC_SWO_I = SWD_MODE ? LOC_SWO_TDO_I : 1'd0;
    end

    assign DAP_GPIO[3] = GPIO_DO[7];


    assign LOC_TRST_I = EXT_TRST_I;
    assign LOC_SRST_I = EXT_SRST_I;
    assign LOC_UART_RX = EXT_UART_RX;


    IODELAY #(.DYN_DLY_EN("TRUE")) SWCLK_TCK_O_delay_inst (
                .DO(EXT_SWCLK_TCK_O),
                .DI(mux_swclk_tck_o),
                .SDTAP(1'd0),
                .VALUE(1'd0),
                .DLYSTEP(SWCLK_TCK_O_DELAY)
            );

    IODELAY #(.DYN_DLY_EN("TRUE")) SWDIO_TMS_T_delay_inst (
                .DO(EXT_SWDIO_TMS_T),
                .DI(mux_swdio_tms_t),
                .SDTAP(1'd0),
                .VALUE(1'd0),
                .DLYSTEP(SWDIO_TMS_T_DELAY)
            );

    IODELAY #(.DYN_DLY_EN("TRUE")) SWDIO_TMS_O_delay_inst (
                .DO(EXT_SWDIO_TMS_O),
                .DI(mux_swdio_tms_o),
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
                .DI(mux_tdi_o),
                .SDTAP(1'd0),
                .VALUE(1'd0),
                .DLYSTEP(TDI_O_DELAY)
            );



    always @(posedge clk or negedge resetn) begin : ahb_mem_write_ctrl
        if (!resetn) begin
            GPIO_REG_CR <= 8'd0;
            SWCLK_TCK_O_DELAY <= 8'd0;
            SWDIO_TMS_T_DELAY <= 8'd0;
            SWDIO_TMS_O_DELAY <= 8'd0;
            SWDIO_TMS_I_DELAY <= 8'd0;
            TDO_I_DELAY <= 8'd0;
            TDI_O_DELAY <= 8'd0;
            GPIO_DO <= 8'd0;
            RGB_PWM_CMP[0] <= 8'd0;
            RGB_PWM_CMP[1] <= 8'd0;
            RGB_PWM_CMP[2] <= 8'd0;
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
                    GPIO_LED_R_PWM_ADDR[ADDRWIDTH-1:2]: begin
                        if (ahb_byte_strobe[0])
                            RGB_PWM_CMP[0] <= ahb_wdata[ 0+:8];
                        if (ahb_byte_strobe[1])
                            RGB_PWM_CMP[1] <= ahb_wdata[ 8+:8];
                        if (ahb_byte_strobe[2])
                            RGB_PWM_CMP[2] <= ahb_wdata[16+:8];
                    end
                endcase
            end
        end
    end

    // LED PWM Controller
    reg [7:0] pwm_cnt;
    reg [2:0] led_output;
    reg [7:0] busy_delay;
    always @(posedge clk or negedge resetn) begin
        if (!resetn) begin
            pwm_cnt <= 8'd0;
            led_output[2:0] <= 3'b000;
            busy_delay <= 8'd0;
        end
        else begin
            // 8位PWM计数器循环递增
            pwm_cnt <= pwm_cnt + 8'd1;

            // RGB PWM输出：计数器小于比较值时输出高电平
            // DAP_GPIO[0] = R, DAP_GPIO[1] = G, DAP_GPIO[2] = B
            led_output[0] <= (pwm_cnt < RGB_PWM_CMP[0]) ? 1'b1 : 1'b0;
            led_output[1] <= (pwm_cnt < RGB_PWM_CMP[1]) ? 1'b1 : 1'b0;
            led_output[2] <= (pwm_cnt < RGB_PWM_CMP[2]) ? 1'b1 : 1'b0;

            // LED模式切换，DAP忙时绿灯熄灭，空闲状态按正常亮度显示
            if (LED_MODE && DAP_BUSY) begin
                busy_delay <= 8'hff;
            end

            if (busy_delay) begin
                if (pwm_cnt == 8'h00) begin
                    busy_delay <= busy_delay - 1'd1;
                end
                led_output[0] <= 1'd0;
                led_output[1] <= 1'd0;
            end
        end
    end

    // 低电平LED点亮，此处取反
    assign DAP_GPIO[2:0] = ~led_output;

    reg [9:0] GPIO_SYMPLE;
    always @(posedge clk or negedge resetn) begin
        if (!resetn) begin
            GPIO_SYMPLE <= 10'd0;
        end
        else begin
            GPIO_SYMPLE <= {
                            mux_swclk_tck_o,
                            mux_swdio_tms_t,
                            mux_swdio_tms_o,
                            LOC_SWDIO_TMS_I,
                            LOC_SWO_TDO_I,
                            mux_tdi_o,
                            EXT_TRST_I,
                            EXT_TRST_O,
                            EXT_SRST_I,
                            EXT_SRST_O
                        };
        end
    end

    always @(*) begin : ahb_mem_read_ctrl
        if (ahb_read_en) begin
            case (ahb_addr[ADDRWIDTH-1:2]) /*synthesis parallel_case*/
                GPIO_CR_ADDR[ADDRWIDTH-1:2]:
                    ahb_rdata = {SWDIO_TMS_O_DELAY, SWDIO_TMS_T_DELAY, SWCLK_TCK_O_DELAY, GPIO_REG_CR};
                SWDIO_TMS_I_DELAY_ADDR[ADDRWIDTH-1:2]:
                    ahb_rdata = {8'd0, TDI_O_DELAY, TDO_I_DELAY, SWDIO_TMS_I_DELAY};
                GPIO_STATUS_ADDR[ADDRWIDTH-1:2]:
                    ahb_rdata = {22'd0, GPIO_SYMPLE};
                GPIO_DO_ADDR[ADDRWIDTH-1:2]:
                    ahb_rdata = {24'd0, GPIO_DO};
                GPIO_LED_R_PWM_ADDR[ADDRWIDTH-1:2]:
                    ahb_rdata = {8'd0, RGB_PWM_CMP[2], RGB_PWM_CMP[1], RGB_PWM_CMP[0]};
                default:
                    ahb_rdata = {32{1'bx}};
            endcase
        end
        else begin
            ahb_rdata = {32{1'bx}};
        end
    end

endmodule
