module DAP_GPIO #(
        parameter ADDRWIDTH = 12,
        parameter [ADDRWIDTH-1:0] BASE_ADDR = 0,
        parameter GPIO_NUM = 8 // 不超过8
    )(
        input clk,
        input resetn,

        // AHB MEM接口
        input ahb_write_en,
        input [ADDRWIDTH-1:0] ahb_addr,
        output reg [31:0] ahb_rdata,
        input [31:0] ahb_wdata,
        input [3:0] ahb_byte_strobe,

        // GPIO输出接口
        inout [GPIO_NUM-1:0] gpio,

        output [GPIO_NUM-1:0] afio0_I,
        input [GPIO_NUM-1:0] afio0_O,
        input [GPIO_NUM-1:0] afio0_T,

        output [GPIO_NUM-1:0] afio1_I,
        input [GPIO_NUM-1:0] afio1_O,
        input [GPIO_NUM-1:0] afio1_T
    );

    localparam [ADDRWIDTH-1:0] GPIO_CR_ADDR = BASE_ADDR + 0;  // RW
    localparam [ADDRWIDTH-1:0] GPIO_DIR_ADDR = BASE_ADDR + 4; // RW
    localparam [ADDRWIDTH-1:0] GPIO_DO_ADDR = BASE_ADDR + 8;  // RW
    localparam [ADDRWIDTH-1:0] GPIO_DI_ADDR = BASE_ADDR + 12; // RO
    localparam [ADDRWIDTH-1:0] GPIO_BS_ADDR = BASE_ADDR + 16; // WO
    localparam [ADDRWIDTH-1:0] GPIO_BR_ADDR = BASE_ADDR + 20; // WO
    localparam [ADDRWIDTH-1:0] GPIO_IDELAY_0_3_ADDR = BASE_ADDR + 24; // WR
    localparam [ADDRWIDTH-1:0] GPIO_IDELAY_4_7_ADDR = BASE_ADDR + 28; // WR
    localparam [ADDRWIDTH-1:0] GPIO_ODELAY_0_3_ADDR = BASE_ADDR + 32; // WR
    localparam [ADDRWIDTH-1:0] GPIO_ODELAY_4_7_ADDR = BASE_ADDR + 36; // WR

    wire [GPIO_NUM-1:0] gpio_I;
    wire [GPIO_NUM-1:0] gpio_O;
    wire [GPIO_NUM-1:0] gpio_T;

    reg [31:0] GPIO_REG_CR;
    reg [31:0] GPIO_REG_DIR;
    reg [31:0] GPIO_REG_DO;
    wire [31:0] GPIO_REG_DI = {{32-GPIO_NUM{1'b0}}, gpio_I};

    reg [7:0] GPIO_IDELAY[0:7];
    reg [7:0] GPIO_ODELAY[0:7];

    wire afio_mux = GPIO_REG_CR[31];

    reg [GPIO_NUM-1:0] afio_O;
    reg [GPIO_NUM-1:0] afio_T;
    wire [GPIO_NUM-1:0] afio_I;

    always @(*) begin
        case(afio_mux)
            1'd0: begin
                afio_O = afio0_O;
                afio_T = afio0_T;
            end
            1'd1: begin
                afio_O = afio1_O;
                afio_T = afio1_T;
            end
        endcase
    end
    assign afio0_I = gpio_I;
    assign afio1_I = gpio_I;

    wire [GPIO_NUM-1:0] buf_i;
    wire [GPIO_NUM-1:0] buf_o;
    wire [GPIO_NUM-1:0] odelay_o;
    wire [GPIO_NUM-1:0] idelay_o;

    genvar gi;
    generate
        for (gi = 0; gi < GPIO_NUM; gi = gi + 1) begin : io_delay_mux
            IOBUF iobuf_inst(
                      .O(buf_o[gi]),
                      .IO(gpio[gi]),
                      .I(buf_i[gi]),
                      .OEN(gpio_T[gi])
                  );

            IODELAY #(.DYN_DLY_EN("TRUE")) idelay_inst(
                        .DO(idelay_o[gi]),
                        .DI(buf_o[gi]),
                        .SDTAP(1'd0),
                        .VALUE(1'd0),
                        .DLYSTEP(GPIO_IDELAY[gi])
                    );

            IODELAY #(.DYN_DLY_EN("TRUE")) odelay_inst(
                        .DO(odelay_o[gi]),
                        .DI(gpio_O[gi]),
                        .SDTAP(1'd0),
                        .VALUE(1'd0),
                        .DLYSTEP(GPIO_ODELAY[gi])
                    );
            assign gpio_I[gi] = (GPIO_IDELAY[gi] == 8'd0) ? buf_o[gi] : idelay_o[gi];
            assign gpio_O[gi] = GPIO_REG_CR[gi] ? GPIO_REG_DO[gi] : afio_O[gi];
            assign gpio_T[gi] = GPIO_REG_CR[gi] ? GPIO_REG_DIR[gi] : afio_T[gi];
            assign buf_i[gi] = (GPIO_ODELAY[gi] == 8'd0) ? gpio_O[gi] : odelay_o[gi];
        end
    endgenerate


    always @(posedge clk or negedge resetn) begin : ahb_mem_write_ctrl
        if (!resetn) begin
            GPIO_REG_CR <= 0;
            GPIO_REG_DIR <= 0;
            GPIO_REG_DO <= 0;
        end
        else begin
            if (ahb_write_en) begin
                case (ahb_addr[ADDRWIDTH-1:2])
                    GPIO_CR_ADDR[ADDRWIDTH-1:2]: begin
                        if (ahb_byte_strobe[0])
                            GPIO_REG_CR[ 0+:8] <= ahb_wdata[ 0+:8];
                        if (ahb_byte_strobe[1])
                            GPIO_REG_CR[ 8+:8] <= ahb_wdata[ 8+:8];
                        if (ahb_byte_strobe[2])
                            GPIO_REG_CR[16+:8] <= ahb_wdata[16+:8];
                        if (ahb_byte_strobe[3])
                            GPIO_REG_CR[24+:8] <= ahb_wdata[24+:8];
                    end
                    GPIO_DIR_ADDR[ADDRWIDTH-1:2]: begin
                        if (ahb_byte_strobe[0])
                            GPIO_REG_DIR[ 0+:8] <= ahb_wdata[ 0+:8];
                        if (ahb_byte_strobe[1])
                            GPIO_REG_DIR[ 8+:8] <= ahb_wdata[ 8+:8];
                        if (ahb_byte_strobe[2])
                            GPIO_REG_DIR[16+:8] <= ahb_wdata[16+:8];
                        if (ahb_byte_strobe[3])
                            GPIO_REG_DIR[24+:8] <= ahb_wdata[24+:8];
                    end
                    GPIO_DO_ADDR[ADDRWIDTH-1:2]: begin
                        if (ahb_byte_strobe[0])
                            GPIO_REG_DO[ 0+:8] <= ahb_wdata[ 0+:8];
                        if (ahb_byte_strobe[1])
                            GPIO_REG_DO[ 8+:8] <= ahb_wdata[ 8+:8];
                        if (ahb_byte_strobe[2])
                            GPIO_REG_DO[16+:8] <= ahb_wdata[16+:8];
                        if (ahb_byte_strobe[3])
                            GPIO_REG_DO[24+:8] <= ahb_wdata[24+:8];
                    end
                    GPIO_BS_ADDR[ADDRWIDTH-1:2]: begin
                        if (ahb_byte_strobe[0])
                            GPIO_REG_DO[ 0+:8] <= GPIO_REG_DO[ 0+:8] | ahb_wdata[ 0+:8];
                        if (ahb_byte_strobe[1])
                            GPIO_REG_DO[ 8+:8] <= GPIO_REG_DO[ 8+:8] | ahb_wdata[ 8+:8];
                        if (ahb_byte_strobe[2])
                            GPIO_REG_DO[16+:8] <= GPIO_REG_DO[16+:8] | ahb_wdata[16+:8];
                        if (ahb_byte_strobe[3])
                            GPIO_REG_DO[24+:8] <= GPIO_REG_DO[24+:8] | ahb_wdata[24+:8];
                    end
                    GPIO_BR_ADDR[ADDRWIDTH-1:2]: begin
                        if (ahb_byte_strobe[0])
                            GPIO_REG_DO[ 0+:8] <= GPIO_REG_DO[ 0+:8] & ~ahb_wdata[ 0+:8];
                        if (ahb_byte_strobe[1])
                            GPIO_REG_DO[ 8+:8] <= GPIO_REG_DO[ 8+:8] & ~ahb_wdata[ 8+:8];
                        if (ahb_byte_strobe[2])
                            GPIO_REG_DO[16+:8] <= GPIO_REG_DO[16+:8] & ~ahb_wdata[16+:8];
                        if (ahb_byte_strobe[3])
                            GPIO_REG_DO[24+:8] <= GPIO_REG_DO[24+:8] & ~ahb_wdata[24+:8];
                    end
                    GPIO_IDELAY_0_3_ADDR[ADDRWIDTH-1:2]: begin
                        if (ahb_byte_strobe[0])
                            GPIO_IDELAY[0] <= ahb_wdata[ 0+:8];
                        if (ahb_byte_strobe[1])
                            GPIO_IDELAY[1] <= ahb_wdata[ 8+:8];
                        if (ahb_byte_strobe[2])
                            GPIO_IDELAY[2] <= ahb_wdata[16+:8];
                        if (ahb_byte_strobe[3])
                            GPIO_IDELAY[3] <= ahb_wdata[24+:8];
                    end
                    GPIO_IDELAY_4_7_ADDR[ADDRWIDTH-1:2]: begin
                        if (ahb_byte_strobe[0])
                            GPIO_IDELAY[4] <= ahb_wdata[ 0+:8];
                        if (ahb_byte_strobe[1])
                            GPIO_IDELAY[5] <= ahb_wdata[ 8+:8];
                        if (ahb_byte_strobe[2])
                            GPIO_IDELAY[6] <= ahb_wdata[16+:8];
                        if (ahb_byte_strobe[3])
                            GPIO_IDELAY[7] <= ahb_wdata[24+:8];
                    end
                    GPIO_ODELAY_0_3_ADDR[ADDRWIDTH-1:2]: begin
                        if (ahb_byte_strobe[0])
                            GPIO_ODELAY[0] <= ahb_wdata[ 0+:8];
                        if (ahb_byte_strobe[1])
                            GPIO_ODELAY[1] <= ahb_wdata[ 8+:8];
                        if (ahb_byte_strobe[2])
                            GPIO_ODELAY[2] <= ahb_wdata[16+:8];
                        if (ahb_byte_strobe[3])
                            GPIO_ODELAY[3] <= ahb_wdata[24+:8];
                    end
                    GPIO_ODELAY_4_7_ADDR[ADDRWIDTH-1:2]: begin
                        if (ahb_byte_strobe[0])
                            GPIO_ODELAY[4] <= ahb_wdata[ 0+:8];
                        if (ahb_byte_strobe[1])
                            GPIO_ODELAY[5] <= ahb_wdata[ 8+:8];
                        if (ahb_byte_strobe[2])
                            GPIO_ODELAY[6] <= ahb_wdata[16+:8];
                        if (ahb_byte_strobe[3])
                            GPIO_ODELAY[7] <= ahb_wdata[24+:8];

                    end
                endcase
            end
        end
    end

    always @(*) begin : ahb_mem_read_ctrl
        case (ahb_addr[ADDRWIDTH-1:2])
            GPIO_CR_ADDR[ADDRWIDTH-1:2]:
                ahb_rdata = GPIO_REG_CR;
            GPIO_DIR_ADDR[ADDRWIDTH-1:2]:
                ahb_rdata = GPIO_REG_DIR;
            GPIO_DO_ADDR[ADDRWIDTH-1:2]:
                ahb_rdata = GPIO_REG_DO;
            GPIO_DI_ADDR[ADDRWIDTH-1:2]:
                ahb_rdata = GPIO_REG_DI;
            GPIO_IDELAY_0_3_ADDR[ADDRWIDTH-1:2]:
                ahb_rdata = {GPIO_IDELAY[3], GPIO_IDELAY[2], GPIO_IDELAY[1], GPIO_IDELAY[0]};
            GPIO_IDELAY_4_7_ADDR[ADDRWIDTH-1:2]:
                ahb_rdata = {GPIO_IDELAY[7], GPIO_IDELAY[6], GPIO_IDELAY[5], GPIO_IDELAY[4]};
            GPIO_ODELAY_0_3_ADDR[ADDRWIDTH-1:2]:
                ahb_rdata = {GPIO_ODELAY[3], GPIO_ODELAY[2], GPIO_ODELAY[1], GPIO_ODELAY[0]};
            GPIO_ODELAY_4_7_ADDR[ADDRWIDTH-1:2]:
                ahb_rdata = {GPIO_ODELAY[7], GPIO_ODELAY[6], GPIO_ODELAY[5], GPIO_ODELAY[4]};
            default:
                ahb_rdata = {32{1'bx}};
        endcase
    end

endmodule
