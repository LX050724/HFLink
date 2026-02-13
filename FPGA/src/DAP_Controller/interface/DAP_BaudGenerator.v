module DAP_BaudGenerator#(
        parameter ADDRWIDTH = 12,
        parameter [ADDRWIDTH-1:0] BASE_ADDR = 0
    )(
        input clk,
        input sclk_in,
        input resetn,

        // AHB MEM接口
        input ahb_write_en,
        input ahb_read_en,
        input [ADDRWIDTH-1:0] ahb_addr,
        output reg [31:0] ahb_rdata,
        input [31:0] ahb_wdata,
        input [3:0] ahb_byte_strobe,

        output sclk_out, // 输出到GPIO的时钟信号
        output sclk_pulse, // 控制器置位时刻参考
        output sclk_delay_pulse // 控制器采样时刻参考
    );

    localparam [ADDRWIDTH-1:0] REG_CR_ADDR = BASE_ADDR + 0;  // RW
    localparam [ADDRWIDTH-1:0] REG_TIMING_ADDR = BASE_ADDR + 4; // RW

    reg [31:0] REG_CR;

    wire REG_CR_CEN = REG_CR[0];
    wire REG_CR_SAMPLINE_EDGE = REG_CR[1]; // 0：上升沿采样；1：下降沿采样
    reg [15:0] REG_TIMING_DIV;
    reg [2:0] REG_TIMING_DELAY;


    always @(posedge clk or negedge resetn) begin : ahb_mem_write_ctrl
        if (!resetn) begin
            REG_CR <= 0;
            REG_TIMING_DIV <= 16'd0;
            REG_TIMING_DELAY <= 3'd0;
        end
        else begin
            if (ahb_write_en) begin
                case (ahb_addr[ADDRWIDTH-1:2])
                    REG_CR_ADDR[ADDRWIDTH-1:2]: begin
                        if (ahb_byte_strobe[0])
                            REG_CR[ 0+:8] <= ahb_wdata[ 0+:8];
                        if (ahb_byte_strobe[1])
                            REG_CR[ 8+:8] <= ahb_wdata[ 8+:8];
                        if (ahb_byte_strobe[2])
                            REG_CR[16+:8] <= ahb_wdata[16+:8];
                        if (ahb_byte_strobe[3])
                            REG_CR[24+:8] <= ahb_wdata[24+:8];
                    end
                    REG_TIMING_ADDR[ADDRWIDTH-1:2]: begin
                        if (!REG_CR_CEN) begin
                            if (ahb_byte_strobe[0])
                                REG_TIMING_DIV[ 0+:8] <= ahb_wdata[ 0+:8];
                            if (ahb_byte_strobe[1])
                                REG_TIMING_DIV[ 8+:8] <= ahb_wdata[ 8+:8];
                            if (ahb_byte_strobe[2])
                                REG_TIMING_DELAY <= ahb_wdata[18:16];
                        end
                    end
                endcase
            end
        end
    end

    always @(*) begin : ahb_mem_read_ctrl
        if (ahb_read_en) begin
            case (ahb_addr[ADDRWIDTH-1:2])
                REG_CR_ADDR[ADDRWIDTH-1:2]:
                    ahb_rdata = REG_CR;
                REG_TIMING_ADDR[ADDRWIDTH-1:2]:
                    ahb_rdata = {13'd0, REG_TIMING_DELAY, REG_TIMING_DIV};
                default:
                    ahb_rdata = {32{1'bx}};
            endcase
        end
        else begin
            ahb_rdata = {32{1'bx}};
        end
    end


    reg cen_ff1;
    reg cen;

    reg [15:0] div_count;
    reg sclk_out_reg;
    reg sclk_pulse_reg;

    reg [6:0] delay_reg;
    wire [7:0] delay_chain = {delay_reg, sclk_pulse_reg};
    wire [15:0] div_count_next = div_count + 1'd1;

    always @(posedge sclk_in or negedge resetn) begin
        if (!resetn) begin
            div_count <= 16'd0;
            sclk_out_reg <= 16'd0;
            delay_reg <= 7'd0;
        end
        else begin
            cen_ff1 <= REG_CR_CEN;
            cen <= cen_ff1;

            sclk_pulse_reg <= 1'd0;

            if (cen) begin
                div_count <= div_count_next;
                if (div_count == REG_TIMING_DIV) begin
                    div_count <= 16'd0;
                    sclk_out_reg <= ~sclk_out_reg;
                end
                
                if (REG_TIMING_DIV == 0) begin
                    sclk_pulse_reg <= sclk_out_reg;
                end
                else if (div_count_next == REG_TIMING_DIV) begin
                    sclk_pulse_reg <= ~sclk_out_reg;
                end

                delay_reg <= {delay_reg[5:0], sclk_pulse_reg};
            end
            else begin
                div_count <= 16'd0;
                delay_reg <= 7'd0;
                sclk_out_reg <= 1'd0;
            end

        end
    end

    assign sclk_out = sclk_out_reg;
    assign sclk_pulse = sclk_pulse_reg;
    assign sclk_delay_pulse = delay_chain[REG_TIMING_DELAY];

endmodule
