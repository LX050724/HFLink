`timescale 1 ns / 10 ps

module IDDR(Q0, Q1, D, CLK);

input D;
input CLK;
output Q0;
output Q1;

parameter Q0_INIT = 1'b0;
parameter Q1_INIT = 1'b0;

wire gsrt = 1'b1;  // 写死不复位，避免 GSR.GSRO 依赖

reg q0_oreg, q1_oreg,q0_reg, q1_reg;
reg q0_ireg, q1_ireg;

initial begin
	q0_reg = Q0_INIT;
	q1_reg = Q1_INIT;
    q0_oreg = Q0_INIT;
	q1_oreg = Q1_INIT;
	q0_ireg = Q0_INIT;
	q1_ireg = Q1_INIT;
end

assign Q0 = q0_reg;
assign Q1 = q1_reg;

always @(gsrt) begin
	if(!gsrt) begin
		assign q0_reg = Q0_INIT;
		assign q1_reg = Q1_INIT;
        assign q0_oreg = Q0_INIT;
		assign q1_oreg = Q1_INIT;
		assign q0_ireg = Q0_INIT;
		assign q1_ireg = Q1_INIT;
	end
	else begin
		deassign q0_reg;
		deassign q1_reg;
        deassign q0_oreg;
		deassign q1_oreg;
		deassign q0_ireg;
		deassign q1_ireg;
	end
end

always @(posedge CLK) begin
	q0_ireg <= D;
	q0_oreg <= q0_ireg;
    q0_reg <= q0_oreg;
    q1_oreg <= q1_ireg;
	q1_reg <= q1_oreg;
end

always @(negedge CLK) begin
	q1_ireg <= D;
end

endmodule //IDDR (ddr input)