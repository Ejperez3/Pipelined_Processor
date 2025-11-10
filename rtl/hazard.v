// Hazard enabled IF
// ID/EX.WriteReg == IF/ID.RegisterRs1 or IF/ID.RegisterRs2
// OR
// Ex/MEM.WriteReg==IF/ID.RegisterRs1 or iF/ID.RegisterRs2

// TODO: UPDATE FOR FORWARDING/BYPASSING LATER
module hazard (
    input wire [6:0] op_code,
    input wire [4:0] IF_ID_RS1,
    input wire [4:0] IF_ID_RS2,
    input wire valid_inst,

    input wire [4:0] ID_EX_WriteReg,
    input wire       ID_EX_MemRead,

    output wire PC_En,
    output wire IF_ID_En,
    output wire Mux_sel
);

  wire is_jump_or_lui;
  assign is_jump_or_lui=(op_code==(7'b0110111)|| op_code==7'b0010111 || op_code==7'b1101111);

  wire load_use_hazard;
  assign load_use_hazard = valid_inst && (~is_jump_or_lui) && (ID_EX_MemRead) && ((ID_EX_WriteReg == IF_ID_RS1) || (ID_EX_WriteReg == IF_ID_RS2)) 
                           && (ID_EX_WriteReg != 5'd0);

  assign PC_En = (~load_use_hazard);
  assign IF_ID_En = (~load_use_hazard);
  assign Mux_sel = (load_use_hazard);

endmodule
