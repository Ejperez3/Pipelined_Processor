// Hazard enabled IF
// ID/EX.WriteReg == IF/ID.RegisterRs1 or IF/ID.RegisterRs2
// OR
// Ex/MEM.WriteReg==IF/ID.RegisterRs1 or iF/ID.RegisterRs2

module hazard(
  input wire ID_EX_WriteReg,
  input wire EX_MEM_WriteReg,
  input wire ID_EX_MemRead,
  output wire PC_En,
  output wire IF_ID_En,
  output wire Mux_sel,
);


endmodule
