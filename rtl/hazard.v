// Hazard enabled IF
// ID/EX.WriteReg == IF/ID.RegisterRs1 or IF/ID.RegisterRs2
// OR
// Ex/MEM.WriteReg==IF/ID.RegisterRs1 or iF/ID.RegisterRs2

// TODO: UPDATE FOR FORWARDING/BYPASSING LATER
module hazard (
    input wire [4:0] IF_ID_RS1,
    input wire [4:0] IF_ID_RS2,

    input wire [4:0] ID_EX_WriteReg,
    input wire ID_EX_RegWrite,


    input wire [4:0] EX_MEM_WriteReg,
    input wire EX_MEM_RegWrite,

    output wire PC_En,
    output wire IF_ID_En,
    output wire Mux_sel
);

  wire hazard;
  assign hazard=((ID_EX_RegWrite && ((ID_EX_WriteReg == IF_ID_RS1) || (ID_EX_WriteReg == IF_ID_RS2))) ||


        (EX_MEM_RegWrite && ((EX_MEM_WriteReg == IF_ID_RS1) || (EX_MEM_WriteReg == IF_ID_RS2)))


    );

  assign PC_En = (~hazard);
  assign IF_ID_En = (~hazard);
  assign Mux_sel = (hazard);
  
endmodule
