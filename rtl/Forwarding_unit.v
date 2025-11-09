`default_nettype none

/*
This unit should overide any hazard detection (can do this by AND with hazard signal to create new stalling signal)
With forwarding stalls should only occur on load-use hazard
*/
module Forwarding_unit (

    //When the instruction reaches EX, it checks if its rs1 or rs2 register matches the rd register of any instruction ahead 
    //in the pipeline (EX/MEM or MEM/WB) If yes — we take that newer value instead of the stale one from the register file.

    input wire [4:0] IDEX_RS1, //rs1 read reg in EX
    input wire [4:0] IDEX_RS2, //rs2 read reg in eX

    input wire [4:0] EXMEM_RD, //future RD registers 
    input wire [4:0] MEMWB_RD,

    input wire EXMEM_regWrite, //Write enable signals 
    input wire MEMWB_regWrite,

    input wire EXMEM_MemRead,  //MemRead signals 


    input wire [31:0] EXMEM_aluResult,
    input wire [31:0] MEMWB_wbValue,

    output wire FW_mux_sel
);




endmodule
`default_nettype wire
