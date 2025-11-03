`default_nettype none
module hart #(
    // After reset, the program counter (PC) should be initialized to this
    // address and start executing instructions from there.
    parameter RESET_ADDR = 32'h00000000
) (
    // Global clock.
    input wire i_clk,

    // Synchronous active-high reset.
    input wire i_rst,

    // Instruction fetch goes through a read only instruction memory (imem)
    // port. The port accepts a 32-bit address (e.g. from the program counter)
    // per cycle and combinationally returns a 32-bit instruction word. This
    // is not representative of a realistic memory interface; it has been
    // modeled as more similar to a DFF or SRAM to simplify phase 3. In
    // later phases, you will replace this with a more realistic memory.
    //
    // 32-bit read address for the instruction memory. This is expected to be
    // 4 byte aligned - that is, the two LSBs should be zero.
    output wire [31:0] o_imem_raddr,

    // Instruction word fetched from memory, available on the same cycle.
    input wire [31:0] i_imem_rdata,

    // Data memory accesses go through a separate read/write data memory (dmem)
    // that is shared between read (load) and write (stored). The port accepts
    // a 32-bit address, read or write enable, and mask (explained below) each
    // cycle. Reads are combinational - values are available immediately after
    // updating the address and asserting read enable. Writes occur on (and
    // are visible at) the next clock edge.
    //
    // Read/write address for the data memory. This should be 32-bit aligned
    // (i.e. the two LSB should be zero). See `o_dmem_mask` for how to perform
    // half-word and byte accesses at unaligned addresses.
    output wire [31:0] o_dmem_addr,
    // When asserted, the memory will perform a read at the aligned address
    // specified by `i_addr` and return the 32-bit word at that address
    // immediately (i.e. combinationally). It is illegal to assert this and
    // `o_dmem_wen` on the same cycle.
    output wire        o_dmem_ren,
    // When asserted, the memory will perform a write to the aligned address
    // `o_dmem_addr`. When asserted, the memory will write the bytes in
    // `o_dmem_wdata` (specified by the mask) to memory at the specified
    // address on the next rising clock edge. It is illegal to assert this and
    // `o_dmem_ren` on the same cycle.
    output wire        o_dmem_wen,
    // The 32-bit word to write to memory when `o_dmem_wen` is asserted. When
    // write enable is asserted, the byte lanes specified by the mask will be
    // written to the memory word at the aligned address at the next rising
    // clock edge. The other byte lanes of the word will be unaffected.
    output wire [31:0] o_dmem_wdata,
    // The dmem interface expects word (32 bit) aligned addresses. However,
    // WISC-25 supports byte and half-word loads and stores at unaligned and
    // 16-bit aligned addresses, respectively. To support this, the access
    // mask specifies which bytes within the 32-bit word are actually read
    // from or written to memory.
    //
    // To perform a half-word read at address 0x00001002, align `o_dmem_addr`
    // to 0x00001000, assert `o_dmem_ren`, and set the mask to 0b1100 to
    // indicate that only the upper two bytes should be read. Only the upper
    // two bytes of `i_dmem_rdata` can be assumed to have valid data; to
    // calculate the final value of the `lh[u]` instruction, shift the rdata
    // word right by 16 bits and sign/zero extend as appropriate.
    //
    // To perform a byte write at address 0x00002003, align `o_dmem_addr` to
    // `0x00002003`, assert `o_dmem_wen`, and set the mask to 0b1000 to
    // indicate that only the upper byte should be written. On the next clock
    // cycle, the upper byte of `o_dmem_wdata` will be written to memory, with
    // the other three bytes of the aligned word unaffected. Remember to shift
    // the value of the `sb` instruction left by 24 bits to place it in the
    // appropriate byte lane.
    output wire [ 3:0] o_dmem_mask,
    // The 32-bit word read from data memory. When `o_dmem_ren` is asserted,
    // this will immediately reflect the contents of memory at the specified
    // address, for the bytes enabled by the mask. When read enable is not
    // asserted, or for bytes not set in the mask, the value is undefined.
    input  wire [31:0] i_dmem_rdata,
    // The output `retire` interface is used to signal to the testbench that
    // the CPU has completed and retired an instruction. A single cycle
    // implementation will assert this every cycle; however, a pipelined
    // implementation that needs to stall (due to internal hazards or waiting
    // on memory accesses) will not assert the signal on cycles where the
    // instruction in the writeback stage is not retiring.
    //
    // Asserted when an instruction is being retired this cycle. If this is
    // not asserted, the other retire signals are ignored and may be left invalid.
    output wire        o_retire_valid,
    // The 32 bit instruction word of the instrution being retired. This
    // should be the unmodified instruction word fetched from instruction
    // memory.
    output wire [31:0] o_retire_inst,
    // Asserted if the instruction produced a trap, due to an illegal
    // instruction, unaligned data memory access, or unaligned instruction
    // address on a taken branch or jump.
    output wire        o_retire_trap,
    // Asserted if the instruction is an `ebreak` instruction used to halt the
    // processor. This is used for debugging and testing purposes to end
    // a program.
    output wire        o_retire_halt,
    // The first register address read by the instruction being retired. If
    // the instruction does not read from a register (like `lui`), this
    // should be 5'd0.
    output wire [ 4:0] o_retire_rs1_raddr,
    // The second register address read by the instruction being retired. If
    // the instruction does not read from a second register (like `addi`), this
    // should be 5'd0.
    output wire [ 4:0] o_retire_rs2_raddr,
    // The first source register data read from the register file (in the
    // decode stage) for the instruction being retired. If rs1 is 5'd0, this
    // should also be 32'd0.
    output wire [31:0] o_retire_rs1_rdata,
    // The second source register data read from the register file (in the
    // decode stage) for the instruction being retired. If rs2 is 5'd0, this
    // should also be 32'd0.
    output wire [31:0] o_retire_rs2_rdata,
    // The destination register address written by the instruction being
    // retired. If the instruction does not write to a register (like `sw`),
    // this should be 5'd0.
    output wire [ 4:0] o_retire_rd_waddr,
    // The destination register data written to the register file in the
    // writeback stage by this instruction. If rd is 5'd0, this field is
    // ignored and can be treated as a don't care.
    output wire [31:0] o_retire_rd_wdata,
    // The current program counter of the instruction being retired - i.e.
    // the instruction memory address that the instruction was fetched from.
    output wire [31:0] o_retire_pc,
    // the next program counter after the instruction is retired. For most
    // instructions, this is `o_retire_pc + 4`, but must be the branch or jump
    // target for *taken* branches and jumps.
    output wire [31:0] o_retire_next_pc

`ifdef RISCV_FORMAL
    `RVFI_OUTPUTS,
`endif
);

  /*
Delcleration of any extra wires needed for connecting modules and for signals used across modules 
*/

  wire [31:0] current_PC;  //will hold current PC value 
  wire [31:0] next_PC;  //holds the adress to be updated in PC next 
  wire [31:0] PC_plus4;  //will hold PC+4 value
  wire [31:0] curr_instruct;  //holds current instruction

  wire        jal_C;  //control signal 
  wire        jalr_C;  //control signal 
  wire        branch_C;  //control signal 
  wire        MemRead_C;  //control signal 
  wire        MemWrite_C;  //control signal 
  wire [ 1:0] Data_sel_C;  //control signal 
  wire [ 2:0] ALUop_C;  //control signal 
  wire [31:0] ALU_operand1;  //alu operand 1
  wire [31:0] ALU_operand2;  //alu operand 2
  wire [31:0] immediate_val;  //generated immeidate 
  wire [31:0] Mem_WD;  //feeds into memory write data port 
  wire [ 2:0] func3_val;  //feeds into branch logic block
  wire [ 3:0] func_val;  //feeds into ALU control block

  wire [31:0] ALU_result;  //ALU operation result
  wire [ 1:0] PC_MUX_SEL;  //MUX select signal for choosing next PC
  wire [31:0] PC_offset;  //PC + immediate offset value
  wire [31:0] MEM_DATA;  //Data returned from memory read  
  wire [31:0] WB_DATA;  //data to be input into register file   
  /* 
 Instantiate IF section of proccesor 
*/

  IF fetch_inst (
      .i_clk   (i_clk),   //input- clk to control PC update
      .i_rst   (i_rst),   //input- used to reset PC to starting value
      .i_NextPC(next_PC), //input- next PC value                           

      .o_PC(current_PC),            //output- current PC feed into instruction memory and other locations (schematic) 
      .o_inc_pc(PC_plus4)  //output- current PC + 4 
  );

  assign o_imem_raddr = current_PC;  //assign instruction memory read adress to current PC
  assign curr_instruct = i_imem_rdata;  //assign current instruction to the input from instruction memory

  /* 
 Instantiate ID section of proccesor 
*/

  //////////Internal Wires////////////
  //to connect top-level register file to decode block
  wire [31:0] regData1;
  wire [31:0] regData2;
  wire regWrite;

  ID decode_I (
      .rst        (i_rst),          //input- to RF
      .clk        (i_clk),          //input- to RF
      .i_instruct (curr_instruct),  //input- full instruction input
      .i_currentPC(current_PC),     //input- the current PC value (used for auipc instruction)
      .o_RegWrite (regWrite),
      .i_regData1 (regData1),
      .i_regData2 (regData2),

      .o_jal     (jal_C),          //output- from control unit
      .o_jalr    (jalr_C),         //output- from control unit
      .o_branch  (branch_C),       //output- from control unit
      .o_MemRead (MemRead_C),      //output- from control unit
      .o_Data_sel(Data_sel_C),     //output- from control unit used for WB module
      .o_MemWrite(MemWrite_C),     //output- from control unit
      .o_op1     (ALU_operand1),   //output- from control unit (select ALU operand)
      .o_op2     (ALU_operand2),   //output- from control unit (selects ALU operand)
      .o_ALUop   (ALUop_C),        //output- from control unit goes to ALU control
      .o_imm     (immediate_val),  //output- the generated immediate 
      .o_Rdata2  (Mem_WD),         //output- will feed into memory 'write data' port 
      .func      (func_val),       //output- combination of func7 and func3 for ALU control block
      .func3     (func3_val)       //output- func 3 for branch logic 
  );

  //wirte adress should be 0 when instruction does not write to register 
  wire [4:0] rf_writeAddress;
  assign rf_writeAddress = (regWrite) ? curr_instruct[11:7] : 5'b00000;

 wire [31:0] WriteDataReg; 
  rf rf (
      .i_clk(i_clk),
      .i_rst(i_rst),

      .i_rs1_raddr(curr_instruct[19:15]),
      .i_rs2_raddr(curr_instruct[24:20]),

      .o_rs1_rdata(regData1),
      .o_rs2_rdata(regData2),

      .i_rd_waddr(rf_writeAddress),
      .i_rd_wen  (regWrite),
      .i_rd_wdata(WriteDataReg)
  );


  /* 
 Instantiate EX section of proccesor 
*/
  EX execute_I (
      .i_pc(current_PC),            //input- Current PC input should be added to immediate (used for branch instructions)
      .func3(func3_val),  //input- func 3 input for branch logic block (branch.v file)
      .i_jal(jal_C),  //input- control signal for branch logic block
      .i_jalr(jalr_C),  //input- control signal for branch logic block
      .i_branch(branch_C),  //input- control signal for branch logic block
      .i_ALUOp(ALUop_C),  //input- input to ALU CTRL unit
      .i_op1(ALU_operand1),  //input- ALU operand1
      .i_op2(ALU_operand2),  //input- ALU operand2
      .i_imm(immediate_val),        //input- i_imm is used for adding to current PC if we are in I instruction i_op2 will be input as immediate already
      .func(func_val),  //input- combination of func7 and func 3 used by ALU control

      .o_result(ALU_result),  //output- ALU result
      .o_PC_Select(PC_MUX_SEL),     //output- MUX select signals from branch logic unit used to select next PC
      .o_inc_pc(PC_offset)          //output- The current PC + Immediate (used for branch adress calculation)
  );

  
  wire [31:0] aligned_address;
  wire byte_hw_unsigned;
  wire [3:0] mask;


  mask_gen mask_gen (
      .address(ALU_result),
      .func3(func3_val),
      .aligned_address(aligned_address),
      .o_unsigned(byte_hw_unsigned),
      .mask(mask),
      .opcode(curr_instruct[6:0])
  );
  /* 
 Instantiate MEM section of proccesor  (actual memory access done outside MEM module)
*/

  MEM memory_acces (
      .i_aluResult(ALU_result),       //input- ALU result can be used for either memory adress or multiplexed to next PC
      .i_PC4(PC_plus4),  //input- previous PC+4 used as input for multiplexer
      .i_PCimm(PC_offset),  //input- previous PC+imm used as input for multiplexer 
      .i_MUXpc(PC_MUX_SEL),  //input- select signal used to select next PC

      .o_nxtPC(next_PC)  //output- the next PC based off Mux select signals 
  );

 wire [31:0] WriteDataMem; 
  assign o_dmem_addr = aligned_address;  //assign memory adress port to ALU result  
  assign o_dmem_ren  = MemRead_C;   //assign Memory Read enable signal 
  assign o_dmem_wen  = MemWrite_C;  //assign Memory Write enable signal 
  assign o_dmem_wdata = WriteDataMem;     //assign Memory Write data port to register output #2
  assign MEM_DATA = i_dmem_rdata;    //data returned from memory 


S_extend dataEXT(
  .i_mask(mask),         
  .i_unsign(byte_hw_unsigned),

  .i_Rs2Data(Mem_WD),   //register data input 
  .o_Memdata(WriteDataMem),           //aligned output based on mask 

  .i_WB(WB_DATA),
  .o_regData(WriteDataReg)
);


  /* 
 Instantiate WB section of proccesor 
*/
  WB writeback (

      .i_MemData(MEM_DATA),  //input-  data coming from Memory
      .i_AluRslt(ALU_result),  //input- ALU operation result 
      .i_imm(immediate_val),  //input- output from immediate generator 
      .i_PC4(PC_plus4),  //input- incremented PC (used to save jump return adress in register)
      .i_MUXsel(Data_sel_C),  //input- MUX select signals coming from control unit 

      .o_dataSel(WB_DATA)        //output- data selected from mux to feedback into write port of Register file
  );




  /*
THIS section still needs to be checked and filled in properly (some values are being used as placeholders for now)
We can add extra output signals from modules to connect below 
*/
  assign o_dmem_mask = mask; //this used to control half word/byte loads and write (set to full word only for now)

  assign o_retire_valid = (i_rst) ? 1'b0 : 1'b1;  //one instruction should be done every cycle
  assign o_retire_inst = curr_instruct;
  assign o_retire_trap = 1'b0;  // implement trap detection later
  assign o_retire_halt = (curr_instruct == 32'h00100073);  // ebreak

  // retire register addresses (taken directly from the instruction fields)
  assign o_retire_rs1_raddr = curr_instruct[19:15];
  assign o_retire_rs2_raddr = curr_instruct[24:20];
  assign o_retire_rd_waddr = rf_writeAddress;

  // retire register read data 
  // Ideally use the raw register-file read data
  assign o_retire_rs1_rdata = regData1;
  assign o_retire_rs2_rdata = regData2;

  // retire write-back info (what is written back this cycle)
  assign o_retire_rd_wdata = WriteDataReg;

  // retire PC values
  assign o_retire_pc = current_PC;
  assign o_retire_next_pc = next_PC;

endmodule
`default_nettype wire
