/**************************************************************************
 * psim.c - Pipelined Y86-64 simulator
 * 
 * Copyright (c) 2010, 2015. Bryant and D. O'Hallaron, All rights reserved.
 * May not be used, modified, or copied without permission.
 **************************************************************************/ 

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <unistd.h>
#include <string.h>

#include "isa.h"
#include "pipeline.h"
#include "stages.h"
#include "sim.h"


/*********************************************************
 * Part 2: This part contains the core simulator routines.
 *********************************************************/


/*****************
 *  Part 2 Globals
 *****************/

/* Performance monitoring */
/* How many cycles have been simulated? */
word_t cycles = 0;
/* How many instructions have passed through the WB stage? */
word_t instructions = 0;

/* Has simulator gotten past initial bubbles? */
static int starting_up = 1;



/* Both instruction and data memory */
mem_t mem;
word_t minAddr = 0;
word_t memCnt = 0;

/* Register file */
mem_t reg;
/* Condition code register */
cc_t cc;
/* Status code */
stat_t status;


/* Pending updates to state */
word_t cc_in = DEFAULT_CC;
word_t wb_destE = REG_NONE;
word_t wb_valE = 0;
word_t wb_destM = REG_NONE;
word_t wb_valM = 0;
word_t mem_addr = 0;
word_t mem_data = 0;
bool_t mem_write = FALSE;
#ifdef SNU
bool_t mem_byte = FALSE;
#endif

/* EX Operand sources */
mux_source_t amux = MUX_NONE;
mux_source_t bmux = MUX_NONE;

/* Current and next states of all pipeline registers */
pc_ptr pc_curr;
if_id_ptr if_id_curr;
id_ex_ptr id_ex_curr;
ex_mem_ptr ex_mem_curr;
mem_wb_ptr mem_wb_curr;

pc_ptr pc_next;
if_id_ptr if_id_next;
id_ex_ptr id_ex_next;
ex_mem_ptr ex_mem_next;
mem_wb_ptr mem_wb_next;

/* Intermediate values */
word_t f_pc;
byte_t imem_icode;
byte_t imem_ifun;
bool_t imem_error;
bool_t instr_valid;
word_t d_regvala;
word_t d_regvalb;
word_t e_vala;
word_t e_valb;
bool_t e_bcond;
bool_t dmem_error;

/* The pipeline state */
pipe_ptr pc_state, if_id_state, id_ex_state, ex_mem_state, mem_wb_state;

/* Simulator operating mode */
sim_mode_t sim_mode = S_FORWARD;
/* Log file */
FILE *dumpfile = NULL;


/*****************************************************************************
 * pipeline control
 * These functions can be used to handle hazards
 *****************************************************************************/

/* bubble stage (has effect at next update) */
void sim_bubble_stage(stage_id_t stage) 
{
    switch (stage)
	{
	case IF_STAGE : pc_state->op     = P_BUBBLE; break;
	case ID_STAGE : if_id_state->op  = P_BUBBLE; break;
	case EX_STAGE : id_ex_state->op  = P_BUBBLE; break;
	case MEM_STAGE: ex_mem_state->op = P_BUBBLE; break;
	case WB_STAGE : mem_wb_state->op = P_BUBBLE; break;
	}
}

/* stall stage (has effect at next update) */
void sim_stall_stage(stage_id_t stage) {
    switch (stage)
	{
	case IF_STAGE : pc_state->op     = P_STALL; break;
	case ID_STAGE : if_id_state->op  = P_STALL; break;
	case EX_STAGE : id_ex_state->op  = P_STALL; break;
	case MEM_STAGE: ex_mem_state->op = P_STALL; break;
	case WB_STAGE : mem_wb_state->op = P_STALL; break;
	}
}


static int initialized = 0;

void sim_init()
{
    /* Create memory and register files */
    initialized = 1;
    mem = init_mem(MEM_SIZE);
    reg = init_reg();
    
    /* create 5 pipe registers */
    pc_state     = new_pipe(sizeof(pc_ele), (void *) &bubble_pc);
    if_id_state  = new_pipe(sizeof(if_id_ele), (void *) &bubble_if_id);
    id_ex_state  = new_pipe(sizeof(id_ex_ele), (void *) &bubble_id_ex);
    ex_mem_state = new_pipe(sizeof(ex_mem_ele), (void *) &bubble_ex_mem);
    mem_wb_state = new_pipe(sizeof(mem_wb_ele), (void *) &bubble_mem_wb);
  
    /* connect them to the pipeline stages */
    pc_next   = pc_state->next;
    pc_curr   = pc_state->current;
  
    if_id_next = if_id_state->next;
    if_id_curr = if_id_state->current;

    id_ex_next = id_ex_state->next;
    id_ex_curr = id_ex_state->current;

    ex_mem_next = ex_mem_state->next;
    ex_mem_curr = ex_mem_state->current;

    mem_wb_next = mem_wb_state->next;
    mem_wb_curr = mem_wb_state->current;

    sim_reset();
    clear_mem(mem);
}

void sim_reset()
{
    if (!initialized)
	sim_init();
    clear_pipes();
    clear_mem(reg);
    minAddr = 0;
    memCnt = 0;
    starting_up = 1;
    cycles = instructions = 0;
    cc = DEFAULT_CC;
    status = STAT_AOK;

#ifdef HAS_GUI
    if (gui_mode) {
	signal_register_clear();
	create_memory_display();
    }
#endif

    amux = bmux = MUX_NONE;
    cc = cc_in = DEFAULT_CC;
    wb_destE = REG_NONE;
    wb_valE = 0;
    wb_destM = REG_NONE;
    wb_valM = 0;
    mem_addr = 0;
    mem_data = 0;
    mem_write = FALSE;
    sim_report();
}

/* Update state elements */
/* May need to disable updating of memory & condition codes */
static void update_state(bool_t update_mem, bool_t update_cc)
{
    /* Writeback(s):
       If either register is REG_NONE, write will have no effect .
       Order of two writes determines semantics of
       popl %rsp.  According to ISA, %rsp will get popped value
    */

    if (wb_destE != REG_NONE) {
	sim_log("\tWriteback: Wrote 0x%llx to register %s\n",
		wb_valE, reg_name(wb_destE));
	set_reg_val(reg, wb_destE, wb_valE);
    }
    if (wb_destM != REG_NONE) {
	sim_log("\tWriteback: Wrote 0x%llx to register %s\n",
		wb_valM, reg_name(wb_destM));
	set_reg_val(reg, wb_destM, wb_valM);
    }

    /* Memory write */
    if (mem_write && !update_mem) {
	sim_log("\tDisabled write of 0x%llx to address 0x%llx\n", mem_data, mem_addr);
    }
    if (update_mem && mem_write) {
	if (!set_word_val(mem, mem_addr, mem_data)) {
	    sim_log("\tCouldn't write to address 0x%llx\n", mem_addr);
	} else {
	    sim_log("\tWrote 0x%llx to address 0x%llx\n", mem_data, mem_addr);

#ifdef HAS_GUI
	    if (gui_mode) {
		if (mem_addr % 8 != 0) {
		    /* Just did a misaligned write.
		       Need to display both words */
		    word_t align_addr = mem_addr & ~0x3;
		    word_t val;
		    get_word_val(mem, align_addr, &val);
		    set_memory(align_addr, val);
		    align_addr+=8;
		    get_word_val(mem, align_addr, &val);
		    set_memory(align_addr, val);
		} else {
		    set_memory(mem_addr, mem_data);
		}
	    }
#endif
	}
    }
    if (update_cc)
	cc = cc_in;
}

/* Text representation of status */
void tty_report(word_t cyc) {
  sim_log("\nCycle %lld. CC=%s, Stat=%s\n", cyc, cc_name(cc), stat_name(status));

  sim_log("F: predPC = 0x%llx\n", pc_curr->pc);

  sim_log("D: instr = %s, rA = %s, rB = %s, valC = 0x%llx, valP = 0x%llx, Stat = %s\n",
	  iname(HPACK(if_id_curr->icode, if_id_curr->ifun)),
	  reg_name(if_id_curr->ra), reg_name(if_id_curr->rb),
	  if_id_curr->valc, if_id_curr->valp,
	  stat_name(if_id_curr->status));

  sim_log("E: instr = %s, valC = 0x%llx, valA = 0x%llx, valB = 0x%llx\n   srcA = %s, srcB = %s, dstE = %s, dstM = %s, Stat = %s\n",
	  iname(HPACK(id_ex_curr->icode, id_ex_curr->ifun)),
	  id_ex_curr->valc, id_ex_curr->vala, id_ex_curr->valb,
	  reg_name(id_ex_curr->srca), reg_name(id_ex_curr->srcb),
	  reg_name(id_ex_curr->deste), reg_name(id_ex_curr->destm),
	  stat_name(id_ex_curr->status));

  sim_log("M: instr = %s, Cnd = %d, valE = 0x%llx, valA = 0x%llx\n   dstE = %s, dstM = %s, Stat = %s\n",
	  iname(HPACK(ex_mem_curr->icode, ex_mem_curr->ifun)),
	  ex_mem_curr->takebranch,
	  ex_mem_curr->vale, ex_mem_curr->vala,
	  reg_name(ex_mem_curr->deste), reg_name(ex_mem_curr->destm),
	  stat_name(ex_mem_curr->status));

  sim_log("W: instr = %s, valE = 0x%llx, valM = 0x%llx, dstE = %s, dstM = %s, Stat = %s\n",
	  iname(HPACK(mem_wb_curr->icode, mem_wb_curr->ifun)),
	  mem_wb_curr->vale, mem_wb_curr->valm,
	  reg_name(mem_wb_curr->deste), reg_name(mem_wb_curr->destm),
	  stat_name(mem_wb_curr->status));
}

/* Run pipeline for one cycle */
/* Return status of processor */
/* Max_instr indicates maximum number of instructions that
   want to complete during this simulation run.  */
static byte_t sim_step_pipe(word_t max_instr, word_t ccount)
{
    byte_t wb_status = mem_wb_curr->status;
    byte_t mem_status = mem_wb_next->status;
    /* How many instructions are ahead of one in wb / ex? */
    int ahead_mem = (wb_status != STAT_BUB);
    int ahead_ex = ahead_mem + (mem_status != STAT_BUB);
    bool_t update_mem = ahead_mem < max_instr;
    bool_t update_cc = ahead_ex < max_instr;

    /* Update program-visible state */
    update_state(update_mem, update_cc);
    /* Update pipe registers */
    update_pipes();
    tty_report(ccount);
    if (pc_state->op == P_ERROR)
	pc_curr->status = STAT_PIP;
    if (if_id_state->op == P_ERROR)
	if_id_curr->status = STAT_PIP;
    if (id_ex_state->op == P_ERROR)
	id_ex_curr->status = STAT_PIP;
    if (ex_mem_state->op == P_ERROR)
	ex_mem_curr->status = STAT_PIP;
    if (mem_wb_state->op == P_ERROR)
	mem_wb_curr->status = STAT_PIP;
    
    /* Need to do decode after execute & memory stages,
       and memory stage before execute, in order to propagate
       forwarding values properly */
    do_if_stage();
    do_mem_stage();
    do_ex_stage();
    do_id_wb_stages();

    do_stall_check();
#if 0
    /* This doesn't seem necessary */
    if (id_ex_curr->status != STAT_AOK
	&& id_ex_curr->status != STAT_BUB) {
	if_id_state->op = P_BUBBLE;
	id_ex_state->op = P_BUBBLE;
    }
#endif

    /* Performance monitoring */
    if (mem_wb_curr->status != STAT_BUB && mem_wb_curr->icode != I_POP2) {
	starting_up = 0;
	instructions++;
	cycles++;
    } else {
	if (!starting_up)
	    cycles++;
    }
    
    sim_report();
    return status;
}

/*
  Run pipeline until one of following occurs:
  - An error status is encountered in WB.
  - max_instr instructions have completed through WB
  - max_cycle cycles have been simulated

  Return number of instructions executed.
  if statusp nonnull, then will be set to status of final instruction
  if ccp nonnull, then will be set to condition codes of final instruction
*/
word_t sim_run_pipe(word_t max_instr, word_t max_cycle, byte_t *statusp, cc_t *ccp)
{
    word_t icount = 0;
    word_t ccount = 0;
    byte_t run_status = STAT_AOK;
    while (icount < max_instr && ccount < max_cycle) {
        run_status = sim_step_pipe(max_instr-icount, ccount);
	if (run_status != STAT_BUB)
	    icount++;
	if (run_status != STAT_AOK && run_status != STAT_BUB)
	    break;
	ccount++;
    }
    if (statusp)
	*statusp = run_status;
    if (ccp)
	*ccp = cc;
    return icount;
}

/* If dumpfile set nonNULL, lots of status info printed out */
void sim_set_dumpfile(FILE *df)
{
    dumpfile = df;
}

/*
 * sim_log dumps a formatted string to the dumpfile, if it exists
 * accepts variable argument list
 */
void sim_log( const char *format, ... ) {
    if (dumpfile) {
	va_list arg;
	va_start( arg, format );
	vfprintf( dumpfile, format, arg );
	va_end( arg );
    }
}


/********************************
 * Part 5: Stage implementations
 *********************************/

/*************** Bubbled version of stages *************/

pc_ele bubble_pc = {0,STAT_AOK};
if_id_ele bubble_if_id = { I_NOP, 0, REG_NONE,REG_NONE,
			   0, 0, STAT_BUB, 0};
id_ex_ele bubble_id_ex = { I_NOP, 0, 0, 0, 0,
			   REG_NONE, REG_NONE, REG_NONE, REG_NONE,
			   STAT_BUB, 0};

ex_mem_ele bubble_ex_mem = { I_NOP, 0, FALSE, 0, 0,
			     REG_NONE, REG_NONE, STAT_BUB, 0};

mem_wb_ele bubble_mem_wb = { I_NOP, 0, 0, 0, REG_NONE, REG_NONE,
			     STAT_BUB, 0};

/*************** Stage Implementations *****************/

word_t gen_f_pc();
word_t gen_need_regids();
word_t gen_need_valC();
word_t gen_instr_valid();
word_t gen_f_predPC();
word_t gen_f_icode();
word_t gen_f_ifun();
word_t gen_f_stat();
word_t gen_instr_valid();

void do_if_stage()
{
    byte_t instr = HPACK(I_NOP, F_NONE);
    byte_t regids = HPACK(REG_NONE, REG_NONE);
    word_t valc = 0;
    word_t valp = f_pc = gen_f_pc();

    /* Ready to fetch instruction.  Speculatively fetch register byte
       and immediate word
    */
    imem_error = !get_byte_val(mem, valp, &instr);
    imem_icode = HI4(instr);
    imem_ifun = LO4(instr);
    if (!imem_error) {
      byte_t junk;
      /* Make sure can read maximum length instruction */
      imem_error = !get_byte_val(mem, valp+5, &junk);
    }
    if_id_next->icode = gen_f_icode();
    if_id_next->ifun  = gen_f_ifun();
    if (!imem_error) {
	sim_log("\tFetch: f_pc = 0x%llx, imem_instr = %s, f_instr = %s\n",
		f_pc, iname(instr),
		iname(HPACK(if_id_next->icode, if_id_next->ifun)));
    }

    instr_valid = gen_instr_valid();
    if (!instr_valid) 
      sim_log("\tFetch: Instruction code 0x%llx invalid\n", instr);
    if_id_next->status = gen_f_stat();

    valp++;
    if (gen_need_regids()) {
	get_byte_val(mem, valp, &regids);
	valp ++;
    }
    if_id_next->ra = HI4(regids);
    if_id_next->rb = LO4(regids);
    if (gen_need_valC()) {
	get_word_val(mem, valp, &valc);
	valp+= 8;
    }
    if_id_next->valp = valp;
    if_id_next->valc = valc;

    pc_next->pc = gen_f_predPC();

    pc_next->status = (if_id_next->status == STAT_AOK) ? STAT_AOK : STAT_BUB;

    if_id_next->stage_pc = f_pc;
}

word_t gen_d_srcA();
word_t gen_d_srcB();
word_t gen_d_dstE();
word_t gen_d_dstM();
word_t gen_d_valA();
word_t gen_d_valB();
word_t gen_w_dstE();
word_t gen_w_valE();
word_t gen_w_dstM();
word_t gen_w_valM();
word_t gen_Stat();

/* Implements both ID and WB */
void do_id_wb_stages()
{
    /* Set up write backs.  Don't occur until end of cycle */
    wb_destE = gen_w_dstE();
    wb_valE = gen_w_valE();
    wb_destM = gen_w_dstM();
    wb_valM = gen_w_valM();

    /* Update processor status */
    status = gen_Stat();

    id_ex_next->srca = gen_d_srcA();
    id_ex_next->srcb = gen_d_srcB();
    id_ex_next->deste = gen_d_dstE();
    id_ex_next->destm = gen_d_dstM();

    /* Read the registers */
    d_regvala = get_reg_val(reg, id_ex_next->srca);
    d_regvalb = get_reg_val(reg, id_ex_next->srcb);

    /* Do forwarding and valA selection */
    id_ex_next->vala = gen_d_valA();
    id_ex_next->valb = gen_d_valB();

    id_ex_next->icode = if_id_curr->icode;
    id_ex_next->ifun = if_id_curr->ifun;
    id_ex_next->valc = if_id_curr->valc;
    id_ex_next->stage_pc = if_id_curr->stage_pc;
    id_ex_next->status = if_id_curr->status;
}

word_t gen_alufun();
word_t gen_set_cc();
word_t gen_Bch();
word_t gen_aluA();
word_t gen_aluB();
word_t gen_e_valA();
word_t gen_e_dstE();

void do_ex_stage()
{
    alu_t alufun = gen_alufun();
    bool_t setcc = gen_set_cc();
    word_t alua, alub;

    alua = gen_aluA();
    alub = gen_aluB();

    e_bcond = 	cond_holds(cc, id_ex_curr->ifun);
    
    ex_mem_next->takebranch = e_bcond;

    if (id_ex_curr->icode == I_JMP)
      sim_log("\tExecute: instr = %s, cc = %s, branch %staken\n",
	      iname(HPACK(id_ex_curr->icode, id_ex_curr->ifun)),
	      cc_name(cc),
	      ex_mem_next->takebranch ? "" : "not ");
    
    /* Perform the ALU operation */
    word_t aluout = compute_alu(alufun, alua, alub);
    ex_mem_next->vale = aluout;
    sim_log("\tExecute: ALU: %c 0x%llx 0x%llx --> 0x%llx\n",
	    op_name(alufun), alua, alub, aluout);

    if (setcc) {
	cc_in = compute_cc(alufun, alua, alub);
	sim_log("\tExecute: New cc = %s\n", cc_name(cc_in));
    }

    ex_mem_next->icode = id_ex_curr->icode;
    ex_mem_next->ifun = id_ex_curr->ifun;
    ex_mem_next->vala = gen_e_valA();
    ex_mem_next->deste = gen_e_dstE();
    ex_mem_next->destm = id_ex_curr->destm;
    ex_mem_next->srca = id_ex_curr->srca;
    ex_mem_next->status = id_ex_curr->status;
    ex_mem_next->stage_pc = id_ex_curr->stage_pc;
}

/* Functions defined using HCL */
word_t gen_mem_addr();
word_t gen_mem_read();
word_t gen_mem_write();
#ifdef SNU
word_t gen_mem_byte();
#endif
word_t gen_m_stat();

void do_mem_stage()
{
    bool_t read = gen_mem_read();

    word_t valm = 0;

    mem_addr = gen_mem_addr();
    mem_data = ex_mem_curr->vala;
    mem_write = gen_mem_write();
    dmem_error = FALSE;

    if (read) {
	dmem_error = dmem_error || !get_word_val(mem, mem_addr, &valm);
	if (!dmem_error)
	  sim_log("\tMemory: Read 0x%llx from 0x%llx\n",
		  valm, mem_addr);
    }
    if (mem_write) {
	word_t sink;
	/* Do a read of address just to check validity */
	dmem_error = dmem_error || !get_word_val(mem, mem_addr, &sink);
	if (dmem_error)
	  sim_log("\tMemory: Invalid address 0x%llx\n",
		  mem_addr);
    }
    mem_wb_next->icode = ex_mem_curr->icode;
    mem_wb_next->ifun = ex_mem_curr->ifun;
    mem_wb_next->vale = ex_mem_curr->vale;
    mem_wb_next->valm = valm;
    mem_wb_next->deste = ex_mem_curr->deste;
    mem_wb_next->destm = ex_mem_curr->destm;
    mem_wb_next->status = gen_m_stat();
    mem_wb_next->stage_pc = ex_mem_curr->stage_pc;
}

/* Set stalling conditions for different stages */

word_t gen_F_stall(), gen_F_bubble();
word_t gen_D_stall(), gen_D_bubble();
word_t gen_E_stall(), gen_E_bubble();
word_t gen_M_stall(), gen_M_bubble();
word_t gen_W_stall(), gen_W_bubble();

p_stat_t pipe_cntl(char *name, word_t stall, word_t bubble)
{
    if (stall) {
	if (bubble) {
	    sim_log("%s: Conflicting control signals for pipe register\n",
		    name);
	    return P_ERROR;
	} else 
	    return P_STALL;
    } else {
	return bubble ? P_BUBBLE : P_LOAD;
    }
}

void do_stall_check()
{
    pc_state->op = pipe_cntl("PC", gen_F_stall(), gen_F_bubble());
    if_id_state->op = pipe_cntl("ID", gen_D_stall(), gen_D_bubble());
    id_ex_state->op = pipe_cntl("EX", gen_E_stall(), gen_E_bubble());
    ex_mem_state->op = pipe_cntl("MEM", gen_M_stall(), gen_M_bubble());
    mem_wb_state->op = pipe_cntl("WB", gen_W_stall(), gen_W_bubble());
}



