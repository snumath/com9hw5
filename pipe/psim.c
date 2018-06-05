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

#define MAXBUF 1024
#define DEFAULTNAME "Y86-64 Simulator: "

#ifdef HAS_GUI
#include <tk.h>
#endif /* HAS_GUI */

#define MAXARGS 128
#define MAXBUF 1024
#define TKARGS 3


/***************
 * Begin Globals
 ***************/

/* Simulator name defined and initialized by the compiled HCL file */
/* according to the -n argument supplied to hcl2c */
extern  char simname[];

/* Parameters modifed by the command line */
int gui_mode = FALSE;    /* Run in GUI mode instead of TTY mode? (-g) */
char *object_filename;   /* The input object file name. */
FILE *object_file;       /* Input file handle */
bool_t verbosity = 2;    /* Verbosity level [TTY only] (-v) */ 
#ifdef SNU
word_t instr_limit = 1000000; /* Instruction limit [TTY only] (-l) */
#else
word_t instr_limit = 10000; /* Instruction limit [TTY only] (-l) */
#endif
bool_t do_check = FALSE; /* Test with ISA simulator? [TTY only] (-t) */
#ifdef SNU
int snu_mode = FALSE;	/* Print output for automatic grading server */
#endif

/************* 
 * End Globals 
 *************/


/***************************
 * Begin function prototypes 
 ***************************/

word_t sim_run_pipe(word_t max_instr, word_t max_cycle, byte_t *statusp, cc_t *ccp);
static void usage(char *name);           /* Print helpful usage message */
static void run_tty_sim();               /* Run simulator in TTY mode */

#ifdef HAS_GUI
void addAppCommands(Tcl_Interp *interp); /* Add application-dependent commands */
#endif /* HAS_GUI */

/*************************
 * End function prototypes
 *************************/


/*******************************************************************
 * Part 1: This part is the initial entry point that handles general
 * initialization. It parses the command line and does any necessary
 * setup to run in either TTY or GUI mode, and then starts the
 * simulation.
 *******************************************************************/

/* 
 * sim_main - main simulator routine. This function is called from the
 * main() routine in the HCL file.
 */
int sim_main(int argc, char **argv)
{
    int i;
    int c;
    char *myargv[MAXARGS];
    
    /* Parse the command line arguments */
#ifdef SNU
    while ((c = getopt(argc, argv, "htgsl:v:")) != -1) {
#else
    while ((c = getopt(argc, argv, "htgl:v:")) != -1) {
#endif
	switch(c) {
	case 'h':
	    usage(argv[0]);
	    break;
	case 'l':
	    instr_limit = atoll(optarg);
	    break;
	case 'v':
	    verbosity = atoi(optarg);
	    if (verbosity < 0 || verbosity > 2) {
		printf("Invalid verbosity %d\n", verbosity);
		usage(argv[0]);
	    }
	    break;
	case 't':
	    do_check = TRUE;
	    break;
	case 'g':
	    gui_mode = TRUE;
	    break;
#ifdef SNU
	case 's':
		snu_mode = TRUE;
		break;
#endif
	default:
	    printf("Invalid option '%c'\n", c);
	    usage(argv[0]);
	    break;
	}
    }
#ifdef SNU
	if (snu_mode)
	{
		gui_mode = FALSE;
		verbosity = 0;
	}
#endif


    /* Do we have too many arguments? */
    if (optind < argc - 1) {
	printf("Too many command line arguments:");
	for (i = optind; i < argc; i++)
	    printf(" %s", argv[i]);
	printf("\n");
	usage(argv[0]);
    }


    /* The single unflagged argument should be the object file name */
    object_filename = NULL;
    object_file = NULL;
    if (optind < argc) {
	object_filename = argv[optind];
	object_file = fopen(object_filename, "r");
	if (!object_file) {
	    fprintf(stderr, "Couldn't open object file %s\n", object_filename);
	    exit(1);
	}
    }


    /* Run the simulator in GUI mode (-g flag) */
    if (gui_mode) {

#ifndef HAS_GUI
	printf("To run in GUI mode, you must recompile with the HAS_GUI constant defined.\n");
	exit(1);
#endif /* HAS_GUI */

	/* In GUI mode, we must specify the object file on command line */ 
	if (!object_file) {
	    printf("Missing object file argument in GUI mode\n");
	    usage(argv[0]);
	}

	/* Build the command line for the GUI simulator */
	for (i = 0; i < TKARGS; i++) {
	    if ((myargv[i] = malloc(MAXBUF*sizeof(char))) == NULL) {
		perror("malloc error");
		exit(1);
	    }
	}
	strcpy(myargv[0], argv[0]);
	strcpy(myargv[1], "pipe.tcl");
	strcpy(myargv[2], object_filename);
	myargv[3] = NULL;

	/* Start the GUI simulator */
#ifdef HAS_GUI
	Tk_Main(TKARGS, myargv, Tcl_AppInit);
#endif /* HAS_GUI */
	exit(0);
    }

    /* Otherwise, run the simulator in TTY mode (no -g flag) */
    run_tty_sim();

    exit(0);
}

/* 
 * run_tty_sim - Run the simulator in TTY mode
 */
static void run_tty_sim() 
{
    word_t icount = 0;
    byte_t run_status = STAT_AOK;
    cc_t result_cc = 0;
    word_t byte_cnt = 0;
    mem_t mem0, reg0;
    state_ptr isa_state = NULL;


    /* In TTY mode, the default object file comes from stdin */
    if (!object_file) {
	object_file = stdin;
    }

    if (verbosity >= 2)
	sim_set_dumpfile(stdout);
    sim_init();

    /* Emit simulator name */
    if (verbosity >= 2)
	printf("%s\n", simname);

    byte_cnt = load_mem(mem, object_file, 1);
    if (byte_cnt == 0) {
	fprintf(stderr, "No lines of code found\n");
	exit(1);
    } else if (verbosity >= 2) {
	printf("%lld bytes of code read\n", byte_cnt);
    }
    fclose(object_file);
    if (do_check) {
	isa_state = new_state(0);
	free_mem(isa_state->r);
	free_mem(isa_state->m);
	isa_state->m = copy_mem(mem);
	isa_state->r = copy_mem(reg);
	isa_state->cc = cc;
    }

    mem0 = copy_mem(mem);
    reg0 = copy_mem(reg);
    
    icount = sim_run_pipe(instr_limit, 5*instr_limit, &run_status, &result_cc);
    if (verbosity > 0) {
	printf("%lld instructions executed\n", icount);
	printf("Status = %s\n", stat_name(run_status));
	printf("Condition Codes: %s\n", cc_name(result_cc));
	printf("Changed Register State:\n");
	diff_reg(reg0, reg, stdout);
	printf("Changed Memory State:\n");
#ifdef SNU
	diff_mem(mem0, mem, stdout, (word_t) 0);
#else
	diff_mem(mem0, mem, stdout);
#endif
    }
#ifdef SNU
	if (snu_mode)
	{
		FILE *fp;
		if ((fp = fopen("memory.out", "w")) == NULL)
		{
			printf("Cannot write memory dump file\n");
			exit(1);
		}
		fprintf(fp, "Changed Memory State:\n");
		diff_mem(mem0, mem, fp, (word_t) 0x1000);
		fclose(fp);
	}
#endif
    if (do_check) {
	byte_t e = STAT_AOK;
	word_t step;
	bool_t match = TRUE;

	for (step = 0; step < instr_limit && e == STAT_AOK; step++) {
	    e = step_state(isa_state, stdout);
	}

	if (diff_reg(isa_state->r, reg, NULL)) {
	    match = FALSE;
	    if (verbosity > 0) {
		printf("ISA Register != Pipeline Register File\n");
		diff_reg(isa_state->r, reg, stdout);
	    }
	}
#ifdef SNU
	if (diff_mem(isa_state->m, mem, NULL, (word_t) 0)) {
#else
	if (diff_mem(isa_state->m, mem, NULL)) {
#endif
	    match = FALSE;
	    if (verbosity > 0) {
		printf("ISA Memory != Pipeline Memory\n");
#ifdef SNU
		diff_mem(isa_state->m, mem, stdout, (word_t) 0);
#else
		diff_mem(isa_state->m, mem, stdout);
#endif
	    }
	}
	if (isa_state->cc != result_cc) {
	    match = FALSE;
	    if (verbosity > 0) {
		printf("ISA Cond. Codes (%s) != Pipeline Cond. Codes (%s)\n",
		       cc_name(isa_state->cc), cc_name(result_cc));
	    }
	}
	if (match) {
	    printf("ISA Check Succeeds\n");
	} else {
	    printf("ISA Check Fails\n");
	}
    }
    /* Emit CPI statistics */
    {
#ifdef SNU
		long long total_cycles = cycles + 4;	// We count warming-up stages 
		printf("%lld instructions in %lld cycles, CPI = %.2f\n", 
				instructions, total_cycles, 
				(instructions)? (double) total_cycles / (double) instructions : (double) 1.0);
#else	
		double cpi = instructions > 0 ? (double) cycles/instructions : 1.0;
		printf("CPI: %lld cycles/%lld instructions = %.2f\n",
	   	    cycles, instructions, cpi);
#endif
    }
}

/*
 * usage - print helpful diagnostic information
 */
static void usage(char *name)
{
    printf("Usage: %s [-htg] [-l m] [-v n] file.yo\n", name);
    printf("file.yo arg required in GUI mode, optional in TTY mode (default stdin)\n");
    printf("   -h     Print this message\n");
    printf("   -g     Run in GUI mode instead of TTY mode (default TTY)\n");  
    printf("   -l m   Set instruction limit to m [TTY mode only] (default %lld)\n", instr_limit);
    printf("   -v n   Set verbosity level to 0 <= n <= 2 [TTY mode only] (default %d)\n", verbosity);
    printf("   -t     Test result against ISA simulator [TTY mode only]\n");
#ifdef SNU
	printf("   -s     Print output for automatic grading server\n");
#endif
    exit(0);
}


/*****************************************************************************
 * reporting code
 *****************************************************************************/

#ifdef HAS_GUI
/* used for formatting instructions */
static char status_msg[128];

static char *format_pc(pc_ptr state)
{
    char pstring[17];
    wstring(state->pc, 4, 64, pstring);
    sprintf(status_msg, "%s %s", stat_name(state->status), pstring);
    return status_msg;
}

static char *format_if_id(if_id_ptr state)
{
    char valcstring[17];
    char valpstring[17];
    wstring(state->valc, 4, 64, valcstring);
    wstring(state->valp, 4, 64, valpstring);
    sprintf(status_msg, "%s %s %s %s %s %s",
	    stat_name(state->status),
	    iname(HPACK(state->icode,state->ifun)),
	    reg_name(state->ra),
	    reg_name(state->rb),
	    valcstring,
	    valpstring);
    return status_msg;
}

static char *format_id_ex(id_ex_ptr state)
{
    char valcstring[17];
    char valastring[17];
    char valbstring[17];
    wstring(state->valc, 4, 64, valcstring);
    wstring(state->vala, 4, 64, valastring);
    wstring(state->valb, 4, 64, valbstring);
    sprintf(status_msg, "%s %s %s %s %s %s %s %s %s",
	    stat_name(state->status),
	    iname(HPACK(state->icode, state->ifun)),
	    valcstring,
	    valastring,
	    valbstring,
	    reg_name(state->deste),
	    reg_name(state->destm),
	    reg_name(state->srca),
	    reg_name(state->srcb));
    return status_msg;
}

static char *format_ex_mem(ex_mem_ptr state)
{
    char valestring[17];
    char valastring[17];
    wstring(state->vale, 4, 64, valestring);
    wstring(state->vala, 4, 64, valastring);
    sprintf(status_msg, "%s %s %c %s %s %s %s",
	    stat_name(state->status),
	    iname(HPACK(state->icode, state->ifun)),
	    state->takebranch ? 'Y' : 'N',
	    valestring,
	    valastring,
	    reg_name(state->deste),
	    reg_name(state->destm));

    return status_msg;
}

static char *format_mem_wb(mem_wb_ptr state)
{
    char valestring[17];
    char valmstring[17];
    wstring(state->vale, 4, 64, valestring);
    wstring(state->valm, 4, 64, valmstring);
    sprintf(status_msg, "%s %s %s %s %s %s",
	    stat_name(state->status),
	    iname(HPACK(state->icode, state->ifun)),
	    valestring,
	    valmstring,
	    reg_name(state->deste),
	    reg_name(state->destm));

    return status_msg;
}
#endif /* HAS_GUI */

/* Report system state */
/* SNU */
// static void sim_report()
void sim_report() 
{

#ifdef HAS_GUI
    if (gui_mode) {
	report_pc(f_pc, pc_curr->status != STAT_BUB,
		  if_id_curr->stage_pc, if_id_curr->status != STAT_BUB,
		  id_ex_curr->stage_pc, id_ex_curr->status != STAT_BUB,
		  ex_mem_curr->stage_pc, ex_mem_curr->status != STAT_BUB,
		  mem_wb_curr->stage_pc, mem_wb_curr->status != STAT_BUB);
	report_state("F", 0, format_pc(pc_next));
	report_state("F", 1, format_pc(pc_curr));
	report_state("D", 0, format_if_id(if_id_next));
	report_state("D", 1, format_if_id(if_id_curr));
	report_state("E", 0, format_id_ex(id_ex_next));
	report_state("E", 1, format_id_ex(id_ex_curr));
	report_state("M", 0, format_ex_mem(ex_mem_next));
	report_state("M", 1, format_ex_mem(ex_mem_curr));
	report_state("W", 0, format_mem_wb(mem_wb_next));
	report_state("W", 1, format_mem_wb(mem_wb_curr));
	/* signal_sources(); */
	show_cc(cc);
	show_stat(status);
	show_cpi();
    }
#endif

}


/*************************************************************
 * Part 3: This part contains support for the GUI simulator
 *************************************************************/

#ifdef HAS_GUI

/**********************
 * Begin Part 3 globals	
 **********************/

/* Hack for SunOS */
extern int matherr();
int *tclDummyMathPtr = (int *) matherr;

static char tcl_msg[256];

/* Keep track of the TCL Interpreter */
static Tcl_Interp *sim_interp = NULL;

static mem_t post_load_mem;

/**********************
 * End Part 3 globals	
 **********************/


/******************************************************************************
 *	function declarations
 ******************************************************************************/

int simResetCmd(ClientData clientData, Tcl_Interp *interp,
		int argc, char *argv[]);

int simLoadCodeCmd(ClientData clientData, Tcl_Interp *interp,
		   int argc, char *argv[]);

int simLoadDataCmd(ClientData clientData, Tcl_Interp *interp,
		   int argc, char *argv[]);

int simRunCmd(ClientData clientData, Tcl_Interp *interp,
	      int argc, char *argv[]);

int simModeCmd(ClientData clientData, Tcl_Interp *interp,
	       int argc, char *argv[]);

void addAppCommands(Tcl_Interp *interp);


/******************************************************************************
 *	tcl command definitions
 ******************************************************************************/

/* Implement command versions of the simulation functions */
int simResetCmd(ClientData clientData, Tcl_Interp *interp,
		int argc, char *argv[])
{
    sim_interp = interp;
    if (argc != 1) {
	interp->result = "No arguments allowed";
	return TCL_ERROR;
    }
    sim_reset();
    if (post_load_mem) {
	free_mem(mem);
	mem = copy_mem(post_load_mem);
    }
    interp->result = stat_name(STAT_AOK);
    return TCL_OK;
}

int simLoadCodeCmd(ClientData clientData, Tcl_Interp *interp,
		   int argc, char *argv[])
{
    FILE *code_file;
    word_t code_count;
    sim_interp = interp;
    if (argc != 2) {
	interp->result = "One argument required";
	return TCL_ERROR;
    }
    code_file = fopen(argv[1], "r");
    if (!code_file) {
	sprintf(tcl_msg, "Couldn't open code file '%s'", argv[1]);
	interp->result = tcl_msg;
	return TCL_ERROR;
    }
    sim_reset();
    code_count = load_mem(mem, code_file, 0);
    post_load_mem = copy_mem(mem);
    sprintf(tcl_msg, "%lld", code_count);
    interp->result = tcl_msg;
    fclose(code_file);
    return TCL_OK;
}

int simLoadDataCmd(ClientData clientData, Tcl_Interp *interp,
		   int argc, char *argv[])
{
    FILE *data_file;
    word_t word_count = 0;
    interp->result = "Not implemented";
    return TCL_ERROR;


    sim_interp = interp;
    if (argc != 2) {
	interp->result = "One argument required";
	return TCL_ERROR;
    }
    data_file = fopen(argv[1], "r");
    if (!data_file) {
	sprintf(tcl_msg, "Couldn't open data file '%s'", argv[1]);
	interp->result = tcl_msg;
	return TCL_ERROR;
    }
    sprintf(tcl_msg, "%lld", word_count);
    interp->result = tcl_msg;
    fclose(data_file);
    return TCL_OK;
}


int simRunCmd(ClientData clientData, Tcl_Interp *interp,
	      int argc, char *argv[])
{
    word_t cycle_limit = 1;
    byte_t status;
    cc_t cc;
    sim_interp = interp;
    if (argc > 2) {
	interp->result = "At most one argument allowed";
	return TCL_ERROR;
    }
    if (argc >= 2 &&
	(sscanf(argv[1], "%lld", &cycle_limit) != 1 ||
	 cycle_limit < 0)) {
	sprintf(tcl_msg, "Cannot run for '%s' cycles!", argv[1]);
	interp->result = tcl_msg;
	return TCL_ERROR;
    }
    sim_run_pipe(cycle_limit + 5, cycle_limit, &status, &cc);
    interp->result = stat_name(status);
    return TCL_OK;
}

int simModeCmd(ClientData clientData, Tcl_Interp *interp,
	       int argc, char *argv[])
{
    sim_interp = interp;
    if (argc != 2) {
	interp->result = "One argument required";
	return TCL_ERROR;
    }
    interp->result = argv[1];
    if (strcmp(argv[1], "wedged") == 0)
	sim_mode = S_WEDGED;
    else if (strcmp(argv[1], "stall") == 0)
	sim_mode = S_STALL;
    else if (strcmp(argv[1], "forward") == 0)
	sim_mode = S_FORWARD;
    else {
	sprintf(tcl_msg, "Unknown mode '%s'", argv[1]);
	interp->result = tcl_msg;
	return TCL_ERROR;
    }
    return TCL_OK;
}


/******************************************************************************
 *	registering the commands with tcl
 ******************************************************************************/

void addAppCommands(Tcl_Interp *interp)
{
    sim_interp = interp;
    Tcl_CreateCommand(interp, "simReset", (Tcl_CmdProc *) simResetCmd,
		      (ClientData) NULL, (Tcl_CmdDeleteProc *) NULL);
    Tcl_CreateCommand(interp, "simCode", (Tcl_CmdProc *) simLoadCodeCmd,
		      (ClientData) NULL, (Tcl_CmdDeleteProc *) NULL);
    Tcl_CreateCommand(interp, "simData", (Tcl_CmdProc *) simLoadDataCmd,
		      (ClientData) NULL, (Tcl_CmdDeleteProc *) NULL);
    Tcl_CreateCommand(interp, "simRun", (Tcl_CmdProc *) simRunCmd,
		      (ClientData) NULL, (Tcl_CmdDeleteProc *) NULL);
    Tcl_CreateCommand(interp, "setSimMode", (Tcl_CmdProc *) simModeCmd,
		      (ClientData) NULL, (Tcl_CmdDeleteProc *) NULL);
} 

/******************************************************************************
 *	tcl functionality called from within C
 ******************************************************************************/

/* Provide mechanism for simulator to update register display */
void signal_register_update(reg_id_t r, word_t val) {
    int code;
    sprintf(tcl_msg, "setReg %d %lld 1", (int) r, (word_t) val);
    code = Tcl_Eval(sim_interp, tcl_msg);
    if (code != TCL_OK) {
	fprintf(stderr, "Failed to signal register set\n");
	fprintf(stderr, "Error Message was '%s'\n", sim_interp->result);
    }
}

/* Provide mechanism for simulator to generate memory display */
void create_memory_display() {
    int code;
    sprintf(tcl_msg, "createMem %lld %lld", minAddr, memCnt);
    code = Tcl_Eval(sim_interp, tcl_msg);
    if (code != TCL_OK) {
	fprintf(stderr, "Command '%s' failed\n", tcl_msg);
	fprintf(stderr, "Error Message was '%s'\n", sim_interp->result);
    } else {
	word_t i;
	for (i = 0; i < memCnt && code == TCL_OK; i+=8) {
	    word_t addr = minAddr+i;
	    word_t val;
	    if (!get_word_val(mem, addr, &val)) {
		fprintf(stderr, "Out of bounds memory display\n");
		return;
	    }
	    sprintf(tcl_msg, "setMem %lld %lld", addr, val);
	    code = Tcl_Eval(sim_interp, tcl_msg);
	}
	if (code != TCL_OK) {
	    fprintf(stderr, "Couldn't set memory value\n");
	    fprintf(stderr, "Error Message was '%s'\n", sim_interp->result);
	}
    }
}

/* Provide mechanism for simulator to update memory value */
void set_memory(word_t addr, word_t val) {
    int code;
    word_t nminAddr = minAddr;
    word_t nmemCnt = memCnt;

    /* First see if we need to expand memory range */
    if (memCnt == 0) {
	nminAddr = addr;
	nmemCnt = 8;
    } else if (addr < minAddr) {
	nminAddr = addr;
	nmemCnt = minAddr + memCnt - addr;
    } else if (addr >= minAddr+memCnt) {
	nmemCnt = addr-minAddr+8;
    }
    /* Now make sure nminAddr & nmemCnt are multiples of 16 */
    nmemCnt = ((nminAddr & 0xF) + nmemCnt + 0xF) & ~0xF;
    nminAddr = nminAddr & ~0xF;

    if (nminAddr != minAddr || nmemCnt != memCnt) {
	minAddr = nminAddr;
	memCnt = nmemCnt;
	create_memory_display();
    } else {
	sprintf(tcl_msg, "setMem %lld %lld", addr, val);
	code = Tcl_Eval(sim_interp, tcl_msg);
	if (code != TCL_OK) {
	    fprintf(stderr, "Couldn't set memory value 0x%llx to 0x%llx\n",
		    addr, val);
	    fprintf(stderr, "Error Message was '%s'\n", sim_interp->result);
	}
    }
}

/* Provide mechanism for simulator to update condition code display */
void show_cc(cc_t cc)
{
    int code;
    sprintf(tcl_msg, "setCC %d %d %d",
	    GET_ZF(cc), GET_SF(cc), GET_OF(cc));
    code = Tcl_Eval(sim_interp, tcl_msg);
    if (code != TCL_OK) {
	fprintf(stderr, "Failed to display condition codes\n");
	fprintf(stderr, "Error Message was '%s'\n", sim_interp->result);
    }
}

/* Provide mechanism for simulator to update status display */
void show_stat(stat_t stat)
{
    int code;
    sprintf(tcl_msg, "showStat %s", stat_name(stat));
    code = Tcl_Eval(sim_interp, tcl_msg);
    if (code != TCL_OK) {
	fprintf(stderr, "Failed to display status\n");
	fprintf(stderr, "Error Message was '%s'\n", sim_interp->result);
    }
}



/* Provide mechanism for simulator to update performance information */
void show_cpi() {
    int code;
    double cpi = instructions > 0 ? (double) cycles/instructions : 1.0;
    sprintf(tcl_msg, "showCPI %lld %lld %.2f",
	    cycles, instructions, (double) cpi);
    code = Tcl_Eval(sim_interp, tcl_msg);
    if (code != TCL_OK) {
	fprintf(stderr, "Failed to display CPI\n");
	fprintf(stderr, "Error Message was '%s'\n", sim_interp->result);
    }
}

char *rname[] = {"none", "ea", "eb", "me", "wm", "we"};

/* provide mechanism for simulator to specify source registers */
void signal_sources() {
    int code;
    sprintf(tcl_msg, "showSources %s %s",
	    rname[amux], rname[bmux]);
    code = Tcl_Eval(sim_interp, tcl_msg);
    if (code != TCL_OK) {
	fprintf(stderr, "Failed to signal forwarding sources\n");
	fprintf(stderr, "Error Message was '%s'\n", sim_interp->result);
    }
}

/* Provide mechanism for simulator to clear register display */
void signal_register_clear() {
    int code;
    code = Tcl_Eval(sim_interp, "clearReg");
    if (code != TCL_OK) {
	fprintf(stderr, "Failed to signal register clear\n");
	fprintf(stderr, "Error Message was '%s'\n", sim_interp->result);
    }
}

/* Provide mechanism for simulator to report instructions as they are 
   read in
*/

void report_line(word_t line_no, word_t addr, char *hex, char *text) {
    int code;
    sprintf(tcl_msg, "addCodeLine %lld %lld {%s} {%s}", line_no, addr, hex, text);
    code = Tcl_Eval(sim_interp, tcl_msg);
    if (code != TCL_OK) {
	fprintf(stderr, "Failed to report code line 0x%llx\n", addr);
	fprintf(stderr, "Error Message was '%s'\n", sim_interp->result);
    }
}


/* Provide mechanism for simulator to report which instructions are in
   which stages */
void report_pc(unsigned fpc, unsigned char fpcv,
	       unsigned dpc, unsigned char dpcv,
	       unsigned epc, unsigned char epcv,
	       unsigned mpc, unsigned char mpcv,
	       unsigned wpc, unsigned char wpcv)
{
    int status;
    char addr[10];
    char code[12];
    Tcl_DString cmd;
    Tcl_DStringInit(&cmd);
    Tcl_DStringAppend(&cmd, "simLabel ", -1);
    Tcl_DStringStartSublist(&cmd);
    if (fpcv) {
	sprintf(addr, "%u", fpc);
	Tcl_DStringAppendElement(&cmd, addr);
    }
    if (dpcv) {
	sprintf(addr, "%u", dpc);
	Tcl_DStringAppendElement(&cmd, addr);
    }
    if (epcv) {
	sprintf(addr, "%u", epc);
	Tcl_DStringAppendElement(&cmd, addr);
    }
    if (mpcv) {
	sprintf(addr, "%u", mpc);
	Tcl_DStringAppendElement(&cmd, addr);
    }
    if (wpcv) {
	sprintf(addr, "%u", wpc);
	Tcl_DStringAppendElement(&cmd, addr);
    }
    Tcl_DStringEndSublist(&cmd);
    Tcl_DStringStartSublist(&cmd);
    sprintf(code, "%s %s %s %s %s",
	    fpcv ? "F" : "",
	    dpcv ? "D" : "",
	    epcv ? "E" : "",
	    mpcv ? "M" : "",
	    wpcv ? "W" : "");
    Tcl_DStringAppend(&cmd, code, -1);
    Tcl_DStringEndSublist(&cmd);
    /* Debug 
       fprintf(stderr, "Code '%s'\n", Tcl_DStringValue(&cmd));
    */
    status = Tcl_Eval(sim_interp, Tcl_DStringValue(&cmd));
    if (status != TCL_OK) {
	fprintf(stderr, "Failed to report pipe code '%s'\n", code);
	fprintf(stderr, "Error Message was '%s'\n", sim_interp->result);
    }
}

/* Report single line of pipeline state */
void report_state(char *id, word_t current, char *txt)
{
    int status;
    sprintf(tcl_msg, "updateStage %s %lld {%s}", id, current,txt);
    status = Tcl_Eval(sim_interp, tcl_msg);
    if (status != TCL_OK) {
	fprintf(stderr, "Failed to report pipe status\n");
	fprintf(stderr, "\tStage %s.%s, status '%s'\n",
		id, current ? "current" : "next", txt);
	fprintf(stderr, "\tError Message was '%s'\n", sim_interp->result);
    }
}


/*
 * Tcl_AppInit - Called by TCL to perform application-specific initialization.
 */
int Tcl_AppInit(Tcl_Interp *interp)
{
    /* Tell TCL about the name of the simulator so it can  */
    /* use it as the title of the main window */
    Tcl_SetVar(interp, "simname", simname, TCL_GLOBAL_ONLY);

    if (Tcl_Init(interp) == TCL_ERROR)
	return TCL_ERROR;
    if (Tk_Init(interp) == TCL_ERROR)
	return TCL_ERROR;
    Tcl_StaticPackage(interp, "Tk", Tk_Init, Tk_SafeInit);

    /* Call procedure to add new commands */
    addAppCommands(interp);

    /*
     * Specify a user-specific startup file to invoke if the application
     * is run interactively.  Typically the startup file is "~/.apprc"
     * where "app" is the name of the application.  If this line is deleted
     * then no user-specific startup file will be run under any conditions.
     */
    Tcl_SetVar(interp, "tcl_rcFileName", "~/.wishrc", TCL_GLOBAL_ONLY);
    return TCL_OK;

}

#endif /* HAS_GUI */


/**************************************************************
 * Part 4: Code for implementing pipelined processor simulators
 *************************************************************/

/******************************************************************************
 *	defines
 ******************************************************************************/

#define MAX_STAGE 10

/******************************************************************************
 *	static variables
 ******************************************************************************/

static pipe_ptr pipes[MAX_STAGE];
static int pipe_count = 0;

/******************************************************************************
 *	function definitions
 ******************************************************************************/

/* Create new pipe with count bytes of state */
/* bubble_val indicates state corresponding to pipeline bubble */
pipe_ptr new_pipe(int count, void *bubble_val)
{
  pipe_ptr result = (pipe_ptr) malloc(sizeof(pipe_ele));
  result->current = malloc(count);
  result->next = malloc(count);
  memcpy(result->current, bubble_val, count);
  memcpy(result->next, bubble_val, count);
  result->count = count;
  result->op = P_LOAD;
  result->bubble_val = bubble_val;
  pipes[pipe_count++] = result; 
  return result;
}

/* Update all pipes */
void update_pipes()
{
  int s;
  for (s = 0; s < pipe_count; s++) {
    pipe_ptr p = pipes[s];
    switch (p->op)
      {
      case P_BUBBLE:
      	/* insert a bubble into the next stage */
      	memcpy(p->current, p->bubble_val, p->count);
      	break;
      
      case P_LOAD:
      	/* copy calculated state from previous stage */
      	memcpy(p->current, p->next, p->count);
      	break;
      case P_ERROR:
	  /* Like a bubble, but insert error condition */
      	memcpy(p->current, p->bubble_val, p->count);
      	break;
      case P_STALL:
      default:
      	/* do nothing: next stage gets same instr again */
      	;
      }
    if (p->op != P_ERROR)
	p->op = P_LOAD;
  }
}

/* Set all pipes to bubble values */
void clear_pipes()
{
  int s;
  for (s = 0; s < pipe_count; s++) {
    pipe_ptr p = pipes[s];
    memcpy(p->current, p->bubble_val, p->count);
    memcpy(p->next, p->bubble_val, p->count);
    p->op = P_LOAD;
  }
}

/******************** Utility Code *************************/

/* Representations of digits */
static char digits[16] =
   {'0', '1', '2', '3', '4', '5', '6', '7',
    '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'};

/* Print hex/oct/binary format with leading zeros */
/* bpd denotes bits per digit  Should be in range 1-4,
   pbw denotes bits per word.*/
void wprint(uword_t x, int bpd, int bpw, FILE *fp)
{
  int digit;
  uword_t mask = ((uword_t) 1 << bpd) - 1;
  for (digit = (bpw-1)/bpd; digit >= 0; digit--) {
    uword_t val = (x >> (digit * bpd)) & mask;
    putc(digits[val], fp);
  }
}

/* Create string in hex/oct/binary format with leading zeros */
/* bpd denotes bits per digit  Should be in range 1-4,
   pbw denotes bits per word.*/
void wstring(uword_t x, int bpd, int bpw, char *str)
{
  int digit;
  uword_t mask = ((uword_t) 1 << bpd) - 1;
  for (digit = (bpw-1)/bpd; digit >= 0; digit--) {
    uword_t val = (x >> (digit * bpd)) & mask;
    *str++ = digits[val];
  }
  *str = '\0';
}

