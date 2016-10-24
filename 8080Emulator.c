#include "stdafx.h"
#include "8080Emulator.h"
#include <stdlib.h>

#define DIAGNOSTIC_MODE 1

// Used to implement O(1) byte parity calculation 
static const byte ParityTable256[256] =
{
#   define P2(n) n, n^1, n^1, n
#   define P4(n) P2(n), P2(n^1), P2(n^1), P2(n)
#   define P6(n) P4(n), P4(n^1), P4(n^1), P4(n)
	P6(0), P6(1), P6(1), P6(0)
};

// Return 1 if condition needed for operation in IR is set
byte isConditionSet(State8080 *);

// Get the address (&state->register) of register(s) referred to in opcode. Return NULL if mem ref.
byte* getOpCodeRegister(State8080 *, int isDestinationRegister); 

// Get the register pair referenced in IR, boolean arg to chose SP or A/F inst
unsigned short int getRegisterPairByOpCode(State8080 *, int getFlags);

// Set the register pair referenced in IR, boolean chose SP or A/F inst
void setRegisterPairByOpCode(State8080 *, unsigned short int, int setFlags);

// Get memory address 0 - 56 of RST interrupt handler
unsigned short int getRstAddress(State8080 *);

// Update the flags with byte of data to compare
void updateFlags(State8080 *, byte dataToEval);

// Update the flags after arithmetic operation
void updateFlagsArithmetic(State8080 *, unsigned short int resultOfOperation, int isSubtraction);

// Update the flags after logical operation
void updateFlagsLogical(State8080 *, unsigned short int resultOfOperation);

// Get the byte in a register reqested by lowest 3 bits in IR
byte getOperandFromRegister(State8080 *);

// Create the state of the 8080 processor + max addressable memory
State8080 *init8080State(void);

// Execute instruction that is pointed by the PC register
void executeInst(State8080 *);

// Load a file in memory starting at address offset
void loadFileIntoMemory(State8080 *, char *fname, unsigned short int offset);

// Debug function headers below
void printGeneralRegisters(State8080 *);
void printStatusRegister(State8080 *);
void printInternalRegisters(State8080 *);



// --------START OF OP CODE FUNCTIONS--------

/*
Carry bit instructions

Condition bits affected: Carry
*/

// Set carry
void stc(State8080 *state) {
	state->flags.c = 1;
	state->pc++;
}

// Complement carry
void cmc(State8080 *state) {
	state->flags.c ^= 1;
	state->pc++;
}

/*
Single register instruction

Condition bits affected:	
INR,DCR	: Zero, sign, parity
CMA		: None
NOP		: None
DAA		: Zero, sign, parity, carry, aux. carry
*/

// Increment register REGM
void inr(State8080 *state) {
	byte *regm = getOpCodeRegister(state, 1);
	if (regm == NULL) { // this means it is mem address pointed by HL pair
		unsigned short int addr = state->h << 8 | state->l;
		updateFlags(state, ++state->memory[addr]);
	} else {
		updateFlags(state, ++*regm);
	}
	state->pc++;
}

// Decrement register REGM
void dcr(State8080 *state) {
	byte *regm = getOpCodeRegister(state, 1);
	if (regm == NULL) { // this means it is mem address pointed by HL pair
		unsigned short int addr = state->h << 8 | state->l;
		updateFlags(state, --state->memory[addr]);
	} else {
		updateFlags(state, --*regm);
	}
	state->pc++;
}

// Complement accumulator
void cma(State8080 *state) {
	state->a = ~state->a;
	state->pc++;
}

// Convert accumulator contents to form two decimal digits
// NOTE: NOT IMPLEMENTED! We don't need this for Space Invaders emulation
void daa(State8080 *state) {
	state->pc++;
}


/*
Data transfer instructions

Condition bits affected: NONE
*/

// Load register DST from register SRC
void mov(State8080 *state) {
	byte* src = getOpCodeRegister(state, 0);
	byte* dst = getOpCodeRegister(state, 1);

	if (src == NULL) { // means we are moving from memory pointed at HL to a register
		unsigned short int addr = state->h << 8 | state->l;
		*dst = state->memory[addr];
	} else if (dst == NULL) { // means we are moving from register to memory pointed at by HL
		unsigned short int addr = state->h << 8 | state->l;
		state->memory[addr] = *src;
	} else { // register to register MOV
		*dst = *src;
	}
	state->pc++;
}

// Store accumulator at memory location referenced by the sepcified register pair
void stax(State8080 *state) {
	unsigned short int addr;
	if (state->ir.middle == 2) { // middle 3 bits of opcode can be 010 for STAX D, or 000 for STAX B
		addr = state->d << 8 | state->e;
	} else {
		addr = state->b << 8 | state->c;
	}
	state->memory[addr] = state->a;
	state->pc++;
}

// Load accumulator at memory location referenced by the sepcified register pair
void ldax(State8080 *state) {
	unsigned short int addr;
	if (state->ir.middle == 3) { // middle 3 bits of opcode can be 011 for LDAX D, or 001 for STAX B
		addr = state->d << 8 | state->e;
	} else {
		addr = state->b << 8 | state->c;
	}
	state->a = state->memory[addr];
	state->pc++;
}

/*
Register or memory to accumulator instructions

Condition bits affected:
ADD, ADC, SUB, SBB	: Carry, sign, zero, parity, aux. carry
ANA, XRA, ORA		: Sign, zero, parity, carry is zeroed
CMP					: Everything! (except interrupt flag)
*/

// Add REGM to accumulator
void add(State8080 *state) {
	unsigned short int res = state->a + getOperandFromRegister(state);
	state->a = (byte)res;
	updateFlagsArithmetic(state, res, 0);
	state->pc++;
}

// Add REGM to accumulator with carry
void adc(State8080 *state) {
	unsigned short int res = state->a + getOperandFromRegister(state) + state->flags.c;
	state->a = (byte)res;
	updateFlagsArithmetic(state, res, 0);
	state->pc++;
}

// Subtract REGM from accumulator
void sub(State8080 *state) {
	// subtraction is being done like how the ALU does it (we do this not for simulation purposes, but to catch carry bit)
	unsigned short int res = state->a + ((getOperandFromRegister(state) ^ 0xFF) + 1);
	state->a = (byte)res;
	updateFlagsArithmetic(state, res, 1);
	state->pc++;
}

// Subtract REGM from accumulator with borrow
void sbb(State8080 *state) {
	unsigned short int res = state->a + (((getOperandFromRegister(state) + state->flags.c) ^ 0xFF) + 1);
	state->a = (byte)res;
	updateFlagsArithmetic(state, res, 1);
	state->pc++;
}

// AND accumulator with REGM
void ana(State8080 *state) {
	unsigned short int res = state->a & getOperandFromRegister(state);
	state->a = (byte)res;
	updateFlagsLogical(state, res);
	state->pc++;
}

// XOR accumulator with REGM
void xra(State8080 *state) {
	unsigned short int res = state->a ^ getOperandFromRegister(state);
	state->a = (byte)res;
	updateFlagsLogical(state, res);
	state->pc++;
}

// OR accumulator with REGM
void ora(State8080 *state) {
	unsigned short int res = state->a | getOperandFromRegister(state);
	state->a = (byte)res;
	updateFlagsLogical(state, res);
	state->pc++;
}

// Compare REGM with accumulator
void cmp(State8080 *state) {
	// note how cmp does not change accumulator, only condition flags
	unsigned short int res = state->a + ((getOperandFromRegister(state) ^ 0xFF) + 1);
	updateFlagsArithmetic(state, res, 1);
	state->pc++;
}

/*
Rotate accumulator instructions

Condition bits affected: Carry
*/

// Set carry = A7, rotate accumulator left
void rlc(State8080 *state) {
	state->flags.c = state->a > 127;
	state->a = state->a << 1 | state->flags.c;
	state->pc++;
}

// Set carry = A0, rotate accumulator right
void rrc(State8080 *state) {
	state->flags.c = state->a & 1;
	state->a = state->a >> 1 | state->flags.c << 7;
	state->pc++;
}

// Rotate accumulator left through carry
void ral(State8080 *state) {
	byte carryBit = state->flags.c;
	state->flags.c = state->a > 127;
	state->a = state->a << 1 | carryBit;
	state->pc++;
}

// Rotate accumulator right through carry
void rar(State8080 *state) {
	byte carryBit = state->flags.c;
	state->flags.c = state->a & 1;
	state->a = state->a >> 1 | carryBit << 7;
	state->pc++;
}

/*
Register pair instructions

Register pairs:
00 : B and C
01 : D and E
10 : H and L
11 : A and flags or SP

Condition bits affected:
PUSH, INX, DCX, XCHG, XTHL, XPHL	: NONE
POP									: If RP=PSW, all condition bits are restored from the stack
DAD									: Carry
*/

// Save RP on the stack, RP = PSW saves accumulator and condition bits
void push(State8080 *state) {
	unsigned short int regPair = getRegisterPairByOpCode(state, 1);
	byte reg1 = regPair >> 8;
	byte reg2 = (byte)regPair;
	state->memory[(unsigned short int) (state->sp + (~1 + 1))] = reg1; // Sub 1 from PC
	state->memory[(unsigned short int) (state->sp + (~2 + 1))] = reg2; // Sub 2 from PC
	state->sp = state->sp + (~2 + 1);
	state->pc++;
}

// Restore RP from the stack, RP = PSW restores accumulator and condition bits
void pop(State8080 *state) {
	byte reg2 = state->memory[state->sp];
	byte reg1 = state->memory[state->sp + 1];
	unsigned short int regPair = reg1 << 8 | reg2;
	setRegisterPairByOpCode(state, regPair, 1);
	state->sp = state->sp + 2;
	state->pc++;
}

// Add RP to the 16-bit number in H and L
void dad(State8080 *state) {
	unsigned short int hl = state->h << 8 | state->l;
	unsigned short int regPair = getRegisterPairByOpCode(state, 0);
	int res = hl + regPair;
	if (res > 65535) {
		state->flags.c = 1;
	}
	state->h = res >> 8;
	state->l = res;
	state->pc++;
}

// Increment RP by 1
void inx(State8080 *state) {
	unsigned short int regPair = getRegisterPairByOpCode(state, 0);
	regPair++;
	setRegisterPairByOpCode(state, regPair, 0);
	state->pc++;
}

// Decrement RP by 1
void dcx(State8080 *state) {
	unsigned short int regPair = getRegisterPairByOpCode(state, 0);
	regPair--;
	setRegisterPairByOpCode(state, regPair, 0);
	state->pc++;
}

// Exchange the 16 bit number in H and L with that in D and E
void xchg(State8080 *state) {
	byte h = state->h;
	byte l = state->l;
	state->h = state->d;
	state->l = state->e;
	state->d = h;
	state->e = l;
	state->pc++;
}

// Exchange the last values saved in the stack with H and L
void xthl(State8080 *state) {
	byte h = state->h;
	byte l = state->l;
	state->h = *(state->memory + state->sp + 1);
	state->l = *(state->memory + state->sp);
	*(state->memory + state->sp + 1) = h;
	*(state->memory + state->sp) = l;
	state->pc++;
}

// Load stack pointer from H and L
void sphl(State8080 *state) {
	state->sp = state->h << 8 | state->l;
	state->pc++;
}

/*
Immediate instructions

Condition bits affected:
LXI, MVI				: NONE
ADI, ACI, SUI, SBI		: Carry, sign, zero, parity, aux. carry
ANI, XRI, ORI			: Zero, sign, parity. CARRY IS ZEROED!
CPI						: All except interrupt flag
*/

// Move 16 bit immediate data into RP
void lxi(State8080 *state) {
	unsigned short int operand = *(state->memory + state->pc + 1) | 
		*(state->memory + state->pc + 2) << 8;
	setRegisterPairByOpCode(state, operand, 0);
	state->pc += 3;
}

// Move immediate DATA into REGM
void mvi(State8080 *state) {
	byte* dstReg = getOpCodeRegister(state, 1);
	byte immediateData = state->memory[state->pc + 1];
	if (dstReg == NULL) { // means we are moving immediate data to memory pointed at by HL
		unsigned short int addr = state->h << 8 | state->l;
		state->memory[addr] = immediateData;
	} else { // store imediate data into register
		*dstReg = immediateData;
	}
	state->pc += 2;
}

// Add immediate data to accumulator
void adi(State8080 *state) {
	unsigned short int result = state->memory[state->pc + 1] + state->a;
	updateFlagsArithmetic(state, result, 0);
	state->a = (byte)result;
	state->pc += 2;
}

// Add immediate data to accumulator with carry
void aci(State8080 *state) {
	unsigned short int result = state->memory[state->pc + 1] + state->a + state->flags.c;
	updateFlagsArithmetic(state, result, 0);
	state->a = (byte)result;
	state->pc += 2;
}

// Subtract immediate data from accumulator
void sui(State8080 *state) {
	unsigned short int result = state->a + ((state->memory[state->pc + 1] ^ 0xFF) + 1);
	updateFlagsArithmetic(state, result, 1);
	state->a = (byte)result;
	state->pc += 2;
}

// Subtract imediate data from accumulator with borrow
void sbi(State8080 *state) {
	unsigned short int result = state->a + (((state->memory[state->pc + 1] + state->flags.c) ^ 0xFF) + 1);
	updateFlagsArithmetic(state, result, 1);
	state->a = (byte)result;
	state->pc += 2;
}

// AND accumulator with immediate data
void ani(State8080 *state) {
	state->a = state->a & state->memory[state->pc + 1];
	updateFlagsLogical(state, (unsigned short int) state->a);
	state->pc += 2;
}

// XOR accumulator with immediate data
void xri(State8080 *state) {
	state->a = state->a ^ state->memory[state->pc + 1];
	updateFlagsLogical(state, (unsigned short int) state->a);
	state->pc += 2;
}

// OR accumulator with immediate data
void ori(State8080 *state) {
	state->a = state->a | state->memory[state->pc + 1];
	updateFlagsLogical(state, (unsigned short int) state->a);
	state->pc += 2;
}

// Compare immediate data with accumulator
void cpi(State8080 *state) {
	updateFlagsArithmetic(state, state->a + ((state->memory[state->pc + 1] ^ 0xFF) + 1), 1);
	state->pc += 2;
}

/*
Direct addressing instructions

Condition bits affected: NONE
*/

// Store accumulator at location ADDR
void sta(State8080 *state) {
	unsigned short int addr = state->memory[state->pc + 2] << 8 | state->memory[state->pc + 1];
	state->memory[addr] = state->a;
	state->pc += 3;
}

// Load accumulator from location ADDR
void lda(State8080 *state) {
	unsigned short int addr = state->memory[state->pc + 2] << 8 | state->memory[state->pc + 1];
	state->a = state->memory[addr];
	state->pc += 3;
}

// Store L and H at ADDR and ADDR+1
void shld(State8080 *state) {
	unsigned short int addr = state->memory[state->pc + 2] << 8 | state->memory[state->pc + 1];
	state->memory[addr + 1] = state->h;
	state->memory[addr] = state->l;
	state->pc += 3;
}

// Load L and H from ADDR and ADDR+1
void lhld(State8080 *state) {
	unsigned short int addr = state->memory[state->pc + 2] << 8 | state->memory[state->pc + 1];
	state->h = state->memory[addr + 1];
	state->l = state->memory[addr];
	state->pc += 3;
}

/*
Jump instructions

Condition bits affected: NONE
*/

// Jump to location specified by register H and L
void pchl(State8080 *state){
	state->pc = state->h << 8 | state->l;
}

// Jump to location ADDR
void jmp(State8080 *state) {
	// if lowest 3 bit of IR is 011, then it is non conditional jump
	// else the middle 3 bits of IR represt what condition must be true for jump
	if (isConditionSet(state) || state->ir.low == 3) {
		unsigned short int addr = state->memory[state->pc + 2] << 8 | state->memory[state->pc + 1];
		state->pc = addr;
	} else {
		state->pc += 3;
	}

}

/*
Call instructions

Condition bits affected: NONE
*/

// Call subroutine and push return address onto stack
void call(State8080 *state) {
	// When lowest 3 bits of IR = 4, it is conditional call
	int conditionForCallMet = (state->ir.low == 5) ? 1 : isConditionSet(state);
#if DIAGNOSTIC_MODE == 1
	// CP/M system call emulation, the address of this system call is 0x0005
	if ((state->memory[state->pc + 2] << 8 | state->memory[state->pc + 1]) == 0x0005) {
		// The starting index of the message is in DE pair
		unsigned short int startOfMessage = state->d << 8 | state->e + 4;
		char *msg = state->memory + startOfMessage;
		while (*msg != '$') {
			printf("%c", *msg);
			msg++;
		}
		state->halted = 1;
		getch();
	}
#endif
	if (conditionForCallMet) {
		unsigned short int nextInstIndex = state->pc + 3;
		byte high8 = nextInstIndex >> 8;
		byte low8 = (byte)nextInstIndex;
		state->memory[(unsigned short int)(state->sp + (~1 + 1))] = high8;
		state->memory[(unsigned short int)(state->sp + (~2 + 1))] = low8;
		state->pc = state->memory[state->pc + 2] << 8 | state->memory[state->pc + 1];
		state->sp -= 2;
	} else {
		state->pc += 3;
	}
}

/*
Return instructions

Condition bits affected: NONE
*/

// Return from subroutine
void ret(State8080 *state) {
	// If lowest 3 bits of instruction register are 001, then it is non conditional return
	if (state->ir.low == 1 || isConditionSet(state)) {
		state->pc = state->memory[state->sp + 1] << 8 | state->memory[state->sp];
		state->sp += 2;
	} else {
		state->pc += 1;
	}
}


/*
Other instructions

Condition bits affected: NONE
*/

// No operation
void nop(State8080 *state) {
	state->pc++;
	getch();
}

// Call subroutine at addresss sepcified by EXP
void rst(State8080 *state) {

}

// Enable the interrupt system
void ei(State8080 *state) {

}

// Disable the interrupt system
void di(State8080 *state) {

}

// Read a byte from device EXP into the accumulator
void in(State8080 *state) {

}

// Send the accumulator contents to device EXP
void out(State8080 *state) {

}

// Halt processor execution
void hlt(State8080 *state) {
	state->halted = 1;
	state->pc++;
}

// -------- END OF OP CODE FUNCTIONS --------

void(*opcodeTable[256]) (State8080 *state) = {
 //  x0	  x1   x2	 x3    x4    x5   x6   x7   x8   x9    xA    xB    xC    xD    xE   xF
	nop, lxi, stax, inx,  inr,  dcr,  mvi, rlc, nop, dad,  ldax, dcx,  inr,  dcr,  mvi, rrc,  // 0x
	nop, lxi, stax, inx,  inr,  dcr,  mvi, ral, nop, dad,  ldax, dcx,  inr,  dcr,  mvi, rar,  // 1x
	nop, lxi, shld, inx,  inr,  dcr,  mvi, daa, nop, dad,  lhld, dcx,  inr,  dcr,  mvi, cma,  // 2x
	nop, lxi, sta,  inx,  inr,  dcr,  mvi, stc, nop, dad,  lda,  dcx,  inr,  dcr,  mvi, cmc,  // 3x
	mov, mov, mov,  mov,  mov,  mov,  mov, mov, mov, mov,  mov,  mov,  mov,  mov,  mov, mov,  // 4x
	mov, mov, mov,  mov,  mov,  mov,  mov, mov, mov, mov,  mov,  mov,  mov,  mov,  mov, mov,  // 5x
	mov, mov, mov,  mov,  mov,  mov,  mov, mov, mov, mov,  mov,  mov,  mov,  mov,  mov, mov,  // 6x
	mov, mov, mov,  mov,  mov,  mov,  hlt, mov, mov, mov,  mov,  mov,  mov,  mov,  mov, mov,  // 7x
	add, add, add,  add,  add,  add,  add, add, adc, adc,  adc,  adc,  adc,  adc,  adc, adc,  // 8x
	sub, sub, sub,  sub,  sub,  sub,  sub, sub, sbb, sbb,  sbb,  sbb,  sbb,  sbb,  sbb, sbb,  // 9x
	ana, ana, ana,  ana,  ana,  ana,  ana, ana, xra, xra,  xra,  xra,  xra,  xra,  xra, xra,  // Ax
	ora, ora, ora,  ora,  ora,  ora,  ora, ora, cmp, cmp,  cmp,  cmp,  cmp,  cmp,  cmp, cmp,  // Bx
	ret, pop, jmp,	jmp,  call, push, adi, rst, ret, ret,  jmp,  jmp,  call, call, aci, rst,  // Cx
	ret, pop, jmp,	out,  call, push, sui, rst, ret, ret,  jmp,  in,   call, call, sbi, rst,  // Dx
	ret, pop, jmp,	xthl, call, push, ani, rst, ret, pchl, jmp,  xchg, call, call, xri, rst,  // Ex
	ret, pop, jmp,	di,   call, push, ori, rst, ret, sphl, jmp,  ei,   call, call, cpi, rst,  // Fx
};


State8080 *init8080State(void) {
	byte *mem = calloc(1, sizeof(byte) * (2 << 15));
	State8080 *state = calloc(1, sizeof(State8080));
	state->memory = mem;
	return state;
}

void executeInst(State8080 *state) {
	// FETCH
	byte inst = state->memory[state->pc];
	// "DECODE" BITS INTO INSTRUCTION REGISTER BITMAP
	state->ir.high = 0;
	state->ir.middle = (inst & 56) >> 3; // 56 is mask 00111000
	state->ir.low = inst & 7; // 7 is mask 00000111
	// EXECUTE
	opcodeTable[inst](state);
}

unsigned short int getRstAddress(State8080 *state) {
	unsigned short int addr = state->ir.middle; // the 3 middle bits 00[bbb]000 of opcode refers to the start address of ISR
	return addr * 8; // just mutiply bbb by 8 since in decimal ISRs start (in decimal) at 0, 8, 16, 24..... 56
}

// Get the register pair referenced in instruction register as 16 bits
unsigned short int getRegisterPairByOpCode(State8080 *state, int getFlags) {
	int rp = state->ir.middle >> 1;
	switch (rp) {
	case 0:
		return state->b << 8 | state->c;
	case 1:
		return state->d << 8 | state->e;
	case 2:
		return state->h << 8 | state->l;
	case 3:
		if (getFlags) {
			// Do some bit shift hacking to format flags struct to byte of format [sz0c0p1c], then OR with accumulator
			unsigned short int flagsInt = state->flags.s << 7 | state->flags.z << 6 | state->flags.h << 4 
				| state->flags.p << 2 | 2 | state->flags.c;
			return state->a << 8 | flagsInt;
		} else {
			return state->sp;
		}
	}

}

void setRegisterPairByOpCode(State8080 *state, unsigned short int value, int setFlags) {
	int rp = state->ir.middle >> 1;
	byte reg1 = value >> 8;
	byte reg2 = (byte)value;
	switch (rp) {
	case 0:
		state->b = reg1;
		state->c = reg2;
		break;
	case 1:
		state->d = reg1;
		state->e = reg2;
		break;
	case 2:
		state->h = reg1;
		state->l = reg2;
		break;
	case 3:
		if (setFlags) {
			// Convert this byte back into C struct, byte format is [sz0c0p1c]
			// All the ints we & w/ are bit masks
			state->a = reg1;
			state->flags.s = reg2 > 127;
			state->flags.z = (reg2 & 64) >> 6;
			state->flags.h = (reg2 & 16) >> 4;
			state->flags.p = (reg2 & 4) >> 2;
			state->flags.c = (reg2 & 1);
		} else {
			state->sp = value;
		}
	}
}

byte* getOpCodeRegister(State8080 *state, int isDestination) {
	byte registerNum;	// the register number (octal) that is target / source for operation in IR
	byte* addressOfReg; // with the register octal from IR, we can grab its corresponding member in state structure
	if (isDestination) {
		registerNum = (byte)state->ir.middle;
	} else {
		registerNum = (byte)state->ir.low;
	}

	switch (registerNum) {
	case 0:
		addressOfReg = &state->b; break;
	case 1:
		addressOfReg = &state->c; break;
	case 2:
		addressOfReg = &state->d; break;
	case 3:
		addressOfReg = &state->e; break;
	case 4:
		addressOfReg = &state->h; break;
	case 5:
		addressOfReg = &state->l; break;
	case 6:	// this is for pseudo register M, which refers to HL pair
		addressOfReg = NULL; break; // this is a hack of just returning null for HL pair (we can't return a pair of addresses)
	case 7:
		addressOfReg = &state->a; break;
	}
	return addressOfReg;
}

byte isConditionSet(State8080 *state) {
	byte conditionNum = state->ir.middle;
	byte conditionVal;

	switch (conditionNum) {
	case 0:	// Not zero?
		conditionVal = (state->flags.z) ? 0 : 1; break;
	case 1: // Zero?
		conditionVal = state->flags.z; break;
	case 2: // Carry reset?
		conditionVal = (state->flags.c) ? 0 : 1; break;
	case 3: // Carry?
		conditionVal = state->flags.c; break;
	case 4: // Parity odd?
		conditionVal = (state->flags.p) ? 0 : 1; break;
	case 5: // Parity even?
		conditionVal = state->flags.p; break;
	case 6:	// Plus?
		conditionVal = (state->flags.s) ? 0 : 1; break;
	case 7: // Minus?
		conditionVal = state->flags.s; break;
	}
	return conditionVal;
}

byte getOperandFromRegister(State8080 *state) {
	byte *opcodeRegister = getOpCodeRegister(state, 0);
	byte operand;
	if (opcodeRegister == NULL) {
		unsigned short int addr = state->h << 8 | state->l;
		operand = state->memory[addr];
	} else {
		operand = *opcodeRegister;
	}
	return operand;
}

void updateFlags(State8080 *state, byte reg) {
	// invert the result from parity table
	// this is because 8080 definition of parity sets bit to 1 if even amt of 1s in accumulator byte
	state->flags.p = ParityTable256[reg] ^ 1;
	state->flags.z = reg == 0;
	state->flags.s = reg > 127;
}

void updateFlagsArithmetic(State8080 *state, unsigned short int result, int isSubtraction) {
	state->flags.c = (isSubtraction) ? result <= 0xFF : result > 0xFF;
	updateFlags(state, (byte) result);
}

void updateFlagsLogical(State8080 *state, unsigned short int result) {
	state->flags.c = 0; // carry is reset after logical operation
	updateFlags(state, (byte) result);
}

void loadFileIntoMemory(State8080 *state, char *name, unsigned short int offset) {
	FILE *fp = fopen(name, "rb");
	if (fp != NULL) {
		fseek(fp, 0, SEEK_END);
		long fsize = ftell(fp);
		rewind(fp);
		fread(state->memory + offset, sizeof(byte), fsize, fp);
		fclose(fp);
	} else {
		printf("Error reading in file!");
		state->halted = 1;
	}
}

void printGeneralRegisters(State8080 *state) {
	printf("A = %x  B = %x  C = %x  D = %x  E = %x  H = %x  L = %x\n",
		state->a, state->b, state->c, state->d, state->e, state->h, state->l);
}

void printStatusRegister(State8080 *state) {
	printf("Sign = %d  Zero = %d  Interrupt = %d  Half-Carry = %d  Parity = %d  Carry = %d\n",
		state->flags.s, state->flags.z, state->flags.i, state->flags.h, state->flags.p, state->flags.c);
}

void printInternalRegisters(State8080 *state) {
	printf("PC = %x		SP = %x		IR = %x\n", state->pc, state->sp, state->memory[state->pc]);
	printStatusRegister(state);
}

void printMemoryContents(State8080 *state, unsigned short int start) {
	printf("ADDR\tVALUE\n");
	for (unsigned short int i = 0; i < 20; i++) {
		printf("%x\t%x\n", i+start, state->memory[i+start]);
	}
}

void setUpCPUDiagTest(State8080 *state) {
	state->pc = 0x0100;
	loadFileIntoMemory(state, "C:\\Users\\Aidan\\Downloads\\cpudiag.bin", 0x100);
	state->memory[368] = 0x7;  
	state->memory[0x59c] = 0xc3; 
	state->memory[0x59d] = 0xc2;
	state->memory[0x59e] = 0x05;
}

int main() {
	State8080 *state = init8080State();
	setUpCPUDiagTest(state);
	while (!state->halted) {
		printInternalRegisters(state);
		printGeneralRegisters(state);
		printf("\n");
		executeInst(state);
	}
	getchar();

	return 0;
}