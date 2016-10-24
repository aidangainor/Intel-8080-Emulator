/*
We assume char type is 8 bits. It can be larger, but this just means a little wasted space. No issue.
Same logic applies for hopefully 16 bit unsigned shorts, they can be larger but wasted space is not an issue.
*/

typedef unsigned char byte;

typedef struct {
	byte s	: 1; // sign flag
	byte z  : 1; // zero flag
	byte i  : 1; // interrupt flag
	byte h  : 1; // half-carry flag
	byte p  : 1; // parity flag
	byte c  : 1; // carry flag
	byte	: 2; // padding (2 bits)
} StatusRegister;

typedef struct {
	byte high	: 2; // highest bits, this probably won't be used for decoding
	byte middle : 3; // middle octal, will be used for decoding register src / dest
	byte low	: 3; // lower octal,
} InstructionRegister;

typedef struct {
	byte a;	// 8 bit accumulator
	byte b;	// all other 8 bit registers below
	byte c;
	byte d;
	byte e;
	byte h;
	byte l;
	byte halted; // used for HLT instruction, when > 0 fetch/decode/execute cycle is stopped
	unsigned short int pc;	// 16bit program counter
	unsigned short int sp;	// 16bit stack pointer
	InstructionRegister ir; // 8 bit instruction register
	StatusRegister flags;	// 8 bit status register
	byte *memory; // byte addressable memory
} State8080;