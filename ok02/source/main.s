//0 geändert:
//1 GPIO16 ersetzt durch GPIO4
//2 nur einige Wiederholungen
//3 GPIO15 in GPIO4 ausgeben
//4 und auch in GPIO14 ausgeben
//5 als nächstes GPIO4 auf GPIO2
//6 GPIO14 und GPIO15 als UART1
//7 GPIO15 wieder an GPIO14 zurückschicken
//9 char_to_int
//10 mul_16_add
//11 load_hex_dump KEY EMIT
/******************************************************************************
*	main.s
*	 by Alex Chadwick
*
*	A sample assembly code implementation of the ok02 operating system, that 
*	simply turns the OK LED on and off repeatedly.
*	Changes since OK01 are marked with NEW.
******************************************************************************/

/*
* .section is a directive to our assembler telling it to place this code first.
* .globl is a directive to our assembler, that tells it to export this symbol
* to the elf file. Convention dictates that the symbol _start is used for the 
* entry point, so this all has the net effect of setting the entry point here.
* Ultimately, this is useless as the elf itself is not used in the final 
* result, and so the entry point really doesn't matter, but it aids clarity,
* allows simulators to run the elf, and also stops us getting a linker warning
* about having no entry point. 
*/
.section .init
.globl _start
_start:

/* 
* This command loads the physical address of the GPIO region into r0.0x20200000
*/
ldr r0,=0x20200000

/*
* Our register use is as follows:
* r0=0x20200000	the address of the GPIO region.
* r1=0x00040000	a number with bits 18-20 set to 001 to put into the GPIO
*				function select to enable output to GPIO 16. 
* then
* r1=0x00010000	a number with bit 16 high, so we can communicate with GPIO 16.
* r2=0x003F0000 a number that will take a noticeable duration for the processor 
*				to decrement to 0, allowing us to create a delay.
*/

//6 GPIO2 auf Funktion OUT einstellen
mov r1,#1      //6 Funktion OUT
lsl r1,#6      //6 GPIO2
str r1,[r0,#0] //6 Funktion auswählen

//6 GPIO2 paarmal schalten
mov r1,#1
lsl r1,#2 //6 Bitposition für  GPIO2
mov r3,#0x0A //2 Anzahl Wiederholungen
loop$: 
str r1,[r0,#40]
mov r2,#0x0F0000 //6 noch etwas schneller
wait1$:
	sub r2,#1
	cmp r2,#0
	bne wait1$
str r1,[r0,#28]
mov r2,#0x0F0000 //6 noch etwas schneller
wait2$:
	sub r2,#1
	cmp r2,#0
	bne wait2$
sub r3,#1 //2 nur wenige Wiederholungen
cmp r3,#0 //2
bne loop$ //2


//6 GPIO14 und GPIO15 auf Funktion UART1 einstellen
mov r1,#2      //6 ALT5
lsl r1,#12     //6 nach GPIO14 schieben
mov r2,#2      //6 nochmal ALT5
lsl r2,#15     //6 nach GPIO15 schieben
orr r1,r1,r2   //6 beide zusammenfassen 
str r1,[r0,#4] //6 und Funktion auswählen


//6 hierher kommt noch Pull up für GPIO14 und GPIO15

//6 UART1 Adressen:
ldr r2,=0x20215000 //AUXIRQ
/*6
#define AUX_ENABLES     (PBASE+0x00215004)
#define AUX_MU_IO_REG   (PBASE+0x00215040)
#define AUX_MU_IER_REG  (PBASE+0x00215044)
#define AUX_MU_IIR_REG  (PBASE+0x00215048)
#define AUX_MU_LCR_REG  (PBASE+0x0021504C)
#define AUX_MU_MCR_REG  (PBASE+0x00215050)
#define AUX_MU_LSR_REG  (PBASE+0x00215054)
#define AUX_MU_MSR_REG  (PBASE+0x00215058)
#define AUX_MU_SCRATCH  (PBASE+0x0021505C)
#define AUX_MU_CNTL_REG (PBASE+0x00215060)
#define AUX_MU_STAT_REG (PBASE+0x00215064)
#define AUX_MU_BAUD_REG (PBASE+0x00215068)
6*/

//6 UART1 konfigurieren (Baudrate und dies und das)
mov r3,#1
str r3,[r2,#0x04] //6 put32(AUX_ENABLES,1);    "Enable mini uart"
mov r3,#0
str r3,[r2,#0x60] //6 put32(AUX_MU_CNTL_REG,0); "Disable flow control and TX and RX"
mov r3,#0
str r3,[r2,#0x44] //6 put32(AUX_MU_IER_REG,0);  "Disable receive and transmit interrupts"
mov r3,#3         //6 3 ist nicht ok
str r3,[r2,#0x4C] //6 put32(AUX_MU_LCR_REG,3);  "Enable 8 bit mode"
mov r3,#0
str r3,[r2,#0x50] //6 put32(AUX_MU_MCR_REG,0);   "Set RTS line to be always high"
ldr r3,=#26040 //#270 für 115200, #26040 für 1200
str r3,[r2,#0x68] //6 put32(AUX_MU_BAUD_REG,270); "Set baud rate to 115200"
mov r3,#3
str r3,[r2,#0x60] //6 put32(AUX_MU_CNTL_REG,3);   "Finally, enable transmitter and receiver"


//6 hier nun RXD lesen und auf GPIO2 und TXD ausgeben
/*6
void uart_send ( char c )
{
    while(1) {
        if(get32(AUX_MU_LSR_REG)&0x20) 
            break;
    }
    put32(AUX_MU_IO_REG,c);
}

char uart_recv ( void )
{
    while(1) {
        if(get32(AUX_MU_LSR_REG)&0x01) 
            break;
    }
    return(get32(AUX_MU_IO_REG)&0xFF);
}

6*/

mov r1,#1
lsl r1,#2 //6 Bitposition für GPIO2
mov r7,#0 //7 Anzahl der empfangenen Zeichen

loop2$:  //2
ldr   r5,[r2,#0x54] //6 abfragen, ob Daten da
and   r5,r5,#1      //6 Bit0
cmp   r5,#0         //7
beq loop2$          //7 zurück wenn keine Daten da
ldr   r6,[r2,#0x40] //7 Zeichen lesen
add   r7,r7,#1    //7 ist ein Zeichen mehr
and   r5,r7,#1    //7 Bit0 der Anzahl
cmp   r5,#0       //3
strne r1,[r0,#40] //3 in GPIO2 ausgeben, LED an 3,3V 
streq r1,[r0,#28] //3
str   r6,[r2,#0x40] //7 Zeichen zurückschicken
cmp   r6,#0x4C      //11 wenn L,
mov   r6,#0x4C
str   r6,[r2,#0x40] //7 Zeichen zurückschicken
bleq  load_hex_dump //11 dann ein hex dump laden
b loop2$ //2 Ende Versuch 2


load_hex_dump:
mov sp,#0x9000
mov r12,#0x9000
STMFD SP!,{R0-R7,LR}
MOV   R0,#0X0D 
STMEA R12!,{R0}// ( CR )
BL    EMIT     // ( )
MOV   R0,#0X0A 
STMEA R12!,{R0}// ( LF )
BL    EMIT     // ( )
MOV   R0,#0X50 
STMEA R12!,{R0}// ( P )
BL    EMIT     // ( )
BL    KEY      // ( c )
BL    EMIT     // ( )
LDMFD SP!,{R0-R7,PC}


char_to_int: //9// ( c --> n )
STMFD SP!,{R0,LR} //A2-8
LDMEA R12!,{R0}
CMP   R0,#0X60    //a,b,c,d,e,f
SUBGT R0,R0,#0X20
CMP   R0,#0X40    //A,B,C,D,E,F
SUBGT R0,R0,#0X07
SUB   R0,R0,#0X30 //0,1,2,3,4,5,6,7,8,9
STMEA R12!,{R0}
LDMFD LR!,{R0,PC}

mul_16_add: //10// ( m n --> 16*m+n )
STMFD SP!,{R0,R1,LR}
LDMEA R12!,{R0,R1}
LSL   R0,R0,#8
ADD   R0,R0,R1
STMEA R12!,{R0}
LDMFD SP!,{R0,R1,PC}

EMIT: //11// ( c --> )
STMFD SP!,{R0-R7,LR}
LDMEA R12!,{R0}
MOV   R8,#0X20000000     //12 PBASE
ADD   R8,R8,#0X200000    //12 GPIO_
ADD   R8,R8,#0X15000     //12 AUXIRQ
emitloop1$:
ldr   r5,[r8,#0x54]
and   r5,r5,#0x20
cmp   r5,#0
beq   emitloop1$
str   r0,[r8,#0x40]
//STMEA R12!,{}
LDMFD SP!,{R0-R7,PC}

KEY: //11// ( --> c)
STMFD SP!,{R0-R7,LR}
//LDMEA R12!,{R0}
MOV   R8,#0X20000000     //12 PBASE
ADD   R8,R8,#0X200000    //12 GPIO_
ADD   R8,R8,#0X15000     //12 AUXIRQ
keyloop1$:
ldr   r5,[r8,#0x54]
and   r5,r5,#1
cmp   r5,#0
beq   keyloop1$
ldr   r5,[r8,#0x40]
STMEA R12!,{R5}
LDMFD SP!,{R0-R7,PC}
