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
//12 DUP ADD
//13 load_hex_dump mit MOV R2,#0X7000 und MOV R1,#8
//14 KEY=0D,0A ignorieren und bei M Start des neuen Programms
//15 zusätzlich R1=7,6,5,4,3,2,1,0,r ausgeben
//16 add r3,r3,#0x0A mit r3=0 als Anfangsannahme
//17 "N" als Neustart schon in loop2 und "M" als return aus load_hex_dump
//18 Blinkanzahl gleich bei Programmstart setzen
//19 neues Programm auf 0x7000 speichern und dort dann 03a0f902 mov pc,#0X8000
//20 EMIT auch in loop2
//21 load_hex_dump nur noch mit Ausgabe "r"
//22 L=speichern, N=starten auf #0x7000, P=speichern, R=starten auf #0x8000, M=speichern zuende
//23 r2 aufheben
//24 Blinkdauer auch gleich am Anfang einstellen, auch Leer- und Sonderzeichen im Hexdump überlesen
//25 zurück auf 115200 baud
//26 ohne ldr r2,=..., und mit BL FFStart
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
mov r3,#0x0A //18 Blinkanzahl #0x0A gleich bei Programmstart setzen //22 #0x2A
mov r4,#0x0F0000 //24 Blinkdauer auch
mov r0,   #0x20000000 //26
add r0,r0,#0x00200000 //26
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
loop$: 
str r1,[r0,#40]
mov r2,r4 //24
wait1$:
	sub r2,#1
	cmp r2,#0
	bne wait1$
str r1,[r0,#28]
mov r2,r4 //24
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
//26 ldr r2,0x20215000 //AUXIRQ
mov r2,   #0x20000000 //26
add r2,r2,#0x00200000 //26
add r2,r2,#0x00015000 //26
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
//ldr r3,=#270      //#270 für 115200, #26040 für 1200
mov r3,   #0x100 //26 #270=#0x10e
add r3,r3,#0x00e //26
str r3,[r2,#0x68] //6 put32(AUX_MU_BAUD_REG,270); "Set baud rate to r3"
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
mov sp,#0x9000
mov r12,#0x9000

MOV   R0,#0X20 
STMEA R12!,{R0}// ( ' ' )
BL    EMIT     // ( )
MOV   R0,#0X48 
STMEA R12!,{R0}// ( 'H' )
BL    EMIT     // ( )
MOV   R0,#0X41
STMEA R12!,{R0}// ( 'A' )
BL    EMIT     // ( )
MOV   R0,#0X4C 
STMEA R12!,{R0}// ( 'L' )
BL    EMIT     // ( )
MOV   R0,#0X4C 
STMEA R12!,{R0}// ( 'L' )
BL    EMIT     // ( )
MOV   R0,#0X4F //22
STMEA R12!,{R0}//22 ( 'O' )
BL    EMIT     // ( )
MOV   R0,#0X0D 
STMEA R12!,{R0}// ( ' ' )
BL    EMIT     // ( )
MOV   R0,#0X0A 
STMEA R12!,{R0}// ( ' ' )
BL    EMIT     // ( )

loop2$:  //2
BL FFStart //26
loop2a$:
ldr   r5,[r2,#0x54] //6 abfragen, ob Daten da
and   r5,r5,#1      //6 Bit0
cmp   r5,#0         //7
beq loop2a$          //7 zurück wenn keine Daten da
ldr   r6,[r2,#0x40] //7 Zeichen lesen
add   r7,r7,#1    //7 ist ein Zeichen mehr
and   r5,r7,#1    //7 Bit0 der Anzahl
cmp   r5,#0       //3
strne r1,[r0,#40] //3 in GPIO2 ausgeben, LED an 3,3V 
streq r1,[r0,#28] //3
str   r6,[r2,#0x40] //7 Zeichen zurückschicken
mov   r4,r2          //23 r2 aufheben
CMP   R6,#0X4E       //17 if "N"...
MOVEQ PC,#0X7000     //17 bei "N" Neustart //19 jetzt ab 0X7000
CMP   R6,#0X52       //22 if "R"...
MOVEQ PC,#0X8000     //22 bei "R" Neustart ab 0X8000
//str   r6,[r2,#0x40] //7 Zeichen zurückschicken //20 jetzt mit EMIT
mov   r5,#0x4C
STMEA R12!,{R5}// ( "L" )
BL    EMIT     // ( )
cmp   r6,#0x4C      //11 wenn L...
MOVEQ R2,#0X7000    //22//13//18 direkt Programmstart überschreiben //19 jetzt ab 0X7000
bleq  load_hex_dump //11 wenn L, dann ein hex dump laden
cmp   r6,#0x50      //22 wenn P...
MOVEQ R2,#0X8000    //22 bei P ab 0X8000
bleq  load_hex_dump //22 wenn P hex dump laden
mov   r2,r4         //23 r2 wieder zurück
b loop2$ //2 Ende Versuch 2


load_hex_dump:
STMFD SP!,{R0-R7,LR}
load_hex_dump_0:
MOV   R1,#8
MOV   R0,#0    // m=0
STMEA R12!,{R0}// ( m )
load_hex_dump_1:
//21 MOV   R0,#0X0D 
//21 STMEA R12!,{R0}// ( m CR )
//21 BL    EMIT     // ( )
//21 MOV   R0,#0X0A 
//21 STMEA R12!,{R0}// ( m LF )
//21 BL    EMIT     // ( )
//21 MOV   R0,#0X50
//21 STMEA R12!,{R0}// ( m "P" )
//21 BL    EMIT     // ( )
load_hex_dump_2:
BL    KEY      // ( m c )
//21 BL    DUP      // ( m c c )
//21 BL    EMIT     // ( m c )
LDMEA R12,{R0} // ( m c )
CMP   R0,#0X30       //14//23
BLT load_hex_dump_2  //14
CMP   R0,#0X4D       //14 if "M"
BEQ load_hex_dump_3  //17
BL char_to_int // ( m n )
BL mul_16_add  // ( 16*m+n )
SUB   R1,R1,#1    //15
ADD   R0,R1,#0X30 //15
//21 STMEA R12!,{R0}   //15 ( m' "7" )
//21 BL    EMIT        //15 ( m' )
CMP   R1,#0
BNE   load_hex_dump_1//( m' )
LDMEA R12!,{R0}// ( )
STMEA R2!,{R0} //
MOV   R0,#0X73
STMEA R12!,{R0}// ( "r" )
BL    EMIT     // ( )
B load_hex_dump_0
load_hex_dump_3:
MOV   R0,#0X20 
STMEA R12!,{R0}// ( ' ' )
BL    EMIT     // ( )
MOV   R0,#0X45 
STMEA R12!,{R0}// ( 'E' )
BL    EMIT     // ( )
MOV   R0,#0X4E
STMEA R12!,{R0}// ( 'N' )
BL    EMIT     // ( )
MOV   R0,#0X44 
STMEA R12!,{R0}// ( 'D' )
BL    EMIT     // ( )
MOV   R0,#0X45 
STMEA R12!,{R0}// ( 'E' )
BL    EMIT     // ( )
MOV   R0,#0X0D 
STMEA R12!,{R0}// ( ' ' )
BL    EMIT     // ( )
MOV   R0,#0X0A 
STMEA R12!,{R0}// ( ' ' )
BL    EMIT     // ( )
LDMEA R12!,{R0,R1} //17 ( )
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
LDMFD SP!,{R0,PC}

mul_16_add: //10// ( m n --> 16*m+n )
STMFD SP!,{R0,R1,LR}
LDMEA R12!,{R0,R1}
LSL   R0,R0,#4
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

DUP: //12// ( n --> n n )
STMFD SP!,{R0,LR}
LDMEA R12!,{R0}
STMEA R12!,{R0}
STMEA R12!,{R0}
LDMFD SP!,{R0,PC}

ADD: //12// ( a b --> a+b )
STMFD SP!,{R0,R1,LR}
LDMEA R12!,{R0,R1}
ADD   R0,R0,R1
STMEA R12!,{R0}
LDMFD SP!,{R0,R1,PC}

ab7000:
mov r3,#0X2A
mov r1,#0X8000
add r1,r1,#4
mov pc,r1

FFDecode: //26 zu BL FFStart
STMFD SP!,{R0-R7,LR}
MOV   R0,#0X20 
STMEA R12!,{R0}// ( ' ' )
BL    EMIT     // ( )
MOV   R0,#0X46 
STMEA R12!,{R0}// ( 'F' )
BL    EMIT     // ( )
MOV   R0,#0X4F //
STMEA R12!,{R0}// ( 'O' )
BL    EMIT     // ( )
LDMFD SP!,{R0-R7,PC}
FFStart:
STMFD SP!,{R0-R7,LR}
BL FFDecode
LDMFD SP!,{R0-R7,PC}
FFCode:
.word 0x12345670
Daten:
