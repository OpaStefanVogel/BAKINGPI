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
//27 sp=E000, L=4000, RAM0000 und RAM3000
//28 RAMB0000 nach 0000 verschieben, RAMB3000 nach 3000
//29 S für einen Step mit CR MDOT MDOT und PC=PC+2
//30 STEP4 Axxx
//31 STEPA A003
//32 STEP0123 und SPDOT
//33 DUP
//34 9xxx EMITCODE FETCH STORE bis Ausgabe "F"
//35 A007, ADD, 9xxx mit LDRH, B200, geht bis vor 036D CRDP @ 
//36 Fxxx und LDRH wieder als LDMEA
//37 RAMB2F00
//38 R8 als Zähler, A000 MINUS, A00D 0= A00B NOT A00E OR A008 AND, bis vor QUIT
//39 r13=C000, R10=RP=R12=SP=C000, alle Bxxx jetzt drin
//40 A009 ab adr=2C00 bis 2FFF nach 5800...
//41 8xxx und 9xxx neu mit CMP R0,#0800
//42 R9 = Breakpoint Schrittzähler
//43 RAM3C00Start und 0LT
//44 2802 @ 2802 ! 3000 @ 3000 !
//45 RP=R10=6000 statt C000
//46 3xxx ! mit R0 und 3000 @ mit adr=R1
//47 LFA=07B3
//48 in @ vorläufig LSL 16 LSR 16
//49 einmal TIB @ 100 EXPECT FIND EXECUTE durch
//50 U* für DP @ M.
//51 2800 @ über Speicherplatz 8, Taste # für Break
//52 in @ Vorzeichenbits auffüllen
//53 0 FFFF 10 U* muss FFF0 000F sein, dann geht HERE 0 0 DUMPZ richtig
//54 Tasten L und N zeigen auf Adresse E000
//55 2801 @ und 2801 ! damit SP? funktioniert in ;
//56 R9=0 und mit "#" anhalten und auch fortsetzen
//57 77 88 1FFF FFFC 2! 1FFF FFFC 2@ M. M.
//58 GPIOBASE R0 nochmal neu setzen
//59 auf 9600 baud zurück und 40 Puffer, dann geht INIT.xml durch
//60 STEPVECTOR
//61 STEPAVECTOR
//62 KEYCODE liest ab 0#010000
//63 Ersatzcode A004 anstelle von 44AC KEYCODE2
//64 wieder vor auf 115200 baud
//65 Keyspeicher ab 10000000, RAM3000 auf 6000, RAM3C00 raus
//66 RAMB0000 auf 10000, RAM3000 auf 16000
//67 Start ohne Taste "S", Break mit ^B
//68 FIQ Versuch 1
//69 enable FIQ in CPSR
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
//60 ldr r3,#loop2$
mov r3,#0x03 //18 Blinkanzahl #0x0A gleich bei Programmstart setzen //22 #0x2A
mov r4,#0x070000 //24 Blinkdauer auch
mov r0,   #0x20000000 //26//58 nochmal in loop2a
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
mov r3,#2 //68 2 statt 0
str r3,[r2,#0x44] //6 put32(AUX_MU_IER_REG,0);  "Disable receive and transmit interrupts"
mov r3,#3         //6 3 ist nicht ok
str r3,[r2,#0x4C] //6 put32(AUX_MU_LCR_REG,3);  "Enable 8 bit mode"
mov r3,#0
str r3,[r2,#0x50] //6 put32(AUX_MU_MCR_REG,0);   "Set RTS line to be always high"
//ldr r3,=#270      //#270 für 115200, #26040 für 1200, 
mov r3,   #0x100 //59//26 #270=#0x10e für 115200, 0xCB7 für 9600
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
mov r13,#0xE000  //27
mov r12,#0xC000 //27 C000 wird auch nochmal in //32 verwendet

//MOV   R0,#0X20 
//STMEA R12!,{R0}// ( ' ' )
//BL    EMIT     // ( )

loop2$:  //2
//BL loop2a$ //27
//BL loop2a$ //27
BL FFStart //26
BL RAM2F00Start //37
BL RAM3000Start //27 eb0004bd
//BL RAM3C00Start //65//43 52415453
BL RAM0000Start //68
MOV R11,#0X10000 //66//29 R11=PC, R12=SP
MOV R10,#0X16000 //30//45 R10=RP, R11=PC, R12=SP
MOV R8,#0X0     //38 Schrittzähler
MOV R9,#0X000000  //56//42//43 Breakpoint Schrittzähler
ADD R9,R9,#0X000  //56//42//43 Breakpoint Schrittzähler
MOV R6,#0X53
mov   r4,r2          //23 r2 aufheben
B   loop2b$

loop2a$:
ldr   r5,[r2,#0x54] //6 abfragen, ob Daten da
and   r5,r5,#1      //6 Bit0
cmp   r5,#0         //7
beq loop2a$          //7 zurück wenn keine Daten da
ldr   r6,[r2,#0x40] //7 Zeichen lesen
add   r7,r7,#1    //7 ist ein Zeichen mehr
and   r5,r7,#1    //7 Bit0 der Anzahl
cmp   r5,#0       //3
mov r0,   #0x20000000 //58
add r0,r0,#0x00200000 //58
strne r1,[r0,#40] //3 in GPIO2 ausgeben, LED an 3,3V 
streq r1,[r0,#28] //3
str   r6,[r2,#0x40] //7 Zeichen zurückschicken
loop2b$:
CMP   R6,#0X4E       //17 if "N"...
MOVEQ PC,#0XE000     //17 bei "N" Neustart //27 jetzt ab 0XE000
CMP   R6,#0X52       //22 if "R"...
MOVEQ PC,#0X8000     //22 bei "R" Neustart ab 0X8000
CMP   R6,#0X53       //29 if "S"... dann "STEP"
BLEQ  STEP           //29 ( )
CMP   R6,#0X23       //56 if "#"... dann fortsetzen
MOVEQ R9,#0          //56 ( )
BLEQ  STEP           //56 ( )
//str   r6,[r2,#0x40] //7 Zeichen zurückschicken //20 jetzt mit EMIT
mov   r5,#0x4C
STMEA R12!,{R5}// ( "L" )
BL    EMIT     // ( )
cmp   r6,#0x4C      //11 wenn L...
MOVEQ R2,#0XE000    //22//13//18 direkt Programmstart überschreiben //27 jetzt ab 0X4000
bleq  load_hex_dump //11 wenn L, dann ein hex dump laden
cmp   r6,#0x50      //22 wenn P...
MOVEQ R2,#0X8000    //22 bei P ab 0X8000
bleq  load_hex_dump //22 wenn P hex dump laden
mov   r2,r4         //23 r2 wieder zurück
b loop2a$ //2 Ende Versuch 2


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
AND   R0,R0,#0XFF
MOV   R7,#0X20000000     //12 PBASE
ADD   R7,R7,#0X200000    //12 GPIO_
ADD   R7,R7,#0X15000     //12 AUXIRQ
emitloop1$:
ldr   r5,[r7,#0x54]
and   r5,r5,#0x20
cmp   r5,#0
beq   emitloop1$
str   r0,[r7,#0x40]
//STMEA R12!,{}
LDMFD SP!,{R0-R7,PC}

KEY: //11// ( --> c)
STMFD SP!,{R0-R8,LR}
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
LDMFD SP!,{R0-R8,PC}

KEYFLAG: //51// ( --> flag )
STMFD SP!,{R0-R8,LR}
//LDMEA R12!,{R0}
MOV   R8,#0X20000000     //12 PBASE
ADD   R8,R8,#0X200000    //12 GPIO_
ADD   R8,R8,#0X15000     //12 AUXIRQ
ldr   r5,[r8,#0x54]
and   r5,r5,#1
STMEA R12!,{R5}
LDMFD SP!,{R0-R8,PC}

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

MOVE: //28 ( von nach bytes --> )
STMFD SP!,{R0-R3,LR}
LDMEA R12!,{R0,R1,R2}
MOVE1:
LDR   R3,[R0]
STR   R3,[R1]
ADD   R0,#4
ADD   R1,#4
SUB   R2,#4
CMP   R2,#0
BNE   MOVE1
LDMFD SP!,{R0-R3,PC}

CR: //29 ( --> )
STMFD SP!,{R0,LR}
MOV   R0,#0X0D 
STMEA R12!,{R0}// ( ' ' )
BL    EMIT     // ( )
MOV   R0,#0X0A 
STMEA R12!,{R0}// ( ' ' )
BL    EMIT     // ( )
LDMFD SP!,{R0,PC}

MDOT: //29 ( w -->  <hex>)
STMFD SP!,{R0-R3,LR}
LDMEA R12!,{R0}
LSR   R1,R0,#0X0C
STMEA R12!,{R1}
BL    DIGIT
BL    EMIT
LSR   R1,R0,#0X08
STMEA R12!,{R1}
BL    DIGIT
BL    EMIT
LSR   R1,R0,#0X04
STMEA R12!,{R1}
BL    DIGIT
BL    EMIT
LSR   R1,R0,#0X00
STMEA R12!,{R1}
BL    DIGIT
BL    EMIT
MOV   R0,#0X20 
STMEA R12!,{R0}// ( ' ' )
BL    EMIT     // ( )
LDMFD SP!,{R0-R3,PC}

DIGIT: //29 ( b --> c )
STMFD SP!,{R0,LR}
LDMEA R12!,{R0}
AND   R0,R0,#0X0F
CMP   R0,#0X0A
ADDGE R0,R0,#7
ADD   R0,R0,#0X30
STMEA R12!,{R0}
LDMFD SP!,{R0,PC}

SPDOT: //32 ( -->  <hex> <hex> <hex>...)
STMFD SP!,{R0-R3,LR}
MOV   R2,R12
MOV   R1,#0XC000 //RP0
SPDOT1:
CMP   R1,R2
BEQ   SPDOT9
LDR   R0,[R1]
STMEA R12!,{R0}
BL    MDOT
ADD   R1,R1,#4
B     SPDOT1
SPDOT9:
LDMFD SP!,{R0-R3,PC}


STEP: //29 ( --> )
STMFD SP!,{R0-R7,LR}
STEPR:
LDRH  R0,[R11]
ADD   R8,R8,#1
ADD   R11,R11,#2
AND   R1,R0,#0XF000 //30
LSR   R2,R1,#10
MOV   R3,PC
ADD   PC,R3,R2

STEPVECTOR:
B     STEP0123
B     STEP0123
B     STEP0123
B     STEP0123
B     STEP4
B     STEP5
B     STEP6
B     STEP7
B     STEP8
B     STEP9
B     STEPA
B     STEPB
B     STEPF
B     STEPF
B     STEPF
B     STEPF

STEP4: //30 4xxx
SUB   R11,R11,#0X10000 //66
LSR   R2,R11,#1
STRH  R2,[R10,#-2]! //45
SUB   R11,R0,#0X4000
ADD   R11,R11,R11
ADD   R11,R11,#0X10000 //66
B     STEPEND

STEP5:
SUB   R11,R11,#0X10000 //66
LSR   R2,R11,#1
STRH  R2,[R10,#-2]! //45
SUB   R11,R0,#0X4000
ADD   R11,R11,R11
ADD   R11,R11,#0X10000 //66
B     STEPEND

STEP6:
STEP7:

STEP8: //30 8xxx
SUB   R0,R0,#0X8000
CMP   R0,#0X0800
MVNGE R1,#0
LSLGE R1,#12
ADDGE R0,R0,R1
ADD   R0,R0,R0
ADD   R11,R11,R0
B     STEPEND

STEP9: //30 4xxx
LDMEA R12!,{R2}
//LDRH  R2,[R12,#-4]!
CMP   R2,#0
BNE   STEPEND
SUB   R0,R0,#0X9000
CMP   R0,#0X0800
MVNGE R1,#0
LSLGE R1,#12
ADDGE R0,R0,R1
ADD   R0,R0,R0
ADD   R11,R11,R0
B     STEPEND

STEPA:
AND   R1,R0,#0X00FF
LSL   R2,R1,#2
MOV   R3,PC
ADD   PC,R3,R2

STEPAVECTOR:
B     STEPA000
B     STEPA001
B     STEPA002
B     STEPA003
B     STEPA004
B     STEPA005
B     STEPA006
B     STEPA007
B     STEPA008
B     STEPA009
B     STEPA00A
B     STEPA00B
B     STEPA00C
B     STEPA00D
B     STEPA00E
B     STEPA00F
B     STEPA000
B     STEPA001
B     STEPA002
B     STEPA003
B     STEPA004
B     STEPA005
B     STEPA006
B     STEPA007
B     STEPA008
B     STEPA009
B     STEPA00A
B     STEPA00B
B     STEPA00C
B     STEPA00D
B     STEPA00E
B     STEPA00F
B     STEPA000
B     STEPA001
B     STEPA002
B     STEPA003
B     STEPA004
B     STEPA005
B     STEPA006
B     STEPA007
B     STEPA008
B     STEPA029
B     STEPA02A
B     STEPA00B
B     STEPA00C
B     STEPA00D
B     STEPA00E
B     STEPA00F

STEPA000:
//0A 000 MLIT MCODE MINUS
LDMEA R12!,{R0} // ( a --> )
SUB   R0,R1,R0
STMEA R12!,{R0} // ( --> -a )
B     STEPEND

STEPA001:
//0A 001 MLIT MCODE FIQ
    mrs r0,cpsr       //69 Enable FIQ https://github.com/dwelch67/raspberrypi/blinker08
    bic r0,r0,#0x40   //69
    msr cpsr_c,r0     //69
B     STEPEND

STEPA004:
//0A 004 MLIT MCODE KX: //63// ( --> c flag )
MOV   R2,#0X10000
ADD   R2,R2,#4
LDR   R0,[R2]
ADD   R2,R2,#8
LDR   R1,[R2]
CMP   R0,R1
BEQ   KEYCODE28
ADD   R1,R1,#0X10000000 //65//62
LDRB  R0,[R1]
ADD   R1,R1,#1
SUB   R1,R1,#0X10000000 //65//62
STR   R1,[R2]
STMEA R12!,{R0}
MOV   R0,#0
SUB   R0,R0,#1
STMEA R12!,{R0}
B     STEPEND
KEYCODE28:
MOV   R0,#0
STMEA R12!,{R0}
STMEA R12!,{R0}
B     STEPEND

STEPA006:

STEPA00C:
//0A 001 MLIT MCODE U+
B     STEPEND

STEPA002:
//0A 002 MLIT MCODE U*
LDMEA R12!,{R0,R1,R2} // ( c b a --> (a*b+c)h (a*b+c)l )
MOV   R4,#0X10000
SUB   R4,R4,#1
AND   R0,R0,R4
AND   R1,R1,R4
AND   R2,R2,R4
MLA   R3,R2,R1,R0
LSR   R2,R3,#16
STMEA R12!,{R2} // ( --> (a*b+c)h )
AND   R3,R3,R4
STMEA R12!,{R3} // ( --> (a*b+c)h (a*b+c)l )
B     STEPEND

STEPA00D:
//0A 00D MLIT MCODE 0=
LDMEA R12!,{R0} // ( a --> )
CMP   R0,#0
MVNEQ R0,#0
MOVNE R0,#0
STMEA R12!,{R0} // ( --> 0= )
B     STEPEND

STEPA00F:
//0A 00F MLIT MCODE 0LT
LDMEA R12!,{R0} // ( a --> )
CMP   R0,#0
MVNLT R0,#0
MOVGE R0,#0
STMEA R12!,{R0} // ( --> 0LT )
B     STEPEND

STEPA005:
//0A 005 MLIT MCODE EMITCODE
BL    EMIT        //34 ( c -->  <char> )
B     STEPEND

STEPA00B:
//0A 00B MLIT MCODE NOT
LDMEA R12!,{R0} // ( a --> )
MVN   R0,R0
STMEA R12!,{R0} // ( --> not(a) )
B     STEPEND

STEPA008:
//0A 008 MLIT MCODE AND
LDMEA R12!,{R0,R1} // ( a b --> )
AND   R0,R0,R1
STMEA R12!,{R0} // ( --> a_and_b )
B     STEPEND

STEPA00E:
//0A 00E MLIT MCODE OR
LDMEA R12!,{R0,R1} //( a b --> )
ORR   R0,R0,R1
STMEA R12!,{R0} // ( --> a_or_b )
B     STEPEND

STEPA007:
//0A 007 MLIT MCODE M+
LDMEA R12!,{R0,R1} //35 ( a b --> )
ADD   R0,R0,R1
STMEA R12!,{R0} //35 ( --> a+b )
B     STEPEND

STEPA009:
//0A 009 MLIT MCODE !
LDMEA R12!,{R0,R1} //34 ( n adr --> )
AND   R2,R1,#0XFF00
CMP   R2,#0X2800
BNE   STORE2
AND   R1,R1,#0XFF
CMP   R1,#3
LSLEQ R0,R0,#1
ADDEQ R11,R0,#0X10000 //66
BEQ   STEPEND
CMP   R1,#2
LSLEQ R0,R0,#1 //45
ADDEQ R10,R0,#0X10000 //66
BEQ   STEPEND
CMP   R1,#1
LSLEQ R12,R0,#2 //55
BEQ   STEPEND
CMP   R1,#0
BEQ   STEPEND
CMP   R1,#9
MOV   R9,R0
B     STEPEND
STORE2:
CMP   R1,#0X3000 //43
ADDGE R1,#0X13000 //65
STRGEB  R0,[R1]
BGE     STEPEND
ADD   R1,R1,R1
ADD   R1,R1,#0X10000 //66
STRH  R0,[R1]
B     STEPEND

STEPA00A:
//0A 00A MLIT MCODE @
LDMEA R12!,{R1} //34 ( adr --> )
LSL   R1,R1,#16 //48
LSR   R1,R1,#16 //48
AND   R2,R1,#0XFF00
CMP   R2,#0X2800
BEQ   FETCH2
CMP   R1,#0X3000
ADDGE R1,#0X13000  //65
LDRGEB  R3,[R1]
STMGEEA R12!,{R3} //34 ( --> n )
BGE     STEPEND
ADD   R1,R1,R1
ADD   R1,R1,#0X10000 //66
LDRH  R3,[R1]
AND   R1,R3,#0X8000  //52 Vorzeichenbits
CMP   R1,#0
ADDNE R3,#0XFF0000
ADDNE R3,#0XFF000000
STMEA R12!,{R3} //34 ( --> n )
B     STEPEND
FETCH2: //44
AND   R1,R1,#0XFF
CMP   R1,#3
SUB   R2,R11,#0X10000 //66
LSREQ R0,R2,#1
STMEQEA R12!,{R0} //44 ( --> n )
BEQ   STEPEND
CMP   R1,#2
SUB   R2,R10,#0X10000 //66
LSREQ R0,R2,#1
STMEQEA R12!,{R0} //44 ( --> n )
BEQ   STEPEND
CMP   R1,#1
LSREQ R0,R12,#2
STMEQEA R12!,{R0} //55 ( --> n )
BEQ   STEPEND
CMP   R1,#0       //51
//MOV   R1,#8
//LDRB  R0,[R1]
MOV   R2,#0X10000
ADD   R2,R2,#0X0C
LDR   R1,[R2]
ADD   R1,R1,#0X10000000 //65//62
LDRB  R0,[R1]
ADD   R1,R1,#1
SUB   R1,R1,#0X10000000 //65//62
STR   R1,[R2]
STMEQEA R12!,{R0} //51 ( --> c )
B     STEPEND

STEPA029:
//0A 029 MLIT MCODE 2!
LDMEA R12!,{R0-R3} //57 ( a b adrh adrl --> )
LSL   R4,R2,#16
LSL   R3,R3,#16
LSR   R3,R3,#16
ADD   R4,R4,R3
LSL   R5,R0,#16
LSL   R1,R1,#16
LSR   R1,R1,#16
ADD   R5,R5,R1
STR   R5,[R4]
B     STEPEND

STEPA02A:
//0A 02A MLIT MCODE 2@
LDMEA R12!,{R2,R3} //57 ( adrh adrl --> )
LSL   R4,R2,#16
LSL   R3,R3,#16
LSR   R3,R3,#16
ADD   R4,R4,R3
LDR   R5,[R4]
LSR   R0,R5,#16
LSL   R1,R5,#16
LSR   R1,R1,#16
STMEA R12!,{R0,R1} //57 ( a b --> )
B     STEPEND

STEPA003:
//( 0A 003 MLIT MCODE RETURN )
LDRH  R2,[R10],#2 //44
LSL   R11,R2,#1
ADD   R11,R11,#0X10000 //66
B     STEPEND

B     STEPEND


STEPB:
AND   R1,R0,#0X00FF
// 0B 412 MLIT MCODE SWAP
CMP   R1,#0X12        //34 ( a b ) SWAP
LDMEQEA R12!,{R0,R1}  //34 ( )
STMEQEA R12!,{R1}     //34 ( b )
STMEQEA R12!,{R0}     //34 ( b a )
BEQ   STEPEND
// 0B 502 MLIT MCODE OVER
CMP   R1,#0X02       //39 ( a b ) OVER
LDMEQEA R12,{R0,R1}  //39 ( a b )
STMEQEA R12!,{R0}    //39 ( a b a )
BEQ   STEPEND
// 0B 501 MLIT MCODE DUP
CMP   R1,#0X01    //33 ( a ) DUP
LDMEQEA R12,{R0}  //33 ( a )
STMEQEA R12!,{R0} //33 ( a a )
BEQ   STEPEND
// 0B 434 MLIT MCODE ROT
CMP   R1,#0X34           //39 ( a b c ) ROT
LDMEQEA R12!,{R0,R1,R2}  //39 ( )
STMEQEA R12!,{R1,R2}     //39 ( b c )
STMEQEA R12!,{R0}        //39 ( b c a )
BEQ   STEPEND
// 0B 300 MLIT MCODE DROP //siehe 2DROP
// 0B 43C MLIT MCODE 2SWAP
CMP   R1,#0X3C              //39 ( a b c d ) 2SWAP
LDMEQEA R12!,{R0,R1,R2,R3}  //39 ( )
STMEQEA R12!,{R2,R3}        //39 ( c d )
STMEQEA R12!,{R0,R1}        //39 ( c d a b)
BEQ   STEPEND
// 0B 60C MLIT MCODE 2OVER
CMP   R1,#0X0C              //39 ( a b c d ) 2OVER
LDMEQEA R12,{R0,R1,R2,R3}   //39 ( a b c d )
STMEQEA R12!,{R0,R1}        //39 ( a b c d a b )
BEQ   STEPEND
// 0B 603 MLIT MCODE 2DUP
CMP   R1,#0X03              //39 ( a b ) 2DUP
LDMEQEA R12,{R0,R1}         //39 ( a b )
STMEQEA R12!,{R0,R1}        //39 ( a b a b )
BEQ   STEPEND
// 0B 200 MLIT MCODE 2DROP
CMP   R1,#0X00    //35 ( a b ) 2DROP und DROP
BNE   DROP9
AND   R1,R0,#0XF00
CMP   R1,#0X300
LDMEA R12!,{R0}   //35 ( a )
LDMNEEA R12!,{R0} //35 ( )
DROP9:
B     STEPEND

STEPF:
MOV   R1,#0
SUB   R1,R1,#1
LSL   R1,R1,#16
ADD   R0,R0,R1
STMEA R12!,{R0} //32 ( --> n )
B     STEPEND


STEP0123:
STMEA R12!,{R0} //32 ( --> n )

STEPEND:
//MOV   R9,R8
BL    KEYFLAG   //51 ( flag )
//BL    DUP
//BL    MDOT
LDMEA R12!,{R0} //51 ( )
CMP   R0,#0
BEQ   STEPEND0
BL    KEY
LDMEA R12!,{R0} //51 ( c )
CMP   R0,#0X02
MOVEQ R9,#1     //56 vorher MOVEQ R9,R8
BEQ   STEPEND0
//MOV   R1,#8
//STRB  R0,[R1]
MOV   R2,#0X10000
ADD   R2,R2,#4
LDR   R1,[R2]
ADD   R1,R1,#0X10000000 //65//62
STRB  R0,[R1]
ADD   R1,R1,#1
SUB   R1,R1,#0X10000000 //65//62
STR   R1,[R2]
B     STEPEND        //63
//B     STEP4
MOV   R0,#0X4000
ADD   R0,R0,#0X12
AND   R1,R0,#0XF000
//B   STEPEND0
B     STEP4
STEPEND0:

CMP   R9,#1    //56 vorher CMP   R8,R9    //42
BLT   STEPR    //42
//MOV   R0,#0X20 //48 Fehlersuch
//STMEA R12!,{R0}// ( ' ' )
//BL    EMIT     // ( )
//MOV   R0,#0X0700
//ADD   R1,R0,#0X00A7
//ADD   R1,R1,R1
//LDRH  R3,[R1]
//STMEA R12!,{R3} // ( --> n )
//BL    MDOT     // ( )
MOV   R0,#0X20 
STMEA R12!,{R0}// ( ' ' )
BL    EMIT     // ( )
BL    SPDOT
BL    CR
STMEA R12!,{R8}// ( nr )
BL    MDOT
STMEA R12!,{R10}//58 ( RP )
BL    MDOT
LDRH  R0,[R10]
STMEA R12!,{R0} //66 ( [RP] )
BL    MDOT
SUB   R2,R11,#0X10000 //66
LSR   R0,R2,#1
STMEA R12!,{R0}// ( PC )
BL    MDOT
LDRH  R0,[R11]
STMEA R12!,{R0}// ( [PC] )
BL    MDOT
LDMFD SP!,{R0-R7,PC}

RAM0000Decode: //43 zu BL RAM0000Start
STMFD SP!,{R0-R3,LR}
ADD   R0,LR,#4 //28 RAMB03C00 nach 3C00 verschieben
STMEA R12!,{R0}//28 ( ramb0000 )
MOV   R0,#0X00
STMEA R12!,{R0}//28 ( ramb0000 0 )
MOV   R0,#0X40
STMEA R12!,{R0}//28 ( ramb3C00 0 40 )
BL    MOVE     //28 ( )
LDMFD SP!,{R0-R3,PC}
RAM0000Start:
STMFD SP!,{R0-R7,LR}
BL RAM0000Decode
LDMFD SP!,{R0-R7,PC}
B     FIQ //0X00
B     FIQ //0X04
B     FIQ //0X08
B     FIQ //0X0C
B     FIQ //0X10
B     FIQ //0X14
B     FIQ //0X18
FIQ:      //0X1C
MOV   R8,#0X10000
ADD   R8,#8
LDR   R9,[R8]
ADD   R9,R9,#1
STR   R9,[R8]
SUBS  PC,R14,#4


FFDecode: //26 zu BL FFStart
STMFD SP!,{R0-R7,LR}
ADD   R0,LR,#4 //28 RAMB0000 nach 0000 verschieben
STMEA R12!,{R0}//28 ( ramb0000 )
MOV   R0,#0X10000 //66
STMEA R12!,{R0}//28 ( ramb0000 10000 )
MOV   R0,#0X3000
STMEA R12!,{R0}//28 ( ramb0000 10000 3000 )
BL    MOVE     //28 ( )
LDMFD SP!,{R0-R7,PC}
FFStart:
STMFD SP!,{R0-R7,LR}
BL FFDecode
LDMFD SP!,{R0-R7,PC}
RAM0000:
  .word 0x00004010 //0
  .word 0x00000000 //2
  .word 0x00000000 //4
  .word 0x00000000 //6
  .word 0x00000000 //8
  .word 0x00000000 //A
  .word 0x00000000 //C
  .word 0x00000000 //E
  .word 0xA0034760 //10
  .word 0xA003447D //12
  .word 0xA0034000 //14
  .word 0xA0034000 //16
  .word 0xA0034000 //18
  .word 0xA0034000 //1A
  .word 0xA0034000 //1C
  .word 0xA0030010 //1E
  .word 0x30000000 //20
  .word 0x47730001 //22
  .word 0x45E40029 //24
  .word 0xA003B200 //26
  .word 0x3002FFF8 //28
  .word 0x47730001 //2A
  .word 0x2F100000 //2C
  .word 0xA003A009 //2E
  .word 0x3004FFF8 //30
  .word 0x47810001 //32
  .word 0x2F100001 //34
  .word 0xA003A009 //36
  .word 0x3006FFF8 //38
  .word 0x47730007 //3A
  .word 0x45E40020 //3C
  .word 0x469E465E //3E
  .word 0x46A7B300 //40
  .word 0xFFF5A003 //42
  .word 0x0004300E //44
  .word 0xB4124781 //46
  .word 0xA0021000 //48
  .word 0xB300B412 //4A
  .word 0xFFF6A003 //4C
  .word 0x00033013 //4E
  .word 0xB5014781 //50
  .word 0x9001A00F //52
  .word 0xA003A000 //54
  .word 0x3017FFF7 //56
  .word 0x47810004 //58
  .word 0x4051B501 //5A
  .word 0x00000004 //5C
  .word 0xA0084047 //5E
  .word 0x01119002 //60
  .word 0x430043C2 //62
  .word 0xFFF1A003 //64
  .word 0x000B301C //66
  .word 0x42CF477A //68
  .word 0x2F10A00A //6A
  .word 0x9001A00A //6C
  .word 0xA003405A //6E
  .word 0x3028FFF5 //70
  .word 0x47810008 //72
  .word 0x406846B1 //74
  .word 0x476B4300 //76
  .word 0xFFF7A003 //78
  .word 0x00063031 //7A
  .word 0x28004069 //7C
  .word 0x3038FFFB //7E
  .word 0x40690002 //80
  .word 0xFFFB2801 //82
  .word 0x0002303B //84
  .word 0x28024069 //86
  .word 0x303EFFFB //88
  .word 0x40690002 //8A
  .word 0xFFFB2803 //8C
  .word 0x00043041 //8E
  .word 0x2F004069 //90
  .word 0x3046FFFB //92
  .word 0x40690009 //94
  .word 0xFFFB2F01 //96
  .word 0x00033050 //98
  .word 0x2F024069 //9A
  .word 0x3054FFFB //9C
  .word 0x40690007 //9E
  .word 0xFFFB2F03 //A0
  .word 0x0007305C //A2
  .word 0x2F044069 //A4
  .word 0x3064FFFB //A6
  .word 0x40690004 //A8
  .word 0xFFFB2F05 //AA
  .word 0x00073069 //AC
  .word 0x2F064069 //AE
  .word 0x3071FFFB //B0
  .word 0x40690004 //B2
  .word 0xFFFB2F07 //B4
  .word 0x00043076 //B6
  .word 0x2F084069 //B8
  .word 0x307BFFFB //BA
  .word 0x40690003 //BC
  .word 0xFFFB2F09 //BE
  .word 0x0003307F //C0
  .word 0x2F0A4069 //C2
  .word 0x3083FFFB //C4
  .word 0x40690003 //C6
  .word 0xFFFB2F0B //C8
  .word 0x00033087 //CA
  .word 0x2F0C4069 //CC
  .word 0x308BFFFB //CE
  .word 0x40690003 //D0
  .word 0xFFFB2F0D //D2
  .word 0x0007308F //D4
  .word 0x2F0E4069 //D6
  .word 0x3097FFFB //D8
  .word 0x40690002 //DA
  .word 0xFFFB2F0F //DC
  .word 0x0004309A //DE
  .word 0x2F104069 //E0
  .word 0x309FFFFB //E2
  .word 0x40690003 //E4
  .word 0xFFFB2F11 //E6
  .word 0x000430A3 //E8
  .word 0x2F124069 //EA
  .word 0x30A8FFFB //EC
  .word 0x40690005 //EE
  .word 0xFFFB2F13 //F0
  .word 0x000630AE //F2
  .word 0x2F144069 //F4
  .word 0x30B5FFFB //F6
  .word 0x40690003 //F8
  .word 0xFFFB2F15 //FA
  .word 0x000530B9 //FC
  .word 0x2F164069 //FE
  .word 0x30BFFFFB //100
  .word 0x4069000C //102
  .word 0xFFFB2F17 //104
  .word 0x000730CC //106
  .word 0x01CB4069 //108
  .word 0x30D4FFFB //10A
  .word 0x47810006 //10C
  .word 0x0003000A //10E
  .word 0xA0034047 //110
  .word 0x30DBFFF8 //112
  .word 0x477A0008 //114
  .word 0x2F1042CF //116
  .word 0x9003A00A //118
  .word 0x4300A00A //11A
  .word 0x430B8001 //11C
  .word 0xFFF3A003 //11E
  .word 0x000530E4 //120
  .word 0x46B14781 //122
  .word 0x43004115 //124
  .word 0x4300410E //126
  .word 0xA003476B //128
  .word 0x30EAFFF5 //12A
  .word 0x41160005 //12C
  .word 0xA003A000 //12E
  .word 0x30F0FFFA //130
  .word 0x41160002 //132
  .word 0xA003A001 //134
  .word 0x30F3FFFA //136
  .word 0x41160002 //138
  .word 0xA003A002 //13A
  .word 0x30F6FFFA //13C
  .word 0x41160002 //13E
  .word 0xA003A00D //140
  .word 0x30F9FFFA //142
  .word 0x41160003 //144
  .word 0xA003A00F //146
  .word 0x30FDFFFA //148
  .word 0x41160008 //14A
  .word 0xA003A005 //14C
  .word 0x3106FFFA //14E
  .word 0x41160003 //150
  .word 0xA003A00B //152
  .word 0x310AFFFA //154
  .word 0x41160003 //156
  .word 0xA003A008 //158
  .word 0x310EFFFA //15A
  .word 0x41160002 //15C
  .word 0xA003A00E //15E
  .word 0x3111FFFA //160
  .word 0x41160002 //162
  .word 0xA003A007 //164
  .word 0x3114FFFA //166
  .word 0x41160001 //168
  .word 0xA003A009 //16A
  .word 0x3116FFFA //16C
  .word 0x41160001 //16E
  .word 0xA003A00A //170
  .word 0x3118FFFA //172
  .word 0x41160004 //174
  .word 0xA003B412 //176
  .word 0x311DFFFA //178
  .word 0x41160004 //17A
  .word 0xA003B502 //17C
  .word 0x3122FFFA //17E
  .word 0x41160003 //180
  .word 0xA003B501 //182
  .word 0x3126FFFA //184
  .word 0x41160003 //186
  .word 0xA003B434 //188
  .word 0x312AFFFA //18A
  .word 0x41160004 //18C
  .word 0xA003B300 //18E
  .word 0x312FFFFA //190
  .word 0x41160005 //192
  .word 0xA003B43C //194
  .word 0x3135FFFA //196
  .word 0x41160005 //198
  .word 0xA003B60C //19A
  .word 0x313BFFFA //19C
  .word 0x41160004 //19E
  .word 0xA003B603 //1A0
  .word 0x3140FFFA //1A2
  .word 0x41160005 //1A4
  .word 0xA003B200 //1A6
  .word 0x3146FFFA //1A8
  .word 0x41160004 //1AA
  .word 0xA0038000 //1AC
  .word 0x314BFFFA //1AE
  .word 0x47810002 //1B0
  .word 0xA00A2F13 //1B2
  .word 0x0001A009 //1B4
  .word 0x42C42F13 //1B6
  .word 0xFFF5A003 //1B8
  .word 0x0002314E //1BA
  .word 0x2F134781 //1BC
  .word 0x405AA00A //1BE
  .word 0x4300B501 //1C0
  .word 0xB501B412 //1C2
  .word 0x41B2A00A //1C4
  .word 0xB4124286 //1C6
  .word 0xB501428D //1C8
  .word 0x9FF6A00D //1CA
  .word 0x0020B200 //1CC
  .word 0xA00341B2 //1CE
  .word 0x3151FFE9 //1D0
  .word 0x477A0007 //1D2
  .word 0x2F1045E4 //1D4
  .word 0x9003A00A //1D6
  .word 0x42CF41BD //1D8
  .word 0xA00346A7 //1DA
  .word 0x3159FFF4 //1DC
  .word 0x47810005 //1DE
  .word 0x000146B1 //1E0
  .word 0xA0092F10 //1E2
  .word 0x41D34300 //1E4
  .word 0x2F15FFFF //1E6
  .word 0xA00342C4 //1E8
  .word 0x315FFFF2 //1EA
  .word 0x00220001 //1EC
  .word 0xA00341D4 //1EE
  .word 0x3161FFFA //1F0
  .word 0x00220002 //1F2
  .word 0x433C41D4 //1F4
  .word 0xFFF9A003 //1F6
  .word 0x00043164 //1F8
  .word 0x2F0F4781 //1FA
  .word 0xA003A00A //1FC
  .word 0x3169FFF9 //1FE
  .word 0x47810005 //200
  .word 0xA0030008 //202
  .word 0x316FFFFA //204
  .word 0x47810006 //206
  .word 0xA0030009 //208
  .word 0x3176FFFA //20A
  .word 0x47810006 //20C
  .word 0x10000000 //20E
  .word 0xA002B434 //210
  .word 0xB300B412 //212
  .word 0x0FFFB412 //214
  .word 0xA00EA008 //216
  .word 0xFFF1A003 //218
  .word 0x0005317D //21A
  .word 0x2F0F4781 //21C
  .word 0xA00342C4 //21E
  .word 0x3183FFF9 //220
  .word 0x47810007 //222
  .word 0x428641FB //224
  .word 0x42024294 //226
  .word 0x4300420E //228
  .word 0xFFF5A003 //22A
  .word 0x0008318B //22C
  .word 0x41FB4781 //22E
  .word 0x42944286 //230
  .word 0x420E4208 //232
  .word 0xA0034300 //234
  .word 0x3194FFF5 //236
  .word 0x47730005 //238
  .word 0xA00341FB //23A
  .word 0x319AFFFA //23C
  .word 0x47730005 //23E
  .word 0xA0034224 //240
  .word 0x31A0FFFA //242
  .word 0x47730005 //244
  .word 0xA003422F //246
  .word 0x31A6FFFA //248
  .word 0x47730002 //24A
  .word 0x00014208 //24C
  .word 0x41FB421D //24E
  .word 0xFFF7A003 //250
  .word 0x000631A9 //252
  .word 0x41FB4773 //254
  .word 0x4294B502 //256
  .word 0x420EB434 //258
  .word 0x428DB412 //25A
  .word 0xA003A009 //25C
  .word 0x31B0FFF3 //25E
  .word 0x47730004 //260
  .word 0x421D0001 //262
  .word 0x42024254 //264
  .word 0xA00341FB //266
  .word 0x31B5FFF6 //268
  .word 0x47730005 //26A
  .word 0xA003424B //26C
  .word 0x31BBFFFA //26E
  .word 0x47730006 //270
  .word 0x423FB434 //272
  .word 0xA0034254 //274
  .word 0x31C2FFF8 //276
  .word 0x47810002 //278
  .word 0xA003A00A //27A
  .word 0x31C5FFFA //27C
  .word 0x47810002 //27E
  .word 0xA003A009 //280
  .word 0x31C8FFFA //282
  .word 0x47810002 //284
  .word 0xA0070001 //286
  .word 0xFFF9A003 //288
  .word 0x000231CB //28A
  .word 0xFFFF4781 //28C
  .word 0xA003A007 //28E
  .word 0x31CEFFF9 //290
  .word 0x47810002 //292
  .word 0xA007A000 //294
  .word 0xFFF9A003 //296
  .word 0x000131D1 //298
  .word 0x42944781 //29A
  .word 0xA003A00D //29C
  .word 0x31D3FFF9 //29E
  .word 0x47810002 //2A0
  .word 0xA00F4294 //2A2
  .word 0xFFF9A003 //2A4
  .word 0x000131D6 //2A6
  .word 0xB4124781 //2A8
  .word 0xA00342A2 //2AA
  .word 0x31D8FFF9 //2AC
  .word 0x47810002 //2AE
  .word 0xB4340000 //2B0
  .word 0xA002B434 //2B2
  .word 0xB300B412 //2B4
  .word 0xFFF5A003 //2B6
  .word 0x000331DB //2B8
  .word 0x31DF4781 //2BA
  .word 0x41F50004 //2BC
  .word 0xA0038FFC //2BE
  .word 0x31E4FFF7 //2C0
  .word 0x47810002 //2C2
  .word 0xB502B412 //2C4
  .word 0xA007A00A //2C6
  .word 0xA009B412 //2C8
  .word 0xFFF5A003 //2CA
  .word 0x000231E7 //2CC
  .word 0x28024781 //2CE
  .word 0x4286A00A //2D0
  .word 0x2802A00A //2D2
  .word 0x4286A00A //2D4
  .word 0xB6032802 //2D6
  .word 0xA00AA00A //2D8
  .word 0xA009B412 //2DA
  .word 0xA003A009 //2DC
  .word 0x31EAFFED //2DE
  .word 0x47810002 //2E0
  .word 0xA00A2802 //2E2
  .word 0x428DB501 //2E4
  .word 0xB6032802 //2E6
  .word 0xA00AA00A //2E8
  .word 0xB501B412 //2EA
  .word 0x2802428D //2EC
  .word 0xA009A009 //2EE
  .word 0xA009A009 //2F0
  .word 0xFFEBA003 //2F2
  .word 0x000131ED //2F4
  .word 0x28024781 //2F6
  .word 0x4286A00A //2F8
  .word 0xA003A00A //2FA
  .word 0x31EFFFF7 //2FC
  .word 0x47810001 //2FE
  .word 0xA00A2F0F //300
  .word 0x0001A009 //302
  .word 0x42C42F0F //304
  .word 0xFFF5A003 //306
  .word 0x000731F1 //308
  .word 0x28034781 //30A
  .word 0xA003A009 //30C
  .word 0x31F9FFF9 //30E
  .word 0x47810003 //310
  .word 0xA0048000 //312 //63 44AC A004
  .word 0x9002A00B //314
  .word 0x8FFAB300 //316
  .word 0xFFF5A003 //318
  .word 0x000431FD //31A
  .word 0x014C4781 //31C
  .word 0xA003430B //31E
  .word 0x3202FFF9 //320
  .word 0x47810005 //322
  .word 0xB4120000 //324
  .word 0xA0020010 //326
  .word 0xA003B412 //328
  .word 0x3208FFF6 //32A
  .word 0x47810003 //32C
  .word 0x000AB501 //32E
  .word 0x900142A2 //330
  .word 0x00078002 //332
  .word 0x0030A007 //334
  .word 0xA003A007 //336
  .word 0x320CFFF2 //338
  .word 0x47810004 //33A
  .word 0x9008B501 //33C
  .word 0xB501B412 //33E
  .word 0x431D427A //340
  .word 0xB4124286 //342
  .word 0x8FF6428D //344
  .word 0xA003B200 //346
  .word 0x3211FFF0 //348
  .word 0x47810003 //34A
  .word 0x432E4324 //34C
  .word 0x4324431D //34E
  .word 0x431D432E //350
  .word 0x432E4324 //352
  .word 0x4324431D //354
  .word 0x431D432E //356
  .word 0xA003B300 //358
  .word 0x3215FFEE //35A
  .word 0x47810002 //35C
  .word 0x0020434C //35E
  .word 0xA003431D //360
  .word 0x3218FFF8 //362
  .word 0x47810002 //364
  .word 0x435EA00A //366
  .word 0xFFF9A003 //368
  .word 0x0002321B //36A
  .word 0x2F074781 //36C
  .word 0x2F0FA00A //36E
  .word 0x4294A00A //370
  .word 0xA00A2F10 //372
  .word 0xA00BA00D //374
  .word 0x2F00A00E //376
  .word 0xA00DA00A //378
  .word 0xA008A00B //37A
  .word 0x003C9028 //37C
  .word 0x321E431D //37E
  .word 0x41F50003 //380
  .word 0xA00A2F07 //382
  .word 0x2F06435E //384
  .word 0x435EA00A //386
  .word 0x431D003C //388
  .word 0x00043222 //38A
  .word 0x003C41F5 //38C
  .word 0x3227431D //38E
  .word 0x41F50003 //390
  .word 0xA00A2F0F //392
  .word 0x2F13435E //394
  .word 0x435EA00A //396
  .word 0x431D003C //398
  .word 0x0004322B //39A
  .word 0x2F0F41F5 //39C
  .word 0x2F07A00A //39E
  .word 0x2F13A009 //3A0
  .word 0x2F06A00A //3A2
  .word 0x000AA009 //3A4
  .word 0xA003431D //3A6
  .word 0x3230FFC1 //3A8
  .word 0x4781000A //3AA
  .word 0xFFFBA003 //3AC
  .word 0x0007323B //3AE
  .word 0x436D4781 //3B0
  .word 0x00193243 //3B2
  .word 0x002041F5 //3B4
  .word 0x0008431D //3B6
  .word 0x4312431D //3B8
  .word 0x429B001B //3BA
  .word 0xA0039FF8 //3BC
  .word 0x325DFFEF //3BE
  .word 0x47810005 //3C0
  .word 0x2F0EB501 //3C2
  .word 0x0000A009 //3C4
  .word 0xA0092F10 //3C6
  .word 0x2F0A436D //3C8
  .word 0x2F0CA00A //3CA
  .word 0x2F0AA00A //3CC
  .word 0x4294A00A //3CE
  .word 0x433C428D //3D0
  .word 0x00033263 //3D2
  .word 0x326741F5 //3D4
  .word 0x41EF000A //3D6
  .word 0x436D46C6 //3D8
  .word 0x00163272 //3DA
  .word 0x435E41F5 //3DC
  .word 0x471343B1 //3DE
  .word 0xFFDDA003 //3E0
  .word 0x00043289 //3E2
  .word 0x28014781 //3E4
  .word 0x2F15A00A //3E6
  .word 0xA003A009 //3E8
  .word 0x328EFFF7 //3EA
  .word 0x47810004 //3EC
  .word 0xA00A2801 //3EE
  .word 0xA00A2F15 //3F0
  .word 0x90024294 //3F2
  .word 0x43C20009 //3F4
  .word 0xFFF3A003 //3F6
  .word 0x00053293 //3F8
  .word 0x42864781 //3FA
  .word 0xA00A2F17 //3FC
  .word 0x4294B502 //3FE
  .word 0x2F17B501 //400
  .word 0xA009A009 //402
  .word 0xFFF2A003 //404
  .word 0x00093299 //406
  .word 0x2F174781 //408
  .word 0xB501A00A //40A
  .word 0xA007A00A //40C
  .word 0xA0092F17 //40E
  .word 0xFFF4A003 //410
  .word 0x000232A3 //412
  .word 0x2F174781 //414
  .word 0x4286A00A //416
  .word 0xFFF8A003 //418
  .word 0x000232A6 //41A
  .word 0x2F174781 //41C
  .word 0x0002A00A //41E
  .word 0xA003A007 //420
  .word 0x32A9FFF7 //422
  .word 0x47810002 //424
  .word 0xA00A2F17 //426
  .word 0xA0070003 //428
  .word 0xFFF7A003 //42A
  .word 0x000232AC //42C
  .word 0x2F174781 //42E
  .word 0x0004A00A //430
  .word 0xA003A007 //432
  .word 0x32AFFFF7 //434
  .word 0x47810002 //436
  .word 0xA00A2F17 //438
  .word 0xA0070005 //43A
  .word 0xFFF7A003 //43C
  .word 0x000232B2 //43E
  .word 0x2F174781 //440
  .word 0x0006A00A //442
  .word 0xA003A007 //444
  .word 0x32B5FFF7 //446
  .word 0x47810002 //448
  .word 0xA00A2F17 //44A
  .word 0xA0070007 //44C
  .word 0xFFF7A003 //44E
  .word 0x000232B8 //450
  .word 0x2F174781 //452
  .word 0x0008A00A //454
  .word 0xA003A007 //456
  .word 0x32BBFFF7 //458
  .word 0x47730001 //45A
  .word 0x45E40020 //45C
  .word 0x469E465E //45E
  .word 0x4286B300 //460
  .word 0xA00A2F10 //462
  .word 0x405A9001 //464
  .word 0xFFF1A003 //466
  .word 0x000532BD //468
  .word 0xB5014781 //46A
  .word 0x4286A00A //46C
  .word 0x03FFB501 //46E
  .word 0x0000A008 //470
  .word 0x9002429B //472
  .word 0xA007FC00 //474
  .word 0xA009B412 //476
  .word 0xFFEEA003 //478
  .word 0x000732C3 //47A
  .word 0x28004781 //47C
  .word 0xB501A00A //47E
  .word 0x42A20008 //480
  .word 0x00189009 //482
  .word 0xA00AA007 //484
  .word 0x9002B501 //486
  .word 0x430BB501 //488
  .word 0x8018B300 //48A
  .word 0xA00A2F03 //48C
  .word 0x2F03A009 //48E
  .word 0x2F03446B //490
  .word 0x2F04A00A //492
  .word 0x4294A00A //494
  .word 0xA00803FF //496
  .word 0x42A90080 //498  //59 80 in 40
  .word 0x2F059009 //49A
  .word 0xA00DA00A //49C
  .word 0xFFFF9005 //49E
  .word 0xA0092F05 //4A0
  .word 0x431D0013 //4A2
  .word 0x28000000 //4A4
  .word 0xA003A009 //4A6
  .word 0x32CBFFD1 //4A8
  .word 0x47810008 //4AA
  .word 0xA00A2F04 //4AC
  .word 0xA00A2F03 //4AE
  .word 0x9003429B //4B0
  .word 0x00000000 //4B2
  .word 0x2F048018 //4B4
  .word 0xA00AA00A //4B6
  .word 0x2F04FFFF //4B8
  .word 0x2F03446B //4BA
  .word 0x2F04A00A //4BC
  .word 0x4294A00A //4BE
  .word 0xA00803FF //4C0
  .word 0x42A20020 //4C2 //59 20 in 10
  .word 0x2F059008 //4C4
  .word 0x9005A00A //4C6
  .word 0x2F050000 //4C8
  .word 0x0011A009 //4CA
  .word 0xA003431D //4CC
  .word 0x32D4FFDA //4CE
  .word 0x47810006 //4D0
  .word 0x43FB0005 //4D2
  .word 0xA0094426 //4D4
  .word 0xA009441D //4D6
  .word 0xA00A441D //4D8
  .word 0xA0094438 //4DA
  .word 0xB5014312 //4DC
  .word 0x429B0014 //4DE
  .word 0xB3009004 //4E0
  .word 0xA00A441D //4E2
  .word 0xB501427A //4E4
  .word 0x429B007F //4E6
  .word 0xB3009002 //4E8
  .word 0xB5010008 //4EA
  .word 0x429B0008 //4EC
  .word 0x44389012 //4EE
  .word 0x441DA00A //4F0
  .word 0x42A2A00A //4F2
  .word 0xFFFF900C //4F4
  .word 0x42C4441D //4F6
  .word 0x44260001 //4F8
  .word 0x000842C4 //4FA
  .word 0x0020431D //4FC
  .word 0x0008431D //4FE
  .word 0xB501431D //500
  .word 0x42A20020 //502
  .word 0x80129001 //504
  .word 0x4426FFFF //506
  .word 0x442642C4 //508
  .word 0xA00FA00A //50A
  .word 0x00069002 //50C
  .word 0xB50143C2 //50E
  .word 0xB501431D //510
  .word 0xA00A441D //512
  .word 0x00014280 //514
  .word 0x42C4441D //516
  .word 0x0020B501 //518
  .word 0xB50242A2 //51A
  .word 0x429B0008 //51C
  .word 0xA008A00B //51E
  .word 0x001BB412 //520
  .word 0xA00B429B //522
  .word 0x4426A008 //524
  .word 0xA00DA00A //526
  .word 0x9FB2A00E //528
  .word 0x431D0020 //52A
  .word 0xA00A4438 //52C
  .word 0xA00A441D //52E
  .word 0xA00A4438 //530
  .word 0xB6034294 //532
  .word 0x0000A007 //534
  .word 0x4280B412 //536
  .word 0xA0034409 //538
  .word 0x32DBFF94 //53A
  .word 0x47810005 //53C
  .word 0x0030B501 //53E
  .word 0xA00B42A2 //540
  .word 0x003AB502 //542
  .word 0xA00842A2 //544
  .word 0x0041B502 //546
  .word 0xA00B42A2 //548
  .word 0xB501A00E //54A
  .word 0xB4129015 //54C
  .word 0x42940030 //54E
  .word 0x000AB501 //550
  .word 0xA00B42A2 //552
  .word 0x00079002 //554
  .word 0xB5014294 //556
  .word 0xA00A2F08 //558
  .word 0xA00B42A2 //55A
  .word 0xB3009004 //55C
  .word 0x0000B300 //55E
  .word 0xB4120000 //560
  .word 0xFFD7A003 //562
  .word 0x000632E1 //564
  .word 0x00074781 //566
  .word 0x441D43FB //568
  .word 0x4415A009 //56A
  .word 0x0000A009 //56C
  .word 0xA00A441D //56E
  .word 0xB5019063 //570
  .word 0xA0094426 //572
  .word 0x44410001 //574
  .word 0xFFFFA009 //576
  .word 0xA009444A //578
  .word 0xA00A4415 //57A
  .word 0xA00A4426 //57C
  .word 0x427AA007 //57E
  .word 0x429B002B //580
  .word 0x44269009 //582
  .word 0x4286A00A //584
  .word 0xA0094426 //586
  .word 0x444A0000 //588
  .word 0x8016A009 //58A
  .word 0xA00A4415 //58C
  .word 0xA00A4426 //58E
  .word 0x427AA007 //590
  .word 0x429B002D //592
  .word 0x4426900D //594
  .word 0x4286A00A //596
  .word 0xA0094426 //598
  .word 0x444A0000 //59A
  .word 0x4441A009 //59C
  .word 0xA000A00A //59E
  .word 0xA0094441 //5A0
  .word 0xA00A444A //5A2
  .word 0x44269FD2 //5A4
  .word 0x441DA00A //5A6
  .word 0x42A2A00A //5A8
  .word 0x44159029 //5AA
  .word 0x4426A00A //5AC
  .word 0xA007A00A //5AE
  .word 0xB501427A //5B0
  .word 0x453E9015 //5B2
  .word 0x9007A00B //5B4
  .word 0x441DB300 //5B6
  .word 0xA000A00A //5B8
  .word 0xA009441D //5BA
  .word 0xB412800A //5BC
  .word 0xA00A2F08 //5BE
  .word 0xA00742B0 //5C0
  .word 0xA00A4426 //5C2
  .word 0x44264286 //5C4
  .word 0x8005A009 //5C6
  .word 0x4426B300 //5C8
  .word 0x441DA00A //5CA
  .word 0x4426A009 //5CC
  .word 0x441DA00A //5CE
  .word 0x42A2A00A //5D0
  .word 0x9FD7A00B //5D2
  .word 0xA00A4441 //5D4
  .word 0x9001A00F //5D6
  .word 0x4426A000 //5D8
  .word 0x441DA00A //5DA
  .word 0x4294A00A //5DC
  .word 0xA0034409 //5DE
  .word 0x32E8FF83 //5E0
  .word 0x47810004 //5E2
  .word 0x2F0C42E2 //5E4
  .word 0x2F0BA00A //5E6
  .word 0x2F0CA009 //5E8
  .word 0x427AA00A //5EA
  .word 0x429B42F7 //5EC
  .word 0xA00A2F0C //5EE
  .word 0xA00A2F0D //5F0
  .word 0xA00842A2 //5F2
  .word 0x00019004 //5F4
  .word 0x42C42F0C //5F6
  .word 0x2F0C8FF0 //5F8
  .word 0x2F0BA00A //5FA
  .word 0x2F0CA009 //5FC
  .word 0x427AA00A //5FE
  .word 0x429B003C //600
  .word 0x2F0C9004 //602
  .word 0x2F0DA00A //604
  .word 0x2F0CA009 //606
  .word 0x427AA00A //608
  .word 0x429B42F7 //60A
  .word 0x2F0CA00B //60C
  .word 0x2F0DA00A //60E
  .word 0x42A2A00A //610
  .word 0x9004A008 //612
  .word 0x2F0C0001 //614
  .word 0x8FE542C4 //616
  .word 0xA00A2F0B //618
  .word 0xA00A2F0C //61A
  .word 0x4294B502 //61C
  .word 0x9003B501 //61E
  .word 0x2F0C0001 //620
  .word 0x42CF42C4 //622
  .word 0xA003B300 //624
  .word 0x32EDFFBA //626
  .word 0x47810002 //628
  .word 0xB50242E2 //62A
  .word 0x429442F7 //62C
  .word 0x42CF9007 //62E
  .word 0xB300B300 //630
  .word 0xB300B300 //632
  .word 0x80230000 //634
  .word 0xB30042CF //636
  .word 0x0000B412 //638
  .word 0x4294B603 //63A
  .word 0x42E29016 //63C
  .word 0xB50242E2 //63E
  .word 0xB502427A //640
  .word 0x4294427A //642
  .word 0xB3009004 //644
  .word 0x0000B300 //646
  .word 0xB5010000 //648
  .word 0x42869004 //64A
  .word 0x4286B412 //64C
  .word 0x42CFB412 //64E
  .word 0x428642CF //650
  .word 0xB2008FE7 //652
  .word 0x9002B300 //654
  .word 0x8001FFFF //656
  .word 0xA0030000 //658
  .word 0x32F0FFCC //65A
  .word 0x47810004 //65C
  .word 0x42E242E2 //65E
  .word 0x2F110000 //660
  .word 0x2F01A00A //662
  .word 0x9003A00A //664
  .word 0xA00AB501 //666
  .word 0xB501A007 //668
  .word 0xB5014286 //66A
  .word 0xB412A00A //66C
  .word 0xA00A4286 //66E
  .word 0x42CF42CF //670
  .word 0x42E2B603 //672
  .word 0x462A42E2 //674
  .word 0xB4129003 //676
  .word 0xB412A00D //678
  .word 0xA00DB502 //67A
  .word 0xA00AB502 //67C
  .word 0xA00BA00D //67E
  .word 0xB502A008 //680
  .word 0xA00AB501 //682
  .word 0x2F11A007 //684
  .word 0x429BA00A //686
  .word 0xA008A00B //688
  .word 0xB5019004 //68A
  .word 0xA007A00A //68C
  .word 0x42CF8FDA //68E
  .word 0x42CFB300 //690
  .word 0xA00DB434 //692
  .word 0xB3009004 //694
  .word 0x0000B300 //696
  .word 0xA0030000 //698
  .word 0x32F5FFC0 //69A
  .word 0x47810004 //69C
  .word 0x0003B412 //69E
  .word 0xB412A007 //6A0
  .word 0xFFF7A003 //6A2
  .word 0x000832FA //6A4
  .word 0x00044781 //6A6
  .word 0x40470000 //6A8
  .word 0x4300A00E //6AA
  .word 0xFFF6A003 //6AC
  .word 0x00063303 //6AE
  .word 0x43E54781 //6B0
  .word 0xA00A2F0F //6B2
  .word 0xA00A2F11 //6B4
  .word 0x4294B502 //6B6
  .word 0x2F114300 //6B8
  .word 0x0020A009 //6BA
  .word 0x41BD45E4 //6BC
  .word 0x2F010001 //6BE
  .word 0xA003A009 //6C0
  .word 0x330AFFEB //6C2
  .word 0x47810009 //6C4
  .word 0xA00A2F0A //6C6
  .word 0x2F0B42E2 //6C8
  .word 0x42E2A00A //6CA
  .word 0xA00A2F0C //6CC
  .word 0x2F0D42E2 //6CE
  .word 0x42E2A00A //6D0
  .word 0xA007B502 //6D2
  .word 0xA0092F0D //6D4
  .word 0x2F0AB501 //6D6
  .word 0xB501A009 //6D8
  .word 0xA0092F0B //6DA
  .word 0xA0092F0C //6DC
  .word 0x45E40020 //6DE
  .word 0x901FB501 //6E0
  .word 0x465EB603 //6E2
  .word 0x9009B501 //6E4
  .word 0x42E242E2 //6E6
  .word 0x42CFB200 //6E8
  .word 0x469E42CF //6EA
  .word 0x430BB300 //6EC
  .word 0xB2008011 //6EE
  .word 0x4567B603 //6F0
  .word 0xB2009005 //6F2
  .word 0x0003B300 //6F4
  .word 0x800843C2 //6F6
  .word 0xB300B434 //6F8
  .word 0xB300B412 //6FA
  .word 0xA00A2F10 //6FC
  .word 0x405A9001 //6FE
  .word 0xB2008FDD //700
  .word 0x2F0D42CF //702
  .word 0x42CFA009 //704
  .word 0xA0092F0C //706
  .word 0x2F0B42CF //708
  .word 0x42CFA009 //70A
  .word 0xA0092F0A //70C
  .word 0xFFB3A003 //70E
  .word 0x00043314 //710
  .word 0x2F024781 //712
  .word 0x2802A00A //714
  .word 0x2F00A009 //716
  .word 0x9006A00A //718
  .word 0x431D003C //71A
  .word 0x00043319 //71C
  .word 0x800341F5 //71E
  .word 0x0002331E //720
  .word 0x436D41F5 //722
  .word 0xA00A2F09 //724
  .word 0x44D20100 //726
  .word 0xA00AB502 //728
  .word 0x429B003C //72A
  .word 0xB2009002 //72C
  .word 0x2F00802B //72E
  .word 0x900CA00A //730
  .word 0x431D003C //732
  .word 0x00033321 //734
  .word 0x46C641F5 //736
  .word 0x431D003C //738
  .word 0x00043325 //73A
  .word 0x801C41F5 //73C
  .word 0x431D001B //73E
  .word 0x431D005B //740
  .word 0x431D0033 //742
  .word 0x431D0036 //744
  .word 0x431D006D //746
  .word 0x2F1046C6 //748
  .word 0xA00DA00A //74A
  .word 0x332A9003 //74C
  .word 0x41F50002 //74E
  .word 0x431D001B //750
  .word 0x431D005B //752
  .word 0x431D0033 //754
  .word 0x431D0039 //756
  .word 0x431D006D //758
  .word 0xA0038FC8 //75A
  .word 0x332DFFB3 //75C
  .word 0x47810005 //75E
  .word 0x000B3333 //760
  .word 0x436D41F5 //762
  .word 0x4713436D //764
  .word 0xFFF5A003 //766
  .word 0x0006333F //768
  .word 0x00004781 //76A
  .word 0xA0092F01 //76C
  .word 0xFFF8A003 //76E
  .word 0x000C3346 //770
  .word 0x42CF4781 //772
  .word 0xA00342E2 //774
  .word 0x3353FFF9 //776
  .word 0x4781000A //778
  .word 0x46A742CF //77A
  .word 0xFFF9A003 //77C
  .word 0x0003335E //77E
  .word 0x42CF4781 //780
  .word 0xA00A2F10 //782
  .word 0x46A79002 //784
  .word 0x42E28001 //786
  .word 0xFFF4A003 //788
  .word 0x000A3362 //78A
  .word 0x46B14781 //78C
  .word 0x2F100001 //78E
  .word 0x4772A009 //790
  .word 0xFFF6A003 //792
  .word 0x0008336D //794
  .word 0x46B14781 //796
  .word 0x2F100001 //798
  .word 0x4779A009 //79A
  .word 0xFFF6A003 //79C
  .word 0x00013376 //79E
  .word 0x46B14781 //7A0
  .word 0x2F100001 //7A2
  .word 0x4780A009 //7A4
  .word 0xFFF6A003 //7A6
  .word 0x00013378 //7A8
  .word 0x00004773 //7AA
  .word 0xA0092F10 //7AC
  .word 0x410E43EE //7AE
  .word 0x476B4300 //7B0
  .word 0xFFF4A003 //7B2
  .word 0x0003337A //7B4
  .word 0x2F164781 //7B6
  .word 0x9005A00A //7B8
  .word 0xB3004324 //7BA
  .word 0xB3004324 //7BC
  .word 0x43248006 //7BE
  .word 0x431D432E //7C0
  .word 0x432E4324 //7C2
  .word 0x4324431D //7C4
  .word 0x431D432E //7C6
  .word 0x432E4324 //7C8
  .word 0xB300431D //7CA
  .word 0xFFE6A003 //7CC
  .word 0x0003337E //7CE
  .word 0x33824781 //7D0
  .word 0x41F50001 //7D2
  .word 0x431D0022 //7D4
  .word 0x002247B7 //7D6
  .word 0x3384431D //7D8
  .word 0x41F50001 //7DA
  .word 0xFFF0A003 //7DC
  .word 0x00053386 //7DE
  .word 0x2F164781 //7E0
  .word 0x2F00A009 //7E2
  .word 0x42E2A00A //7E4
  .word 0x2F000000 //7E6
  .word 0x338CA009 //7E8
  .word 0x41EF0008 //7EA
  .word 0x000446C6 //7EC
  .word 0x40470000 //7EE
  .word 0x0010A00E //7F0
  .word 0x436DA009 //7F2
  .word 0x431D003C //7F4
  .word 0x00063395 //7F6
  .word 0x436D41F5 //7F8
  .word 0x0002339C //7FA
  .word 0x000041F5 //7FC
  .word 0xA007B603 //7FE
  .word 0x2F03B501 //800
  .word 0x9002429B //802
  .word 0x2F04B300 //804
  .word 0x2F17B501 //806
  .word 0x9005429B //808
  .word 0x2D00B300 //80A
  .word 0xA0092F80 //80C
  .word 0xB5012F80 //80E
  .word 0x429B2F05 //810
  .word 0xB3009002 //812
  .word 0xA00A2F00 //814
  .word 0x428647D1 //816
  .word 0x0010B501 //818
  .word 0x9FE2429B //81A
  .word 0x339FB300 //81C
  .word 0x41F50004 //81E
  .word 0x434CB501 //820
  .word 0x000133A4 //822
  .word 0xB50141F5 //824
  .word 0xA007000F //826
  .word 0x0010434C //828
  .word 0xB603A007 //82A
  .word 0xA00B42A9 //82C
  .word 0xB2009FCA //82E
  .word 0x003C436D //830
  .word 0x33A6431D //832
  .word 0x41F50007 //834
  .word 0x2F0042CF //836
  .word 0xA003A009 //838
  .word 0x000833B4 //83A
  .word 0x2F204781 //83C
  .word 0xB501A00A //83E
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0

RAM3000Decode: //26 zu BL RAM3000Start
STMFD SP!,{R0-R3,LR}
ADD   R0,LR,#4 //28 RAMB03000 nach 3000 verschieben
STMEA R12!,{R0}//28 ( ramb3000 )
MOV   R0,#0X16000 //65
STMEA R12!,{R0}//28 ( ramb3000 16000 )
MOV   R0,#0X1000
STMEA R12!,{R0}//28 ( ramb3000 16000 1000 )
BL    MOVE     //28 ( )
LDMFD SP!,{R0-R3,PC}
RAM3000Start:
STMFD SP!,{R0-R7,LR}
BL RAM3000Decode
LDMFD SP!,{R0-R7,PC}
RAM3000:
  .word 0x207B2028 //0
  .word 0x4F43207D //4
  .word 0x4C49504D //8
  .word 0x4C4D2045 //C
  .word 0x41205449 //10
  .word 0x4C205342 //14
  .word 0x202C5449 //18
  .word 0x4E4F4328 //1C
  .word 0x4E415453 //20
  .word 0x20293A54 //24
  .word 0x534E4F43 //28
  .word 0x544E4154 //2C
  .word 0x59454B20 //30
  .word 0x20524441 //34
  .word 0x52205053 //38
  .word 0x43502050 //3C
  .word 0x49425820 //40
  .word 0x4D532054 //44
  .word 0x45474455 //48
  .word 0x20544942 //4C
  .word 0x20305052 //50
  .word 0x4D415249 //54
  .word 0x20524441 //58
  .word 0x4D41524A //5C
  .word 0x20524441 //60
  .word 0x46464F58 //64
  .word 0x42524320 //68
  .word 0x4749455A //6C
  .word 0x44524320 //70
  .word 0x41422050 //74
  .word 0x54204553 //78
  .word 0x49204249 //7C
  .word 0x4920314E //80
  .word 0x4920324E //84
  .word 0x4920334E //88
  .word 0x4520344E //8C
  .word 0x524F5252 //90
  .word 0x4420524E //94
  .word 0x54532050 //98
  .word 0x4C205441 //9C
  .word 0x42204146 //A0
  .word 0x20464E41 //A4
  .word 0x49455A42 //A8
  .word 0x50442047 //AC
  .word 0x4B52454D //B0
  .word 0x50534320 //B4
  .word 0x42554420 //B8
  .word 0x4C205449 //BC
  .word 0x4C41434F //C0
  .word 0x45524441 //C4
  .word 0x20455353 //C8
  .word 0x53524556 //CC
  .word 0x204E4F49 //D0
  .word 0x55544552 //D4
  .word 0x28204E52 //D8
  .word 0x444F434D //DC
  .word 0x20293A45 //E0
  .word 0x444F434D //E4
  .word 0x494D2045 //E8
  .word 0x2053554E //EC
  .word 0x55202B55 //F0
  .word 0x3D30202A //F4
  .word 0x544C3020 //F8
  .word 0x494D4520 //FC
  .word 0x444F4354 //100
  .word 0x4F4E2045 //104
  .word 0x4E412054 //108
  .word 0x524F2044 //10C
  .word 0x202B4D20 //110
  .word 0x20402021 //114
  .word 0x50415753 //118
  .word 0x45564F20 //11C
  .word 0x55442052 //120
  .word 0x4F522050 //124
  .word 0x52442054 //128
  .word 0x3220504F //12C
  .word 0x50415753 //130
  .word 0x564F3220 //134
  .word 0x32205245 //138
  .word 0x20505544 //13C
  .word 0x4F524432 //140
  .word 0x4F4E2050 //144
  .word 0x4220504F //148
  .word 0x2C5A202C //14C
  .word 0x4F572820 //150
  .word 0x293A4452 //154
  .word 0x524F5720 //158
  .word 0x22203A44 //15C
  .word 0x20222E20 //160
  .word 0x45524548 //164
  .word 0x42524A20 //168
  .word 0x4A205449 //16C
  .word 0x49423052 //170
  .word 0x53582054 //174
  .word 0x54425445 //178
  .word 0x4C4C4120 //17C
  .word 0x4220544F //180
  .word 0x434E4152 //184
  .word 0x30202C48 //188
  .word 0x4E415242 //18C
  .word 0x202C4843 //190
  .word 0x49474542 //194
  .word 0x4741204E //198
  .word 0x204E4941 //19C
  .word 0x49544E55 //1A0
  .word 0x4649204C //1A4
  .word 0x444E4520 //1A8
  .word 0x2046495F //1AC
  .word 0x45534C45 //1B0
  .word 0x49485720 //1B4
  .word 0x5220454C //1B8
  .word 0x41455045 //1BC
  .word 0x40432054 //1C0
  .word 0x20214320 //1C4
  .word 0x31202B31 //1C8
  .word 0x2D4D202D //1CC
  .word 0x4C203D20 //1D0
  .word 0x203E2054 //1D4
  .word 0x42202A4D //1D8
  .word 0x42204559 //1DC
  .word 0x20204559 //1E0
  .word 0x5220212B //1E4
  .word 0x523E203E //1E8
  .word 0x2C205220 //1EC
  .word 0x45584520 //1F0
  .word 0x45545543 //1F4
  .word 0x59454B20 //1F8
  .word 0x494D4520 //1FC
  .word 0x48532054 //200
  .word 0x2036314C //204
  .word 0x20474944 //208
  .word 0x45505954 //20C
  .word 0x2E474820 //210
  .word 0x202E4D20 //214
  .word 0x43203F4D //218
  .word 0x6C662052 //21C
  .word 0x662F203E //220
  .word 0x66203E6C //224
  .word 0x2F203E72 //228
  .word 0x203E7266 //22C
  .word 0x4C484546 //230
  .word 0x45545245 //234
  .word 0x44205458 //238
  .word 0x42415349 //23C
  .word 0x7720454C //240
  .word 0x65746965 //244
  .word 0x616E2072 //248
  .word 0x54206863 //24C
  .word 0x65747361 //250
  .word 0x43534520 //254
  .word 0x20455041 //258
  .word 0x52524520 //25C
  .word 0x3F20524F //260
  .word 0x46203F3F //264
  .word 0x454C4845 //268
  .word 0x58455452 //26C
  .word 0x52452054 //270
  .word 0x20524F52 //274
  .word 0x6546202D //278
  .word 0x72656C68 //27C
  .word 0x6D754E20 //280
  .word 0x2072656D //284
  .word 0x50534320 //288
  .word 0x53432021 //28C
  .word 0x4C203F50 //290
  .word 0x4C41434F //294
  .word 0x444E4520 //298
  .word 0x434F4C5F //29C
  .word 0x4C204C41 //2A0
  .word 0x314C2030 //2A4
  .word 0x20324C20 //2A8
  .word 0x4C20334C //2AC
  .word 0x354C2034 //2B0
  .word 0x20364C20 //2B4
  .word 0x2720374C //2B8
  .word 0x434E4920 //2BC
  .word 0x4B203452 //2C0
  .word 0x495F5945 //2C4
  .word 0x4B20544E //2C8
  .word 0x4F435945 //2CC
  .word 0x20324544 //2D0
  .word 0x45505845 //2D4
  .word 0x44205443 //2D8
  .word 0x54494749 //2DC
  .word 0x4D554E20 //2E0
  .word 0x20524542 //2E4
  .word 0x44524F57 //2E8
  .word 0x203D5A20 //2EC
  .word 0x444E4946 //2F0
  .word 0x46434C20 //2F4
  .word 0x4F432041 //2F8
  .word 0x4C49504D //2FC
  .word 0x43202C45 //300
  .word 0x54414552 //304
  .word 0x4E492045 //308
  .word 0x50524554 //30C
  .word 0x20544552 //310
  .word 0x54495551 //314
  .word 0x6B6F2F20 //318
  .word 0x6B6F203E //31C
  .word 0x3E6B6F20 //320
  .word 0x6B6F2F20 //324
  .word 0x6B6F203E //328
  .word 0x41545320 //32C
  .word 0x46205452 //330
  .word 0x5954524F //334
  .word 0x524F462D //338
  .word 0x53204854 //33C
  .word 0x4744554D //340
  .word 0x49282045 //344
  .word 0x44454D4D //348
  .word 0x45544149 //34C
  .word 0x2820293A //350
  .word 0x504D4F43 //354
  .word 0x3A454C49 //358
  .word 0x3A282029 //35C
  .word 0x4D492029 //360
  .word 0x4944454D //364
  .word 0x3A455441 //368
  .word 0x4D4F4320 //36C
  .word 0x454C4950 //370
  .word 0x203A203A //374
  .word 0x474C203B //378
  .word 0x474E202E //37C
  .word 0x2078202E //380
  .word 0x5544202C //384
  .word 0x205A504D //388
  .word 0x54532027 //38C
  .word 0x20545241 //390
  .word 0x4D554420 //394
  .word 0x203E5A50 //398
  .word 0x20202020 //39C
  .word 0x20202D2D //3A0
  .word 0x442F202D //3A4
  .word 0x5A504D55 //3A8
  .word 0x4152203E //3AC

RAM2F00Decode: //37 zu BL RAM2F00Start
STMFD SP!,{R0-R3,LR}
ADD   R0,LR,#4 //37 RAMB2F00 nach 2F00 verschieben
STMEA R12!,{R0}//37 ( ramb2F00 )
MOV   R0,#0X15000 //66
ADD   R0,R0,#0XE00
STMEA R12!,{R0}//37 ( ramb2F00 15E00 )
MOV   R0,#0X80
STMEA R12!,{R0}//37 ( ramb2F00 15E00 80 )
BL    MOVE     //37 ( )
LDMFD SP!,{R0-R3,PC}
RAM2F00Start:
STMFD SP!,{R0-R7,LR}
BL RAM2F00Decode
LDMFD SP!,{R0-R7,PC}
RAM2F00:
  .word 0x00000000 //2F00 CONSTANT XBIT   //2F01 CONSTANT SMUDGEBIT
  .word 0x3C103000 //2F02 CONSTANT RP0 3000 RP0 ! //2F03 CONSTANT IRAMADR
  .word 0x00003C00 //2F04 CONSTANT JRAMADR        //2F05 CONSTANT XOFF
  .word 0x1B5D3AF4 //2F06 CONSTANT CRBZEIG BZEIG @ CRBZEIG !  2F07 CONSTANT CRDP DP @ 8 M- CRDP !

  .word 0x3B000010 //2F08 CONSTANT BASE   2F09 CONSTANT TIB
  .word 0x3B0C3B00 //2F0A CONSTANT IN1    2F0B CONSTANT IN2
  .word 0x3B453B12 //2F0C CONSTANT IN3    2F0D CONSTANT IN4
  .word 0x083A0000 //2F0E CONSTANT ERRORNR 0 ERRORNR ! 2F0F CONSTANT DP
  .word 0x07DD0000 //2F10 CONSTANT STAT 0 STAT !       2F11 CONSTANT LFA
  .word 0x33AE3000 //2F12 CONSTANT BANF   2F13 CONSTANT BZEIG
  .word 0x00200020 //2F14 CONSTANT DPMERK 2F15 CONSTANT CSP
  .word 0x2D000000 //2F16 CONSTANT DUBIT  2F17 CONSTANT LOCALADRESSE
Daten:
