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
//70 FIQ mit KEY speichern
//71 FIQ ARM Timer geht
//72 FIQ geht nur ein und nicht wieder aus, 04 00C MLIT MCODE FIQ
//73 L und N ab 4000 statt vorher E000, Stapelfehler in load_hex_dump
//74 Platz machen ab FIQ und Ausgabe FIQ auf LED GPIO2
//75 FIQ mit UART
//76 ^B wieder zuschalten, in KEY am besten, dann aber FIQ umschalten auf 0
//77 stockend beseitigt mit -1 4 !
//78 warum in FIQ die vielen ADD R10,#1 STR R10 nicht gingen. jetzt gehen sie auf einmal
//79 D+
//80 Start STEP_32
//81 BL RAM100000Start
//82 RP=RP-4 und 2F80-2FFF auf (*4+10000) 1BE00-1BFFF
//83 2900-2FFF auf (*4+10000) 1A400-1BFFF
//84 UI IU und @ ! ab 4000 direkt
//85 RAM100000 vollständig, aus FF.html, von N aus laden weil sonst zu groß
//86 STEP4_32, STEP8_32, STEP9_32, STEPF_32
//87 MINUS EMITCODE M+ AND NOT 0= ORR 0< 
//88 @ und ! unverändert, 4+ 4-
//89 STEPB_32 unverändert wie STEPB
//90 STEPA004_32=STEPA004 KX
//91 U*

//N Neustart ab 4000
//R Neuatart ab 8000
//L Laden auf 4000
//P Laden auf 8000
//M Laden benden
//S Step
//# continue
//^B break


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
mov r3,#5 //75 5 statt 2 laut dwelch67 uart04 //68 2 statt 0
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
BL RQ0000Start //68
BL RAM100000Start //81
MOV R11,#0X10000 //66//29 R11=PC, R12=SP
MOV R10,#0X1C000 //30//45 R10=RP, R11=PC, R12=SP
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
MOVEQ PC,#0X4000     //17 bei "N" Neustart //27 jetzt ab 0XE000 //73 4000
CMP   R6,#0X52       //22 if "R"...
MOVEQ PC,#0X8000     //22 bei "R" Neustart ab 0X8000
CMP   R6,#0X53       //29 if "S"... dann "STEP"
BLEQ  STEP           //29 ( )
CMP   R6,#0X23       //56 if "#"... dann fortsetzen
MOVEQ R9,#0          //56 ( )
BLEQ  STEP           //56 ( )
//str   r6,[r2,#0x40] //7 Zeichen zurückschicken //20 jetzt mit EMIT
mov   r5,#0x4B
STMEA R12!,{R5}// ( "L" )
BL    EMIT     // ( )
cmp   r6,#0x4C      //11 wenn L...
MOVEQ R2,#0X4000    //22//13//18 direkt Programmstart überschreiben //27//73 jetzt ab 0X4000
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
LDMEA R12!,{R0} // ( m )
CMP   R0,#0X30       //14//23
BLT load_hex_dump_2  //14
CMP   R0,#0X4D       //14 if "M"
BEQ load_hex_dump_3  //17
STMEA R12!,{R0} // ( m c )
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

//Speicher
//original    aktuell                     neu                      1000000
//0000-27FF   *2+10000   10000-14FFF                               1000000-...
//2800-28ff   ? ? RP PC
//2900-2EFF   *2+10000   15010-15DFF      *4+10000   1A400-1BBFF
//2F00-2F7F   *2+10000   15E00-15EFF      *4+10000   1BC00-1BDFF
//2F80-2FFF   *2+10000   15F00-15FFF      *4+10000   1BE00-1BFFF
//3000-3FFF   *1+13000   16000-16FFF

STEP: //29 ( --> )
STMFD SP!,{R0-R7,LR}
STEPR:
CMP   R11,#0X100000
BGE   STEP_32
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
STR   R2,[R10,#-4]! //45
SUB   R11,R0,#0X4000
ADD   R11,R11,R11
ADD   R11,R11,#0X10000 //66
B     STEPEND

STEP5:
SUB   R11,R11,#0X10000 //66
LSR   R2,R11,#1
STR   R2,[R10,#-4]! //45
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
B     STEPA021 //D+
B     STEPA002
B     STEPA003
B     STEPA004
B     STEPA005
B     STEPA006
B     STEPA007
B     STEPA008
B     STEPA029
B     STEPA02A
B     STEPA02B
B     STEPA02C
B     STEPA02D
B     STEPA00E
B     STEPA00F

STEPA000:
//0A 000 MLIT MCODE MINUS
LDMEA R12!,{R0} // ( a --> )
SUB   R0,R1,R0
STMEA R12!,{R0} // ( --> -a )
B     STEPEND

STEPA00C:
//0A 00C MLIT MCODE FIQ
LDMEA R12!,{R1} // ( bic --> )
    mrs r0,cpsr       //69 Enable FIQ https://github.com/dwelch67/raspberrypi/blinker08
    bic r0,r0,R1      //72//69
    msr cpsr_c,r0     //69
STMEA R12!,{R0} // ( --> cpsr_neu )
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
CMP   R0,#0X02
BNE   KEYCODE02
MOV   R9,#1     //56 vorher MOVEQ R9,R8
MOV   R0,   #0X20000000  //75 PBASE
ADD   R0,R0,#0X0000B200  //75 Interrupt
MOV   R1,#0
STR   R1,[R0,#0x0C]    //75 FIQ=0
B     STEPA004
KEYCODE02:
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

STEPA001:
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
CMP   R1,#0X4000 //84
BGE   STORE4000
CMP   R1,#0X3000 //43
ADDGE R1,#0X13000 //65
STRGEB  R0,[R1]
BGE     STEPEND
CMP   R1,#0X2900 //82
BGE   STORE2F00
CMP   R1,#0X2800
BGE   STORE2
ADD   R1,R1,R1
ADD   R1,R1,#0X10000 //66
STRH  R0,[R1]
B     STEPEND
STORE4000:
STR   R0,[R1]
B     STEPEND
STORE2F00:
ADD   R1,R1,R1
ADD   R1,R1,R1
ADD   R1,R1,#0X10000
STR   R0,[R1]
B     STEPEND
STORE2:
AND   R1,R1,#0XFF
CMP   R1,#3
LSLEQ R0,R0,#1
ADDEQ R11,R0,#0X10000 //66
BEQ   STEPEND
CMP   R1,#2
LSLEQ R0,R0,#2 //45
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

STEPA00A:
//0A 00A MLIT MCODE @
LDMEA R12!,{R1} //34 ( adr --> )
CMP   R1,#0X4000 //84
BGE   FETCH4000
LSL   R1,R1,#16 //48
LSR   R1,R1,#16 //48
CMP   R1,#0X3000
ADDGE R1,#0X13000  //65
LDRGEB  R3,[R1]
STMGEEA R12!,{R3} //34 ( --> n )
BGE     STEPEND
CMP   R1,#0X2900
BGE   FETCH2F00
CMP   R1,#0X2800
BGE   FETCH2
ADD   R1,R1,R1
ADD   R1,R1,#0X10000 //66
LDRH  R3,[R1]
AND   R1,R3,#0X8000  //52 Vorzeichenbits
CMP   R1,#0
ADDNE R3,#0XFF0000
ADDNE R3,#0XFF000000
STMEA R12!,{R3} //34 ( --> n )
B     STEPEND
FETCH4000:
LDR   R3,[R1]
STMEA R12!,{R3} // ( --> n )
B     STEPEND
FETCH2F00:
ADD   R1,R1,R1
ADD   R1,R1,R1
ADD   R1,R1,#0X10000 //66
LDR   R3,[R1]
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
LSREQ R0,R2,#2
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

STEPA021:
//0A 021 MLIT MCODE D+
LDMEA R12!,{R0,R1,R2,R3} //79 ( ah al bh bl --> (a+b)h (a+b)l )
LSL   R0,R0,#16
LSL   R1,R1,#16
LSR   R1,R1,#16
ADD   R4,R0,R1
LSL   R2,R2,#16
LSL   R3,R3,#16
LSR   R3,R3,#16
ADD   R5,R2,R3
ADD   R4,R4,R5
LSL   R5,R4,#16
LSR   R5,R5,#16
LSR   R4,R4,#16
STMEA R12!,{R4,R5} //57 ( (a+b)h (a+b)l )
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
STMEA R12!,{R0,R1} //57 ( a b )
B     STEPEND

STEPA02B://80
//0A 02B MLIT MCODE 100000EXECUTE
MOV   R11,#0X100000
B     STEPEND

STEPA02C://84
//0A 02C MLIT MCODE UI ( ah al --> a )
LDMEA R12!,{R2,R3} 
LSL   R4,R2,#16
LSL   R3,R3,#16
LSR   R3,R3,#16
ADD   R4,R4,R3
STMEA R12!,{R4} //( a )
B     STEPEND

STEPA02D://84
//0A 02A MLIT MCODE IU ( a --> ah al )
LDMEA R12!,{R2}
LSR   R0,R2,#16
LSL   R1,R2,#16
LSR   R1,R1,#16
STMEA R12!,{R0,R1} //57 ( ah al )
B     STEPEND

STEPA003:
//( 0A 003 MLIT MCODE RETURN )
LDR   R2,[R10],#4 //44
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
MOV   R2,#0X10000 //77 kein KEY mehr nach -1 4 !
ADD   R2,R2,#8
LDR   R1,[R2]
CMP   R1,#0
BNE   STEPEND0
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
LDR   R0,[R10]
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

RQ0000Decode: //70//43 zu BL RQ0000Start
STMFD SP!,{R0-R3,LR}
ADD   R0,LR,#4 //28 RAMB03C00 nach 3C00 verschieben
STMEA R12!,{R0}//28 ( ramb0000 )
MOV   R0,#0X00
STMEA R12!,{R0}//28 ( ramb0000 0 )
MOV   R0,#0X80 //70
STMEA R12!,{R0}//28 ( ramb3C00 0 80 )
BL    MOVE     //28 ( )
LDMFD SP!,{R0-R3,PC}
RQ0000Start:
STMFD SP!,{R0-R7,LR}
BL RQ0000Decode
LDMFD SP!,{R0-R7,PC}
B     FIQ //0X00
B     FIQ //0X04
B     FIQ //0X08
B     FIQ //0X0C
B     FIQ //0X10
B     FIQ //0X14
B     FIQ //0X18
B     FIQ //0X1C
FIQ:      //0X20
//MOV   R8,#0X20000000 //71 ARM Timer IRQ clear
//ADD   R8,R8, #0XB400
//ADD   R8,R8,   #0X0C
//MOV   R9,#0
//STR   R9,[R8]
//MOV   R8,#0X10000
//ADD   R8,R8,#0X10
//LDR   R9,[R8]
//ADD   R9,R9,#1
//STR   R9,[R8]

//MOV   R8,   #0X20000000  //75 PBASE
//ADD   R8,R8,#0X0000B200  //75 ARM timer
//MOV   R9,#0
//STR   R9,[R8,#0x0C]    //75 FIQ=ARM timer
MOV   R8,#0X20000000     //70 PBASE
ADD   R8,R8,#0X210000    //70 GPIO_
ADD   R8,R8,#0X5000     //70 AUX_IRQ
//MOV   R9,#0
//STR   R9,[R8,#0x44]    //  "Disable receive and transmit interrupts"
LDR   R10,[R8,#0x40]     //70 AUX_MU_IO_REG
//STR   R10,[R8,#0x40]     //70 AUX_MU_IO_REG
//STR   R10,[R8,#0x40]     //70 AUX_MU_IO_REG
MOV   R8,#0X10000
ADD   R8,R8,#4
LDR   R9,[R8]
ADD   R9,R9,#0X10000000 //65//62
STRB  R10,[R9]
ADD   R9,R9,#1
SUB   R9,R9,#0X10000000 //65//62
STR   R9,[R8]

MOV   R8,#0X20000000 //74 GPIO
ADD   R8,R8,#0X200000
MOV   R11,#4 //Bitposition für GPIO2
AND   R10,R9,#1
CMP   R10,#1
STREQ R11,[R8,#40]
STRNE R11,[R8,#28]

SUBS  PC,R14,#4

//CMP   R10,#0X02
//MOVEQ R9,#1
//BEQ   STEPEND0


MOV   R8,#0X20000000     //70 PBASE
ADD   R8,R8,#0X210000    //70 GPIO_
ADD   R8,R8,#0X5000     //70 AUX_IRQ
UART5048:
LDR   R10,[R8,#0x48]     //70 AUX_MU_II_REG
AND   R10,R10,#1
CMP   R10,#0
BEQ   UART5048

SUBS  PC,R14,#4


//BL    KEY
MOV   R8,#0X20000000     //70 PBASE
ADD   R8,R8,#0X200000    //70 GPIO_
ADD   R8,R8,#0X15000     //70 AUX_IRQ
LDR   R10,[R8,#0x40]     //70 AUX_MU_IO_REG
//CMP   R10,#0X02
//MOVEQ R9,#1
//BEQ   STEPEND0

MOV   R8,#0X10000
ADD   R8,R8,#4
LDR   R9,[R8]
ADD   R9,R9,#0X10000000
STRB  R10,[R9]
ADD   R9,R9,#1
SUB   R9,R9,#0X10000000
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
MOV   R0,#0X1B000 //66
ADD   R0,R0,#0XC00
STMEA R12!,{R0}//37 ( ramb2F00 1BC00 )
MOV   R0,#0X100
STMEA R12!,{R0}//37 ( ramb2F00 1BC00 100 )
BL    MOVE     //37 ( )
LDMFD SP!,{R0-R3,PC}
RAM2F00Start:
STMFD SP!,{R0-R7,LR}
BL RAM2F00Decode
LDMFD SP!,{R0-R7,PC}
RAM2F00:
  .word 0x0000 //2F00 CONSTANT XBIT
  .word 0x0000 //2F01 CONSTANT SMUDGEBIT
  .word 0x3000 //2F02 CONSTANT RP0 3000 RP0 !
  .word 0x3C10 //2F03 CONSTANT IRAMADR
  .word 0x3C00 //2F04 CONSTANT JRAMADR
  .word 0x0000 //2F05 CONSTANT XOFF
  .word 0x3AF4 //2F06 CONSTANT CRBZEIG BZEIG @ CRBZEIG !
  .word 0x1B5D //2F07 CONSTANT CRDP DP @ 8 M- CRDP !
  .word 0x0010 //2F08 CONSTANT BASE
  .word 0x3B00 //2F09 CONSTANT TIB
  .word 0x3B00 //2F0A CONSTANT IN1
  .word 0x3B0C //2F0B CONSTANT IN2
  .word 0x3B12 //2F0C CONSTANT IN3
  .word 0x3B45 //2F0D CONSTANT IN4
  .word 0x0000 //2F0E CONSTANT ERRORNR 0 ERRORNR !
  .word 0x083A //2F0F CONSTANT DP
  .word 0x0000 //2F10 CONSTANT STAT 0 STAT !
  .word 0x07DD //2F11 CONSTANT LFA
  .word 0x3000 //2F12 CONSTANT BANF
  .word 0x33AE //2F13 CONSTANT BZEIG
  .word 0x0020 //2F14 CONSTANT DPMERK
  .word 0x0020 //2F15 CONSTANT CSP
  .word 0x0000 //2F16 CONSTANT DUBIT
  .word 0x2D00 //2F17 CONSTANT LOCALADRESSE

STEP_32: //
LDR   R0,[R11]
ADD   R8,R8,#1
ADD   R11,R11,#4
AND   R1,R0,#0XF0000000
LSR   R2,R1,#26
MOV   R3,PC
ADD   PC,R3,R2

STEPVECTOR_32:
B     STEP0123_32
B     STEP0123_32
B     STEP0123_32
B     STEP0123_32
B     STEP4_32
B     STEP5_32
B     STEP6_32
B     STEP7_32
B     STEP8_32
B     STEP9_32
B     STEPA_32
B     STEPB_32
B     STEPF_32
B     STEPF_32
B     STEPF_32
B     STEPF_32

//#########################################################################################
//  else switch (PD>>>28) {
//    case 0xA: switch (PD&0xFF) {
//      case 0x01: /* U+ */ alert(STEP.toHex0000()+" "+PD.toHex0000()); break;
//      case 0x06: /*  */ alert(STEP.toHex0000()+" "+PD.toHex0000()); break;
//      case 0x0C: /*  */ alert(STEP.toHex0000()+" "+PD.toHex0000()); break;
//      case 0x21: /* D+ */ 
//        var A=(STAPEL[ST-4]<<16)+STAPEL[ST-3]; 
//        var B=(STAPEL[ST-2]<<16)+STAPEL[ST-1]; 
//        var C=A+B;
//        STAPEL[ST-4]=C>>>16;
//        STAPEL[ST-3]=C&0xFFFF;
//        ST=ST-2; 
//        break;
//      case 0x27: /* N+ */ STAPEL[ST-2]=STAPEL[ST-2]+STAPEL[ST-1]; ST=ST-1; break;
//      case 0x29: /* 2! */ switch (STAPEL[ST-2]) {
//        case 0x2020: RAMB20200000[STAPEL[ST-1]]=STAPEL[ST-3]+0x10000*STAPEL[ST-4]; ST=ST-4; break;
//        default: alert(PD.toHex0000()); Startarray=[]; return; 
//        } break;
//      case 0x2A: /* 2@ */ switch (STAPEL[ST-2]) {
//        case 0x2020: alert(RAMB20200000);STAPEL[ST-2]=(RAMB20200000[STAPEL[ST-1]]&0xFFFF0000)>>>16; STAPEL[ST-1]=RAMB20200000[STAPEL[ST-1]]&0xFFFF; break;
//        case 0x0010: //alert(RAMB100000[STAPEL[ST-1]/4].toHex0000());
//          if (STAPEL[ST-1]&0x3) alert(STAPEL[ST-1].toHex0000());
//          STAPEL[ST-2]=(RAMB100000[STAPEL[ST-1]/4]&0xFFFF0000)>>>16; 
//          STAPEL[ST-1]=RAMB100000[STAPEL[ST-1]/4]&0xFFFF; 
//          break;
//        default: alert(PD.toHex0000()); Startarray=[]; return; 
//        } break;
//      case 0x2B: /* 100000 EXECUTE */ RP=RP-1; RAMB0000[RP]=PC; PC=0X100000; break;
//      case 0x2C: /* UI */ STAPEL[ST-2]=STAPEL[ST-2]*0x10000+STAPEL[ST-1]; ST=ST-1; break;
//      case 0x2D: /* IU */ STAPEL[ST]=STAPEL[ST-1]&0xFFFF; STAPEL[ST-1]=STAPEL[ST-1]>>>16; ST=ST+1; break;
//      default: alert(PD.toHex0000()); Startarray=[]; return; break;
//      } break;
//    default: /*alert("num="+(PD&0X0FFFFFFF).toHex0000());*/
//      STAPEL[ST]=PD; ST=ST+1;
//      break;

STEP0123_32:
STEPF_32:
STMEA R12!,{R0} //32 ( --> n )
MOV   R9,#1
B     STEPEND

STEP4_32: //86
STEP5_32:
STEP6_32:
STEP7_32:
//    case 0x4: RP=RP-1; RAMB0000[RP]=PC; PC=(PD&0X3FFFFFFF); break;
//    case 0x5: RP=RP-1; RAMB0000[RP]=PC; PC=(PD&0X3FFFFFFF); break;
//    case 0x6: RP=RP-1; RAMB0000[RP]=PC; PC=(PD&0X3FFFFFFF); break;
//    case 0x7: RP=RP-1; RAMB0000[RP]=PC; PC=(PD&0X3FFFFFFF); break;
//SUB   R11,R11,#0X10000 //66
//LSR   R2,R11,#1
//STR   R2,[R10,#-4]! //45
//SUB   R11,R0,#0X4000
//ADD   R11,R11,R11
//ADD   R11,R11,#0X10000 //66
//B     STEPEND
STR   R11,[R10,#-4]!
SUB   R11,R0,#0X40000000
B     STEPEND

STEP8_32:
//    case 0x8: if ((PD&0x0FFFFFFF)<0x08000000) PC=PC+(PD&0X0FFFFFFF); else PC=PC+(PD&0X0FFFFFFF)-0x10000000; break;
//SUB   R0,R0,#0X8000
//CMP   R0,#0X0800
//MVNGE R1,#0
//LSLGE R1,#12
//ADDGE R0,R0,R1
//ADD   R0,R0,R0
//ADD   R11,R11,R0
//B     STEPEND
SUB   R0,R0,#0X80000000
ADD   R11,R11,R0
CMP   R0,#0X08000000
ADDGE R11,R11,#0XF0000000
B     STEPEND

STEP9_32:
//    case 0x9: if (STAPEL[ST-1]==0) {if ((PD&0x0FFFFFFF)<0x08000000) PC=PC+(PD&0X0FFFFFFF); else PC=PC+(PD&0X0FFFFFFF)-0x10000000} ST=ST-1; break;
//LDMEA R12!,{R2}
//CMP   R2,#0
//BNE   STEPEND
//SUB   R0,R0,#0X9000
//.. weiter wie STEP8_32
LDMEA R12!,{R2}
CMP   R2,#0
BNE   STEPEND
SUB   R0,R0,#0X90000000
ADD   R11,R11,R0
CMP   R0,#0X08000000
ADDGE R11,R11,#0XF0000000
B     STEPEND

STEPA_32:
AND   R1,R0,#0X00FF
LSL   R2,R1,#2
MOV   R3,PC
ADD   PC,R3,R2

STEPAVECTOR_32:
B     STEPA000_32
B     STEPA001_32
B     STEPA002_32
B     STEPA003_32
B     STEPA004_32
B     STEPA005_32
B     STEPA006_32
B     STEPA007_32
B     STEPA008_32
B     STEPA009_32
B     STEPA00A_32
B     STEPA00B_32
B     STEPA00C_32
B     STEPA00D_32
B     STEPA00E_32
B     STEPA00F_32
B     STEPA000_32
B     STEPA001_32
B     STEPA002_32
B     STEPA003_32
B     STEPA004_32
B     STEPA005_32
B     STEPA006_32
B     STEPA007_32
B     STEPA008_32
B     STEPA009_32
B     STEPA00A_32
B     STEPA00B_32
B     STEPA00C_32
B     STEPA00D_32
B     STEPA00E_32
B     STEPA00F_32
B     STEPA000_32
B     STEPA021_32 //D+
B     STEPA002_32
B     STEPA003_32
B     STEPA004_32
B     STEPA005_32
B     STEPA006_32
B     STEPA007_32
B     STEPA008_32
B     STEPA029_32
B     STEPA02A_32
B     STEPA00B_32
B     STEPA00C_32
B     STEPA00D_32
B     STEPA02E_32
B     STEPA02F_32


STEPA000_32: //87
//      case 0x00: /* MINUS */ STAPEL[ST-1]=-STAPEL[ST-1]; break;
//0A 000 MLIT MCODE MINUS
//LDMEA R12!,{R0} // ( a --> )
//SUB   R0,R1,R0
//STMEA R12!,{R0} // ( --> -a )
//B     STEPEND
LDMEA R12!,{R0} // ( a --> )
SUB   R0,R1,R0
STMEA R12!,{R0} // ( --> -a )
B     STEPEND

STEPA001_32:

STEPA002_32: //91
//      case 0x02: /* U* ( c b a -- (a*b+c)_high (a*b+c)_low ) unsigned */  
//        //alert(STEP.toHex0000()+" "+PD.toHex0000()+" "+STAPEL[ST-3].toHex0000()+" "+STAPEL[ST-2].toHex0000()+" "+STAPEL[ST-1].toHex0000());
//        var M=(BigInt(STAPEL[ST-3])&0xFFFFFFFFn)+(BigInt(STAPEL[ST-2])&0xFFFFFFFFn)*(BigInt(STAPEL[ST-1])&0xFFFFFFFFn); 
////alert(M);alert(">>>"+(M>>BigInt(16)));
//        STAPEL[ST-3]=Number("0x"+((M&0xFFFFFFFF00000000n)>>32n).toHex0000());
////alert(STAPEL[ST-3]);
//        STAPEL[ST-2]=Number("0x"+(M&BigInt(0xFFFFFFFF)).toHex0000());
////alert(STAPEL[ST-2]);
//        ST=ST-1;
//        break; 
//0A 002 MLIT MCODE U*
//LDMEA R12!,{R0,R1,R2} // ( c b a --> (a*b+c)h (a*b+c)l )
//MOV   R4,#0X10000
//SUB   R4,R4,#1
//AND   R0,R0,R4
//AND   R1,R1,R4
//AND   R2,R2,R4
//MLA   R3,R2,R1,R0
//LSR   R2,R3,#16
//STMEA R12!,{R2} // ( --> (a*b+c)h )
//AND   R3,R3,R4
//STMEA R12!,{R3} // ( --> (a*b+c)h (a*b+c)l )
//B     STEPEND
LDMEA R12!,{R0,R2,R3} // ( c b a --> (a*b+c)h (a*b+c)l )
MOV   R1,#0
UMLAL R0,R1,R2,R3     //UMLAL{<cond>}{S} <RdLo>, <RdHi>, <Rm>,  <Rs>
STMEA R12!,{R1,R0}    // ( --> (a*b+c)h (a*b+c)l )
B     STEPEND

STEPA003_32:
//      case 0x03: /* RETURN */ PC=RAMB0000[RP]; RP=RP+1; break;
//( 0A 003 MLIT MCODE RETURN )
LDR   R2,[R10],#4 //44
CMP   R2,#0X100000
MOVGE R11,R2
BGE   STEPEND
LSL   R11,R2,#1
ADD   R11,R11,#0X10000 //66
MOV   R9,#0
B     STEPEND

STEPA004_32: //90
//      case 0x04: /*  alert("case 0x04: "+PC.toHex0000()+" "+PD.toHex0000()+" "+RAMB0000[0x2F03]+" "+RAMB0000[0x2F04]+" "+STEP+" "+ENDSTEP); */ if (RAMB0000[0x2F03]==RAMB0000[0x2F04]) STEP=ENDSTEP; break;
B STEPA004

STEPA005_32: //87
//      case 0x05: /* EMITCODE */  EText=EText+String.fromCharCode(STAPEL[ST-1]); Logtext=Logtext+String.fromCharCode(STAPEL[ST-1]); ST=ST-1; break;
//0A 005 MLIT MCODE EMITCODE
BL    EMIT        //34 ( c -->  <char> )
B     STEPEND

STEPA006_32:

STEPA007_32: //87
//      case 0x07: /* M+ */ STAPEL[ST-2]=(STAPEL[ST-2]+STAPEL[ST-1])&0xFFFFFFFF;  ST=ST-1; break;
//0A 007 MLIT MCODE M+
LDMEA R12!,{R0,R1} //35 ( a b --> )
ADD   R0,R0,R1
STMEA R12!,{R0} //35 ( --> a+b )
B     STEPEND

STEPA008_32: //87
//      case 0x08: /* AND */ STAPEL[ST-2]=STAPEL[ST-2]&STAPEL[ST-1]; ST=ST-1; break;
//0A 008 MLIT MCODE AND
LDMEA R12!,{R0,R1} // ( a b --> )
AND   R0,R0,R1
STMEA R12!,{R0} // ( --> a_and_b )
B     STEPEND

STEPA009_32: //88
//      case 0x09: /* ! */ switch (STAPEL[ST-1]) {
//        case 0x2803: PC=STAPEL[ST-2]; ST=ST-2; break;
//        case 0x2802: RP=STAPEL[ST-2]; ST=ST-2; break;
//        case 0x2801: ST=STAPEL[ST-2]; break;
//        default: switch (STAPEL[ST-1]>>>16) {
//          case 0x2020: RAMB20200000[(STAPEL[ST-1]&0xFFFF)/4]=STAPEL[ST-2]; ST=ST-2; break;
//          case 0x0010: 
//            if (STAPEL[ST-1]&0x3) alert("! PC="+(((PC-4)&0xFFFFF)/4).toHex0000()+" STAPEL[ST-1]="+STAPEL[ST-1].toHex0000());
//            RAMB100000[(STAPEL[ST-1]&0xFFFF)/4]=STAPEL[ST-2]; ST=ST-2; break;
//          default: RAMB0000[STAPEL[ST-1]]=STAPEL[ST-2]; ST=ST-2; break;
//          }
//        } break;
B STEPA009

STEPA00A_32: //88
//      case 0x0A: /* @ */ switch (STAPEL[ST-1]) {
//        case 0x2803: STAPEL[ST-1]=PC; break;
//        case 0x2802: STAPEL[ST-1]=RP; break;
//        case 0x2801: STAPEL[ST-1]=ST; break;
//        default: switch (STAPEL[ST-1]>>>16) {
//          case 0x2020: STAPEL[ST-1]=RAMB20200000[(STAPEL[ST-1]&0xFFFF)/4]; break;
//          case 0x0010:
//            if (STAPEL[ST-1]&0x3) alert("@ PC="+(((PC-4)&0xFFFFF)/4).toHex0000()+" STAPEL[ST-1]="+STAPEL[ST-1].toHex0000());
//            STAPEL[ST-1]=RAMB100000[(STAPEL[ST-1]&0xFFFF)/4]; break;
//          default: STAPEL[ST-1]=RAMB0000[STAPEL[ST-1]]; break;
//          }
//        } break;
B STEPA00A

STEPA00B_32: //87
//      case 0x0B: /* NOT */ STAPEL[ST-1]=-1-STAPEL[ST-1]; break;
//0A 00B MLIT MCODE NOT
LDMEA R12!,{R0} // ( a --> )
MVN   R0,R0
STMEA R12!,{R0} // ( --> not(a) )
B     STEPEND

STEPA00C_32:

STEPA00D_32: //87
//      case 0x0D: /*  0= */ if (STAPEL[ST-1]==0) STAPEL[ST-1]=-1; else STAPEL[ST-1]=0; break;
//0A 00D MLIT MCODE 0=
LDMEA R12!,{R0} // ( a --> )
CMP   R0,#0
MVNEQ R0,#0
MOVNE R0,#0
STMEA R12!,{R0} // ( --> 0= )
B     STEPEND

STEPA00E_32: //87
//      case 0x0E: /* ORR */ STAPEL[ST-2]=STAPEL[ST-2]|STAPEL[ST-1]; ST=ST-1; break;
//0A 00E MLIT MCODE OR
LDMEA R12!,{R0,R1} //( a b --> )
ORR   R0,R0,R1
STMEA R12!,{R0} // ( --> a_or_b )
B     STEPEND

STEPA00F_32: //87
//      case 0x0F: /*  0< */ if (STAPEL[ST-1]<0) STAPEL[ST-1]=-1; else STAPEL[ST-1]=0; break;
//0A 00F MLIT MCODE 0LT
LDMEA R12!,{R0} // ( a --> )
CMP   R0,#0
MVNLT R0,#0
MOVGE R0,#0
STMEA R12!,{R0} // ( --> 0LT )
B     STEPEND

STEPA021_32: //D+
STEPA029_32:
STEPA02A_32:

STEPA02E_32: //88
//      case 0x2E: /* 4+ */ STAPEL[ST-1]=STAPEL[ST-1]+4; break;
LDMEA R12!,{R0} // ( a --> )
ADD   R0,R0,#4
STMEA R12!,{R0} // ( --> a+4 )
B     STEPEND

STEPA02F_32: //88
//      case 0x2F: /* 4- */ STAPEL[ST-1]=STAPEL[ST-1]-4; break;
LDMEA R12!,{R0} // ( a --> )
SUB   R0,R0,#4
STMEA R12!,{R0} // ( --> a-4 )
B     STEPEND

STEPB_32: //89
//    case 0xB: switch (PD&0xFFF) {
//      case 0x200: /* 2DROP */ ST=ST-2; break;
//      case 0x300: /* DROP */ ST=ST-1; break;
//      case 0x412: /* SWAP */ PD=STAPEL[ST-2]; STAPEL[ST-2]=STAPEL[ST-1]; STAPEL[ST-1]=PD; break;
//      case 0x434: /*  ROT */ PD=STAPEL[ST-3]; STAPEL[ST-3]=STAPEL[ST-2]; STAPEL[ST-2]=STAPEL[ST-1]; STAPEL[ST-1]=PD; break;
//      case 0x43C: /* 2SWAP */ PD=[STAPEL[ST-4],STAPEL[ST-3]]; STAPEL[ST-4]=STAPEL[ST-2]; STAPEL[ST-3]=STAPEL[ST-1]; STAPEL[ST-2]=PD[0]; STAPEL[ST-1]=PD[1]; break;
//      case 0x501: /*  DUP */ STAPEL[ST]=STAPEL[ST-1]; ST=ST+1; break;
//      case 0x502: /* OVER */ STAPEL[ST]=STAPEL[ST-2]; ST=ST+1; break;
//      case 0x603: /* 2DUP */ STAPEL[ST]=STAPEL[ST-2]; STAPEL[ST+1]=STAPEL[ST-1]; ST=ST+2; break;
//      case 0x60C: /* 2OVER */ STAPEL[ST]=STAPEL[ST-4]; STAPEL[ST+1]=STAPEL[ST-3]; ST=ST+2; break;
//      } break;
B STEPB

RAM100000Decode: //37 zu BL RAM2F00Start
STMFD SP!,{R0-R3,LR}
ADD   R0,LR,#4 //37 RAMB100000 nach 100000 verschieben
STMEA R12!,{R0}//37 ( ramb100000 )
MOV   R0,#0X100000 //66
ADD   R0,R0,#0X000
STMEA R12!,{R0}//37 ( ramb100000 100000 )
MOV   R0,#0X4000
STMEA R12!,{R0}//37 ( ramb100000 100000 4000 )
BL    MOVE     //37 ( )
LDMFD SP!,{R0-R3,PC}
RAM100000Start:
STMFD SP!,{R0-R7,LR}
BL RAM100000Decode
LDMFD SP!,{R0-R7,PC}
RAM100000:
  .word 0x00000123 //00 
  .word 0x00000234 //02
  .word 0x40100010 //04 CR
  .word 0xA0000003 //06
  .word 0x00000345 //08
  .word 0x80000008 //0A JR
  .word 0x00000567 //0C
  .word 0xA0000003 //0E
  .word 0x00000456 //10
  .word 0x90000004 //12 456 JR0
  .word 0x00000567 //14
  .word 0x00000000 //16
  .word 0x90000004 //18 000 JR0
  .word 0x00000765 //1A
  .word 0x00000678 //1C
  .word 0xA0000000 //1E MINUS
  .word 0x0000007A //20
  .word 0xA0000005 //22 EMITCODE
  .word 0x00000077 //24
  .word 0x00000088 //26
  .word 0xA0000007 //28 M+
  .word 0x000000F1 //2A
  .word 0xA0000008 //2C AND
  .word 0xA000000B //2E NOT
  .word 0xA000000D //30 0=
  .word 0xA000000D //32 0=
  .word 0x00001234 //34
  .word 0x00008888 //36
  .word 0xA000000E //38 ORR
  .word 0xA000000F //3A 0<
  .word 0xF0008888 //3C
  .word 0xA000000F //3E 0<
  .word 0x00100000 //40
  .word 0xA000000A //42 @
  .word 0x00000321 //44
  .word 0x00100000 //46
  .word 0xA0000009 //48 !S
  .word 0x00100000 //4A
  .word 0xA000000A //4C @
  .word 0xA000002E //4E 4+
  .word 0xA000002F //50 4-
  .word 0xB0000412 //52 SWAP
  .word 0xB0000502 //54 OVER
  .word 0xB0000501 //56 DUP
  .word 0xB0000300 //58 DROP
  .word 0xB000043C //5A 2SWAP
  .word 0xB000060C //5C 2OVER
  .word 0xB0000603 //5E 2DUP
  .word 0xB0000200 //60 2DROP
  .word 0x00000001 //62
  .word 0x23456789 //64
  .word 0x00000010 //66
  .word 0xA0000002 //68 U*
  .word 0x8FFFFF40 //6A


//85
  .word  0x40100040 //0000 0000 40100040
  .word  0xA0000003 //0004 0001 A0000003
  .word  0x00000000 //0008 0002 0000
  .word  0x00000000 //000C 0003 0000
  .word  0x00000000 //0010 0004 0000
  .word  0x00000000 //0014 0005 0000
  .word  0x00000000 //0018 0006 0000
  .word  0x00000000 //001C 0007 0000
  .word  0x00000000 //0020 0008 0000
  .word  0x00000000 //0024 0009 0000
  .word  0x00000000 //0028 000A 0000
  .word  0x00000000 //002C 000B 0000
  .word  0x00000000 //0030 000C 0000
  .word  0x00000000 //0034 000D 0000
  .word  0x00000000 //0038 000E 0000
  .word  0x00000000 //003C 000F 0000
  .word  0x40101D80 //0040 0010 40101D80
  .word  0xA0000003 //0044 0011 A0000003
  .word  0x401011F4 //0048 0012 401011F4
  .word  0xA0000003 //004C 0013 A0000003
  .word  0x40100000 //0050 0014 40100000
  .word  0xA0000003 //0054 0015 A0000003
  .word  0x40100000 //0058 0016 40100000
  .word  0xA0000003 //005C 0017 A0000003
  .word  0x40100000 //0060 0018 40100000
  .word  0xA0000003 //0064 0019 A0000003
  .word  0x40100000 //0068 001A 40100000
  .word  0xA0000003 //006C 001B A0000003
  .word  0x40100000 //0070 001C 40100000
  .word  0xA0000003 //0074 001D A0000003
  .word  0x00000010 //0078 001E 0010
  .word  0xA0000003 //007C 001F A0000003
  .word  0x00000000 //0080 0020 0000
  .word  0x00003000 //0084 0021 3000
  .word  0x00000001 //0088 0022 0001
  .word  0x40101DCC //008C 0023 40101DCC
  .word  0x00000029 //0090 0024 0029
  .word  0x40101790 //0094 0025 40101790
  .word  0xB0000200 //0098 0026 B0000200
  .word  0xA0000003 //009C 0027 A0000003
  .word  0xFFFFFFE0 //00A0 0028 -20
  .word  0x00003002 //00A4 0029 3002
  .word  0x00000001 //00A8 002A 0001
  .word  0x40101DCC //00AC 002B 40101DCC
  .word  0x00000000 //00B0 002C 0000
  .word  0x00002F10 //00B4 002D 2F10
  .word  0xA0000009 //00B8 002E A0000009
  .word  0xA0000003 //00BC 002F A0000003
  .word  0xFFFFFFE0 //00C0 0030 -20
  .word  0x00003004 //00C4 0031 3004
  .word  0x00000001 //00C8 0032 0001
  .word  0x40101E04 //00CC 0033 40101E04
  .word  0x00000001 //00D0 0034 0001
  .word  0x00002F10 //00D4 0035 2F10
  .word  0xA0000009 //00D8 0036 A0000009
  .word  0xA0000003 //00DC 0037 A0000003
  .word  0xFFFFFFE0 //00E0 0038 -20
  .word  0x00003006 //00E4 0039 3006
  .word  0x00000007 //00E8 003A 0007
  .word  0x40101DCC //00EC 003B 40101DCC
  .word  0x00000020 //00F0 003C 0020
  .word  0x40101790 //00F4 003D 40101790
  .word  0x40101978 //00F8 003E 40101978
  .word  0x40101A78 //00FC 003F 40101A78
  .word  0xB0000300 //0100 0040 B0000300
  .word  0x40101A9C //0104 0041 40101A9C
  .word  0xA0000003 //0108 0042 A0000003
  .word  0xFFFFFFD4 //010C 0043 -2C
  .word  0x0000300E //0110 0044 300E
  .word  0x00000004 //0114 0045 0004
  .word  0x40101E04 //0118 0046 40101E04
  .word  0xB0000412 //011C 0047 B0000412
  .word  0x10000000 //0120 0048 10000000
  .word  0xA0000002 //0124 0049 A0000002
  .word  0xB0000412 //0128 004A B0000412
  .word  0xB0000300 //012C 004B B0000300
  .word  0xA0000003 //0130 004C A0000003
  .word  0xFFFFFFD8 //0134 004D -28
  .word  0x00003013 //0138 004E 3013
  .word  0x00000003 //013C 004F 0003
  .word  0x40101E04 //0140 0050 40101E04
  .word  0xB0000501 //0144 0051 B0000501
  .word  0xA000000F //0148 0052 A000000F
  .word  0x90000004 //014C 0053 90000004
  .word  0xA0000000 //0150 0054 A0000000
  .word  0xA0000003 //0154 0055 A0000003
  .word  0xFFFFFFDC //0158 0056 -24
  .word  0x00003017 //015C 0057 3017
  .word  0x00000004 //0160 0058 0004
  .word  0x40101E04 //0164 0059 40101E04
  .word  0xB0000501 //0168 005A B0000501
  .word  0x40100144 //016C 005B 40100144
  .word  0x00000004 //0170 005C 0004
  .word  0x00000000 //0174 005D 0000
  .word  0x4010011C //0178 005E 4010011C
  .word  0xA0000008 //017C 005F A0000008
  .word  0x90000008 //0180 0060 90000008
  .word  0x00000111 //0184 0061 0111
  .word  0x40100F08 //0188 0062 40100F08
  .word  0x40100C00 //018C 0063 40100C00
  .word  0xA0000003 //0190 0064 A0000003
  .word  0xFFFFFFC4 //0194 0065 -3C
  .word  0x0000301C //0198 0066 301C
  .word  0x0000000B //019C 0067 000B
  .word  0x40101DE8 //01A0 0068 40101DE8
  .word  0x40100B3C //01A4 0069 40100B3C
  .word  0xA000000A //01A8 006A A000000A
  .word  0x00002F10 //01AC 006B 2F10
  .word  0xA000000A //01B0 006C A000000A
  .word  0x90000004 //01B4 006D 90000004
  .word  0x40100168 //01B8 006E 40100168
  .word  0xA0000003 //01BC 006F A0000003
  .word  0xFFFFFFD4 //01C0 0070 -2C
  .word  0x00003028 //01C4 0071 3028
  .word  0x00000008 //01C8 0072 0008
  .word  0x40101E04 //01CC 0073 40101E04
  .word  0x40101AC4 //01D0 0074 40101AC4
  .word  0x401001A0 //01D4 0075 401001A0
  .word  0x40100C00 //01D8 0076 40100C00
  .word  0x40101DAC //01DC 0077 40101DAC
  .word  0xA0000003 //01E0 0078 A0000003
  .word  0xFFFFFFDC //01E4 0079 -24
  .word  0x00003031 //01E8 007A 3031
  .word  0x00000006 //01EC 007B 0006
  .word  0x401001A4 //01F0 007C 401001A4
  .word  0x00002800 //01F4 007D 2800
  .word  0xFFFFFFEC //01F8 007E -14
  .word  0x00003038 //01FC 007F 3038
  .word  0x00000002 //0200 0080 0002
  .word  0x401001A4 //0204 0081 401001A4
  .word  0x00002801 //0208 0082 2801
  .word  0xFFFFFFEC //020C 0083 -14
  .word  0x0000303B //0210 0084 303B
  .word  0x00000002 //0214 0085 0002
  .word  0x401001A4 //0218 0086 401001A4
  .word  0x00002802 //021C 0087 2802
  .word  0xFFFFFFEC //0220 0088 -14
  .word  0x0000303E //0224 0089 303E
  .word  0x00000002 //0228 008A 0002
  .word  0x401001A4 //022C 008B 401001A4
  .word  0x00002803 //0230 008C 2803
  .word  0xFFFFFFEC //0234 008D -14
  .word  0x00003041 //0238 008E 3041
  .word  0x00000004 //023C 008F 0004
  .word  0x401001A4 //0240 0090 401001A4
  .word  0x00002F00 //0244 0091 2F00
  .word  0xFFFFFFEC //0248 0092 -14
  .word  0x00003046 //024C 0093 3046
  .word  0x00000009 //0250 0094 0009
  .word  0x401001A4 //0254 0095 401001A4
  .word  0x00002F01 //0258 0096 2F01
  .word  0xFFFFFFEC //025C 0097 -14
  .word  0x00003050 //0260 0098 3050
  .word  0x00000003 //0264 0099 0003
  .word  0x401001A4 //0268 009A 401001A4
  .word  0x00002F02 //026C 009B 2F02
  .word  0xFFFFFFEC //0270 009C -14
  .word  0x00003054 //0274 009D 3054
  .word  0x00000007 //0278 009E 0007
  .word  0x401001A4 //027C 009F 401001A4
  .word  0x00002F03 //0280 00A0 2F03
  .word  0xFFFFFFEC //0284 00A1 -14
  .word  0x0000305C //0288 00A2 305C
  .word  0x00000007 //028C 00A3 0007
  .word  0x401001A4 //0290 00A4 401001A4
  .word  0x00002F04 //0294 00A5 2F04
  .word  0xFFFFFFEC //0298 00A6 -14
  .word  0x00003064 //029C 00A7 3064
  .word  0x00000004 //02A0 00A8 0004
  .word  0x401001A4 //02A4 00A9 401001A4
  .word  0x00002F05 //02A8 00AA 2F05
  .word  0xFFFFFFEC //02AC 00AB -14
  .word  0x00003069 //02B0 00AC 3069
  .word  0x00000007 //02B4 00AD 0007
  .word  0x401001A4 //02B8 00AE 401001A4
  .word  0x00002F06 //02BC 00AF 2F06
  .word  0xFFFFFFEC //02C0 00B0 -14
  .word  0x00003071 //02C4 00B1 3071
  .word  0x00000004 //02C8 00B2 0004
  .word  0x401001A4 //02CC 00B3 401001A4
  .word  0x00002F07 //02D0 00B4 2F07
  .word  0xFFFFFFEC //02D4 00B5 -14
  .word  0x00003076 //02D8 00B6 3076
  .word  0x00000004 //02DC 00B7 0004
  .word  0x401001A4 //02E0 00B8 401001A4
  .word  0x00002F08 //02E4 00B9 2F08
  .word  0xFFFFFFEC //02E8 00BA -14
  .word  0x0000307B //02EC 00BB 307B
  .word  0x00000003 //02F0 00BC 0003
  .word  0x401001A4 //02F4 00BD 401001A4
  .word  0x00002F09 //02F8 00BE 2F09
  .word  0xFFFFFFEC //02FC 00BF -14
  .word  0x0000307F //0300 00C0 307F
  .word  0x00000003 //0304 00C1 0003
  .word  0x401001A4 //0308 00C2 401001A4
  .word  0x00002F0A //030C 00C3 2F0A
  .word  0xFFFFFFEC //0310 00C4 -14
  .word  0x00003083 //0314 00C5 3083
  .word  0x00000003 //0318 00C6 0003
  .word  0x401001A4 //031C 00C7 401001A4
  .word  0x00002F0B //0320 00C8 2F0B
  .word  0xFFFFFFEC //0324 00C9 -14
  .word  0x00003087 //0328 00CA 3087
  .word  0x00000003 //032C 00CB 0003
  .word  0x401001A4 //0330 00CC 401001A4
  .word  0x00002F0C //0334 00CD 2F0C
  .word  0xFFFFFFEC //0338 00CE -14
  .word  0x0000308B //033C 00CF 308B
  .word  0x00000003 //0340 00D0 0003
  .word  0x401001A4 //0344 00D1 401001A4
  .word  0x00002F0D //0348 00D2 2F0D
  .word  0xFFFFFFEC //034C 00D3 -14
  .word  0x0000308F //0350 00D4 308F
  .word  0x00000007 //0354 00D5 0007
  .word  0x401001A4 //0358 00D6 401001A4
  .word  0x00002F0E //035C 00D7 2F0E
  .word  0xFFFFFFEC //0360 00D8 -14
  .word  0x00003097 //0364 00D9 3097
  .word  0x00000002 //0368 00DA 0002
  .word  0x401001A4 //036C 00DB 401001A4
  .word  0x00002F0F //0370 00DC 2F0F
  .word  0xFFFFFFEC //0374 00DD -14
  .word  0x0000309A //0378 00DE 309A
  .word  0x00000004 //037C 00DF 0004
  .word  0x401001A4 //0380 00E0 401001A4
  .word  0x00002F10 //0384 00E1 2F10
  .word  0xFFFFFFEC //0388 00E2 -14
  .word  0x0000309F //038C 00E3 309F
  .word  0x00000003 //0390 00E4 0003
  .word  0x401001A4 //0394 00E5 401001A4
  .word  0x00002F11 //0398 00E6 2F11
  .word  0xFFFFFFEC //039C 00E7 -14
  .word  0x000030A3 //03A0 00E8 30A3
  .word  0x00000004 //03A4 00E9 0004
  .word  0x401001A4 //03A8 00EA 401001A4
  .word  0x00002F12 //03AC 00EB 2F12
  .word  0xFFFFFFEC //03B0 00EC -14
  .word  0x000030A8 //03B4 00ED 30A8
  .word  0x00000005 //03B8 00EE 0005
  .word  0x401001A4 //03BC 00EF 401001A4
  .word  0x00002F13 //03C0 00F0 2F13
  .word  0xFFFFFFEC //03C4 00F1 -14
  .word  0x000030AE //03C8 00F2 30AE
  .word  0x00000006 //03CC 00F3 0006
  .word  0x401001A4 //03D0 00F4 401001A4
  .word  0x00002F14 //03D4 00F5 2F14
  .word  0xFFFFFFEC //03D8 00F6 -14
  .word  0x000030B5 //03DC 00F7 30B5
  .word  0x00000003 //03E0 00F8 0003
  .word  0x401001A4 //03E4 00F9 401001A4
  .word  0x00002F15 //03E8 00FA 2F15
  .word  0xFFFFFFEC //03EC 00FB -14
  .word  0x000030B9 //03F0 00FC 30B9
  .word  0x00000005 //03F4 00FD 0005
  .word  0x401001A4 //03F8 00FE 401001A4
  .word  0x00002F16 //03FC 00FF 2F16
  .word  0xFFFFFFEC //0400 0100 -14
  .word  0x000030BF //0404 0101 30BF
  .word  0x0000000C //0408 0102 000C
  .word  0x401001A4 //040C 0103 401001A4
  .word  0x00002F17 //0410 0104 2F17
  .word  0xFFFFFFEC //0414 0105 -14
  .word  0x000030CC //0418 0106 30CC
  .word  0x00000007 //041C 0107 0007
  .word  0x401001A4 //0420 0108 401001A4
  .word  0x000001CB //0424 0109 01CB
  .word  0xFFFFFFEC //0428 010A -14
  .word  0x000030D4 //042C 010B 30D4
  .word  0x00000006 //0430 010C 0006
  .word  0x40101E04 //0434 010D 40101E04
  .word  0x0000000A //0438 010E 000A
  .word  0x00000003 //043C 010F 0003
  .word  0x4010011C //0440 0110 4010011C
  .word  0xA0000003 //0444 0111 A0000003
  .word  0xFFFFFFE0 //0448 0112 -20
  .word  0x000030DB //044C 0113 30DB
  .word  0x00000008 //0450 0114 0008
  .word  0x40101DE8 //0454 0115 40101DE8
  .word  0x40100B3C //0458 0116 40100B3C
  .word  0x00002F10 //045C 0117 2F10
  .word  0xA000000A //0460 0118 A000000A
  .word  0x9000000C //0464 0119 9000000C
  .word  0xA000000A //0468 011A A000000A
  .word  0x40100C00 //046C 011B 40100C00
  .word  0x80000004 //0470 011C 80000004
  .word  0x40100C2C //0474 011D 40100C2C
  .word  0xA0000003 //0478 011E A0000003
  .word  0xFFFFFFCC //047C 011F -34
  .word  0x000030E4 //0480 0120 30E4
  .word  0x00000005 //0484 0121 0005
  .word  0x40101E04 //0488 0122 40101E04
  .word  0x40101AC4 //048C 0123 40101AC4
  .word  0x40100454 //0490 0124 40100454
  .word  0x40100C00 //0494 0125 40100C00
  .word  0x40100438 //0498 0126 40100438
  .word  0x40100C00 //049C 0127 40100C00
  .word  0x40101DAC //04A0 0128 40101DAC
  .word  0xA0000003 //04A4 0129 A0000003
  .word  0xFFFFFFD4 //04A8 012A -2C
  .word  0x000030EA //04AC 012B 30EA
  .word  0x00000005 //04B0 012C 0005
  .word  0x40100458 //04B4 012D 40100458
  .word  0xA0000000 //04B8 012E A0000000
  .word  0xA0000003 //04BC 012F A0000003
  .word  0xFFFFFFE8 //04C0 0130 -18
  .word  0x000030F0 //04C4 0131 30F0
  .word  0x00000002 //04C8 0132 0002
  .word  0x40100458 //04CC 0133 40100458
  .word  0xA0000001 //04D0 0134 A0000001
  .word  0xA0000003 //04D4 0135 A0000003
  .word  0xFFFFFFE8 //04D8 0136 -18
  .word  0x000030F3 //04DC 0137 30F3
  .word  0x00000002 //04E0 0138 0002
  .word  0x40100458 //04E4 0139 40100458
  .word  0xA0000002 //04E8 013A A0000002
  .word  0xA0000003 //04EC 013B A0000003
  .word  0xFFFFFFE8 //04F0 013C -18
  .word  0x000030F6 //04F4 013D 30F6
  .word  0x00000002 //04F8 013E 0002
  .word  0x40100458 //04FC 013F 40100458
  .word  0xA000000D //0500 0140 A000000D
  .word  0xA0000003 //0504 0141 A0000003
  .word  0xFFFFFFE8 //0508 0142 -18
  .word  0x000030F9 //050C 0143 30F9
  .word  0x00000003 //0510 0144 0003
  .word  0x40100458 //0514 0145 40100458
  .word  0xA000000F //0518 0146 A000000F
  .word  0xA0000003 //051C 0147 A0000003
  .word  0xFFFFFFE8 //0520 0148 -18
  .word  0x000030FD //0524 0149 30FD
  .word  0x00000008 //0528 014A 0008
  .word  0x40100458 //052C 014B 40100458
  .word  0xA0000005 //0530 014C A0000005
  .word  0xA0000003 //0534 014D A0000003
  .word  0xFFFFFFE8 //0538 014E -18
  .word  0x00003106 //053C 014F 3106
  .word  0x00000003 //0540 0150 0003
  .word  0x40100458 //0544 0151 40100458
  .word  0xA000000B //0548 0152 A000000B
  .word  0xA0000003 //054C 0153 A0000003
  .word  0xFFFFFFE8 //0550 0154 -18
  .word  0x0000310A //0554 0155 310A
  .word  0x00000003 //0558 0156 0003
  .word  0x40100458 //055C 0157 40100458
  .word  0xA0000008 //0560 0158 A0000008
  .word  0xA0000003 //0564 0159 A0000003
  .word  0xFFFFFFE8 //0568 015A -18
  .word  0x0000310E //056C 015B 310E
  .word  0x00000002 //0570 015C 0002
  .word  0x40100458 //0574 015D 40100458
  .word  0xA000000E //0578 015E A000000E
  .word  0xA0000003 //057C 015F A0000003
  .word  0xFFFFFFE8 //0580 0160 -18
  .word  0x00003111 //0584 0161 3111
  .word  0x00000002 //0588 0162 0002
  .word  0x40100458 //058C 0163 40100458
  .word  0xA0000007 //0590 0164 A0000007
  .word  0xA0000003 //0594 0165 A0000003
  .word  0xFFFFFFE8 //0598 0166 -18
  .word  0x00003114 //059C 0167 3114
  .word  0x00000001 //05A0 0168 0001
  .word  0x40100458 //05A4 0169 40100458
  .word  0xA0000009 //05A8 016A A0000009
  .word  0xA0000003 //05AC 016B A0000003
  .word  0xFFFFFFE8 //05B0 016C -18
  .word  0x00003116 //05B4 016D 3116
  .word  0x00000001 //05B8 016E 0001
  .word  0x40100458 //05BC 016F 40100458
  .word  0xA000000A //05C0 0170 A000000A
  .word  0xA0000003 //05C4 0171 A0000003
  .word  0xFFFFFFE8 //05C8 0172 -18
  .word  0x00003118 //05CC 0173 3118
  .word  0x00000004 //05D0 0174 0004
  .word  0x40100458 //05D4 0175 40100458
  .word  0xB0000412 //05D8 0176 B0000412
  .word  0xA0000003 //05DC 0177 A0000003
  .word  0xFFFFFFE8 //05E0 0178 -18
  .word  0x0000311D //05E4 0179 311D
  .word  0x00000004 //05E8 017A 0004
  .word  0x40100458 //05EC 017B 40100458
  .word  0xB0000502 //05F0 017C B0000502
  .word  0xA0000003 //05F4 017D A0000003
  .word  0xFFFFFFE8 //05F8 017E -18
  .word  0x00003122 //05FC 017F 3122
  .word  0x00000003 //0600 0180 0003
  .word  0x40100458 //0604 0181 40100458
  .word  0xB0000501 //0608 0182 B0000501
  .word  0xA0000003 //060C 0183 A0000003
  .word  0xFFFFFFE8 //0610 0184 -18
  .word  0x00003126 //0614 0185 3126
  .word  0x00000003 //0618 0186 0003
  .word  0x40100458 //061C 0187 40100458
  .word  0xB0000434 //0620 0188 B0000434
  .word  0xA0000003 //0624 0189 A0000003
  .word  0xFFFFFFE8 //0628 018A -18
  .word  0x0000312A //062C 018B 312A
  .word  0x00000004 //0630 018C 0004
  .word  0x40100458 //0634 018D 40100458
  .word  0xB0000300 //0638 018E B0000300
  .word  0xA0000003 //063C 018F A0000003
  .word  0xFFFFFFE8 //0640 0190 -18
  .word  0x0000312F //0644 0191 312F
  .word  0x00000005 //0648 0192 0005
  .word  0x40100458 //064C 0193 40100458
  .word  0xB000043C //0650 0194 B000043C
  .word  0xA0000003 //0654 0195 A0000003
  .word  0xFFFFFFE8 //0658 0196 -18
  .word  0x00003135 //065C 0197 3135
  .word  0x00000005 //0660 0198 0005
  .word  0x40100458 //0664 0199 40100458
  .word  0xB000060C //0668 019A B000060C
  .word  0xA0000003 //066C 019B A0000003
  .word  0xFFFFFFE8 //0670 019C -18
  .word  0x0000313B //0674 019D 313B
  .word  0x00000004 //0678 019E 0004
  .word  0x40100458 //067C 019F 40100458
  .word  0xB0000603 //0680 01A0 B0000603
  .word  0xA0000003 //0684 01A1 A0000003
  .word  0xFFFFFFE8 //0688 01A2 -18
  .word  0x00003140 //068C 01A3 3140
  .word  0x00000005 //0690 01A4 0005
  .word  0x40100458 //0694 01A5 40100458
  .word  0xB0000200 //0698 01A6 B0000200
  .word  0xA0000003 //069C 01A7 A0000003
  .word  0xFFFFFFE8 //06A0 01A8 -18
  .word  0x00003146 //06A4 01A9 3146
  .word  0x00000004 //06A8 01AA 0004
  .word  0x40100458 //06AC 01AB 40100458
  .word  0x80000000 //06B0 01AC 80000000
  .word  0xA0000003 //06B4 01AD A0000003
  .word  0xFFFFFFE8 //06B8 01AE -18
  .word  0x0000314B //06BC 01AF 314B
  .word  0x00000002 //06C0 01B0 0002
  .word  0x40101E04 //06C4 01B1 40101E04
  .word  0x00002F13 //06C8 01B2 2F13
  .word  0xA000000A //06CC 01B3 A000000A
  .word  0xA0000009 //06D0 01B4 A0000009
  .word  0x00000001 //06D4 01B5 0001
  .word  0x00002F13 //06D8 01B6 2F13
  .word  0x40100B10 //06DC 01B7 40100B10
  .word  0xA0000003 //06E0 01B8 A0000003
  .word  0xFFFFFFD4 //06E4 01B9 -2C
  .word  0x0000314E //06E8 01BA 314E
  .word  0x00000002 //06EC 01BB 0002
  .word  0x40101E04 //06F0 01BC 40101E04
  .word  0x00002F13 //06F4 01BD 2F13
  .word  0xA000000A //06F8 01BE A000000A
  .word  0x40100168 //06FC 01BF 40100168
  .word  0xB0000501 //0700 01C0 B0000501
  .word  0x40100C00 //0704 01C1 40100C00
  .word  0xB0000412 //0708 01C2 B0000412
  .word  0xB0000501 //070C 01C3 B0000501
  .word  0xA000000A //0710 01C4 A000000A
  .word  0x401006C8 //0714 01C5 401006C8
  .word  0x40100A18 //0718 01C6 40100A18
  .word  0xB0000412 //071C 01C7 B0000412
  .word  0x40100A34 //0720 01C8 40100A34
  .word  0xB0000501 //0724 01C9 B0000501
  .word  0xA000000D //0728 01CA A000000D
  .word  0x9FFFFFD8 //072C 01CB 9FFFFFD8
  .word  0xB0000200 //0730 01CC B0000200
  .word  0x00000020 //0734 01CD 0020
  .word  0x401006C8 //0738 01CE 401006C8
  .word  0xA0000003 //073C 01CF A0000003
  .word  0xFFFFFFA4 //0740 01D0 -5C
  .word  0x00003151 //0744 01D1 3151
  .word  0x00000007 //0748 01D2 0007
  .word  0x40101DE8 //074C 01D3 40101DE8
  .word  0x40101790 //0750 01D4 40101790
  .word  0x00002F10 //0754 01D5 2F10
  .word  0xA000000A //0758 01D6 A000000A
  .word  0x9000000C //075C 01D7 9000000C
  .word  0x401006F4 //0760 01D8 401006F4
  .word  0x40100B3C //0764 01D9 40100B3C
  .word  0x40101A9C //0768 01DA 40101A9C
  .word  0xA0000003 //076C 01DB A0000003
  .word  0xFFFFFFD0 //0770 01DC -30
  .word  0x00003159 //0774 01DD 3159
  .word  0x00000005 //0778 01DE 0005
  .word  0x40101E04 //077C 01DF 40101E04
  .word  0x40101AC4 //0780 01E0 40101AC4
  .word  0x00000001 //0784 01E1 0001
  .word  0x00002F10 //0788 01E2 2F10
  .word  0xA0000009 //078C 01E3 A0000009
  .word  0x40100C00 //0790 01E4 40100C00
  .word  0x4010074C //0794 01E5 4010074C
  .word  0xFFFFFFFF //0798 01E6 -1
  .word  0x00002F15 //079C 01E7 2F15
  .word  0x40100B10 //07A0 01E8 40100B10
  .word  0xA0000003 //07A4 01E9 A0000003
  .word  0xFFFFFFC8 //07A8 01EA -38
  .word  0x0000315F //07AC 01EB 315F
  .word  0x00000001 //07B0 01EC 0001
  .word  0x00000022 //07B4 01ED 0022
  .word  0x40100750 //07B8 01EE 40100750
  .word  0xA0000003 //07BC 01EF A0000003
  .word  0xFFFFFFE8 //07C0 01F0 -18
  .word  0x00003161 //07C4 01F1 3161
  .word  0x00000002 //07C8 01F2 0002
  .word  0x00000022 //07CC 01F3 0022
  .word  0x40100750 //07D0 01F4 40100750
  .word  0x40100CF0 //07D4 01F5 40100CF0
  .word  0xA0000003 //07D8 01F6 A0000003
  .word  0xFFFFFFE4 //07DC 01F7 -1C
  .word  0x00003164 //07E0 01F8 3164
  .word  0x00000004 //07E4 01F9 0004
  .word  0x40101E04 //07E8 01FA 40101E04
  .word  0x00002F0F //07EC 01FB 2F0F
  .word  0xA000000A //07F0 01FC A000000A
  .word  0xA0000003 //07F4 01FD A0000003
  .word  0xFFFFFFE4 //07F8 01FE -1C
  .word  0x00003169 //07FC 01FF 3169
  .word  0x00000005 //0800 0200 0005
  .word  0x40101E04 //0804 0201 40101E04
  .word  0x00000008 //0808 0202 0008
  .word  0xA0000003 //080C 0203 A0000003
  .word  0xFFFFFFE8 //0810 0204 -18
  .word  0x0000316F //0814 0205 316F
  .word  0x00000006 //0818 0206 0006
  .word  0x40101E04 //081C 0207 40101E04
  .word  0x00000009 //0820 0208 0009
  .word  0xA0000003 //0824 0209 A0000003
  .word  0xFFFFFFE8 //0828 020A -18
  .word  0x00003176 //082C 020B 3176
  .word  0x00000006 //0830 020C 0006
  .word  0x40101E04 //0834 020D 40101E04
  .word  0x00000000 //0838 020E 0000
  .word  0x10000000 //083C 020F 10000000
  .word  0xB0000434 //0840 0210 B0000434
  .word  0xA0000002 //0844 0211 A0000002
  .word  0xB0000412 //0848 0212 B0000412
  .word  0xB0000300 //084C 0213 B0000300
  .word  0xB0000412 //0850 0214 B0000412
  .word  0x0FFFFFFF //0854 0215 FFFFFFF
  .word  0xA0000008 //0858 0216 A0000008
  .word  0xA000000E //085C 0217 A000000E
  .word  0xA0000003 //0860 0218 A0000003
  .word  0xFFFFFFC4 //0864 0219 -3C
  .word  0x0000317D //0868 021A 317D
  .word  0x00000005 //086C 021B 0005
  .word  0x40101E04 //0870 021C 40101E04
  .word  0x00002F0F //0874 021D 2F0F
  .word  0x40100B10 //0878 021E 40100B10
  .word  0xA0000003 //087C 021F A0000003
  .word  0xFFFFFFE4 //0880 0220 -1C
  .word  0x00003183 //0884 0221 3183
  .word  0x00000007 //0888 0222 0007
  .word  0x40101E04 //088C 0223 40101E04
  .word  0x401007EC //0890 0224 401007EC
  .word  0xA000002E //0894 0225 A000002E
  .word  0x40100A50 //0898 0226 40100A50
  .word  0x40100808 //089C 0227 40100808
  .word  0x40100838 //08A0 0228 40100838
  .word  0x40100C00 //08A4 0229 40100C00
  .word  0xA0000003 //08A8 022A A0000003
  .word  0xFFFFFFD4 //08AC 022B -2C
  .word  0x0000318B //08B0 022C 318B
  .word  0x00000008 //08B4 022D 0008
  .word  0x40101E04 //08B8 022E 40101E04
  .word  0x401007EC //08BC 022F 401007EC
  .word  0xA000002E //08C0 0230 A000002E
  .word  0x40100A50 //08C4 0231 40100A50
  .word  0x40100820 //08C8 0232 40100820
  .word  0x40100838 //08CC 0233 40100838
  .word  0x40100C00 //08D0 0234 40100C00
  .word  0xA0000003 //08D4 0235 A0000003
  .word  0xFFFFFFD4 //08D8 0236 -2C
  .word  0x00003194 //08DC 0237 3194
  .word  0x00000005 //08E0 0238 0005
  .word  0x40101DCC //08E4 0239 40101DCC
  .word  0x401007EC //08E8 023A 401007EC
  .word  0xA0000003 //08EC 023B A0000003
  .word  0xFFFFFFE8 //08F0 023C -18
  .word  0x0000319A //08F4 023D 319A
  .word  0x00000005 //08F8 023E 0005
  .word  0x40101DCC //08FC 023F 40101DCC
  .word  0x40100890 //0900 0240 40100890
  .word  0xA0000003 //0904 0241 A0000003
  .word  0xFFFFFFE8 //0908 0242 -18
  .word  0x000031A0 //090C 0243 31A0
  .word  0x00000005 //0910 0244 0005
  .word  0x40101DCC //0914 0245 40101DCC
  .word  0x401008BC //0918 0246 401008BC
  .word  0xA0000003 //091C 0247 A0000003
  .word  0xFFFFFFE8 //0920 0248 -18
  .word  0x000031A6 //0924 0249 31A6
  .word  0x00000002 //0928 024A 0002
  .word  0x40101DCC //092C 024B 40101DCC
  .word  0x40100820 //0930 024C 40100820
  .word  0x00000004 //0934 024D 0004
  .word  0x40100874 //0938 024E 40100874
  .word  0x401007EC //093C 024F 401007EC
  .word  0xA0000003 //0940 0250 A0000003
  .word  0xFFFFFFDC //0944 0251 -24
  .word  0x000031A9 //0948 0252 31A9
  .word  0x00000006 //094C 0253 0006
  .word  0x40101DCC //0950 0254 40101DCC
  .word  0x401007EC //0954 0255 401007EC
  .word  0xB0000502 //0958 0256 B0000502
  .word  0x40100A50 //095C 0257 40100A50
  .word  0xB0000434 //0960 0258 B0000434
  .word  0x40100838 //0964 0259 40100838
  .word  0xB0000412 //0968 025A B0000412
  .word  0xA000002F //096C 025B A000002F
  .word  0xA0000009 //0970 025C A0000009
  .word  0xA0000003 //0974 025D A0000003
  .word  0xFFFFFFCC //0978 025E -34
  .word  0x000031B0 //097C 025F 31B0
  .word  0x00000004 //0980 0260 0004
  .word  0x40101DCC //0984 0261 40101DCC
  .word  0x00000004 //0988 0262 0004
  .word  0x40100874 //098C 0263 40100874
  .word  0x40100950 //0990 0264 40100950
  .word  0x40100808 //0994 0265 40100808
  .word  0x401007EC //0998 0266 401007EC
  .word  0xA0000003 //099C 0267 A0000003
  .word  0xFFFFFFD8 //09A0 0268 -28
  .word  0x000031B5 //09A4 0269 31B5
  .word  0x00000005 //09A8 026A 0005
  .word  0x40101DCC //09AC 026B 40101DCC
  .word  0x4010092C //09B0 026C 4010092C
  .word  0xA0000003 //09B4 026D A0000003
  .word  0xFFFFFFE8 //09B8 026E -18
  .word  0x000031BB //09BC 026F 31BB
  .word  0x00000006 //09C0 0270 0006
  .word  0x40101DCC //09C4 0271 40101DCC
  .word  0xB0000434 //09C8 0272 B0000434
  .word  0x401008FC //09CC 0273 401008FC
  .word  0x40100950 //09D0 0274 40100950
  .word  0xA0000003 //09D4 0275 A0000003
  .word  0xFFFFFFE0 //09D8 0276 -20
  .word  0x000031C2 //09DC 0277 31C2
  .word  0x00000002 //09E0 0278 0002
  .word  0x40101E04 //09E4 0279 40101E04
  .word  0xA000000A //09E8 027A A000000A
  .word  0xA0000003 //09EC 027B A0000003
  .word  0xFFFFFFE8 //09F0 027C -18
  .word  0x000031C5 //09F4 027D 31C5
  .word  0x00000002 //09F8 027E 0002
  .word  0x40101E04 //09FC 027F 40101E04
  .word  0xA0000009 //0A00 0280 A0000009
  .word  0xA0000003 //0A04 0281 A0000003
  .word  0xFFFFFFE8 //0A08 0282 -18
  .word  0x000031C8 //0A0C 0283 31C8
  .word  0x00000002 //0A10 0284 0002
  .word  0x40101E04 //0A14 0285 40101E04
  .word  0x00000001 //0A18 0286 0001
  .word  0xA0000007 //0A1C 0287 A0000007
  .word  0xA0000003 //0A20 0288 A0000003
  .word  0xFFFFFFE4 //0A24 0289 -1C
  .word  0x000031CB //0A28 028A 31CB
  .word  0x00000002 //0A2C 028B 0002
  .word  0x40101E04 //0A30 028C 40101E04
  .word  0xFFFFFFFF //0A34 028D -1
  .word  0xA0000007 //0A38 028E A0000007
  .word  0xA0000003 //0A3C 028F A0000003
  .word  0xFFFFFFE4 //0A40 0290 -1C
  .word  0x000031CE //0A44 0291 31CE
  .word  0x00000002 //0A48 0292 0002
  .word  0x40101E04 //0A4C 0293 40101E04
  .word  0xA0000000 //0A50 0294 A0000000
  .word  0xA0000007 //0A54 0295 A0000007
  .word  0xA0000003 //0A58 0296 A0000003
  .word  0xFFFFFFE4 //0A5C 0297 -1C
  .word  0x000031D1 //0A60 0298 31D1
  .word  0x00000001 //0A64 0299 0001
  .word  0x40101E04 //0A68 029A 40101E04
  .word  0x40100A50 //0A6C 029B 40100A50
  .word  0xA000000D //0A70 029C A000000D
  .word  0xA0000003 //0A74 029D A0000003
  .word  0xFFFFFFE4 //0A78 029E -1C
  .word  0x000031D3 //0A7C 029F 31D3
  .word  0x00000002 //0A80 02A0 0002
  .word  0x40101E04 //0A84 02A1 40101E04
  .word  0x40100A50 //0A88 02A2 40100A50
  .word  0xA000000F //0A8C 02A3 A000000F
  .word  0xA0000003 //0A90 02A4 A0000003
  .word  0xFFFFFFE4 //0A94 02A5 -1C
  .word  0x000031D6 //0A98 02A6 31D6
  .word  0x00000001 //0A9C 02A7 0001
  .word  0x40101E04 //0AA0 02A8 40101E04
  .word  0xB0000412 //0AA4 02A9 B0000412
  .word  0x40100A88 //0AA8 02AA 40100A88
  .word  0xA0000003 //0AAC 02AB A0000003
  .word  0xFFFFFFE4 //0AB0 02AC -1C
  .word  0x000031D8 //0AB4 02AD 31D8
  .word  0x00000002 //0AB8 02AE 0002
  .word  0x40101E04 //0ABC 02AF 40101E04
  .word  0x00000000 //0AC0 02B0 0000
  .word  0xB0000434 //0AC4 02B1 B0000434
  .word  0xB0000434 //0AC8 02B2 B0000434
  .word  0xA0000002 //0ACC 02B3 A0000002
  .word  0xB0000412 //0AD0 02B4 B0000412
  .word  0xB0000300 //0AD4 02B5 B0000300
  .word  0xA0000003 //0AD8 02B6 A0000003
  .word  0xFFFFFFD4 //0ADC 02B7 -2C
  .word  0x000031DB //0AE0 02B8 31DB
  .word  0x00000003 //0AE4 02B9 0003
  .word  0x40101E04 //0AE8 02BA 40101E04
  .word  0x000031DF //0AEC 02BB 31DF
  .word  0x00000004 //0AF0 02BC 0004
  .word  0x401007D4 //0AF4 02BD 401007D4
  .word  0x8FFFFFF0 //0AF8 02BE 8FFFFFF0
  .word  0xA0000003 //0AFC 02BF A0000003
  .word  0xFFFFFFDC //0B00 02C0 -24
  .word  0x000031E4 //0B04 02C1 31E4
  .word  0x00000002 //0B08 02C2 0002
  .word  0x40101E04 //0B0C 02C3 40101E04
  .word  0xB0000412 //0B10 02C4 B0000412
  .word  0xB0000502 //0B14 02C5 B0000502
  .word  0xA000000A //0B18 02C6 A000000A
  .word  0xA0000007 //0B1C 02C7 A0000007
  .word  0xB0000412 //0B20 02C8 B0000412
  .word  0xA0000009 //0B24 02C9 A0000009
  .word  0xA0000003 //0B28 02CA A0000003
  .word  0xFFFFFFD4 //0B2C 02CB -2C
  .word  0x000031E7 //0B30 02CC 31E7
  .word  0x00000002 //0B34 02CD 0002
  .word  0x40101E04 //0B38 02CE 40101E04
  .word  0x00002802 //0B3C 02CF 2802
  .word  0xA000000A //0B40 02D0 A000000A
  .word  0x40100A18 //0B44 02D1 40100A18
  .word  0xA000000A //0B48 02D2 A000000A
  .word  0x00002802 //0B4C 02D3 2802
  .word  0xA000000A //0B50 02D4 A000000A
  .word  0x40100A18 //0B54 02D5 40100A18
  .word  0x00002802 //0B58 02D6 2802
  .word  0xB0000603 //0B5C 02D7 B0000603
  .word  0xA000000A //0B60 02D8 A000000A
  .word  0xA000000A //0B64 02D9 A000000A
  .word  0xB0000412 //0B68 02DA B0000412
  .word  0xA0000009 //0B6C 02DB A0000009
  .word  0xA0000009 //0B70 02DC A0000009
  .word  0xA0000003 //0B74 02DD A0000003
  .word  0xFFFFFFB4 //0B78 02DE -4C
  .word  0x000031EA //0B7C 02DF 31EA
  .word  0x00000002 //0B80 02E0 0002
  .word  0x40101E04 //0B84 02E1 40101E04
  .word  0x00002802 //0B88 02E2 2802
  .word  0xA000000A //0B8C 02E3 A000000A
  .word  0xB0000501 //0B90 02E4 B0000501
  .word  0x40100A34 //0B94 02E5 40100A34
  .word  0x00002802 //0B98 02E6 2802
  .word  0xB0000603 //0B9C 02E7 B0000603
  .word  0xA000000A //0BA0 02E8 A000000A
  .word  0xA000000A //0BA4 02E9 A000000A
  .word  0xB0000412 //0BA8 02EA B0000412
  .word  0xB0000501 //0BAC 02EB B0000501
  .word  0x40100A34 //0BB0 02EC 40100A34
  .word  0x00002802 //0BB4 02ED 2802
  .word  0xA0000009 //0BB8 02EE A0000009
  .word  0xA0000009 //0BBC 02EF A0000009
  .word  0xA0000009 //0BC0 02F0 A0000009
  .word  0xA0000009 //0BC4 02F1 A0000009
  .word  0xA0000003 //0BC8 02F2 A0000003
  .word  0xFFFFFFAC //0BCC 02F3 -54
  .word  0x000031ED //0BD0 02F4 31ED
  .word  0x00000001 //0BD4 02F5 0001
  .word  0x40101E04 //0BD8 02F6 40101E04
  .word  0x00002802 //0BDC 02F7 2802
  .word  0xA000000A //0BE0 02F8 A000000A
  .word  0x40100A18 //0BE4 02F9 40100A18
  .word  0xA000000A //0BE8 02FA A000000A
  .word  0xA0000003 //0BEC 02FB A0000003
  .word  0xFFFFFFDC //0BF0 02FC -24
  .word  0x000031EF //0BF4 02FD 31EF
  .word  0x00000001 //0BF8 02FE 0001
  .word  0x40101E04 //0BFC 02FF 40101E04
  .word  0x00002F0F //0C00 0300 2F0F
  .word  0xA000000A //0C04 0301 A000000A
  .word  0xA0000009 //0C08 0302 A0000009
  .word  0x00000004 //0C0C 0303 0004
  .word  0x00002F0F //0C10 0304 2F0F
  .word  0x40100B10 //0C14 0305 40100B10
  .word  0xA0000003 //0C18 0306 A0000003
  .word  0xFFFFFFD4 //0C1C 0307 -2C
  .word  0x000031F1 //0C20 0308 31F1
  .word  0x00000007 //0C24 0309 0007
  .word  0x40101E04 //0C28 030A 40101E04
  .word  0x00002803 //0C2C 030B 2803
  .word  0xA0000009 //0C30 030C A0000009
  .word  0xA0000003 //0C34 030D A0000003
  .word  0xFFFFFFE4 //0C38 030E -1C
  .word  0x000031F9 //0C3C 030F 31F9
  .word  0x00000003 //0C40 0310 0003
  .word  0x40101E04 //0C44 0311 40101E04
  .word  0xA0000004 //0C48 0312 A0000004
  .word  0x401012B0 //0C4C 0313 401012B0
  .word  0xA000000B //0C50 0314 A000000B
  .word  0x90000008 //0C54 0315 90000008
  .word  0xB0000300 //0C58 0316 B0000300
  .word  0x8FFFFFE8 //0C5C 0317 8FFFFFE8
  .word  0xA0000003 //0C60 0318 A0000003
  .word  0xFFFFFFD4 //0C64 0319 -2C
  .word  0x000031FD //0C68 031A 31FD
  .word  0x00000004 //0C6C 031B 0004
  .word  0x40101E04 //0C70 031C 40101E04
  .word  0x00100530 //0C74 031D 100530
  .word  0x40100C2C //0C78 031E 40100C2C
  .word  0xA0000003 //0C7C 031F A0000003
  .word  0xFFFFFFE4 //0C80 0320 -1C
  .word  0x00003202 //0C84 0321 3202
  .word  0x00000005 //0C88 0322 0005
  .word  0x40101E04 //0C8C 0323 40101E04
  .word  0x00000000 //0C90 0324 0000
  .word  0xB0000412 //0C94 0325 B0000412
  .word  0x00000010 //0C98 0326 0010
  .word  0xA0000002 //0C9C 0327 A0000002
  .word  0xB0000412 //0CA0 0328 B0000412
  .word  0xA0000003 //0CA4 0329 A0000003
  .word  0xFFFFFFD8 //0CA8 032A -28
  .word  0x00003208 //0CAC 032B 3208
  .word  0x00000003 //0CB0 032C 0003
  .word  0x40101E04 //0CB4 032D 40101E04
  .word  0xB0000501 //0CB8 032E B0000501
  .word  0x0000000A //0CBC 032F 000A
  .word  0x40100A88 //0CC0 0330 40100A88
  .word  0x90000004 //0CC4 0331 90000004
  .word  0x80000008 //0CC8 0332 80000008
  .word  0x00000007 //0CCC 0333 0007
  .word  0xA0000007 //0CD0 0334 A0000007
  .word  0x00000030 //0CD4 0335 0030
  .word  0xA0000007 //0CD8 0336 A0000007
  .word  0xA0000003 //0CDC 0337 A0000003
  .word  0xFFFFFFC8 //0CE0 0338 -38
  .word  0x0000320C //0CE4 0339 320C
  .word  0x00000004 //0CE8 033A 0004
  .word  0x40101E04 //0CEC 033B 40101E04
  .word  0xB0000501 //0CF0 033C B0000501
  .word  0x90000020 //0CF4 033D 90000020
  .word  0xB0000412 //0CF8 033E B0000412
  .word  0xB0000501 //0CFC 033F B0000501
  .word  0x401009E8 //0D00 0340 401009E8
  .word  0x40100C74 //0D04 0341 40100C74
  .word  0x40100A18 //0D08 0342 40100A18
  .word  0xB0000412 //0D0C 0343 B0000412
  .word  0x40100A34 //0D10 0344 40100A34
  .word  0x8FFFFFD8 //0D14 0345 8FFFFFD8
  .word  0xB0000200 //0D18 0346 B0000200
  .word  0xA0000003 //0D1C 0347 A0000003
  .word  0xFFFFFFC0 //0D20 0348 -40
  .word  0x00003211 //0D24 0349 3211
  .word  0x00000003 //0D28 034A 0003
  .word  0x40101E04 //0D2C 034B 40101E04
  .word  0x00000008 //0D30 034C 0008
  .word  0xB0000412 //0D34 034D B0000412
  .word  0x40100C90 //0D38 034E 40100C90
  .word  0x40100CB8 //0D3C 034F 40100CB8
  .word  0x40100C74 //0D40 0350 40100C74
  .word  0xB0000412 //0D44 0351 B0000412
  .word  0x00000001 //0D48 0352 0001
  .word  0x40100A50 //0D4C 0353 40100A50
  .word  0xB0000501 //0D50 0354 B0000501
  .word  0xA000000D //0D54 0355 A000000D
  .word  0x9FFFFFD8 //0D58 0356 9FFFFFD8
  .word  0xB0000200 //0D5C 0357 B0000200
  .word  0xA0000003 //0D60 0358 A0000003
  .word  0xA0000003 //0D64 0359 A0000003
  .word  0xFFFFFFB8 //0D68 035A -48
  .word  0x00003215 //0D6C 035B 3215
  .word  0x00000002 //0D70 035C 0002
  .word  0x40101E04 //0D74 035D 40101E04
  .word  0x40100D30 //0D78 035E 40100D30
  .word  0x00000020 //0D7C 035F 0020
  .word  0x40100C74 //0D80 0360 40100C74
  .word  0xA0000003 //0D84 0361 A0000003
  .word  0xFFFFFFE0 //0D88 0362 -20
  .word  0x00003218 //0D8C 0363 3218
  .word  0x00000002 //0D90 0364 0002
  .word  0x40101E04 //0D94 0365 40101E04
  .word  0xA000000A //0D98 0366 A000000A
  .word  0x40100D78 //0D9C 0367 40100D78
  .word  0xA0000003 //0DA0 0368 A0000003
  .word  0xFFFFFFE4 //0DA4 0369 -1C
  .word  0x0000321B //0DA8 036A 321B
  .word  0x00000002 //0DAC 036B 0002
  .word  0x40101E04 //0DB0 036C 40101E04
  .word  0x00002F07 //0DB4 036D 2F07
  .word  0xA000000A //0DB8 036E A000000A
  .word  0x00002F0F //0DBC 036F 2F0F
  .word  0xA000000A //0DC0 0370 A000000A
  .word  0x40100A50 //0DC4 0371 40100A50
  .word  0x00002F10 //0DC8 0372 2F10
  .word  0xA000000A //0DCC 0373 A000000A
  .word  0xA000000D //0DD0 0374 A000000D
  .word  0xA000000B //0DD4 0375 A000000B
  .word  0xA000000E //0DD8 0376 A000000E
  .word  0x00002F00 //0DDC 0377 2F00
  .word  0xA000000A //0DE0 0378 A000000A
  .word  0xA000000D //0DE4 0379 A000000D
  .word  0xA000000B //0DE8 037A A000000B
  .word  0xA0000008 //0DEC 037B A0000008
  .word  0x900000A0 //0DF0 037C 900000A0
  .word  0x0000003C //0DF4 037D 003C
  .word  0x40100C74 //0DF8 037E 40100C74
  .word  0x0000321E //0DFC 037F 321E
  .word  0x00000003 //0E00 0380 0003
  .word  0x401007D4 //0E04 0381 401007D4
  .word  0x00002F07 //0E08 0382 2F07
  .word  0xA000000A //0E0C 0383 A000000A
  .word  0x40100D78 //0E10 0384 40100D78
  .word  0x00002F06 //0E14 0385 2F06
  .word  0xA000000A //0E18 0386 A000000A
  .word  0x40100D78 //0E1C 0387 40100D78
  .word  0x0000003C //0E20 0388 003C
  .word  0x40100C74 //0E24 0389 40100C74
  .word  0x00003222 //0E28 038A 3222
  .word  0x00000004 //0E2C 038B 0004
  .word  0x401007D4 //0E30 038C 401007D4
  .word  0x0000003C //0E34 038D 003C
  .word  0x40100C74 //0E38 038E 40100C74
  .word  0x00003227 //0E3C 038F 3227
  .word  0x00000003 //0E40 0390 0003
  .word  0x401007D4 //0E44 0391 401007D4
  .word  0x00002F0F //0E48 0392 2F0F
  .word  0xA000000A //0E4C 0393 A000000A
  .word  0x40100D78 //0E50 0394 40100D78
  .word  0x00002F13 //0E54 0395 2F13
  .word  0xA000000A //0E58 0396 A000000A
  .word  0x40100D78 //0E5C 0397 40100D78
  .word  0x0000003C //0E60 0398 003C
  .word  0x40100C74 //0E64 0399 40100C74
  .word  0x0000322B //0E68 039A 322B
  .word  0x00000004 //0E6C 039B 0004
  .word  0x401007D4 //0E70 039C 401007D4
  .word  0x00002F0F //0E74 039D 2F0F
  .word  0xA000000A //0E78 039E A000000A
  .word  0x00002F07 //0E7C 039F 2F07
  .word  0xA0000009 //0E80 03A0 A0000009
  .word  0x00002F13 //0E84 03A1 2F13
  .word  0xA000000A //0E88 03A2 A000000A
  .word  0x00002F06 //0E8C 03A3 2F06
  .word  0xA0000009 //0E90 03A4 A0000009
  .word  0x0000000A //0E94 03A5 000A
  .word  0x40100C74 //0E98 03A6 40100C74
  .word  0xA0000003 //0E9C 03A7 A0000003
  .word  0xFFFFFF04 //0EA0 03A8 -FC
  .word  0x00003230 //0EA4 03A9 3230
  .word  0x0000000A //0EA8 03AA 000A
  .word  0x40101E04 //0EAC 03AB 40101E04
  .word  0xA0000003 //0EB0 03AC A0000003
  .word  0xFFFFFFEC //0EB4 03AD -14
  .word  0x0000323B //0EB8 03AE 323B
  .word  0x00000007 //0EBC 03AF 0007
  .word  0x40101E04 //0EC0 03B0 40101E04
  .word  0x40100DB4 //0EC4 03B1 40100DB4
  .word  0x00003243 //0EC8 03B2 3243
  .word  0x00000019 //0ECC 03B3 0019
  .word  0x401007D4 //0ED0 03B4 401007D4
  .word  0x00000020 //0ED4 03B5 0020
  .word  0x40100C74 //0ED8 03B6 40100C74
  .word  0x00000008 //0EDC 03B7 0008
  .word  0x40100C74 //0EE0 03B8 40100C74
  .word  0x40100C48 //0EE4 03B9 40100C48
  .word  0x0000001B //0EE8 03BA 001B
  .word  0x40100A6C //0EEC 03BB 40100A6C
  .word  0x9FFFFFE0 //0EF0 03BC 9FFFFFE0
  .word  0xA0000003 //0EF4 03BD A0000003
  .word  0xFFFFFFBC //0EF8 03BE -44
  .word  0x0000325D //0EFC 03BF 325D
  .word  0x00000005 //0F00 03C0 0005
  .word  0x40101E04 //0F04 03C1 40101E04
  .word  0xB0000501 //0F08 03C2 B0000501
  .word  0x00002F0E //0F0C 03C3 2F0E
  .word  0xA0000009 //0F10 03C4 A0000009
  .word  0x00000000 //0F14 03C5 0000
  .word  0x00002F10 //0F18 03C6 2F10
  .word  0xA0000009 //0F1C 03C7 A0000009
  .word  0x40100DB4 //0F20 03C8 40100DB4
  .word  0x00002F0A //0F24 03C9 2F0A
  .word  0xA000000A //0F28 03CA A000000A
  .word  0x00002F0C //0F2C 03CB 2F0C
  .word  0xA000000A //0F30 03CC A000000A
  .word  0x00002F0A //0F34 03CD 2F0A
  .word  0xA000000A //0F38 03CE A000000A
  .word  0x40100A50 //0F3C 03CF 40100A50
  .word  0x40100A34 //0F40 03D0 40100A34
  .word  0x40100CF0 //0F44 03D1 40100CF0
  .word  0x00003263 //0F48 03D2 3263
  .word  0x00000003 //0F4C 03D3 0003
  .word  0x401007D4 //0F50 03D4 401007D4
  .word  0x00003267 //0F54 03D5 3267
  .word  0x0000000A //0F58 03D6 000A
  .word  0x401007BC //0F5C 03D7 401007BC
  .word  0x40101B18 //0F60 03D8 40101B18
  .word  0x40100DB4 //0F64 03D9 40100DB4
  .word  0x00003272 //0F68 03DA 3272
  .word  0x00000016 //0F6C 03DB 0016
  .word  0x401007D4 //0F70 03DC 401007D4
  .word  0x40100D78 //0F74 03DD 40100D78
  .word  0x40100EC4 //0F78 03DE 40100EC4
  .word  0x40101C4C //0F7C 03DF 40101C4C
  .word  0xA0000003 //0F80 03E0 A0000003
  .word  0xFFFFFF74 //0F84 03E1 -8C
  .word  0x00003289 //0F88 03E2 3289
  .word  0x00000004 //0F8C 03E3 0004
  .word  0x40101E04 //0F90 03E4 40101E04
  .word  0x00002801 //0F94 03E5 2801
  .word  0xA000000A //0F98 03E6 A000000A
  .word  0x00002F15 //0F9C 03E7 2F15
  .word  0xA0000009 //0FA0 03E8 A0000009
  .word  0xA0000003 //0FA4 03E9 A0000003
  .word  0xFFFFFFDC //0FA8 03EA -24
  .word  0x0000328E //0FAC 03EB 328E
  .word  0x00000004 //0FB0 03EC 0004
  .word  0x40101E04 //0FB4 03ED 40101E04
  .word  0x00002801 //0FB8 03EE 2801
  .word  0xA000000A //0FBC 03EF A000000A
  .word  0x00002F15 //0FC0 03F0 2F15
  .word  0xA000000A //0FC4 03F1 A000000A
  .word  0x40100A50 //0FC8 03F2 40100A50
  .word  0x90000008 //0FCC 03F3 90000008
  .word  0x00000009 //0FD0 03F4 0009
  .word  0x40100F08 //0FD4 03F5 40100F08
  .word  0xA0000003 //0FD8 03F6 A0000003
  .word  0xFFFFFFCC //0FDC 03F7 -34
  .word  0x00003293 //0FE0 03F8 3293
  .word  0x00000005 //0FE4 03F9 0005
  .word  0x40101E04 //0FE8 03FA 40101E04
  .word  0x40100A18 //0FEC 03FB 40100A18
  .word  0x00002F17 //0FF0 03FC 2F17
  .word  0xA000000A //0FF4 03FD A000000A
  .word  0xB0000502 //0FF8 03FE B0000502
  .word  0x40100A50 //0FFC 03FF 40100A50
  .word  0xB0000501 //1000 0400 B0000501
  .word  0x00002F17 //1004 0401 2F17
  .word  0xA0000009 //1008 0402 A0000009
  .word  0xA0000009 //100C 0403 A0000009
  .word  0xA0000003 //1010 0404 A0000003
  .word  0xFFFFFFC8 //1014 0405 -38
  .word  0x00003299 //1018 0406 3299
  .word  0x00000009 //101C 0407 0009
  .word  0x40101E04 //1020 0408 40101E04
  .word  0x00002F17 //1024 0409 2F17
  .word  0xA000000A //1028 040A A000000A
  .word  0xB0000501 //102C 040B B0000501
  .word  0xA000000A //1030 040C A000000A
  .word  0xA0000007 //1034 040D A0000007
  .word  0x00002F17 //1038 040E 2F17
  .word  0xA0000009 //103C 040F A0000009
  .word  0xA0000003 //1040 0410 A0000003
  .word  0xFFFFFFD0 //1044 0411 -30
  .word  0x000032A3 //1048 0412 32A3
  .word  0x00000002 //104C 0413 0002
  .word  0x40101E04 //1050 0414 40101E04
  .word  0x00002F17 //1054 0415 2F17
  .word  0xA000000A //1058 0416 A000000A
  .word  0x40100A18 //105C 0417 40100A18
  .word  0xA0000003 //1060 0418 A0000003
  .word  0xFFFFFFE0 //1064 0419 -20
  .word  0x000032A6 //1068 041A 32A6
  .word  0x00000002 //106C 041B 0002
  .word  0x40101E04 //1070 041C 40101E04
  .word  0x00002F17 //1074 041D 2F17
  .word  0xA000000A //1078 041E A000000A
  .word  0x00000002 //107C 041F 0002
  .word  0xA0000007 //1080 0420 A0000007
  .word  0xA0000003 //1084 0421 A0000003
  .word  0xFFFFFFDC //1088 0422 -24
  .word  0x000032A9 //108C 0423 32A9
  .word  0x00000002 //1090 0424 0002
  .word  0x40101E04 //1094 0425 40101E04
  .word  0x00002F17 //1098 0426 2F17
  .word  0xA000000A //109C 0427 A000000A
  .word  0x00000003 //10A0 0428 0003
  .word  0xA0000007 //10A4 0429 A0000007
  .word  0xA0000003 //10A8 042A A0000003
  .word  0xFFFFFFDC //10AC 042B -24
  .word  0x000032AC //10B0 042C 32AC
  .word  0x00000002 //10B4 042D 0002
  .word  0x40101E04 //10B8 042E 40101E04
  .word  0x00002F17 //10BC 042F 2F17
  .word  0xA000000A //10C0 0430 A000000A
  .word  0x00000004 //10C4 0431 0004
  .word  0xA0000007 //10C8 0432 A0000007
  .word  0xA0000003 //10CC 0433 A0000003
  .word  0xFFFFFFDC //10D0 0434 -24
  .word  0x000032AF //10D4 0435 32AF
  .word  0x00000002 //10D8 0436 0002
  .word  0x40101E04 //10DC 0437 40101E04
  .word  0x00002F17 //10E0 0438 2F17
  .word  0xA000000A //10E4 0439 A000000A
  .word  0x00000005 //10E8 043A 0005
  .word  0xA0000007 //10EC 043B A0000007
  .word  0xA0000003 //10F0 043C A0000003
  .word  0xFFFFFFDC //10F4 043D -24
  .word  0x000032B2 //10F8 043E 32B2
  .word  0x00000002 //10FC 043F 0002
  .word  0x40101E04 //1100 0440 40101E04
  .word  0x00002F17 //1104 0441 2F17
  .word  0xA000000A //1108 0442 A000000A
  .word  0x00000006 //110C 0443 0006
  .word  0xA0000007 //1110 0444 A0000007
  .word  0xA0000003 //1114 0445 A0000003
  .word  0xFFFFFFDC //1118 0446 -24
  .word  0x000032B5 //111C 0447 32B5
  .word  0x00000002 //1120 0448 0002
  .word  0x40101E04 //1124 0449 40101E04
  .word  0x00002F17 //1128 044A 2F17
  .word  0xA000000A //112C 044B A000000A
  .word  0x00000007 //1130 044C 0007
  .word  0xA0000007 //1134 044D A0000007
  .word  0xA0000003 //1138 044E A0000003
  .word  0xFFFFFFDC //113C 044F -24
  .word  0x000032B8 //1140 0450 32B8
  .word  0x00000002 //1144 0451 0002
  .word  0x40101E04 //1148 0452 40101E04
  .word  0x00002F17 //114C 0453 2F17
  .word  0xA000000A //1150 0454 A000000A
  .word  0x00000008 //1154 0455 0008
  .word  0xA0000007 //1158 0456 A0000007
  .word  0xA0000003 //115C 0457 A0000003
  .word  0xFFFFFFDC //1160 0458 -24
  .word  0x000032BB //1164 0459 32BB
  .word  0x00000001 //1168 045A 0001
  .word  0x40101DCC //116C 045B 40101DCC
  .word  0x00000020 //1170 045C 0020
  .word  0x40101790 //1174 045D 40101790
  .word  0x40101978 //1178 045E 40101978
  .word  0x40101A78 //117C 045F 40101A78
  .word  0xB0000300 //1180 0460 B0000300
  .word  0xA000002E //1184 0461 A000002E
  .word  0x00002F10 //1188 0462 2F10
  .word  0xA000000A //118C 0463 A000000A
  .word  0x90000004 //1190 0464 90000004
  .word  0x40100168 //1194 0465 40100168
  .word  0xA0000003 //1198 0466 A0000003
  .word  0xFFFFFFC4 //119C 0467 -3C
  .word  0x000032BD //11A0 0468 32BD
  .word  0x00000005 //11A4 0469 0005
  .word  0x40101E04 //11A8 046A 40101E04
  .word  0xB0000501 //11AC 046B B0000501
  .word  0xA000000A //11B0 046C A000000A
  .word  0x40100A18 //11B4 046D 40100A18
  .word  0xB0000501 //11B8 046E B0000501
  .word  0x000003FF //11BC 046F 03FF
  .word  0xA0000008 //11C0 0470 A0000008
  .word  0x00000000 //11C4 0471 0000
  .word  0x40100A6C //11C8 0472 40100A6C
  .word  0x90000008 //11CC 0473 90000008
  .word  0xFFFFFC00 //11D0 0474 -400
  .word  0xA0000007 //11D4 0475 A0000007
  .word  0xB0000412 //11D8 0476 B0000412
  .word  0xA0000009 //11DC 0477 A0000009
  .word  0xA0000003 //11E0 0478 A0000003
  .word  0xFFFFFFB8 //11E4 0479 -48
  .word  0x000032C3 //11E8 047A 32C3
  .word  0x00000007 //11EC 047B 0007
  .word  0x40101E04 //11F0 047C 40101E04
  .word  0x00002800 //11F4 047D 2800
  .word  0xA000000A //11F8 047E A000000A
  .word  0xB0000501 //11FC 047F B0000501
  .word  0x00000008 //1200 0480 0008
  .word  0x40100A88 //1204 0481 40100A88
  .word  0x90000024 //1208 0482 90000024
  .word  0x00000018 //120C 0483 0018
  .word  0xA0000007 //1210 0484 A0000007
  .word  0xA000000A //1214 0485 A000000A
  .word  0xB0000501 //1218 0486 B0000501
  .word  0x90000008 //121C 0487 90000008
  .word  0xB0000501 //1220 0488 B0000501
  .word  0x40100C2C //1224 0489 40100C2C
  .word  0xB0000300 //1228 048A B0000300
  .word  0x80000060 //122C 048B 80000060
  .word  0x00002F03 //1230 048C 2F03
  .word  0xA000000A //1234 048D A000000A
  .word  0xA0000009 //1238 048E A0000009
  .word  0x00002F03 //123C 048F 2F03
  .word  0x401011AC //1240 0490 401011AC
  .word  0x00002F03 //1244 0491 2F03
  .word  0xA000000A //1248 0492 A000000A
  .word  0x00002F04 //124C 0493 2F04
  .word  0xA000000A //1250 0494 A000000A
  .word  0x40100A50 //1254 0495 40100A50
  .word  0x000003FF //1258 0496 03FF
  .word  0xA0000008 //125C 0497 A0000008
  .word  0x00000080 //1260 0498 0080
  .word  0x40100AA4 //1264 0499 40100AA4
  .word  0x90000024 //1268 049A 90000024
  .word  0x00002F05 //126C 049B 2F05
  .word  0xA000000A //1270 049C A000000A
  .word  0xA000000D //1274 049D A000000D
  .word  0x90000014 //1278 049E 90000014
  .word  0xFFFFFFFF //127C 049F -1
  .word  0x00002F05 //1280 04A0 2F05
  .word  0xA0000009 //1284 04A1 A0000009
  .word  0x00000013 //1288 04A2 0013
  .word  0x40100C74 //128C 04A3 40100C74
  .word  0x00000000 //1290 04A4 0000
  .word  0x00002800 //1294 04A5 2800
  .word  0xA0000009 //1298 04A6 A0000009
  .word  0xA0000003 //129C 04A7 A0000003
  .word  0xFFFFFF44 //12A0 04A8 -BC
  .word  0x000032CB //12A4 04A9 32CB
  .word  0x00000008 //12A8 04AA 0008
  .word  0x40101E04 //12AC 04AB 40101E04
  .word  0x00002F04 //12B0 04AC 2F04
  .word  0xA000000A //12B4 04AD A000000A
  .word  0x00002F03 //12B8 04AE 2F03
  .word  0xA000000A //12BC 04AF A000000A
  .word  0x40100A6C //12C0 04B0 40100A6C
  .word  0x9000000C //12C4 04B1 9000000C
  .word  0x00000000 //12C8 04B2 0000
  .word  0x00000000 //12CC 04B3 0000
  .word  0x80000060 //12D0 04B4 80000060
  .word  0x00002F04 //12D4 04B5 2F04
  .word  0xA000000A //12D8 04B6 A000000A
  .word  0xA000000A //12DC 04B7 A000000A
  .word  0xFFFFFFFF //12E0 04B8 -1
  .word  0x00002F04 //12E4 04B9 2F04
  .word  0x401011AC //12E8 04BA 401011AC
  .word  0x00002F03 //12EC 04BB 2F03
  .word  0xA000000A //12F0 04BC A000000A
  .word  0x00002F04 //12F4 04BD 2F04
  .word  0xA000000A //12F8 04BE A000000A
  .word  0x40100A50 //12FC 04BF 40100A50
  .word  0x000003FF //1300 04C0 03FF
  .word  0xA0000008 //1304 04C1 A0000008
  .word  0x00000020 //1308 04C2 0020
  .word  0x40100A88 //130C 04C3 40100A88
  .word  0x90000020 //1310 04C4 90000020
  .word  0x00002F05 //1314 04C5 2F05
  .word  0xA000000A //1318 04C6 A000000A
  .word  0x90000014 //131C 04C7 90000014
  .word  0x00000000 //1320 04C8 0000
  .word  0x00002F05 //1324 04C9 2F05
  .word  0xA0000009 //1328 04CA A0000009
  .word  0x00000011 //132C 04CB 0011
  .word  0x40100C74 //1330 04CC 40100C74
  .word  0xA0000003 //1334 04CD A0000003
  .word  0xFFFFFF68 //1338 04CE -98
  .word  0x000032D4 //133C 04CF 32D4
  .word  0x00000006 //1340 04D0 0006
  .word  0x40101E04 //1344 04D1 40101E04
  .word  0x00000005 //1348 04D2 0005
  .word  0x40100FEC //134C 04D3 40100FEC
  .word  0x40101098 //1350 04D4 40101098
  .word  0xA0000009 //1354 04D5 A0000009
  .word  0x40101074 //1358 04D6 40101074
  .word  0xA0000009 //135C 04D7 A0000009
  .word  0x40101074 //1360 04D8 40101074
  .word  0xA000000A //1364 04D9 A000000A
  .word  0x401010E0 //1368 04DA 401010E0
  .word  0xA0000009 //136C 04DB A0000009
  .word  0x40100C48 //1370 04DC 40100C48
  .word  0xB0000501 //1374 04DD B0000501
  .word  0x00000014 //1378 04DE 0014
  .word  0x40100A6C //137C 04DF 40100A6C
  .word  0x90000010 //1380 04E0 90000010
  .word  0xB0000300 //1384 04E1 B0000300
  .word  0x40101074 //1388 04E2 40101074
  .word  0xA000000A //138C 04E3 A000000A
  .word  0x401009E8 //1390 04E4 401009E8
  .word  0xB0000501 //1394 04E5 B0000501
  .word  0x0000007F //1398 04E6 007F
  .word  0x40100A6C //139C 04E7 40100A6C
  .word  0x90000008 //13A0 04E8 90000008
  .word  0xB0000300 //13A4 04E9 B0000300
  .word  0x00000008 //13A8 04EA 0008
  .word  0xB0000501 //13AC 04EB B0000501
  .word  0x00000008 //13B0 04EC 0008
  .word  0x40100A6C //13B4 04ED 40100A6C
  .word  0x90000048 //13B8 04EE 90000048
  .word  0x401010E0 //13BC 04EF 401010E0
  .word  0xA000000A //13C0 04F0 A000000A
  .word  0x40101074 //13C4 04F1 40101074
  .word  0xA000000A //13C8 04F2 A000000A
  .word  0x40100A88 //13CC 04F3 40100A88
  .word  0x90000030 //13D0 04F4 90000030
  .word  0xFFFFFFFF //13D4 04F5 -1
  .word  0x40101074 //13D8 04F6 40101074
  .word  0x40100B10 //13DC 04F7 40100B10
  .word  0x00000001 //13E0 04F8 0001
  .word  0x40101098 //13E4 04F9 40101098
  .word  0x40100B10 //13E8 04FA 40100B10
  .word  0x00000008 //13EC 04FB 0008
  .word  0x40100C74 //13F0 04FC 40100C74
  .word  0x00000020 //13F4 04FD 0020
  .word  0x40100C74 //13F8 04FE 40100C74
  .word  0x00000008 //13FC 04FF 0008
  .word  0x40100C74 //1400 0500 40100C74
  .word  0xB0000501 //1404 0501 B0000501
  .word  0x00000020 //1408 0502 0020
  .word  0x40100A88 //140C 0503 40100A88
  .word  0x90000004 //1410 0504 90000004
  .word  0x80000048 //1414 0505 80000048
  .word  0xFFFFFFFF //1418 0506 -1
  .word  0x40101098 //141C 0507 40101098
  .word  0x40100B10 //1420 0508 40100B10
  .word  0x40101098 //1424 0509 40101098
  .word  0xA000000A //1428 050A A000000A
  .word  0xA000000F //142C 050B A000000F
  .word  0x90000008 //1430 050C 90000008
  .word  0x00000006 //1434 050D 0006
  .word  0x40100F08 //1438 050E 40100F08
  .word  0xB0000501 //143C 050F B0000501
  .word  0x40100C74 //1440 0510 40100C74
  .word  0xB0000501 //1444 0511 B0000501
  .word  0x40101074 //1448 0512 40101074
  .word  0xA000000A //144C 0513 A000000A
  .word  0x40100A00 //1450 0514 40100A00
  .word  0x00000001 //1454 0515 0001
  .word  0x40101074 //1458 0516 40101074
  .word  0x40100B10 //145C 0517 40100B10
  .word  0xB0000501 //1460 0518 B0000501
  .word  0x00000020 //1464 0519 0020
  .word  0x40100A88 //1468 051A 40100A88
  .word  0xB0000502 //146C 051B B0000502
  .word  0x00000008 //1470 051C 0008
  .word  0x40100A6C //1474 051D 40100A6C
  .word  0xA000000B //1478 051E A000000B
  .word  0xA0000008 //147C 051F A0000008
  .word  0xB0000412 //1480 0520 B0000412
  .word  0x0000001B //1484 0521 001B
  .word  0x40100A6C //1488 0522 40100A6C
  .word  0xA000000B //148C 0523 A000000B
  .word  0xA0000008 //1490 0524 A0000008
  .word  0x40101098 //1494 0525 40101098
  .word  0xA000000A //1498 0526 A000000A
  .word  0xA000000D //149C 0527 A000000D
  .word  0xA000000E //14A0 0528 A000000E
  .word  0x9FFFFEC8 //14A4 0529 9FFFFEC8
  .word  0x00000020 //14A8 052A 0020
  .word  0x40100C74 //14AC 052B 40100C74
  .word  0x401010E0 //14B0 052C 401010E0
  .word  0xA000000A //14B4 052D A000000A
  .word  0x40101074 //14B8 052E 40101074
  .word  0xA000000A //14BC 052F A000000A
  .word  0x401010E0 //14C0 0530 401010E0
  .word  0xA000000A //14C4 0531 A000000A
  .word  0x40100A50 //14C8 0532 40100A50
  .word  0xB0000603 //14CC 0533 B0000603
  .word  0xA0000007 //14D0 0534 A0000007
  .word  0x00000000 //14D4 0535 0000
  .word  0xB0000412 //14D8 0536 B0000412
  .word  0x40100A00 //14DC 0537 40100A00
  .word  0x40101024 //14E0 0538 40101024
  .word  0xA0000003 //14E4 0539 A0000003
  .word  0xFFFFFE50 //14E8 053A -1B0
  .word  0x000032DB //14EC 053B 32DB
  .word  0x00000005 //14F0 053C 0005
  .word  0x40101E04 //14F4 053D 40101E04
  .word  0xB0000501 //14F8 053E B0000501
  .word  0x00000030 //14FC 053F 0030
  .word  0x40100A88 //1500 0540 40100A88
  .word  0xA000000B //1504 0541 A000000B
  .word  0xB0000502 //1508 0542 B0000502
  .word  0x0000003A //150C 0543 003A
  .word  0x40100A88 //1510 0544 40100A88
  .word  0xA0000008 //1514 0545 A0000008
  .word  0xB0000502 //1518 0546 B0000502
  .word  0x00000041 //151C 0547 0041
  .word  0x40100A88 //1520 0548 40100A88
  .word  0xA000000B //1524 0549 A000000B
  .word  0xA000000E //1528 054A A000000E
  .word  0xB0000501 //152C 054B B0000501
  .word  0x90000054 //1530 054C 90000054
  .word  0xB0000412 //1534 054D B0000412
  .word  0x00000030 //1538 054E 0030
  .word  0x40100A50 //153C 054F 40100A50
  .word  0xB0000501 //1540 0550 B0000501
  .word  0x0000000A //1544 0551 000A
  .word  0x40100A88 //1548 0552 40100A88
  .word  0xA000000B //154C 0553 A000000B
  .word  0x90000008 //1550 0554 90000008
  .word  0x00000007 //1554 0555 0007
  .word  0x40100A50 //1558 0556 40100A50
  .word  0xB0000501 //155C 0557 B0000501
  .word  0x00002F08 //1560 0558 2F08
  .word  0xA000000A //1564 0559 A000000A
  .word  0x40100A88 //1568 055A 40100A88
  .word  0xA000000B //156C 055B A000000B
  .word  0x90000010 //1570 055C 90000010
  .word  0xB0000300 //1574 055D B0000300
  .word  0xB0000300 //1578 055E B0000300
  .word  0x00000000 //157C 055F 0000
  .word  0x00000000 //1580 0560 0000
  .word  0xB0000412 //1584 0561 B0000412
  .word  0xA0000003 //1588 0562 A0000003
  .word  0xFFFFFF5C //158C 0563 -A4
  .word  0x000032E1 //1590 0564 32E1
  .word  0x00000006 //1594 0565 0006
  .word  0x40101E04 //1598 0566 40101E04
  .word  0x00000007 //159C 0567 0007
  .word  0x40100FEC //15A0 0568 40100FEC
  .word  0x40101074 //15A4 0569 40101074
  .word  0xA0000009 //15A8 056A A0000009
  .word  0x40101054 //15AC 056B 40101054
  .word  0xA0000009 //15B0 056C A0000009
  .word  0x00000000 //15B4 056D 0000
  .word  0x40101074 //15B8 056E 40101074
  .word  0xA000000A //15BC 056F A000000A
  .word  0x9000018C //15C0 0570 9000018C
  .word  0xB0000501 //15C4 0571 B0000501
  .word  0x40101098 //15C8 0572 40101098
  .word  0xA0000009 //15CC 0573 A0000009
  .word  0x00000001 //15D0 0574 0001
  .word  0x40101104 //15D4 0575 40101104
  .word  0xA0000009 //15D8 0576 A0000009
  .word  0xFFFFFFFF //15DC 0577 -1
  .word  0x40101128 //15E0 0578 40101128
  .word  0xA0000009 //15E4 0579 A0000009
  .word  0x40101054 //15E8 057A 40101054
  .word  0xA000000A //15EC 057B A000000A
  .word  0x40101098 //15F0 057C 40101098
  .word  0xA000000A //15F4 057D A000000A
  .word  0xA0000007 //15F8 057E A0000007
  .word  0x401009E8 //15FC 057F 401009E8
  .word  0x0000002B //1600 0580 002B
  .word  0x40100A6C //1604 0581 40100A6C
  .word  0x90000024 //1608 0582 90000024
  .word  0x40101098 //160C 0583 40101098
  .word  0xA000000A //1610 0584 A000000A
  .word  0x40100A18 //1614 0585 40100A18
  .word  0x40101098 //1618 0586 40101098
  .word  0xA0000009 //161C 0587 A0000009
  .word  0x00000000 //1620 0588 0000
  .word  0x40101128 //1624 0589 40101128
  .word  0xA0000009 //1628 058A A0000009
  .word  0x80000058 //162C 058B 80000058
  .word  0x40101054 //1630 058C 40101054
  .word  0xA000000A //1634 058D A000000A
  .word  0x40101098 //1638 058E 40101098
  .word  0xA000000A //163C 058F A000000A
  .word  0xA0000007 //1640 0590 A0000007
  .word  0x401009E8 //1644 0591 401009E8
  .word  0x0000002D //1648 0592 002D
  .word  0x40100A6C //164C 0593 40100A6C
  .word  0x90000034 //1650 0594 90000034
  .word  0x40101098 //1654 0595 40101098
  .word  0xA000000A //1658 0596 A000000A
  .word  0x40100A18 //165C 0597 40100A18
  .word  0x40101098 //1660 0598 40101098
  .word  0xA0000009 //1664 0599 A0000009
  .word  0x00000000 //1668 059A 0000
  .word  0x40101128 //166C 059B 40101128
  .word  0xA0000009 //1670 059C A0000009
  .word  0x40101104 //1674 059D 40101104
  .word  0xA000000A //1678 059E A000000A
  .word  0xA0000000 //167C 059F A0000000
  .word  0x40101104 //1680 05A0 40101104
  .word  0xA0000009 //1684 05A1 A0000009
  .word  0x40101128 //1688 05A2 40101128
  .word  0xA000000A //168C 05A3 A000000A
  .word  0x9FFFFF48 //1690 05A4 9FFFFF48
  .word  0x40101098 //1694 05A5 40101098
  .word  0xA000000A //1698 05A6 A000000A
  .word  0x40101074 //169C 05A7 40101074
  .word  0xA000000A //16A0 05A8 A000000A
  .word  0x40100A88 //16A4 05A9 40100A88
  .word  0x900000A4 //16A8 05AA 900000A4
  .word  0x40101054 //16AC 05AB 40101054
  .word  0xA000000A //16B0 05AC A000000A
  .word  0x40101098 //16B4 05AD 40101098
  .word  0xA000000A //16B8 05AE A000000A
  .word  0xA0000007 //16BC 05AF A0000007
  .word  0x401009E8 //16C0 05B0 401009E8
  .word  0xB0000501 //16C4 05B1 B0000501
  .word  0x90000054 //16C8 05B2 90000054
  .word  0x401014F8 //16CC 05B3 401014F8
  .word  0xA000000B //16D0 05B4 A000000B
  .word  0x9000001C //16D4 05B5 9000001C
  .word  0xB0000300 //16D8 05B6 B0000300
  .word  0x40101074 //16DC 05B7 40101074
  .word  0xA000000A //16E0 05B8 A000000A
  .word  0xA0000000 //16E4 05B9 A0000000
  .word  0x40101074 //16E8 05BA 40101074
  .word  0xA0000009 //16EC 05BB A0000009
  .word  0x80000028 //16F0 05BC 80000028
  .word  0xB0000412 //16F4 05BD B0000412
  .word  0x00002F08 //16F8 05BE 2F08
  .word  0xA000000A //16FC 05BF A000000A
  .word  0x40100AC0 //1700 05C0 40100AC0
  .word  0xA0000007 //1704 05C1 A0000007
  .word  0x40101098 //1708 05C2 40101098
  .word  0xA000000A //170C 05C3 A000000A
  .word  0x40100A18 //1710 05C4 40100A18
  .word  0x40101098 //1714 05C5 40101098
  .word  0xA0000009 //1718 05C6 A0000009
  .word  0x80000014 //171C 05C7 80000014
  .word  0xB0000300 //1720 05C8 B0000300
  .word  0x40101098 //1724 05C9 40101098
  .word  0xA000000A //1728 05CA A000000A
  .word  0x40101074 //172C 05CB 40101074
  .word  0xA0000009 //1730 05CC A0000009
  .word  0x40101098 //1734 05CD 40101098
  .word  0xA000000A //1738 05CE A000000A
  .word  0x40101074 //173C 05CF 40101074
  .word  0xA000000A //1740 05D0 A000000A
  .word  0x40100A88 //1744 05D1 40100A88
  .word  0xA000000B //1748 05D2 A000000B
  .word  0x9FFFFF5C //174C 05D3 9FFFFF5C
  .word  0x40101104 //1750 05D4 40101104
  .word  0xA000000A //1754 05D5 A000000A
  .word  0xA000000F //1758 05D6 A000000F
  .word  0x90000004 //175C 05D7 90000004
  .word  0xA0000000 //1760 05D8 A0000000
  .word  0x40101098 //1764 05D9 40101098
  .word  0xA000000A //1768 05DA A000000A
  .word  0x40101074 //176C 05DB 40101074
  .word  0xA000000A //1770 05DC A000000A
  .word  0x40100A50 //1774 05DD 40100A50
  .word  0x40101024 //1778 05DE 40101024
  .word  0xA0000003 //177C 05DF A0000003
  .word  0xFFFFFE0C //1780 05E0 -1F4
  .word  0x000032E8 //1784 05E1 32E8
  .word  0x00000004 //1788 05E2 0004
  .word  0x40101E04 //178C 05E3 40101E04
  .word  0x40100B88 //1790 05E4 40100B88
  .word  0x00002F0C //1794 05E5 2F0C
  .word  0xA000000A //1798 05E6 A000000A
  .word  0x00002F0B //179C 05E7 2F0B
  .word  0xA0000009 //17A0 05E8 A0000009
  .word  0x00002F0C //17A4 05E9 2F0C
  .word  0xA000000A //17A8 05EA A000000A
  .word  0x401009E8 //17AC 05EB 401009E8
  .word  0x40100BDC //17B0 05EC 40100BDC
  .word  0x40100A6C //17B4 05ED 40100A6C
  .word  0x00002F0C //17B8 05EE 2F0C
  .word  0xA000000A //17BC 05EF A000000A
  .word  0x00002F0D //17C0 05F0 2F0D
  .word  0xA000000A //17C4 05F1 A000000A
  .word  0x40100A88 //17C8 05F2 40100A88
  .word  0xA0000008 //17CC 05F3 A0000008
  .word  0x90000010 //17D0 05F4 90000010
  .word  0x00000001 //17D4 05F5 0001
  .word  0x00002F0C //17D8 05F6 2F0C
  .word  0x40100B10 //17DC 05F7 40100B10
  .word  0x8FFFFFC0 //17E0 05F8 8FFFFFC0
  .word  0x00002F0C //17E4 05F9 2F0C
  .word  0xA000000A //17E8 05FA A000000A
  .word  0x00002F0B //17EC 05FB 2F0B
  .word  0xA0000009 //17F0 05FC A0000009
  .word  0x00002F0C //17F4 05FD 2F0C
  .word  0xA000000A //17F8 05FE A000000A
  .word  0x401009E8 //17FC 05FF 401009E8
  .word  0x0000003C //1800 0600 003C
  .word  0x40100A6C //1804 0601 40100A6C
  .word  0x90000010 //1808 0602 90000010
  .word  0x00002F0C //180C 0603 2F0C
  .word  0xA000000A //1810 0604 A000000A
  .word  0x00002F0D //1814 0605 2F0D
  .word  0xA0000009 //1818 0606 A0000009
  .word  0x00002F0C //181C 0607 2F0C
  .word  0xA000000A //1820 0608 A000000A
  .word  0x401009E8 //1824 0609 401009E8
  .word  0x40100BDC //1828 060A 40100BDC
  .word  0x40100A6C //182C 060B 40100A6C
  .word  0xA000000B //1830 060C A000000B
  .word  0x00002F0C //1834 060D 2F0C
  .word  0xA000000A //1838 060E A000000A
  .word  0x00002F0D //183C 060F 2F0D
  .word  0xA000000A //1840 0610 A000000A
  .word  0x40100A88 //1844 0611 40100A88
  .word  0xA0000008 //1848 0612 A0000008
  .word  0x90000010 //184C 0613 90000010
  .word  0x00000001 //1850 0614 0001
  .word  0x00002F0C //1854 0615 2F0C
  .word  0x40100B10 //1858 0616 40100B10
  .word  0x8FFFFF94 //185C 0617 8FFFFF94
  .word  0x00002F0B //1860 0618 2F0B
  .word  0xA000000A //1864 0619 A000000A
  .word  0x00002F0C //1868 061A 2F0C
  .word  0xA000000A //186C 061B A000000A
  .word  0xB0000502 //1870 061C B0000502
  .word  0x40100A50 //1874 061D 40100A50
  .word  0xB0000501 //1878 061E B0000501
  .word  0x9000000C //187C 061F 9000000C
  .word  0x00000001 //1880 0620 0001
  .word  0x00002F0C //1884 0621 2F0C
  .word  0x40100B10 //1888 0622 40100B10
  .word  0x40100B3C //188C 0623 40100B3C
  .word  0xB0000300 //1890 0624 B0000300
  .word  0xA0000003 //1894 0625 A0000003
  .word  0xFFFFFEE8 //1898 0626 -118
  .word  0x000032ED //189C 0627 32ED
  .word  0x00000002 //18A0 0628 0002
  .word  0x40101E04 //18A4 0629 40101E04
  .word  0x40100B88 //18A8 062A 40100B88
  .word  0xB0000502 //18AC 062B B0000502
  .word  0x40100BDC //18B0 062C 40100BDC
  .word  0x40100A50 //18B4 062D 40100A50
  .word  0x9000001C //18B8 062E 9000001C
  .word  0x40100B3C //18BC 062F 40100B3C
  .word  0xB0000300 //18C0 0630 B0000300
  .word  0xB0000300 //18C4 0631 B0000300
  .word  0xB0000300 //18C8 0632 B0000300
  .word  0xB0000300 //18CC 0633 B0000300
  .word  0x00000000 //18D0 0634 0000
  .word  0x8000008C //18D4 0635 8000008C
  .word  0x40100B3C //18D8 0636 40100B3C
  .word  0xB0000300 //18DC 0637 B0000300
  .word  0xB0000412 //18E0 0638 B0000412
  .word  0x00000000 //18E4 0639 0000
  .word  0xB0000603 //18E8 063A B0000603
  .word  0x40100A50 //18EC 063B 40100A50
  .word  0x90000058 //18F0 063C 90000058
  .word  0x40100B88 //18F4 063D 40100B88
  .word  0x40100B88 //18F8 063E 40100B88
  .word  0xB0000502 //18FC 063F B0000502
  .word  0x401009E8 //1900 0640 401009E8
  .word  0xB0000502 //1904 0641 B0000502
  .word  0x401009E8 //1908 0642 401009E8
  .word  0x40100A50 //190C 0643 40100A50
  .word  0x90000010 //1910 0644 90000010
  .word  0xB0000300 //1914 0645 B0000300
  .word  0xB0000300 //1918 0646 B0000300
  .word  0x00000000 //191C 0647 0000
  .word  0x00000000 //1920 0648 0000
  .word  0xB0000501 //1924 0649 B0000501
  .word  0x90000010 //1928 064A 90000010
  .word  0x40100A18 //192C 064B 40100A18
  .word  0xB0000412 //1930 064C B0000412
  .word  0x40100A18 //1934 064D 40100A18
  .word  0xB0000412 //1938 064E B0000412
  .word  0x40100B3C //193C 064F 40100B3C
  .word  0x40100B3C //1940 0650 40100B3C
  .word  0x40100A18 //1944 0651 40100A18
  .word  0x8FFFFF9C //1948 0652 8FFFFF9C
  .word  0xB0000200 //194C 0653 B0000200
  .word  0xB0000300 //1950 0654 B0000300
  .word  0x90000008 //1954 0655 90000008
  .word  0xFFFFFFFF //1958 0656 -1
  .word  0x80000004 //195C 0657 80000004
  .word  0x00000000 //1960 0658 0000
  .word  0xA0000003 //1964 0659 A0000003
  .word  0xFFFFFF30 //1968 065A -D0
  .word  0x000032F0 //196C 065B 32F0
  .word  0x00000004 //1970 065C 0004
  .word  0x40101E04 //1974 065D 40101E04
  .word  0x40100B88 //1978 065E 40100B88
  .word  0x40100B88 //197C 065F 40100B88
  .word  0x00000000 //1980 0660 0000
  .word  0x00002F11 //1984 0661 2F11
  .word  0xA000000A //1988 0662 A000000A
  .word  0x00002F01 //198C 0663 2F01
  .word  0xA000000A //1990 0664 A000000A
  .word  0x9000000C //1994 0665 9000000C
  .word  0xB0000501 //1998 0666 B0000501
  .word  0xA000000A //199C 0667 A000000A
  .word  0xA0000007 //19A0 0668 A0000007
  .word  0xB0000501 //19A4 0669 B0000501
  .word  0xA000002E //19A8 066A A000002E
  .word  0xB0000501 //19AC 066B B0000501
  .word  0xA000000A //19B0 066C A000000A
  .word  0xB0000412 //19B4 066D B0000412
  .word  0xA000002E //19B8 066E A000002E
  .word  0xA000000A //19BC 066F A000000A
  .word  0x40100B3C //19C0 0670 40100B3C
  .word  0x40100B3C //19C4 0671 40100B3C
  .word  0xB0000603 //19C8 0672 B0000603
  .word  0x40100B88 //19CC 0673 40100B88
  .word  0x40100B88 //19D0 0674 40100B88
  .word  0x401018A8 //19D4 0675 401018A8
  .word  0x9000000C //19D8 0676 9000000C
  .word  0xB0000412 //19DC 0677 B0000412
  .word  0xA000000D //19E0 0678 A000000D
  .word  0xB0000412 //19E4 0679 B0000412
  .word  0xB0000502 //19E8 067A B0000502
  .word  0xA000000D //19EC 067B A000000D
  .word  0xB0000502 //19F0 067C B0000502
  .word  0xA000000A //19F4 067D A000000A
  .word  0xA000000D //19F8 067E A000000D
  .word  0xA000000B //19FC 067F A000000B
  .word  0xA0000008 //1A00 0680 A0000008
  .word  0xB0000502 //1A04 0681 B0000502
  .word  0xB0000501 //1A08 0682 B0000501
  .word  0xA000000A //1A0C 0683 A000000A
  .word  0xA0000007 //1A10 0684 A0000007
  .word  0x00002F11 //1A14 0685 2F11
  .word  0xA000000A //1A18 0686 A000000A
  .word  0x40100A6C //1A1C 0687 40100A6C
  .word  0xA000000B //1A20 0688 A000000B
  .word  0xA0000008 //1A24 0689 A0000008
  .word  0x90000010 //1A28 068A 90000010
  .word  0xB0000501 //1A2C 068B B0000501
  .word  0xA000000A //1A30 068C A000000A
  .word  0xA0000007 //1A34 068D A0000007
  .word  0x8FFFFF68 //1A38 068E 8FFFFF68
  .word  0x40100B3C //1A3C 068F 40100B3C
  .word  0xB0000300 //1A40 0690 B0000300
  .word  0x40100B3C //1A44 0691 40100B3C
  .word  0xB0000434 //1A48 0692 B0000434
  .word  0xA000000D //1A4C 0693 A000000D
  .word  0x90000010 //1A50 0694 90000010
  .word  0xB0000300 //1A54 0695 B0000300
  .word  0xB0000300 //1A58 0696 B0000300
  .word  0x00000000 //1A5C 0697 0000
  .word  0x00000000 //1A60 0698 0000
  .word  0xA0000003 //1A64 0699 A0000003
  .word  0xFFFFFF00 //1A68 069A -100
  .word  0x000032F5 //1A6C 069B 32F5
  .word  0x00000004 //1A70 069C 0004
  .word  0x40101E04 //1A74 069D 40101E04
  .word  0xB0000412 //1A78 069E B0000412
  .word  0x0000000C //1A7C 069F 000C
  .word  0xA0000007 //1A80 06A0 A0000007
  .word  0xB0000412 //1A84 06A1 B0000412
  .word  0xA0000003 //1A88 06A2 A0000003
  .word  0xFFFFFFDC //1A8C 06A3 -24
  .word  0x000032FA //1A90 06A4 32FA
  .word  0x00000008 //1A94 06A5 0008
  .word  0x40101E04 //1A98 06A6 40101E04
  .word  0x00000004 //1A9C 06A7 0004
  .word  0x00000000 //1AA0 06A8 0000
  .word  0x4010011C //1AA4 06A9 4010011C
  .word  0xA000000E //1AA8 06AA A000000E
  .word  0x40100C00 //1AAC 06AB 40100C00
  .word  0xA0000003 //1AB0 06AC A0000003
  .word  0xFFFFFFD8 //1AB4 06AD -28
  .word  0x00003303 //1AB8 06AE 3303
  .word  0x00000006 //1ABC 06AF 0006
  .word  0x40101E04 //1AC0 06B0 40101E04
  .word  0x40100F94 //1AC4 06B1 40100F94
  .word  0x00002F0F //1AC8 06B2 2F0F
  .word  0xA000000A //1ACC 06B3 A000000A
  .word  0x00002F11 //1AD0 06B4 2F11
  .word  0xA000000A //1AD4 06B5 A000000A
  .word  0xB0000502 //1AD8 06B6 B0000502
  .word  0x40100A50 //1ADC 06B7 40100A50
  .word  0x40100C00 //1AE0 06B8 40100C00
  .word  0x00002F11 //1AE4 06B9 2F11
  .word  0xA0000009 //1AE8 06BA A0000009
  .word  0x00000020 //1AEC 06BB 0020
  .word  0x40101790 //1AF0 06BC 40101790
  .word  0x401006F4 //1AF4 06BD 401006F4
  .word  0x00000001 //1AF8 06BE 0001
  .word  0x00002F01 //1AFC 06BF 2F01
  .word  0xA0000009 //1B00 06C0 A0000009
  .word  0xA0000003 //1B04 06C1 A0000003
  .word  0xFFFFFFAC //1B08 06C2 -54
  .word  0x0000330A //1B0C 06C3 330A
  .word  0x00000009 //1B10 06C4 0009
  .word  0x40101E04 //1B14 06C5 40101E04
  .word  0x00002F0A //1B18 06C6 2F0A
  .word  0xA000000A //1B1C 06C7 A000000A
  .word  0x40100B88 //1B20 06C8 40100B88
  .word  0x00002F0B //1B24 06C9 2F0B
  .word  0xA000000A //1B28 06CA A000000A
  .word  0x40100B88 //1B2C 06CB 40100B88
  .word  0x00002F0C //1B30 06CC 2F0C
  .word  0xA000000A //1B34 06CD A000000A
  .word  0x40100B88 //1B38 06CE 40100B88
  .word  0x00002F0D //1B3C 06CF 2F0D
  .word  0xA000000A //1B40 06D0 A000000A
  .word  0x40100B88 //1B44 06D1 40100B88
  .word  0xB0000502 //1B48 06D2 B0000502
  .word  0xA0000007 //1B4C 06D3 A0000007
  .word  0x00002F0D //1B50 06D4 2F0D
  .word  0xA0000009 //1B54 06D5 A0000009
  .word  0xB0000501 //1B58 06D6 B0000501
  .word  0x00002F0A //1B5C 06D7 2F0A
  .word  0xA0000009 //1B60 06D8 A0000009
  .word  0xB0000501 //1B64 06D9 B0000501
  .word  0x00002F0B //1B68 06DA 2F0B
  .word  0xA0000009 //1B6C 06DB A0000009
  .word  0x00002F0C //1B70 06DC 2F0C
  .word  0xA0000009 //1B74 06DD A0000009
  .word  0x00000020 //1B78 06DE 0020
  .word  0x40101790 //1B7C 06DF 40101790
  .word  0xB0000501 //1B80 06E0 B0000501
  .word  0x9000007C //1B84 06E1 9000007C
  .word  0xB0000603 //1B88 06E2 B0000603
  .word  0x40101978 //1B8C 06E3 40101978
  .word  0xB0000501 //1B90 06E4 B0000501
  .word  0x90000024 //1B94 06E5 90000024
  .word  0x40100B88 //1B98 06E6 40100B88
  .word  0x40100B88 //1B9C 06E7 40100B88
  .word  0xB0000200 //1BA0 06E8 B0000200
  .word  0x40100B3C //1BA4 06E9 40100B3C
  .word  0x40100B3C //1BA8 06EA 40100B3C
  .word  0x40101A78 //1BAC 06EB 40101A78
  .word  0xB0000300 //1BB0 06EC B0000300
  .word  0x40100C2C //1BB4 06ED 40100C2C
  .word  0x80000044 //1BB8 06EE 80000044
  .word  0xB0000200 //1BBC 06EF B0000200
  .word  0xB0000603 //1BC0 06F0 B0000603
  .word  0x4010159C //1BC4 06F1 4010159C
  .word  0x90000014 //1BC8 06F2 90000014
  .word  0xB0000200 //1BCC 06F3 B0000200
  .word  0xB0000300 //1BD0 06F4 B0000300
  .word  0x00000003 //1BD4 06F5 0003
  .word  0x40100F08 //1BD8 06F6 40100F08
  .word  0x80000020 //1BDC 06F7 80000020
  .word  0xB0000434 //1BE0 06F8 B0000434
  .word  0xB0000300 //1BE4 06F9 B0000300
  .word  0xB0000412 //1BE8 06FA B0000412
  .word  0xB0000300 //1BEC 06FB B0000300
  .word  0x00002F10 //1BF0 06FC 2F10
  .word  0xA000000A //1BF4 06FD A000000A
  .word  0x90000004 //1BF8 06FE 90000004
  .word  0x40100168 //1BFC 06FF 40100168
  .word  0x8FFFFF74 //1C00 0700 8FFFFF74
  .word  0xB0000200 //1C04 0701 B0000200
  .word  0x40100B3C //1C08 0702 40100B3C
  .word  0x00002F0D //1C0C 0703 2F0D
  .word  0xA0000009 //1C10 0704 A0000009
  .word  0x40100B3C //1C14 0705 40100B3C
  .word  0x00002F0C //1C18 0706 2F0C
  .word  0xA0000009 //1C1C 0707 A0000009
  .word  0x40100B3C //1C20 0708 40100B3C
  .word  0x00002F0B //1C24 0709 2F0B
  .word  0xA0000009 //1C28 070A A0000009
  .word  0x40100B3C //1C2C 070B 40100B3C
  .word  0x00002F0A //1C30 070C 2F0A
  .word  0xA0000009 //1C34 070D A0000009
  .word  0xA0000003 //1C38 070E A0000003
  .word  0xFFFFFECC //1C3C 070F -134
  .word  0x00003314 //1C40 0710 3314
  .word  0x00000004 //1C44 0711 0004
  .word  0x40101E04 //1C48 0712 40101E04
  .word  0x00002F02 //1C4C 0713 2F02
  .word  0xA000000A //1C50 0714 A000000A
  .word  0x00002802 //1C54 0715 2802
  .word  0xA0000009 //1C58 0716 A0000009
  .word  0x00002F00 //1C5C 0717 2F00
  .word  0xA000000A //1C60 0718 A000000A
  .word  0x90000018 //1C64 0719 90000018
  .word  0x0000003C //1C68 071A 003C
  .word  0x40100C74 //1C6C 071B 40100C74
  .word  0x00003319 //1C70 071C 3319
  .word  0x00000004 //1C74 071D 0004
  .word  0x401007D4 //1C78 071E 401007D4
  .word  0x8000000C //1C7C 071F 8000000C
  .word  0x0000331E //1C80 0720 331E
  .word  0x00000002 //1C84 0721 0002
  .word  0x401007D4 //1C88 0722 401007D4
  .word  0x40100DB4 //1C8C 0723 40100DB4
  .word  0x00002F09 //1C90 0724 2F09
  .word  0xA000000A //1C94 0725 A000000A
  .word  0x00000100 //1C98 0726 0100
  .word  0x40101348 //1C9C 0727 40101348
  .word  0xB0000502 //1CA0 0728 B0000502
  .word  0xA000000A //1CA4 0729 A000000A
  .word  0x0000003C //1CA8 072A 003C
  .word  0x40100A6C //1CAC 072B 40100A6C
  .word  0x90000008 //1CB0 072C 90000008
  .word  0xB0000200 //1CB4 072D B0000200
  .word  0x800000AC //1CB8 072E 800000AC
  .word  0x00002F00 //1CBC 072F 2F00
  .word  0xA000000A //1CC0 0730 A000000A
  .word  0x90000030 //1CC4 0731 90000030
  .word  0x0000003C //1CC8 0732 003C
  .word  0x40100C74 //1CCC 0733 40100C74
  .word  0x00003321 //1CD0 0734 3321
  .word  0x00000003 //1CD4 0735 0003
  .word  0x401007D4 //1CD8 0736 401007D4
  .word  0x40101B18 //1CDC 0737 40101B18
  .word  0x0000003C //1CE0 0738 003C
  .word  0x40100C74 //1CE4 0739 40100C74
  .word  0x00003325 //1CE8 073A 3325
  .word  0x00000004 //1CEC 073B 0004
  .word  0x401007D4 //1CF0 073C 401007D4
  .word  0x80000070 //1CF4 073D 80000070
  .word  0x0000001B //1CF8 073E 001B
  .word  0x40100C74 //1CFC 073F 40100C74
  .word  0x0000005B //1D00 0740 005B
  .word  0x40100C74 //1D04 0741 40100C74
  .word  0x00000033 //1D08 0742 0033
  .word  0x40100C74 //1D0C 0743 40100C74
  .word  0x00000036 //1D10 0744 0036
  .word  0x40100C74 //1D14 0745 40100C74
  .word  0x0000006D //1D18 0746 006D
  .word  0x40100C74 //1D1C 0747 40100C74
  .word  0x40101B18 //1D20 0748 40101B18
  .word  0x00002F10 //1D24 0749 2F10
  .word  0xA000000A //1D28 074A A000000A
  .word  0xA000000D //1D2C 074B A000000D
  .word  0x9000000C //1D30 074C 9000000C
  .word  0x0000332A //1D34 074D 332A
  .word  0x00000002 //1D38 074E 0002
  .word  0x401007D4 //1D3C 074F 401007D4
  .word  0x0000001B //1D40 0750 001B
  .word  0x40100C74 //1D44 0751 40100C74
  .word  0x0000005B //1D48 0752 005B
  .word  0x40100C74 //1D4C 0753 40100C74
  .word  0x00000033 //1D50 0754 0033
  .word  0x40100C74 //1D54 0755 40100C74
  .word  0x00000039 //1D58 0756 0039
  .word  0x40100C74 //1D5C 0757 40100C74
  .word  0x0000006D //1D60 0758 006D
  .word  0x40100C74 //1D64 0759 40100C74
  .word  0x8FFFFF20 //1D68 075A 8FFFFF20
  .word  0xA0000003 //1D6C 075B A0000003
  .word  0xFFFFFECC //1D70 075C -134
  .word  0x0000332D //1D74 075D 332D
  .word  0x00000005 //1D78 075E 0005
  .word  0x40101E04 //1D7C 075F 40101E04
  .word  0x00003333 //1D80 0760 3333
  .word  0x0000000B //1D84 0761 000B
  .word  0x401007D4 //1D88 0762 401007D4
  .word  0x40100DB4 //1D8C 0763 40100DB4
  .word  0x40100DB4 //1D90 0764 40100DB4
  .word  0x40101C4C //1D94 0765 40101C4C
  .word  0xA0000003 //1D98 0766 A0000003
  .word  0xFFFFFFD4 //1D9C 0767 -2C
  .word  0x0000333F //1DA0 0768 333F
  .word  0x00000006 //1DA4 0769 0006
  .word  0x40101E04 //1DA8 076A 40101E04
  .word  0x00000000 //1DAC 076B 0000
  .word  0x00002F01 //1DB0 076C 2F01
  .word  0xA0000009 //1DB4 076D A0000009
  .word  0xA0000003 //1DB8 076E A0000003
  .word  0xFFFFFFE0 //1DBC 076F -20
  .word  0x00003346 //1DC0 0770 3346
  .word  0x0000000C //1DC4 0771 000C
  .word  0x40101E04 //1DC8 0772 40101E04
  .word  0x40100B3C //1DCC 0773 40100B3C
  .word  0x40100B88 //1DD0 0774 40100B88
  .word  0xA0000003 //1DD4 0775 A0000003
  .word  0xFFFFFFE4 //1DD8 0776 -1C
  .word  0x00003353 //1DDC 0777 3353
  .word  0x0000000A //1DE0 0778 000A
  .word  0x40101E04 //1DE4 0779 40101E04
  .word  0x40100B3C //1DE8 077A 40100B3C
  .word  0x40101A9C //1DEC 077B 40101A9C
  .word  0xA0000003 //1DF0 077C A0000003
  .word  0xFFFFFFE4 //1DF4 077D -1C
  .word  0x0000335E //1DF8 077E 335E
  .word  0x00000003 //1DFC 077F 0003
  .word  0x40101E04 //1E00 0780 40101E04
  .word  0x40100B3C //1E04 0781 40100B3C
  .word  0x00002F10 //1E08 0782 2F10
  .word  0xA000000A //1E0C 0783 A000000A
  .word  0x90000008 //1E10 0784 90000008
  .word  0x40101A9C //1E14 0785 40101A9C
  .word  0x80000004 //1E18 0786 80000004
  .word  0x40100B88 //1E1C 0787 40100B88
  .word  0xA0000003 //1E20 0788 A0000003
  .word  0xFFFFFFD0 //1E24 0789 -30
  .word  0x00003362 //1E28 078A 3362
  .word  0x0000000A //1E2C 078B 000A
  .word  0x40101E04 //1E30 078C 40101E04
  .word  0x40101AC4 //1E34 078D 40101AC4
  .word  0x00000001 //1E38 078E 0001
  .word  0x00002F10 //1E3C 078F 2F10
  .word  0xA0000009 //1E40 0790 A0000009
  .word  0x40101DC8 //1E44 0791 40101DC8
  .word  0xA0000003 //1E48 0792 A0000003
  .word  0xFFFFFFD8 //1E4C 0793 -28
  .word  0x0000336D //1E50 0794 336D
  .word  0x00000008 //1E54 0795 0008
  .word  0x40101E04 //1E58 0796 40101E04
  .word  0x40101AC4 //1E5C 0797 40101AC4
  .word  0x00000001 //1E60 0798 0001
  .word  0x00002F10 //1E64 0799 2F10
  .word  0xA0000009 //1E68 079A A0000009
  .word  0x40101DE4 //1E6C 079B 40101DE4
  .word  0xA0000003 //1E70 079C A0000003
  .word  0xFFFFFFD8 //1E74 079D -28
  .word  0x00003376 //1E78 079E 3376
  .word  0x00000001 //1E7C 079F 0001
  .word  0x40101E04 //1E80 07A0 40101E04
  .word  0x40101AC4 //1E84 07A1 40101AC4
  .word  0x00000001 //1E88 07A2 0001
  .word  0x00002F10 //1E8C 07A3 2F10
  .word  0xA0000009 //1E90 07A4 A0000009
  .word  0x40101E00 //1E94 07A5 40101E00
  .word  0xA0000003 //1E98 07A6 A0000003
  .word  0xFFFFFFD8 //1E9C 07A7 -28
  .word  0x00003378 //1EA0 07A8 3378
  .word  0x00000001 //1EA4 07A9 0001
  .word  0x40101DCC //1EA8 07AA 40101DCC
  .word  0x00000000 //1EAC 07AB 0000
  .word  0x00002F10 //1EB0 07AC 2F10
  .word  0xA0000009 //1EB4 07AD A0000009
  .word  0x40100FB8 //1EB8 07AE 40100FB8
  .word  0x40100438 //1EBC 07AF 40100438
  .word  0x40100C00 //1EC0 07B0 40100C00
  .word  0x40101DAC //1EC4 07B1 40101DAC
  .word  0xA0000003 //1EC8 07B2 A0000003
  .word  0xFFFFFFD0 //1ECC 07B3 -30
  .word  0x0000337A //1ED0 07B4 337A
  .word  0x00000003 //1ED4 07B5 0003
  .word  0x40101E04 //1ED8 07B6 40101E04
  .word  0x00002F16 //1EDC 07B7 2F16
  .word  0xA000000A //1EE0 07B8 A000000A
  .word  0x90000014 //1EE4 07B9 90000014
  .word  0x40100C90 //1EE8 07BA 40100C90
  .word  0xB0000300 //1EEC 07BB B0000300
  .word  0x40100C90 //1EF0 07BC 40100C90
  .word  0xB0000300 //1EF4 07BD B0000300
  .word  0x80000018 //1EF8 07BE 80000018
  .word  0x40100C90 //1EFC 07BF 40100C90
  .word  0x40100CB8 //1F00 07C0 40100CB8
  .word  0x40100C74 //1F04 07C1 40100C74
  .word  0x40100C90 //1F08 07C2 40100C90
  .word  0x40100CB8 //1F0C 07C3 40100CB8
  .word  0x40100C74 //1F10 07C4 40100C74
  .word  0x40100C90 //1F14 07C5 40100C90
  .word  0x40100CB8 //1F18 07C6 40100CB8
  .word  0x40100C74 //1F1C 07C7 40100C74
  .word  0x40100C90 //1F20 07C8 40100C90
  .word  0x40100CB8 //1F24 07C9 40100CB8
  .word  0x40100C74 //1F28 07CA 40100C74
  .word  0xB0000300 //1F2C 07CB B0000300
  .word  0xA0000003 //1F30 07CC A0000003
  .word  0xFFFFFF98 //1F34 07CD -68
  .word  0x0000337E //1F38 07CE 337E
  .word  0x00000003 //1F3C 07CF 0003
  .word  0x40101E04 //1F40 07D0 40101E04
  .word  0x00003382 //1F44 07D1 3382
  .word  0x00000001 //1F48 07D2 0001
  .word  0x401007D4 //1F4C 07D3 401007D4
  .word  0x00000022 //1F50 07D4 0022
  .word  0x40100C74 //1F54 07D5 40100C74
  .word  0x40101EDC //1F58 07D6 40101EDC
  .word  0x00000022 //1F5C 07D7 0022
  .word  0x40100C74 //1F60 07D8 40100C74
  .word  0x00003384 //1F64 07D9 3384
  .word  0x00000001 //1F68 07DA 0001
  .word  0x401007D4 //1F6C 07DB 401007D4
  .word  0xA0000003 //1F70 07DC A0000003
  .word  0xFFFFFFC0 //1F74 07DD -40
  .word  0x00003386 //1F78 07DE 3386
  .word  0x00000005 //1F7C 07DF 0005
  .word  0x40101E04 //1F80 07E0 40101E04
  .word  0x00002F16 //1F84 07E1 2F16
  .word  0xA0000009 //1F88 07E2 A0000009
  .word  0x00002F00 //1F8C 07E3 2F00
  .word  0xA000000A //1F90 07E4 A000000A
  .word  0x40100B88 //1F94 07E5 40100B88
  .word  0x00000000 //1F98 07E6 0000
  .word  0x00002F00 //1F9C 07E7 2F00
  .word  0xA0000009 //1FA0 07E8 A0000009
  .word  0x0000338C //1FA4 07E9 338C
  .word  0x00000008 //1FA8 07EA 0008
  .word  0x401007BC //1FAC 07EB 401007BC
  .word  0x40101B18 //1FB0 07EC 40101B18
  .word  0x00000004 //1FB4 07ED 0004
  .word  0x00000000 //1FB8 07EE 0000
  .word  0x4010011C //1FBC 07EF 4010011C
  .word  0xA000000E //1FC0 07F0 A000000E
  .word  0x00000010 //1FC4 07F1 0010
  .word  0xA0000009 //1FC8 07F2 A0000009
  .word  0x40100DB4 //1FCC 07F3 40100DB4
  .word  0x0000003C //1FD0 07F4 003C
  .word  0x40100C74 //1FD4 07F5 40100C74
  .word  0x00003395 //1FD8 07F6 3395
  .word  0x00000006 //1FDC 07F7 0006
  .word  0x401007D4 //1FE0 07F8 401007D4
  .word  0x40100DB4 //1FE4 07F9 40100DB4
  .word  0x0000339C //1FE8 07FA 339C
  .word  0x00000002 //1FEC 07FB 0002
  .word  0x401007D4 //1FF0 07FC 401007D4
  .word  0x00000000 //1FF4 07FD 0000
  .word  0xB0000603 //1FF8 07FE B0000603
  .word  0xA0000007 //1FFC 07FF A0000007
  .word  0xB0000501 //2000 0800 B0000501
  .word  0x00002F03 //2004 0801 2F03
  .word  0x40100A6C //2008 0802 40100A6C
  .word  0x90000008 //200C 0803 90000008
  .word  0xB0000300 //2010 0804 B0000300
  .word  0x00002F04 //2014 0805 2F04
  .word  0xB0000501 //2018 0806 B0000501
  .word  0x00002F17 //201C 0807 2F17
  .word  0x40100A6C //2020 0808 40100A6C
  .word  0x90000014 //2024 0809 90000014
  .word  0xB0000300 //2028 080A B0000300
  .word  0x00002D00 //202C 080B 2D00
  .word  0x00002F80 //2030 080C 2F80
  .word  0xA0000009 //2034 080D A0000009
  .word  0x00002F80 //2038 080E 2F80
  .word  0xB0000501 //203C 080F B0000501
  .word  0x00002F05 //2040 0810 2F05
  .word  0x40100A6C //2044 0811 40100A6C
  .word  0x90000008 //2048 0812 90000008
  .word  0xB0000300 //204C 0813 B0000300
  .word  0x00002F00 //2050 0814 2F00
  .word  0xA000000A //2054 0815 A000000A
  .word  0x40101F44 //2058 0816 40101F44
  .word  0x40100A18 //205C 0817 40100A18
  .word  0xB0000501 //2060 0818 B0000501
  .word  0x00000010 //2064 0819 0010
  .word  0x40100A6C //2068 081A 40100A6C
  .word  0x9FFFFF88 //206C 081B 9FFFFF88
  .word  0xB0000300 //2070 081C B0000300
  .word  0x0000339F //2074 081D 339F
  .word  0x00000004 //2078 081E 0004
  .word  0x401007D4 //207C 081F 401007D4
  .word  0xB0000501 //2080 0820 B0000501
  .word  0x40100D30 //2084 0821 40100D30
  .word  0x000033A4 //2088 0822 33A4
  .word  0x00000001 //208C 0823 0001
  .word  0x401007D4 //2090 0824 401007D4
  .word  0xB0000501 //2094 0825 B0000501
  .word  0x0000000F //2098 0826 000F
  .word  0xA0000007 //209C 0827 A0000007
  .word  0x40100D30 //20A0 0828 40100D30
  .word  0x00000010 //20A4 0829 0010
  .word  0xA0000007 //20A8 082A A0000007
  .word  0xB0000603 //20AC 082B B0000603
  .word  0x40100AA4 //20B0 082C 40100AA4
  .word  0xA000000B //20B4 082D A000000B
  .word  0x9FFFFF28 //20B8 082E 9FFFFF28
  .word  0xB0000200 //20BC 082F B0000200
  .word  0x40100DB4 //20C0 0830 40100DB4
  .word  0x0000003C //20C4 0831 003C
  .word  0x40100C74 //20C8 0832 40100C74
  .word  0x000033A6 //20CC 0833 33A6
  .word  0x00000007 //20D0 0834 0007
  .word  0x401007D4 //20D4 0835 401007D4
  .word  0x40100B3C //20D8 0836 40100B3C
  .word  0x00002F00 //20DC 0837 2F00
  .word  0xA0000009 //20E0 0838 A0000009
  .word  0xA0000003 //20E4 0839 A0000003
  .word  0x000033B4 //20E8 083A 33B4
  .word  0x00000008 //20EC 083B 0008
  .word  0x40101E04 //20F0 083C 40101E04
  .word  0x00002F20 //20F4 083D 2F20
  .word  0xA000000A //20F8 083E A000000A
  .word  0xB0000501 //20FC 083F B0000501



Daten:
