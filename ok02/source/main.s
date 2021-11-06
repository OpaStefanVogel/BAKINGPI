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
//92 MDOT_32 SPDOT_32
//93 SP von C000 nach 1C000, Tasten L und N von 0x4000 nach 0x44000
//94 C! C@ ALS A0000019 A000001A und B, mit C!
//95 RAM30000Start
//96 gleich mit WAAGE starten ab R11=0x100000
//97 FIQ mit extra FF
//98 R> <R R mit 4 M+ anstelle 1+ und RP0 von 3000 nach 1C000 mit CMP R1,#0X2900
//99 FIQ=40 abfragen und auswerten

//Tasten bei R9=STEPFLAG=1
//N Neustart ab 44000 //4000
//R Neuatart ab 8000
//L Laden auf 44000 //4000
//P Laden auf 8000
//M Laden benden
//S Step
//# und 0 2809 ! continue R9=STEPFLAG=0
//^B und 1 2809 ! break R9=STEPFLAG=1

//R8=STEPNR, R9=STEPFLAG, R10=RP, R11=PC, R12=SP, R13=sp, R14=lr, R15=pc

//Speicher:
//original    aktuell                     neu                      1000000
//0000-27FF   *2+10000   10000-14FFF                               1000000-...
//2800-28ff   ? ? RP PC
//2900-2EFF   *2+10000   15010-15DFF      *4+10000   1A400-1BBFF
//2F00-2F7F   *2+10000   15E00-15EFF      *4+10000   1BC00-1BDFF
//2F80-2FFF   *2+10000   15F00-15FFF      *4+10000   1BE00-1BFFF
//3000-3FFF   *1+13000   16000-16FFF
//FF00 ob FIQ=40 vorliegt
//10004 Anzahl der empfangenen Zeichen
//10008 ob mit FIQ=UART empfangen
//1000C Anzahl der von KX abgeholten Zeichen
//100000 Start FF_RPIB
//100008 Start FIQ=40 auswerten
//2C0000 SP RP FIQ //97
//10000000... empfangene Zeichen

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
mov r12,#0x1C000 //93 1C000 //27 C000 wird auch nochmal in //32 verwendet

//MOV   R0,#0X20 
//STMEA R12!,{R0}// ( ' ' )
//BL    EMIT     // ( )

loop2$:  //2
//BL loop2a$ //27
//BL loop2a$ //27
//BL FFStart //26
//BL RAM2F00Start //37
//BL RAM3000Start //27 eb0004bd
//BL RAM3C00Start //65//43 52415453
BL RQ0000Start //68
BL RAM2F000Start //94
BL RAM30000Start //95
BL RAM100000Start //81
MOV R10,#0        //99
MOV R11,#0XFF00
STR R10,[R11],#4
STR R10,[R11],#4
MOV R11,#0X100000 //96//66 #0X10000//29 R11=PC, R12=SP
MOV R10,#0X1C000 //93 1C000 //30//45 R10=RP, R11=PC, R12=SP
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
CMP   R6,#0X46       //96 if "F"...
MOVEQ R11,#0X100000  //96 R11=0x100000
CMP   R6,#0X4E       //17 if "N"...
MOVEQ PC,#0X44000    //93 44000 //17 bei "N" Neustart //27 jetzt ab 0XE000 //73 4000
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
MOVEQ R2,#0X44000   //93 44000 //22//13//18 direkt Programmstart überschreiben //27//73 jetzt ab 0X4000
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

MDOT_32: //29 ( w -->  <hex>)
STMFD SP!,{R0-R3,LR}
LDMEA R12!,{R0}
LSR   R1,R0,#0X1C
STMEA R12!,{R1}
BL    DIGIT
BL    EMIT
LSR   R1,R0,#0X18
STMEA R12!,{R1}
BL    DIGIT
BL    EMIT
LSR   R1,R0,#0X14
STMEA R12!,{R1}
BL    DIGIT
BL    EMIT
LSR   R1,R0,#0X10
STMEA R12!,{R1}
BL    DIGIT
BL    EMIT
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
MOV   R1,#0X1C000 //SP0
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

SPDOT_32: //92 ( -->  <hex> <hex> <hex>...)
STMFD SP!,{R0-R3,LR}
MOV   R2,R12
MOV   R1,#0X1C000 //SP0
SPDOT1_32:
CMP   R1,R2
BEQ   SPDOT9_32
LDR   R0,[R1]
STMEA R12!,{R0}
BL    MDOT_32
ADD   R1,R1,#4
B     SPDOT1_32
SPDOT9_32:
LDMFD SP!,{R0-R3,PC}


//----------------------------------------------
STEP: //29 ( --> )
STMFD SP!,{R0-R7,LR}
STEPR:
CMP   R11,#0X100000
B     STEP_32



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

STEPEND0:
CMP   R9,#1    //56 vorher CMP   R8,R9    //42
BLT   STEPR    //42
MOV   R0,#0
MOV   R1,#0X20000000
ADD   R1,R1,#0XB200
ADD   R1,R1,#0X0C    // CONSTANT FIQ_KANAL
STR   R0,[R1]        // FIQ=0

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
CMP   R11,#0X100000  //92
BGE   STEPDOT_32
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
STEPDOT_32:
BL    SPDOT_32
BL    CR
STMEA R12!,{R8}// ( nr )
BL    MDOT_32
STMEA R12!,{R10}//58 ( RP )
BL    MDOT
LDR   R0,[R10]
STMEA R12!,{R0} //66 ( [RP] )
BL    MDOT_32
STMEA R12!,{R11}// ( PC )
BL    MDOT_32
LSR   R0,R11,#2
STMEA R12!,{R0}// ( PC/4 )
BL    MDOT
LDR   R0,[R11]
STMEA R12!,{R0}// ( [PC] )
BL    MDOT_32
LDMFD SP!,{R0-R7,PC}


STEP_32_FIQ:
MOV R15,R14
STMFD R13!,{R14}  //97
LDMFD R13!,{PC}


RQ0000Decode: //70//43 zu BL RQ0000Start
STMFD SP!,{R0-R3,LR}
ADD   R0,LR,#4 //28 
STMEA R12!,{R0}//28 ( rq0000 )
MOV   R0,#0X00
STMEA R12!,{R0}//28 ( rq0000 0 )
MOV   R0,#0X200 //97 //70
STMEA R12!,{R0}//28 ( rq0000 0 200 )
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
//2000B20C CONSTANT FIQ_KANAL
MOV   R8,#0XFF00
LDR   R9,[R8]
ADD   R9,R9,#1
STR   R9,[R8]
MOV   R8,#0X20000000 //PBASE
ADD   R8,R8, #0XB200 //interrupt register
ADD   R8,R8,   #0X0C //FIQ control register
LDR   R9,[R8]
AND   R9,R9,#0X7F
CMP   R9,#0X40
BEQ   FIQ_40
CMP   R9,#0X1D
BEQ   FIQ_1D
SUBS  PC,R14,#4

//MOV   R9,#0
//STR   R9,[R8]
//MOV   R8,#0X10000
//ADD   R8,R8,#0X10
//LDR   R9,[R8]
//ADD   R9,R9,#1
//STR   R9,[R8]

//B hupferstmalnur extra FF starten geht nicht
MOV   R9,#0            //97
MOV   R10,#0X2C0000    //97 RP FIQ
MOV   R12,#0X2C0000    //97 SP FIQ
MOV   R11,#0X100000
MOV   R13,#0X1F0000
ADD   R11,R11,#4           //97 PC FIQ
STR   R9,[R10,#-4]!
STMFD R13!,{R0-R12,R14}  //97
//BL    STEP_32_FIQ
LDMFD R13!,{R0-R12,R14}

FIQ_40: //ARM timer
MOV   R8,   #0X20000000  //75 PBASE
ADD   R8,R8,#0X0000B400  //75 ARM timer
MOV   R9,#0
STR   R9,[R8,#0x0C]    //75 FIQ=ARM timer clear interrupt flag
SUBS  PC,R14,#4

FIQ_1D: //
MOV   R8,#0X20000000     //70 PBASE
ADD   R8,R8,#0X210000    //70 GPIO_
ADD   R8,R8,#0X5000     //70 AUX_IRQ
LDR   R10,[R8,#0x40]     //70 AUX_MU_IO_REG, liest char und löscht Int.
MOV   R8,#0X10000
ADD   R8,R8,#4
LDR   R9,[R8]
ADD   R9,R9,#0X10000000 //65//62
STRB  R10,[R9]
ADD   R9,R9,#1
SUB   R9,R9,#0X10000000 //65//62
STR   R9,[R8]

MOV   R8,#0X20000000 //74 GPIO Blinklicht
ADD   R8,R8,#0X200000
MOV   R11,#4 //Bitposition für GPIO2
AND   R10,R9,#1
CMP   R10,#1
STREQ R11,[R8,#40]
STRNE R11,[R8,#28]

SUBS  PC,R14,#4

//soweit FIQ





STEP_32: //
MOV   R1,#0xFF00          //99 FIQ=40 abfragen
LDR   R0,[R1]
//MOV   R0,#0
CMP   R0,#0
SUBNE R0,R0,#1
STRNE R0,[R1]
MOVNE R0,   #0X40000000
ADDNE R0,R0,#0X00100000
ADDNE R0,R0,#0X00000008  //99 FIQ=40 auswerten
LDREQ R0,[R11]
ADDEQ R11,R11,#4
ADD   R8,R8,#1
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

STEP0123_32:
STEPF_32:
STMEA R12!,{R0} //32 ( --> n )
//MOV   R9,#1
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
B     STEPA019_32
B     STEPA01A_32
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
STMEA R12!,{R1}    // ( --> (a*b+c)h )
STMEA R12!,{R0}    // ( --> (a*b+c)h (a*b+c)l )
//MOV   R9,#1
B     STEPEND

STEPA003_32:
//      case 0x03: /* RETURN */ PC=RAMB0000[RP]; RP=RP+1; break;
//( 0A 003 MLIT MCODE RETURN )
LDR   R2,[R10],#4 //44
CMP   R2,#0X100000
MOVGE R11,R2
BGE   STEPEND
CMP   R2,#0           //97
LDMEQFD SP!,{R0-R7,PC}  //97
LSL   R11,R2,#1
ADD   R11,R11,#0X10000 //66
MOV   R9,#0
B     STEPEND

STEPA004_32: //90
//      case 0x04: /*  alert("case 0x04: "+PC.toHex0000()+" "+PD.toHex0000()+" "+RAMB0000[0x2F03]+" "+RAMB0000[0x2F04]+" "+STEP+" "+ENDSTEP); */ if (RAMB0000[0x2F03]==RAMB0000[0x2F04]) STEP=ENDSTEP; break;
//MOV   R9,#1
B STEPA004
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

STEPA005_32: //87
//      case 0x05: /* EMITCODE */  EText=EText+String.fromCharCode(STAPEL[ST-1]); Logtext=Logtext+String.fromCharCode(STAPEL[ST-1]); ST=ST-1; break;
//0A 005 MLIT MCODE EMITCODE
//MOV   R9,#1
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

STEPA009_32: //98 //88
//0A 009 MLIT MCODE !
LDMEA R12!,{R0,R1} //34 ( n adr --> )
CMP   R1,#0X2900 //84
BGE   STORE4000_32
CMP   R1,#0X2800
BGE   STORE2_32
STORE4000_32:
STR   R0,[R1]
B     STEPEND
STORE2_32:
AND   R1,R1,#0XFF
CMP   R1,#3
MOVEQ   R11,R0
BEQ   STEPEND
CMP   R1,#2
MOVEQ R10,R0 //98
BEQ   STEPEND
CMP   R1,#1
LSLEQ R12,R0,#2 //55
BEQ   STEPEND
CMP   R1,#0
BEQ   STEPEND
CMP   R1,#9
MOV   R9,R0
B     STEPEND

STEPA00A_32: //98 //88
//0A 00A MLIT MCODE @
LDMEA R12!,{R1} //34 ( adr --> )
CMP   R1,#0X2900 //84
BGE   FETCH4000_32
CMP   R1,#0X2800
BGE   FETCH2_32
FETCH4000_32:
LDR   R3,[R1]
STMEA R12!,{R3} // ( --> n )
B     STEPEND
FETCH2_32: //44
AND   R1,R1,#0XFF
CMP   R1,#3
STMEQEA R12!,{R11} //44 ( --> n )
BEQ   STEPEND
CMP   R1,#2
MOVEQ   R0,R10 //98
STMEQEA R12!,{R0} //44 ( --> n )
BEQ   STEPEND
CMP   R1,#1
LSREQ R0,R12,#2
STMEQEA R12!,{R0} //55 ( --> n )
BEQ   STEPEND
CMP   R1,#0       //51
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

STEPA00B_32: //87
//      case 0x0B: /* NOT */ STAPEL[ST-1]=-1-STAPEL[ST-1]; break;
//0A 00B MLIT MCODE NOT
LDMEA R12!,{R0} // ( a --> )
MVN   R0,R0
STMEA R12!,{R0} // ( --> not(a) )
B     STEPEND

STEPA00C_32:
//0A 00C MLIT MCODE FIQ
LDMEA R12!,{R1} // ( bic --> )
    mrs r0,cpsr       //69 Enable FIQ https://github.com/dwelch67/raspberrypi/blinker08
    bic r0,r0,R1      //72//69
    msr cpsr_c,r0     //69
STMEA R12!,{R0} // ( --> cpsr_neu )
B     STEPEND

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

STEPA019_32:
//0A 019 MLIT MCODE C!
LDMEA R12!,{R0,R1} //34 ( n adr --> )
CMP   R1,#0X4000 //84
BGE   STORE4000_C32
CMP   R1,#0X3000 //43
ADDGE R1,#0X13000 //65
STRGEB  R0,[R1]
BGE     STEPEND
CMP   R1,#0X2900 //82
BGE   STORE2F00_C32
CMP   R1,#0X2800
BGE   STORE2_C32
ADD   R1,R1,R1
ADD   R1,R1,#0X10000 //66
STRH  R0,[R1]
B     STEPEND
STORE4000_C32:
STRB  R0,[R1]
B     STEPEND
STORE2F00_C32:
ADD   R1,R1,R1
ADD   R1,R1,R1
ADD   R1,R1,#0X10000
STR   R0,[R1]
B     STEPEND
STORE2_C32:
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

STEPA01A_32:
//0A 01A MLIT MCODE C@
//MOV R9,#1
LDMEA R12!,{R1} //34 ( adr --> )
CMP   R1,#0X4000 //84
BGE   FETCH4000_C32
LSL   R1,R1,#16 //48
LSR   R1,R1,#16 //48
CMP   R1,#0X3000
ADDGE R1,#0X13000  //65
LDRGEB  R3,[R1]
STMGEEA R12!,{R3} //34 ( --> n )
BGE     STEPEND
CMP   R1,#0X2900
BGE   FETCH2F00_C32
CMP   R1,#0X2800
BGE   FETCH2_C32
ADD   R1,R1,R1
ADD   R1,R1,#0X10000 //66
LDRH  R3,[R1]
AND   R1,R3,#0X8000  //52 Vorzeichenbits
CMP   R1,#0
ADDNE R3,#0XFF0000
ADDNE R3,#0XFF000000
STMEA R12!,{R3} //34 ( --> n )
B     STEPEND
FETCH4000_C32:
LDRB  R3,[R1,#0]
STMEA R12!,{R3} // ( --> n )
B     STEPEND
FETCH2F00_C32:
ADD   R1,R1,R1
ADD   R1,R1,R1
ADD   R1,R1,#0X10000 //66
LDR   R3,[R1]
STMEA R12!,{R3} //34 ( --> n )
B     STEPEND
FETCH2_C32: //44
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

//Testcode für ab 0x100000:
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
  .word 0xA0000009 //48 !
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

RAM2F000Decode: //37 zu BL RAM2F00Start
STMFD SP!,{R0-R3,LR}
ADD   R0,LR,#4 //37 RAMB2F00 nach 2F00 verschieben
STMEA R12!,{R0}//37 ( ramb2F000 )
MOV   R0,#0X2F000 //66
ADD   R0,R0,#0X00
STMEA R12!,{R0}//37 ( ramb2F000 2F000 )
MOV   R0,#0X400
STMEA R12!,{R0}//37 ( ramb2F000 2F000 400 )
BL    MOVE     //37 ( )
LDMFD SP!,{R0-R3,PC}
RAM2F000Start:
STMFD SP!,{R0-R7,LR}
BL RAM2F000Decode
LDMFD SP!,{R0-R7,PC}
RAM2F000:
//.include "RAM2F000.s"
  .word 0x00000000 //0002F000 
  .word 0x00000000 //0002F004 
  .word 0x0001C000 //0002F008 
  .word 0x0003C000 //0002F00C 
  .word 0x0003C000 //0002F010 
  .word 0x00000000 //0002F014 
  .word 0x00030071 //0002F018 
  .word 0x001002CC //0002F01C 
  .word 0x00000010 //0002F020 
  .word 0x0003B000 //0002F024 
  .word 0x0003B000 //0002F028 
  .word 0x0003B01B //0002F02C 
  .word 0x0003B021 //0002F030 
  .word 0x0003B020 //0002F034 
  .word 0x00000000 //0002F038 
  .word 0x00101F70 //0002F03C 
  .word 0x00000000 //0002F040 
  .word 0x00101EFC //0002F044 
  .word 0x00030000 //0002F048 
  .word 0x00030391 //0002F04C 
  .word 0x00100080 //0002F050 
  .word 0x00007001 //0002F054 
  .word 0x00000000 //0002F058 
  .word 0x0002D000 //0002F05C 

 
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

RAM30000Decode: //95 zu BL RAM30000Start
STMFD SP!,{R0-R3,LR}
ADD   R0,LR,#4 //37 RAMB30000 nach 30000 verschieben
STMEA R12!,{R0}//37 ( ramb30000 )
MOV   R0,#0X30000 //66
ADD   R0,R0,#0X00
STMEA R12!,{R0}//37 ( ramb30000 30000 )
MOV   R0,#0X800
STMEA R12!,{R0}//37 ( ramb30000 30000 800 )
BL    MOVE     //37 ( )
LDMFD SP!,{R0-R3,PC}
RAM30000Start:
STMFD SP!,{R0-R7,LR}
BL RAM30000Decode
LDMFD SP!,{R0-R7,PC}
RAM30000:
//.include "RAM30000.s"
  .word 0x207B2028 //00030000 
  .word 0x4F43207D //00030004 
  .word 0x4C49504D //00030008 
  .word 0x4C4D2045 //0003000C 
  .word 0x41205449 //00030010 
  .word 0x4C205342 //00030014 
  .word 0x202C5449 //00030018 
  .word 0x4E4F4328 //0003001C 
  .word 0x4E415453 //00030020 
  .word 0x20293A54 //00030024 
  .word 0x534E4F43 //00030028 
  .word 0x544E4154 //0003002C 
  .word 0x59454B20 //00030030 
  .word 0x20524441 //00030034 
  .word 0x52205053 //00030038 
  .word 0x43502050 //0003003C 
  .word 0x49425820 //00030040 
  .word 0x4D532054 //00030044 
  .word 0x45474455 //00030048 
  .word 0x20544942 //0003004C 
  .word 0x20305052 //00030050 
  .word 0x4D415249 //00030054 
  .word 0x20524441 //00030058 
  .word 0x4D41524A //0003005C 
  .word 0x20524441 //00030060 
  .word 0x46464F58 //00030064 
  .word 0x42524320 //00030068 
  .word 0x4749455A //0003006C 
  .word 0x44524320 //00030070 
  .word 0x41422050 //00030074 
  .word 0x54204553 //00030078 
  .word 0x49204249 //0003007C 
  .word 0x4920314E //00030080 
  .word 0x4920324E //00030084 
  .word 0x4920334E //00030088 
  .word 0x4520344E //0003008C 
  .word 0x524F5252 //00030090 
  .word 0x4420524E //00030094 
  .word 0x54532050 //00030098 
  .word 0x4C205441 //0003009C 
  .word 0x42204146 //000300A0 
  .word 0x20464E41 //000300A4 
  .word 0x49455A42 //000300A8 
  .word 0x50442047 //000300AC 
  .word 0x4B52454D //000300B0 
  .word 0x50534320 //000300B4 
  .word 0x42554420 //000300B8 
  .word 0x4C205449 //000300BC 
  .word 0x4C41434F //000300C0 
  .word 0x45524441 //000300C4 
  .word 0x20455353 //000300C8 
  .word 0x53524556 //000300CC 
  .word 0x204E4F49 //000300D0 
  .word 0x55544552 //000300D4 
  .word 0x28204E52 //000300D8 
  .word 0x444F434D //000300DC 
  .word 0x20293A45 //000300E0 
  .word 0x444F434D //000300E4 
  .word 0x494D2045 //000300E8 
  .word 0x2053554E //000300EC 
  .word 0x55202B55 //000300F0 
  .word 0x3D30202A //000300F4 
  .word 0x544C3020 //000300F8 
  .word 0x494D4520 //000300FC 
  .word 0x444F4354 //00030100 
  .word 0x4F4E2045 //00030104 
  .word 0x4E412054 //00030108 
  .word 0x524F2044 //0003010C 
  .word 0x202B4D20 //00030110 
  .word 0x20402021 //00030114 
  .word 0x4320584B //00030118 
  .word 0x40432021 //0003011C 
  .word 0x41575320 //00030120 
  .word 0x564F2050 //00030124 
  .word 0x44205245 //00030128 
  .word 0x52205055 //0003012C 
  .word 0x4420544F //00030130 
  .word 0x20504F52 //00030134 
  .word 0x41575332 //00030138 
  .word 0x4F322050 //0003013C 
  .word 0x20524556 //00030140 
  .word 0x50554432 //00030144 
  .word 0x52443220 //00030148 
  .word 0x4E20504F //0003014C 
  .word 0x20504F4F //00030150 
  .word 0x5A202C42 //00030154 
  .word 0x5728202C //00030158 
  .word 0x3A44524F //0003015C 
  .word 0x4F572029 //00030160 
  .word 0x203A4452 //00030164 
  .word 0x222E2022 //00030168 
  .word 0x52454820 //0003016C 
  .word 0x524A2045 //00030170 
  .word 0x20544942 //00030174 
  .word 0x4230524A //00030178 
  .word 0x58205449 //0003017C 
  .word 0x42544553 //00030180 
  .word 0x4C412054 //00030184 
  .word 0x20544F4C //00030188 
  .word 0x4E415242 //0003018C 
  .word 0x202C4843 //00030190 
  .word 0x41524230 //00030194 
  .word 0x2C48434E //00030198 
  .word 0x47454220 //0003019C 
  .word 0x41204E49 //000301A0 
  .word 0x4E494147 //000301A4 
  .word 0x544E5520 //000301A8 
  .word 0x49204C49 //000301AC 
  .word 0x4E452046 //000301B0 
  .word 0x46495F44 //000301B4 
  .word 0x534C4520 //000301B8 
  .word 0x48572045 //000301BC 
  .word 0x20454C49 //000301C0 
  .word 0x45504552 //000301C4 
  .word 0x31205441 //000301C8 
  .word 0x2D31202B //000301CC 
  .word 0x202D4D20 //000301D0 
  .word 0x544C203D //000301D4 
  .word 0x4D203E20 //000301D8 
  .word 0x5942202A //000301DC 
  .word 0x59422045 //000301E0 
  .word 0x2B202045 //000301E4 
  .word 0x3E522021 //000301E8 
  .word 0x20523E20 //000301EC 
  .word 0x202C2052 //000301F0 
  .word 0x43455845 //000301F4 
  .word 0x20455455 //000301F8 
  .word 0x2059454B //000301FC 
  .word 0x54494D45 //00030200 
  .word 0x4C485320 //00030204 
  .word 0x44203631 //00030208 
  .word 0x54204749 //0003020C 
  .word 0x20455059 //00030210 
  .word 0x202E4748 //00030214 
  .word 0x4D202E4D //00030218 
  .word 0x5243203F //0003021C 
  .word 0x3E6C6620 //00030220 
  .word 0x6C662F20 //00030224 
  .word 0x7266203E //00030228 
  .word 0x662F203E //0003022C 
  .word 0x46203E72 //00030230 
  .word 0x454C4845 //00030234 
  .word 0x58455452 //00030238 
  .word 0x49442054 //0003023C 
  .word 0x4C424153 //00030240 
  .word 0x65772045 //00030244 
  .word 0x72657469 //00030248 
  .word 0x63616E20 //0003024C 
  .word 0x61542068 //00030250 
  .word 0x20657473 //00030254 
  .word 0x41435345 //00030258 
  .word 0x20204550 //0003025C 
  .word 0x4F525245 //00030260 
  .word 0x3F3F2052 //00030264 
  .word 0x4546203F //00030268 
  .word 0x52454C48 //0003026C 
  .word 0x54584554 //00030270 
  .word 0x52524520 //00030274 
  .word 0x2D20524F //00030278 
  .word 0x68654620 //0003027C 
  .word 0x2072656C //00030280 
  .word 0x6D6D754E //00030284 
  .word 0x20207265 //00030288 
  .word 0x21505343 //0003028C 
  .word 0x50534320 //00030290 
  .word 0x4F4C203F //00030294 
  .word 0x204C4143 //00030298 
  .word 0x5F444E45 //0003029C 
  .word 0x41434F4C //000302A0 
  .word 0x304C204C //000302A4 
  .word 0x20314C20 //000302A8 
  .word 0x4C20324C //000302AC 
  .word 0x344C2033 //000302B0 
  .word 0x20354C20 //000302B4 
  .word 0x4C20364C //000302B8 
  .word 0x20272037 //000302BC 
  .word 0x52434E49 //000302C0 
  .word 0x454B2034 //000302C4 
  .word 0x4E495F59 //000302C8 
  .word 0x454B2054 //000302CC 
  .word 0x444F4359 //000302D0 
  .word 0x45203245 //000302D4 
  .word 0x43455058 //000302D8 
  .word 0x49442054 //000302DC 
  .word 0x20544947 //000302E0 
  .word 0x424D554E //000302E4 
  .word 0x57205245 //000302E8 
  .word 0x2044524F //000302EC 
  .word 0x46203D5A //000302F0 
  .word 0x20444E49 //000302F4 
  .word 0x4146434C //000302F8 
  .word 0x4D4F4320 //000302FC 
  .word 0x454C4950 //00030300 
  .word 0x5243202C //00030304 
  .word 0x45544145 //00030308 
  .word 0x544E4920 //0003030C 
  .word 0x52505245 //00030310 
  .word 0x51205445 //00030314 
  .word 0x20544955 //00030318 
  .word 0x3E6B6F2F //0003031C 
  .word 0x206B6F20 //00030320 
  .word 0x203E6B6F //00030324 
  .word 0x3E6B6F2F //00030328 
  .word 0x206B6F20 //0003032C 
  .word 0x52415453 //00030330 
  .word 0x4F462054 //00030334 
  .word 0x2D595452 //00030338 
  .word 0x54524F46 //0003033C 
  .word 0x4D532048 //00030340 
  .word 0x45474455 //00030344 
  .word 0x4D492820 //00030348 
  .word 0x4944454D //0003034C 
  .word 0x3A455441 //00030350 
  .word 0x43282029 //00030354 
  .word 0x49504D4F //00030358 
  .word 0x293A454C //0003035C 
  .word 0x293A2820 //00030360 
  .word 0x4D4D4920 //00030364 
  .word 0x41494445 //00030368 
  .word 0x203A4554 //0003036C 
  .word 0x504D4F43 //00030370 
  .word 0x3A454C49 //00030374 
  .word 0x3B203A20 //00030378 
  .word 0x55445720 //0003037C 
  .word 0x2020504D //00030380 
  .word 0x6F772E20 //00030384 
  .word 0x30206472 //00030388 
  .word 0x2F2F2078 //0003038C 
  .word 0x2F207820 //00030390 



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


//85
//MOV R9,#1
//.include "RAM100000.s"
  .word 0x40100040 //00100000 
  .word 0xA0000003 //00100004 
  .word 0xA0000003 //00100008 //99 FIQ=40 auswerten
  .word 0xA0000003 //0010000C //99
  .word 0x00000000 //00100010 
  .word 0x00000000 //00100014 
  .word 0x00000000 //00100018 
  .word 0x00000000 //0010001C 
  .word 0x00000000 //00100020 
  .word 0x00000000 //00100024 
  .word 0x00000000 //00100028 
  .word 0x00000000 //0010002C 
  .word 0x00000000 //00100030 
  .word 0x00000000 //00100034 
  .word 0x00000000 //00100038 
  .word 0x00000000 //0010003C 
  .word 0x40101DB0 //00100040 
  .word 0xA0000003 //00100044 
  .word 0x401011F4 //00100048 
  .word 0xA0000003 //0010004C 
  .word 0x40100000 //00100050 
  .word 0xA0000003 //00100054 
  .word 0x40100000 //00100058 
  .word 0xA0000003 //0010005C 
  .word 0x40100000 //00100060 
  .word 0xA0000003 //00100064 
  .word 0x40100000 //00100068 
  .word 0xA0000003 //0010006C 
  .word 0x40100000 //00100070 
  .word 0xA0000003 //00100074 
  .word 0x00000010 //00100078 
  .word 0xA0000003 //0010007C 
  .word 0x00000000 //00100080 
  .word 0x00030000 //00100084 
  .word 0x00000001 //00100088 
  .word 0x40101DFC //0010008C 
  .word 0x00000029 //00100090 
  .word 0x401017D8 //00100094 
  .word 0xB0000200 //00100098 
  .word 0xA0000003 //0010009C 
  .word 0xFFFFFFE0 //001000A0 
  .word 0x00030002 //001000A4 
  .word 0x00000001 //001000A8 
  .word 0x40101DFC //001000AC 
  .word 0x00000000 //001000B0 
  .word 0x0002F040 //001000B4 
  .word 0xA0000009 //001000B8 
  .word 0xA0000003 //001000BC 
  .word 0xFFFFFFE0 //001000C0 
  .word 0x00030004 //001000C4 
  .word 0x00000001 //001000C8 
  .word 0x40101E34 //001000CC 
  .word 0x00000001 //001000D0 
  .word 0x0002F040 //001000D4 
  .word 0xA0000009 //001000D8 
  .word 0xA0000003 //001000DC 
  .word 0xFFFFFFE0 //001000E0 
  .word 0x00030006 //001000E4 
  .word 0x00000007 //001000E8 
  .word 0x40101DFC //001000EC 
  .word 0x00000020 //001000F0 
  .word 0x401017D8 //001000F4 
  .word 0x401019A0 //001000F8 
  .word 0x40101AA8 //001000FC 
  .word 0xB0000300 //00100100 
  .word 0x40101ACC //00100104 
  .word 0xA0000003 //00100108 
  .word 0xFFFFFFD4 //0010010C 
  .word 0x0003000E //00100110 
  .word 0x00000004 //00100114 
  .word 0x40101E34 //00100118 
  .word 0xB0000412 //0010011C 
  .word 0x10000000 //00100120 
  .word 0xA0000002 //00100124 
  .word 0xB0000412 //00100128 
  .word 0xB0000300 //0010012C 
  .word 0xA0000003 //00100130 
  .word 0xFFFFFFD8 //00100134 
  .word 0x00030013 //00100138 
  .word 0x00000003 //0010013C 
  .word 0x40101E34 //00100140 
  .word 0xB0000501 //00100144 
  .word 0xA000000F //00100148 
  .word 0x90000004 //0010014C 
  .word 0xA0000000 //00100150 
  .word 0xA0000003 //00100154 
  .word 0xFFFFFFDC //00100158 
  .word 0x00030017 //0010015C 
  .word 0x00000004 //00100160 
  .word 0x40101E34 //00100164 
  .word 0xB0000501 //00100168 
  .word 0x40100144 //0010016C 
  .word 0x0000000C //00100170 
  .word 0x00000000 //00100174 
  .word 0x4010011C //00100178 
  .word 0xA0000008 //0010017C 
  .word 0x90000008 //00100180 
  .word 0x00000111 //00100184 
  .word 0x40100F40 //00100188 
  .word 0x40100C38 //0010018C 
  .word 0xA0000003 //00100190 
  .word 0xFFFFFFC4 //00100194 
  .word 0x0003001C //00100198 
  .word 0x0000000B //0010019C 
  .word 0x40101E18 //001001A0 
  .word 0x40100B60 //001001A4 
  .word 0xA000000A //001001A8 
  .word 0x0002F040 //001001AC 
  .word 0xA000000A //001001B0 
  .word 0x90000004 //001001B4 
  .word 0x40100168 //001001B8 
  .word 0xA0000003 //001001BC 
  .word 0xFFFFFFD4 //001001C0 
  .word 0x00030028 //001001C4 
  .word 0x00000008 //001001C8 
  .word 0x40101E34 //001001CC 
  .word 0x40101AF4 //001001D0 
  .word 0x401001A0 //001001D4 
  .word 0x40100C38 //001001D8 
  .word 0x40101DDC //001001DC 
  .word 0xA0000003 //001001E0 
  .word 0xFFFFFFDC //001001E4 
  .word 0x00030031 //001001E8 
  .word 0x00000006 //001001EC 
  .word 0x401001A4 //001001F0 
  .word 0x00002800 //001001F4 
  .word 0xFFFFFFEC //001001F8 
  .word 0x00030038 //001001FC 
  .word 0x00000002 //00100200 
  .word 0x401001A4 //00100204 
  .word 0x00002801 //00100208 
  .word 0xFFFFFFEC //0010020C 
  .word 0x0003003B //00100210 
  .word 0x00000002 //00100214 
  .word 0x401001A4 //00100218 
  .word 0x00002802 //0010021C 
  .word 0xFFFFFFEC //00100220 
  .word 0x0003003E //00100224 
  .word 0x00000002 //00100228 
  .word 0x401001A4 //0010022C 
  .word 0x00002803 //00100230 
  .word 0xFFFFFFEC //00100234 
  .word 0x00030041 //00100238 
  .word 0x00000004 //0010023C 
  .word 0x401001A4 //00100240 
  .word 0x0002F000 //00100244 
  .word 0xFFFFFFEC //00100248 
  .word 0x00030046 //0010024C 
  .word 0x00000009 //00100250 
  .word 0x401001A4 //00100254 
  .word 0x0002F004 //00100258 
  .word 0xFFFFFFEC //0010025C 
  .word 0x00030050 //00100260 
  .word 0x00000003 //00100264 
  .word 0x401001A4 //00100268 
  .word 0x0002F008 //0010026C 
  .word 0xFFFFFFEC //00100270 
  .word 0x00030054 //00100274 
  .word 0x00000007 //00100278 
  .word 0x401001A4 //0010027C 
  .word 0x0002F00C //00100280 
  .word 0xFFFFFFEC //00100284 
  .word 0x0003005C //00100288 
  .word 0x00000007 //0010028C 
  .word 0x401001A4 //00100290 
  .word 0x0002F010 //00100294 
  .word 0xFFFFFFEC //00100298 
  .word 0x00030064 //0010029C 
  .word 0x00000004 //001002A0 
  .word 0x401001A4 //001002A4 
  .word 0x0002F014 //001002A8 
  .word 0xFFFFFFEC //001002AC 
  .word 0x00030069 //001002B0 
  .word 0x00000007 //001002B4 
  .word 0x401001A4 //001002B8 
  .word 0x0002F018 //001002BC 
  .word 0xFFFFFFEC //001002C0 
  .word 0x00030071 //001002C4 
  .word 0x00000004 //001002C8 
  .word 0x401001A4 //001002CC 
  .word 0x0002F01C //001002D0 
  .word 0xFFFFFFEC //001002D4 
  .word 0x00030076 //001002D8 
  .word 0x00000004 //001002DC 
  .word 0x401001A4 //001002E0 
  .word 0x0002F020 //001002E4 
  .word 0xFFFFFFEC //001002E8 
  .word 0x0003007B //001002EC 
  .word 0x00000003 //001002F0 
  .word 0x401001A4 //001002F4 
  .word 0x0002F024 //001002F8 
  .word 0xFFFFFFEC //001002FC 
  .word 0x0003007F //00100300 
  .word 0x00000003 //00100304 
  .word 0x401001A4 //00100308 
  .word 0x0002F028 //0010030C 
  .word 0xFFFFFFEC //00100310 
  .word 0x00030083 //00100314 
  .word 0x00000003 //00100318 
  .word 0x401001A4 //0010031C 
  .word 0x0002F02C //00100320 
  .word 0xFFFFFFEC //00100324 
  .word 0x00030087 //00100328 
  .word 0x00000003 //0010032C 
  .word 0x401001A4 //00100330 
  .word 0x0002F030 //00100334 
  .word 0xFFFFFFEC //00100338 
  .word 0x0003008B //0010033C 
  .word 0x00000003 //00100340 
  .word 0x401001A4 //00100344 
  .word 0x0002F034 //00100348 
  .word 0xFFFFFFEC //0010034C 
  .word 0x0003008F //00100350 
  .word 0x00000007 //00100354 
  .word 0x401001A4 //00100358 
  .word 0x0002F038 //0010035C 
  .word 0xFFFFFFEC //00100360 
  .word 0x00030097 //00100364 
  .word 0x00000002 //00100368 
  .word 0x401001A4 //0010036C 
  .word 0x0002F03C //00100370 
  .word 0xFFFFFFEC //00100374 
  .word 0x0003009A //00100378 
  .word 0x00000004 //0010037C 
  .word 0x401001A4 //00100380 
  .word 0x0002F040 //00100384 
  .word 0xFFFFFFEC //00100388 
  .word 0x0003009F //0010038C 
  .word 0x00000003 //00100390 
  .word 0x401001A4 //00100394 
  .word 0x0002F044 //00100398 
  .word 0xFFFFFFEC //0010039C 
  .word 0x000300A3 //001003A0 
  .word 0x00000004 //001003A4 
  .word 0x401001A4 //001003A8 
  .word 0x0002F048 //001003AC 
  .word 0xFFFFFFEC //001003B0 
  .word 0x000300A8 //001003B4 
  .word 0x00000005 //001003B8 
  .word 0x401001A4 //001003BC 
  .word 0x0002F04C //001003C0 
  .word 0xFFFFFFEC //001003C4 
  .word 0x000300AE //001003C8 
  .word 0x00000006 //001003CC 
  .word 0x401001A4 //001003D0 
  .word 0x0002F050 //001003D4 
  .word 0xFFFFFFEC //001003D8 
  .word 0x000300B5 //001003DC 
  .word 0x00000003 //001003E0 
  .word 0x401001A4 //001003E4 
  .word 0x0002F054 //001003E8 
  .word 0xFFFFFFEC //001003EC 
  .word 0x000300B9 //001003F0 
  .word 0x00000005 //001003F4 
  .word 0x401001A4 //001003F8 
  .word 0x0002F058 //001003FC 
  .word 0xFFFFFFEC //00100400 
  .word 0x000300BF //00100404 
  .word 0x0000000C //00100408 
  .word 0x401001A4 //0010040C 
  .word 0x0002F05C //00100410 
  .word 0xFFFFFFEC //00100414 
  .word 0x000300CC //00100418 
  .word 0x00000007 //0010041C 
  .word 0x401001A4 //00100420 
  .word 0x000001E0 //00100424 
  .word 0xFFFFFFEC //00100428 
  .word 0x000300D4 //0010042C 
  .word 0x00000006 //00100430 
  .word 0x40101E34 //00100434 
  .word 0x0000000A //00100438 
  .word 0x00000003 //0010043C 
  .word 0x4010011C //00100440 
  .word 0xA0000003 //00100444 
  .word 0xFFFFFFE0 //00100448 
  .word 0x000300DB //0010044C 
  .word 0x00000008 //00100450 
  .word 0x40101E18 //00100454 
  .word 0x40100B60 //00100458 
  .word 0x0002F040 //0010045C 
  .word 0xA000000A //00100460 
  .word 0x9000000C //00100464 
  .word 0xA000000A //00100468 
  .word 0x40100C38 //0010046C 
  .word 0x80000004 //00100470 
  .word 0x40100C64 //00100474 
  .word 0xA0000003 //00100478 
  .word 0xFFFFFFCC //0010047C 
  .word 0x000300E4 //00100480 
  .word 0x00000005 //00100484 
  .word 0x40101E34 //00100488 
  .word 0x40101AF4 //0010048C 
  .word 0x40100454 //00100490 
  .word 0x40100C38 //00100494 
  .word 0x40100438 //00100498 
  .word 0x40100C38 //0010049C 
  .word 0x40101DDC //001004A0 
  .word 0xA0000003 //001004A4 
  .word 0xFFFFFFD4 //001004A8 
  .word 0x000300EA //001004AC 
  .word 0x00000005 //001004B0 
  .word 0x40100458 //001004B4 
  .word 0xA0000000 //001004B8 
  .word 0xA0000003 //001004BC 
  .word 0xFFFFFFE8 //001004C0 
  .word 0x000300F0 //001004C4 
  .word 0x00000002 //001004C8 
  .word 0x40100458 //001004CC 
  .word 0xA0000001 //001004D0 
  .word 0xA0000003 //001004D4 
  .word 0xFFFFFFE8 //001004D8 
  .word 0x000300F3 //001004DC 
  .word 0x00000002 //001004E0 
  .word 0x40100458 //001004E4 
  .word 0xA0000002 //001004E8 
  .word 0xA0000003 //001004EC 
  .word 0xFFFFFFE8 //001004F0 
  .word 0x000300F6 //001004F4 
  .word 0x00000002 //001004F8 
  .word 0x40100458 //001004FC 
  .word 0xA000000D //00100500 
  .word 0xA0000003 //00100504 
  .word 0xFFFFFFE8 //00100508 
  .word 0x000300F9 //0010050C 
  .word 0x00000003 //00100510 
  .word 0x40100458 //00100514 
  .word 0xA000000F //00100518 
  .word 0xA0000003 //0010051C 
  .word 0xFFFFFFE8 //00100520 
  .word 0x000300FD //00100524 
  .word 0x00000008 //00100528 
  .word 0x40100458 //0010052C 
  .word 0xA0000005 //00100530 
  .word 0xA0000003 //00100534 
  .word 0xFFFFFFE8 //00100538 
  .word 0x00030106 //0010053C 
  .word 0x00000003 //00100540 
  .word 0x40100458 //00100544 
  .word 0xA000000B //00100548 
  .word 0xA0000003 //0010054C 
  .word 0xFFFFFFE8 //00100550 
  .word 0x0003010A //00100554 
  .word 0x00000003 //00100558 
  .word 0x40100458 //0010055C 
  .word 0xA0000008 //00100560 
  .word 0xA0000003 //00100564 
  .word 0xFFFFFFE8 //00100568 
  .word 0x0003010E //0010056C 
  .word 0x00000002 //00100570 
  .word 0x40100458 //00100574 
  .word 0xA000000E //00100578 
  .word 0xA0000003 //0010057C 
  .word 0xFFFFFFE8 //00100580 
  .word 0x00030111 //00100584 
  .word 0x00000002 //00100588 
  .word 0x40100458 //0010058C 
  .word 0xA0000007 //00100590 
  .word 0xA0000003 //00100594 
  .word 0xFFFFFFE8 //00100598 
  .word 0x00030114 //0010059C 
  .word 0x00000001 //001005A0 
  .word 0x40100458 //001005A4 
  .word 0xA0000009 //001005A8 
  .word 0xA0000003 //001005AC 
  .word 0xFFFFFFE8 //001005B0 
  .word 0x00030116 //001005B4 
  .word 0x00000001 //001005B8 
  .word 0x40100458 //001005BC 
  .word 0xA000000A //001005C0 
  .word 0xA0000003 //001005C4 
  .word 0xFFFFFFE8 //001005C8 
  .word 0x00030118 //001005CC 
  .word 0x00000002 //001005D0 
  .word 0x40100458 //001005D4 
  .word 0xA0000004 //001005D8 
  .word 0xA0000003 //001005DC 
  .word 0xFFFFFFE8 //001005E0 
  .word 0x0003011B //001005E4 
  .word 0x00000002 //001005E8 
  .word 0x40100458 //001005EC 
  .word 0xA0000019 //001005F0 
  .word 0xA0000003 //001005F4 
  .word 0xFFFFFFE8 //001005F8 
  .word 0x0003011E //001005FC 
  .word 0x00000002 //00100600 
  .word 0x40100458 //00100604 
  .word 0xA000001A //00100608 
  .word 0xA0000003 //0010060C 
  .word 0xFFFFFFE8 //00100610 
  .word 0x00030121 //00100614 
  .word 0x00000004 //00100618 
  .word 0x40100458 //0010061C 
  .word 0xB0000412 //00100620 
  .word 0xA0000003 //00100624 
  .word 0xFFFFFFE8 //00100628 
  .word 0x00030126 //0010062C 
  .word 0x00000004 //00100630 
  .word 0x40100458 //00100634 
  .word 0xB0000502 //00100638 
  .word 0xA0000003 //0010063C 
  .word 0xFFFFFFE8 //00100640 
  .word 0x0003012B //00100644 
  .word 0x00000003 //00100648 
  .word 0x40100458 //0010064C 
  .word 0xB0000501 //00100650 
  .word 0xA0000003 //00100654 
  .word 0xFFFFFFE8 //00100658 
  .word 0x0003012F //0010065C 
  .word 0x00000003 //00100660 
  .word 0x40100458 //00100664 
  .word 0xB0000434 //00100668 
  .word 0xA0000003 //0010066C 
  .word 0xFFFFFFE8 //00100670 
  .word 0x00030133 //00100674 
  .word 0x00000004 //00100678 
  .word 0x40100458 //0010067C 
  .word 0xB0000300 //00100680 
  .word 0xA0000003 //00100684 
  .word 0xFFFFFFE8 //00100688 
  .word 0x00030138 //0010068C 
  .word 0x00000005 //00100690 
  .word 0x40100458 //00100694 
  .word 0xB000043C //00100698 
  .word 0xA0000003 //0010069C 
  .word 0xFFFFFFE8 //001006A0 
  .word 0x0003013E //001006A4 
  .word 0x00000005 //001006A8 
  .word 0x40100458 //001006AC 
  .word 0xB000060C //001006B0 
  .word 0xA0000003 //001006B4 
  .word 0xFFFFFFE8 //001006B8 
  .word 0x00030144 //001006BC 
  .word 0x00000004 //001006C0 
  .word 0x40100458 //001006C4 
  .word 0xB0000603 //001006C8 
  .word 0xA0000003 //001006CC 
  .word 0xFFFFFFE8 //001006D0 
  .word 0x00030149 //001006D4 
  .word 0x00000005 //001006D8 
  .word 0x40100458 //001006DC 
  .word 0xB0000200 //001006E0 
  .word 0xA0000003 //001006E4 
  .word 0xFFFFFFE8 //001006E8 
  .word 0x0003014F //001006EC 
  .word 0x00000004 //001006F0 
  .word 0x40100458 //001006F4 
  .word 0x80000000 //001006F8 
  .word 0xA0000003 //001006FC 
  .word 0xFFFFFFE8 //00100700 
  .word 0x00030154 //00100704 
  .word 0x00000002 //00100708 
  .word 0x40101E34 //0010070C 
  .word 0x0002F04C //00100710 
  .word 0xA000000A //00100714 
  .word 0xA0000019 //00100718 
  .word 0x00000001 //0010071C 
  .word 0x0002F04C //00100720 
  .word 0x40100B34 //00100724 
  .word 0xA0000003 //00100728 
  .word 0xFFFFFFD4 //0010072C 
  .word 0x00030157 //00100730 
  .word 0x00000002 //00100734 
  .word 0x40101E34 //00100738 
  .word 0x0002F04C //0010073C 
  .word 0xA000000A //00100740 
  .word 0x40100168 //00100744 
  .word 0xB0000501 //00100748 
  .word 0x40100C38 //0010074C 
  .word 0xB0000412 //00100750 
  .word 0xB0000501 //00100754 
  .word 0xA000001A //00100758 
  .word 0x40100710 //0010075C 
  .word 0x40100A3C //00100760 
  .word 0xB0000412 //00100764 
  .word 0x40100A58 //00100768 
  .word 0xB0000501 //0010076C 
  .word 0xA000000D //00100770 
  .word 0x9FFFFFD8 //00100774 
  .word 0xB0000200 //00100778 
  .word 0x00000020 //0010077C 
  .word 0x40100710 //00100780 
  .word 0xA0000003 //00100784 
  .word 0xFFFFFFA4 //00100788 
  .word 0x0003015A //0010078C 
  .word 0x00000007 //00100790 
  .word 0x40101E18 //00100794 
  .word 0x401017D8 //00100798 
  .word 0x0002F040 //0010079C 
  .word 0xA000000A //001007A0 
  .word 0x9000000C //001007A4 
  .word 0x4010073C //001007A8 
  .word 0x40100B60 //001007AC 
  .word 0x40101ACC //001007B0 
  .word 0xA0000003 //001007B4 
  .word 0xFFFFFFD0 //001007B8 
  .word 0x00030162 //001007BC 
  .word 0x00000005 //001007C0 
  .word 0x40101E34 //001007C4 
  .word 0x40101AF4 //001007C8 
  .word 0x00000001 //001007CC 
  .word 0x0002F040 //001007D0 
  .word 0xA0000009 //001007D4 
  .word 0x40100C38 //001007D8 
  .word 0x40100794 //001007DC 
  .word 0xFFFFFFFF //001007E0 
  .word 0x0002F054 //001007E4 
  .word 0x40100B34 //001007E8 
  .word 0xA0000003 //001007EC 
  .word 0xFFFFFFC8 //001007F0 
  .word 0x00030168 //001007F4 
  .word 0x00000001 //001007F8 
  .word 0x00000022 //001007FC 
  .word 0x40100798 //00100800 
  .word 0xA0000003 //00100804 
  .word 0xFFFFFFE8 //00100808 
  .word 0x0003016A //0010080C 
  .word 0x00000002 //00100810 
  .word 0x00000022 //00100814 
  .word 0x40100798 //00100818 
  .word 0x40100D28 //0010081C 
  .word 0xA0000003 //00100820 
  .word 0xFFFFFFE4 //00100824 
  .word 0x0003016D //00100828 
  .word 0x00000004 //0010082C 
  .word 0x40101E34 //00100830 
  .word 0x0002F03C //00100834 
  .word 0xA000000A //00100838 
  .word 0xA0000003 //0010083C 
  .word 0xFFFFFFE4 //00100840 
  .word 0x00030172 //00100844 
  .word 0x00000005 //00100848 
  .word 0x40101E34 //0010084C 
  .word 0x00000008 //00100850 
  .word 0xA0000003 //00100854 
  .word 0xFFFFFFE8 //00100858 
  .word 0x00030178 //0010085C 
  .word 0x00000006 //00100860 
  .word 0x40101E34 //00100864 
  .word 0x00000009 //00100868 
  .word 0xA0000003 //0010086C 
  .word 0xFFFFFFE8 //00100870 
  .word 0x0003017F //00100874 
  .word 0x00000006 //00100878 
  .word 0x40101E34 //0010087C 
  .word 0x00000000 //00100880 
  .word 0x10000000 //00100884 
  .word 0xB0000434 //00100888 
  .word 0xA0000002 //0010088C 
  .word 0xB0000412 //00100890 
  .word 0xB0000300 //00100894 
  .word 0xB0000412 //00100898 
  .word 0x0FFFFFFF //0010089C 
  .word 0xA0000008 //001008A0 
  .word 0xA000000E //001008A4 
  .word 0xA0000003 //001008A8 
  .word 0xFFFFFFC4 //001008AC 
  .word 0x00030186 //001008B0 
  .word 0x00000005 //001008B4 
  .word 0x40101E34 //001008B8 
  .word 0x0002F03C //001008BC 
  .word 0x40100B34 //001008C0 
  .word 0xA0000003 //001008C4 
  .word 0xFFFFFFE4 //001008C8 
  .word 0x0003018C //001008CC 
  .word 0x00000007 //001008D0 
  .word 0x40101E34 //001008D4 
  .word 0x40100834 //001008D8 
  .word 0x00000004 //001008DC 
  .word 0xA0000007 //001008E0 
  .word 0x40100A74 //001008E4 
  .word 0x40100850 //001008E8 
  .word 0x40100880 //001008EC 
  .word 0x40100C38 //001008F0 
  .word 0xA0000003 //001008F4 
  .word 0xFFFFFFD0 //001008F8 
  .word 0x00030194 //001008FC 
  .word 0x00000008 //00100900 
  .word 0x40101E34 //00100904 
  .word 0x40100834 //00100908 
  .word 0x00000004 //0010090C 
  .word 0xA0000007 //00100910 
  .word 0x40100A74 //00100914 
  .word 0x40100868 //00100918 
  .word 0x40100880 //0010091C 
  .word 0x40100C38 //00100920 
  .word 0xA0000003 //00100924 
  .word 0xFFFFFFD0 //00100928 
  .word 0x0003019D //0010092C 
  .word 0x00000005 //00100930 
  .word 0x40101DFC //00100934 
  .word 0x40100834 //00100938 
  .word 0xA0000003 //0010093C 
  .word 0xFFFFFFE8 //00100940 
  .word 0x000301A3 //00100944 
  .word 0x00000005 //00100948 
  .word 0x40101DFC //0010094C 
  .word 0x401008D8 //00100950 
  .word 0xA0000003 //00100954 
  .word 0xFFFFFFE8 //00100958 
  .word 0x000301A9 //0010095C 
  .word 0x00000005 //00100960 
  .word 0x40101DFC //00100964 
  .word 0x40100908 //00100968 
  .word 0xA0000003 //0010096C 
  .word 0xFFFFFFE8 //00100970 
  .word 0x000301AF //00100974 
  .word 0x00000002 //00100978 
  .word 0x40101DFC //0010097C 
  .word 0x40100868 //00100980 
  .word 0x00000004 //00100984 
  .word 0x401008BC //00100988 
  .word 0x40100834 //0010098C 
  .word 0xA0000003 //00100990 
  .word 0xFFFFFFDC //00100994 
  .word 0x000301B2 //00100998 
  .word 0x00000006 //0010099C 
  .word 0x40101DFC //001009A0 
  .word 0x40100834 //001009A4 
  .word 0xB0000502 //001009A8 
  .word 0x40100A74 //001009AC 
  .word 0xB0000434 //001009B0 
  .word 0x40100880 //001009B4 
  .word 0xB0000412 //001009B8 
  .word 0x00000004 //001009BC 
  .word 0x40100A74 //001009C0 
  .word 0xA0000009 //001009C4 
  .word 0xA0000003 //001009C8 
  .word 0xFFFFFFC8 //001009CC 
  .word 0x000301B9 //001009D0 
  .word 0x00000004 //001009D4 
  .word 0x40101DFC //001009D8 
  .word 0x00000004 //001009DC 
  .word 0x401008BC //001009E0 
  .word 0x401009A0 //001009E4 
  .word 0x40100850 //001009E8 
  .word 0x40100834 //001009EC 
  .word 0xA0000003 //001009F0 
  .word 0xFFFFFFD8 //001009F4 
  .word 0x000301BE //001009F8 
  .word 0x00000005 //001009FC 
  .word 0x40101DFC //00100A00 
  .word 0x4010097C //00100A04 
  .word 0xA0000003 //00100A08 
  .word 0xFFFFFFE8 //00100A0C 
  .word 0x000301C4 //00100A10 
  .word 0x00000006 //00100A14 
  .word 0x40101DFC //00100A18 
  .word 0xB0000434 //00100A1C 
  .word 0x4010094C //00100A20 
  .word 0x401009A0 //00100A24 
  .word 0xA0000003 //00100A28 
  .word 0xFFFFFFE0 //00100A2C 
  .word 0x000301CB //00100A30 
  .word 0x00000002 //00100A34 
  .word 0x40101E34 //00100A38 
  .word 0x00000001 //00100A3C 
  .word 0xA0000007 //00100A40 
  .word 0xA0000003 //00100A44 
  .word 0xFFFFFFE4 //00100A48 
  .word 0x000301CE //00100A4C 
  .word 0x00000002 //00100A50 
  .word 0x40101E34 //00100A54 
  .word 0xFFFFFFFF //00100A58 
  .word 0xA0000007 //00100A5C 
  .word 0xA0000003 //00100A60 
  .word 0xFFFFFFE4 //00100A64 
  .word 0x000301D1 //00100A68 
  .word 0x00000002 //00100A6C 
  .word 0x40101E34 //00100A70 
  .word 0xA0000000 //00100A74 
  .word 0xA0000007 //00100A78 
  .word 0xA0000003 //00100A7C 
  .word 0xFFFFFFE4 //00100A80 
  .word 0x000301D4 //00100A84 
  .word 0x00000001 //00100A88 
  .word 0x40101E34 //00100A8C 
  .word 0x40100A74 //00100A90 
  .word 0xA000000D //00100A94 
  .word 0xA0000003 //00100A98 
  .word 0xFFFFFFE4 //00100A9C 
  .word 0x000301D6 //00100AA0 
  .word 0x00000002 //00100AA4 
  .word 0x40101E34 //00100AA8 
  .word 0x40100A74 //00100AAC 
  .word 0xA000000F //00100AB0 
  .word 0xA0000003 //00100AB4 
  .word 0xFFFFFFE4 //00100AB8 
  .word 0x000301D9 //00100ABC 
  .word 0x00000001 //00100AC0 
  .word 0x40101E34 //00100AC4 
  .word 0xB0000412 //00100AC8 
  .word 0x40100AAC //00100ACC 
  .word 0xA0000003 //00100AD0 
  .word 0xFFFFFFE4 //00100AD4 
  .word 0x000301DB //00100AD8 
  .word 0x00000002 //00100ADC 
  .word 0x40101E34 //00100AE0 
  .word 0x00000000 //00100AE4 
  .word 0xB0000434 //00100AE8 
  .word 0xB0000434 //00100AEC 
  .word 0xA0000002 //00100AF0 
  .word 0xB0000412 //00100AF4 
  .word 0xB0000300 //00100AF8 
  .word 0xA0000003 //00100AFC 
  .word 0xFFFFFFD4 //00100B00 
  .word 0x000301DE //00100B04 
  .word 0x00000003 //00100B08 
  .word 0x40101E34 //00100B0C 
  .word 0x000301E2 //00100B10 
  .word 0x00000004 //00100B14 
  .word 0x4010081C //00100B18 
  .word 0x8FFFFFF0 //00100B1C 
  .word 0xA0000003 //00100B20 
  .word 0xFFFFFFDC //00100B24 
  .word 0x000301E7 //00100B28 
  .word 0x00000002 //00100B2C 
  .word 0x40101E34 //00100B30 
  .word 0xB0000412 //00100B34 
  .word 0xB0000502 //00100B38 
  .word 0xA000000A //00100B3C 
  .word 0xA0000007 //00100B40 
  .word 0xB0000412 //00100B44 
  .word 0xA0000009 //00100B48 
  .word 0xA0000003 //00100B4C 
  .word 0xFFFFFFD4 //00100B50 
  .word 0x000301EA //00100B54 
  .word 0x00000002 //00100B58 
  .word 0x40101E34 //00100B5C 
  .word 0x00002802 //00100B60 
  .word 0xA000000A //00100B64 
  .word 0x00000004 //00100B68 // 1
  .word 0xA0000007 //00100B6C 
  .word 0xA000000A //00100B70 
  .word 0x00002802 //00100B74 
  .word 0xA000000A //00100B78 
  .word 0x00000004 //00100B7C // 1
  .word 0xA0000007 //00100B80 
  .word 0x00002802 //00100B84 
  .word 0xB0000603 //00100B88 
  .word 0xA000000A //00100B8C 
  .word 0xA000000A //00100B90 
  .word 0xB0000412 //00100B94 
  .word 0xA0000009 //00100B98 
  .word 0xA0000009 //00100B9C 
  .word 0xA0000003 //00100BA0 
  .word 0xFFFFFFAC //00100BA4 
  .word 0x000301ED //00100BA8 
  .word 0x00000002 //00100BAC 
  .word 0x40101E34 //00100BB0 
  .word 0x00002802 //00100BB4 
  .word 0xA000000A //00100BB8 
  .word 0xB0000501 //00100BBC 
  .word 0xFFFFFFFC //00100BC0 // -1
  .word 0xA0000007 //00100BC4 
  .word 0x00002802 //00100BC8 
  .word 0xB0000603 //00100BCC 
  .word 0xA000000A //00100BD0 
  .word 0xA000000A //00100BD4 
  .word 0xB0000412 //00100BD8 
  .word 0xB0000501 //00100BDC 
  .word 0xFFFFFFFC //00100BE0 // -1
  .word 0xA0000007 //00100BE4 
  .word 0x00002802 //00100BE8 
  .word 0xA0000009 //00100BEC 
  .word 0xA0000009 //00100BF0 
  .word 0xA0000009 //00100BF4 
  .word 0xA0000009 //00100BF8 
  .word 0xA0000003 //00100BFC 
  .word 0xFFFFFFA4 //00100C00 
  .word 0x000301F0 //00100C04 
  .word 0x00000001 //00100C08 
  .word 0x40101E34 //00100C0C 
  .word 0x00002802 //00100C10 
  .word 0xA000000A //00100C14 
  .word 0x00000004 //00100C18 // 1
  .word 0xA0000007 //00100C1C 
  .word 0xA000000A //00100C20 
  .word 0xA0000003 //00100C24 
  .word 0xFFFFFFD8 //00100C28 
  .word 0x000301F2 //00100C2C 
  .word 0x00000001 //00100C30 
  .word 0x40101E34 //00100C34 
  .word 0x0002F03C //00100C38 
  .word 0xA000000A //00100C3C 
  .word 0xA0000009 //00100C40 
  .word 0x00000004 //00100C44 
  .word 0x0002F03C //00100C48 
  .word 0x40100B34 //00100C4C 
  .word 0xA0000003 //00100C50 
  .word 0xFFFFFFD4 //00100C54 
  .word 0x000301F4 //00100C58 
  .word 0x00000007 //00100C5C 
  .word 0x40101E34 //00100C60 
  .word 0x00002803 //00100C64 
  .word 0xA0000009 //00100C68 
  .word 0xA0000003 //00100C6C 
  .word 0xFFFFFFE4 //00100C70 
  .word 0x000301FC //00100C74 
  .word 0x00000003 //00100C78 
  .word 0x40101E34 //00100C7C 
  .word 0xA0000004 //00100C80 
  .word 0x80000000 //00100C84 
  .word 0xA000000B //00100C88 
  .word 0x90000008 //00100C8C 
  .word 0xB0000300 //00100C90 
  .word 0x8FFFFFE8 //00100C94 
  .word 0xA0000003 //00100C98 
  .word 0xFFFFFFD4 //00100C9C 
  .word 0x00030200 //00100CA0 
  .word 0x00000004 //00100CA4 
  .word 0x40101E34 //00100CA8 
  .word 0x00100530 //00100CAC 
  .word 0x40100C64 //00100CB0 
  .word 0xA0000003 //00100CB4 
  .word 0xFFFFFFE4 //00100CB8 
  .word 0x00030205 //00100CBC 
  .word 0x00000005 //00100CC0 
  .word 0x40101E34 //00100CC4 
  .word 0x00000000 //00100CC8 
  .word 0xB0000412 //00100CCC 
  .word 0x00000010 //00100CD0 
  .word 0xA0000002 //00100CD4 
  .word 0xB0000412 //00100CD8 
  .word 0xA0000003 //00100CDC 
  .word 0xFFFFFFD8 //00100CE0 
  .word 0x0003020B //00100CE4 
  .word 0x00000003 //00100CE8 
  .word 0x40101E34 //00100CEC 
  .word 0xB0000501 //00100CF0 
  .word 0x0000000A //00100CF4 
  .word 0x40100AAC //00100CF8 
  .word 0x90000004 //00100CFC 
  .word 0x80000008 //00100D00 
  .word 0x00000007 //00100D04 
  .word 0xA0000007 //00100D08 
  .word 0x00000030 //00100D0C 
  .word 0xA0000007 //00100D10 
  .word 0xA0000003 //00100D14 
  .word 0xFFFFFFC8 //00100D18 
  .word 0x0003020F //00100D1C 
  .word 0x00000004 //00100D20 
  .word 0x40101E34 //00100D24 
  .word 0xB0000501 //00100D28 
  .word 0x90000020 //00100D2C 
  .word 0xB0000412 //00100D30 
  .word 0xB0000501 //00100D34 
  .word 0xA000001A //00100D38 
  .word 0x40100CAC //00100D3C 
  .word 0x40100A3C //00100D40 
  .word 0xB0000412 //00100D44 
  .word 0x40100A58 //00100D48 
  .word 0x8FFFFFD8 //00100D4C 
  .word 0xB0000200 //00100D50 
  .word 0xA0000003 //00100D54 
  .word 0xFFFFFFC0 //00100D58 
  .word 0x00030214 //00100D5C 
  .word 0x00000003 //00100D60 
  .word 0x40101E34 //00100D64 
  .word 0x00000008 //00100D68 
  .word 0xB0000412 //00100D6C 
  .word 0x40100CC8 //00100D70 
  .word 0x40100CF0 //00100D74 
  .word 0x40100CAC //00100D78 
  .word 0xB0000412 //00100D7C 
  .word 0x00000001 //00100D80 
  .word 0x40100A74 //00100D84 
  .word 0xB0000501 //00100D88 
  .word 0xA000000D //00100D8C 
  .word 0x9FFFFFD8 //00100D90 
  .word 0xB0000200 //00100D94 
  .word 0x80000000 //00100D98 
  .word 0xA0000003 //00100D9C 
  .word 0xFFFFFFB8 //00100DA0 
  .word 0x00030218 //00100DA4 
  .word 0x00000002 //00100DA8 
  .word 0x40101E34 //00100DAC 
  .word 0x40100D68 //00100DB0 
  .word 0x00000020 //00100DB4 
  .word 0x40100CAC //00100DB8 
  .word 0xA0000003 //00100DBC 
  .word 0xFFFFFFE0 //00100DC0 
  .word 0x0003021B //00100DC4 
  .word 0x00000002 //00100DC8 
  .word 0x40101E34 //00100DCC 
  .word 0xA000000A //00100DD0 
  .word 0x40100DB0 //00100DD4 
  .word 0xA0000003 //00100DD8 
  .word 0xFFFFFFE4 //00100DDC 
  .word 0x0003021E //00100DE0 
  .word 0x00000002 //00100DE4 
  .word 0x40101E34 //00100DE8 
  .word 0x0002F01C //00100DEC 
  .word 0xA000000A //00100DF0 
  .word 0x0002F03C //00100DF4 
  .word 0xA000000A //00100DF8 
  .word 0x40100A74 //00100DFC 
  .word 0x0002F040 //00100E00 
  .word 0xA000000A //00100E04 
  .word 0xA000000D //00100E08 
  .word 0xA000000B //00100E0C 
  .word 0xA000000E //00100E10 
  .word 0x0002F000 //00100E14 
  .word 0xA000000A //00100E18 
  .word 0xA000000D //00100E1C 
  .word 0xA000000B //00100E20 
  .word 0xA0000008 //00100E24 
  .word 0x900000A0 //00100E28 
  .word 0x0000003C //00100E2C 
  .word 0x40100CAC //00100E30 
  .word 0x00030221 //00100E34 
  .word 0x00000003 //00100E38 
  .word 0x4010081C //00100E3C 
  .word 0x0002F01C //00100E40 
  .word 0xA000000A //00100E44 
  .word 0x40100DB0 //00100E48 
  .word 0x0002F018 //00100E4C 
  .word 0xA000000A //00100E50 
  .word 0x40100DB0 //00100E54 
  .word 0x0000003C //00100E58 
  .word 0x40100CAC //00100E5C 
  .word 0x00030225 //00100E60 
  .word 0x00000004 //00100E64 
  .word 0x4010081C //00100E68 
  .word 0x0000003C //00100E6C 
  .word 0x40100CAC //00100E70 
  .word 0x0003022A //00100E74 
  .word 0x00000003 //00100E78 
  .word 0x4010081C //00100E7C 
  .word 0x0002F03C //00100E80 
  .word 0xA000000A //00100E84 
  .word 0x40100DB0 //00100E88 
  .word 0x0002F04C //00100E8C 
  .word 0xA000000A //00100E90 
  .word 0x40100DB0 //00100E94 
  .word 0x0000003C //00100E98 
  .word 0x40100CAC //00100E9C 
  .word 0x0003022E //00100EA0 
  .word 0x00000004 //00100EA4 
  .word 0x4010081C //00100EA8 
  .word 0x0002F03C //00100EAC 
  .word 0xA000000A //00100EB0 
  .word 0x0002F01C //00100EB4 
  .word 0xA0000009 //00100EB8 
  .word 0x0002F04C //00100EBC 
  .word 0xA000000A //00100EC0 
  .word 0x0002F018 //00100EC4 
  .word 0xA0000009 //00100EC8 
  .word 0x0000000A //00100ECC 
  .word 0x40100CAC //00100ED0 
  .word 0xA0000003 //00100ED4 
  .word 0xFFFFFF04 //00100ED8 
  .word 0x00030233 //00100EDC 
  .word 0x0000000A //00100EE0 
  .word 0x40101E34 //00100EE4 
  .word 0xA0000003 //00100EE8 
  .word 0xFFFFFFEC //00100EEC 
  .word 0x0003023E //00100EF0 
  .word 0x00000007 //00100EF4 
  .word 0x40101E34 //00100EF8 
  .word 0x40100DEC //00100EFC 
  .word 0x00030246 //00100F00 
  .word 0x00000019 //00100F04 
  .word 0x4010081C //00100F08 
  .word 0x00000020 //00100F0C 
  .word 0x40100CAC //00100F10 
  .word 0x00000008 //00100F14 
  .word 0x40100CAC //00100F18 
  .word 0x40100C80 //00100F1C 
  .word 0x0000001B //00100F20 
  .word 0x40100A90 //00100F24 
  .word 0x9FFFFFE0 //00100F28 
  .word 0xA0000003 //00100F2C 
  .word 0xFFFFFFBC //00100F30 
  .word 0x00030260 //00100F34 
  .word 0x00000005 //00100F38 
  .word 0x40101E34 //00100F3C 
  .word 0xB0000501 //00100F40 
  .word 0x0002F038 //00100F44 
  .word 0xA0000009 //00100F48 
  .word 0x00000000 //00100F4C 
  .word 0x0002F040 //00100F50 
  .word 0xA0000009 //00100F54 
  .word 0x40100DEC //00100F58 
  .word 0x0002F028 //00100F5C 
  .word 0xA000000A //00100F60 
  .word 0x0002F030 //00100F64 
  .word 0xA000000A //00100F68 
  .word 0x0002F028 //00100F6C 
  .word 0xA000000A //00100F70 
  .word 0x40100A74 //00100F74 
  .word 0x40100A58 //00100F78 
  .word 0x40100D28 //00100F7C 
  .word 0x00030266 //00100F80 
  .word 0x00000003 //00100F84 
  .word 0x4010081C //00100F88 
  .word 0x0003026A //00100F8C 
  .word 0x0000000A //00100F90 
  .word 0x40100804 //00100F94 
  .word 0x40101B48 //00100F98 
  .word 0x40100DEC //00100F9C 
  .word 0x00030275 //00100FA0 
  .word 0x00000016 //00100FA4 
  .word 0x4010081C //00100FA8 
  .word 0x40100DB0 //00100FAC 
  .word 0x40100EFC //00100FB0 
  .word 0x40101C7C //00100FB4 
  .word 0xA0000003 //00100FB8 
  .word 0xFFFFFF74 //00100FBC 
  .word 0x0003028C //00100FC0 
  .word 0x00000004 //00100FC4 
  .word 0x40101E34 //00100FC8 
  .word 0x00002801 //00100FCC 
  .word 0xA000000A //00100FD0 
  .word 0x0002F054 //00100FD4 
  .word 0xA0000009 //00100FD8 
  .word 0xA0000003 //00100FDC 
  .word 0xFFFFFFDC //00100FE0 
  .word 0x00030291 //00100FE4 
  .word 0x00000004 //00100FE8 
  .word 0x40101E34 //00100FEC 
  .word 0x00002801 //00100FF0 
  .word 0xA000000A //00100FF4 
  .word 0x0002F054 //00100FF8 
  .word 0xA000000A //00100FFC 
  .word 0x40100A74 //00101000 
  .word 0x90000008 //00101004 
  .word 0x00000009 //00101008 
  .word 0x40100F40 //0010100C 
  .word 0xA0000003 //00101010 
  .word 0xFFFFFFCC //00101014 
  .word 0x00030296 //00101018 
  .word 0x00000005 //0010101C 
  .word 0x40101E34 //00101020 
  .word 0x40100A3C //00101024 
  .word 0x00000004 //00101028 
  .word 0x40100AE4 //0010102C 
  .word 0x0002F05C //00101030 
  .word 0xA000000A //00101034 
  .word 0xB0000502 //00101038 
  .word 0x40100A74 //0010103C 
  .word 0xB0000501 //00101040 
  .word 0x0002F05C //00101044 
  .word 0xA0000009 //00101048 
  .word 0xA0000009 //0010104C 
  .word 0xA0000003 //00101050 
  .word 0xFFFFFFC0 //00101054 
  .word 0x0003029C //00101058 
  .word 0x00000009 //0010105C 
  .word 0x40101E34 //00101060 
  .word 0x0002F05C //00101064 
  .word 0xA000000A //00101068 
  .word 0xB0000501 //0010106C 
  .word 0xA000000A //00101070 
  .word 0xA0000007 //00101074 
  .word 0x0002F05C //00101078 
  .word 0xA0000009 //0010107C 
  .word 0xA0000003 //00101080 
  .word 0xFFFFFFD0 //00101084 
  .word 0x000302A6 //00101088 
  .word 0x00000002 //0010108C 
  .word 0x40101E34 //00101090 
  .word 0x0002F05C //00101094 
  .word 0xA000000A //00101098 
  .word 0x00000004 //0010109C 
  .word 0xA0000007 //001010A0 
  .word 0xA0000003 //001010A4 
  .word 0xFFFFFFDC //001010A8 
  .word 0x000302A9 //001010AC 
  .word 0x00000002 //001010B0 
  .word 0x40101E34 //001010B4 
  .word 0x0002F05C //001010B8 
  .word 0xA000000A //001010BC 
  .word 0x00000008 //001010C0 
  .word 0xA0000007 //001010C4 
  .word 0xA0000003 //001010C8 
  .word 0xFFFFFFDC //001010CC 
  .word 0x000302AC //001010D0 
  .word 0x00000002 //001010D4 
  .word 0x40101E34 //001010D8 
  .word 0x0002F05C //001010DC 
  .word 0xA000000A //001010E0 
  .word 0x0000000C //001010E4 
  .word 0xA0000007 //001010E8 
  .word 0xA0000003 //001010EC 
  .word 0xFFFFFFDC //001010F0 
  .word 0x000302AF //001010F4 
  .word 0x00000002 //001010F8 
  .word 0x40101E34 //001010FC 
  .word 0x0002F05C //00101100 
  .word 0xA000000A //00101104 
  .word 0x00000010 //00101108 
  .word 0xA0000007 //0010110C 
  .word 0xA0000003 //00101110 
  .word 0xFFFFFFDC //00101114 
  .word 0x000302B2 //00101118 
  .word 0x00000002 //0010111C 
  .word 0x40101E34 //00101120 
  .word 0x0002F05C //00101124 
  .word 0xA000000A //00101128 
  .word 0x00000014 //0010112C 
  .word 0xA0000007 //00101130 
  .word 0xA0000003 //00101134 
  .word 0xFFFFFFDC //00101138 
  .word 0x000302B5 //0010113C 
  .word 0x00000002 //00101140 
  .word 0x40101E34 //00101144 
  .word 0x0002F05C //00101148 
  .word 0xA000000A //0010114C 
  .word 0x00000018 //00101150 
  .word 0xA0000007 //00101154 
  .word 0xA0000003 //00101158 
  .word 0xFFFFFFDC //0010115C 
  .word 0x000302B8 //00101160 
  .word 0x00000002 //00101164 
  .word 0x40101E34 //00101168 
  .word 0x0002F05C //0010116C 
  .word 0xA000000A //00101170 
  .word 0x0000001C //00101174 
  .word 0xA0000007 //00101178 
  .word 0xA0000003 //0010117C 
  .word 0xFFFFFFDC //00101180 
  .word 0x000302BB //00101184 
  .word 0x00000002 //00101188 
  .word 0x40101E34 //0010118C 
  .word 0x0002F05C //00101190 
  .word 0xA000000A //00101194 
  .word 0x00000020 //00101198 
  .word 0xA0000007 //0010119C 
  .word 0xA0000003 //001011A0 
  .word 0xFFFFFFDC //001011A4 
  .word 0x000302BE //001011A8 
  .word 0x00000001 //001011AC 
  .word 0x40101DFC //001011B0 
  .word 0x00000020 //001011B4 
  .word 0x401017D8 //001011B8 
  .word 0x401019A0 //001011BC 
  .word 0x40101AA8 //001011C0 
  .word 0xB0000300 //001011C4 
  .word 0x00000004 //001011C8 
  .word 0xA0000007 //001011CC 
  .word 0x0002F040 //001011D0 
  .word 0xA000000A //001011D4 
  .word 0x90000004 //001011D8 
  .word 0x40100168 //001011DC 
  .word 0xA0000003 //001011E0 
  .word 0xFFFFFFC0 //001011E4 
  .word 0x000302C0 //001011E8 
  .word 0x00000005 //001011EC 
  .word 0x40101E34 //001011F0 
  .word 0xB0000501 //001011F4 
  .word 0xA000000A //001011F8 
  .word 0x40100A3C //001011FC 
  .word 0xB0000501 //00101200 
  .word 0x000003FF //00101204 
  .word 0xA0000008 //00101208 
  .word 0x00000000 //0010120C 
  .word 0x40100A90 //00101210 
  .word 0x90000008 //00101214 
  .word 0xFFFFFC00 //00101218 
  .word 0xA0000007 //0010121C 
  .word 0xB0000412 //00101220 
  .word 0xA0000009 //00101224 
  .word 0xA0000003 //00101228 
  .word 0xFFFFFFB8 //0010122C 
  .word 0x000302C6 //00101230 
  .word 0x00000007 //00101234 
  .word 0x40101E34 //00101238 
  .word 0x00002800 //0010123C 
  .word 0xA000000A //00101240 
  .word 0xB0000501 //00101244 
  .word 0x00000008 //00101248 
  .word 0x40100AAC //0010124C 
  .word 0x90000024 //00101250 
  .word 0x00000018 //00101254 
  .word 0xA0000007 //00101258 
  .word 0xA000000A //0010125C 
  .word 0xB0000501 //00101260 
  .word 0x90000008 //00101264 
  .word 0xB0000501 //00101268 
  .word 0x40100C64 //0010126C 
  .word 0xB0000300 //00101270 
  .word 0x80000060 //00101274 
  .word 0x0002F00C //00101278 
  .word 0xA000000A //0010127C 
  .word 0xA0000009 //00101280 
  .word 0x0002F00C //00101284 
  .word 0x401011F4 //00101288 
  .word 0x0002F00C //0010128C 
  .word 0xA000000A //00101290 
  .word 0x0002F010 //00101294 
  .word 0xA000000A //00101298 
  .word 0x40100A74 //0010129C 
  .word 0x000003FF //001012A0 
  .word 0xA0000008 //001012A4 
  .word 0x00000080 //001012A8 
  .word 0x40100AC8 //001012AC 
  .word 0x90000024 //001012B0 
  .word 0x0002F014 //001012B4 
  .word 0xA000000A //001012B8 
  .word 0xA000000D //001012BC 
  .word 0x90000014 //001012C0 
  .word 0xFFFFFFFF //001012C4 
  .word 0x0002F014 //001012C8 
  .word 0xA0000009 //001012CC 
  .word 0x00000013 //001012D0 
  .word 0x40100CAC //001012D4 
  .word 0x00000000 //001012D8 
  .word 0x00002800 //001012DC 
  .word 0xA0000009 //001012E0 
  .word 0xA0000003 //001012E4 
  .word 0xFFFFFF44 //001012E8 
  .word 0x000302CE //001012EC 
  .word 0x00000008 //001012F0 
  .word 0x40101E34 //001012F4 
  .word 0x0002F010 //001012F8 
  .word 0xA000000A //001012FC 
  .word 0x0002F00C //00101300 
  .word 0xA000000A //00101304 
  .word 0x40100A90 //00101308 
  .word 0x9000000C //0010130C 
  .word 0x00000000 //00101310 
  .word 0x00000000 //00101314 
  .word 0x80000060 //00101318 
  .word 0x0002F010 //0010131C 
  .word 0xA000000A //00101320 
  .word 0xA000000A //00101324 
  .word 0xFFFFFFFF //00101328 
  .word 0x0002F010 //0010132C 
  .word 0x401011F4 //00101330 
  .word 0x0002F00C //00101334 
  .word 0xA000000A //00101338 
  .word 0x0002F010 //0010133C 
  .word 0xA000000A //00101340 
  .word 0x40100A74 //00101344 
  .word 0x000003FF //00101348 
  .word 0xA0000008 //0010134C 
  .word 0x00000020 //00101350 
  .word 0x40100AAC //00101354 
  .word 0x90000020 //00101358 
  .word 0x0002F014 //0010135C 
  .word 0xA000000A //00101360 
  .word 0x90000014 //00101364 
  .word 0x00000000 //00101368 
  .word 0x0002F014 //0010136C 
  .word 0xA0000009 //00101370 
  .word 0x00000011 //00101374 
  .word 0x40100CAC //00101378 
  .word 0xA0000003 //0010137C 
  .word 0xFFFFFF68 //00101380 
  .word 0x000302D7 //00101384 
  .word 0x00000006 //00101388 
  .word 0x40101E34 //0010138C 
  .word 0x00000005 //00101390 
  .word 0x40101024 //00101394 
  .word 0x401010DC //00101398 
  .word 0xA0000009 //0010139C 
  .word 0x401010B8 //001013A0 
  .word 0xA0000009 //001013A4 
  .word 0x401010B8 //001013A8 
  .word 0xA000000A //001013AC 
  .word 0x40101124 //001013B0 
  .word 0xA0000009 //001013B4 
  .word 0x40100C80 //001013B8 
  .word 0xB0000501 //001013BC 
  .word 0x00000014 //001013C0 
  .word 0x40100A90 //001013C4 
  .word 0x90000010 //001013C8 
  .word 0xB0000300 //001013CC 
  .word 0x401010B8 //001013D0 
  .word 0xA000000A //001013D4 
  .word 0xA000001A //001013D8 
  .word 0xB0000501 //001013DC 
  .word 0x0000007F //001013E0 
  .word 0x40100A90 //001013E4 
  .word 0x90000008 //001013E8 
  .word 0xB0000300 //001013EC 
  .word 0x00000008 //001013F0 
  .word 0xB0000501 //001013F4 
  .word 0x00000008 //001013F8 
  .word 0x40100A90 //001013FC 
  .word 0x90000048 //00101400 
  .word 0x40101124 //00101404 
  .word 0xA000000A //00101408 
  .word 0x401010B8 //0010140C 
  .word 0xA000000A //00101410 
  .word 0x40100AAC //00101414 
  .word 0x90000030 //00101418 
  .word 0xFFFFFFFF //0010141C 
  .word 0x401010B8 //00101420 
  .word 0x40100B34 //00101424 
  .word 0x00000001 //00101428 
  .word 0x401010DC //0010142C 
  .word 0x40100B34 //00101430 
  .word 0x00000008 //00101434 
  .word 0x40100CAC //00101438 
  .word 0x00000020 //0010143C 
  .word 0x40100CAC //00101440 
  .word 0x00000008 //00101444 
  .word 0x40100CAC //00101448 
  .word 0xB0000501 //0010144C 
  .word 0x00000020 //00101450 
  .word 0x40100AAC //00101454 
  .word 0x90000004 //00101458 
  .word 0x80000048 //0010145C 
  .word 0xFFFFFFFF //00101460 
  .word 0x401010DC //00101464 
  .word 0x40100B34 //00101468 
  .word 0x401010DC //0010146C 
  .word 0xA000000A //00101470 
  .word 0xA000000F //00101474 
  .word 0x90000008 //00101478 
  .word 0x00000006 //0010147C 
  .word 0x40100F40 //00101480 
  .word 0xB0000501 //00101484 
  .word 0x40100CAC //00101488 
  .word 0xB0000501 //0010148C 
  .word 0x401010B8 //00101490 
  .word 0xA000000A //00101494 
  .word 0xA0000019 //00101498 
  .word 0x00000001 //0010149C 
  .word 0x401010B8 //001014A0 
  .word 0x40100B34 //001014A4 
  .word 0xB0000501 //001014A8 
  .word 0x00000020 //001014AC 
  .word 0x40100AAC //001014B0 
  .word 0xB0000502 //001014B4 
  .word 0x00000008 //001014B8 
  .word 0x40100A90 //001014BC 
  .word 0xA000000B //001014C0 
  .word 0xA0000008 //001014C4 
  .word 0xB0000412 //001014C8 
  .word 0x0000001B //001014CC 
  .word 0x40100A90 //001014D0 
  .word 0xA000000B //001014D4 
  .word 0xA0000008 //001014D8 
  .word 0x401010DC //001014DC 
  .word 0xA000000A //001014E0 
  .word 0xA000000D //001014E4 
  .word 0xA000000E //001014E8 
  .word 0x9FFFFEC8 //001014EC 
  .word 0x00000020 //001014F0 
  .word 0x40100CAC //001014F4 
  .word 0x40101124 //001014F8 
  .word 0xA000000A //001014FC 
  .word 0x401010B8 //00101500 
  .word 0xA000000A //00101504 
  .word 0x40101124 //00101508 
  .word 0xA000000A //0010150C 
  .word 0x40100A74 //00101510 
  .word 0xB0000603 //00101514 
  .word 0xA0000007 //00101518 
  .word 0x00000000 //0010151C 
  .word 0xB0000412 //00101520 
  .word 0xA0000019 //00101524 
  .word 0x40101064 //00101528 
  .word 0xA0000003 //0010152C 
  .word 0xFFFFFE50 //00101530 
  .word 0x000302DE //00101534 
  .word 0x00000005 //00101538 
  .word 0x40101E34 //0010153C 
  .word 0xB0000501 //00101540 
  .word 0x00000030 //00101544 
  .word 0x40100AAC //00101548 
  .word 0xA000000B //0010154C 
  .word 0xB0000502 //00101550 
  .word 0x0000003A //00101554 
  .word 0x40100AAC //00101558 
  .word 0xA0000008 //0010155C 
  .word 0xB0000502 //00101560 
  .word 0x00000041 //00101564 
  .word 0x40100AAC //00101568 
  .word 0xA000000B //0010156C 
  .word 0xA000000E //00101570 
  .word 0xB0000501 //00101574 
  .word 0x90000054 //00101578 
  .word 0xB0000412 //0010157C 
  .word 0x00000030 //00101580 
  .word 0x40100A74 //00101584 
  .word 0xB0000501 //00101588 
  .word 0x0000000A //0010158C 
  .word 0x40100AAC //00101590 
  .word 0xA000000B //00101594 
  .word 0x90000008 //00101598 
  .word 0x00000007 //0010159C 
  .word 0x40100A74 //001015A0 
  .word 0xB0000501 //001015A4 
  .word 0x0002F020 //001015A8 
  .word 0xA000000A //001015AC 
  .word 0x40100AAC //001015B0 
  .word 0xA000000B //001015B4 
  .word 0x90000010 //001015B8 
  .word 0xB0000300 //001015BC 
  .word 0xB0000300 //001015C0 
  .word 0x00000000 //001015C4 
  .word 0x00000000 //001015C8 
  .word 0xB0000412 //001015CC 
  .word 0xA0000003 //001015D0 
  .word 0xFFFFFF5C //001015D4 
  .word 0x000302E4 //001015D8 
  .word 0x00000006 //001015DC 
  .word 0x40101E34 //001015E0 
  .word 0x00000007 //001015E4 
  .word 0x40101024 //001015E8 
  .word 0x401010B8 //001015EC 
  .word 0xA0000009 //001015F0 
  .word 0x40101094 //001015F4 
  .word 0xA0000009 //001015F8 
  .word 0x00000000 //001015FC 
  .word 0x401010B8 //00101600 
  .word 0xA000000A //00101604 
  .word 0x9000018C //00101608 
  .word 0xB0000501 //0010160C 
  .word 0x401010DC //00101610 
  .word 0xA0000009 //00101614 
  .word 0x00000001 //00101618 
  .word 0x40101148 //0010161C 
  .word 0xA0000009 //00101620 
  .word 0xFFFFFFFF //00101624 
  .word 0x4010116C //00101628 
  .word 0xA0000009 //0010162C 
  .word 0x40101094 //00101630 
  .word 0xA000000A //00101634 
  .word 0x401010DC //00101638 
  .word 0xA000000A //0010163C 
  .word 0xA0000007 //00101640 
  .word 0xA000001A //00101644 
  .word 0x0000002B //00101648 
  .word 0x40100A90 //0010164C 
  .word 0x90000024 //00101650 
  .word 0x401010DC //00101654 
  .word 0xA000000A //00101658 
  .word 0x40100A3C //0010165C 
  .word 0x401010DC //00101660 
  .word 0xA0000009 //00101664 
  .word 0x00000000 //00101668 
  .word 0x4010116C //0010166C 
  .word 0xA0000009 //00101670 
  .word 0x80000058 //00101674 
  .word 0x40101094 //00101678 
  .word 0xA000000A //0010167C 
  .word 0x401010DC //00101680 
  .word 0xA000000A //00101684 
  .word 0xA0000007 //00101688 
  .word 0xA000001A //0010168C 
  .word 0x0000002D //00101690 
  .word 0x40100A90 //00101694 
  .word 0x90000034 //00101698 
  .word 0x401010DC //0010169C 
  .word 0xA000000A //001016A0 
  .word 0x40100A3C //001016A4 
  .word 0x401010DC //001016A8 
  .word 0xA0000009 //001016AC 
  .word 0x00000000 //001016B0 
  .word 0x4010116C //001016B4 
  .word 0xA0000009 //001016B8 
  .word 0x40101148 //001016BC 
  .word 0xA000000A //001016C0 
  .word 0xA0000000 //001016C4 
  .word 0x40101148 //001016C8 
  .word 0xA0000009 //001016CC 
  .word 0x4010116C //001016D0 
  .word 0xA000000A //001016D4 
  .word 0x9FFFFF48 //001016D8 
  .word 0x401010DC //001016DC 
  .word 0xA000000A //001016E0 
  .word 0x401010B8 //001016E4 
  .word 0xA000000A //001016E8 
  .word 0x40100AAC //001016EC 
  .word 0x900000A4 //001016F0 
  .word 0x40101094 //001016F4 
  .word 0xA000000A //001016F8 
  .word 0x401010DC //001016FC 
  .word 0xA000000A //00101700 
  .word 0xA0000007 //00101704 
  .word 0xA000001A //00101708 
  .word 0xB0000501 //0010170C 
  .word 0x90000054 //00101710 
  .word 0x40101540 //00101714 
  .word 0xA000000B //00101718 
  .word 0x9000001C //0010171C 
  .word 0xB0000300 //00101720 
  .word 0x401010B8 //00101724 
  .word 0xA000000A //00101728 
  .word 0xA0000000 //0010172C 
  .word 0x401010B8 //00101730 
  .word 0xA0000009 //00101734 
  .word 0x80000028 //00101738 
  .word 0xB0000412 //0010173C 
  .word 0x0002F020 //00101740 
  .word 0xA000000A //00101744 
  .word 0x40100AE4 //00101748 
  .word 0xA0000007 //0010174C 
  .word 0x401010DC //00101750 
  .word 0xA000000A //00101754 
  .word 0x40100A3C //00101758 
  .word 0x401010DC //0010175C 
  .word 0xA0000009 //00101760 
  .word 0x80000014 //00101764 
  .word 0xB0000300 //00101768 
  .word 0x401010DC //0010176C 
  .word 0xA000000A //00101770 
  .word 0x401010B8 //00101774 
  .word 0xA0000009 //00101778 
  .word 0x401010DC //0010177C 
  .word 0xA000000A //00101780 
  .word 0x401010B8 //00101784 
  .word 0xA000000A //00101788 
  .word 0x40100AAC //0010178C 
  .word 0xA000000B //00101790 
  .word 0x9FFFFF5C //00101794 
  .word 0x40101148 //00101798 
  .word 0xA000000A //0010179C 
  .word 0xA000000F //001017A0 
  .word 0x90000004 //001017A4 
  .word 0xA0000000 //001017A8 
  .word 0x401010DC //001017AC 
  .word 0xA000000A //001017B0 
  .word 0x401010B8 //001017B4 
  .word 0xA000000A //001017B8 
  .word 0x40100A74 //001017BC 
  .word 0x40101064 //001017C0 
  .word 0xA0000003 //001017C4 
  .word 0xFFFFFE0C //001017C8 
  .word 0x000302EB //001017CC 
  .word 0x00000004 //001017D0 
  .word 0x40101E34 //001017D4 
  .word 0x40100BB4 //001017D8 
  .word 0x0002F030 //001017DC 
  .word 0xA000000A //001017E0 
  .word 0x0002F02C //001017E4 
  .word 0xA0000009 //001017E8 
  .word 0x0002F030 //001017EC 
  .word 0xA000000A //001017F0 
  .word 0xA000001A //001017F4 
  .word 0x40100C10 //001017F8 
  .word 0x40100A90 //001017FC 
  .word 0x0002F030 //00101800 
  .word 0xA000000A //00101804 
  .word 0x0002F034 //00101808 
  .word 0xA000000A //0010180C 
  .word 0x40100AAC //00101810 
  .word 0xA0000008 //00101814 
  .word 0x90000010 //00101818 
  .word 0x00000001 //0010181C 
  .word 0x0002F030 //00101820 
  .word 0x40100B34 //00101824 
  .word 0x8FFFFFC0 //00101828 
  .word 0x0002F030 //0010182C 
  .word 0xA000000A //00101830 
  .word 0x0002F02C //00101834 
  .word 0xA0000009 //00101838 
  .word 0x0002F030 //0010183C 
  .word 0xA000000A //00101840 
  .word 0xA000001A //00101844 
  .word 0x0000003C //00101848 
  .word 0x40100A90 //0010184C 
  .word 0x90000010 //00101850 
  .word 0x0002F030 //00101854 
  .word 0xA000000A //00101858 
  .word 0x0002F034 //0010185C 
  .word 0xA0000009 //00101860 
  .word 0x0002F030 //00101864 
  .word 0xA000000A //00101868 
  .word 0xA000001A //0010186C 
  .word 0x40100C10 //00101870 
  .word 0x40100A90 //00101874 
  .word 0xA000000B //00101878 
  .word 0x0002F030 //0010187C 
  .word 0xA000000A //00101880 
  .word 0x0002F034 //00101884 
  .word 0xA000000A //00101888 
  .word 0x40100AAC //0010188C 
  .word 0xA0000008 //00101890 
  .word 0x90000010 //00101894 
  .word 0x00000001 //00101898 
  .word 0x0002F030 //0010189C 
  .word 0x40100B34 //001018A0 
  .word 0x8FFFFF94 //001018A4 
  .word 0x0002F02C //001018A8 
  .word 0xA000000A //001018AC 
  .word 0x0002F030 //001018B0 
  .word 0xA000000A //001018B4 
  .word 0xB0000502 //001018B8 
  .word 0x40100A74 //001018BC 
  .word 0xB0000501 //001018C0 
  .word 0x9000000C //001018C4 
  .word 0x00000001 //001018C8 
  .word 0x0002F030 //001018CC 
  .word 0x40100B34 //001018D0 
  .word 0x40100B60 //001018D4 
  .word 0xB0000300 //001018D8 
  .word 0xA0000003 //001018DC 
  .word 0xFFFFFEE8 //001018E0 
  .word 0x000302F0 //001018E4 
  .word 0x00000002 //001018E8 
  .word 0x40101E34 //001018EC 
  .word 0xB0000434 //001018F0 
  .word 0xB0000603 //001018F4 
  .word 0x40100A74 //001018F8 
  .word 0x90000010 //001018FC 
  .word 0xB0000200 //00101900 
  .word 0xB0000200 //00101904 
  .word 0x00000000 //00101908 
  .word 0x8000007C //0010190C 
  .word 0xB0000501 //00101910 
  .word 0x9000005C //00101914 
  .word 0xB000043C //00101918 
  .word 0xB0000502 //0010191C 
  .word 0xA000001A //00101920 
  .word 0xB0000502 //00101924 
  .word 0xA000001A //00101928 
  .word 0x40100A74 //0010192C 
  .word 0x90000018 //00101930 
  .word 0xB0000200 //00101934 
  .word 0xB0000200 //00101938 
  .word 0x00000001 //0010193C 
  .word 0x00000001 //00101940 
  .word 0x00000000 //00101944 
  .word 0x00000000 //00101948 
  .word 0xB0000501 //0010194C 
  .word 0x90000010 //00101950 
  .word 0x40100A3C //00101954 
  .word 0xB0000412 //00101958 
  .word 0x40100A3C //0010195C 
  .word 0xB0000412 //00101960 
  .word 0xB000043C //00101964 
  .word 0xFFFFFFFF //00101968 
  .word 0xA0000007 //0010196C 
  .word 0x8FFFFF9C //00101970 
  .word 0xB0000200 //00101974 
  .word 0xB0000300 //00101978 
  .word 0x90000008 //0010197C 
  .word 0xFFFFFFFF //00101980 
  .word 0x80000004 //00101984 
  .word 0x00000000 //00101988 
  .word 0xA0000003 //0010198C 
  .word 0xFFFFFF50 //00101990 
  .word 0x000302F3 //00101994 
  .word 0x00000004 //00101998 
  .word 0x40101E34 //0010199C 
  .word 0x40100BB4 //001019A0 
  .word 0x40100BB4 //001019A4 
  .word 0x00000000 //001019A8 
  .word 0x0002F044 //001019AC 
  .word 0xA000000A //001019B0 
  .word 0x0002F004 //001019B4 
  .word 0xA000000A //001019B8 
  .word 0x9000000C //001019BC 
  .word 0xB0000501 //001019C0 
  .word 0xA000000A //001019C4 
  .word 0xA0000007 //001019C8 
  .word 0xB0000501 //001019CC 
  .word 0x00000004 //001019D0 
  .word 0xA0000007 //001019D4 
  .word 0xB0000501 //001019D8 
  .word 0xA000000A //001019DC 
  .word 0xB0000412 //001019E0 
  .word 0x00000004 //001019E4 
  .word 0xA0000007 //001019E8 
  .word 0xA000000A //001019EC 
  .word 0x40100B60 //001019F0 
  .word 0x40100B60 //001019F4 
  .word 0xB0000603 //001019F8 
  .word 0x40100BB4 //001019FC 
  .word 0x40100BB4 //00101A00 
  .word 0x401018F0 //00101A04 
  .word 0x9000000C //00101A08 
  .word 0xB0000412 //00101A0C 
  .word 0xA000000D //00101A10 
  .word 0xB0000412 //00101A14 
  .word 0xB0000502 //00101A18 
  .word 0xA000000D //00101A1C 
  .word 0xB0000502 //00101A20 
  .word 0xA000000A //00101A24 
  .word 0xA000000D //00101A28 
  .word 0xA000000B //00101A2C 
  .word 0xA0000008 //00101A30 
  .word 0xB0000502 //00101A34 
  .word 0xB0000501 //00101A38 
  .word 0xA000000A //00101A3C 
  .word 0xA0000007 //00101A40 
  .word 0x0002F044 //00101A44 
  .word 0xA000000A //00101A48 
  .word 0x40100A90 //00101A4C 
  .word 0xA000000B //00101A50 
  .word 0xA0000008 //00101A54 
  .word 0x90000010 //00101A58 
  .word 0xB0000501 //00101A5C 
  .word 0xA000000A //00101A60 
  .word 0xA0000007 //00101A64 
  .word 0x8FFFFF60 //00101A68 
  .word 0x40100B60 //00101A6C 
  .word 0xB0000300 //00101A70 
  .word 0x40100B60 //00101A74 
  .word 0xB0000434 //00101A78 
  .word 0xA000000D //00101A7C 
  .word 0x90000010 //00101A80 
  .word 0xB0000300 //00101A84 
  .word 0xB0000300 //00101A88 
  .word 0x00000000 //00101A8C 
  .word 0x00000000 //00101A90 
  .word 0xA0000003 //00101A94 
  .word 0xFFFFFEF8 //00101A98 
  .word 0x000302F8 //00101A9C 
  .word 0x00000004 //00101AA0 
  .word 0x40101E34 //00101AA4 
  .word 0xB0000412 //00101AA8 
  .word 0x0000000C //00101AAC 
  .word 0xA0000007 //00101AB0 
  .word 0xB0000412 //00101AB4 
  .word 0xA0000003 //00101AB8 
  .word 0xFFFFFFDC //00101ABC 
  .word 0x000302FD //00101AC0 
  .word 0x00000008 //00101AC4 
  .word 0x40101E34 //00101AC8 
  .word 0x00000004 //00101ACC 
  .word 0x00000000 //00101AD0 
  .word 0x4010011C //00101AD4 
  .word 0xA000000E //00101AD8 
  .word 0x40100C38 //00101ADC 
  .word 0xA0000003 //00101AE0 
  .word 0xFFFFFFD8 //00101AE4 
  .word 0x00030306 //00101AE8 
  .word 0x00000006 //00101AEC 
  .word 0x40101E34 //00101AF0 
  .word 0x40100FCC //00101AF4 
  .word 0x0002F03C //00101AF8 
  .word 0xA000000A //00101AFC 
  .word 0x0002F044 //00101B00 
  .word 0xA000000A //00101B04 
  .word 0xB0000502 //00101B08 
  .word 0x40100A74 //00101B0C 
  .word 0x40100C38 //00101B10 
  .word 0x0002F044 //00101B14 
  .word 0xA0000009 //00101B18 
  .word 0x00000020 //00101B1C 
  .word 0x401017D8 //00101B20 
  .word 0x4010073C //00101B24 
  .word 0x00000001 //00101B28 
  .word 0x0002F004 //00101B2C 
  .word 0xA0000009 //00101B30 
  .word 0xA0000003 //00101B34 
  .word 0xFFFFFFAC //00101B38 
  .word 0x0003030D //00101B3C 
  .word 0x00000009 //00101B40 
  .word 0x40101E34 //00101B44 
  .word 0x0002F028 //00101B48 
  .word 0xA000000A //00101B4C 
  .word 0x40100BB4 //00101B50 
  .word 0x0002F02C //00101B54 
  .word 0xA000000A //00101B58 
  .word 0x40100BB4 //00101B5C 
  .word 0x0002F030 //00101B60 
  .word 0xA000000A //00101B64 
  .word 0x40100BB4 //00101B68 
  .word 0x0002F034 //00101B6C 
  .word 0xA000000A //00101B70 
  .word 0x40100BB4 //00101B74 
  .word 0xB0000502 //00101B78 
  .word 0xA0000007 //00101B7C 
  .word 0x0002F034 //00101B80 
  .word 0xA0000009 //00101B84 
  .word 0xB0000501 //00101B88 
  .word 0x0002F028 //00101B8C 
  .word 0xA0000009 //00101B90 
  .word 0xB0000501 //00101B94 
  .word 0x0002F02C //00101B98 
  .word 0xA0000009 //00101B9C 
  .word 0x0002F030 //00101BA0 
  .word 0xA0000009 //00101BA4 
  .word 0x00000020 //00101BA8 
  .word 0x401017D8 //00101BAC 
  .word 0xB0000501 //00101BB0 
  .word 0x9000007C //00101BB4 
  .word 0xB0000603 //00101BB8 
  .word 0x401019A0 //00101BBC 
  .word 0xB0000501 //00101BC0 
  .word 0x90000024 //00101BC4 
  .word 0x40100BB4 //00101BC8 
  .word 0x40100BB4 //00101BCC 
  .word 0xB0000200 //00101BD0 
  .word 0x40100B60 //00101BD4 
  .word 0x40100B60 //00101BD8 
  .word 0x40101AA8 //00101BDC 
  .word 0xB0000300 //00101BE0 
  .word 0x40100C64 //00101BE4 
  .word 0x80000044 //00101BE8 
  .word 0xB0000200 //00101BEC 
  .word 0xB0000603 //00101BF0 
  .word 0x401015E4 //00101BF4 
  .word 0x90000014 //00101BF8 
  .word 0xB0000200 //00101BFC 
  .word 0xB0000300 //00101C00 
  .word 0x00000003 //00101C04 
  .word 0x40100F40 //00101C08 
  .word 0x80000020 //00101C0C 
  .word 0xB0000434 //00101C10 
  .word 0xB0000300 //00101C14 
  .word 0xB0000412 //00101C18 
  .word 0xB0000300 //00101C1C 
  .word 0x0002F040 //00101C20 
  .word 0xA000000A //00101C24 
  .word 0x90000004 //00101C28 
  .word 0x40100168 //00101C2C 
  .word 0x8FFFFF74 //00101C30 
  .word 0xB0000200 //00101C34 
  .word 0x40100B60 //00101C38 
  .word 0x0002F034 //00101C3C 
  .word 0xA0000009 //00101C40 
  .word 0x40100B60 //00101C44 
  .word 0x0002F030 //00101C48 
  .word 0xA0000009 //00101C4C 
  .word 0x40100B60 //00101C50 
  .word 0x0002F02C //00101C54 
  .word 0xA0000009 //00101C58 
  .word 0x40100B60 //00101C5C 
  .word 0x0002F028 //00101C60 
  .word 0xA0000009 //00101C64 
  .word 0xA0000003 //00101C68 
  .word 0xFFFFFECC //00101C6C 
  .word 0x00030317 //00101C70 
  .word 0x00000004 //00101C74 
  .word 0x40101E34 //00101C78 
  .word 0x0002F008 //00101C7C 
  .word 0xA000000A //00101C80 
  .word 0x00002802 //00101C84 
  .word 0xA0000009 //00101C88 
  .word 0x0002F000 //00101C8C 
  .word 0xA000000A //00101C90 
  .word 0x90000018 //00101C94 
  .word 0x0000003C //00101C98 
  .word 0x40100CAC //00101C9C 
  .word 0x0003031C //00101CA0 
  .word 0x00000004 //00101CA4 
  .word 0x4010081C //00101CA8 
  .word 0x8000000C //00101CAC 
  .word 0x00030321 //00101CB0 
  .word 0x00000002 //00101CB4 
  .word 0x4010081C //00101CB8 
  .word 0x40100DEC //00101CBC 
  .word 0x0002F024 //00101CC0 
  .word 0xA000000A //00101CC4 
  .word 0x00000100 //00101CC8 
  .word 0x40101390 //00101CCC 
  .word 0xB0000502 //00101CD0 
  .word 0xA000001A //00101CD4 
  .word 0x0000003C //00101CD8 
  .word 0x40100A90 //00101CDC 
  .word 0x90000008 //00101CE0 
  .word 0xB0000200 //00101CE4 
  .word 0x800000AC //00101CE8 
  .word 0x0002F000 //00101CEC 
  .word 0xA000000A //00101CF0 
  .word 0x90000030 //00101CF4 
  .word 0x0000003C //00101CF8 
  .word 0x40100CAC //00101CFC 
  .word 0x00030324 //00101D00 
  .word 0x00000003 //00101D04 
  .word 0x4010081C //00101D08 
  .word 0x40101B48 //00101D0C 
  .word 0x0000003C //00101D10 
  .word 0x40100CAC //00101D14 
  .word 0x00030328 //00101D18 
  .word 0x00000004 //00101D1C 
  .word 0x4010081C //00101D20 
  .word 0x80000070 //00101D24 
  .word 0x0000001B //00101D28 
  .word 0x40100CAC //00101D2C 
  .word 0x0000005B //00101D30 
  .word 0x40100CAC //00101D34 
  .word 0x00000033 //00101D38 
  .word 0x40100CAC //00101D3C 
  .word 0x00000036 //00101D40 
  .word 0x40100CAC //00101D44 
  .word 0x0000006D //00101D48 
  .word 0x40100CAC //00101D4C 
  .word 0x40101B48 //00101D50 
  .word 0x0002F040 //00101D54 
  .word 0xA000000A //00101D58 
  .word 0xA000000D //00101D5C 
  .word 0x9000000C //00101D60 
  .word 0x0003032D //00101D64 
  .word 0x00000002 //00101D68 
  .word 0x4010081C //00101D6C 
  .word 0x0000001B //00101D70 
  .word 0x40100CAC //00101D74 
  .word 0x0000005B //00101D78 
  .word 0x40100CAC //00101D7C 
  .word 0x00000033 //00101D80 
  .word 0x40100CAC //00101D84 
  .word 0x00000039 //00101D88 
  .word 0x40100CAC //00101D8C 
  .word 0x0000006D //00101D90 
  .word 0x40100CAC //00101D94 
  .word 0x8FFFFF20 //00101D98 
  .word 0xA0000003 //00101D9C 
  .word 0xFFFFFECC //00101DA0 
  .word 0x00030330 //00101DA4 
  .word 0x00000005 //00101DA8 
  .word 0x40101E34 //00101DAC 
  .word 0x00030336 //00101DB0 
  .word 0x0000000B //00101DB4 
  .word 0x4010081C //00101DB8 
  .word 0x40100DEC //00101DBC 
  .word 0x40100DEC //00101DC0 
  .word 0x40101C7C //00101DC4 
  .word 0xA0000003 //00101DC8 
  .word 0xFFFFFFD4 //00101DCC 
  .word 0x00030342 //00101DD0 
  .word 0x00000006 //00101DD4 
  .word 0x40101E34 //00101DD8 
  .word 0x00000000 //00101DDC 
  .word 0x0002F004 //00101DE0 
  .word 0xA0000009 //00101DE4 
  .word 0xA0000003 //00101DE8 
  .word 0xFFFFFFE0 //00101DEC 
  .word 0x00030349 //00101DF0 
  .word 0x0000000C //00101DF4 
  .word 0x40101E34 //00101DF8 
  .word 0x40100B60 //00101DFC 
  .word 0x40100BB4 //00101E00 
  .word 0xA0000003 //00101E04 
  .word 0xFFFFFFE4 //00101E08 
  .word 0x00030356 //00101E0C 
  .word 0x0000000A //00101E10 
  .word 0x40101E34 //00101E14 
  .word 0x40100B60 //00101E18 
  .word 0x40101ACC //00101E1C 
  .word 0xA0000003 //00101E20 
  .word 0xFFFFFFE4 //00101E24 
  .word 0x00030361 //00101E28 
  .word 0x00000003 //00101E2C 
  .word 0x40101E34 //00101E30 
  .word 0x40100B60 //00101E34 
  .word 0x0002F040 //00101E38 
  .word 0xA000000A //00101E3C 
  .word 0x90000008 //00101E40 
  .word 0x40101ACC //00101E44 
  .word 0x80000004 //00101E48 
  .word 0x40100BB4 //00101E4C 
  .word 0xA0000003 //00101E50 
  .word 0xFFFFFFD0 //00101E54 
  .word 0x00030365 //00101E58 
  .word 0x0000000A //00101E5C 
  .word 0x40101E34 //00101E60 
  .word 0x40101AF4 //00101E64 
  .word 0x00000001 //00101E68 
  .word 0x0002F040 //00101E6C 
  .word 0xA0000009 //00101E70 
  .word 0x40101DF8 //00101E74 
  .word 0xA0000003 //00101E78 
  .word 0xFFFFFFD8 //00101E7C 
  .word 0x00030370 //00101E80 
  .word 0x00000008 //00101E84 
  .word 0x40101E34 //00101E88 
  .word 0x40101AF4 //00101E8C 
  .word 0x00000001 //00101E90 
  .word 0x0002F040 //00101E94 
  .word 0xA0000009 //00101E98 
  .word 0x40101E14 //00101E9C 
  .word 0xA0000003 //00101EA0 
  .word 0xFFFFFFD8 //00101EA4 
  .word 0x00030379 //00101EA8 
  .word 0x00000001 //00101EAC 
  .word 0x40101E34 //00101EB0 
  .word 0x40101AF4 //00101EB4 
  .word 0x00000001 //00101EB8 
  .word 0x0002F040 //00101EBC 
  .word 0xA0000009 //00101EC0 
  .word 0x40101E30 //00101EC4 
  .word 0xA0000003 //00101EC8 
  .word 0xFFFFFFD8 //00101ECC 
  .word 0x0003037B //00101ED0 
  .word 0x00000001 //00101ED4 
  .word 0x40101DFC //00101ED8 
  .word 0x00000000 //00101EDC 
  .word 0x0002F040 //00101EE0 
  .word 0xA0000009 //00101EE4 
  .word 0x40100FF0 //00101EE8 
  .word 0x40100438 //00101EEC 
  .word 0x40100C38 //00101EF0 
  .word 0x40101DDC //00101EF4 
  .word 0xA0000003 //00101EF8 
  .word 0xFFFFFFD0 //00101EFC 
  .word 0x0003037D //00101F00 
  .word 0x00000005 //00101F04 
  .word 0x40101E34 //00101F08 
  .word 0xB0000412 //00101F0C 
  .word 0x40100DEC //00101F10 
  .word 0x00030383 //00101F14 
  .word 0x0000000A //00101F18 
  .word 0x4010081C //00101F1C 
  .word 0xB0000501 //00101F20 
  .word 0xA000000A //00101F24 
  .word 0x40100DB0 //00101F28 
  .word 0x0003038E //00101F2C 
  .word 0x00000002 //00101F30 
  .word 0x4010081C //00101F34 
  .word 0xB0000501 //00101F38 
  .word 0x40100DB0 //00101F3C 
  .word 0x00000004 //00101F40 
  .word 0xA0000007 //00101F44 
  .word 0xB0000412 //00101F48 
  .word 0xFFFFFFFC //00101F4C 
  .word 0xA0000007 //00101F50 
  .word 0xB0000501 //00101F54 
  .word 0xA000000F //00101F58 
  .word 0x9FFFFFAC //00101F5C 
  .word 0xB0000200 //00101F60 
  .word 0x40100DEC //00101F64 
  .word 0x40100DEC //00101F68 
  .word 0xA0000003 //00101F6C 
  .word 0xB0000501 //00101F70 



Daten:
