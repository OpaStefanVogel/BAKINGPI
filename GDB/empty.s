.text
.global _start
_start:
    mov r0, #1
    mov r1, #2
    mov r2, #3
    add r3, r3, r2
    mov r4, #4
    b   _start
