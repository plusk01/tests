SECTION .data
ALIGN 32
four DQ 4.0, 4.0, 4.0, 4.0
two DQ 2.0, 2.0, 2.0, 2.0
one DQ 1.0, 1.0, 1.0, 1.0
ofs DQ 0.5, 1.5, 2.5, 3.5

SECTION .text

extern width,sum,num_rects

global calcPi_AVX

calcPi_AVX:
     push ebp
     mov ebp, esp
     push ebx
     push ecx

     xor ecx, ecx               ; ecx = i = 0
     vxorpd ymm0, ymm0, ymm0    ; ymm0 = sum = 0
     vbroadcastsd ymm1, [width] ; set ymm1 to step
     vmovapd ymm2, [ofs]        ; set ymm2 to (0.5, 1.5, 2.5, 3.5)
     vmovapd ymm3, [four]       ; set ymm3 to (4.0, 4.0, 4.0, 4.0)

L1:
     cmp ecx, [num_rects]       ; check termination condition
     jge L2
     ; calculate (i+0.5)*width
     vmulpd  ymm4, ymm1, ymm2
     ; square the interim result
     ; and add one to it
     vmulpd ymm4, ymm4, ymm4
     vaddpd ymm4, ymm4, [one]
     ; divide 4 by the interim result
     vdivpd  ymm4, ymm3, ymm4
     ; accumulate heights of the rectangles
     vaddpd ymm0, ymm0, ymm4
     ; increase loop counters and
     ; jump to the beginning of the loop
     vaddpd ymm2, ymm2, ymm3
     add ecx, 4
     jmp L1
L2:
     ; accumulate all element of xmm0 (= all heights)
     ; to one result (= first element of xmm3)
     vperm2f128 ymm3, ymm0, ymm0, 0x1
     vaddpd ymm3, ymm3, ymm0
     vhaddpd ymm3, ymm3, ymm3
     vmovsd [sum], xmm3

     pop ecx
     pop ebx
     pop ebp
     ret
