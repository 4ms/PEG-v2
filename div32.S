
;************************************************************************
;*                                                                      *
;*                      divide 2^31 by a 27-bit number                  *
;*  0x8000 0000 0000 0000 / (divisor32 >> 5) 
;*  0x8000 0000 0000 0000 / 0x0000 0nnn nnnn nnnn
;*                                                                      *
;************************************************************************
 
//takes one parameter: uint32_t divisor = r25:r24:r23:r22
//returns a uint32_t = r25:r24:r23:r22

; r19..0 = r19..0 / r23..0
; r23..0 = remainde

;;.def	a0	= r21
;.def	a1	= r20
;.def	a2	= r19
;.def	a3	= r18
 
;.def	b0	= r25
;.def	b1	= r24
;.def	b2	= r23
;.def	b3	= r22
 
;.def	t0	= r17
;.def	t1	= r18
;.def	t2	= r19
;.def	t3	= r20
;.def	t4	= r21

 
.text
.global udiv32
.type udiv32, @function

udiv32:
	push r13	;a0
	push r14	;a1
	push r15	;a2
	push r16	;a3
	push r17	;t0
	push r18	;t1
	push r19	;t2
	push r20	;t3
	push r21	;t4
				;b0-b4 is r22:r23:r24:r25
	clr	r17
	clr	r18
	clr	r19
	clr	r20

;	ldi r16, 0x00
;	mov r15, r16
				
	ldi r16, 0x80	;must be regisiter r16 to r31 
	clr r15
	clr r14
	clr r13

	ldi	r21, 32	;must be regisiter r16 to r31
udi1:	lsl	r13
	rol	r14
	rol	r15
	rol	r16
	rol	r17
	rol	r18
	rol	r19
	rol	r20
	cp	r17, r22
	cpc	r18, r23
	cpc	r19, r24
	cpc	r20, r25
	brcs	udi2
	sub	r17, r22
	sbc	r18, r23
	sbc	r19, r24
	sbc	r20, r25
	inc	r13
udi2:	dec	r21
	brne	udi1
					;output is r25:r24:r23:r22
	mov r25, r16
	mov r24, r15
	mov r23, r14
	mov r22, r13

	pop r21
	pop r20
	pop r19
	pop r18
	pop r17
	pop r16
	pop r15
	pop r14
	pop r13
	ret


.size udiv32, .-udiv32
