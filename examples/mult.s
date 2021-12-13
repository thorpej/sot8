	; Multiply 12 x 3
	push	#3
	push	#12
	call	$multiply
	spa	#3

	; Multiply the result by 16
	push	#16
	push	%r0
	call	$multiply
	spa	#2

	; I'm le tired.
	hlt

;
; Multiply the first argument by the second argument and return the
; result in %r0.
;
$multiply:
	ld	%r0, #1[%sp]	; %r0 <- first argument
	tst	%r0		; %r0 == 0?
	jz	$Lmul02		; Yes, return 0 (and save an insn)
	ld	%r2, #2[%sp]	; %r2 <- second argument
	tst	%r2		; %r2 == 0?
	jz	$Lmul03		; Yes, return 0
	mov	%r1, %r0	; %r1 <- %r0
$Lmul01:
	dec	%r2		; %r2--
	jz	$Lmul02		; get out if done
	add	%r0, %r1	; %r0 <- %r0 + %r1
	jmp	$Lmul01		; and again
$Lmul02:
	ret			; result already in %r0
$Lmul03:
	; We get here because %r2 is 0, and we need to return
	; 0 in %r0.  Use the known-0 value in %r2 to save a
	; byte of program space (no need for an immediate value).
	mov	%r0, %r2
	ret
