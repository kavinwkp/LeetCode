	.file	"main.c"
	.text
	.globl	exchange
	.type	exchange, @function
exchange:
.LFB23:
	.cfi_startproc
	movq	(%rdi), %rax
	movq	%rsi, (%rdi)
	ret
	.cfi_endproc
.LFE23:
	.size	exchange, .-exchange
	.section	.rodata.str1.1,"aMS",@progbits,1
.LC0:
	.string	"a = %ld, b = %ld\n"
	.text
	.globl	main
	.type	main, @function
main:
.LFB24:
	.cfi_startproc
	subq	$24, %rsp
	.cfi_def_cfa_offset 32
	movq	%fs:40, %rax
	movq	%rax, 8(%rsp)
	xorl	%eax, %eax
	movq	$4, (%rsp)
	movq	%rsp, %rdi
	movl	$3, %esi
	call	exchange
	movq	%rax, %rcx
	movq	(%rsp), %rdx
	leaq	.LC0(%rip), %rsi
	movl	$1, %edi
	movl	$0, %eax
	call	__printf_chk@PLT
	movq	8(%rsp), %rdx
	xorq	%fs:40, %rdx
	jne	.L5
	movl	$0, %eax
	addq	$24, %rsp
	.cfi_remember_state
	.cfi_def_cfa_offset 8
	ret
.L5:
	.cfi_restore_state
	call	__stack_chk_fail@PLT
	.cfi_endproc
.LFE24:
	.size	main, .-main
	.ident	"GCC: (Ubuntu 7.5.0-3ubuntu1~18.04) 7.5.0"
	.section	.note.GNU-stack,"",@progbits
