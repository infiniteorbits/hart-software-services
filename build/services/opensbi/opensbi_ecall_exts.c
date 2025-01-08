#include <sbi/sbi_ecall.h>

extern struct sbi_ecall_extension ecall_base;

struct sbi_ecall_extension *sbi_ecall_exts[] = {
	&ecall_base,
};

unsigned long sbi_ecall_exts_size = sizeof(sbi_ecall_exts) / sizeof(struct sbi_ecall_extension *);
