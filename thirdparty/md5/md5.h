#ifndef MD5_H
#define MD5_H

#include <stdio.h>
#include <stdint.h>

typedef struct{
    uint64_t size;        // Size of input in bytes
    uint32_t buffer[4];   // Current accumulation of hash
    uint8_t input[64];    // Input to be used in the next step
    uint8_t digest[16];   // Result of algorithm
}MD5Context;

void md5_init(MD5Context *ctx);
void md5_update(MD5Context *ctx, uint8_t *input, size_t input_len);
void md5_finalize(MD5Context *ctx);
void md5_step(uint32_t *buffer, uint32_t *input);
void md5_string(char *input, uint8_t *result);
void md5_file(FILE *file, uint8_t *result);

#endif