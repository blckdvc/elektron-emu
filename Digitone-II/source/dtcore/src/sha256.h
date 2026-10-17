/* SHA-256 (FIPS 180-4), per le impronte di confronto col Python. */
#ifndef DT_SHA256_H
#define DT_SHA256_H

#include <stddef.h>
#include <stdint.h>

#if defined(__APPLE__)
#include <CommonCrypto/CommonDigest.h>
typedef struct {
    CC_SHA256_CTX cc;             /* veloce (estensioni SHA della CPU) */
} dt_sha256;
#else
typedef struct {
    uint32_t h[8];
    uint64_t len;
    uint8_t buf[64];
    size_t fill;
} dt_sha256;
#endif

void dt_sha256_init(dt_sha256 *s);
void dt_sha256_update(dt_sha256 *s, const void *data, size_t n);
void dt_sha256_final(dt_sha256 *s, uint8_t out[32]);
void dt_sha256_hex(const uint8_t digest[32], char out[65]);

#endif
