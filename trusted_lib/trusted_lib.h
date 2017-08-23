#ifndef TRUSTED_LIB_H
#define TRUSTED_LIB_H

#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

void printf(const char *fmt, ...);

uint32_t do_seal_data(const uint8_t *data, size_t data_len, const uint8_t *aad, size_t aad_len);
uint32_t do_unseal_data(const uint8_t *sealed_data, size_t data_len, uint8_t *aad, size_t *aad_len,
                        uint8_t **unsealed_data, size_t *unsealed_len);

uint32_t ecall_seal_data(const uint8_t *data, size_t data_len, const uint8_t *aad, size_t aad_len);
uint32_t ecall_unseal_data(const uint8_t *sealed_data, size_t data_len, uint8_t *aad, size_t *aad_len);

#ifdef __cplusplus
}
#endif

#endif // TRUSTED_LIB_H
