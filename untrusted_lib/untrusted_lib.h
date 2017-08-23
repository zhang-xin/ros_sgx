#ifndef UNTRUSTED_LIB_H
#define UNTRUSTED_LIB_H

#include <vector>

#include "sgx_urts.h"
#include "sgx_eid.h"
#include "sgx_error.h"

void set_sealed_data_buf(std::vector<uint8_t> &buf);
void set_unsealed_data_buf(std::vector<uint8_t> &buf);

#if defined(__cplusplus)
extern "C" {
#endif

int load_enclave(sgx_enclave_id_t *sgx_eid, const char *enclave_file);
void unload_enclave(sgx_enclave_id_t *sgx_eid);

void ocall_print_string(const char *str);

void ocall_get_sealed_data(const uint8_t *sealed_data, size_t len);
void ocall_get_unsealed_data(const uint8_t *unsealed_data, size_t len);

#if defined(__cplusplus)
}
#endif

#endif // UNTRUSTED_LIB_H
