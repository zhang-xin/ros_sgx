#include <stdarg.h>
#include <stdio.h>
#include <string>
#include "sgx_tseal.h"
#include "sgx_tae_service.h"

#include "trusted_lib.h"
#include "trusted_lib_t.h"

void printf(const char *fmt, ...)
{
    char buf[BUFSIZ] = {'\0'};
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, BUFSIZ, fmt, ap);
    va_end(ap);
    ocall_print_string(buf);
}

uint32_t do_seal_data(const uint8_t *data, size_t data_len, const uint8_t *aad, size_t aad_len)
{
    sgx_status_t ret = SGX_SUCCESS;

    uint32_t sealed_data_size = sgx_calc_sealed_data_size(aad_len, data_len);

    uint8_t *sealed_data = (uint8_t *)malloc(sealed_data_size);
    if (sealed_data == NULL)
        return SGX_ERROR_OUT_OF_MEMORY;

    if (data_len == 0)
        ret = sgx_mac_aadata(aad_len, aad, sealed_data_size, (sgx_sealed_data_t*)sealed_data);
    else
        ret = sgx_seal_data(aad_len, aad, data_len, data, sealed_data_size, (sgx_sealed_data_t*)sealed_data);

    if (ret != SGX_SUCCESS) {
        free(sealed_data);
        return ret;
    }

    ocall_get_sealed_data(sealed_data, sealed_data_size);

    free(sealed_data);

    return ret;
}

uint32_t ecall_seal_data(const uint8_t *data, size_t data_len, const uint8_t *aad, size_t aad_len)
{
    return do_seal_data(data, data_len, aad, aad_len);
}

uint32_t do_unseal_data(const uint8_t *sealed_data, size_t data_len, uint8_t *aad, size_t *aad_len,
                        uint8_t **unsealed_data, size_t *unsealed_len)
{
    sgx_status_t ret = SGX_SUCCESS;

    *unsealed_len = sgx_get_encrypt_txt_len((sgx_sealed_data_t *)sealed_data);

    if (sgx_get_add_mac_txt_len((sgx_sealed_data_t *)sealed_data) > *aad_len)
        return SGX_ERROR_INVALID_PARAMETER;

    *unsealed_data = (uint8_t *)malloc(*unsealed_len);
    if (*unsealed_data == NULL)
        return SGX_ERROR_OUT_OF_MEMORY;

    if (*unsealed_len != 0)
        ret = sgx_unseal_data((sgx_sealed_data_t *)sealed_data, aad, (uint32_t *)aad_len,
                              *unsealed_data, (uint32_t *)unsealed_len);
    else
        ret = sgx_unmac_aadata((sgx_sealed_data_t *)sealed_data, aad, (uint32_t *)aad_len);

    if (ret != SGX_SUCCESS) {
        free(*unsealed_data);
        return ret;
    }

    return ret;
}

uint32_t ecall_unseal_data(const uint8_t *sealed_data, size_t data_len, uint8_t *aad, size_t *aad_len)
{
    uint8_t *unsealed_data = NULL;
    size_t unsealed_len = 0;
    uint32_t ret = do_unseal_data(sealed_data, data_len, aad, aad_len, &unsealed_data, &unsealed_len);
    if (ret != 0)
        return ret;

    if (unsealed_len != 0) {
        ocall_get_unsealed_data(unsealed_data, unsealed_len);
        free(unsealed_data);
    }

    return ret;
}
