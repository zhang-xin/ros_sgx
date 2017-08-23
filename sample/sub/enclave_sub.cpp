#include <string>
#include <algorithm>

#include "trusted_lib.h"
#include "enclave_sub_t.h"

int decrypt_msg(const uint8_t *data, size_t len)
{
    size_t aad_len = 0;
    size_t unsealed_len = 0;
    uint8_t *unsealed_data = NULL;
    if (0 != do_unseal_data(data, len, NULL, &aad_len, &unsealed_data, &unsealed_len))
        return -1;

    std::string msg;
    std::copy(unsealed_data, unsealed_data + unsealed_len, msg.begin());

    printf("msg is %s", msg.c_str());

    if (unsealed_len != 0)
        free(unsealed_data);

    return 0;
}
