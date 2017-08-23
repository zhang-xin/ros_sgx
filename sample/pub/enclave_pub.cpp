#include <string>

#include "trusted_lib.h"
#include "enclave_pub_t.h"

int gen_msg(int count)
{
    std::string msg;
    msg += "hello world " + std::to_string(count);
    size_t len = msg.size();

    if (0 != do_seal_data((uint8_t *)msg.c_str(), len, NULL, 0))
        return -1;

    return 0;
}
