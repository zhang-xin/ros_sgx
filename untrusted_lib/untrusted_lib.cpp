#include <stdio.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <algorithm>

#include "untrusted_lib.h"

std::vector<uint8_t> *p_sealed_data_buf;
std::vector<uint8_t> *p_unsealed_data_buf;

int load_enclave(sgx_enclave_id_t *sgx_eid, const char *enclave_file)
{
    sgx_launch_token_t token = {0};
    int updated = 0;
    sgx_status_t ret = SGX_ERROR_UNEXPECTED;

    if (SGX_SUCCESS != sgx_create_enclave(enclave_file, SGX_DEBUG_FLAG, &token, &updated, sgx_eid, NULL)) {
        ROS_FATAL("creating enclave failed");
        return -1;
    }

    return 0;
}

void unload_enclave(sgx_enclave_id_t *sgx_eid)
{
    if (*sgx_eid != 0)
        sgx_destroy_enclave(*sgx_eid);
}

void ocall_print_string(const char *str)
{
    ROS_INFO("%s", str);
}

void ocall_get_sealed_data(const uint8_t *sealed_data, size_t len)
{
    if (len > p_sealed_data_buf->size())
        p_sealed_data_buf->resize(len);

    std::copy(&sealed_data[0], &sealed_data[len], p_sealed_data_buf->begin());
}

void ocall_get_unsealed_data(const uint8_t *unsealed_data, size_t len)
{
    if (len > p_unsealed_data_buf->size())
        p_unsealed_data_buf->resize(len);

    std::copy(&unsealed_data[0], &unsealed_data[len], p_unsealed_data_buf->begin());
}

void set_sealed_data_buf(std::vector<uint8_t> &buf)
{
    p_sealed_data_buf = &buf;
}

void set_unsealed_data_buf(std::vector<uint8_t> &buf)
{
    p_unsealed_data_buf = &buf;
}
