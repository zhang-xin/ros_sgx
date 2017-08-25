#include "ros/ros.h"
#include "ros_sgx/sgxmsg.h"

#include <vector>

#include "untrusted_lib.h"
#include "sgx_urts.h"
#include "sgx_error.h"
#include "sgx_eid.h"
#include "enclave_sub_u.h"

sgx_enclave_id_t sgx_eid = 0;

void chatterCallback(const ros_sgx::sgxmsg::ConstPtr& msg)
{
    sgx_status_t sgx_ret = SGX_ERROR_UNEXPECTED;
    int enclave_ret = -1;

    sgx_ret = decrypt_msg(sgx_eid, &enclave_ret, &msg->sealed_data[0], msg->sealed_data.size());
    if (sgx_ret || enclave_ret)
        ROS_FATAL("cannot decrypt message");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "listener");
    ros::NodeHandle n;

    if (0 != load_enclave(&sgx_eid, "enclave_sub.signed.so")) {
        ROS_FATAL("load enclave failed.");
        return EXIT_FAILURE;
    }

    ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);

    ros::spin();

    unload_enclave(&sgx_eid);

    return 0;
}
