#include "ros/ros.h"
#include "ros_sgx/sgxmsg.h"

#include <vector>

#include "untrusted_lib.h"
#include "sgx_urts.h"
#include "sgx_error.h"
#include "sgx_eid.h"
#include "enclave_pub_u.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "talker");
    ros::NodeHandle n;

    ros::Publisher chatter_pub = n.advertise<ros_sgx::sgxmsg>("chatter", 1000);

    ros::Rate loop_rate(10);

    sgx_enclave_id_t sgx_eid;
    sgx_status_t sgx_ret = SGX_ERROR_UNEXPECTED;
    int enclave_ret = -1;


    if (0 != load_enclave(&sgx_eid, "enclave_pub.signed.so")) {
        ROS_FATAL("load enclave failed.");
        return EXIT_FAILURE;
    }

    std::vector<uint8_t> buffer;
    set_sealed_data_buf(buffer);

    int count = 0;
    while (ros::ok())
    {
        ros_sgx::sgxmsg msg;

        sgx_ret = gen_msg(sgx_eid, &enclave_ret, count);

        if (sgx_ret || enclave_ret)
            ROS_FATAL("generate message failed.");

        msg.sealed_data = buffer;
        chatter_pub.publish(msg);

        ROS_INFO("published count #%d msg.", count);

        ros::spinOnce();

        loop_rate.sleep();
        ++count;
    }

    unload_enclave(&sgx_eid);

    return 0;
}
