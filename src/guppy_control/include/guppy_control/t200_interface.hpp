#ifndef GUPPY_T200_INTERFACE_H
#define GUPPY_T200_INTERFACE_H

#include <string>
#include <vector>
#include <Eigen/Core>

#include <iostream>

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <unistd.h>

namespace t200_interface
{
class T200Interface {
    std::vector<unsigned int> can_ids;
    int sock_;
    std::string can_interface_;

    bool setup_can();
    bool send_to_can(unsigned int id, double value);

public:
    T200Interface(std::string can_interface, std::vector<unsigned int> can_ids) : can_interface_(can_interface), can_ids(can_ids) {
        setup_can();
    };

    bool write(Eigen::VectorXd throttles);

    bool initialize();
    bool shutdown();
};
}

#endif