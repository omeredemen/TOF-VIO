#ifndef TIC_TOC_H
#define TIC_TOC_H

#include "rclcpp/rclcpp.hpp"


//usage:
//tic_toc_ros tt;
//tt.dT_s(); will return the dT in second;
//tt.dT_ms(); will return the dT in mili-second


class tic_toc_ros
{
public:
    tic_toc_ros(void) {
        time = tic.seconds();
    }
    double dT_s(void){
        return (tic.seconds() - time);
    }
    double dT_ms(void){
        return (tic.seconds() - time) * 1000;
    }
    void toc(void)
    {
        std::cout << (tic.seconds() - time) * 1000 << "ms" << std::endl;
    }
    void toc(std::string str)
    {
        std::cout << str << " time:" << (tic.seconds() - time) * 1000 << "ms" << std::endl;
    }
private:
    rclcpp::Time tic;
    double time;
};



#endif // TIC_TOC_H
