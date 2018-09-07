#include <mutex>
#include <list>
#include <condition_variable>
#include <functional>
#include <thread>
#include <chrono>

#include <ros/ros.h>
#include <librealsense2/rs.hpp>
//#include "std_msgs/String.h"
//#include <sstream>


void resetDevice(rs2::device& dev, rs2::context& ctx, const char * sn)
{
    std::mutex mtx;
    std::condition_variable cv;
    //std_msgs::String msg;
    //std::stringstream ss;

    ctx.set_devices_changed_callback([&dev, &cv, &sn](rs2::event_information& info)
    {
        if (info.was_removed(dev))
        {
            cv.notify_one();
            ROS_INFO("Device reset (%s)", sn);
        }
    });

    ROS_INFO("Resetting device (%s) ...", sn);
    dev.hardware_reset();
    {
        std::unique_lock<std::mutex> lk(mtx);
        auto status = cv.wait_for(lk, std::chrono::seconds(5));

        if (status == std::cv_status::timeout)//{
	  ROS_WARN("Device reset may not be successful (%s)", sn);
	  //ss << status;
	  //msg.data = ss.str();
	  
	  //reset_pub.publish(msg);
	//}
	
	  
    }
}

int main(int argc, char *argv[])
{
  //ros::NodeHandle nh;
  //ros::Publisher reset_pub = nh.advertise<std_msgs::String>("/realsense_reset", 5);
    try{
        rs2::context ctx;

        auto list = ctx.query_devices();
        if (0 == list.size())
        {
            ROS_ERROR("No RealSense devices found.");
            exit(1);
        }

        bool device_found = false;

        std::list<std::thread> threads;

        for (auto&& dev : list)
        {
            auto sn = dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
            if (argc <= 1 || sn == argv[1])
            {
                device_found = true;
                threads.emplace_back(std::bind(resetDevice, dev, std::ref(ctx), sn));
            }
        }
        for (auto& t : threads) t.join();

        if (!device_found && argc > 1)
        {
            ROS_WARN("Device not found (serial: %s)", argv[1]);
        }
    }
    catch(const std::exception& ex)
    {
        ROS_ERROR_STREAM("An exception has been thrown: " << ex.what());
        return 1;
    }
    catch(...)
    {
        ROS_ERROR_STREAM("Unknown exception has occured!");

        throw;
    }

    return 0;
}
