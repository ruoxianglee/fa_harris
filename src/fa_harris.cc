/****************************************************************************************
*  FA-Harris corner detection. 
*
*  This method will save corner events to `/corner/scene_/fa_harris.txt` 
*  with (x, y, t, p). Modify `corner.launch` file to choose scene before `launch`.
*
*  Author: Ruoxiang Li
*  Date: 2019.1.20
*
****************************************************************************************/

#include <chrono>
#include <fstream>
#include <cmath>
#include "fa_harris_detector.h"

using namespace std;

typedef std::chrono::high_resolution_clock Clock;

fa_harris::FAHarrisDetector fa_harris_detector = fa_harris::FAHarrisDetector();

ros::Publisher corner_pub;

// global stats
double total_time_ = 0.;
int total_events_ = 0, total_corners_ = 0;
double total_time_for_add_ = 0.;

// used to save corner info
ofstream outfile;

void EventMsgCallback(const dvs_msgs::EventArray::ConstPtr &event_msg) 
{
    const int n_event = event_msg->events.size();
    if (n_event == 0) {return;}

    dvs_msgs::EventArray corner_msg;
    corner_msg.header = event_msg->header;
    corner_msg.width = event_msg->width;
    corner_msg.height = event_msg->height;

    auto t_init = Clock::now();

    for (const auto& e : event_msg->events)
    {
        if (fa_harris_detector.isCorner(e))
        {
            corner_msg.events.push_back(e);
        }
    }
    auto t_end = Clock::now();

    // Summary from the processed event-package
    // Note: DO NOT use this ROS wrapper for timing benchmarking. Use the stand-alone implementation instead.
    const double elapsed_time = std::chrono::duration_cast<std::chrono::nanoseconds>(t_end - t_init).count();
    const int n_corner = corner_msg.events.size();
    const double percentage_corners = (double(n_corner)/n_event)*100;
    const double time_per_event = elapsed_time/n_event; // Average time to process one event [ns/ev]
    const double event_rate = 1/(time_per_event*1e-3) ; // Average Event Rate [Million ev / s]

    ROS_INFO(" FA-Harris. Percetange of corners: %.1f%%. Avg. timing: %0.0f ns/ev. Max event rate: %0.2f Mev/s",
            percentage_corners, time_per_event, event_rate);

    // global stats
    total_time_ += elapsed_time;
    total_events_ += n_event;
    total_corners_ += n_corner;

    // Send detected corner events
    corner_pub.publish(corner_msg);

    // save corner events to txt file
    for (const auto& e : corner_msg.events)
    {
        outfile << e.x << " " << e.y << " " << e.ts << " ";
        const int pol = e.polarity ? 1 : 0;
        outfile << pol << "\n";
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "fa_harris");
    ros::NodeHandle nh;

    // load parameter
    string scene_;
    ros::param::param<string>("~scene", scene_, "shapes_6dof");

    outfile.open("/home/eric/EventCamera/evaluation/corner/" + scene_ + "/fa_harris.txt", ios::out);

    if (!outfile.is_open())
    {
        ROS_INFO("Error opening file. ");
        return 0;
    }

    corner_pub = nh.advertise<dvs_msgs::EventArray>("corners", 1);
    ros::Subscriber event_sub = nh.subscribe("events", 0, &EventMsgCallback);

    while (ros::ok()) {ros::spinOnce();} // Preferred over ros::spin() for performance

    // print overall statistics
    std::cout << "Global Statistics: " << std::endl
    << " Total time [ns]: " << total_time_ << std::endl
    << " Total number of events: " << total_events_ << std::endl
    << " Total number of corners: " << total_corners_ << std::endl
    << " Time/event [ns]: " << total_time_ / (double)total_events_ << std::endl
    << " Events/s: " << total_events_ / total_time_ *  1e9  << std::endl
    << " Reduction (%): " << (1.  - total_corners_  / (double)total_events_) * 100 << std::endl
    << std::endl;

    outfile.close();
    return 0;
}

