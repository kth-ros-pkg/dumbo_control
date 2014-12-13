/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/


/*
 *  dumbo_control_loop.cpp
 *
 *  Runs Dumbo's main low level hardware control loop
    (Schunk arms, parallel gripper and force-torque sensors)
    following the ros_control framework.
    Much of the code is adapted from PR2's ethercat realtime control loop
    located in the pr2_robot package.
 *  Created on: Nov 29, 2014
 *  Authors:   Francisco Viña
 *            fevb <at> kth.se
 */

/* Copyright (c) 2014, Francisco Vina, CVAP, KTH
   All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:
      * Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
      * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
      * Neither the name of the KTH nor the
        names of its contributors may be used to endorse or promote products
        derived from this software without specific prior written permission.

   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
   ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
   WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
   DISCLAIMED. IN NO EVENT SHALL KTH BE LIABLE FOR ANY
   DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
   (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
   ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
   SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include <dumbo_hardware_interface/dumbo_hw.h>
#include <realtime_tools/realtime_publisher.h>
#include <std_srvs/Empty.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <control_msgs/GripperCommand.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_updater/DiagnosticStatusWrapper.h>
#include <boost/scoped_ptr.hpp>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/max.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <pthread.h>
#include <signal.h>
#include <sys/mman.h>

using namespace boost::accumulators;

namespace dumbo_control
{


class RTLoopHistory
{
public:
  RTLoopHistory(unsigned length, double default_value) :
    index_(0),
    length_(length),
    history_(new double[length])
  {
    for (unsigned i=0; i<length_; ++i)
      history_[i] = default_value;
  }

  ~RTLoopHistory()
  {
    delete[] history_;
    history_ = NULL;
  }

  void sample(double value)
  {
    index_ = (index_+1) % length_;
    history_[index_] = value;
  }

  double average() const
  {
    double sum(0.0);
    for (unsigned i=0; i<length_; ++i)
      sum+=history_[i];
    return sum / double(length_);
  }

protected:
  unsigned index_;
  unsigned length_;
  double *history_;
};

bool g_quit = false;

static struct
{
  accumulator_set<double, stats<tag::max, tag::mean> > read_acc;
  accumulator_set<double, stats<tag::max, tag::mean> > cm_acc;
  accumulator_set<double, stats<tag::max, tag::mean> > write_acc;
  accumulator_set<double, stats<tag::max, tag::mean> > loop_acc;
  accumulator_set<double, stats<tag::max, tag::mean> > jitter_acc;
  int overruns;
  int recent_overruns;
  int last_overrun;
  int last_severe_overrun;
  double overrun_loop_sec;
  double overrun_read;
  double overrun_cm;
  double overrun_write;

  // These values are set when realtime loop does not meet performace expections
  bool rt_loop_not_making_timing;
  double halt_rt_loop_frequency;
  double rt_loop_frequency;
} g_stats;

// other variables used in realtime control loop
static const int NSEC_PER_SECOND = 1e+9;
static const int USEC_PER_SECOND = 1e+6;

class DumboHWControlLoop
{
public:

    // thread variables for realtime control loop
    pthread_t controlThread;
    pthread_attr_t controlThreadAttr;


     DumboHWControlLoop():
        connect_dumbo_(false),
        disconnect_dumbo_(false),
        stop_dumbo_(false),
        recover_dumbo_(false),
        connect_left_arm_(false),
        disconnect_left_arm_(false),
        connect_right_arm_(false),
        disconnect_right_arm_(false),
        connect_pg70_(false),
        disconnect_pg70_(false)
    {
        nh_ = ros::NodeHandle("~");

        advertiseServices();
        advertiseTopics();

        // subscribe to gripper pos command
        pg70_pos_command_sub_ = nh_.subscribe("/PG70_gripper/pos_command", 1, &DumboHWControlLoop::pg70PosCommandCallback, this);

        // Catch attempts to quit
        signal(SIGTERM, quitRequested);
        signal(SIGINT, quitRequested);
        signal(SIGHUP, quitRequested);
        signal(SIGKILL, quitRequested);

        // start realtime hw control loop thread
        pthread_attr_init(&controlThreadAttr);

        int rv;
        if ((rv = pthread_create(&controlThread, &controlThreadAttr, DumboHWControlLoop::controlLoop, this)) != 0)
        {
          ROS_FATAL("Unable to create control thread: rv = %d", rv);
          exit(EXIT_FAILURE);
        }

    }

    ~DumboHWControlLoop()
    {
    }

    void advertiseServices()
    {

        connect_dumbo_srv_server_ = nh_.advertiseService("/dumbo/connect", &DumboHWControlLoop::connectDumboSrvCallback, this);
        disconnect_dumbo_srv_server_ = nh_.advertiseService("/dumbo/disconnect", &DumboHWControlLoop::disconnectDumboSrvCallback, this);

        stop_dumbo_srv_server_ = nh_.advertiseService("/dumbo/stop", &DumboHWControlLoop::stopDumboSrvCallback, this);
        recover_dumbo_srv_server_ = nh_.advertiseService("/dumbo/recover", &DumboHWControlLoop::recoverDumboSrvCallback, this);

        connect_left_arm_srv_server_ = nh_.advertiseService("/left_arm/connect", &DumboHWControlLoop::connectLeftArmSrvCallback, this);
        disconnect_left_arm_srv_server_ = nh_.advertiseService("/left_arm/disconnect", &DumboHWControlLoop::disconnectLeftArmSrvCallback, this);

        connect_right_arm_srv_server_ = nh_.advertiseService("/right_arm/connect", &DumboHWControlLoop::connectRightArmSrvCallback, this);
        disconnect_right_arm_srv_server_ = nh_.advertiseService("/right_arm/disconnect", &DumboHWControlLoop::disconnectRightArmSrvCallback, this);

        connect_pg70_srv_server_ = nh_.advertiseService("/PG70_gripper/connect", &DumboHWControlLoop::connectPG70SrvCallback, this);
        disconnect_pg70_srv_server_ = nh_.advertiseService("/PG70_gripper/disconnect", &DumboHWControlLoop::disconnectPG70SrvCallback, this);
    }

    void advertiseTopics()
    {
        left_arm_status_publisher_.reset(new realtime_tools::RealtimePublisher<std_msgs::Bool>(nh_, "/left_arm/connected", 1));
        right_arm_status_publisher_.reset(new realtime_tools::RealtimePublisher<std_msgs::Bool>(nh_, "/right_arm/connected", 1));

        pg70_status_publisher_.reset(new realtime_tools::RealtimePublisher<std_msgs::Bool>(nh_, "/PG70_gripper/connected", 1));

        left_arm_ft_sensor_status_publisher_.reset(new realtime_tools::RealtimePublisher<std_msgs::Bool>(nh_, "/left_arm_ft_sensor/connected", 1));
        right_arm_ft_sensor_status_publisher_.reset(new realtime_tools::RealtimePublisher<std_msgs::Bool>(nh_, "/right_arm_ft_sensor/connected", 1));
    }

    // service callback definitions

    bool connectDumboSrvCallback(std_srvs::Empty::Request &req,
                                 std_srvs::Empty::Response &res)
    {
        connect_dumbo_ = true;
    }

    bool disconnectDumboSrvCallback(std_srvs::Empty::Request &req,
                                    std_srvs::Empty::Response &res)
    {
        disconnect_dumbo_ = true;
    }

    bool stopDumboSrvCallback(std_srvs::Empty::Request &req,
                              std_srvs::Empty::Response &res)
    {
        stop_dumbo_ = true;
    }

    bool recoverDumboSrvCallback(std_srvs::Empty::Request &req,
                                 std_srvs::Empty::Response &res)
    {
        recover_dumbo_ = true;
    }

    bool connectLeftArmSrvCallback(std_srvs::Empty::Request &req,
                                   std_srvs::Empty::Response &res)
    {
        connect_left_arm_ = true;
    }

    bool disconnectLeftArmSrvCallback(std_srvs::Empty::Request &req,
                                      std_srvs::Empty::Response &res)
    {
        disconnect_left_arm_ = true;
    }


    bool connectRightArmSrvCallback(std_srvs::Empty::Request &req,
                                    std_srvs::Empty::Response &res)
    {
        connect_right_arm_ = true;
    }

    bool disconnectRightArmSrvCallback(std_srvs::Empty::Request &req,
                                       std_srvs::Empty::Response &res)
    {
        disconnect_right_arm_ = true;
    }

    bool connectPG70SrvCallback(std_srvs::Empty::Request &req,
                                std_srvs::Empty::Response &res)
    {
        connect_pg70_ = true;
    }

    bool disconnectPG70SrvCallback(std_srvs::Empty::Request &req,
                                   std_srvs::Empty::Response &res)
    {
        disconnect_pg70_ = true;
    }

    void pg70PosCommandCallback(const control_msgs::GripperCommand::ConstPtr &gripper_pos_command)
    {
        pg70_pos_command_requested_ = true;
        pg70_pos_command_ = gripper_pos_command->position;
    }

    static void publishDiagnostics(realtime_tools::RealtimePublisher<diagnostic_msgs::DiagnosticArray> &publisher)
    {
      if (publisher.trylock())
      {
        accumulator_set<double, stats<tag::max, tag::mean> > zero;
        std::vector<diagnostic_msgs::DiagnosticStatus> statuses;
        diagnostic_updater::DiagnosticStatusWrapper status;

        static double max_read = 0, max_cm = 0, max_write = 0, max_loop = 0, max_jitter = 0;
        double avg_read, avg_cm, avg_write, avg_loop, avg_jitter;

        avg_read           = extract_result<tag::mean>(g_stats.read_acc);
        avg_cm           = extract_result<tag::mean>(g_stats.cm_acc);
        avg_write           = extract_result<tag::mean>(g_stats.write_acc);
        avg_loop         = extract_result<tag::mean>(g_stats.loop_acc);
        max_read           = std::max(max_read, extract_result<tag::max>(g_stats.read_acc));
        max_cm           = std::max(max_cm, extract_result<tag::max>(g_stats.cm_acc));
        max_write           = std::max(max_write, extract_result<tag::max>(g_stats.write_acc));
        max_loop         = std::max(max_loop, extract_result<tag::max>(g_stats.loop_acc));
        g_stats.read_acc   = zero;
        g_stats.cm_acc   = zero;
        g_stats.write_acc = zero;
        g_stats.loop_acc = zero;

        // Publish average loop jitter
        avg_jitter         = extract_result<tag::mean>(g_stats.jitter_acc);
        max_jitter         = std::max(max_jitter, extract_result<tag::max>(g_stats.jitter_acc));
        g_stats.jitter_acc = zero;

        static bool first = true;
        if (first)
        {
          first = false;
          status.add("Robot Description", "dumbo");
        }

        status.addf("Max dumbo read roundtrip (us)", "%.2f", max_read*USEC_PER_SECOND);
        status.addf("Avg dumbo read roundtrip (us)", "%.2f", avg_read*USEC_PER_SECOND);
        status.addf("Max Controller Manager roundtrip (us)", "%.2f", max_cm*USEC_PER_SECOND);
        status.addf("Avg Controller Manager roundtrip (us)", "%.2f", avg_cm*USEC_PER_SECOND);
        status.addf("Max dumbo write roundtrip (us)", "%.2f", max_write*USEC_PER_SECOND);
        status.addf("Avg dumbo write roundtrip (us)", "%.2f", avg_write*USEC_PER_SECOND);
        status.addf("Max Total Loop roundtrip (us)", "%.2f", max_loop*USEC_PER_SECOND);
        status.addf("Avg Total Loop roundtrip (us)", "%.2f", avg_loop*USEC_PER_SECOND);
        status.addf("Max Loop Jitter (us)", "%.2f", max_jitter * USEC_PER_SECOND);
        status.addf("Avg Loop Jitter (us)", "%.2f", avg_jitter * USEC_PER_SECOND);
        status.addf("Control Loop Overruns", "%d", g_stats.overruns);
        status.addf("Recent Control Loop Overruns", "%d", g_stats.recent_overruns);
        status.addf("Last Control Loop Overrun Cause", "read: %.2fus, cm: %.2fus, write: %.2fus",
                    g_stats.overrun_read*USEC_PER_SECOND, g_stats.overrun_cm*USEC_PER_SECOND,
                    g_stats.overrun_write*USEC_PER_SECOND);
        status.addf("Last Overrun Loop Time (us)", "%.2f", g_stats.overrun_loop_sec * USEC_PER_SECOND);
        status.addf("Realtime Loop Frequency", "%.4f", g_stats.rt_loop_frequency);

        status.name = "Realtime Control Loop";
        if (g_stats.overruns > 0 && g_stats.last_overrun < 30)
        {
          if (g_stats.last_severe_overrun < 30)
        status.level = 1;
          else
        status.level = 0;
          status.message = "Realtime loop used too much time in the last 30 seconds.";
        }
        else
        {
          status.level = 0;
          status.message = "OK";
        }
        g_stats.recent_overruns = 0;
        g_stats.last_overrun++;
        g_stats.last_severe_overrun++;

        if (g_stats.rt_loop_not_making_timing)
        {
          status.mergeSummaryf(status.ERROR, "Halting, realtime loop only ran at %.4f Hz", g_stats.halt_rt_loop_frequency);
        }

        statuses.push_back(status);
        publisher.msg_.status = statuses;
        publisher.msg_.header.stamp = ros::Time::now();
        publisher.unlockAndPublish();
      }
    }


    static void *controlLoop(void * ptr)
    {
        DumboHWControlLoop* object_ptr = (DumboHWControlLoop *) ptr;
        int rv = 0;
        double last_published, last_loop_start;
        int period;
        int policy;

        ros::NodeHandle nh;

        // Realtime loop should be running at least 450Hz
        // Calculate realtime loop frequency every 200mseec
        // Halt motors if average frequency over last 600msec is less than 450Hz
        double min_acceptable_rt_loop_frequency;
        if (!nh.getParam("min_acceptable_rt_loop_frequency", min_acceptable_rt_loop_frequency))
        {
          min_acceptable_rt_loop_frequency = 450.0;
        }
        else
        {
          ROS_WARN("min_acceptable_rt_loop_frequency changed to %f", min_acceptable_rt_loop_frequency);
        }
        unsigned rt_cycle_count = 0;
        double last_rt_monitor_time;
        double rt_loop_monitor_period = 0.6 / 3;
        // Keep history of last 3 calculation intervals.
        RTLoopHistory rt_loop_history(3, 500.0);

        dumbo_hardware_interface::DumboHW dumbo_hw;
        controller_manager::ControllerManager cm(&dumbo_hw, nh);

        realtime_tools::RealtimePublisher<diagnostic_msgs::DiagnosticArray> diag_publisher(nh, "/diagnostics", 2);

        // Publish one-time before entering real-time to pre-allocate message vectors
        publishDiagnostics(diag_publisher);

        // Set to realtime scheduler for this thread
        struct sched_param thread_param;
        policy = SCHED_FIFO;
        thread_param.sched_priority = sched_get_priority_max(policy);
        pthread_setschedparam(pthread_self(), policy, &thread_param);

        struct timespec tick;
        clock_gettime(CLOCK_REALTIME, &tick);
        //period = 1e+6; // 1 ms in nanoseconds
        period = 2e+6; // 2 ms in nanoseconds

        // Snap to the nearest second
        tick.tv_sec = tick.tv_sec;
        tick.tv_nsec = (tick.tv_nsec / period + 1) * period;
        clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &tick, NULL);

        last_published = now();
        last_rt_monitor_time = now();
        last_loop_start = now();

        while(!g_quit)
        {
            // Track how long the actual loop takes
            double this_loop_start = now();
            double this_loop_period = this_loop_start - last_loop_start;
            g_stats.loop_acc(this_loop_start - last_loop_start);
            last_loop_start = this_loop_start;

            double start = now();

            // read sensors and encoders
            dumbo_hw.read();

            double after_read = now();

            // update controller manager
            cm.update(ros::Time::now(), ros::Duration(this_loop_period));

            double after_update = now();

            // now write to HW (joints + gripper + ft sensors)
            // if a parallel gripper command has been requested
            // then send the command to the gripper
            if(object_ptr->pg70_pos_command_requested_)
            {
                dumbo_hw.write(object_ptr->pg70_pos_command_);
                object_ptr->pg70_pos_command_requested_ = false;
            }

            // if no PG70 pos command, then execute
            // velocity command on arms & gripper
            else
            {
                dumbo_hw.write();
            }

            double end = now();

            g_stats.read_acc(after_read - start);
            g_stats.cm_acc(after_update - after_read);
            g_stats.write_acc(end-after_update);


            // attend service requests and publish HW status and diagnostics
            // every second
            if((end-last_published) > 1.0)
            {
                publishDiagnostics(diag_publisher);

                // attend service requests (connect, disconnect, stop, recover)
                object_ptr->attendServiceRequests(dumbo_hw);

                // publish hw status message with realtime publishers
                object_ptr->publishHWStatus(dumbo_hw);

                last_published = end;
            }

            // set the control frequency to 500 Hz / 1KHz depending
            // on whether the parallel gripper and left arm are both connected
            // both connected --> 500 Hz
            // only one of them connected --> 1KHz
//            if((!dumbo_hw.left_arm_hw->isInitialized())&&(dumbo_hw.pg70_hw->isInitialized()))
//            {
//                period = 1e+6;
//            }

//            else if((dumbo_hw.left_arm_hw->isInitialized())&&(!dumbo_hw.pg70_hw->isInitialized()))
//            {
//                period = 1e+6;
//            }

//            else
//            {
//                period = 2e+6;
//            }


            // Realtime loop should run about 500 or 1000Hz.
            // Missing timing on a control cycles usually causes a controller glitch and actuators to jerk.
            // When realtime loop misses a lot of cycles controllers will perform poorly and may cause robot to shake.
            // Halt motors if realtime loop does not run enough cycles over a given period.
            ++rt_cycle_count;
            if ((start - last_rt_monitor_time) > rt_loop_monitor_period)
            {
                // Calculate new average rt loop frequency
                double rt_loop_frequency = double(rt_cycle_count) / rt_loop_monitor_period;

                // Use last X samples of frequency when deciding whether or not to halt
                rt_loop_history.sample(rt_loop_frequency);
                double avg_rt_loop_frequency = rt_loop_history.average();
                if (avg_rt_loop_frequency < min_acceptable_rt_loop_frequency)
                {
//                    g_halt_motors = true;
                    if (!g_stats.rt_loop_not_making_timing)
                    {
                        // Only update this value if motors when this first occurs (used for diagnostics error message)
                        g_stats.halt_rt_loop_frequency = avg_rt_loop_frequency;
                    }
                    g_stats.rt_loop_not_making_timing = true;
                }
                g_stats.rt_loop_frequency = avg_rt_loop_frequency;
                rt_cycle_count = 0;
                last_rt_monitor_time = start;
            }


            // Compute end of next period
            timespecInc(tick, period);

            struct timespec before;
            clock_gettime(CLOCK_REALTIME, &before);
            if ((before.tv_sec + double(before.tv_nsec)/NSEC_PER_SECOND) > (tick.tv_sec + double(tick.tv_nsec)/NSEC_PER_SECOND))
            {
                // Total amount of time the loop took to run
                g_stats.overrun_loop_sec = (before.tv_sec + double(before.tv_nsec)/NSEC_PER_SECOND) -
                        (tick.tv_sec + double(tick.tv_nsec)/NSEC_PER_SECOND);

                // We overran, snap to next "period"
                tick.tv_sec = before.tv_sec;
                tick.tv_nsec = (before.tv_nsec / period) * period;
                timespecInc(tick, period);

                // initialize overruns
                if (g_stats.overruns == 0){
                    g_stats.last_overrun = 1000;
                    g_stats.last_severe_overrun = 1000;
                }
                // check for overruns
                if (g_stats.recent_overruns > 10)
                    g_stats.last_severe_overrun = 0;
                g_stats.last_overrun = 0;

                g_stats.overruns++;
                g_stats.recent_overruns++;
                g_stats.overrun_read = after_read - start;
                g_stats.overrun_cm = after_update - after_read;
                g_stats.overrun_write = end - after_update;
            }

            // Sleep until end of period
            clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &tick, NULL);

            // Calculate RT loop jitter
            struct timespec after;
            clock_gettime(CLOCK_REALTIME, &after);
            double jitter = (after.tv_sec - tick.tv_sec + double(after.tv_nsec-tick.tv_nsec)/NSEC_PER_SECOND);

            g_stats.jitter_acc(jitter);

        }

        // read any remaining messages in the CAN bus
        dumbo_hw.read();

        // stop all motors by writing zero vel to the joints
        dumbo_hw.left_arm_hw->writeZeroVel();
        dumbo_hw.right_arm_hw->writeZeroVel();
        dumbo_hw.pg70_hw->writeZeroVel();

        ROS_INFO("====== Disconnecting Dumbo!! ======");
        dumbo_hw.disconnect();
        object_ptr->publishHWStatus(dumbo_hw);

        diag_publisher.stop();

        ros::Duration(3.0).sleep();

        ROS_INFO("===== Exiting Dumbo HW control loop!!!!!!! ======");
        ros::shutdown();

        return (void *)rv;
    }

    inline void attendServiceRequests(dumbo_hardware_interface::DumboHW &dumbo_hw)
    {
        if(connect_dumbo_)
        {
            dumbo_hw.connect();
            connect_dumbo_ = false;
        }

        else if(disconnect_dumbo_)
        {
            dumbo_hw.disconnect();
            disconnect_dumbo_ = false;
        }

        if(stop_dumbo_)
        {
            dumbo_hw.stop();
            stop_dumbo_ = false;
        }

        else if(recover_dumbo_)
        {
            dumbo_hw.recover();
            recover_dumbo_ = false;
        }

        if(connect_left_arm_)
        {
            dumbo_hw.left_arm_hw->connect();
            connect_left_arm_ = false;
        }

        else if(disconnect_left_arm_)
        {
            dumbo_hw.left_arm_hw->disconnect();
            disconnect_left_arm_ = false;
        }

        if(connect_right_arm_)
        {
            dumbo_hw.right_arm_hw->connect();
            connect_right_arm_ = false;
        }

        else if(disconnect_right_arm_)
        {
            dumbo_hw.right_arm_hw->disconnect();
            disconnect_right_arm_ = false;
        }

        if(connect_pg70_)
        {
            dumbo_hw.pg70_hw->connect();
            connect_pg70_ = false;
        }

        else if(disconnect_pg70_)
        {
            dumbo_hw.pg70_hw->disconnect();
            disconnect_pg70_ = false;
        }
    }

    inline void publishHWStatus(dumbo_hardware_interface::DumboHW &dumbo_hw)
    {
        if(left_arm_status_publisher_->trylock())
        {
            left_arm_status_publisher_->msg_.data = dumbo_hw.left_arm_hw->isInitialized();
            left_arm_status_publisher_->unlockAndPublish();
        }

        if(right_arm_status_publisher_->trylock())
        {
            right_arm_status_publisher_->msg_.data = dumbo_hw.right_arm_hw->isInitialized();
            right_arm_status_publisher_->unlockAndPublish();
        }

        if(pg70_status_publisher_->trylock())
        {
            pg70_status_publisher_->msg_.data = dumbo_hw.pg70_hw->isInitialized();
            pg70_status_publisher_->unlockAndPublish();
        }

        if(left_arm_ft_sensor_status_publisher_->trylock())
        {
            left_arm_ft_sensor_status_publisher_->msg_.data = dumbo_hw.left_ft_sensor_hw->isInitialized();
            left_arm_ft_sensor_status_publisher_->unlockAndPublish();
        }

        if(right_arm_ft_sensor_status_publisher_->trylock())
        {
            right_arm_ft_sensor_status_publisher_->msg_.data = dumbo_hw.right_ft_sensor_hw->isInitialized();
            right_arm_ft_sensor_status_publisher_->unlockAndPublish();
        }
    }

    // gets time from monotonic clock
    static inline double now()
    {
      struct timespec n;
      clock_gettime(CLOCK_MONOTONIC, &n);
      return double(n.tv_nsec) / NSEC_PER_SECOND + n.tv_sec;
    }


    static void quitRequested(int sig)
    {
        ROS_INFO("======= Requesting shutdown of Dumbo!!! =======");
        g_quit = true;
    }

    static void timespecInc(struct timespec &tick, int nsec)
    {
      tick.tv_nsec += nsec;
      while (tick.tv_nsec >= NSEC_PER_SECOND)
      {
        tick.tv_nsec -= NSEC_PER_SECOND;
        tick.tv_sec++;
      }
    }



private:
    ros::NodeHandle nh_;

    // services for connecting/disconnecting to hw
    // as well as stopping/recovering manipulators
    ros::ServiceServer connect_dumbo_srv_server_;
    ros::ServiceServer disconnect_dumbo_srv_server_;

    ros::ServiceServer stop_dumbo_srv_server_;
    ros::ServiceServer recover_dumbo_srv_server_;

    ros::ServiceServer connect_left_arm_srv_server_;
    ros::ServiceServer disconnect_left_arm_srv_server_;

    ros::ServiceServer connect_right_arm_srv_server_;
    ros::ServiceServer disconnect_right_arm_srv_server_;

    ros::ServiceServer connect_pg70_srv_server_;
    ros::ServiceServer disconnect_pg70_srv_server_;

    bool connect_dumbo_;
    bool disconnect_dumbo_;

    bool stop_dumbo_;
    bool recover_dumbo_;

    bool connect_left_arm_;
    bool disconnect_left_arm_;

    bool connect_right_arm_;
    bool disconnect_right_arm_;

    bool connect_pg70_;
    bool disconnect_pg70_;

    // publishers for hw state (connected = True or False)
    typedef boost::scoped_ptr<realtime_tools::RealtimePublisher<std_msgs::Bool> > RTBoolPublisherPtr;
    RTBoolPublisherPtr left_arm_status_publisher_;
    RTBoolPublisherPtr right_arm_status_publisher_;

    RTBoolPublisherPtr pg70_status_publisher_;

    RTBoolPublisherPtr left_arm_ft_sensor_status_publisher_;
    RTBoolPublisherPtr right_arm_ft_sensor_status_publisher_;

    // subscriber for parallel gripper commands
    // todo: place this is as a separate action controller
    ros::Subscriber pg70_pos_command_sub_;

    bool pg70_pos_command_requested_;
    double pg70_pos_command_;


};


}

int main(int argc, char **argv)
{
    // Keep the kernel from swapping us out
    if (mlockall(MCL_CURRENT | MCL_FUTURE) < 0) {
      perror("mlockall");
      return -1;
    }

    ros::init(argc, argv, "dumbo_hw_control_loop");

    // creates the realtime hw control loop
    dumbo_control::DumboHWControlLoop dumbo_hw_control_loop;

    ros::spin();

    int rv;
    pthread_join(dumbo_hw_control_loop.controlThread, (void **)&rv);

    munlockall();

    return 0;
}
