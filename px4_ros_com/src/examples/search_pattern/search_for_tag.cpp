/****************************************************************************
 *
 * Copyright 2020 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @brief Offboard control example
 * @file offboard_control.cpp
 * @addtogroup examples
 * @author Mickey Cowden <info@cowden.tech>
 * @author Nuno Marques <nuno.marques@dronesolutions.io>
 */

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>
#include <math.h>
#include <chrono>
#include <iostream>
#include <string.h>

#define FLIGHT_ALTITUDE -0.5f		// flight altitude (m)
#define	HOVER_ALT	-1.5f		// hover altitude (m)
#define RATE            20 		// loop rate (hz)
#define SEARCH_WIDTH	1.5		// search area width (m)
#define SEARCH_LENGTH	2		// search area length (m)
#define LIN_VEL 	0.1		// linear velocity (m/s)
#define ANG_RATE	5		// time to complete a full circle (s)
#define PATH_DIST	0.5		// distance between passes along length (m)
#define CYCLE_S		10000		// time to complete a search
#define STEPS		(CYCLE_S*RATE)	// iterations
#define ALT_VEL		0.25		// altitude velocity (m/s)
#define X0		0		// initial x-coordinate (m)
#define Y0		0		// initial y-coordinate (m)
//#define TAG_ID		'0'		// april tag ID to initiate search area	
//#define CLASS		'backpack'	// object class to search for
// hard-coded	
	
#define PI  3.14159265358979323846264338327950

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

class OffboardControl : public rclcpp::Node
{
public:
	OffboardControl() : Node("offboard_control")
	{		
		offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
		trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
		vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);
		
        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

        vehicle_command_listener_ = this->create_subscription<VehicleControlMode>("/fmu/out/vehicle_control_mode", qos, [this](const px4_msgs::msg::VehicleControlMode::UniquePtr msg){
            c_mode = *msg;
        });
        obj_class_listener_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/object/pose_camera", qos, [this](const geometry_msgs::msg::PoseStamped::UniquePtr objmsg){
            obj_data = *objmsg;
        });
        tag_id_listener_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/tag_detections/tagpose", qos, [this](const geometry_msgs::msg::PoseStamped::UniquePtr tagmsg){
            tag_data = *tagmsg;
       });
       tag_pose_listener_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/tag_detections/tagpose_inertial", qos, [this](const geometry_msgs::msg::PoseStamped::UniquePtr tagposmsg){
       	    tag_pos_data = *tagposmsg;
       });

		offboard_setpoint_counter_ = 0;
		pub_flag_ = 0;
		// 0 = nominal trajectory execution
		// 1 = return to start



		InitPath();

		auto timer_callback = [this]() -> void {


			// offboard_control_mode needs to be paired with trajectory_setpoint

			
            double intpart;
            //printf("modf:%7.3f", );
            if(modf(((double)offboard_setpoint_counter_)/2, &intpart)==0.0){
                publish_offboard_control_mode();
                //printf("published mode msg");

            }
            //print(modf(offboard_setpoint_counter_/5, &intpart))
			publish_trajectory_setpoint();


		};
		timer_ = this->create_wall_timer(50ms, timer_callback);
	}


private:
	rclcpp::TimerBase::SharedPtr timer_;

	rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
	rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;
    	rclcpp::Subscription<VehicleControlMode>::SharedPtr vehicle_command_listener_;
    	rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr obj_class_listener_;
    	rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr tag_id_listener_;
    	rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr tag_pose_listener_;

	std::atomic<uint64_t> timestamp_;   //!< common synced timestamped
    

	uint64_t offboard_setpoint_counter_;   //!< counter for the number of setpoints sent
	uint64_t pub_flag_; // flag for publisher
        
    	TrajectorySetpoint path[STEPS];
    	TrajectorySetpoint obj_loc;
    	TrajectorySetpoint tag_loc;
    	VehicleControlMode c_mode;
    	geometry_msgs::msg::PoseStamped obj_data;
    	geometry_msgs::msg::PoseStamped tag_data;
    	geometry_msgs::msg::PoseStamped tag_pos_data;
    	
	double tag_pos_inert[2] = {X0,Y0};
	double obj_pos_inert[2] = {0,0};
    	        
	void publish_offboard_control_mode();
	void publish_trajectory_setpoint();
	//void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);
	void InitPath();
};

/**
 * @brief Publish the offboard control mode.
 *        For this example, only position and altitude controls are active.
 */
void OffboardControl::publish_offboard_control_mode()
{
	OffboardControlMode msg{};
	msg.position = true;
	msg.velocity = true;
	msg.acceleration = true;
	msg.attitude = true;
	msg.body_rate = true;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	offboard_control_mode_publisher_->publish(msg);

}

/**
 * @brief Publish a trajectory setpoint
 *        For this example, it sends a trajectory setpoint to make the
 *        vehicle hover at 5 meters with a yaw angle of 180 degrees.
 */
void OffboardControl::publish_trajectory_setpoint()
{

	const std::string tag_des = "0";
	const std::string obj_des = "backpack";
	const std::string obj_des2 = "rand2";
	const std::string obj_des3 = "rand3";
	std::string tag_ID; // AprilTag ID detected
	std::string class_det; // object class detected
	
	
        // Increment the setpoint counter
        offboard_setpoint_counter_++;
        //if(offboard_setpoint_counter_>=STEPS){
	//	offboard_setpoint_counter_ = 0;
	//}
	path[offboard_setpoint_counter_].timestamp = this->get_clock()->now().nanoseconds() / 1000;
	if(c_mode.flag_control_offboard_enabled==1 && offboard_setpoint_counter_<=STEPS){
		if(offboard_setpoint_counter_<=250) {
			trajectory_setpoint_publisher_->publish(path[offboard_setpoint_counter_]);
			//printf("i:%ld x:%7.3f y:%7.3f vx:%7.3f vy:%7.3f yaw:%7.1f\n",offboard_setpoint_counter_, path[offboard_setpoint_counter_].position[0], path[offboard_setpoint_counter_].position[1], path[offboard_setpoint_counter_].velocity[0], path[offboard_setpoint_counter_].velocity[1], path[offboard_setpoint_counter_].yaw*180.0f/PI);
			tag_ID = tag_data.header.frame_id;
			printf("Tag ID: %s    x: %7.3f    y: %7.3f\n", tag_ID.c_str(), tag_pos_data.pose.position.x, tag_pos_data.pose.position.y);
			tag_pos_inert[0] = tag_pos_data.pose.position.x;
			tag_pos_inert[1] = tag_pos_data.pose.position.y;
			//if(strcmp(tag_ID.c_str(),tag_des.c_str())!=0 && offboard_setpoint_counter_==250) {
			//	offboard_setpoint_counter_ = 0;
			//}
			if (strcmp(tag_ID.c_str(),tag_des.c_str())==0) {
				printf("TAG %s FOUND!\n",tag_ID.c_str());
			}	
				
		}
		else if (offboard_setpoint_counter_>250) {
			class_det = obj_data.header.frame_id;
			if(strcmp(class_det.c_str(),obj_des.c_str())!=0 && strcmp(class_det.c_str(),obj_des2.c_str())!=0 && strcmp(class_det.c_str(),obj_des3.c_str())!=0) {
				trajectory_setpoint_publisher_->publish(path[offboard_setpoint_counter_]);
			}
			if(strcmp(class_det.c_str(),obj_des.c_str())==0 || strcmp(class_det.c_str(),obj_des2.c_str())==0 || strcmp(class_det.c_str(),obj_des3.c_str())==0) {
				pub_flag_ = 1;
			}
				//printf("i:%ld x:%7.3f y:%7.3f vx:%7.3f vy:%7.3f yaw:%7.1f\n",offboard_setpoint_counter_, path[offboard_setpoint_counter_].position[0], path[offboard_setpoint_counter_].position[1], path[offboard_setpoint_counter_].velocity[0], path[offboard_setpoint_counter_].velocity[1], path[offboard_setpoint_counter_].yaw*180.0f/PI);
				//printf("Object: %s     x: %7.3f     y: %7.3f\n", class_det.c_str(), obj_data.pose.position.x, obj_data.pose.position.y);
			printf("OBJECT OF TYPE %s FOUND!\n", class_det.c_str());
			
			//else {
			//	obj_loc.position[0] = obj_data.pose.position.x;
			//	obj_loc.position[1] = obj_data.pose.position.y;
			//	obj_loc.position[2] = HOVER_ALT;
			//	trajectory_setpoint_publisher_->publish(obj_loc);
			//	offboard_setpoint_counter_ = STEPS;
			//	printf("FOUND! Object: %s     x: %7.3f     y: %7.3f\n", class_det.c_str(), obj_data.pose.position.x, obj_data.pose.position.y);
			//}
		}
		if(path[offboard_setpoint_counter_].position[1]>=SEARCH_LENGTH || pub_flag_==1) {
			tag_loc.position[0] = tag_pos_inert[0];
			tag_loc.position[1] = tag_pos_inert[1];
			tag_loc.position[2] = HOVER_ALT;
			offboard_setpoint_counter_ = STEPS;
			printf("RETURN TO START! x: %7.3f    y: %7.3f\n", tag_loc.position[0], tag_loc.position[1]);
		}
	}
	else{
		offboard_setpoint_counter_ = 0;
	}
}

void OffboardControl::InitPath()
{
    int i;
    const double loop_rate = RATE;
    const double width = SEARCH_WIDTH;
    const double length = SEARCH_LENGTH;
    const double linvel = LIN_VEL;
    const double angrate = ANG_RATE;
    const double path_dist = PATH_DIST;
    const double altvel = ALT_VEL;
    const double x0 = X0;
    const double y0 = Y0;
    
    const double dt = 1.0/loop_rate;
    const double dadt_cw = (2.0*PI)/angrate;
    const double dadt_ccw = -(2.0*PI)/angrate;
    const double rad = path_dist/2.0;
    
    double pos[2] = {x0, y0}; // holds the position within the search area
    double t = 0;
    
    // assumes length is larger than width
    // assumes the drone begins the search at the lower left vertex of the rectangular search area
    // assumes the drone's heading is aligned with the width of the search area
    // things to keep track of:
    	// distance traveled along width 
    	// distance traveled along length
    // x-distance is along width
    // y-distance is along length
    // need to determine number of paths
    // parts of the pattern:
    	// move along +width
    	// turn right
    	// move along -width
    	// turn left
    	// stop at end of +length
	
    int flag = -1;
	
    for(i=0;i<STEPS;i++) {
	if (flag == -1 || i<=250) {
		path[i].position[0] = i*pos[0]/250;
		path[i].position[1] = i*pos[1]/250;
		path[i].position[2] = FLIGHT_ALTITUDE;
		
		path[i].velocity[2] = 0;
		path[i].acceleration[2] = 0;
		
		path[i].yaw = 0;
		
		pos[0] = path[i].position[0];
		pos[1] = path[i].position[1];
		
		flag = 0;
	}
	else {
		if (flag==0) {
			// moving in positive width direction
			// needs to continue straight along +width
			pos[0] = path[i-1].position[0];
			pos[1] = path[i-1].position[1];
			
			path[i].position[0] = pos[0]+linvel*dt;
			path[i].position[1] = pos[1];
			path[i].position[2] = FLIGHT_ALTITUDE;
			
			path[i].velocity[2] = 0;
			path[i].acceleration[2] = 0;
			
			path[i].yaw = 0;
			
			pos[0] = path[i].position[0];
			pos[1] = path[i].position[1];
			
			if (pos[0]>=(width+x0)) {
				flag = 1; // turn right
			}
		}
		else if (flag==1) {
			// needs to turn right
			
			if (pos[0]==path[i-1].position[0]) {
				t = 0.01;
			}
			else {
				t = t+0.01;
			}
			
			double a = -(PI/2.0)+t*(2.0*PI/angrate);
			double c = cos(a);
			double s = sin(a);
			
			path[i].position[0] = rad*c+pos[0];
			path[i].position[1] = rad*s+pos[1]+rad;
			path[i].position[2] = FLIGHT_ALTITUDE;
			
			path[i].velocity[0] = -dadt_cw*rad*s;
			path[i].velocity[1] = dadt_cw*rad*c;
			path[i].velocity[2] = 0;
			
			path[i].acceleration[0] = -dadt_cw*dadt_cw*rad*c;
			path[i].acceleration[1] = -dadt_cw*dadt_cw*rad*s;
			path[i].acceleration[2] = 0;
			
			path[i].yaw = -atan2(-path[i].velocity[1], path[i].velocity[0]);
			
			if (abs(180.0-abs(path[i].yaw)*180.0/PI) <= 0.05) {
				flag = 2; // go negative width
			}
			else if (path[i].position[1]>=(length+y0)) {
				flag = 4;
			}			
		}
		else if (flag==2) {
			// moving in negative width direction
			// needs to continue straight along -width
			pos[0] = path[i-1].position[0];
			pos[1] = path[i-1].position[1];
			
			path[i].position[0] = pos[0]-linvel*dt;
			path[i].position[1] = pos[1];
			path[i].position[2] = FLIGHT_ALTITUDE;
			
			path[i].velocity[2] = 0;
			path[i].acceleration[2] = 0;
			
			path[i].yaw = PI;
			
			pos[0] = path[i].position[0];
			pos[1] = path[i].position[1];
			
			if (pos[0]<=x0) {
				flag = 3; // turn left
			}
		}
		else if (flag==3) {
			// needs to turn left
			
			if (pos[0]==path[i-1].position[0]) {
				t = 0.01;
			}
			else {
				t = t+0.01;
			}
			
			double a = -(3.0*PI/2.0)+t*(2.0*PI/angrate);
			double c = cos(-a);
			double s = sin(-a);
			
			path[i].position[0] = rad*c+pos[0];
			path[i].position[1] = rad*s+pos[1]+rad;
			path[i].position[2] = FLIGHT_ALTITUDE;
			
			path[i].velocity[0] = -dadt_ccw*rad*s;
			path[i].velocity[1] = dadt_ccw*rad*c;
			path[i].velocity[2] = 0;
			
			path[i].acceleration[0] = -dadt_ccw*dadt_ccw*rad*c;
			path[i].acceleration[1] = -dadt_ccw*dadt_ccw*rad*s;
			path[i].acceleration[2] = 0;
			
			path[i].yaw = -atan2(-path[i].velocity[1], path[i].velocity[0]);
			
			if ((abs(0-abs(path[i].yaw)*180.0/PI) <= 0.05) ) {
				flag = 0; // go positive width
			}
			else if (path[i].position[1]>=(length+y0)) {
				flag = 4;
			}
		}
		else if (flag==4) {
			// end of search path, drone should hover low
			pos[0] = path[i-1].position[0];
			pos[1] = path[i-1].position[1];
			
			path[i].position[0] = pos[0];
			path[i].position[1] = pos[1];
			path[i].position[2] = HOVER_ALT;
			
			path[i].yaw = path[i-1].yaw;
			
			pos[0] = path[i].position[0];
			pos[1] = path[i].position[1];
		}
	}
    }
}

/**
 * @brief Publish vehicle commands
 * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
 * @param param1    Command parameter 1
 * @param param2    Command parameter 2
 */
//void OffboardControl::publish_vehicle_command(uint16_t command, float param1, float param2)
//{
//	VehicleCommand msg{};
//	msg.param1 = param1;
//	msg.param2 = param2;
//	msg.command = command;
//	msg.target_system = 1;
//	msg.target_component = 1;
//	msg.source_system = 1;
//	msg.source_component = 1;
//	msg.from_external = true;
//	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
//	vehicle_command_publisher_->publish(msg);
//}

int main(int argc, char *argv[])
{
	std::cout << "Starting offboard control node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<OffboardControl>());

	rclcpp::shutdown();
	return 0;
}
