#include <roboticarm_hardware_interface/roboticarm_hardware_interface.h>


roboticarm::roboticarm(ros::NodeHandle& nh) : nh_(nh) {
    init();
    controller_manager_.reset(new controller_manager::ControllerManager(this, nh_));
    loop_hz_=4;
    ros::Duration update_freq = ros::Duration(1.0/loop_hz_);
    
    pub = nh_.advertise<rospy_tutorials::Floats>("/write_joint_values",10);
	client = nh_.serviceClient<roboticarm_hardware_interface::Floats_array>("/read_joint_values");

    my_control_loop_ = nh_.createTimer(update_freq, &roboticarm::update, this);
}

roboticarm::~roboticarm() {
}

void roboticarm::init() {
    num_joints_=5;
	joint_names_[0]="j0";	
	joint_names_[1]="j1";
	joint_names_[2]="j2";
	joint_names_[3]="j3";
	joint_names_[4]="j4";
        
// for (int i = 0; i < num_joints_; ++i) {

//          // Create joint state interface
//         hardware_interface::JointStateHandle jointStateHandle(joint_names_[i], &joint_position_[i], &joint_velocity_[i], &joint_effort_[i]);
//         joint_state_interface_.registerHandle(jointStateHandle);

//         // Create position joint interface
//         hardware_interface::JointHandle jointPositionHandle(jointStateHandle, &joint_position_command_[i]);
        
//         position_joint_interface_.registerHandle(jointPositionHandle);
   
//     }
//     registerInterface(&joint_state_interface_);
    // registerInterface(&position_joint_interface_);   

    for (int i = 0; i < num_joints_; ++i) {

         // Create joint state interface
        hardware_interface::JointStateHandle jointStateHandle(joint_names_[i], &joint_position_[i], &joint_velocity_[i], &joint_effort_[i]);
        joint_state_interface_.registerHandle(jointStateHandle);

        // Create position joint interface
        hardware_interface::JointHandle jointPositionHandle(jointStateHandle, &joint_position_command_[i]);
        
        position_joint_interface_.registerHandle(jointPositionHandle);
   
    }
    registerInterface(&joint_state_interface_);
    registerInterface(&position_joint_interface_);
}

//This is the control loop
void roboticarm::update(const ros::TimerEvent& e) {
    elapsed_time_ = ros::Duration(e.current_real - e.last_real);
    read();
    controller_manager_->update(ros::Time::now(), elapsed_time_);
    write(elapsed_time_);
}

void roboticarm::read() {
    
    joint_current.request.req=1.0;
	
	if(client.call(joint_current))
	{
	    joint_position_[0] = angles::from_degrees(joint_current.response.res[0]);
        joint_position_[1] = angles::from_degrees(joint_current.response.res[1]);
        joint_position_[2] = angles::from_degrees(joint_current.response.res[2]);
        joint_position_[3] = angles::from_degrees(joint_current.response.res[3]);
        joint_position_[4] = angles::from_degrees(joint_current.response.res[4]);
	    ROS_INFO("Current Pos of j0: %.2f",joint_position_[0]);
	}else{
	    joint_position_[0] = 0;
        joint_position_[1] = 0;
        joint_position_[2] = 0;
        joint_position_[3] = 0;
        joint_position_[4] = 0;
    }
}

void roboticarm::write(ros::Duration elapsed_time) {
  // Safety
    joints_val.data.clear();
	joints_val.data.push_back((angles::to_degrees(joint_position_command_[0])));
	joints_val.data.push_back((angles::to_degrees(joint_position_command_[1])));
	joints_val.data.push_back((angles::to_degrees(joint_position_command_[2])));
    joints_val.data.push_back((angles::to_degrees(joint_position_command_[3])));
    joints_val.data.push_back((angles::to_degrees(joint_position_command_[4])));
	//ROS_INFO("Publishing j1: %.2f, j2: %.2f, j3: %.2f",joints_pub.data[0],joints_pub.data[1],joints_pub.data[2]);
	pub.publish(joints_val);
  // Write the protocol (I2C/CAN/ros_serial/ros_industrial)used to send the commands to the robot's actuators.
  // the output commands need to send are joint_effort_command_[0] for JointA, joint_effort_command_[1] for JointB and 
  //joint_position_command_ for JointC.
}

int main(int argc, char** argv) {
    //Initialze the ROS node.
    ros::init(argc, argv, "robot_hardware_interface");
    ros::NodeHandle nh;
    ros::MultiThreadedSpinner spinner(2); 
    //Separate Sinner thread for the Non-Real time callbacks such as service callbacks to load controllers
    // ros::AsyncSpinner spinner(1);
    spinner.spin();
    // Create the object of the robot hardware_interface class and spin the thread. 
    roboticarm roboticarm(nh);
    // ros::spin();
    
    return 0;
}

