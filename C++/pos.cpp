position_target_sub = command_nh.subscribe<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/target_local", 10, &command_to_mavros::pos_target_cb, this);
attitude_target_sub = command_nh.subscribe<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/target_attitude", 10, &command_to_mavros::att_target_cb, this);
setpoint_raw_local_pub = command_nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/target_local", 10);
setpoint_raw_attitude_pub = command_nh.advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude", 10);
actuator_setpoint_pub = command_nh.advertise<mavros_msgs::ActuatorControl>("/mavros/actuator_control", 10);
arming_client = command_nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
set_mode_client = command_nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
void pos_target_cb(const mavros_msgs::PositionTarget::ConstPtr &msg) {
    pos_drone_fcu_target = Eigen::Vector3d(msg->position.x, msg->position.y, msg->position.z);
    vel_drone_fcu_target = Eigen::Vector3d(msg->velocity.x, msg->velocity.y, msg->velocity.z);
    acc_drone_fcu_target = Eigen::Vector3d(msg->acceleration_or_force.x, msg->acceleration_or_force.y, msg->acceleration_or_force.z);

}

void command_to_mavros::takeoff() {
    mavros_msgs::PositionTarget pos_setpoint;
    pos_setpoint.type_mask = 0x1000;
    setpoint_raw_local_pub.publish(pos_setpoint);
}

void command_to_mavros::send_pos_setpoint(const Eigen::Vector3d& pos_sp, float yaw_sp) {
    mavros_msgs::PositionTarget pos_setpoint;
    pos_setpoint.type_mask = 0b100111111000;
    pos_setpoint.coordinate_frame = 1;
    pos_setpoint.position.x = pos_sp[0];
    pos_setpoint.position.y = pos_sp[1];
    pos_setpoint.position.z = pos_sp[2];
    pos_setpoint.yaw = yaw_sp;
    setpoint_raw_local_pub.publish(pos_setpoint);
}

void command_to_mavros::send_attitude_setpoint(const prometheus_msgs::AttitudeReference& _AttitudeReference) {
    movros_msgs::AttitudeTarget att_setpoint;
    att_setpoint.type_mask = 0b00000111;
    att_setpoint.orientation.x = _AttitudeReference.desired_att_q.x;
    att_setpoint.orientation.y = _AttitudeReference.desired_att_q.y;
    att_setpoint.orientation.z = _AttitudeReference.desired_att_q.z;
    att_setpoint.orientation.w = _AttitudeReference.desired_att_q.w;
    att_setpoint.thrust = _AttitudeReference.desired_throttle;
    
}