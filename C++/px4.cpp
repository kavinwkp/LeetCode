while (ros::ok()) {
    ros::spinOnce();
    Command_Now.header.stamp = ros::Time::now();
    Command_Now.Command_ID = Command_Now.Command_ID + 1;
    Command_Now.source = node_name;
    Command_Now.Mode = prometheus_msgs::ControlCommand::Move;
    Command_Now.Reference_State.Move_mode = prometheus_msgs::PositionReference::XY_VEL_Z_POS;
    Command_Now.Reference_State.Move_frame = prometheus_msgs::PositionReference::MIX_FRAME;
    Command_Now.Reference_State.velocity_ref[0] = following_vel;
    Command_Now.Reference_State.velocity_ref[1] = following_kp * error_body_y;
    Command_Now.Reference_State.velocity_ref[2] = start_point_z;
    Command_Now.Reference_State.yaw_ref = yaw_sp;
    command_pub.publish(Command_Now);
}