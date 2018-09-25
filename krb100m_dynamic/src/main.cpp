#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <krb100m_msg/krb100m_msgConfig.h>
#include <krb100m_msg/Dynamic.h>

ros::Publisher pub;
ros::Subscriber sub;
boost::recursive_mutex config_mutex; //I am not sure whether I need it 
dynamic_reconfigure::Server<krb100m_msg::krb100m_msgConfig> *parameter_server;
krb100m_msg::krb100m_msgConfig rt_config;

void dynamic_out_callback(const krb100m_msg::Dynamic::ConstPtr &msg)
{
	bool need_update = false;
	if (msg->kp != rt_config.kp)
	{
		rt_config.kp = msg->kp;
		need_update = true;
	}
	if (msg->ki != rt_config.ki)
	{
		rt_config.ki = msg->ki;
		need_update = true;
	}
	if (msg->kd != rt_config.kd)
	{
		rt_config.kd = msg->kd;
		need_update = true;
	}
	if (msg->left_pwm != rt_config.left_pwm)
	{
		rt_config.left_pwm = msg->left_pwm;
		need_update = true;
	}
	if (msg->right_pwm != rt_config.right_pwm)
	{
		rt_config.right_pwm = msg->right_pwm;
		need_update = true;
	}
	if (msg->linear_x_vel != rt_config.linear_x_vel)
	{
		rt_config.linear_x_vel = msg->linear_x_vel;
		need_update = true;
	}
	if (msg->angular_z_vel != rt_config.angular_z_vel)
	{
		rt_config.angular_z_vel = msg->angular_z_vel;
		need_update = true;
	}
	if (msg->dbgpid != rt_config.dbgpid)
	{
		rt_config.dbgpid = msg->dbgpid;
		need_update = true;
	}
	if (need_update) 
	{
		boost::recursive_mutex::scoped_lock lock(config_mutex); // I am not sure I need it
		parameter_server->updateConfig(rt_config);
		lock.unlock();
	}
	//ROS_INFO("Dynamic msg: %d %d %d %d %d %d", 
	//msg->kp, 
	//msg->ki, 
	//msg->kd, 
	//msg->left_pwm, 
	//msg->right_pwm, 
	//msg->dbgpid);
}

void callback(krb100m_msg::krb100m_msgConfig &config, uint32_t level) 
{
	krb100m_msg::Dynamic dynamic;
	ROS_INFO("Reconfigure Request: %d %d %d %d %d %d %d %d", 
			config.kp, 
			config.ki, 
			config.kd, 
			config.left_pwm, 
			config.right_pwm, 
			config.linear_x_vel, 
			config.angular_z_vel, 
			config.dbgpid);
	dynamic.right_pwm = config.right_pwm;
	dynamic.kp = config.kp;
	dynamic.ki = config.ki;
	dynamic.kd = config.kd;
	dynamic.left_pwm = config.left_pwm;
	dynamic.right_pwm = config.right_pwm;
	dynamic.linear_x_vel = config.linear_x_vel;
	dynamic.angular_z_vel = config.angular_z_vel;
	dynamic.dbgpid = config.dbgpid;
	pub.publish(dynamic);
}

int main(int argc, char **argv) 
{
	ros::init(argc, argv, "krb100m_dynamic_configure");
	ros::NodeHandle nh;
	sub = nh.subscribe("dynamic_out", 100, dynamic_out_callback);
	pub = nh.advertise<krb100m_msg::Dynamic>("dynamic_in", 1000);

	dynamic_reconfigure::Server<krb100m_msg::krb100m_msgConfig> server;
	dynamic_reconfigure::Server<krb100m_msg::krb100m_msgConfig>::CallbackType f;

	f = boost::bind(&callback, _1, _2);
	server.setCallback(f);
	parameter_server = &server;

	ROS_INFO("Spinning node");
	ros::spin();
	return 0;
}

