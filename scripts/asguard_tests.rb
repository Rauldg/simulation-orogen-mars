require 'orocos'
require 'readline'
include Orocos

Orocos.initialize

Orocos.run 'asguard_tests', 
    "skid4_control::SimpleController" => "simple_controller",  
    "joint_dispatcher::Task" => "joint_dispatcher" do
    
    mars = TaskContext.get 'mars'
    mars_actuators = TaskContext.get 'mars_actuators'
    sysmon = TaskContext.get 'sysmon'
    joint_dispatcher = TaskContext.get 'joint_dispatcher'
    xsens = TaskContext.get 'xsens'
    velodyne = TaskContext.get 'velodyne'
    simple_controller = Orocos.name_service.get 'simple_controller'
    mars.apply_conf_file("mars::Task.yml", ["default", "asguard_in_dlr_scene"])
    mars.configure
    mars.start
    mars_actuators.apply_conf_file("mars::Joints.yml", ["base"]) 
    mars_actuators.configure
    sysmon.apply_conf_file("mars::Joints.yml", ["sysmon"])
    sysmon.configure
    joint_dispatcher.apply_conf_file("joint_dispatcher::Task.yml", ["default"])
    joint_dispatcher.configure
    simple_controller.apply_conf_file("skid4_control::SimpleController.yml", ["default"])
    simple_controller.configure
    xsens.apply_conf_file("mars::IMU.yml", ["default"])
    xsens.configure
    velodyne.apply_conf_file("mars::RotatingLaserRangeFinder.yml", ["default"])
    velodyne.configure
    # Connections
    joint_dispatcher.command_out.connect_to(mars_actuators.command, :type=>:buffer, :size=>100)
    mars_actuators.status_samples.connect_to(joint_dispatcher.status_samples_in, :type=>:buffer, :size=>100)
    sysmon.status_samples.connect_to(joint_dispatcher.body_joint_in, :type=>:buffer, :size=>100)
    simple_controller.command.connect_to(joint_dispatcher.command, :type=>:buffer, :size=>100)
    joint_dispatcher.status_samples.connect_to(simple_controller.status_samples, :type=>:buffer, :size=>100)
    mars_actuators.start
    sysmon.start
    joint_dispatcher.start
    simple_controller.start
    xsens.start
    velodyne.start
  
    # To test just give a translation command
    mc_writer = simple_controller.motion_command.writer()
    mc_cmd = mc_writer.new_sample()
    mc_cmd.translation = 0.05;
    mc_cmd.rotation = 0.0;
    mc_writer.write(mc_cmd)

    Readline::readline("Press ENTER to quit") do
    end

end 
