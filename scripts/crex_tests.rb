require 'orocos'
require 'readline'
include Orocos

Orocos.initialize

Orocos.run 'crex_tests',
    'trajectory_generation::Task' => "crexLegInterpolator",
    'behavior_graph::MotionControlTask' => "motion_control",
    'joint_dispatcher::Task' => "joint_dispatcher", 
    :output=>nil  do

    mars = TaskContext.get 'mars'
    xsens = TaskContext.get 'xsens'
    velodyne = TaskContext.get 'velodyne'
    mars.apply_conf_file("mars::Task.yml", ["default", "crex_in_dlr_scene"])
    mars.configure
    mars.start
    xsens.apply_conf_file("mars::IMU.yml", ["default"])
    xsens.configure
    velodyne.apply_conf_file("mars::RotatingLaserRangeFinder.yml", ["default"])
    velodyne.configure
    # Connections
    xsens.start
    velodyne.start
  
    Readline::readline("Press ENTER to quit") do
    end

end 

