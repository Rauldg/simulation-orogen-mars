require 'orocos'
require 'readline'
include Orocos

Orocos.initialize

#Orocos.run 'just_mars', 'mars::RotatingLaserRangeFinder' => 'velodyne', :gdb=>true do
#Orocos.run 'asguard_tests', :gdb=>true do
Orocos.run 'asguard_tests' do

    mars = TaskContext.get 'mars'
    velodyne = TaskContext.get 'velodyne'

    mars.apply_conf_file("mars::Task.yml", ["default", "asguard_in_dlr_scene"])
    #mars.apply_conf_file("mars::Task.yml", ["default", "asguard_in_smurfs_scene"])
    mars.configure
    mars.start
    velodyne.apply_conf_file("mars::RotatingLaserRangeFinder.yml", ["default"])
    velodyne.configure
    velodyne.start

    Readline::readline("Press ENTER to quit") do
    end
end
