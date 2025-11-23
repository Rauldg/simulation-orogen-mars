#!/usr/bin/env ruby

require 'orocos'
require 'readline'
require 'yaml'
include Orocos

# Configuration file with environment definitions
ENVIRONMENTS_CONFIG = File.join(File.dirname(__FILE__), 'environments_and_goals.yml')

Orocos.initialize

# Check that the script is called with the right number of arguments
if ARGV.size != 1
    puts "Usage: load_initial_pose.rb <internal_simulator_task_name>"
    puts ""
    puts "Arguments:"
    puts "  internal_simulator_task_name  - Name of the running mars::InternalSimulatorTask"
    puts ""
    puts "Examples:"
    puts "  ./load_initial_pose.rb asguard_v4_internal_sim_refactored"
    puts ""
    puts "Process:"
    puts "  1. User selects an environment"
    puts "  2. User selects a pose from that environment"
    puts "  3. Script sends the pose to the internal simulator task"
    exit 1
end

is_task_name = ARGV[0].to_s

# Get the internal simulator task
begin
    is_task = Orocos.get(is_task_name)
rescue Orocos::NotFound
    puts "ERROR: Task '#{is_task_name}' not found!"
    puts "Make sure the task is running."
    exit 1
end

# Load environment definitions from YAML config file
unless File.exist?(ENVIRONMENTS_CONFIG)
    puts "ERROR: Configuration file not found: #{ENVIRONMENTS_CONFIG}"
    puts ""
    puts "Please create the configuration file with environment definitions."
    exit 1
end

config = YAML.load_file(ENVIRONMENTS_CONFIG)
unless config && config['environments'] && config['environments'].is_a?(Array)
    puts "ERROR: Invalid configuration file format!"
    puts "Expected YAML file with 'environments' array."
    exit 1
end

environments = config['environments']

if environments.empty?
    puts "ERROR: No environments defined in configuration file!"
    puts "Please add environment definitions to: #{ENVIRONMENTS_CONFIG}"
    exit 1
end

# Main loop - allows user to go back and reselect environment
loop do
    # Display available environments to the user
    puts ""
    puts "=" * 80
    puts "Select Environment for Initial Pose"
    puts "=" * 80
    puts ""

    environments.each_with_index do |env, index|
        puts "  [#{index + 1}] #{env['name']}"
        puts "      Description: #{env['description']}" if env['description']
        puts ""
    end

    puts "=" * 80

    # Prompt user to select an environment
    print "\nSelect environment number [1-#{environments.size}] (or 'q' to quit): "
    selection = Readline.readline.strip

    if selection.downcase == 'q'
        puts "Cancelled."
        exit 0
    end

    # Validate selection
    selected_env_index = selection.to_i - 1
    if selected_env_index < 0 || selected_env_index >= environments.size
        puts "ERROR: Invalid selection!"
        sleep(1)
        next
    end

    selected_env = environments[selected_env_index]
    
    # Check if environment has poses defined
    unless selected_env['poses'] && selected_env['poses'].is_a?(Hash) && !selected_env['poses'].empty?
        puts ""
        puts "ERROR: No poses defined for environment '#{selected_env['name']}'!"
        puts "Please add pose definitions to the configuration file."
        sleep(2)
        next
    end

    # Inner loop - pose selection with option to go back
    loop do
        # Display available poses for the selected environment
        puts ""
        puts "=" * 80
        puts "Environment: #{selected_env['name']}"
        puts "Select Initial Pose"
        puts "=" * 80
        puts ""

        pose_list = selected_env['poses'].keys.sort
        pose_list.each_with_index do |pose_name, index|
            pose_values = selected_env['poses'][pose_name]
            x, y, z, yaw = pose_values
            puts "  [#{index + 1}] #{pose_name}"
            puts "      Position: x=#{x}, y=#{y}, z=#{z}"
            puts "      Yaw: #{yaw} rad (#{(yaw * 180 / Math::PI).round(2)} deg)"
            puts ""
        end

        puts "=" * 80

        # Prompt user to select a pose
        print "\nSelect pose number [1-#{pose_list.size}], 'b' to go back, or 'q' to quit: "
        pose_selection = Readline.readline.strip

        if pose_selection.downcase == 'q'
            puts "Cancelled."
            exit 0
        end

        if pose_selection.downcase == 'b'
            puts "Going back to environment selection..."
            break  # Break inner loop to go back to environment selection
        end

        # Validate pose selection
        selected_pose_index = pose_selection.to_i - 1
        if selected_pose_index < 0 || selected_pose_index >= pose_list.size
            puts "ERROR: Invalid selection!"
            sleep(1)
            next
        end

        selected_pose_name = pose_list[selected_pose_index]
        selected_pose_values = selected_env['poses'][selected_pose_name]

        # Extract pose values
        x, y, z, yaw = selected_pose_values

        puts ""
        puts "Loading initial pose: #{selected_pose_name}"
        puts "-" * 80
        puts "Environment: #{selected_env['name']}"
        puts "Pose: #{selected_pose_name}"
        puts "  Position: [#{x}, #{y}, #{z}]"
        puts "  Yaw: #{yaw} rad (#{(yaw * 180 / Math::PI).round(2)} deg)"
        puts ""

        # Create RigidBodyState message
        pose = Types.base.samples.RigidBodyState.new
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        pose.orientation = Eigen::Quaternion.from_angle_axis(yaw, Eigen::Vector3.UnitZ)
        pose.targetFrame = "body"  # Default target frame, adjust if needed

        # Write pose to internal simulator task
        begin
            pose_writer = is_task.start_pose.writer
            pose_writer.write(pose)
            puts "SUCCESS: Initial pose sent to internal simulator task!"
            puts ""
            
            # Show task state
            puts "Internal simulator task state: #{is_task.state}"
            puts ""
            
        rescue Exception => e
            puts "ERROR: Failed to write pose to task!"
            puts "Error: #{e.message}"
            exit 1
        end

        puts "=" * 80
        puts "Initial pose '#{selected_pose_name}' loaded successfully!"
        puts "The internal simulator should now have the start pose."
        puts "=" * 80
        puts ""
        
        exit 0  # Successfully completed
    end
end
