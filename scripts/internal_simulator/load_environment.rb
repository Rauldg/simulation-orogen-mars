#!/usr/bin/env ruby

require 'orocos'
require 'readline'
require 'yaml'
include Orocos

# Configuration file with environment definitions
ENVIRONMENTS_CONFIG = File.join(File.dirname(__FILE__), 'environments_and_goals.yml')

Orocos.initialize

# Check that the script is called with the right number of arguments
if ARGV.size != 2
    puts "Usage: load_environment.rb <internal_simulator_task_name> <map_loader_task_name>"
    puts ""
    puts "Arguments:"
    puts "  internal_simulator_task_name  - Name of the running mars::InternalSimulatorTask"
    puts "  map_loader_task_name          - Name of the map loader task to generate MLS from environment model"
    puts ""
    puts "Examples:"
    puts "  ./load_environment.rb asguard_v4_internal_sim_refactored asguard_v4_mls_loader"
    puts ""
    puts "Process:"
    puts "  1. User selects an environment from the list"
    puts "  2. Script configures the map loader with the environment model path"
    puts "  3. Map loader generates MLS and publishes to internal simulator"
    puts "  4. Internal simulator receives environment data on its input port"
    exit 1
end

is_task_name = ARGV[0].to_s
ml_task_name = ARGV[1].to_s

# Get the internal simulator task
begin
    is_task = Orocos.get(is_task_name)
rescue Orocos::NotFound
    puts "ERROR: Task '#{is_task_name}' not found!"
    puts "Make sure the task is running."
    exit 1
end

# Get the map loader task (required)
begin
    ml_task = Orocos.get(ml_task_name)
rescue Orocos::NotFound
    puts "ERROR: Map loader task '#{ml_task_name}' not found!"
    puts "The map loader task is required to generate MLS from environment models."
    puts "Make sure the task is running."
    exit 1
end

# Load environment definitions from YAML config file
unless File.exist?(ENVIRONMENTS_CONFIG)
    puts "ERROR: Configuration file not found: #{ENVIRONMENTS_CONFIG}"
    puts ""
    puts "Please create the configuration file with environment definitions."
    puts "See environments.yml.example for the expected format."
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

# Display available environments to the user
puts ""
puts "=" * 80
puts "Available Environments for Internal Simulation"
puts "=" * 80
puts ""

environments.each_with_index do |env, index|
    puts "  [#{index + 1}] #{env['name']}"
    puts "      Description: #{env['description']}" if env['description']
    puts "      Model: #{File.basename(env['path'])}"
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
selected_index = selection.to_i - 1
if selected_index < 0 || selected_index >= environments.size
    puts "ERROR: Invalid selection!"
    exit 1
end

selected_env = environments[selected_index]

puts ""
puts "Loading environment: #{selected_env['name']}"
puts "-" * 80

# Validate environment model path
environment_base_dir = ENV['AUTOPROJ_CURRENT_ROOT']
model_path = File.join(environment_base_dir, selected_env['path'])
unless File.exist?(model_path)
    puts "ERROR: Environment model file not found: #{model_path}"
    exit 1
end

puts "Environment model: #{model_path}"
puts "Description: #{selected_env['description']}" if selected_env['description']

# Process:
# 1. Stop the map loader task
puts ""
puts "Step 1: Stopping map loader task..."
ml_task.stop if ml_task.running?

# 2. Configure the map loader with the environment model path
puts "Step 2: Configuring map loader with environment model..."
ml_task.cleanup if ml_task.state != :PRE_OPERATIONAL

# Set the path property
ml_task.path = model_path

# Additional properties that might be needed (adjust based on actual map loader)
# ml_task.resolution = 0.1 if ml_task.respond_to?(:resolution=)

# 3. Reconfigure and start the map loader
puts "Step 3: Configuring map loader..."
ml_task.configure

# Wait for configuration to complete
puts "Waiting for map loader to reach STOPPED state..."
timeout = 30
start_time = Time.now
while ml_task.state != :STOPPED && (Time.now - start_time) < timeout
    sleep(0.5)
end

if ml_task.state != :STOPPED
    puts "ERROR: Map loader failed to configure (state: #{ml_task.state})!"
    exit 1
end

puts "Step 4: Starting map loader..."
ml_task.start

# Wait for task to start
puts "Waiting for map loader to start..."
timeout = 10
start_time = Time.now
while ml_task.state != :RUNNING && (Time.now - start_time) < timeout
    sleep(0.5)
end

if ml_task.state != :RUNNING
    puts "ERROR: Map loader failed to start (state: #{ml_task.state})!"
    exit 1
end

sleep(2)  # Give it a moment to fully initialize

# 5. Trigger MLS generation and publishing
puts "Step 5: Generating and publishing MLS to internal simulator..."

ml_task.publishMap

# if ml_task.respond_to?(:publishMap)
#     result = ml_task.publishMap
#     if result
#         puts "SUCCESS: MLS map published to internal simulator!"
#     else
#         puts "WARNING: publishMap returned false - check map loader status"
#     end
# else
#     puts "WARNING: publishMap operation not available on #{ml_task_name}"
#     puts "The map loader may publish automatically - check task connections"
# end

# Verify connection status
puts ""
puts "Verifying task connections..."
puts "Map loader output ports:"
ml_task.each_port do |port|
    if port.kind_of?(Orocos::OutputPort)
        puts "  - #{port.name} (#{port.type_name})"
    end
end

puts ""
puts "Internal simulator input ports:"
is_task.each_port do |port|
    if port.kind_of?(Orocos::InputPort)
        puts "  - #{port.name} (#{port.type_name})"
    end
end

puts ""
puts "=" * 80
puts "Environment '#{selected_env['name']}' loading complete!"
puts ""
puts "The environment MLS should now be available to the internal simulator."
puts "Check that the map loader output is connected to the internal simulator input:"
puts "  - Map loader MLS output -> Internal simulator environment_precalculated or environment_sloped"
puts "=" * 80
puts ""




