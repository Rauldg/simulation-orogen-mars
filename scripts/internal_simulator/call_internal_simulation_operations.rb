#!/usr/bin/env ruby

require 'orocos'
require 'readline'
include Orocos

Orocos.initialize

# Check that the script is called with the right number of arguments
if ARGV.size != 1
    puts "Usage: call_internal_simulation_operations.rb <internal_simulator_task_name>"
    puts ""
    puts "Arguments:"
    puts "  internal_simulator_task_name  - Name of the running mars::InternalSimulatorTask"
    puts ""
    puts "Examples:"
    puts "  ./call_internal_simulation_operations.rb asguard_v4_internal_sim_refactored"
    puts ""
    puts "Available operations:"
    puts "  run()              - Start simulation with loaded environment and pose"
    puts "  finish()           - Stop simulation and clear all data"
    puts "  reset_simulation() - Reset to READY_TO_SIMULATE state (keeps data)"
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

# Display task information
puts ""
puts "=" * 80
puts "Internal Simulation - Operation Control"
puts "=" * 80
puts ""
puts "Task: #{is_task_name}"
puts "Task state: #{is_task.state}"
puts ""

# Check available operations
available_operations = []
begin
    available_operations << 'run' if is_task.respond_to?(:run)
    available_operations << 'finish' if is_task.respond_to?(:finish)
    available_operations << 'reset_simulation' if is_task.respond_to?(:reset_simulation)
rescue
    # Ignore errors checking operations
end

if available_operations.empty?
    puts "WARNING: Could not detect available operations on task."
    puts "Proceeding anyway..."
end

puts "Available operations:"
puts "  [1] run()              - Start simulation"
puts "  [2] finish()           - Stop simulation and clear data"
puts "  [3] reset_simulation() - Reset to ready state (keeps data)"
puts "  [4] Check task state"
puts "  [q] Quit"
puts ""
puts "=" * 80

# Main loop
loop do
    puts ""
    print "Select operation [1-4, q to quit]: "
    selection = Readline.readline.strip

    case selection.downcase
    when 'q'
        puts "Exiting..."
        exit 0
        
    when '1'
        # Call run()
        puts ""
        puts "-" * 80
        puts "Calling run() operation..."
        puts "-" * 80
        begin
            result = is_task.run
            puts ""
            if result
                puts "SUCCESS: Simulation started!"
                puts "Task state: #{is_task.state}"
            else
                puts "FAILED: run() returned false"
                puts "Task state: #{is_task.state}"
                puts ""
                puts "Possible reasons:"
                puts "  - Environment not loaded"
                puts "  - Start pose not set"
                puts "  - Task not in READY_TO_SIMULATE state"
            end
        rescue Exception => e
            puts "ERROR: Failed to call run() operation!"
            puts "Error: #{e.message}"
        end
        
    when '2'
        # Call finish()
        puts ""
        puts "-" * 80
        puts "Calling finish() operation..."
        puts "-" * 80
        begin
            result = is_task.finish
            puts ""
            if result
                puts "SUCCESS: Simulation stopped and data cleared!"
                puts "Task state: #{is_task.state}"
            else
                puts "FAILED: finish() returned false"
                puts "Task state: #{is_task.state}"
            end
        rescue Exception => e
            puts "ERROR: Failed to call finish() operation!"
            puts "Error: #{e.message}"
        end
        
    when '3'
        # Call reset_simulation()
        puts ""
        puts "-" * 80
        puts "Calling reset_simulation() operation..."
        puts "-" * 80
        begin
            result = is_task.reset_simulation
            puts ""
            if result
                puts "SUCCESS: Simulation reset to ready state!"
                puts "Task state: #{is_task.state}"
                puts ""
                puts "You can now call run() again to restart simulation"
                puts "with the same environment and pose."
            else
                puts "FAILED: reset_simulation() returned false"
                puts "Task state: #{is_task.state}"
            end
        rescue Exception => e
            puts "ERROR: Failed to call reset_simulation() operation!"
            puts "Error: #{e.message}"
        end
        
    when '4'
        # Check task state
        puts ""
        puts "-" * 80
        puts "Task Information"
        puts "-" * 80
        begin
            puts "Task name: #{is_task_name}"
            puts "Task state: #{is_task.state}"
            puts "Task runtime state: #{is_task.runtime_state}" if is_task.respond_to?(:runtime_state)
            
            # Try to get debug ports if available
            if is_task.respond_to?(:debug_ports_enabled)
                puts "Debug ports enabled: #{is_task.debug_ports_enabled}"
            end
            
            puts ""
            puts "Input ports:"
            is_task.each_port do |port|
                if port.kind_of?(Orocos::InputPort)
                    connected = port.connected? ? "connected" : "disconnected"
                    puts "  - #{port.name} (#{port.type_name}) [#{connected}]"
                end
            end
            
            puts ""
            puts "Output ports:"
            is_task.each_port do |port|
                if port.kind_of?(Orocos::OutputPort)
                    connected = port.connected? ? "connected" : "disconnected"
                    puts "  - #{port.name} (#{port.type_name}) [#{connected}]"
                end
            end
        rescue Exception => e
            puts "ERROR: Failed to get task information!"
            puts "Error: #{e.message}"
        end
        
    else
        puts "Invalid selection! Please enter 1-4 or 'q'."
    end
    
    puts ""
    puts "-" * 80
end
