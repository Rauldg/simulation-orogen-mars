/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "InternalSimulatorTask.hpp"
#include <envire_core/items/Item.hpp>
#include <envire_core/graph/EnvireGraph.hpp>
#include <mars/interfaces/sim/ControlCenter.h>
#include <mars/interfaces/sim/SimulatorInterface.h>
#include <mars/interfaces/sim/NodeManagerInterface.h>
#include <mars/interfaces/sim/StorageManagerInterface.h>
#include <boost/tuple/tuple.hpp>

using namespace mars;

InternalSimulatorTask::InternalSimulatorTask(std::string const& name)
    : InternalSimulatorTaskBase(name),
      has_environment(false),
      has_precalculated(false),
      has_pose(false),
      is_simulating(false)
{
}

InternalSimulatorTask::InternalSimulatorTask(std::string const& name, RTT::ExecutionEngine* engine)
    : InternalSimulatorTaskBase(name, engine),
      has_environment(false),
      has_precalculated(false),
      has_pose(false),
      is_simulating(false)
{
}

InternalSimulatorTask::~InternalSimulatorTask()
{
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See InternalSimulatorTask.hpp for more detailed
// documentation about them.

bool InternalSimulatorTask::configureHook()
{
    if (! InternalSimulatorTaskBase::configureHook())
        return false;
    return true;
}

bool InternalSimulatorTask::startHook()
{
    if (! InternalSimulatorTaskBase::startHook())
        return false;
    
    // Initialize state flags
    has_environment = false;
    has_precalculated = false;
    has_pose = false;
    is_simulating = false;
    
    return true;
}

void InternalSimulatorTask::updateHook()
{
    InternalSimulatorTaskBase::updateHook();

    // Only read input ports if not currently simulating
    if (!is_simulating)
    {
        // Read environment ports (precalculated takes priority)
        maps::grid::MLSMapPrecalculated temp_precalculated;
        if (_environment_precalculated.read(temp_precalculated) == RTT::NewData)
        {
            mls_precalculated = temp_precalculated;
            has_environment = true;
            has_precalculated = true;
        }

        maps::grid::MLSMapSloped temp_sloped;
        if (_environment_sloped.read(temp_sloped) == RTT::NewData)
        {
            mls_sloped = temp_sloped;
            // Only update has_environment from sloped if we don't already have precalculated
            if (!has_precalculated)
            {
                has_environment = true;
                has_precalculated = false;  // Explicitly mark that we're using sloped
            }
        }

        // Read start pose port
        base::samples::RigidBodyState temp_pose;
        if (_start_pose.read(temp_pose) == RTT::NewData)
        {
            start_pose = temp_pose;
            has_pose = true;
        }

        // Write debug ports if enabled and not simulating
        if (_debug_ports_enabled.get())
        {
            if (has_environment)
            {
                // Output the environment that will be used (prefer precalculated)
                if (has_precalculated)
                {
                    _debug_selected_environment.write(mls_precalculated);
                }
                else
                {
                    // Convert sloped to precalculated for debug output
                    maps::grid::MLSMapPrecalculated debug_mls = mls_sloped;
                    _debug_selected_environment.write(debug_mls);
                }
            }

            if (has_pose)
            {
                _debug_selected_pose.write(start_pose);
            }
        }

        // Update runtime state based on available data
        updateReadyState();
    }

    // Verify simulation state consistency
    checkSimulationState();
}

void InternalSimulatorTask::errorHook()
{
    InternalSimulatorTaskBase::errorHook();
    
    // Clean up MLS data when entering error state
    cleanupMLSData();
}

void InternalSimulatorTask::stopHook()
{
    InternalSimulatorTaskBase::stopHook();
    
    // Stop simulation if running
    if (is_simulating && control && control->sim)
    {
        control->sim->StopSimulation();
        is_simulating = false;
    }
    
    // Clean up on stop
    cleanupMLSData();
    has_environment = false;
    has_precalculated = false;
    has_pose = false;
}

void InternalSimulatorTask::cleanupHook()
{
    InternalSimulatorTaskBase::cleanupHook();
}

void InternalSimulatorTask::updateReadyState()
{
    if (is_simulating)
    {
        state(SIMULATING);
    }
    else if (has_environment && has_pose)
    {
        state(READY_TO_SIMULATE);
    }
    else if (has_environment)
    {
        state(ENVIRONMENT_RECEIVED);
    }
    else if (has_pose)
    {
        state(POSE_RECEIVED);
    }
    else
    {
        state(RUNNING);
    }
}

void InternalSimulatorTask::checkSimulationState()
{
    // Assert that our internal state matches reality
    if (control && control->sim)
    {
        bool mars_running = control->sim->isSimRunning();
        if (mars_running != is_simulating)
        {
            LOG_ERROR_S << "Simulation state mismatch: is_simulating=" << is_simulating 
                       << " but MARS reports running=" << mars_running;
            // Sync our state to MARS reality
            is_simulating = mars_running;
            updateReadyState();
        }
    }
}

void InternalSimulatorTask::cleanupMLSData()
{
    if (!control || !control->storage)
        return;

    try
    {
        envire::core::FrameId frame_id = "mls_01";
        
        // Check if frame exists
        if (control->storage->getGraph()->containsFrame(frame_id))
        {
            // Remove all items from the frame
            using MLSPrecalculatedItem = envire::core::Item<maps::grid::MLSMapPrecalculated>;
            using MLSPrecalculatedItr = envire::core::EnvireGraph::ItemIterator<MLSPrecalculatedItem>;
            
            MLSPrecalculatedItr itr, end_itr;
            boost::tie(itr, end_itr) = control->storage->getGraph()->getItems<MLSPrecalculatedItem>(frame_id);
            for (; itr != end_itr; itr++)
            {
                control->storage->getGraph()->removeItemFromFrame(frame_id, itr);
            }
        }
    }
    catch (const std::exception& e)
    {
        LOG_ERROR_S << "Failed to cleanup MLS data: " << e.what();
    }
}

bool InternalSimulatorTask::loadMLSToEnvire(bool precalculated)
{
    if (!control || !control->storage)
    {
        LOG_ERROR_S << "Control or storage not available";
        return false;
    }

    try
    {
        envire::core::FrameId frame_id = "mls_01";
        
        // Ensure frame exists (should be created by mars::Task)
        if (!control->storage->getGraph()->containsFrame(frame_id))
        {
            LOG_ERROR_S << "Frame " << frame_id << " does not exist in graph";
            return false;
        }

        // Create and add the MLS item
        if (precalculated)
        {
            envire::core::Item<maps::grid::MLSMapPrecalculated>::Ptr item(
                new envire::core::Item<maps::grid::MLSMapPrecalculated>(mls_precalculated));
            control->storage->getGraph()->addItemToFrame(frame_id, item);
            LOG_INFO_S << "Loaded PRECALCULATED MLS to frame " << frame_id;
        }
        else
        {
            // Convert sloped to precalculated before loading (simple assignment does conversion)
            maps::grid::MLSMapPrecalculated converted_mls = mls_sloped;
            
            envire::core::Item<maps::grid::MLSMapPrecalculated>::Ptr item(
                new envire::core::Item<maps::grid::MLSMapPrecalculated>(converted_mls));
            control->storage->getGraph()->addItemToFrame(frame_id, item);
            LOG_INFO_S << "Converted SLOPE MLS to PRECALCULATED and loaded to frame " << frame_id;
        }

        return true;
    }
    catch (const std::exception& e)
    {
        LOG_ERROR_S << "Failed to load MLS to EnvireGraph: " << e.what();
        return false;
    }
}

bool InternalSimulatorTask::setRobotPose(const base::samples::RigidBodyState& pose)
{
    if (!control || !control->nodes)
    {
        LOG_ERROR_S << "Control or nodes interface not available";
        return false;
    }

    try
    {
        // Use targetFrame to identify which MARS node to position
        std::string node_name = pose.targetFrame;
        if (node_name.empty())
        {
            LOG_ERROR_S << "targetFrame not specified in pose";
            return false;
        }

        // Convert base::Position to mars::utils::Vector
        mars::utils::Vector mars_position(pose.position.x(), pose.position.y(), pose.position.z());
        
        // Convert base::Orientation to mars::utils::Quaternion
        mars::utils::Quaternion mars_orientation(pose.orientation.w(), pose.orientation.x(), 
                                                  pose.orientation.y(), pose.orientation.z());

        // Set absolute pose in MARS
        control->nodes->setAbsolutePose(node_name, mars_position, mars_orientation);
        
        LOG_INFO_S << "Set pose for node '" << node_name << "' at position [" 
                   << pose.position.x() << ", " << pose.position.y() << ", " << pose.position.z() << "]";
        
        return true;
    }
    catch (const std::exception& e)
    {
        LOG_ERROR_S << "Failed to set robot pose: " << e.what();
        return false;
    }
}

bool InternalSimulatorTask::run()
{
    // Check preconditions
    if (is_simulating)
    {
        LOG_WARN_S << "Simulation already running, cannot start again";
        return false;
    }

    if (!has_environment || !has_pose)
    {
        LOG_ERROR_S << "Cannot start simulation: missing data (has_environment=" 
                    << has_environment << ", has_pose=" << has_pose << ")";
        return false;
    }

    if (state() != READY_TO_SIMULATE)
    {
        LOG_ERROR_S << "Cannot start simulation: not in READY_TO_SIMULATE state";
        return false;
    }

    if (!control || !control->sim)
    {
        LOG_ERROR_S << "Control or simulation interface not available";
        return false;
    }

    LOG_INFO_S << "Starting simulation sequence...";

    // Step 1: Set robot pose
    if (!setRobotPose(start_pose))
    {
        LOG_ERROR_S << "Failed to set robot pose";
        return false;
    }

    // Step 2: Clean up any existing MLS data
    cleanupMLSData();

    // Step 3: Load MLS environment (prefer precalculated)
    if (!loadMLSToEnvire(has_precalculated))
    {
        LOG_ERROR_S << "Failed to load MLS environment";
        return false;
    }

    // Step 4: Start MARS simulation
    try
    {
        control->sim->StartSimulation();
        is_simulating = true;
        state(SIMULATING);
        LOG_INFO_S << "Simulation started successfully";
        return true;
    }
    catch (const std::exception& e)
    {
        LOG_ERROR_S << "Failed to start MARS simulation: " << e.what();
        cleanupMLSData();
        return false;
    }
}

bool InternalSimulatorTask::finish()
{
    LOG_INFO_S << "Finishing simulation...";

    // Stop simulation if running
    if (is_simulating && control && control->sim)
    {
        try
        {
            control->sim->StopSimulation();
            LOG_INFO_S << "Simulation stopped";
        }
        catch (const std::exception& e)
        {
            LOG_ERROR_S << "Failed to stop simulation: " << e.what();
        }
        is_simulating = false;
    }

    // Clean up MLS from EnvireGraph
    cleanupMLSData();

    // Clear all stored data
    mls_precalculated = maps::grid::MLSMapPrecalculated();
    mls_sloped = maps::grid::MLSMapSloped();
    start_pose = base::samples::RigidBodyState();
    has_environment = false;
    has_precalculated = false;
    has_pose = false;

    // Reset to RUNNING state
    state(RUNNING);
    
    LOG_INFO_S << "Simulation finished and data cleared";
    return true;
}

bool InternalSimulatorTask::reset_simulation()
{
    LOG_INFO_S << "Resetting simulation state...";

    // No-op if already simulating
    if (is_simulating)
    {
        LOG_INFO_S << "Already simulating, no reset needed";
        return true;
    }

    // Update ready state based on current data availability
    updateReadyState();

    LOG_INFO_S << "Reset complete, state updated to " << state();
    return true;
}
