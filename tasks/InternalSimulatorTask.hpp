/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef SIMULATION_INTERNALSIMULATOR_TASK_TASK_HPP
#define SIMULATION_INTERNALSIMULATOR_TASK_TASK_HPP

#include "mars/InternalSimulatorTaskBase.hpp"
#include <maps/grid/MLSMap.hpp>
#include <base/samples/RigidBodyState.hpp>

namespace mars {

    /*! \class InternalSimulatorTask 
     * \brief Consolidated internal simulation task combining functionality from
     * mars::CorobxTask and mars::MLS for setting up and running internal simulations.
     * 
     * This task manages:
     * - Loading MLS environment data into MARS EnvireGraph (preferring PRECALCULATED format)
     * - Setting robot initial pose in MARS simulation
     * - Controlling simulation lifecycle through operations (run, finish, reset)
     * - Providing debug outputs for inspecting loaded data before simulation runs
     * 
     * The task uses runtime states to indicate readiness:
     * - POSE_RECEIVED: Start pose received on input port
     * - ENVIRONMENT_RECEIVED: Environment MLS received on input port
     * - READY_TO_SIMULATE: Both pose and environment available, ready for run()
     * - SIMULATING: Simulation actively running after successful run() call
     * 
     * Key behaviors:
     * - Environment preference: PRECALCULATED input takes priority over SLOPED
     * - SLOPED environments are converted to PRECALCULATED before loading to MARS
     * - MLS is stored at hardcoded frame "mls_01" in EnvireGraph
     * - finish() clears all data from memory and stops simulation
     * - reset() keeps data but transitions back to READY_TO_SIMULATE for re-running
     * - Debug ports output only when enabled AND not in SIMULATING state
     */
    class InternalSimulatorTask : public InternalSimulatorTaskBase
    {
	friend class InternalSimulatorTaskBase;
    protected:
        // Stored simulation data
        maps::grid::MLSMapPrecalculated mls_precalculated;
        maps::grid::MLSMapSloped mls_sloped;
        base::samples::RigidBodyState start_pose;
        
        // State flags
        bool has_environment;
        bool has_precalculated;  // true if mls_precalculated has data, false if mls_sloped has data
        bool has_pose;
        bool is_simulating;

        /**
         * Update runtime state based on current data availability flags.
         * Transitions to READY_TO_SIMULATE if both environment and pose available,
         * otherwise reflects which data is available.
         */
        void updateReadyState();

        /**
         * Check if simulation state matches is_simulating flag.
         * Asserts that internal state is consistent.
         */
        void checkSimulationState();

        /**
         * Remove existing MLS data from EnvireGraph at frame "mls_01".
         * Called before loading new environment or when entering error state.
         */
        void cleanupMLSData();

        /**
         * Load MLS environment into MARS EnvireGraph at frame "mls_01".
         * Handles both PRECALCULATED and SLOPED formats.
         * @param precalculated If true, loads mls_precalculated; otherwise mls_sloped
         * @return true if successful
         */
        bool loadMLSToEnvire(bool precalculated);

        /**
         * Set robot pose in MARS simulation using control->nodes interface.
         * @param pose The pose to set, targetFrame specifies which MARS node
         * @return true if successful
         */
        bool setRobotPose(const base::samples::RigidBodyState& pose);

    public:
        /** TaskContext constructor for InternalSimulatorTask
         * \param name Name of the task. This name needs to be unique to make it identifiable via nameservices.
         * \param initial_state The initial TaskState of the TaskContext. Default is Stopped state.
         */
        InternalSimulatorTask(std::string const& name = "simulation::InternalSimulatorTask");

        /** TaskContext constructor for InternalSimulatorTask 
         * \param name Name of the task. This name needs to be unique to make it identifiable via nameservices.
         * \param engine The RTT Execution engine to be used for this task, which serialises the execution of all commands, programs, state machines and incoming events for a task.
         * 
         */
        InternalSimulatorTask(std::string const& name, RTT::ExecutionEngine* engine);

	/** Default deconstructor of InternalSimulatorTask
	 */
	~InternalSimulatorTask();

        /** This hook is called by Orocos when the state machine transitions
         * from PreOperational to Stopped. If it returns false, then the
         * component will stay in PreOperational. Otherwise, it goes into
         * Stopped.
         *
         * It is meaningful only if the #needs_configuration has been specified
         * in the task context definition with (for example):
         \verbatim
         task_context "TaskName" do
           needs_configuration
           ...
         end
         \endverbatim
         */
        bool configureHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Stopped to Running. If it returns false, then the component will
         * stay in Stopped. Otherwise, it goes into Running and updateHook()
         * will be called.
         */
        bool startHook();

        /** This hook is called by Orocos when the component is in the Running
         * state, at each activity step. Here, the activity gives the "ticks"
         * when the hook should be called.
         *
         * The error(), exception() and fatal() calls, when called in this hook,
         * allow to get into the associated RunTimeError, Exception and
         * FatalError states. 
         *
         * In the first case, updateHook() is still called, and recover() allows
         * you to go back into the Running state.  In the second case, the
         * errorHook() will be called instead of updateHook(). In Exception, the
         * component is stopped and recover() needs to be called before starting
         * it again. Finally, FatalError cannot be recovered.
         */
        void updateHook();

        /** This hook is called by Orocos when the component is in the
         * RunTimeError state, at each activity step. See the discussion in
         * updateHook() about triggering options.
         *
         * Call recover() to go back in the Runtime state.
         */
        void errorHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Running to Stopped after stop() has been called.
         */
        void stopHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Stopped to PreOperational, requiring the call to configureHook()
         * before calling start() again.
         */
        void cleanupHook();

        // Operation implementations
        
        /** Start simulation with currently loaded environment and pose.
         * Sequences: pose setting -> MLS loading to EnvireGraph -> start MARS simulation.
         * @return false if not in READY_TO_SIMULATE state or if simulation is already running
         */
        virtual bool run();

        /** Stop simulation and clear all stored environment and pose data from memory.
         * Removes MLS from EnvireGraph, stops MARS simulation if running, resets to RUNNING state.
         * @return true immediately if simulation already stopped
         */
        virtual bool finish();

        /** Reset to READY_TO_SIMULATE state if both environment and pose data are available.
         * Has no effect if data is incomplete (remains in current state).
         * Used to prepare for another simulation run with same data.
         * @return true if reset successful or already in correct state
         */
        virtual bool reset_simulation();
    };
}

#endif
