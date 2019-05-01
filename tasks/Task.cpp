#include "Task.hpp"

using namespace traversability;

Task::Task(std::string const& name) : TaskBase(name) {}
Task::Task(std::string const& name, RTT::ExecutionEngine* engine) : TaskBase(name, engine) {}
Task::~Task() {}

bool Task::configureHook()
{
    if (!TaskBase::configureHook())
    {
        return false;
    }

    trav.configureTraversability(_rover_obstacle_clearance.value(),
                                 _rover_slope_gradeability.value(),
                                 _robot_size.value(),
                                 _map_resolution.value());

    return true;
}

bool Task::startHook() { return TaskBase::startHook(); }

void Task::updateHook()
{
    TaskBase::updateHook();

    if (_elevation_map.read(elevation_map) == RTT::NewData)
    {
        cv::Mat traversability;
        trav.setElevationMap(elevation_map.data, elevation_map.width, elevation_map.height);
        traversability = trav.computeTraversability();
        frame_helper::FrameHelper::copyMatToFrame(traversability, traversability_map);
        _traversability_map.write(traversability_map);
    }

    if (_local2global_orientation.read(local2global) == RTT::NewData)
    {
        if (_pose.read(pose) == RTT::NewData)
        {
            cv::Mat traversability;
            traversability = trav.local2globalOrientation(
                frame_helper::FrameHelper::convertToCvMat(local2global), pose.getYaw());
            frame_helper::FrameHelper::copyMatToFrame(traversability, traversability_map);
            _traversability_map.write(traversability_map);
        }
    }
}

void Task::errorHook() { TaskBase::errorHook(); }
void Task::stopHook() { TaskBase::stopHook(); }
void Task::cleanupHook() { TaskBase::cleanupHook(); }
