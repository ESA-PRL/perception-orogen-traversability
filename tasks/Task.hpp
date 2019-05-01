#pragma once

#include <frame_helper/FrameHelper.h>
#include <opencv/cv.h>
#include <base/samples/Frame.hpp>
#include <opencv2/highgui/highgui.hpp>  // -> Needed?
#include <traversability/traversability.hpp>
#include "traversability/TaskBase.hpp"

namespace traversability
{

class Task : public TaskBase
{
    friend class TaskBase;

  protected:
    Traversability trav;
    base::samples::DistanceImage elevation_map;
    base::samples::frame::Frame local2global;
    base::samples::frame::Frame traversability_map;
    base::samples::RigidBodyState pose;

  public:
    Task(std::string const& name = "traversability::Task");
    Task(std::string const& name, RTT::ExecutionEngine* engine);
    ~Task();
    bool configureHook();
    bool startHook();
    void updateHook();
    void errorHook();
    void stopHook();
    void cleanupHook();
};

}  // namespace traversability
