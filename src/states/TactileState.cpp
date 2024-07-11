#include <mc_rtc/gui/Button.h>
#include <mc_rtc/ros.h>
#include <MultiContactController/LimbManager.h>
#include <MultiContactController/MultiContactController.h>
#include <MultiContactController/states/TactileState.h>

using namespace MCC;

void TactileState::start(mc_control::fsm::Controller & _ctl)
{
  State::start(_ctl);

  // Skip if ROS is not initialized
  if(!mc_rtc::ROSBridge::get_node_handle())
  {
    mc_rtc::log::warning("[TactileState] ROS is not initialized.");
    output("OK");
    return;
  }

  // Load configuration
  std::string tactileTopicName = "/tactile_sensor_system_client/contact_area";
  if(config_.has("configs"))
  {
    // 無視する領域のconfigを読み込む

    // if(config_("configs").has("velScale"))
    // {
    //   velScale_ = config_("configs")("velScale");
    //   velScale_[2] = mc_rtc::constants::toRad(velScale_[2]);
    // }
    config_("configs")("tactileTopicName", tactileTopicName);
  }

  // Setup ROS
  nh_ = std::make_unique<ros::NodeHandle>();
  // Use a dedicated queue so as not to call callbacks of other modules
  nh_->setCallbackQueue(&callbackQueue_);
  tactileSub_ = nh_->subscribe<std_msgs::Float32MultiArray>(tactileTopicName, 1, &TactileState::tactileCallback, this);

  for(auto foot : feet)
  {
    contactArea_[foot].clear();
    min_pose_[foot] = Eigen::Vector3d::Zero();
    max_pose_[foot] = Eigen::Vector3d::Zero();
    touchDown_[foot] = false;
  }

  output("OK");
}

bool TactileState::run(mc_control::fsm::Controller &)
{
  // Finish if ROS is not initialized
  if(!mc_rtc::ROSBridge::get_node_handle())
  {
    return true;
  }

  // Call ROS callback
  callbackQueue_.callAvailable(ros::WallDuration());

  ctl().limbManagerSet_->setSensordContactArea(contactArea_);
  ctl().limbManagerSet_->setSensorMinContactPosition(min_pose_);
  ctl().limbManagerSet_->setSensorMaxContactPosition(max_pose_);
  ctl().limbManagerSet_->setSensorTouchDown(touchDown_);

  return false;
}

void TactileState::teardown(mc_control::fsm::Controller &) {}

void TactileState::tactileCallback(const std_msgs::Float32MultiArray::ConstPtr & tactileMsg)
{

  for(auto foot : feet)
  {
    int footIndex_offset = foot == "LeftFoot" ? 0 : 9;
    auto tactileData = tactileMsg->data;

    bool touchDown = false;
    // for(const auto & contact : ctl().footManager_->getCurrentContactFeet())
    // {
    //   if(contact == foot && !tactileData[8 + footIndex_offset])
    //   {
    //     touchDown = true;
    //   }
    // }

    if(!touchDown)
    {
      contactArea_[foot].push_back(
          Eigen::Vector2d(tactileData[0 + footIndex_offset], tactileData[1 + footIndex_offset]));
      contactArea_[foot].push_back(
          Eigen::Vector2d(tactileData[2 + footIndex_offset], tactileData[3 + footIndex_offset]));
      contactArea_[foot].push_back(
          Eigen::Vector2d(tactileData[4 + footIndex_offset], tactileData[5 + footIndex_offset]));
      contactArea_[foot].push_back(
          Eigen::Vector2d(tactileData[6 + footIndex_offset], tactileData[7 + footIndex_offset]));

      min_pose_[foot] = Eigen::Vector3d(tactileData[2 + footIndex_offset], tactileData[7 + footIndex_offset], -0.097);
      max_pose_[foot] = Eigen::Vector3d(tactileData[0 + footIndex_offset], tactileData[5 + footIndex_offset], -0.097);
    }
    touchDown_[foot] = tactileData[8 + footIndex_offset] ? true : false;
  }
}

EXPORT_SINGLE_STATE("MCC::Tactile", TactileState)
