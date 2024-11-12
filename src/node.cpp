#include "lane_width_calculator/node.hpp"

namespace lane_width_calculator
{

// return the lateral offset from the boundary line
// value is positive if the vehicle is on the right side of the boundary line
double calcLateralOffset(
  const lanelet::ConstLineString2d & boundary_line, const geometry_msgs::msg::Pose & search_pose)
{
  std::vector<geometry_msgs::msg::Point> boundary_path(boundary_line.size());
  for (size_t i = 0; i < boundary_path.size(); ++i) {
    const double x = boundary_line[i].x();
    const double y = boundary_line[i].y();
    boundary_path[i] = tier4_autoware_utils::createPoint(x, y, 0.0);
  }

  return motion_utils::calcLateralOffset(boundary_path, search_pose.position);
}

geometry_msgs::msg::Pose calcPoseFromRelativeOffset(
  const geometry_msgs::msg::Pose & base_pose, const double longitudinal_offset, const double lateral_offset)
{
  const double yaw = tf2::getYaw(base_pose.orientation);
  const double x = base_pose.position.x + longitudinal_offset * std::cos(yaw) -
                   lateral_offset * std::sin(yaw);
  const double y = base_pose.position.y + longitudinal_offset * std::sin(yaw) +
                   lateral_offset * std::cos(yaw);
  geometry_msgs::msg::Pose output_pose;
  output_pose = base_pose;
  output_pose.position.x = x;
  output_pose.position.y = y;
  return output_pose;
}

// return left and right lateral offsets from the boundary line
std::array<double,2> calcLeftOrRightOffsetsInOneLane(lanelet::ConstLanelet lanelet, const geometry_msgs::msg::Pose & search_pose)
{
  const lanelet::ConstLineString2d left_bound = lanelet.leftBound2d();
  const lanelet::ConstLineString2d right_bound = lanelet.rightBound2d();
  const double left_offset = calcLateralOffset(left_bound, search_pose);
  const double right_offset = calcLateralOffset(right_bound, search_pose);
  return {left_offset, right_offset};
}


CalculatorNode::CalculatorNode(const rclcpp::NodeOptions & options)
:Node("lane_width_calculator", options),
diagnostics_updater_(this)
{
  sub_objects_ = this->create_subscription<PoseStamped>(
    "~/input/pose", 1,
    std::bind(&CalculatorNode::poseCallback, this, std::placeholders::_1));
  sub_map_ = this->create_subscription<HADMapBin>(
    "~/input/vector_map", rclcpp::QoS{1}.transient_local(),
    std::bind(&CalculatorNode::mapCallback, this, std::placeholders::_1));
  sub_odom_ = this->create_subscription<Odometry>(
    "~/input/odom", 1,
    std::bind(&CalculatorNode::odomCallback, this, std::placeholders::_1));
  sub_path_ = this->create_subscription<autoware_auto_planning_msgs::msg::PathWithLaneId>(
    "/planning/scenario_planning/lane_driving/behavior_planning/path_with_lane_id",
    1, std::bind(&CalculatorNode::pathCallback, this, std::placeholders::_1));

  pub_debug_markers_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("~/output/debug_markers", 1);
  pub_candidate_lanelet_ids_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("candidate_lanelet_ids", 1);
  pub_driving_lanelet_id_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("driving_lanelet_ids", 1);

  // parameters
  vehicle_width_ = this->declare_parameter<double>("vehicle_width");
  vehicle_length_ = this->declare_parameter<double>("vehicle_length");
  vehicle_height_ = this->declare_parameter<double>("vehicle_height");
  outside_lane_threshold_sec_ = this->declare_parameter<double>("outside_lane_threshold_sec", 1.0);

  // if set to true, use odometry instead of pose
  use_odom_ = this->declare_parameter<bool>("use_odom");

  // file logging parameters
  save_to_csv_ = this->declare_parameter<bool>("save_to_csv");
  save_file_name_ = this->declare_parameter<std::string>("save_file_name");

  // initialize csv file
  position_list_ = {"center", "front_left", "front_right", "rear_left", "rear_right"};
  if(save_to_csv_){
    file_stream_.open(save_file_name_);
    file_stream_ << "time, vehicle_x, vehicle_y, ";
    for(auto & position_name: position_list_){
      file_stream_ << position_name << "_left_offset, " << position_name << "_right_offset, ";
    }
    file_stream_ << "is_inside_lane" << std::endl;
  }
  // Initialize diagnostic updater
  diagnostics_updater_.setHardwareID("lane_width_calculator");
  diagnostics_updater_.add("lane_status", this, &CalculatorNode::checkLaneStatus);
  is_vehicle_inside_lane_ = false;  // Initialize to true
}

// deconstructor
CalculatorNode::~CalculatorNode()
{
  // close csv file
  if(save_to_csv_){
    file_stream_.close();
  }
}

// Add the path callback implementation
void CalculatorNode::pathCallback(
  const autoware_auto_planning_msgs::msg::PathWithLaneId::ConstSharedPtr msg)
{
  current_path_ids_.clear();
  for (const auto & point : msg->points) {
    for (const auto & lane_id : point.lane_ids) {
      current_path_ids_.insert(lane_id);
    }
  }
}

void CalculatorNode::mapCallback(const HADMapBin::ConstSharedPtr msg)
{
  RCLCPP_INFO(get_logger(), "[lane width calculator]: Start loading lanelet");
  lanelet_map_ptr_ = std::make_shared<lanelet::LaneletMap>();
  lanelet::utils::conversion::fromBinMsg(
    *msg, lanelet_map_ptr_, &traffic_rules_ptr_, &routing_graph_ptr_);
  RCLCPP_INFO(get_logger(), "[lane width calculator]: Map is loaded");

  const_lanelets_.clear();
  for (auto l : lanelet_map_ptr_->laneletLayer) {
    const_lanelets_.push_back(l);
  }
  map_loaded_ = true;
}

// pose callback is not used if use_odom_ is true
void CalculatorNode::poseCallback(const PoseStamped::ConstSharedPtr msg)
{
  // If map is empty or use odom flag do nothing
  if (!map_loaded_ ||use_odom_) {
    return;
  }

  geometry_msgs::msg::Pose query_pose = msg->pose;
  const auto all_lanelets = lanelet::utils::query::laneletLayer(lanelet_map_ptr_);

  lanelet::ConstLanelets current_lanelets;
  std::cout << "pose cb" << std::endl;
  if (lanelet::utils::query::getCurrentLanelets(const_lanelets_, query_pose, &current_lanelets)) {
    // do something
    updateVehiclePoses(query_pose);



    calcLeftOrRightOffsetsInOneLane(current_lanelets.front(), query_pose);
  }
}

void CalculatorNode::odomCallback(const Odometry::ConstSharedPtr msg)
{
  // If map is empty or use odom flag do nothing
  if (!map_loaded_ ||!use_odom_) {
    return;
  }

  geometry_msgs::msg::Pose query_pose = msg->pose.pose;
  const auto all_lanelets = lanelet::utils::query::laneletLayer(lanelet_map_ptr_);
  lanelet::ConstLanelets current_lanelets;
  updateVehiclePoses(query_pose);
  vehicleIsInsideLane();
  diagnostics_updater_.force_update();
  if (lanelet::utils::query::getCurrentLanelets(const_lanelets_, query_pose, &current_lanelets)) {

    // Publish current lanelet IDs as an int32 multi array
    std_msgs::msg::Int32MultiArray msg_ids;
    

    msg_ids.data.reserve(current_lanelets.size());
    for (const auto& lanelet : current_lanelets) {
      msg_ids.data.push_back(lanelet.id());
    }
    pub_candidate_lanelet_ids_->publish(msg_ids);

    // calc left/right lateral offsets for each pose
    for(auto & pose_pair: position_pose_map_){
      position_offset_map_[pose_pair.first] = calcLeftOrRightOffsetsInOneLane(current_lanelets.front(), pose_pair.second);
    }
    // find by angle
    double degrees = 20.0;
    double min_angle = degrees * 0.01745329;//std::numeric_limits<double>::max();
    double pose_yaw = tf2::getYaw(query_pose.orientation);
    lanelet::BasicPoint2d search_point(query_pose.position.x, query_pose.position.y);

    std_msgs::msg::Int32MultiArray msg_ids_;
    msg_ids_.data.reserve(current_lanelets.size());
    for (const auto & llt : current_lanelets) {
      lanelet::ConstLineString3d segment = lanelet::utils::getClosestSegment(search_point, llt.centerline());
      double segment_angle = std::atan2(
        segment.back().y() - segment.front().y(), segment.back().x() - segment.front().x());
      double angle_diff = std::abs(autoware_utils::normalize_radian(segment_angle - pose_yaw));
      if (angle_diff < min_angle) {
        msg_ids_.data.push_back(llt.id());
      }
      else
      {
      }
      pub_driving_lanelet_id_->publish(msg_ids_);
    }
    // publish visualization marker
    publishBBOX();
    // save to csv
    if(save_to_csv_){
      appendToCSV();
    }
  }
}


void CalculatorNode::updateVehiclePoses(const geometry_msgs::msg::Pose & pose)
{
  position_pose_map_["center"] = pose;
  position_pose_map_["front_left"] = calcPoseFromRelativeOffset(
    pose, vehicle_length_ / 2.0, vehicle_width_ / 2.0);
  position_pose_map_["front_right"] = calcPoseFromRelativeOffset(
    pose, vehicle_length_ / 2.0, -vehicle_width_ / 2.0);
  position_pose_map_["rear_left"] = calcPoseFromRelativeOffset(
    pose, -vehicle_length_ / 2.0, vehicle_width_ / 2.0);
  position_pose_map_["rear_right"] = calcPoseFromRelativeOffset(
    pose, -vehicle_length_ / 2.0, -vehicle_width_ / 2.0);
}

// Update diagnostic function
void CalculatorNode::checkLaneStatus(diagnostic_updater::DiagnosticStatusWrapper& stat)
{
  // Get offsets for center position
  const auto& offsets = position_offset_map_["center"];
  const double left_offset = offsets[0];
  const double right_offset = offsets[1];

  // Add path IDs
  std::stringstream path_ss;
  for (const auto& id : current_path_ids_) {
    path_ss << id << " ";
  }
  stat.add("path_lanelet_ids", path_ss.str());

  // Add current lanelet IDs where vehicle is
  geometry_msgs::msg::Pose center_pose = position_pose_map_["center"];
  lanelet::ConstLanelets current_lanelets;
  std::stringstream current_ss;
  if (lanelet::utils::query::getCurrentLanelets(const_lanelets_, center_pose, &current_lanelets)) {
    for (const auto& lanelet : current_lanelets) {
      current_ss << lanelet.id() << " ";
    }
  }
  stat.add("current_lanelet_ids", current_ss.str());

  // Add the lanelet IDs used for judgment
  std::stringstream judge_ss;
  for (const auto& id : judging_lanelet_id_) {
    judge_ss << id << " ";
  }
  stat.add("judging_lanelets_ids", judge_ss.str());

  const auto current_time = this->now();
  if (!is_vehicle_inside_lane_) {
    if (!was_previously_outside_) {
      last_outside_lane_time_ = current_time;
      was_previously_outside_ = true;
    }
    const double time_outside = (current_time - last_outside_lane_time_).seconds();
    
    if (time_outside >= outside_lane_threshold_sec_) {
      std::stringstream ss;
      ss << std::fixed << std::setprecision(3);
      if (left_offset > 0.0) {
        ss << "Outside left boundary by " << left_offset << "m for " << time_outside << "s";
      } else if (right_offset < 0.0) {
        ss << "Outside right boundary by " << std::abs(right_offset) << "m for " << time_outside << "s";
      }
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, ss.str());
    } else {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, 
        "Vehicle outside lane but within time threshold");
    }
    
    stat.add("time_outside_lane", std::to_string(time_outside));
  } else {
    was_previously_outside_ = false;
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Vehicle is inside lane");
  }
  
  // Add offset values
  stat.add("left_offset", std::to_string(left_offset));
  stat.add("right_offset", std::to_string(right_offset));

}

// Helper function to check if lanelet is in current path
bool CalculatorNode::isLaneletInCurrentPath(const lanelet::ConstLanelet& lanelet) const 
{
  return current_path_ids_.find(lanelet.id()) != current_path_ids_.end();
}


// Update the vehicleIsInsideLane function
bool CalculatorNode::vehicleIsInsideLane()
{
  // current_path_ids_はpath_with_lane_idと同等 planningから計算されるpathのid

  if (current_path_ids_.empty()) {
    is_vehicle_inside_lane_ = false;
    return false;
  }
  geometry_msgs::msg::Pose center_pose = position_pose_map_["center"];
  lanelet::ConstLanelets current_lanelets;
  if (!lanelet::utils::query::getCurrentLanelets(const_lanelets_, center_pose, &current_lanelets)) {
    is_vehicle_inside_lane_ = false;
    return false;
  }

  bool found_matching_lanelet = false;
  bool is_inside_any = false;
  judging_lanelet_id_.clear();  // Reset at the start
  
  // Check each current lanelet
  for (const auto& lanelet : current_lanelets) {
    // Skip if this lanelet is not in the current path
    if (current_path_ids_.find(lanelet.id()) == current_path_ids_.end()) {
      continue;
    }

    found_matching_lanelet = true;
    bool is_inside_current = true;
    
    
    // Check vehicle position relative to this lanelet's bounds
    for (const auto & pose_pair : position_pose_map_) {
      // const auto & position_name = pose_pair.first;
      const auto & pose = pose_pair.second;
      
      // Calculate offsets for this position in this lanelet
      auto offsets = calcLeftOrRightOffsetsInOneLane(lanelet, pose);
      const auto left_offset = offsets[0];
      const auto right_offset = offsets[1];
      
      if (left_offset > 0.0) {
        std::cout << "    Outside left boundary by " << left_offset << "m" << std::endl;
        is_inside_current = false;
        judging_lanelet_id_.insert(lanelet.id());
        break;
      }
      if (right_offset < 0.0) {
        std::cout << "    Outside right boundary by " << std::abs(right_offset) << "m" << std::endl;
        is_inside_current = false;
        judging_lanelet_id_.insert(lanelet.id());
        break;
      }
    }
    
    if (is_inside_current) {
      is_inside_any = true;
      judging_lanelet_id_.insert(lanelet.id());
      break;
    }
  }


  is_vehicle_inside_lane_ = is_inside_any;
  
  if (!found_matching_lanelet) {
    return false;
  }
  
  return is_inside_any;
}

void CalculatorNode::publishBBOX()
{
  const bool is_inside = vehicleIsInsideLane();

  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "map"; // frame id
  marker.type = visualization_msgs::msg::Marker::CUBE;
  marker.action = visualization_msgs::msg::Marker::ADD;

  // define marker position
  marker.pose = position_pose_map_["center"];

  // set merker size
  marker.scale.x = vehicle_length_;
  marker.scale.y = vehicle_width_;
  marker.scale.z = vehicle_height_;
  
  // set marker color
  if(is_inside){
    setColor(marker, ColorSetting::GREEN);
  }else{
    setColor(marker, ColorSetting::RED);
  }
  
  // push
  visualization_msgs::msg::MarkerArray marker_array;
  marker_array.markers.push_back(marker);

  // publish
  pub_debug_markers_->publish(marker_array);
}

bool CalculatorNode::appendToCSV()
{
  const auto current_time = this->now();
  const double current_time_sec = current_time.seconds();
  const auto current_pose = position_pose_map_["center"];
  const auto vehicle_x = current_pose.position.x;
  const auto vehicle_y = current_pose.position.y;
  // 1. write time, vehicle_x, vehicle_y to csv 
  file_stream_ << std::fixed << std::setprecision(6);
  file_stream_ << current_time_sec << "," << vehicle_x << "," << vehicle_y << "," ;

  // 2. for each keys in position_list_ get left and right offset
  for(auto & position_name: position_list_){
    const auto & offset_pair = position_offset_map_.at(position_name);
    const auto left_offset = offset_pair[0];
    const auto right_offset = offset_pair[1];
    file_stream_ << left_offset << "," << right_offset << ",";
  }

  // 3. get if the vehicle is inside lane
  const bool is_inside = vehicleIsInsideLane();
  const int is_inside_int = is_inside ? 1 : 0;
  file_stream_ << is_inside_int << std::endl;
  return true;
}

} // namespace lane_width_calculator

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(lane_width_calculator::CalculatorNode)