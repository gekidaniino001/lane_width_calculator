#include "lane_width_calculator/node.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace lane_width_calculator
{

namespace
{
// findCurrentLanelets()でR-treeを引くときに、車両位置の周りへ広げる幅[m]。
// 境界ちょうどの点が浮動小数点の誤差で候補から漏れないようにするための余裕で、
// 候補が増えても最後にlanelet::geometry::inside()で判定するので結果は変わらない。
constexpr double kCurrentLaneletSearchMargin = 0.01;
}  // namespace

// return the lateral offset from the boundary line (すでにPoint配列に変換済みの場合)
// value is positive if the vehicle is on the right side of the boundary line
double calcLateralOffset(
  const std::vector<geometry_msgs::msg::Point> & boundary_path, const geometry_msgs::msg::Pose & search_pose)
{
  return motion_utils::calcLateralOffset(boundary_path, search_pose.position);
}

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

  return calcLateralOffset(boundary_path, search_pose);
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
  boundary_path_cache_.clear();
  lanelet_index_by_id_.clear();
  for (auto l : lanelet_map_ptr_->laneletLayer) {
    // const_lanelets_内の位置(findCurrentLanelets()で結果をこの並びに揃えるために使う)
    lanelet_index_by_id_.emplace(l.id(), const_lanelets_.size());
    const_lanelets_.push_back(l);

    // leftBound2d/rightBound2dのPoint変換はmap読み込み時に1回だけ行い、
    // 毎odomコールバックでの再変換を避ける(calcLeftOrRightOffsetsCached参照)
    BoundaryPaths boundary_paths;
    const lanelet::ConstLineString2d left_bound = l.leftBound2d();
    boundary_paths.left.resize(left_bound.size());
    for (size_t i = 0; i < left_bound.size(); ++i) {
      boundary_paths.left[i] =
        tier4_autoware_utils::createPoint(left_bound[i].x(), left_bound[i].y(), 0.0);
    }
    const lanelet::ConstLineString2d right_bound = l.rightBound2d();
    boundary_paths.right.resize(right_bound.size());
    for (size_t i = 0; i < right_bound.size(); ++i) {
      boundary_paths.right[i] =
        tier4_autoware_utils::createPoint(right_bound[i].x(), right_bound[i].y(), 0.0);
    }
    boundary_path_cache_.emplace(l.id(), std::move(boundary_paths));
  }
  map_loaded_ = true;
}

// キャッシュ済みのleft/right境界線(BoundaryPaths)を使って左右オフセットを計算する。
// calcLeftOrRightOffsetsInOneLane()と異なり、boundary_line->vector<Point>の変換を
// 毎回行わずに済む(mapCallbackで変換済みのものを使い回す)。
std::array<double, 2> CalculatorNode::calcLeftOrRightOffsetsCached(
  const lanelet::ConstLanelet & lanelet, const geometry_msgs::msg::Pose & search_pose)
{
  const auto it = boundary_path_cache_.find(lanelet.id());
  if (it == boundary_path_cache_.end()) {
    // キャッシュに無い場合(通常は発生しない)は従来通りその場で変換して計算する
    RCLCPP_WARN_ONCE(
      get_logger(), "boundary_path_cache_ has no entry for lanelet id %ld. Falling back.",
      lanelet.id());
    return calcLeftOrRightOffsetsInOneLane(lanelet, search_pose);
  }
  const double left_offset = calcLateralOffset(it->second.left, search_pose);
  const double right_offset = calcLateralOffset(it->second.right, search_pose);
  return {left_offset, right_offset};
}

// search_poseを含むlaneletを求める。結果はlanelet::utils::query::getCurrentLanelets(const_lanelets_, ...)と
// 要素も順序も同じになる(順序はconst_lanelets_の並び。front()を基準にしたオフセット計算や
// publishする配列の順を変えないため)。
// getCurrentLanelets()は地図の全laneletについてポリゴンを作って内外判定するので重い
// (1257 laneletの地図で1回約0.35ms)。ここでは地図のR-tree(laneletLayer.search)で、
// バウンディングボックスが車両位置を含むlaneletだけに絞ってから、同じ判定(lanelet::geometry::inside)を行う。
// current_lanelets は最初にclearする(getCurrentLanelets()と違い、呼び出し側でclearしなくてよい)。
bool CalculatorNode::findCurrentLanelets(
  const geometry_msgs::msg::Pose & search_pose, lanelet::ConstLanelets * current_lanelets) const
{
  current_lanelets->clear();
  if (!lanelet_map_ptr_) {
    return false;
  }
  if (!std::isfinite(search_pose.position.x) || !std::isfinite(search_pose.position.y)) {
    // NaN/無限大の位置ではR-treeの検索に頼らず、従来の線形走査と同じ結果を返す
    return lanelet::utils::query::getCurrentLanelets(const_lanelets_, search_pose, current_lanelets);
  }

  const lanelet::BasicPoint2d search_point(search_pose.position.x, search_pose.position.y);
  const lanelet::BasicPoint2d margin(kCurrentLaneletSearchMargin, kCurrentLaneletSearchMargin);
  const lanelet::BasicPoint2d search_min = search_point - margin;
  const lanelet::BasicPoint2d search_max = search_point + margin;
  const lanelet::LaneletMap & map = *lanelet_map_ptr_;
  const auto candidates = map.laneletLayer.search(lanelet::BoundingBox2d(search_min, search_max));

  // (const_lanelets_内の位置, lanelet)
  std::vector<std::pair<size_t, lanelet::ConstLanelet>> found;
  for (const auto & llt : candidates) {
    if (!lanelet::geometry::inside(llt, search_point)) {
      continue;
    }
    const auto it = lanelet_index_by_id_.find(llt.id());
    if (it == lanelet_index_by_id_.end()) {
      // lanelet_index_by_id_はconst_lanelets_と同時に作るので通常は起きない。念のため従来の線形走査に戻す
      RCLCPP_WARN_ONCE(
        get_logger(), "lanelet id %ld is not in lanelet_index_by_id_. Falling back to linear search.",
        llt.id());
      current_lanelets->clear();
      return lanelet::utils::query::getCurrentLanelets(const_lanelets_, search_pose, current_lanelets);
    }
    found.emplace_back(it->second, llt);
  }

  std::sort(found.begin(), found.end(), [](const auto & a, const auto & b) {
    return a.first < b.first;
  });
  current_lanelets->reserve(found.size());
  for (const auto & index_and_lanelet : found) {
    current_lanelets->push_back(index_and_lanelet.second);
  }
  return !current_lanelets->empty();
}

// pose callback is not used if use_odom_ is true
void CalculatorNode::poseCallback(const PoseStamped::ConstSharedPtr msg)
{
  // If map is empty or use odom flag do nothing
  if (!map_loaded_ ||use_odom_) {
    return;
  }

  geometry_msgs::msg::Pose query_pose = msg->pose;

  lanelet::ConstLanelets current_lanelets;
  std::cout << "pose cb" << std::endl;
  if (findCurrentLanelets(query_pose, &current_lanelets)) {
    // do something
    updateVehiclePoses(query_pose);



    calcLeftOrRightOffsetsCached(current_lanelets.front(), query_pose);
  }
}

void CalculatorNode::odomCallback(const Odometry::ConstSharedPtr msg)
{
  // If map is empty or use odom flag do nothing
  if (!map_loaded_ ||!use_odom_) {
    return;
  }

  geometry_msgs::msg::Pose query_pose = msg->pose.pose;
  updateVehiclePoses(query_pose);

  // 車両位置を含むlaneletは1tickにつき1回だけ求め、
  // vehicleIsInsideLane/checkLaneStatus/publishBBOX/appendToCSV で使い回す。
  // findCurrentLanelets()は最初に結果をclearするので、メンバ変数を使い回しても要素は積み上がらない
  // (query::getCurrentLanelets()はclearしないので、直接使う場合は呼び出し前のclear()が必須)。
  has_current_lanelets_by_pose_ = findCurrentLanelets(query_pose, &current_lanelets_by_pose_);

  vehicleIsInsideLane();
  diagnostics_updater_.force_update();
  if (has_current_lanelets_by_pose_) {
    const auto & current_lanelets = current_lanelets_by_pose_;

    // Publish current lanelet IDs as an int32 multi array
    std_msgs::msg::Int32MultiArray msg_ids;


    msg_ids.data.reserve(current_lanelets.size());
    for (const auto& lanelet : current_lanelets) {
      msg_ids.data.push_back(lanelet.id());
    }
    pub_candidate_lanelet_ids_->publish(msg_ids);

    // calc left/right lateral offsets for each pose
    for(auto & pose_pair: position_pose_map_){
      position_offset_map_[pose_pair.first] = calcLeftOrRightOffsetsCached(current_lanelets.front(), pose_pair.second);
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
    }
    pub_driving_lanelet_id_->publish(msg_ids_);
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
  // (odomCallbackで計算済みのcurrent_lanelets_by_pose_を使い回し、再計算しない)
  std::stringstream current_ss;
  if (has_current_lanelets_by_pose_) {
    for (const auto& lanelet : current_lanelets_by_pose_) {
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
  // odomCallbackで計算済みのcurrent_lanelets_by_pose_を使い回し、再計算しない
  if (!has_current_lanelets_by_pose_) {
    is_vehicle_inside_lane_ = false;
    return false;
  }

  bool found_matching_lanelet = false;
  bool is_inside_any = false;
  judging_lanelet_id_.clear();  // Reset at the start

  // Check each current lanelet
  for (const auto& lanelet : current_lanelets_by_pose_) {
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
      auto offsets = calcLeftOrRightOffsetsCached(lanelet, pose);
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
  // vehicleIsInsideLane()はodomCallback内で既に呼ばれ、is_vehicle_inside_lane_に結果が入っている。
  // ここで再計算はしない。
  const bool is_inside = is_vehicle_inside_lane_;

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
  // (vehicleIsInsideLane()はodomCallback内で既に呼ばれているため再計算しない)
  const bool is_inside = is_vehicle_inside_lane_;
  const int is_inside_int = is_inside ? 1 : 0;
  file_stream_ << is_inside_int << std::endl;
  return true;
}

} // namespace lane_width_calculator

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(lane_width_calculator::CalculatorNode)