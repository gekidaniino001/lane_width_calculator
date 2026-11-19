// テスト対象: lane_width_calculator::CalculatorNode
//
// 変更点:
//  - getCurrentLanelets（マップ全体を走査する重い処理）を1コールバックにつき1回だけ計算し、
//    vehicleIsInsideLane/checkLaneStatus/publishBBOX/appendToCSVで使い回すようにした
//    (current_lanelets_by_pose_ / has_current_lanelets_by_pose_ メンバを追加)
//  - driving_lanelet_idsトピックが for ループの中でpublishされていたバグ(候補レーンの数だけ
//    冗長にpublishしていた)を、ループの外で1回だけpublishするよう修正
//  - leftBound2d/rightBound2dのPoint配列変換をmapCallback時に1回だけ行いキャッシュする
//    ようにした(boundary_path_cache_ / calcLeftOrRightOffsetsCached)。以前は毎odom
//    コールバックの度に、レーン数×5点(center等)×2(left/right)ぶん変換し直していた。
//  - 車両位置を含むlaneletの検索を、全laneletの線形走査(query::getCurrentLanelets)から、
//    地図のR-treeで候補を絞ってから同じ内外判定をする方式(findCurrentLanelets)に変えた。
//    結果は要素・順序とも従来と同じ(lanelet_index_by_id_でconst_lanelets_の並びに揃える)。
//
// ヘッダのメンバ・メソッドが全てpublicなので、コールバックを直接呼び出して
// 内部状態(current_lanelets_by_pose_ 等)を直接検証できる。

#include "lane_width_calculator/node.hpp"
#include "gtest/gtest.h"

#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

using lane_width_calculator::CalculatorNode;
using lanelet::Lanelet;
using lanelet::LineString3d;
using lanelet::Point3d;
using lanelet::utils::getId;

namespace
{

rclcpp::NodeOptions makeNodeOptions()
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("vehicle_width", 1.0);
  options.append_parameter_override("vehicle_length", 2.0);
  options.append_parameter_override("vehicle_height", 1.5);
  options.append_parameter_override("use_odom", true);
  options.append_parameter_override("save_to_csv", false);
  options.append_parameter_override("save_file_name", std::string("/tmp/lane_width_calculator_test.csv"));
  return options;
}

// x=[0,20], y=[-half_width, half_width] の直進レーンを1本作る
Lanelet makeStraightLanelet(double half_width)
{
  Point3d p_left_start(getId(), 0.0, half_width, 0.0);
  Point3d p_left_end(getId(), 20.0, half_width, 0.0);
  Point3d p_right_start(getId(), 0.0, -half_width, 0.0);
  Point3d p_right_end(getId(), 20.0, -half_width, 0.0);

  LineString3d left_bound(getId(), {p_left_start, p_left_end});
  LineString3d right_bound(getId(), {p_right_start, p_right_end});

  Lanelet lanelet(getId(), left_bound, right_bound);
  lanelet.attributes()[lanelet::AttributeName::Subtype] = lanelet::AttributeValueString::Road;
  lanelet.attributes()[lanelet::AttributeName::Location] = lanelet::AttributeValueString::Urban;
  return lanelet;
}

// 左右境界線の点列(x, y)から任意形状のレーンを1本作る
Lanelet makeLaneletFromPoints(
  const std::vector<std::pair<double, double>> & left_points,
  const std::vector<std::pair<double, double>> & right_points)
{
  std::vector<Point3d> left;
  for (const auto & p : left_points) {
    left.emplace_back(getId(), p.first, p.second, 0.0);
  }
  std::vector<Point3d> right;
  for (const auto & p : right_points) {
    right.emplace_back(getId(), p.first, p.second, 0.0);
  }
  Lanelet lanelet(getId(), LineString3d(getId(), left), LineString3d(getId(), right));
  lanelet.attributes()[lanelet::AttributeName::Subtype] = lanelet::AttributeValueString::Road;
  lanelet.attributes()[lanelet::AttributeName::Location] = lanelet::AttributeValueString::Urban;
  return lanelet;
}

std::vector<lanelet::Id> toIds(const lanelet::ConstLanelets & lanelets)
{
  std::vector<lanelet::Id> ids;
  for (const auto & ll : lanelets) {
    ids.push_back(ll.id());
  }
  return ids;
}

geometry_msgs::msg::Pose makePose(double x, double y)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.orientation.w = 1.0;
  return pose;
}

autoware_auto_mapping_msgs::msg::HADMapBin::SharedPtr toMapMsg(
  const std::vector<Lanelet> & lanelets)
{
  auto map_ptr = std::make_shared<lanelet::LaneletMap>();
  for (const auto & ll : lanelets) {
    map_ptr->add(ll);
  }
  auto msg = std::make_shared<autoware_auto_mapping_msgs::msg::HADMapBin>();
  lanelet::utils::conversion::toBinMsg(map_ptr, msg.get());
  return msg;
}

autoware_auto_planning_msgs::msg::PathWithLaneId::SharedPtr makePathMsg(
  const std::vector<int64_t> & lane_ids)
{
  auto msg = std::make_shared<autoware_auto_planning_msgs::msg::PathWithLaneId>();
  autoware_auto_planning_msgs::msg::PathPointWithLaneId point;
  point.lane_ids = lane_ids;
  msg->points.push_back(point);
  return msg;
}

nav_msgs::msg::Odometry::SharedPtr makeOdomMsg(double x, double y, double yaw = 0.0)
{
  auto msg = std::make_shared<nav_msgs::msg::Odometry>();
  msg->pose.pose.position.x = x;
  msg->pose.pose.position.y = y;
  msg->pose.pose.position.z = 0.0;
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  msg->pose.pose.orientation = tf2::toMsg(q);
  return msg;
}

}  // namespace

// 各テストの前後でrclcppを初期化・終了する。ASSERT_*でテスト本体が途中で抜けても
// TearDown()は必ず呼ばれるので、1つのテストの失敗で後続のテストまでrclcpp::init()の
// 二重呼び出しで巻き添えに落ちることがない。
class LaneWidthCalculatorNode : public ::testing::Test
{
protected:
  void SetUp() override { rclcpp::init(0, nullptr); }
  void TearDown() override { rclcpp::shutdown(); }
};

// 実バグの回帰テスト: lanelet::utils::query::getCurrentLanelets()は結果をpush_backするだけで
// 内部でclearしない。current_lanelets_by_pose_はコールバックをまたいで使い回すメンバ変数なので、
// 呼び出し前にclear()し忘れると、同じ場所に居続けるだけでtickの度に要素が増え続けてしまう
// (実際に実車のrosbag再生でCPU使用率が数分でジワジワ100%まで張り付くのを確認して発覚したバグ)。
TEST_F(LaneWidthCalculatorNode, CurrentLaneletsByPoseDoesNotGrowAcrossTicks)
{
  auto node = std::make_shared<CalculatorNode>(makeNodeOptions());

  const Lanelet lanelet = makeStraightLanelet(2.0);
  node->mapCallback(toMapMsg({lanelet}));
  node->pathCallback(makePathMsg({lanelet.id()}));

  // 同じ場所(レーン内)で何度もodomCallbackを呼んでも、current_lanelets_by_pose_のサイズは
  // 常に「今のポーズにマッチするlanelet数(=1)」のままで、増え続けてはいけない
  for (int i = 0; i < 50; ++i) {
    node->odomCallback(makeOdomMsg(10.0, 0.0));
    ASSERT_EQ(node->current_lanelets_by_pose_.size(), 1u) << "tick " << i;
  }
}

TEST_F(LaneWidthCalculatorNode, VehicleInsideLaneIsDetected)
{
  auto node = std::make_shared<CalculatorNode>(makeNodeOptions());

  const Lanelet lanelet = makeStraightLanelet(2.0);  // 幅4m
  node->mapCallback(toMapMsg({lanelet}));
  node->pathCallback(makePathMsg({lanelet.id()}));

  // 車両サイズ(width=1.0, length=2.0)がレーン内に収まる位置
  node->odomCallback(makeOdomMsg(10.0, 0.0));

  EXPECT_TRUE(node->has_current_lanelets_by_pose_);
  ASSERT_EQ(node->current_lanelets_by_pose_.size(), 1u);
  EXPECT_EQ(node->current_lanelets_by_pose_.front().id(), lanelet.id());
  EXPECT_TRUE(node->is_vehicle_inside_lane_);
}

TEST_F(LaneWidthCalculatorNode, VehicleOutsideLaneIsDetected)
{
  auto node = std::make_shared<CalculatorNode>(makeNodeOptions());

  const Lanelet lanelet = makeStraightLanelet(2.0);  // 幅4m -> y=+-2.0が境界
  node->mapCallback(toMapMsg({lanelet}));
  node->pathCallback(makePathMsg({lanelet.id()}));

  // レーンのポリゴン自体からも外れる位置(y=3.5)
  node->odomCallback(makeOdomMsg(10.0, 3.5));

  EXPECT_FALSE(node->has_current_lanelets_by_pose_);
  EXPECT_TRUE(node->current_lanelets_by_pose_.empty());
  EXPECT_FALSE(node->is_vehicle_inside_lane_);
}

// checkLaneStatus (diagnostic_updater経由で呼ばれる関数) が、内部で再計算せずに
// odomCallbackでキャッシュされたcurrent_lanelets_by_pose_をそのまま使えていることを確認する。
TEST_F(LaneWidthCalculatorNode, CheckLaneStatusUsesCachedCurrentLanelets)
{
  auto node = std::make_shared<CalculatorNode>(makeNodeOptions());

  const Lanelet lanelet = makeStraightLanelet(2.0);
  node->mapCallback(toMapMsg({lanelet}));
  node->pathCallback(makePathMsg({lanelet.id()}));
  node->odomCallback(makeOdomMsg(10.0, 0.0));

  diagnostic_updater::DiagnosticStatusWrapper stat;
  node->checkLaneStatus(stat);

  bool found_current_lanelet_id = false;
  const std::string expected_id_str = std::to_string(lanelet.id());
  for (const auto & value : stat.values) {
    if (value.key == "current_lanelet_ids") {
      found_current_lanelet_id = (value.value.find(expected_id_str) != std::string::npos);
    }
  }
  EXPECT_TRUE(found_current_lanelet_id);
}

// 修正前は driving_lanelet_ids が for ループの中でpublishされており、現在レーン候補の数だけ
// (今回は2つ)冗長にpublishされていた。修正後は1odomCallbackにつき1回だけpublishされることを確認する。
TEST_F(LaneWidthCalculatorNode, DrivingLaneletIdsPublishedOnlyOncePerTick)
{
  auto node = std::make_shared<CalculatorNode>(makeNodeOptions());

  // 同じ場所に重なる2本のレーンを用意し、current_lanelets候補を2つにする
  const Lanelet lanelet_a = makeStraightLanelet(2.0);
  const Lanelet lanelet_b = makeStraightLanelet(2.0);
  node->mapCallback(toMapMsg({lanelet_a, lanelet_b}));
  node->pathCallback(makePathMsg({lanelet_a.id(), lanelet_b.id()}));

  int receive_count = 0;
  auto sub = node->create_subscription<std_msgs::msg::Int32MultiArray>(
    "driving_lanelet_ids", 10,
    [&receive_count](std_msgs::msg::Int32MultiArray::ConstSharedPtr) { ++receive_count; });

  node->odomCallback(makeOdomMsg(10.0, 0.0));

  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (receive_count == 0 && std::chrono::steady_clock::now() < deadline) {
    rclcpp::spin_some(node);
    rclcpp::sleep_for(std::chrono::milliseconds(10));
  }
  // 念のためもう少し待ち、追加のpublishが来ないことを確認する
  const auto extra_deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(200);
  while (std::chrono::steady_clock::now() < extra_deadline) {
    rclcpp::spin_some(node);
    rclcpp::sleep_for(std::chrono::milliseconds(10));
  }

  ASSERT_EQ(node->current_lanelets_by_pose_.size(), 2u)
    << "テスト前提: current_lanelets候補が2つであること";
  EXPECT_EQ(receive_count, 1);
}

// mapCallback時にboundary_path_cache_が各lanelet分正しく作られていることを確認する。
TEST_F(LaneWidthCalculatorNode, BoundaryPathCacheIsPopulatedOnMapCallback)
{
  auto node = std::make_shared<CalculatorNode>(makeNodeOptions());

  const Lanelet lanelet_a = makeStraightLanelet(2.0);
  const Lanelet lanelet_b = makeStraightLanelet(3.0);
  node->mapCallback(toMapMsg({lanelet_a, lanelet_b}));

  ASSERT_EQ(node->boundary_path_cache_.size(), 2u);
  for (const auto & ll : {lanelet_a, lanelet_b}) {
    const auto it = node->boundary_path_cache_.find(ll.id());
    ASSERT_NE(it, node->boundary_path_cache_.end());
    // 直進レーン(2点の境界線)なので、変換後も2点のままであること
    EXPECT_EQ(it->second.left.size(), 2u);
    EXPECT_EQ(it->second.right.size(), 2u);
  }
}

// calcLeftOrRightOffsetsCached が、キャッシュされたPoint配列に対してmotion_utils::calcLateralOffsetを
// 呼んだ場合と完全に同じ値を返すことを確認する(=キャッシュ化してもロジック・数値が変わらないこと)。
TEST_F(LaneWidthCalculatorNode, CachedOffsetsMatchDirectCalculation)
{
  auto node = std::make_shared<CalculatorNode>(makeNodeOptions());

  const double half_width = 2.0;
  const Lanelet lanelet = makeStraightLanelet(half_width);
  node->mapCallback(toMapMsg({lanelet}));

  // makeStraightLanelet()と全く同じ境界線を手動で再構築し、期待値をmotion_utils側で直接計算する
  std::vector<geometry_msgs::msg::Point> left_bound_path;
  left_bound_path.push_back(tier4_autoware_utils::createPoint(0.0, half_width, 0.0));
  left_bound_path.push_back(tier4_autoware_utils::createPoint(20.0, half_width, 0.0));
  std::vector<geometry_msgs::msg::Point> right_bound_path;
  right_bound_path.push_back(tier4_autoware_utils::createPoint(0.0, -half_width, 0.0));
  right_bound_path.push_back(tier4_autoware_utils::createPoint(20.0, -half_width, 0.0));

  for (const double y : {0.0, 1.0, -1.0}) {
    geometry_msgs::msg::Pose pose;
    pose.position.x = 10.0;
    pose.position.y = y;
    pose.orientation.w = 1.0;

    const double expected_left = motion_utils::calcLateralOffset(left_bound_path, pose.position);
    const double expected_right = motion_utils::calcLateralOffset(right_bound_path, pose.position);

    const auto cached_offsets = node->calcLeftOrRightOffsetsCached(lanelet, pose);

    EXPECT_DOUBLE_EQ(cached_offsets[0], expected_left) << "y=" << y;
    EXPECT_DOUBLE_EQ(cached_offsets[1], expected_right) << "y=" << y;
  }
}

// キャッシュに存在しないlanelet(マップに登録されていないもの)を渡した場合でも
// フォールバックでクラッシュせず計算できることを確認する。
TEST_F(LaneWidthCalculatorNode, CalcOffsetsFallsBackWhenNotCached)
{
  auto node = std::make_shared<CalculatorNode>(makeNodeOptions());

  const Lanelet lanelet_in_map = makeStraightLanelet(2.0);
  node->mapCallback(toMapMsg({lanelet_in_map}));

  // マップに登録していない(=boundary_path_cache_に無い)レーン
  const Lanelet lanelet_not_cached = makeStraightLanelet(2.0);
  ASSERT_EQ(
    node->boundary_path_cache_.find(lanelet_not_cached.id()), node->boundary_path_cache_.end());

  geometry_msgs::msg::Pose pose;
  pose.position.x = 10.0;
  pose.position.y = 0.0;
  pose.orientation.w = 1.0;

  const auto offsets = node->calcLeftOrRightOffsetsCached(lanelet_not_cached, pose);
  // 幅4mレーンの中心(y=0)なので、左右オフセットはおよそ+-2m前後になるはず
  EXPECT_NEAR(std::abs(offsets[0]), 2.0, 1e-6);
  EXPECT_NEAR(std::abs(offsets[1]), 2.0, 1e-6);
}

// ---- findCurrentLanelets(R-treeで候補を絞る版) ----

// findCurrentLanelets()の結果が、従来のquery::getCurrentLanelets(const_lanelets_, ...)と
// 戻り値・要素・順序まで一致することを確認する。重なったレーン・曲がったレーン・離れたレーンを含む地図の
// 0.25m間隔の格子点で比べる(格子は境界線上や頂点ちょうどの点も含む)。
TEST_F(LaneWidthCalculatorNode, FindCurrentLaneletsMatchesLinearScan)
{
  auto node = std::make_shared<CalculatorNode>(makeNodeOptions());

  const Lanelet straight_a = makeStraightLanelet(2.0);  // x=[0,20], y=[-2,2]
  const Lanelet straight_b = makeStraightLanelet(2.0);  // straight_aと完全に重なる
  const Lanelet shifted =
    makeLaneletFromPoints({{5.0, 3.0}, {25.0, 3.0}}, {{5.0, -1.0}, {25.0, -1.0}});
  const Lanelet curved = makeLaneletFromPoints(
    {{0.0, 4.0}, {5.0, 5.0}, {10.0, 4.5}, {15.0, 6.0}, {20.0, 5.0}},
    {{0.0, 1.0}, {5.0, 2.0}, {10.0, 1.5}, {15.0, 3.0}, {20.0, 2.0}});
  const Lanelet far_away =
    makeLaneletFromPoints({{100.0, 2.0}, {120.0, 2.0}}, {{100.0, -2.0}, {120.0, -2.0}});
  node->mapCallback(toMapMsg({straight_a, straight_b, shifted, curved, far_away}));

  size_t checked = 0;
  size_t found_multiple = 0;
  for (double x = -5.0; x <= 130.0; x += 0.25) {
    for (double y = -6.0; y <= 8.0; y += 0.25) {
      const auto pose = makePose(x, y);
      lanelet::ConstLanelets expected;
      const bool expected_found =
        lanelet::utils::query::getCurrentLanelets(node->const_lanelets_, pose, &expected);
      lanelet::ConstLanelets actual;
      const bool actual_found = node->findCurrentLanelets(pose, &actual);
      ASSERT_EQ(actual_found, expected_found) << "x=" << x << " y=" << y;
      ASSERT_EQ(toIds(actual), toIds(expected)) << "x=" << x << " y=" << y;
      ++checked;
      if (expected.size() >= 2) {
        ++found_multiple;
      }
    }
  }
  // テストの前提: 十分な数の点を比べていて、複数のレーンが重なる点も多く含むこと
  EXPECT_GT(checked, 10000u);
  EXPECT_GT(found_multiple, 100u);
}

// 重なったレーンが複数見つかるとき、並びがconst_lanelets_の順(=従来の線形走査の順)になること。
// odomCallbackはfront()を基準に左右オフセットを計算し、IDの配列もこの順でpublishするので、順序も変えてはいけない。
TEST_F(LaneWidthCalculatorNode, FindCurrentLaneletsKeepsConstLaneletsOrder)
{
  auto node = std::make_shared<CalculatorNode>(makeNodeOptions());

  const Lanelet a = makeStraightLanelet(2.0);
  const Lanelet b = makeStraightLanelet(2.0);
  const Lanelet c = makeStraightLanelet(2.0);
  node->mapCallback(toMapMsg({a, b, c}));

  lanelet::ConstLanelets actual;
  ASSERT_TRUE(node->findCurrentLanelets(makePose(10.0, 0.0), &actual));
  // const_lanelets_の並びは地図の内部順(idの昇順とは限らない)。それと一致すること
  EXPECT_EQ(toIds(actual), toIds(node->const_lanelets_));
}

// 出力先に前の結果が残っていても積み上がらないこと(query::getCurrentLanelets()はclearしないため、
// 以前それが原因でCPUが張り付いた)。どのレーンにも入らない点ではfalseと空を返すこと。
// NaN/無限大の位置でも、従来のgetCurrentLanelets()と同じ結果になること。
TEST_F(LaneWidthCalculatorNode, FindCurrentLaneletsClearsOutputAndHandlesOutside)
{
  auto node = std::make_shared<CalculatorNode>(makeNodeOptions());

  const Lanelet lanelet = makeStraightLanelet(2.0);
  node->mapCallback(toMapMsg({lanelet}));

  lanelet::ConstLanelets result{node->const_lanelets_.front(), node->const_lanelets_.front()};
  ASSERT_TRUE(node->findCurrentLanelets(makePose(10.0, 0.0), &result));
  EXPECT_EQ(toIds(result), std::vector<lanelet::Id>{lanelet.id()});

  EXPECT_FALSE(node->findCurrentLanelets(makePose(10.0, 50.0), &result));
  EXPECT_TRUE(result.empty());

  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  for (const auto & xy : std::vector<std::pair<double, double>>{
         {nan, 0.0}, {10.0, nan}, {nan, nan}, {inf, 0.0}, {-inf, 0.0}, {10.0, inf}}) {
    const auto pose = makePose(xy.first, xy.second);
    lanelet::ConstLanelets expected;
    const bool expected_found =
      lanelet::utils::query::getCurrentLanelets(node->const_lanelets_, pose, &expected);
    lanelet::ConstLanelets actual;
    const bool actual_found = node->findCurrentLanelets(pose, &actual);
    EXPECT_EQ(actual_found, expected_found) << "x=" << xy.first << " y=" << xy.second;
    EXPECT_EQ(toIds(actual), toIds(expected)) << "x=" << xy.first << " y=" << xy.second;
  }
}

// 地図を読み直したとき、lanelet_index_by_id_が新しい地図で作り直され、
// 新しい地図に対して検索できること(古い地図のレーンが残らないこと)。
TEST_F(LaneWidthCalculatorNode, LaneletIndexIsRebuiltOnMapReload)
{
  auto node = std::make_shared<CalculatorNode>(makeNodeOptions());

  const Lanelet old_lanelet = makeStraightLanelet(2.0);  // x=[0,20]
  node->mapCallback(toMapMsg({old_lanelet}));

  const Lanelet new_a =
    makeLaneletFromPoints({{50.0, 2.0}, {70.0, 2.0}}, {{50.0, -2.0}, {70.0, -2.0}});
  const Lanelet new_b =
    makeLaneletFromPoints({{60.0, 3.0}, {80.0, 3.0}}, {{60.0, -1.0}, {80.0, -1.0}});
  node->mapCallback(toMapMsg({new_a, new_b}));

  ASSERT_EQ(node->const_lanelets_.size(), 2u);
  ASSERT_EQ(node->lanelet_index_by_id_.size(), 2u);
  for (size_t i = 0; i < node->const_lanelets_.size(); ++i) {
    EXPECT_EQ(node->lanelet_index_by_id_.at(node->const_lanelets_[i].id()), i);
  }
  EXPECT_EQ(node->lanelet_index_by_id_.count(old_lanelet.id()), 0u);

  lanelet::ConstLanelets result;
  EXPECT_FALSE(node->findCurrentLanelets(makePose(10.0, 0.0), &result));  // 古い地図の場所
  ASSERT_TRUE(node->findCurrentLanelets(makePose(65.0, 0.0), &result));   // new_aとnew_bが重なる場所
  lanelet::ConstLanelets expected;
  lanelet::utils::query::getCurrentLanelets(node->const_lanelets_, makePose(65.0, 0.0), &expected);
  EXPECT_EQ(toIds(result), toIds(expected));
  EXPECT_EQ(result.size(), 2u);
}

// レーン数が多い地図では、R-treeが返す候補の順番はconst_lanelets_の並びと一致しない
// (実地図で並べ直しを外すと、結果の順序が食い違う点が大量に出ることを確認済み)。
// 格子状に重ねた400本のレーンで、戻り値・要素・順序まで従来の線形走査と一致することを確認する。
TEST_F(LaneWidthCalculatorNode, FindCurrentLaneletsMatchesLinearScanOnManyOverlappingLanelets)
{
  auto node = std::make_shared<CalculatorNode>(makeNodeOptions());

  // 長さ10m・幅4mのレーンを、x方向6m・y方向3m間隔で20x20本並べる(1点に最大4本重なる)
  std::vector<Lanelet> lanelets;
  for (int i = 0; i < 20; ++i) {
    for (int j = 0; j < 20; ++j) {
      const double x0 = 6.0 * i;
      const double yc = 3.0 * j;
      lanelets.push_back(makeLaneletFromPoints(
        {{x0, yc + 2.0}, {x0 + 10.0, yc + 2.0}}, {{x0, yc - 2.0}, {x0 + 10.0, yc - 2.0}}));
    }
  }
  node->mapCallback(toMapMsg(lanelets));
  ASSERT_EQ(node->const_lanelets_.size(), 400u);

  size_t mismatches = 0;
  size_t found_multiple = 0;
  for (double x = -2.0; x <= 126.0; x += 1.0) {
    for (double y = -4.0; y <= 62.0; y += 1.0) {
      const auto pose = makePose(x, y);
      lanelet::ConstLanelets expected;
      const bool expected_found =
        lanelet::utils::query::getCurrentLanelets(node->const_lanelets_, pose, &expected);
      lanelet::ConstLanelets actual;
      const bool actual_found = node->findCurrentLanelets(pose, &actual);
      if (actual_found != expected_found || toIds(actual) != toIds(expected)) {
        if (mismatches == 0) {
          ADD_FAILURE() << "first mismatch at x=" << x << " y=" << y;
        }
        ++mismatches;
      }
      if (expected.size() >= 2) {
        ++found_multiple;
      }
    }
  }
  EXPECT_EQ(mismatches, 0u);
  // テストの前提: 複数のレーンが重なる点を十分に含むこと
  EXPECT_GT(found_multiple, 1000u);
}
