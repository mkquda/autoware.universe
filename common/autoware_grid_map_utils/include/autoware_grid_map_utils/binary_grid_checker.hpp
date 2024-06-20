// Copyright 2024 Tier IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef AUTOWARE_GRID_MAP_UTILS__BINARY_GRID_CHECKER_HPP_
#define AUTOWARE_GRID_MAP_UTILS__BINARY_GRID_CHECKER_HPP_

#include "grid_map_core/TypeDefs.hpp"


#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <grid_map_core/GridMap.hpp>
#include <grid_map_core/GridMapMath.hpp>
#include <grid_map_core/Polygon.hpp>
#include <tier4_autoware_utils/geometry/boost_geometry.hpp>
#include <tier4_autoware_utils/geometry/boost_polygon_utils.hpp>

#include <utility>
#include <vector>
#include <memory>

namespace autoware::grid_map_utils
{
using autoware_perception_msgs::msg::PredictedObjects;
using tier4_autoware_utils::MultiPolygon2d;
using tier4_autoware_utils::Polygon2d;
using tier4_autoware_utils::Box2d;
using tier4_autoware_utils::Point2d;
using grid_map::Index;

enum Corners { TopLeft, TopRight, BottomRight, BottomLeft };

class BinaryGridChecker {
public:

    void fillPrimitives (
        grid_map::GridMap& grid_map,
        const Index& start_index,
        const grid_map::Size& size,
        const std::string & layer_name,
        const double grid_max_value,
        const double grid_min_value
    );

    void fillObjects (
        grid_map::GridMap& grid_map,
        const Index& start_index,
        const grid_map::Size& size,
        const std::string & layer_name,
        const double grid_max_value,
        const double grid_min_value,
        const PredictedObjects::ConstSharedPtr in_objects
    );

    void setPrimitivesPolygons(const std::vector<std::vector<geometry_msgs::msg::Point>>& primitives_points);

private:
    Point2d getVertex(const grid_map::GridMap& grid_map, const grid_map::Position ref_position, Corners corner);
    std::vector<Point2d> getVertices(const grid_map::GridMap& grid_map, const Index& index);
    std::vector<Point2d> getVertices(const grid_map::GridMap& grid_map, const Index& start_index, const Index& end_index);

    bool getBox(const grid_map::GridMap& grid_map, const Index& index, Box2d& box);
    bool getBox(const grid_map::GridMap& grid_map, const Index& start_index, const Index& end_index, Box2d& box);

    template<typename Condition>
    void doBinaryCheck(
        grid_map::GridMap& grid_map,
        const Index& start_index,
        const Index& end_index,
        const std::string & layer_name,
        const double condition_true_value,
        const double condition_false_value,
        Condition condition
    );

    MultiPolygon2d getObjectsMultiPolygon(const PredictedObjects::ConstSharedPtr in_objects);

    double grid_resolution;
    MultiPolygon2d primitives_polygons_;

    static constexpr std::array<double, 4> x_vertex_offset_multipliers{-1, 1, 1, -1};
    static constexpr std::array<double, 4> y_vertex_offset_multipliers{1, 1, -1, -1};
};

}  // namespace autoware::grid_map_utils

#endif  // AUTOWARE_GRID_MAP_UTILS__BINARY_GRID_CHECKER_HPP_
