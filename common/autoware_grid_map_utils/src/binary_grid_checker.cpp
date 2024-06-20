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

#include "autoware_grid_map_utils/binary_grid_checker.hpp"

#include "grid_map_core/GridMap.hpp"
#include "grid_map_core/Polygon.hpp"
#include "grid_map_core/TypeDefs.hpp"

#include <boost/geometry/algorithms/correct.hpp>
#include <boost/geometry/algorithms/intersects.hpp>

#include <algorithm>
#include <functional>
#include <utility>
#include <iostream>

namespace autoware::grid_map_utils
{

void BinaryGridChecker::setPrimitivesPolygons(const std::vector<std::vector<geometry_msgs::msg::Point>>& primitives_points)
{
    if (primitives_points.empty()) return;
    for (const auto& ring : primitives_points){
        Polygon2d polygon;
        for (const auto& point : ring){
            polygon.outer().push_back(Point2d{point.x, point.y});
        }
        polygon.outer().push_back(Point2d{ring.front().x, ring.front().y});
        boost::geometry::correct(polygon);
        primitives_polygons_.push_back(polygon);
    }
}

Point2d BinaryGridChecker::getVertex(const grid_map::GridMap& grid_map, const grid_map::Position ref_position, Corners corner)
{
    double offset = 0.5 * grid_map.getResolution();
    Point2d vertex;
    vertex[0] = ref_position[0] + x_vertex_offset_multipliers[corner]*offset;
    vertex[1] = ref_position[1] + y_vertex_offset_multipliers[corner]*offset;
    return vertex;
}

std::vector<Point2d> BinaryGridChecker::getVertices(const grid_map::GridMap& grid_map, const Index& index)
{
    std::vector<Point2d> vertices;
    grid_map::Position ref_position;
    if (!grid_map.getPosition(index, ref_position)) return vertices;
    vertices.push_back(getVertex(grid_map, ref_position, Corners::TopLeft));
    vertices.push_back(getVertex(grid_map, ref_position, Corners::TopRight));
    vertices.push_back(getVertex(grid_map, ref_position, Corners::BottomRight));
    vertices.push_back(getVertex(grid_map, ref_position, Corners::BottomLeft));
    return vertices;
}

std::vector<Point2d> BinaryGridChecker::getVertices(const grid_map::GridMap& grid_map, const Index& start_index, const Index& end_index)
{
    std::vector<Point2d> vertices;
    std::array<grid_map::Position, 4> ref_positions;
    if (!grid_map.getPosition({end_index[0], start_index[1]}, ref_positions[Corners::TopLeft]) ||
        !grid_map.getPosition(start_index, ref_positions[Corners::TopRight]) ||
        !grid_map.getPosition({start_index[0], end_index[1]}, ref_positions[Corners::BottomRight]) ||
        !grid_map.getPosition(end_index, ref_positions[Corners::BottomLeft])
    ){
        return vertices;
    }
    vertices.push_back(getVertex(grid_map, ref_positions[Corners::TopLeft], Corners::TopLeft));
    vertices.push_back(getVertex(grid_map, ref_positions[Corners::TopRight], Corners::TopRight));
    vertices.push_back(getVertex(grid_map, ref_positions[Corners::BottomRight], Corners::BottomRight));
    vertices.push_back(getVertex(grid_map, ref_positions[Corners::BottomLeft], Corners::BottomLeft));
    return vertices;
}

bool BinaryGridChecker::getBox(const grid_map::GridMap& grid_map, const Index& index, Box2d& box)
{
    auto vertices = getVertices(grid_map, index);
    if (vertices.empty()) return false;
    box.min_corner() = vertices[Corners::BottomLeft];
    box.max_corner() = vertices[Corners::TopRight];
    boost::geometry::correct(box);
    return true;
}

bool BinaryGridChecker::getBox(const grid_map::GridMap& grid_map, const Index& start_index, const Index& end_index, Box2d& box)
{
    auto vertices = getVertices(grid_map, start_index, end_index);
    if (vertices.empty()) return false;
    box.min_corner() = vertices[Corners::BottomLeft];
    box.max_corner() = vertices[Corners::TopRight];
    boost::geometry::correct(box);
    return true;
}

template<typename Condition>
void BinaryGridChecker::doBinaryCheck(
    grid_map::GridMap& grid_map,
    const Index& start_index,
    const Index& end_index,
    const std::string & layer_name,
    const double condition_true_value,
    const double condition_false_value,
    Condition condition
){
    // if region is a single grid cell
    if (start_index[0] == end_index[0] && start_index[1] == end_index[1]){
        Box2d box;
        if (!getBox(grid_map, start_index, box)) return;
        if (condition(box)) grid_map.at(layer_name, start_index) = condition_true_value;
        else grid_map.at(layer_name, start_index) = condition_false_value;
        return;
    }

    // Divide region into sections
    std::vector<std::array<Index, 2>> sectionsIndices;
    sectionsIndices.reserve(4);
    if (start_index[0] == end_index[0]) { // if region is a single row
        uint8_t cols_mid = start_index[1] + ((end_index[1] - start_index[1]) / 2);
        sectionsIndices.push_back(std::array<Index, 2>{start_index, Index{start_index[0], cols_mid}});
        sectionsIndices.push_back(std::array<Index, 2>{Index{start_index[0], cols_mid + 1}, end_index});
    } else if (start_index[1] == end_index[1]) { // if region is a single column
        uint8_t rows_mid = start_index[0] + ((end_index[0] - start_index[0]) / 2);
        sectionsIndices.push_back(std::array<Index, 2>{start_index, Index{rows_mid, start_index[1]}});
        sectionsIndices.push_back(std::array<Index, 2>{Index{rows_mid + 1, start_index[1]}, end_index});
    } else {
        uint8_t rows_mid = start_index[0] + ((end_index[0] - start_index[0]) / 2);
        uint8_t cols_mid = start_index[1] + ((end_index[1] - start_index[1]) / 2);
        sectionsIndices.push_back(std::array<Index, 2>{start_index, Index{rows_mid, cols_mid}});
        sectionsIndices.push_back(std::array<Index, 2>{Index{rows_mid + 1, cols_mid + 1}, end_index});
        sectionsIndices.push_back(std::array<Index, 2>{Index{rows_mid + 1, start_index[1]}, Index{end_index[0], cols_mid}});
        sectionsIndices.push_back(std::array<Index, 2>{Index{start_index[0], cols_mid + 1}, Index{rows_mid, end_index[1]}});
    }

    for (const auto& section : sectionsIndices) {
        Box2d box;
        if (!getBox(grid_map, section[0], section[1], box)) continue;
        if (condition(box)) {
            for (int i = section[0][0]; i <= section[1][0]; ++i){
                for (int j = section[0][1]; j <= section[1][1]; ++j){
                    grid_map.at(layer_name, {i,j}) = condition_true_value;
                }
            }
            doBinaryCheck(grid_map, section[0], section[1], layer_name, condition_true_value, condition_false_value, condition);
        } else {
            for (int i = section[0][0]; i <= section[1][0]; ++i){
                for (int j = section[0][1]; j <= section[1][1]; ++j){
                    grid_map.at(layer_name, {i,j}) = condition_false_value;
                }
            }
        }
    }
}

void BinaryGridChecker::fillPrimitives (
    grid_map::GridMap& grid_map,
    const Index& start_index,
    const grid_map::Size& size,
    const std::string & layer_name,
    const double grid_max_value,
    const double grid_min_value
) {
    if(primitives_polygons_.empty() || size[0] < 1 || size[1] < 1) return;
    Index end_index{start_index[0] + size[0] - 1, start_index[1] + size[1] - 1};
    const auto condition = [this](const Box2d& box){
        return boost::geometry::intersects(box, primitives_polygons_);
    };
    doBinaryCheck(grid_map, start_index, end_index, layer_name, grid_min_value, grid_max_value, condition);
}

void BinaryGridChecker::fillObjects (
    grid_map::GridMap& grid_map,
    const Index& start_index,
    const grid_map::Size& size,
    const std::string & layer_name,
    const double grid_max_value,
    const double grid_min_value,
    const PredictedObjects::ConstSharedPtr in_objects
) {
    if(in_objects->objects.empty() || size[0] < 1 || size[1] < 1) return;
    Index end_index{start_index[0] + size[0] - 1, start_index[1] + size[1] - 1};
    const auto objectsMultiPolygon = getObjectsMultiPolygon(in_objects);
    const auto condition = [&objectsMultiPolygon](const Box2d& box){
        return boost::geometry::intersects(box, objectsMultiPolygon);
    };
    doBinaryCheck(grid_map, start_index, end_index, layer_name, grid_max_value, grid_min_value, condition);
}

MultiPolygon2d BinaryGridChecker::getObjectsMultiPolygon(
    const PredictedObjects::ConstSharedPtr in_objects
){
    MultiPolygon2d polygons;
    for (const auto& object : in_objects->objects) {
        polygons.emplace_back(std::move(tier4_autoware_utils::toPolygon2d(object)));
    }
    return polygons;
}

}  // namespace autoware::grid_map_utils