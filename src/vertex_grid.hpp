#ifndef GRAPH_SLAM_VERTEX_GRID_HPP
#define GRAPH_SLAM_VERTEX_GRID_HPP

#include <vector>
#include <boost/multi_array.hpp>
#include <Eigen/Dense>

namespace graph_slam 
{

class VertexGrid
{
public:
    typedef boost::multi_array< std::vector<int>, 2 > VertexGridData;

    VertexGrid(double grid_size_x, double grid_size_y, double cell_resolution, size_t max_vertices_per_cell);
    ~VertexGrid() {}

    bool addVertex(int vertex_id, Eigen::Vector3d vertex_position);
    void removeVertices(std::vector<int>& vertices_removed);
    void setMaxVerticesPerCell(size_t max_vertices_per_cell) {this->max_vertices_per_cell = max_vertices_per_cell;}
    size_t getMaxVerticesPerCell() const {return max_vertices_per_cell;}

protected:
    std::vector<int>& getCell(double x, double y);
    bool toGrid(double x, double y, size_t& xi, size_t& yi);

    VertexGridData grid;
    double offset_x, offset_y;
    double scale_x, scale_y;
    size_t cell_size_x, cell_size_y;
    size_t max_vertices_per_cell;
};

}

#endif
