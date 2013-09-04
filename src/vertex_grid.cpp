#include "vertex_grid.hpp"
#include <boost/foreach.hpp>
#include <iostream>

namespace graph_slam 
{

VertexGrid::VertexGrid(double grid_size_x, double grid_size_y, double cell_resolution, size_t max_vertices_per_cell) : 
                        offset_x(-0.5 * grid_size_x), offset_y(-0.5 * grid_size_y), 
                        scale_x(cell_resolution), scale_y(cell_resolution), max_vertices_per_cell(max_vertices_per_cell)
{
    cell_size_x = (size_t)(grid_size_x / cell_resolution);
    cell_size_y = (size_t)(grid_size_y / cell_resolution);
    grid.resize( boost::extents[cell_size_y][cell_size_x] );
}

bool VertexGrid::addVertex(int vertex_id, Eigen::Vector3d vertex_position)
{
    try
    {
        std::vector<int>& vertex_cell = getCell(vertex_position[0], vertex_position[1]);
        vertex_cell.push_back(vertex_id);
        return true;
    }
    catch (std::runtime_error e)
    {
        std::cerr << "vertex position (" << vertex_position[0] << ", " << vertex_position[1] << ") is out of vertex grid." << std::endl;
    }
    return false;
}

void VertexGrid::removeVertices(std::vector<int>& vertices_removed)
{
    vertices_removed.clear();

    for(size_t y = 0; y != cell_size_y; y++)
    {
        for(size_t x = 0; x != cell_size_x; x++)
        {
            std::vector<int>& cell = grid[y][x];
            while(cell.size() > max_vertices_per_cell)
            {
                vertices_removed.push_back(cell.front());
                cell.erase(cell.begin());
            }
        }
    }
}

std::vector<int>& VertexGrid::getCell(double x, double y)
{
    size_t xi, yi;
    if (!toGrid(x, y, xi, yi))
        throw std::runtime_error("provided coordinates are out of the grid");
    return grid[yi][xi];
}

bool VertexGrid::toGrid(double x, double y, size_t& xi, size_t& yi)
{
    size_t xi_t = floor((x-offset_x)/scale_x);
    size_t yi_t = floor((y-offset_y)/scale_y);

    if( 0 <= xi_t && xi_t < cell_size_x && 0 <= yi_t && yi_t < cell_size_y )
    {
        xi = xi_t;
        yi = yi_t;
        return true;
    }
    return false;
}

}
