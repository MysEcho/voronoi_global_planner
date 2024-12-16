#ifndef VORONOI_GLOBAL_PLANNER__VORONOI_DIAGRAM_HPP_
#define VORONOI_GLOBAL_PLANNER__VORONOI_DIAGRAM_HPP_

#include <vector>
#include <cmath>
#include <limits>

namespace voronoi_global_planner {

struct Point {
    double x, y;
    Point(double x = 0.0, double y = 0.0) : x(x), y(y) {}
};

class VoronoiDiagram {
public:
    VoronoiDiagram() = default;

    // Generate Voronoi diagram from obstacles
    void generateVoronoiDiagram(const std::vector<Point>& obstacles);

    // Find the nearest Voronoi vertex to a given point
    Point findNearestVoronoiVertex(const Point& point) const;

    // Check if a path through Voronoi vertices is valid
    bool isValidPath(const std::vector<Point>& path) const;

private:
    std::vector<Point> voronoiVertices_;
    std::vector<Point> obstacles_;

    // Helper methods for Voronoi diagram computation
    double euclideanDistance(const Point& p1, const Point& p2) const;
    bool isVoronoiVertex(const Point& vertex) const;
};

} // namespace voronoi_global_planner

#endif // VORONOI_GLOBAL_PLANNER__VORONOI_DIAGRAM_HPP_