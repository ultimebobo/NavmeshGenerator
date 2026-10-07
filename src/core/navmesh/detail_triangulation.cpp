#include "core/navmesh/detail_triangulation.h"

#include <algorithm>
#include <cmath>
#include <optional>
#include <set>
#include <stdexcept>

namespace
{
    using namespace navmesh::core;
    using Face = std::array<std::uint32_t, 3>;

    /// Double intermediates keep orientation stable for thin triangles at distant world coordinates.
    double Orientation(Vec3 a, Vec3 b, Vec3 c)
    {
        return (static_cast<double>(b.x) - a.x) * (static_cast<double>(c.y) - a.y) -
               (static_cast<double>(b.y) - a.y) * (static_cast<double>(c.x) - a.x);
    }

    bool Contains(Vec3 point, const Face &face, std::span<const Vec3> points)
    {
        return Orientation(points[face[0]], points[face[1]], point) >= 0 &&
               Orientation(points[face[1]], points[face[2]], point) >= 0 &&
               Orientation(points[face[2]], points[face[0]], point) >= 0;
    }

    double RingArea(std::span<const Vec3> points, const std::vector<std::uint32_t> &ring,
                    std::optional<std::size_t> omitted = {})
    {
        std::vector<std::uint32_t> retained;
        for (std::size_t index{}; index < ring.size(); ++index)
        {
            if (omitted != index)
            {
                retained.push_back(ring[index]);
            }
        }
        double area{};
        for (std::size_t index = 2; index < retained.size(); ++index)
        {
            area += Orientation(points[retained[0]], points[retained[index - 1]], points[retained[index]]);
        }
        return area;
    }

    /// Ear clipping preserves every hull segment, including collinear height samples.
    std::vector<Face> TriangulateBoundary(std::span<const Vec3> points, std::span<const std::uint32_t> boundary)
    {
        std::vector<std::uint32_t> ring(boundary.begin(), boundary.end());
        std::vector<Face> triangles;
        while (ring.size() > 3)
        {
            std::optional<std::size_t> selected;
            double bestArea{};
            for (std::size_t index{}; index < ring.size(); ++index)
            {
                const Face face{ring[(index + ring.size() - 1) % ring.size()], ring[index],
                                ring[(index + 1) % ring.size()]};
                const auto area = Orientation(points[face[0]], points[face[1]], points[face[2]]);
                if (area <= bestArea || RingArea(points, ring, index) <= 0)
                {
                    continue;
                }
                const auto occupied = std::any_of(ring.begin(), ring.end(),
                                                  [&](auto other)
                                                  {
                                                      return std::find(face.begin(), face.end(), other) == face.end() &&
                                                             Contains(points[other], face, points);
                                                  });
                if (!occupied)
                {
                    selected = index;
                    bestArea = area;
                }
            }
            if (!selected)
            {
                throw std::runtime_error("Sampled floor boundary cannot be triangulated");
            }
            const auto index = *selected;
            triangles.push_back(
                {ring[(index + ring.size() - 1) % ring.size()], ring[index], ring[(index + 1) % ring.size()]});
            ring.erase(ring.begin() + index);
        }
        if (ring.size() != 3 || Orientation(points[ring[0]], points[ring[1]], points[ring[2]]) <= 0)
        {
            throw std::runtime_error("Sampled floor boundary is degenerate");
        }
        triangles.push_back({ring[0], ring[1], ring[2]});
        return triangles;
    }

    /// Insert one sample into a containing face or both faces consuming an internal edge.
    void InsertSample(std::vector<Face> &triangles, std::span<const Vec3> points, std::uint32_t sample)
    {
        const auto found = std::find_if(triangles.begin(), triangles.end(),
                                        [&](const auto &face) { return Contains(points[sample], face, points); });
        if (found == triangles.end())
        {
            throw std::runtime_error("Height sample lies outside its floor boundary");
        }
        const auto face = *found;
        std::optional<std::pair<std::uint32_t, std::uint32_t>> edge;
        for (std::size_t side{}; side < 3; ++side)
        {
            if (Orientation(points[face[side]], points[face[(side + 1) % 3]], points[sample]) == 0)
            {
                edge = std::minmax(face[side], face[(side + 1) % 3]);
                break;
            }
        }
        if (!edge)
        {
            *found = {face[0], face[1], sample};
            triangles.push_back({face[1], face[2], sample});
            triangles.push_back({face[2], face[0], sample});
            return;
        }
        const auto originalCount = triangles.size();
        for (std::size_t index{}; index < originalCount; ++index)
        {
            const auto current = triangles[index];
            for (std::size_t side{}; side < 3; ++side)
            {
                if (std::pair<std::uint32_t, std::uint32_t>(std::minmax(current[side], current[(side + 1) % 3])) ==
                    *edge)
                {
                    triangles[index] = {current[side], sample, current[(side + 2) % 3]};
                    triangles.push_back({sample, current[(side + 1) % 3], current[(side + 2) % 3]});
                    break;
                }
            }
        }
    }
} // namespace

namespace navmesh::core
{
    std::vector<std::array<std::uint32_t, 3>> TriangulateDetailSamples(std::span<const Vec3> points,
                                                                       std::span<const std::uint32_t> boundary)
    {
        std::set<std::pair<float, float>> coordinates;
        for (const auto point : points)
        {
            if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z))
            {
                throw std::runtime_error("Sampled floor contains nonfinite coordinates");
            }
            if (!coordinates.emplace(point.x, point.y).second)
            {
                throw std::runtime_error("Sampled floor has duplicate XY samples");
            }
        }
        const std::set<std::uint32_t> hull(boundary.begin(), boundary.end());
        if (hull.size() != boundary.size() || boundary.size() < 3 || *hull.rbegin() >= points.size())
        {
            throw std::runtime_error("Sampled floor has an invalid boundary");
        }
        auto triangles = TriangulateBoundary(points, boundary);
        for (std::uint32_t sample{}; sample < points.size(); ++sample)
        {
            if (!hull.contains(sample))
            {
                InsertSample(triangles, points, sample);
            }
        }
        return triangles;
    }
} // namespace navmesh::core
