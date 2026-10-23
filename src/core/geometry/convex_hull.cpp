#include "core/geometry/convex_hull.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <set>
#include <utility>

namespace navmesh::core
{
    namespace
    {
        // Scale geometric degeneracy tests independently of input units. Visibility uses exact orientation
        // signs so nearly coplanar samples cannot create inconsistent visible-face horizons.
        constexpr double Tolerance = 1.0e-10;

        struct Point
        {
            double x{};
            double y{};
            double z{};

            Point operator-(const Point &other) const
            {
                return {x - other.x, y - other.y, z - other.z};
            }
        };

        double Dot(const Point &a, const Point &b)
        {
            return a.x * b.x + a.y * b.y + a.z * b.z;
        }

        Point Cross(const Point &a, const Point &b)
        {
            return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
        }

        /// Nonoverlapping doubles, ordered by increasing magnitude, retain the exact sum of predicate terms.
        using Expansion = std::vector<double>;

        /// Add one term without discarding the rounding residual; zero terms carry no sign information.
        void AddTerm(Expansion &expansion, double term)
        {
            Expansion sum;
            for (const auto component : expansion)
            {
                const double combined = term + component;
                const double recovered = combined - term;
                const double residual = (term - (combined - recovered)) + (component - recovered);
                if (residual != 0)
                {
                    sum.push_back(residual);
                }
                term = combined;
            }
            if (term != 0)
            {
                sum.push_back(term);
            }
            expansion = std::move(sum);
        }

        Expansion Difference(double left, double right)
        {
            Expansion result;
            AddTerm(result, left);
            AddTerm(result, -right);
            return result;
        }

        /// Fused multiply-add recovers each product's rounding error before summing exact expansion terms.
        Expansion Product(const Expansion &left, const Expansion &right)
        {
            Expansion result;
            for (const auto a : left)
            {
                for (const auto b : right)
                {
                    const double product = a * b;
                    AddTerm(result, std::fma(a, b, -product));
                    AddTerm(result, product);
                }
            }
            return result;
        }

        /// Classify a point against an oriented face, falling back to an exact determinant near coplanarity.
        /// Power-of-two-scaled float coordinates keep predicate products finite and preserve input positions.
        int Orientation(const Point &a, const Point &b, const Point &c, const Point &point)
        {
            const auto ab = b - a;
            const auto ac = c - a;
            const auto ap = point - a;
            const double positive = ab.x * ac.y * ap.z + ab.y * ac.z * ap.x + ab.z * ac.x * ap.y;
            const double negative = ab.z * ac.y * ap.x + ab.y * ac.x * ap.z + ab.x * ac.z * ap.y;
            const double determinant = positive - negative;
            const double magnitude = std::abs(ab.x * ac.y * ap.z) + std::abs(ab.y * ac.z * ap.x) +
                                     std::abs(ab.z * ac.x * ap.y) + std::abs(ab.z * ac.y * ap.x) +
                                     std::abs(ab.y * ac.x * ap.z) + std::abs(ab.x * ac.z * ap.y);
            const double error = 16 * std::numeric_limits<double>::epsilon() * magnitude;
            if (std::abs(determinant) > error)
            {
                return determinant > 0 ? 1 : -1;
            }
            const std::array<Expansion, 3> exactAB = {Difference(b.x, a.x), Difference(b.y, a.y), Difference(b.z, a.z)};
            const std::array<Expansion, 3> exactAC = {Difference(c.x, a.x), Difference(c.y, a.y), Difference(c.z, a.z)};
            const std::array<Expansion, 3> exactAP = {Difference(point.x, a.x), Difference(point.y, a.y),
                                                      Difference(point.z, a.z)};
            Expansion exact;
            for (std::size_t axis = 0; axis < 3; ++axis)
            {
                const auto next = (axis + 1) % 3;
                const auto last = (axis + 2) % 3;
                for (const auto term : Product(Product(exactAB[axis], exactAC[next]), exactAP[last]))
                {
                    AddTerm(exact, term);
                }
                for (const auto term : Product(Product(exactAB[axis], exactAC[last]), exactAP[next]))
                {
                    AddTerm(exact, -term);
                }
            }
            return exact.empty() ? 0 : exact.back() > 0 ? 1 : -1;
        }

        struct Face
        {
            Triangle triangle;
        };

        /// The seed tetrahedron's interior remains inside every expanded hull and fixes outward winding.
        std::optional<Face> MakeFace(std::uint32_t a, std::uint32_t b, std::uint32_t c,
                                     const std::vector<Point> &points, const Point &interior)
        {
            auto normal = Cross(points[b] - points[a], points[c] - points[a]);
            const auto length = std::sqrt(Dot(normal, normal));
            if (length <= Tolerance * Tolerance)
            {
                return std::nullopt;
            }
            if (Orientation(points[a], points[b], points[c], interior) > 0)
            {
                std::swap(b, c);
            }
            return Face{{{a, b, c}}};
        }

        /// Reject open or overlapping hulls rather than publish partial collision geometry.
        bool IsClosedHull(const std::vector<Face> &faces, const std::vector<Point> &points)
        {
            std::set<std::pair<std::uint32_t, std::uint32_t>> edges;
            for (const auto &face : faces)
            {
                for (const auto &point : points)
                {
                    const auto &indices = face.triangle.vertices;
                    if (Orientation(points[indices[0]], points[indices[1]], points[indices[2]], point) > 0)
                    {
                        return false;
                    }
                }
                const auto &indices = face.triangle.vertices;
                for (std::size_t edge = 0; edge < indices.size(); ++edge)
                {
                    if (!edges.emplace(indices[edge], indices[(edge + 1) % indices.size()]).second)
                    {
                        return false;
                    }
                }
            }
            return std::all_of(edges.begin(), edges.end(),
                               [&](const auto &edge) { return edges.contains({edge.second, edge.first}); });
        }
    } // namespace

    std::optional<std::vector<Triangle>> TriangulateConvexHull(std::span<const Vec3> vertices)
    {
        if (vertices.size() < 4 || vertices.size() > std::numeric_limits<std::uint32_t>::max())
        {
            return std::nullopt;
        }
        AABB bounds;
        for (const auto &vertex : vertices)
        {
            if (!std::isfinite(vertex.x) || !std::isfinite(vertex.y) || !std::isfinite(vertex.z))
            {
                return std::nullopt;
            }
            bounds.Expand(vertex);
        }
        const double extent = std::max({static_cast<double>(bounds.max.x) - bounds.min.x,
                                        static_cast<double>(bounds.max.y) - bounds.min.y,
                                        static_cast<double>(bounds.max.z) - bounds.min.z});
        if (extent <= 0)
        {
            return std::nullopt;
        }

        // Binary scaling preserves every float coordinate exactly, including narrow bevels. Translation and
        // division by an arbitrary extent would introduce rounding before the exact orientation predicates.
        const double scale = std::scalbn(1.0, -std::ilogb(extent));
        // Keep original indices while discarding duplicate samples from the working hull.
        std::vector<Point> points;
        std::vector<std::uint32_t> originalIndices;
        for (std::size_t index = 0; index < vertices.size(); ++index)
        {
            const auto &vertex = vertices[index];
            const Point point{vertex.x * scale, vertex.y * scale, vertex.z * scale};
            if (std::none_of(points.begin(), points.end(),
                             [&](const auto &other)
                             {
                                 const auto delta = point - other;
                                 return Dot(delta, delta) <= Tolerance * Tolerance;
                             }))
            {
                points.push_back(point);
                originalIndices.push_back(static_cast<std::uint32_t>(index));
            }
        }
        if (points.size() < 4)
        {
            return std::nullopt;
        }

        // Seed a nondegenerate tetrahedron using the most separated point, line and plane samples.
        const std::uint32_t first = 0;
        std::uint32_t second = 0, third = 0, fourth = 0;
        double maximum = 0;
        for (std::uint32_t index = 1; index < points.size(); ++index)
        {
            const auto delta = points[index] - points[first];
            const auto distance = Dot(delta, delta);
            if (distance > maximum)
            {
                second = index;
                maximum = distance;
            }
        }
        const auto line = points[second] - points[first];
        const auto lineLengthSquared = Dot(line, line);
        maximum = 0;
        for (std::uint32_t index = 0; index < points.size(); ++index)
        {
            const auto cross = Cross(line, points[index] - points[first]);
            const auto distance = Dot(cross, cross) / lineLengthSquared;
            if (distance > maximum)
            {
                third = index;
                maximum = distance;
            }
        }
        if (maximum <= Tolerance * Tolerance)
        {
            return std::nullopt;
        }
        const auto plane = Cross(line, points[third] - points[first]);
        const auto planeLength = std::sqrt(Dot(plane, plane));
        maximum = 0;
        for (std::uint32_t index = 0; index < points.size(); ++index)
        {
            const auto distance = std::abs(Dot(plane, points[index] - points[first])) / planeLength;
            if (distance > maximum)
            {
                fourth = index;
                maximum = distance;
            }
        }
        if (maximum <= Tolerance)
        {
            return std::nullopt;
        }
        const Point interior{(points[first].x + points[second].x + points[third].x + points[fourth].x) / 4,
                             (points[first].y + points[second].y + points[third].y + points[fourth].y) / 4,
                             (points[first].z + points[second].z + points[third].z + points[fourth].z) / 4};
        std::vector<Face> faces;
        for (const Triangle triangle : {Triangle{{first, second, third}}, Triangle{{first, fourth, second}},
                                        Triangle{{first, third, fourth}}, Triangle{{second, fourth, third}}})
        {
            const auto &indices = triangle.vertices;
            const auto face = MakeFace(indices[0], indices[1], indices[2], points, interior);
            if (!face)
            {
                return std::nullopt;
            }
            faces.push_back(*face);
        }

        // Replace faces visible from each exterior sample with a fan over the visible region's horizon.
        // Opposite edges cancel inside that region; only edges shared with retained faces survive.
        for (std::uint32_t index = 0; index < points.size(); ++index)
        {
            std::set<std::pair<std::uint32_t, std::uint32_t>> horizon;
            std::vector<Face> retained;
            for (const auto &face : faces)
            {
                const auto &vertices = face.triangle.vertices;
                if (Orientation(points[vertices[0]], points[vertices[1]], points[vertices[2]], points[index]) <= 0)
                {
                    retained.push_back(face);
                    continue;
                }
                const auto &indices = face.triangle.vertices;
                for (std::size_t edge = 0; edge < indices.size(); ++edge)
                {
                    const auto a = indices[edge];
                    const auto b = indices[(edge + 1) % indices.size()];
                    if (horizon.erase({b, a}) == 0)
                    {
                        horizon.emplace(a, b);
                    }
                }
            }
            for (const auto &[a, b] : horizon)
            {
                const auto face = MakeFace(a, b, index, points, interior);
                if (!face)
                {
                    return std::nullopt;
                }
                retained.push_back(*face);
            }
            faces = std::move(retained);
        }
        if (!IsClosedHull(faces, points))
        {
            return std::nullopt;
        }
        std::vector<Triangle> result;
        for (const auto &face : faces)
        {
            auto triangle = face.triangle;
            for (auto &index : triangle.vertices)
            {
                index = originalIndices[index];
            }
            result.push_back(triangle);
        }
        return result;
    }
} // namespace navmesh::core
