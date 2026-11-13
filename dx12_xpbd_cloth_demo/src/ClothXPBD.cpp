#include "ClothXPBD.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace dx12xpbd
{
    namespace
    {
        inline float Dot(const Vec3& a, const Vec3& b)
        {
            return a.x * b.x + a.y * b.y + a.z * b.z;
        }

        inline Vec3 Cross(const Vec3& a, const Vec3& b)
        {
            return {
                a.y * b.z - a.z * b.y,
                a.z * b.x - a.x * b.z,
                a.x * b.y - a.y * b.x
            };
        }

        inline Vec3 Normalize(const Vec3& v)
        {
            const float len = v.Length();
            if (len > 1e-6f)\n                return v / len;
            return Vec3::Zero();
        }

        inline Vec3 ClosestPointOnSegment(const Vec3& p, const Vec3& a, const Vec3& b)
        {
            const Vec3 ab = b - a;
            const float t = std::clamp(Dot(p - a, ab) / std::max(Dot(ab, ab), 1e-6f), 0.0f, 1.0f);
            return a + ab * t;
        }
    }

    void ClothXPBD::Initialize(const MeshData& mesh, float stiffness)
    {
        stiffness_ = stiffness;
        Reset(mesh);
    }

    void ClothXPBD::Reset(const MeshData& mesh)
    {
        vertices_.clear();
        triangles_.clear();
        constraints_.clear();

        vertices_.resize(mesh.positions.size());
        for (size_t i = 0; i < mesh.positions.size(); ++i)
        {
            vertices_[i].position = mesh.positions[i];
            vertices_[i].previousPosition = mesh.positions[i];
            vertices_[i].velocity = Vec3::Zero();
        }

        for (size_t i = 0; i + 2 < mesh.indices.size(); i += 3)
        {
            triangles_.push_back({mesh.indices[i], mesh.indices[i + 1], mesh.indices[i + 2]});
        }

        BuildConstraints();
    }

    void ClothXPBD::BuildConstraints()
    {
        std::vector<std::vector<uint32_t>> adjacency(vertices_.size());
        for (const auto& tri : triangles_)
        {
            adjacency[tri.i0].push_back(tri.i1);
            adjacency[tri.i0].push_back(tri.i2);
            adjacency[tri.i1].push_back(tri.i0);
            adjacency[tri.i1].push_back(tri.i2);
            adjacency[tri.i2].push_back(tri.i0);
            adjacency[tri.i2].push_back(tri.i1);
        }

        for (size_t i = 0; i < vertices_.size(); ++i)
        {
            auto& neigh = vertices_[i].neighbors;
            neigh.clear();
            std::sort(adjacency[i].begin(), adjacency[i].end());
            neigh.insert(neigh.end(), adjacency[i].begin(), adjacency[i].end());
            neigh.erase(std::unique(neigh.begin(), neigh.end()), neigh.end());
        }

        std::vector<bool> visited(vertices_.size(), false);
        for (size_t i = 0; i < vertices_.size(); ++i)
        {
            for (uint32_t j : vertices_[i].neighbors)
            {
                if (j <= i) continue;
                const float rest = (vertices_[i].position - vertices_[j].position).Length();
                constraints_.push_back({static_cast<uint32_t>(i), j, rest});
            }
        }

        for (const auto& tri : triangles_)
        {
            const auto& a = vertices_[tri.i0].position;
            const auto& b = vertices_[tri.i1].position;
            const auto& c = vertices_[tri.i2].position;
            const float ab = (a - b).Length();
            const float bc = (b - c).Length();
            const float ca = (c - a).Length();

            if (ab > 1e-4f)
                constraints_.push_back({tri.i0, tri.i1, ab});
            if (bc > 1e-4f)
                constraints_.push_back({tri.i1, tri.i2, bc});
            if (ca > 1e-4f)
                constraints_.push_back({tri.i2, tri.i0, ca});
        }
    }

    void ClothXPBD::SolveStretchConstraints(float dt, int iterations)
    {
        for (int iter = 0; iter < iterations; ++iter)
        {
            for (const auto& c : constraints_)
            {
                auto& p0 = vertices_[c.i0].position;
                auto& p1 = vertices_[c.i1].position;
                const Vec3 delta = p1 - p0;
                const float dist = delta.Length();
                if (dist < 1e-6f) continue;

                const float diff = (dist - c.restLength) * stiffness_;
                const Vec3 n = delta / dist;
                const Vec3 correction = n * (diff * 0.5f);

                p0 += correction;
                p1 -= correction;
            }
        }
    }

    void ClothXPBD::ResolveCollisions(const std::vector<SphereCollider>& spheres,
                                     const std::vector<CapsuleCollider>& capsules,
                                     float penetrationBias)
    {
        for (auto& v : vertices_)
        {
            for (const auto& sphere : spheres)
            {
                const Vec3 toVertex = v.position - sphere.center;
                const float dist = toVertex.Length();
                const float minDist = sphere.radius + penetrationBias;
                if (dist < minDist)
                {
                    const Vec3 normal = Normalize(toVertex);
                    v.position = sphere.center + normal * minDist;
                }
            }

            for (const auto& cap : capsules)
            {
                const Vec3 closest = ClosestPointOnSegment(v.position, cap.start, cap.end);
                const Vec3 delta = v.position - closest;
                const float dist = delta.Length();
                const float minDist = cap.radius + penetrationBias;
                if (dist < minDist)
                {
                    const Vec3 normal = Normalize(delta);
                    v.position = closest + normal * minDist;
                }
            }
        }
    }

    void ClothXPBD::Update(float dt,
                          const std::vector<SphereCollider>& spheres,
                          const std::vector<CapsuleCollider>& capsules,
                          const Vec3& gravity,
                          int iterations,
                          float damping)
    {
        if (dt <= 0.0f) return;

        for (auto& v : vertices_)
        {
            const Vec3 vel = (v.position - v.previousPosition) * damping;
            v.previousPosition = v.position;
            v.velocity = vel + gravity * dt * dt;
            v.position += v.velocity;
        }

        SolveStretchConstraints(dt, iterations);
        ResolveCollisions(spheres, capsules);
    }

    std::vector<Vec3> ClothXPBD::Positions() const
    {
        std::vector<Vec3> out;
        out.reserve(vertices_.size());
        for (const auto& v : vertices_)
            out.push_back(v.position);
        return out;
    }
}
