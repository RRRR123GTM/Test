#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

namespace dx12xpbd
{
    struct Vec3
    {
        float x = 0.0f;
        float y = 0.0f;
        float z = 0.0f;

        Vec3() = default;
        Vec3(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}

        static Vec3 Zero() { return {0.0f, 0.0f, 0.0f}; }

        Vec3 operator+(const Vec3& rhs) const { return {x + rhs.x, y + rhs.y, z + rhs.z}; }
        Vec3 operator-(const Vec3& rhs) const { return {x - rhs.x, y - rhs.y, z - rhs.z}; }
        Vec3 operator*(float s) const { return {x * s, y * s, z * s}; }
        Vec3 operator/(float s) const { return {x / s, y / s, z / s}; }
        Vec3& operator+=(const Vec3& rhs) { x += rhs.x; y += rhs.y; z += rhs.z; return *this; }
        Vec3& operator-=(const Vec3& rhs) { x -= rhs.x; y -= rhs.y; z -= rhs.z; return *this; }

        float LengthSquared() const { return x * x + y * y + z * z; }
        float Length() const { return std::sqrt(LengthSquared()); }

        Vec3 Normalize() const
        {
            const float len = Length();
            if (len > 1e-6f)
            {
                return {x / len, y / len, z / len};
            }
            return Zero();
        }
    };

    inline Vec3 operator*(float s, const Vec3& v) { return v * s; }

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

    struct Triangle
    {
        uint32_t i0 = 0;
        uint32_t i1 = 0;
        uint32_t i2 = 0;
    };

    struct MeshData
    {
        std::vector<Vec3> positions;
        std::vector<uint32_t> indices;
    };

    struct SphereCollider
    {
        Vec3 center = Vec3::Zero();
        float radius = 0.5f;
    };

    struct CapsuleCollider
    {
        Vec3 start = Vec3::Zero();
        Vec3 end = Vec3::Zero();
        float radius = 0.5f;
    };

    struct ClothVertex
    {
        Vec3 position = Vec3::Zero();
        Vec3 previousPosition = Vec3::Zero();
        Vec3 velocity = Vec3::Zero();
        std::vector<uint32_t> neighbors;
    };

    struct Constraint
    {
        uint32_t i0 = 0;
        uint32_t i1 = 0;
        float restLength = 0.0f;
    };

    class ClothXPBD
    {
    public:
        void Initialize(const MeshData& mesh, float stiffness = 0.9f);
        void Reset(const MeshData& mesh);
        void Update(float dt,
                    const std::vector<SphereCollider>& spheres,
                    const std::vector<CapsuleCollider>& capsules,
                    const Vec3& gravity,
                    int iterations = 6,
                    float damping = 0.98f);

        const std::vector<ClothVertex>& Vertices() const { return vertices_; }
        const std::vector<Triangle>& Triangles() const { return triangles_; }
        std::vector<Vec3> Positions() const;

    private:
        std::vector<ClothVertex> vertices_;
        std::vector<Triangle> triangles_;
        std::vector<Constraint> constraints_;
        float stiffness_ = 0.9f;

        void BuildConstraints();
        void SolveStretchConstraints(float dt, int iterations);
        void ResolveCollisions(const std::vector<SphereCollider>& spheres,
                               const std::vector<CapsuleCollider>& capsules,
                               float penetrationBias = 0.04f);
    };
}
