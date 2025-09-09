// lib/obstacles_yaml.cpp
#include "obstacles_yaml.hpp"
#include <Eigen/Dense>
#include <memory>
#include <cmath>

namespace {
template <typename T>
bool asVec2(const YAML::Node& n, T& x, T& y) {
    if (!n || !n.IsSequence() || n.size() < 2) return false;
    try { x = static_cast<T>(n[0].as<double>()); y = static_cast<T>(n[1].as<double>()); }
    catch(...) { return false; }
    return true;
}

inline std::string lower(std::string s) {
    for (auto& c : s) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    return s;
}

inline fcl::Transform3f tfFromXY(const float x, const float y) {
    fcl::Transform3f tf;
    tf.linear() = Eigen::Matrix3f::Identity();
    tf.translation() = Eigen::Vector3f(x, y, 0.0f);
    return tf;
}

// Build rotation that maps local X axis to given XY unit vector u = (ux,uy,0).
// We keep Z world-up, so local axes become:
//   x' = u
//   y' = [-uy, ux, 0]
//   z' = [0,  0,  1]
inline Eigen::Matrix3f rotAlignLocalXtoXYDir(const Eigen::Vector2f& dir) {
    Eigen::Vector2f u = dir;
    float n = u.norm();
    if (n < 1e-8f) return Eigen::Matrix3f::Identity();
    u /= n;
    const Eigen::Vector3f x(u.x(), u.y(),  0.0f);
    const Eigen::Vector3f y(-u.y(), u.x(), 0.0f);
    const Eigen::Vector3f z(0.0f,   0.0f,  1.0f);
    Eigen::Matrix3f R;
    R.col(0) = x; R.col(1) = y; R.col(2) = z;
    return R;
}

    // Build a convex prism from a CCW 2D polygon. No holes. Thickness in Z.
    std::shared_ptr<fcl::CollisionGeometryf>
    makeConvexPrism(const std::vector<Eigen::Vector2f>& poly, float thicknessZ)
{
    const int N = static_cast<int>(poly.size());
    if (N < 3) return nullptr;

    const float hz = 0.5f * std::max(thicknessZ, 1e-3f);

    // ---- 2N vertices: bottom (z=-hz), then top (z=+hz) ----
    std::vector<Eigen::Vector3f> pts;
    pts.reserve(2 * N);
    for (int i = 0; i < N; ++i)
        pts.emplace_back(poly[i].x(), poly[i].y(), -hz);
    for (int i = 0; i < N; ++i)
        pts.emplace_back(poly[i].x(), poly[i].y(), +hz);

    // ---- Faces encoded as [count, i0, i1, ...] ----
    // Bottom face (reverse order for outward normal)
    std::vector<int> faces;
    faces.reserve((1 + N) + (1 + N) + N * (1 + 4)); // rough reservation
    faces.push_back(N);
    for (int i = N - 1; i >= 0; --i) faces.push_back(i);

    // Top face (CCW as given)
    faces.push_back(N);
    for (int i = 0; i < N; ++i) faces.push_back(N + i);

    // Side quads: (i, i+1, N+i+1, N+i)
    for (int i = 0; i < N; ++i) {
        const int i2 = (i + 1) % N;
        faces.push_back(4);
        faces.push_back(i);
        faces.push_back(i2);
        faces.push_back(N + i2);
        faces.push_back(N + i);
    }

    const int num_polygons = 2 + N;

    // FCL expects shared_ptr to const vectors plus counts
    auto pts_sp   = std::make_shared<const std::vector<Eigen::Vector3f>>(std::move(pts));
    auto faces_sp = std::make_shared<const std::vector<int>>(std::move(faces));

    // Most portable ctor across 0.5/0.6:
    auto convex = std::make_shared<fcl::Convexf>(
        pts_sp, static_cast<int>(pts_sp->size()),
        faces_sp, num_polygons);

    return std::static_pointer_cast<fcl::CollisionGeometryf>(convex);
}
} // namespace

bool buildEnvironmentObstaclesFCL(
    const YAML::Node& envNode,
    std::vector<fcl::CollisionObjectf*>& out,
    float prismThicknessZ)
{
    out.clear();
    if (!envNode) return false;

    auto obstacles = envNode["obstacles"];
    if (!obstacles || !obstacles.IsSequence()) return true; // no obstacles is fine

    for (const auto& o : obstacles) {
        const std::string ty = o["type"] ? lower(o["type"].as<std::string>()) : "";

        if (ty == "box") {
            double cx=0, cy=0, w=0, h=0;
            if (!asVec2(o["center"], cx, cy) || !asVec2(o["size"], w, h)) continue;

            auto geom = std::make_shared<fcl::Boxf>(static_cast<float>(w),
                                                    static_cast<float>(h),
                                                    std::max(prismThicknessZ, 1e-3f));
            auto* co = new fcl::CollisionObjectf(geom);
            co->setTranslation(fcl::Vector3f(static_cast<float>(cx),
                                             static_cast<float>(cy), 0.0f));
            co->computeAABB();
            out.push_back(co);
        }
        else if (ty == "circle") {
            double cx=0, cy=0, r=0;
            if (!asVec2(o["center"], cx, cy) || !o["radius"]) continue;
            try { r = o["radius"].as<double>(); } catch(...) { r = 0; }
            if (r <= 0) continue;

            auto geom = std::make_shared<fcl::Spheref>(static_cast<float>(r));
            auto* co = new fcl::CollisionObjectf(geom);
            co->setTranslation(fcl::Vector3f(static_cast<float>(cx),
                                             static_cast<float>(cy), 0.0f));
            co->computeAABB();
            out.push_back(co);
        }
        else if (ty == "polygon") {
            auto pts = o["points"];
            if (!pts || !pts.IsSequence() || pts.size() < 3) continue;

            std::vector<Eigen::Vector2f> poly;
            poly.reserve(pts.size());
            for (const auto& p : pts) {
                double px=0, py=0; if (!asVec2(p, px, py)) { poly.clear(); break; }
                poly.emplace_back(static_cast<float>(px), static_cast<float>(py));
            }
            if (poly.size() < 3) continue;

            auto geom = makeConvexPrism(poly, prismThicknessZ);
            if (!geom) continue;

            auto* co = new fcl::CollisionObjectf(geom);
            co->setTranslation(fcl::Vector3f(0,0,0)); // vertices already in world XY
            co->computeAABB();
            out.push_back(co);
        }
        else if (ty == "capsule") {
            double x0=0,y0=0,x1=0,y1=0, r=0;
            if (!asVec2(o["p0"], x0, y0) || !asVec2(o["p1"], x1, y1) || !o["radius"]) continue;
            try { r = o["radius"].as<double>(); } catch(...) { r = 0; }
            if (r <= 0) continue;

            const Eigen::Vector2f P0(static_cast<float>(x0), static_cast<float>(y0));
            const Eigen::Vector2f P1(static_cast<float>(x1), static_cast<float>(y1));
            const Eigen::Vector2f d = P1 - P0;
            const float L = d.norm();

            if (L < 1e-6f) {
                // Degenerates to a circle at P0
                auto geom = std::make_shared<fcl::Spheref>(static_cast<float>(r));
                auto* co = new fcl::CollisionObjectf(geom);
                co->setTranslation(fcl::Vector3f(P0.x(), P0.y(), 0.0f));
                co->computeAABB();
                out.push_back(co);
                continue;
            }

            // Central bar: oriented box length=L, width=2r, thickness=prismThicknessZ
            const float w = 2.0f * static_cast<float>(r);
            const float t = std::max(prismThicknessZ, 1e-3f);
            auto barGeom = std::make_shared<fcl::Boxf>(L, w, t);
            auto* bar = new fcl::CollisionObjectf(barGeom);
            fcl::Transform3f T;
            T.linear() = rotAlignLocalXtoXYDir(d);
            const Eigen::Vector2f mid = 0.5f * (P0 + P1);
            T.translation() = Eigen::Vector3f(mid.x(), mid.y(), 0.0f);
            bar->setTransform(T);
            bar->computeAABB();
            out.push_back(bar);

            // End caps (spheres) at P0 and P1
            auto sphGeom = std::make_shared<fcl::Spheref>(static_cast<float>(r));
            auto* s0 = new fcl::CollisionObjectf(sphGeom);
            s0->setTranslation(fcl::Vector3f(P0.x(), P0.y(), 0.0f));
            s0->computeAABB();
            out.push_back(s0);

            auto* s1 = new fcl::CollisionObjectf(sphGeom);
            s1->setTranslation(fcl::Vector3f(P1.x(), P1.y(), 0.0f));
            s1->computeAABB();
            out.push_back(s1);
        }
        else {
            // Unknown type — skip silently or log as needed
            continue;
        }
    }

    return true;
}
