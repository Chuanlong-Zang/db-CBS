// lib/obstacles_yaml.cpp
#include "obstacles_yaml.hpp"
#include <Eigen/Dense>
#include <memory>
#include <cmath>
#include <fcl/common/types.h>              // for fcl::Vector3f (and aliases)
#include <fcl/geometry/shape/convex.h>

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

    static inline double signedArea2(const std::vector<Eigen::Vector2f>& P) {
    double A=0; int n=(int)P.size(); for (int i=0,j=n-1;i<n;j=i++)
        A += (double)P[j].x()*P[i].y() - (double)P[i].x()*P[j].y();
    return A; // >0 => CCW
}

    static inline bool sanitizeConvexLoop(std::vector<Eigen::Vector2f>& poly) {
    if (poly.size() < 3) return false;
    // drop closing duplicate
    if ((poly.front() - poly.back()).squaredNorm() < 1e-12f) poly.pop_back();
    if (poly.size() < 3) return false;

    // drop consecutive duplicates / colinear middles
    std::vector<Eigen::Vector2f> q; q.reserve(poly.size());
    auto colinear = [](const Eigen::Vector2f& a,const Eigen::Vector2f& b,const Eigen::Vector2f& c){
        Eigen::Vector2f u=b-a, v=c-b; return std::abs(u.x()*v.y()-u.y()*v.x()) <= 1e-8f;
    };
    for (size_t i=0;i<poly.size();++i){
        const auto& prev = poly[(i+poly.size()-1)%poly.size()];
        const auto& curr = poly[i];
        const auto& next = poly[(i+1)%poly.size()];
        if ((curr-prev).squaredNorm() < 1e-14f) continue;
        if (colinear(prev,curr,next)) continue;
        q.push_back(curr);
    }
    poly.swap(q);
    if (poly.size() < 3) return false;

    // enforce CCW
    if (signedArea2(poly) < 0.0) std::reverse(poly.begin(), poly.end());

    // skip tiny area (scale for your [0,50] world)
    const double A = 0.5 * std::abs(signedArea2(poly));
    if (A < 1e-8) return false;

    // skip tiny bbox (sliver)
    float minx=poly[0].x(), maxx=minx, miny=poly[0].y(), maxy=miny;
    for (auto& p: poly){ minx=std::min(minx,p.x()); maxx=std::max(maxx,p.x());
        miny=std::min(miny,p.y()); maxy=std::max(maxy,p.y()); }
    if ((maxx-minx) < 1e-4f || (maxy-miny) < 1e-4f) return false;

    return true;
}
    std::shared_ptr<fcl::CollisionGeometryf>
    makeTriPrism(const std::vector<Eigen::Vector2f>& poly_in, float thicknessZ)
{
    std::vector<Eigen::Vector2f> poly = poly_in;
    if (!sanitizeConvexLoop(poly)) return nullptr;

    const int N = (int)poly.size();
    const float hz = 0.5f * std::max(thicknessZ, 1e-3f);

    using V3 = fcl::Vector3f;
    using Tri = fcl::Triangle;
    using BVH = fcl::BVHModel<fcl::OBBRSSf>;

    std::vector<V3> V; V.reserve(2*N);
    for (int i=0;i<N;++i) V.emplace_back(poly[i].x(), poly[i].y(), -hz); // bottom
    for (int i=0;i<N;++i) V.emplace_back(poly[i].x(), poly[i].y(), +hz); // top

    std::vector<Tri> T; T.reserve(2*(N-2) + 2*N);

    // bottom fan (reverse winding for outward -Z)
    for (int i=1;i<N-1;++i) T.emplace_back(0, i+1, i);

    // top fan (as-given, outward +Z)
    for (int i=1;i<N-1;++i) T.emplace_back(N+0, N+i, N+i+1);

    // sides: two triangles per edge (outward)
    for (int i=0;i<N;++i) {
        int j = (i+1)%N;
        T.emplace_back(i, j, N+j);
        T.emplace_back(i, N+j, N+i);
    }

    auto model = std::make_shared<BVH>();
    model->beginModel((int)T.size(), (int)V.size());
    model->addSubModel(V, T);
    model->endModel();
    return model;
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

            auto geom = makeTriPrism(poly, prismThicknessZ);
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
