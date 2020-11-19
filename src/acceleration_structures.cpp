#include "acceleration_structures.h"

//#define TINYOBJLOADER_IMPLEMENTATION
#include "tiny_obj_loader.h"

#include <filesystem>

AccelerationStructures::AccelerationStructures(short width, short height) : LightingAndShadows(width, height)
{
}

AccelerationStructures::~AccelerationStructures()
{
}

bool TLAS::AABBTest(const Ray& ray) const
{
    float3 inv_ray_dir = float3(1.f) / ray.direction;
    float3 t0 = (aabb_max - ray.position) * inv_ray_dir;
    float3 t1 = (aabb_min - ray.position) * inv_ray_dir;
    float3 tmin = min(t0, t1);
    float3 tmax = max(t0, t1);

    return maxelem(tmin) <= minelem(tmax);
}

void TLAS::AddMesh(const Mesh mesh)
{
    if (meshes.empty()) {
        aabb_max = mesh.aabb_max;
        aabb_min = mesh.aabb_min;
    }
    meshes.push_back(mesh);
    aabb_min = min(aabb_min, mesh.aabb_min);
    aabb_max = max(aabb_max, mesh.aabb_max);
}

bool operator<(const Mesh& a, const Mesh& b)
{
	return a.aabb_center() < b.aabb_center();
}

void AccelerationStructures::BuildBVH(std::vector<Mesh>&& meshes)
{
    std::sort(meshes.begin(), meshes.end());
    auto mid = meshes.begin() + meshes.size() / 2;
    TLAS left, right;
    for (auto it = meshes.begin(); it != mid; it++) {
        left.AddMesh(*it);
    }
    for (auto it = mid; it != meshes.end(); it++) {
        right.AddMesh(*it);
    }
    tlases.push_back(std::move(left));
    tlases.push_back(std::move(right));
}

int AccelerationStructures::LoadGeometry(std::string filename)
{
    tinyobj::attrib_t attrib;
    std::vector<tinyobj::shape_t> shapes;
    std::vector<tinyobj::material_t> materials;

    std::string warn;
    std::string err;

    bool ret = tinyobj::LoadObj(&attrib, &shapes, &materials, &warn, &err,
        filename.c_str(),
        std::filesystem::path(filename).parent_path().string().c_str(), true);

    if (!warn.empty()) {
        std::cout << warn << std::endl;
    }

    if (!err.empty()) {
        std::cerr << err << std::endl;
    }

    if (!ret) {
        throw std::runtime_error("Failed to parse OBJ file " + filename +
            " reason: " + err);
    }
    std::vector<Mesh> meshes;
    meshes.reserve(shapes.size());
    // Loop over shapes
    for (size_t s = 0; s < shapes.size(); s++) {
        Mesh m;
        // Loop over faces(polygon)
        size_t index_offset = 0;
        for (size_t f = 0; f < shapes[s].mesh.num_face_vertices.size(); f++) {
            int fv = shapes[s].mesh.num_face_vertices[f];

            std::vector<Vertex> vertices;
            // Loop over vertices in the face.
            for (size_t v = 0; v < fv; v++) {
                // access to vertex
                tinyobj::index_t idx = shapes[s].mesh.indices[index_offset + v];
                tinyobj::real_t vx = attrib.vertices[3 * idx.vertex_index + 0];
                tinyobj::real_t vy = attrib.vertices[3 * idx.vertex_index + 1];
                tinyobj::real_t vz = attrib.vertices[3 * idx.vertex_index + 2];

                if (idx.normal_index < 0) {
                    vertices.emplace_back(float3{ vx, vy, vz });
                }
                else {
                    tinyobj::real_t nx = attrib.normals[3 * idx.normal_index + 0];
                    tinyobj::real_t ny = attrib.normals[3 * idx.normal_index + 1];
                    tinyobj::real_t nz = attrib.normals[3 * idx.normal_index + 2];
                    vertices.push_back(Vertex{ float3{vx, vy, vz}, float3{nx, ny, nz} });
                }
            }
            index_offset += fv;
            MaterialTriangle tri{ vertices[0], vertices[1], vertices[2] };

            tinyobj::material_t mat = materials[shapes[s].mesh.material_ids[f]];
            tri.SetEmisive(float3{ mat.emission });
            tri.SetAmbient(float3{ mat.ambient });
            tri.SetDiffuse(float3{ mat.diffuse });
            tri.SetSpecular(float3{ mat.specular }, mat.shininess);
            tri.SetReflectiveness(mat.illum == 5);
            tri.SetReflectivenessAndTransparency(mat.illum == 7);
            tri.SetIor(mat.ior);

            m.AddTriangle(std::move(tri));
        }

        meshes.push_back(std::move(m));
    }
    BuildBVH(std::move(meshes));
    return 0;
}

Payload AccelerationStructures::TraceRay(const Ray& ray, const unsigned int max_raytrace_depth) const
{
    if (max_raytrace_depth == 0) return Miss(ray);
    IntersectableData closest_data{ t_max };
    const MaterialTriangle* closest_object{};
    for (size_t i = 0; i < tlases.size(); i++) {
        if (not tlases[i].AABBTest(ray)) {
            continue;
        }
        for (auto const& m : tlases[i].GetMeshes()) {
            if (not m.AABBTest(ray)) {
                continue;
            }
            for (auto& t : m.Triangles()) {
                auto data = t.Intersect(ray);
                if (data.t > t_min && data.t < closest_data.t) {
                    closest_object = &t;
                    closest_data = data;
                }
            }
        }
    }
    if (closest_object != nullptr && closest_data.t < t_max) {
        return Hit(ray, closest_data, *closest_object, max_raytrace_depth - 1);
    }
    return Miss(ray);
}

float AccelerationStructures::TraceShadowRay(const Ray& ray, const float max_t) const {
    for (auto& tlas : tlases) {
        if (not tlas.AABBTest(ray)) {
            continue;
        }
        for (auto& mesh : tlas.GetMeshes()) {
            if (!mesh.AABBTest(ray)) continue;

            for (auto& t : mesh.Triangles()) {
                auto data = t.Intersect(ray);
                if (data.t > t_min && data.t < max_t) {
                    return data.t;
                }
            }
        }
    }
    return max_t;
}

void Mesh::AddTriangle(const MaterialTriangle triangle)
{
    if (triangles.empty()) {
        aabb_max = triangle.a.position;
        aabb_min = aabb_max;
    }
    triangles.push_back(triangle);
    aabb_max = max(aabb_max, triangle.a.position);
    aabb_max = max(aabb_max, triangle.b.position);
    aabb_max = max(aabb_max, triangle.c.position);

    aabb_min = min(aabb_min, triangle.a.position);
    aabb_min = min(aabb_min, triangle.b.position);
    aabb_min = min(aabb_min, triangle.c.position);
}

bool Mesh::AABBTest(const Ray& ray) const
{
	float3 inv_ray_dir = float3(1.f) / ray.direction;
	float3 t0 = (aabb_max - ray.position) * inv_ray_dir;
	float3 t1 = (aabb_min - ray.position) * inv_ray_dir;
	float3 tmin = min(t0, t1);
	float3 tmax = max(t0, t1);

	return maxelem(tmin) <= minelem(tmax);
}
