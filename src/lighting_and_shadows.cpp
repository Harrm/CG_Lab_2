#include "lighting_and_shadows.h"

#define TINYOBJLOADER_IMPLEMENTATION
#include "tiny_obj_loader.h"

#include <algorithm>
#include <filesystem>

LightingAndShadows::LightingAndShadows(short width, short height) : MTAlgorithm(width, height)
{
}

int LightingAndShadows::LoadGeometry(std::string filename)
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

    // Loop over shapes
    for (size_t s = 0; s < shapes.size(); s++) {
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
                    vertices.push_back(Vertex{ float3{ vx, vy, vz } });
                }
                else {
                    tinyobj::real_t nx = attrib.normals[3 * idx.normal_index + 0];
                    tinyobj::real_t ny = attrib.normals[3 * idx.normal_index + 1];
                    tinyobj::real_t nz = attrib.normals[3 * idx.normal_index + 2];
                    vertices.push_back(Vertex{ float3{vx, vy, vz}, float3{nx, ny, nz} });
                }
            }
            index_offset += fv;
            auto tri = std::make_unique<MaterialTriangle>(vertices[0], vertices[1], vertices[2]);

            tinyobj::material_t mat = materials[shapes[s].mesh.material_ids[f]];
            tri->SetEmisive(float3{ mat.emission });
            tri->SetAmbient(float3{ mat.ambient });
            tri->SetDiffuse(float3{ mat.diffuse });
            tri->SetSpecular(float3{ mat.specular }, mat.shininess);
            tri->SetReflectiveness(mat.illum == 5);
            tri->SetReflectivenessAndTransparency(mat.illum == 7);
            tri->SetIor(mat.ior);

            material_objects.push_back(std::move(tri));
        }
    }

    return 0;
}

void LightingAndShadows::AddLight(Light const& light)
{
    lights.push_back(std::make_unique<Light>(light));
    objects.push_back(std::make_unique<Sphere>(light.position, 0.03f));
}

Payload LightingAndShadows::TraceRay(const Ray& ray, const unsigned int max_raytrace_depth) const
{
    if (max_raytrace_depth == 0) return Miss(ray);
    IntersectableData closest_simple_data{ t_max };
    Intersectable* closest_simple_object{};

    for (auto& object : objects) {
        auto data = object->Intersect(ray);
        if (data.t > t_min && data.t < closest_simple_data.t) {
            closest_simple_object = object.get();
            closest_simple_data = data;
        }
    }

    MaterialTriangle* closest_object{};
    IntersectableData closest_data{ t_max };
    for (auto& object : material_objects) {
        auto data = object->Intersect(ray);
        if (data.t > t_min && data.t < closest_data.t) {
            closest_object = object.get();
            closest_data = data;
        }
    }
    if (closest_data.t < t_max && closest_data.t < closest_simple_data.t) {
        return Hit(ray, closest_data, *closest_object, max_raytrace_depth - 1);
    }
    if (closest_simple_data.t < t_max) {
        return MTAlgorithm::Hit(ray, closest_simple_data);
    }
    return Miss(ray);
}

float LightingAndShadows::TraceShadowRay(const Ray& ray, const float max_t) const
{
    for (auto& object : material_objects) {
        auto data = object->Intersect(ray);
        if (data.t > t_min && data.t < max_t) {
            return data.t;
        }
    }
    return max_t;
}


Payload LightingAndShadows::Hit(
    const Ray& ray,
    const IntersectableData& data,
    const MaterialTriangle& triangle, 
    const unsigned int max_raytrace_depth) const
{
    if (max_raytrace_depth == 0) return Miss(ray);

    float3 touch = ray.position + ray.direction * data.t;
    Payload p;
    p.color = triangle.emissive_color + 0.1f * triangle.ambient_color;
    float3 normal = triangle.GetNormal(data.baricentric);

    for (auto& light : lights) {
        Ray to_light{ .position = touch, .direction = linalg::normalize(light->position - touch) };        
        float light_length = length(light->position - touch);
        float t = TraceShadowRay(to_light, light_length);
        if (fabs(t - light_length) < 0.00001) {
            p.color += light->color * triangle.diffuse_color * std::max(0.f, dot(to_light.direction, normal));

            Ray from_light{ light->position, touch - light->position };
            float3 specular_direction = from_light.direction - 2.f * dot(from_light.direction, normal) * normal;
            p.color += light->color * triangle.specular_color
                * std::powf(
                    std::max(0.f, dot(to_light.direction, normal)),
                    triangle.specular_exponent);
        }
    }
    p.color.x = std::min(p.color.x, 1.0f);
    p.color.y = std::min(p.color.y, 1.0f);
    p.color.z = std::min(p.color.z, 1.0f);
	return p;
}

float3 MaterialTriangle::GetNormal(float3 barycentr) const
{
	if (length(a.normal) == 0 || length(b.normal) == 0 || length(c.normal) == 0) {
        return geo_normal;
	}
	return barycentr.x * a.normal + barycentr.y * b.normal + barycentr.z * c.normal;
}
