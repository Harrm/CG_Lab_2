#include "ray_generation.h"

#define STBI_MSC_SECURE_CRT
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

RayGenerationApp::RayGenerationApp(uint32_t width, uint32_t height) :
	width(width),
	height(height)
{
	camera.SetRenderTargetSize(width, height);
	frame_buffer.resize(width * height);
}

void RayGenerationApp::SetCamera(float3 position, float3 direction, float3 approx_up)
{
	camera.SetPosition(position);
	camera.LookAt(direction);
	camera.SetUp(approx_up);
}

void RayGenerationApp::Clear()
{
	std::fill(frame_buffer.begin(), frame_buffer.end(), byte3 { 0, 0, 0 });
}

void RayGenerationApp::DrawScene()
{
	for (uint32_t x = 0; x < width; x++) {
		for (uint32_t y = 0; y < height; y++) {
			auto ray = camera.GetCameraRay(x, y);
			auto payload = TraceRay(ray, 32);
			SetPixel(x, y, payload.color);
		}
	}
}

int RayGenerationApp::Save(std::string filename) const
{
	int result = stbi_write_png(filename.c_str(), width, height, 3, frame_buffer.data(), width * 3);

	if (result == 1)
		system((std::string("start ") + filename).c_str());

	return (1 - result); // convert stb OK code to the most usable
}

Payload RayGenerationApp::TraceRay(const Ray& ray, const uint32_t max_raytrace_depth) const
{
	return Miss(ray);
}

Payload RayGenerationApp::Miss(const Ray& ray) const
{
	Payload payload{ .color {0, 0, (0.5f + ray.direction.y)} };
	return payload;
}

void RayGenerationApp::SetPixel(uint32_t x, uint32_t y, float3 color)
{
	frame_buffer[static_cast<size_t>(y)* static_cast<size_t>(width) + static_cast<size_t>(x)] =
		byte3{ static_cast<uint8_t>(255 * color.x), static_cast<uint8_t>(255 * color.y), static_cast<uint8_t>(255 * color.z) };
}

Camera::Camera():
	width {0},
	height {0}
{
}

void Camera::SetPosition(float3 position)
{
	this->position = position;
}

void Camera::LookAt(float3 direction)
{
	this->direction = normalize(direction - position);
}

void Camera::SetUp(float3 approx_up)
{
	right = cross(direction, normalize(approx_up));
	up = cross(right, direction);
}

void Camera::SetRenderTargetSize(uint32_t width, uint32_t height)
{
	this->width = width;
	this->height = height;
}

Ray Camera::GetCameraRay(uint32_t x, uint32_t y, float3 jitter) const
{
	float u{ ((static_cast<float>(x) / static_cast<float>(width)) - 0.5f) / 2.f * (static_cast<float>(width) / static_cast<float>(height)) };
	float v{ ((static_cast<float>(y) / static_cast<float>(height)) - 0.5f) / 2.f };
	float3 ray_dir = direction + u * right - v * up + jitter;
	return Ray{ .position {position}, .direction {ray_dir} };
}
