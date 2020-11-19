#pragma once

#include "linalg.h"
using namespace linalg::aliases;
using namespace linalg::ostream_overloads;

#include <string>
#include <vector>


#include <iostream>

struct Ray 
{
	float3 position;
	float3 direction;
};

struct Payload 
{
	float3 color;
};


class Camera
{
public:
	Camera();

	void SetPosition(float3 position);
	void LookAt(float3 direction);
	void SetUp(float3 approx_up);
	void SetRenderTargetSize(uint32_t width, uint32_t height);

	Ray GetCameraRay(uint32_t x, uint32_t y, float3 jitter = { 0, 0, 0 }) const;

private:
	float3 position;
	float3 direction;
	float3 up;
	float3 right;

	uint32_t width;
	uint32_t height;
};



class RayGenerationApp
{
public:
	RayGenerationApp(uint32_t width, uint32_t height);
	virtual ~RayGenerationApp() = default;

	void SetCamera(float3 position, float3 direction, float3 approx_up);
	void Clear();
	virtual void DrawScene();
	int Save(std::string filename) const;
	// Public method to compare the final image with a reference
	std::vector<byte3> GetFrameBuffer() const { return frame_buffer; }
protected:
	void SetPixel(const uint32_t x, const uint32_t y, const float3 color);
	virtual Payload TraceRay(const Ray& ray, const uint32_t max_raytrace_depth) const;

	virtual Payload Miss(const Ray& ray) const;

	uint32_t width;
	uint32_t height;

	uint32_t raytracing_depth = 10;

	std::vector<byte3> frame_buffer;
	Camera camera;
};