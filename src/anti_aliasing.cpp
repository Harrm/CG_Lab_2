#include "anti_aliasing.h"

AntiAliasing::AntiAliasing(short width, short height) : LightingAndShadows(width, height)
{
}

void AntiAliasing::DrawScene()
{
	camera.SetRenderTargetSize(width * 2, height * 2);
	for (int32_t x = 0; x < width; x++) {
		for (int32_t y = 0; y < height; y++) {
			auto ray1 = camera.GetCameraRay(2 * x, 2 * y);
			auto payload1 = TraceRay(ray1, raytracing_depth);

			auto ray2 = camera.GetCameraRay(2 * x + 1, 2 * y);
			auto payload2 = TraceRay(ray2, raytracing_depth);

			auto ray3 = camera.GetCameraRay(2 * x, 2 * y + 1);
			auto payload3 = TraceRay(ray3, raytracing_depth);

			auto ray4 = camera.GetCameraRay(2 * x + 1, 2 * y + 1);
			auto payload4 = TraceRay(ray4, raytracing_depth);

			auto color = (payload1.color + payload2.color + payload3.color + payload4.color) / 4.f;
			SetPixel(x, y, color);
		}
	}

}
