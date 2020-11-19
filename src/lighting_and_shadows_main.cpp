#include "lighting_and_shadows.h"

#include <chrono>
namespace chrono = std::chrono;

int main(int argc, char* argv[])
{
	std::ios_base::sync_with_stdio(false);
	LightingAndShadows* render = new LightingAndShadows(1980, 1080);
	int result = render->LoadGeometry("models/CornellBox-Original.obj");
	if (result)
	{
		return result;
	}
	auto start = chrono::high_resolution_clock::now();
	render->SetCamera(float3{ 0, 1.1f, 5 }, float3{ 0, 1, -1 }, float3{ 0, 1, 0 });
	render->AddLight(Light(float3{ 0, 1.9f, 0.16f }, float3{ 0.78f, 0.78f, 0.78f }));
	render->Clear();
	render->DrawScene();
	auto end = chrono::high_resolution_clock::now();
	std::cout << chrono::duration_cast<chrono::milliseconds>(end - start).count() << '\n';
	result = render->Save("results/lighting.png");
	return result;
}