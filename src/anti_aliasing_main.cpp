#include "anti_aliasing.h"

int main(int argc, char* argv[])
{
	AntiAliasing* render = new AntiAliasing(1920, 1080);
	int result = render->LoadGeometry("models/CornellBox-Original.obj");
	if (result)
	{
		return result;
	}
	render->SetCamera(float3{ 0, 1.1f, 5 }, float3{ 0, 1, -1 }, float3{ 0, 1, 0 });
	render->AddLight(Light(float3{ 0, 1.9f, 0.16f }, float3{ 0.58f, 0.58f, 0.58f }));
	render->Clear();
	render->DrawScene();
	result = render->Save("results/anti_aliasing.png");
	return result;
}