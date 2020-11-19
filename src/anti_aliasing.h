#pragma once

#include "acceleration_structures.h"

class AntiAliasing : public LightingAndShadows
{
public:
	AntiAliasing(short width, short height);
	virtual ~AntiAliasing() = default;
	virtual void DrawScene();
};