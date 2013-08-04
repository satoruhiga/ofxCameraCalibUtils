#pragma once

#include "ofMain.h"

inline bool calcRayCrossing(const ofMatrix4x4& m, const ofVec3f &ray, ofVec3f &result)
{
	const ofVec3f p = m.preMult(ofVec3f(0, 0, 0));
	const ofVec3f n = m.preMult(ofVec4f(0, 0, 1, 0));
	
	float a = n.x;
	float b = n.y;
	float c = n.z;
	float d = -(a * p.x + b * p.y + c * p.z);
	
	float dot = n.dot(ray);
	if (dot == 0)
		return false;
	
	float dT = d / dot;
	result = ray * -dT;
	
	return true;
}
