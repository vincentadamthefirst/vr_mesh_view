#pragma once

#include <cgv\math\fvec.h>
#include <cgv\render\render_types.h>
#include <cgv\math\fmat.h>

/// type of 3d vector
typedef typename cgv::math::fvec<float, 3> vec3;
typedef typename cgv::render::render_types::mat3 mat3;

struct OcculusController {
	// buttons of the controller
	bool yButton;
	bool yChanged;

	bool xButton;
	bool xChanged;

	bool menuButon;
	bool menuChanged;

	bool trigger;
	bool tiggerChanged;

	bool grab;
	bool grabChanged;

	bool stickTouch;
	bool stickTouchChanged;

	// values of the controller
	vec3 position;
	vec3 lastPosition;
	mat3 rotation;
	mat3 lastRotation;

	// stick information
	bool stickMoved;
	float stickX;
	float stickY;
};
