#pragma once
#include <cgv/media/axis_aligned_box.h>

class triangle {

	typedef cgv::math::fvec<float, 3> vec3;
	typedef typename cgv::media::axis_aligned_box<float, 3> box_type;
	vec3 v_0;
	vec3 v_1;
	vec3 v_2;
public:
	triangle() {

	}
	triangle(vec3& v_0, vec3& v_1, vec3& v_2) : v_0(v_0), v_1(v_1), v_2(v_2) {

	}
	box_type compute_box() {
		box_type box;
		box.add_point(v_0);
		box.add_point(v_1);
		box.add_point(v_2);
		return box;
	}
	void show() {
		std::cout << "tri: " << v_0 << " " << v_1 << " " << v_2 << " " << std::endl;
	}
	vec3 reference_point()const {
		return (v_0 + v_1 + v_2) / 3.0f;
	}
	std::vector<vec3> get() {
		std::vector<vec3> tri;
		tri.push_back(v_0);
		tri.push_back(v_1);
		tri.push_back(v_2);
		return tri;
	}
};