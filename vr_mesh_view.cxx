
#include "vr_mesh_view.h"

#include <cgv/signal/rebind.h>
#include <cgv/base/register.h>
#include <cgv/math/ftransform.h>
#include <cgv/utils/scan.h>
#include <cgv/utils/options.h>
#include <cgv/gui/dialog.h>
#include <cgv/render/attribute_array_binding.h>
#include <cgv_gl/sphere_renderer.h>
#include <cgv/media/mesh/simple_mesh.h>
#include <cg_vr/vr_events.h>

#include <random>

#include "intersection.h"
#include "ray_intersection.h"

// typedefs
typedef cgv::media::mesh::simple_mesh<float> mesh_type;
typedef mesh_type::idx_type idx_type;
typedef mesh_type::vec3i vec3i;

std::vector<vec3> color_list;
int pathi = 0;

bool rightButton2IsPressed = false;

bool animation_start = false;
vec3 mesh_centroid;

std::vector<vec3> defined_path2;
std::vector<vec3> defined_path;








void vr_mesh_view::init_cameras(vr::vr_kit* kit_ptr)
{
	vr::vr_camera* camera_ptr = kit_ptr->get_camera();
	if (!camera_ptr)
		return;
	nr_cameras = camera_ptr->get_nr_cameras();
	frame_split = camera_ptr->get_frame_split();
	for (int i = 0; i < nr_cameras; ++i) {
		std::cout << "camera " << i << "(" << nr_cameras << "):" << std::endl;
		camera_ptr->put_camera_intrinsics(i, false, &focal_lengths[i](0), &camera_centers[i](0));
		camera_ptr->put_camera_intrinsics(i, true, &focal_lengths[2 + i](0), &camera_centers[2 + i](0));
		std::cout << "  fx=" << focal_lengths[i][0] << ", fy=" << focal_lengths[i][1] << ", center=[" << camera_centers[i] << "]" << std::endl;
		std::cout << "  fx=" << focal_lengths[2+i][0] << ", fy=" << focal_lengths[2+i][1] << ", center=[" << camera_centers[2+i] << "]" << std::endl;
		float camera_to_head[12];
		camera_ptr->put_camera_to_head_matrix(i, camera_to_head);
		kit_ptr->put_eye_to_head_matrix(i, camera_to_head);
		camera_to_head_matrix[i] = vr::get_mat4_from_pose(camera_to_head);
		std::cout << "  C2H=" << camera_to_head_matrix[i] << std::endl;
		camera_ptr->put_projection_matrix(i, false, 0.001f, 10.0f, &camera_projection_matrix[i](0, 0));
		camera_ptr->put_projection_matrix(i, true, 0.001f, 10.0f, &camera_projection_matrix[2+i](0, 0));
		std::cout << "  dP=" << camera_projection_matrix[i] << std::endl;
		std::cout << "  uP=" << camera_projection_matrix[2+i] << std::endl;
	}
	post_recreate_gui();
}

void vr_mesh_view::start_camera()
{
	if (!vr_view_ptr)
		return;
	vr::vr_kit* kit_ptr = vr_view_ptr->get_current_vr_kit();
	if (!kit_ptr)
		return;
	vr::vr_camera* camera_ptr = kit_ptr->get_camera();
	if (!camera_ptr)
		return;
	if (!camera_ptr->start())
		cgv::gui::message(camera_ptr->get_last_error());
}

void vr_mesh_view::stop_camera()
{
	if (!vr_view_ptr)
		return;
	vr::vr_kit* kit_ptr = vr_view_ptr->get_current_vr_kit();
	if (!kit_ptr)
		return;
	vr::vr_camera* camera_ptr = kit_ptr->get_camera();
	if (!camera_ptr)
		return;
	if (!camera_ptr->stop())
		cgv::gui::message(camera_ptr->get_last_error());
}

/// compute intersection points of controller ray with movable boxes
void vr_mesh_view::compute_intersections(const vec3& origin, const vec3& direction, int ci, const rgb& color)
{
	// transform to local environment for HE-Mesh
	vec3 new_origin = global_to_local(origin);
	vec3 point_on_ray = origin + direction;
	vec3 new_point_on_ray = global_to_local(point_on_ray);
	vec3 new_dir = new_point_on_ray - new_origin;

	// create ray
	const ray_intersection::ray r = ray_intersection::ray(new_origin, new_dir);
	float t = 0.0;

	if (aabb_tree.is_completed() && ray_intersection::rayTreeIntersect(r, aabb_tree, t)) {
		vec3 intersection_point = ray_intersection::getIntersectionPoint(r, t);

		//transform back to global
		vec3 new_pos;
		new_pos = local_to_global(intersection_point);
		intersection_point = new_pos;

		// store intersection information
		intersection_points.push_back(intersection_point);
		intersection_offsets.push_back(t);
		intersection_colors.push_back(color);
		intersection_controller_indices.push_back(ci);
		intersection_box_indices.push_back(0);
	}
}

/// register on device change events
void vr_mesh_view::on_device_change(void* kit_handle, bool attach)
{
	if (attach) {
		if (last_kit_handle == 0) {
			vr::vr_kit* kit_ptr = vr::get_vr_kit(kit_handle);
			init_cameras(kit_ptr);
			if (kit_ptr) {
				last_kit_handle = kit_handle;
				left_deadzone_and_precision = kit_ptr->get_controller_throttles_and_sticks_deadzone_and_precision(0);
				cgv::gui::ref_vr_server().provide_controller_throttles_and_sticks_deadzone_and_precision(kit_handle, 0, &left_deadzone_and_precision);
				post_recreate_gui();
			}
		}
	}
	else {
		if (kit_handle == last_kit_handle) {
			last_kit_handle = 0;
			post_recreate_gui();
		}
	}
}

/// construct boxes that represent a room of dimensions w,d,h and wall width W
void vr_mesh_view::construct_room(float w, float d, float h, float W) {
	// construct floor
	if (show_floor) {
		environment_boxes.push_back(box3(vec3(-0.5f * w, -W, -0.5f * d), vec3(0.5f * w, 0, 0.5f * d)));
		box_colors.push_back(rgb(0.2f, 0.2f, 0.2f));
	}
	if(show_walls) {
		// construct walls
		environment_boxes.push_back(box3(vec3(-0.5f*w, -W, -0.5f*d - W), vec3(0.5f*w, h, -0.5f*d)));
		box_colors.push_back(rgb(0.8f, 0.5f, 0.5f));
		environment_boxes.push_back(box3(vec3(-0.5f*w, -W, 0.5f*d), vec3(0.5f*w, h, 0.5f*d + W)));
		box_colors.push_back(rgb(0.8f, 0.5f, 0.5f));

		environment_boxes.push_back(box3(vec3(0.5f*w, -W, -0.5f*d - W), vec3(0.5f*w + W, h, 0.5f*d + W)));
		box_colors.push_back(rgb(0.5f, 0.8f, 0.5f));
	}
	if(show_ceiling) {
		// construct ceiling
		environment_boxes.push_back(box3(vec3(-0.5f*w - W, h, -0.5f*d - W), vec3(0.5f*w + W, h + W, 0.5f*d + W)));
		box_colors.push_back(rgb(0.5f, 0.5f, 0.8f));
	}
}

/// construct boxes for environment
void vr_mesh_view::construct_environment(float s, float ew, float ed, float w, float d, float h) {
	if (!show_colored_boxes) return;

	std::default_random_engine generator;
	std::uniform_real_distribution<float> distribution(0, 1);
	unsigned n = unsigned(ew / s);
	unsigned m = unsigned(ed / s);
	float ox = 0.5f*float(n)*s;
	float oz = 0.5f*float(m)*s;
	for(unsigned i = 0; i < n; ++i) {
		float x = i * s - ox;
		for(unsigned j = 0; j < m; ++j) {
			float z = j * s - oz;
			if(fabsf(x) < 0.5f*w && fabsf(x + s) < 0.5f*w && fabsf(z) < 0.5f*d && fabsf(z + s) < 0.5f*d)
				continue;
			float h = 0.2f*(std::max(abs(x) - 0.5f*w, 0.0f) + std::max(abs(z) - 0.5f*d, 0.0f))*distribution(generator) + 0.1f;
			environment_boxes.push_back(box3(vec3(x, 0.0f, z), vec3(x + s, h, z + s)));
			rgb color = cgv::media::color<float, cgv::media::HLS>(distribution(generator), 0.1f*distribution(generator) + 0.15f, 0.3f);
			box_colors.push_back(color);
		}
	}
}

/// construct a scene with a table
void vr_mesh_view::build_scene(float w, float d, float h, float W, float tw, float td, float th, float tW)
{
	construct_room(w, d, h, W);
	construct_environment(0.3f, 3 * w, 3 * d, w, d, h);
}

vr_mesh_view::vr_mesh_view() 
{
	frame_split = 0;
	extent_texcrd = vec2(0.5f, 0.5f);
	center_left  = vec2(0.5f,0.25f);
	center_right = vec2(0.5f,0.25f);
	seethrough_gamma = 0.33f;
	frame_width = frame_height = 0;
	background_distance = 2;
	background_extent = 2;
	undistorted = true;
	shared_texture = true;
	max_rectangle = false;
	nr_cameras = 0;
	camera_tex_id = -1;
	camera_aspect = 1;
	use_matrix = true;
	show_seethrough = false;
	set_name("vr_mesh_view");
	vr_view_ptr = 0;
	ray_length = 2;
	last_kit_handle = 0;
	connect(cgv::gui::ref_vr_server().on_device_change, this, &vr_mesh_view::on_device_change);

	mesh_scale = 1;
	mesh_location = dvec3(0, 0, 0);
	mesh_orientation = dquat(1, 0, 0, 0);

	srs.radius = 0.005f;

	// for mesh view
	translate_vector = vec3(0.0f);

	// surface renderer default values
	show_surface = true;
	cull_mode = cgv::render::CM_BACKFACE;
	color_mapping = cgv::render::CM_COLOR;
	surface_color = rgb(0.7f, 0.2f, 1.0f);
	illumination_mode = cgv::render::IM_ONE_SIDED;

	// sphere renderer default values
	sphere_style.map_color_to_material = cgv::render::CM_COLOR_AND_OPACITY;
	sphere_style.surface_color = rgb(0.8f, 0.4f, 0.4f);
	sphere_hidden_style.percentual_halo_width = -50;
	sphere_hidden_style.halo_color_strength = 1.0f;
	sphere_hidden_style.halo_color = rgba(0.5f, 0.5f, 0.5f, 0.5f);
	show_vertices = true;

	// wireframe default values
	show_wireframe = true;
	cone_style.surface_color = rgb(1.0f, 0.8f, 0.4f);

	// bounding box default values
	show_bounding_box = false;

	// if a new mesh has been loaded
	have_new_mesh = false;



	show_smoothing = false;


	// ONLY in code changeable values
	show_floor = true;
	show_walls = false;
	show_ceiling = false;
	show_environment_boxes = true;
	show_colored_boxes = true;
	build_scene(5, 7, 3, 0.2f, 1.6f, 0.8f, 0.7f, 0.03f);


	label_outofdate = true;
	label_text = "Surface:\nVolume:";
	label_font_idx = 0;
	label_upright = true;
	label_face_type = cgv::media::font::FFA_BOLD;
	label_resolution = 256;
	label_size = 20.0f;
	label_color = rgb(1, 1, 1);


}
	
void vr_mesh_view::stream_help(std::ostream& os) {
	os << "vr_mesh_view: no shortcuts defined" << std::endl;
}
	
void vr_mesh_view::on_set(void* member_ptr)
{
	if (member_ptr == &file_name) {
		if (cgv::gui::provider::ref_tree_node_visible_flag(file_name)) {
			M.write(file_name);
		}
		else {
			if (read_mesh(file_name))
				have_new_mesh = true;
		}
	}
	update_member(member_ptr);
	post_redraw();
}


float map(float input, float input_start, float input_end, float output_start, float output_end) {
	return output_start + ((output_end - output_start) / (input_end - input_start)) * (input - input_start);
}
	
bool vr_mesh_view::handle(cgv::gui::event& e)
{
	// check if vr event flag is not set and don't process events in this case
	if ((e.get_flags() & cgv::gui::EF_VR) == 0)
		return false;

	// check event id
	switch (e.get_kind()) {
	case cgv::gui::EID_KEY:
	{
		cgv::gui::vr_key_event& vrke = static_cast<cgv::gui::vr_key_event&>(e);

		if (vrke.get_action() == cgv::gui::KA_PRESS) {
			switch (vrke.get_key()) {
			case vr::VR_RIGHT_MENU:
			{
				
				bButtonIsPressed = true;
				animationmode = animationmode ? false : true;
				if(animationmode)
					std::cout << "Animation Mode" << std::endl;
				else
					std::cout << "Mesh Editing Mode" << std::endl;
				label_outofdate = true;
				post_redraw();
				break;
			}
			case vr::VR_LEFT_MENU: 
			{
				animationmode = animationmode ? false : true;
				if (animationmode)
					std::cout << "Animation Mode" << std::endl;
				else
					std::cout << "Mesh Editing Mode" << std::endl;
				yButtonIsPressed = true;
				label_outofdate = true;
				post_redraw();
				break;
			}
				

			case vr::VR_LEFT_BUTTON1:
			{
				auto fs = he->GetFaces();
				HE_Face* f = *fs->begin();
				HE_Face* f2 = *(fs->begin()+2);
				show_selected_smoothing_faces(f);
				show_selected_smoothing_faces(f2);
				break;
			}
				
			case vr::VR_LEFT_STICK_UP:
			//case vr::VR_RIGHT_BUTTON1:
			{
				if (!animationmode) {
					vec3 origin, direction;
					rightButton1IsPressed = true;
					vrke.get_state().controller[0].put_ray(&origin(0), &direction(0));
					tessellation(origin, direction);
					std::cout << "tessellation is here" << std::endl;
				}
				break;
			}
				
			case vr::VR_LEFT_STICK_LEFT: 
			{
				if (!animationmode) {
					leftButton1IsPressed = true;
					std::cout << "VR left stick left is pressed" << std::endl;
					vec3 origin, direction;
					vrke.get_state().controller[0].put_ray(&origin(0), &direction(0));
					//global to local
					vec3 new_origin = global_to_local(origin);
					vec3 point_on_ray = origin + direction;
					vec3 new_point_on_ray = global_to_local(point_on_ray);
					vec3 new_dir = new_point_on_ray - new_origin;
					// create ray
					ray_intersection::ray vertex_ray = ray_intersection::ray(new_origin, new_dir);
					//ray_intersection::ray vertex_ray = ray_intersection::ray(origin, direction);

					float t = 0.0f;
					HE_Face* face;
					bool f = ray_intersection::getIntersectedFace_with_t(vertex_ray, he, t, face);
					if (f) {
						std::cout << "there is an intersecting face" << std::endl;
						std::vector<HE_Vertex*> vertices_of_face = he->GetVerticesForFace(face);

						vec3 local_intersection_point = ray_intersection::getIntersectionPoint(vertex_ray, t);
						if (ray_intersection::vertexIntersection(local_intersection_point, vertices_of_face, intersectedVertex)) {
							isVertexPicked = true;
							std::cout << "Vertex is picked" << std::endl;
						}

					}
					else
						std::cout << "no intersecting face" << std::endl;
				}break;
			}
				
			case vr::VR_RIGHT_STICK_DOWN:
			{
				if (animationmode) {
					rightButton2IsPressed = true;
					vec3 go_origin = defined_path[0] - defined_path[pathi];
					/*add_translation(go_origin);
					mat3 dummyRotation;
					dummyRotation.identity();
					M.transform(dummyRotation, go_origin);*/


					mat3 dummyRotation;
					dummyRotation.identity();
					add_translation(dummyRotation, go_origin);
					//M.transform(dummyRotation, translation);
					M.transform(dummyRotation, go_origin);


					B = M.compute_box();
					have_new_mesh = true;
					post_redraw();
					pathi = 0;
					
				}break;
			}
						
			case vr::VR_LEFT_STICK_RIGHT:
			//case vr::VR_RIGHT_BUTTON1:
			{
				if (!animationmode) {
					if (smoothingpoints.size() == 0) {
						std::cout << "smoothing whole mesh" << std::endl;
						applySmoothing();
					}
					else {
						std::cout << "smoothing some points" << std::endl;
						applySmoothingPoints();
						show_smoothing = true;
					}
					smoothingMesh.clear();
					smoothingpoints.clear();
					
				}break;
			}
			//case vr::VR_RIGHT_BUTTON2:
			case vr::VR_LEFT_STICK_DOWN:
			{
				if (!animationmode) {
					std::cout << "choose smoothing face" << std::endl;
					vec3 origin, direction;
					//rightButton3IsPressed = true;
					vrke.get_state().controller[0].put_ray(&origin(0), &direction(0));
					vec3 new_origin = global_to_local(origin);
					vec3 point_on_ray = origin + direction;
					vec3 new_point_on_ray = global_to_local(point_on_ray);
					vec3 new_dir = new_point_on_ray - new_origin;
					ray_intersection::ray tes_ray = ray_intersection::ray(new_origin, new_dir);
					//ray_intersection::ray tes_ray = ray_intersection::ray(origin, direction);
					float t = 0.0;
					bool tt = ray_intersection::rayTreeIntersect(tes_ray, aabb_tree, t);
					std::cout << tt << std::endl;
					if (tt) {
						HE_Face* face = ray_intersection::getIntersectedFace(tes_ray, he);

						std::vector<HE_Vertex*> vertices_of_face = he->GetVerticesForFace(face);
						for (int i = 0; i < 3; ++i) {
							// f is not a element of boundaryFaces
							if (std::find(smoothingpoints.begin(), smoothingpoints.end(), vertices_of_face[i]) == smoothingpoints.end())
								smoothingpoints.push_back(vertices_of_face[i]);
						}
						std::cout << "select face" << std::endl;
						show_selected_smoothing_faces(face);


					}
				}
			}
			break;
			}
	
		}
		else if (vrke.get_action() == cgv::gui::KA_RELEASE) {
			switch (vrke.get_key()) {
			case vr::VR_RIGHT_MENU:
				bButtonIsPressed = false;
				break;
			case vr::VR_LEFT_MENU:
				yButtonIsPressed = false;
				break;
			case vr::VR_LEFT_STICK_UP:
				rightButton1IsPressed = false;
				break;
			case vr::VR_LEFT_STICK_LEFT:
			{
				if (!animationmode) {
					leftButton1IsPressed = false;
					if (isVertexPicked) {
						M.compute_vertex_normals();
						B = M.compute_box();
						have_new_mesh = true;
						post_redraw();
						std::cout << "Vertex is moved" << std::endl;
						build_aabbtree_from_triangles(he, aabb_tree);
					}
					isVertexPicked = false;
					
				}break;
			}
			case vr::VR_RIGHT_STICK_DOWN:
			{

				{
					rightButton2IsPressed = false;

				}

				break;
			}			
			break;
			}
		}

		return true;
		break;
	}
	case cgv::gui::EID_THROTTLE:
	{
		cgv::gui::vr_throttle_event& vrte = static_cast<cgv::gui::vr_throttle_event&>(e);

		std::cout << "throttle " << vrte.get_throttle_index() << " of controller " << vrte.get_controller_index()
			<< " adjusted from " << vrte.get_last_value() << " to " << vrte.get_value() << std::endl;

		return true;
	}
	case cgv::gui::EID_STICK:
	{
		cgv::gui::vr_stick_event& vrse = static_cast<cgv::gui::vr_stick_event&>(e);
		switch (vrse.get_action()) {
		case cgv::gui::SA_TOUCH:
			if (state[vrse.get_controller_index()] == IS_OVER)
				state[vrse.get_controller_index()] = IS_GRAB;
			break;
		case cgv::gui::SA_RELEASE:
			if (state[vrse.get_controller_index()] == IS_GRAB)
				state[vrse.get_controller_index()] = IS_OVER;
			break;
		case cgv::gui::SA_PRESS:
		case cgv::gui::SA_UNPRESS:
			return true;
		case cgv::gui::SA_MOVE:
		case cgv::gui::SA_DRAG:
			return true;
			return true;
		}
		return true;
	}
	case cgv::gui::EID_POSE:
		cgv::gui::vr_pose_event& vrpe = static_cast<cgv::gui::vr_pose_event&>(e);
		// check for controller pose events
		int ci = vrpe.get_trackable_index();

		if (ci != -1) {
			//get controller ray
			vec3 origin, direction;
			vrpe.get_state().controller[ci].put_ray(&origin(0), &direction(0));

			if (state[ci] == IS_GRAB) {
				// get previous and current intersection point

				for (size_t i = 0; i < intersection_points.size(); ++i) {
					if (intersection_controller_indices[i] != ci)
						continue;

					if (ci == 1) { // right controller
						if (rightButton2IsPressed == false && animationmode) {
							if (animation_start == false) {
								animation_start = true;
								vec3 mesh_centroid = aabb_tree.Root()->get_box().get_center();
								defined_path.push_back(mesh_centroid);
								defined_path2.push_back(mesh_centroid);
							}
							else { defined_path2.push_back(defined_path2[defined_path2.size() - 1]); }



							// get translation between previous and current intersection point
							vec3 new_intersection = origin + intersection_offsets[i] * direction;
							vec3 translation = new_intersection - intersection_points[i];
							intersection_points[i] = new_intersection;


							vec3 new_position = defined_path[defined_path.size() - 1] + translation;
							defined_path.push_back(new_position);
							defined_path2.push_back(new_position);
							pathi++;

							//add_translation(translation);

							mat3 dummyRotation;
							dummyRotation.identity();
							add_translation(dummyRotation, translation);
							//M.transform(dummyRotation, translation);
							M.transform(dummyRotation, translation);

							B = M.compute_box();
							have_new_mesh = true;
							post_redraw();
						}
					}

					if (ci == 0) { // left controller
						//Vertex Manipulation
						if (leftButton1IsPressed && !animationmode) {
							if (isVertexPicked) {
								vec3 last_pos = vrpe.get_last_position();
								vec3 pos = vrpe.get_position();
								vr_mesh_view::vertex_manipulate(intersectedVertex, global_to_local(pos), global_to_local(last_pos));
							}
						}
						else if (rightButton2IsPressed == true && animation_start == true && animationmode) {
								if (pathi != defined_path.size() - 1) {
									vec3 translation_1 = defined_path[pathi + 1] - defined_path[pathi];
									//add_translation(translation_1);
									mat3 dummyRotation;
									dummyRotation.identity();
									add_translation(dummyRotation,translation_1);
									M.transform(dummyRotation, translation_1);
									pathi++;
									B = M.compute_box();
									have_new_mesh = true;
									post_redraw();
								}
							}						
						//Rotation
						else if(animationmode){
							vec3 last_pos = vrpe.get_last_position();

							vec3 pos = vrpe.get_position();



							mat3 rotation = vrpe.get_rotation_matrix();



							vec3 dummyTranslation;

							dummyTranslation.zeros();



							add_rotation(rotation);

							add_translation(rotation, rotation* (dummyTranslation - last_pos) + pos);



							M.transform(rotation, rotation* (dummyTranslation - last_pos) + pos);



							// mesh is animated

							B = M.compute_box();

							have_new_mesh = true;

							post_redraw();
						}
						
						
					}
				}

			}
			else {// not grab
				// clear intersections of current controller 
				size_t i = 0;
				while (i < intersection_points.size()) {
					if (intersection_controller_indices[i] == ci) {
						intersection_points.erase(intersection_points.begin() + i);
						intersection_offsets.erase(intersection_offsets.begin() + i);
						intersection_colors.erase(intersection_colors.begin() + i);
						intersection_box_indices.erase(intersection_box_indices.begin() + i);
						intersection_controller_indices.erase(intersection_controller_indices.begin() + i);
					}
					else
						++i;
				}

				// compute intersections
				/*if (defined_path.size()>0){
				compute_intersections(origin+defined_path[0]- defined_path[pathi], direction, ci, ci == 0 ? rgb(1, 0, 0) : rgb(0, 0, 1));
				}


				else{
					compute_intersections(origin, direction, ci, ci == 0 ? rgb(1, 0, 0) : rgb(0, 0, 1));
					
				}*/
				compute_intersections(origin, direction, ci, ci == 0 ? rgb(1, 0, 0) : rgb(0, 0, 1));

				if (ci == 0) { // left controller
					leftControllerPosition = origin;
				}
				else if (ci == 1) { // right contoller
					rightControllerPosition = origin;
				}

				// update state based on whether we have found at least 
				// one intersection with controller ray
				if (intersection_points.size() == i)
					state[ci] = IS_NONE;
				else
					if (state[ci] == IS_NONE)
						state[ci] = IS_OVER;
			}
			post_redraw();
		}
		return true;
	}
	return false;
}

bool vr_mesh_view::init(cgv::render::context& ctx)
{
	if (!cgv::utils::has_option("NO_OPENVR"))
		ctx.set_gamma(1.0f);

	cgv::gui::connect_vr_server(true);

	auto view_ptr = find_view_as_node();
	if (view_ptr) {
		view_ptr->set_eye_keep_view_angle(dvec3(0, 4, -4));
		// if the view points to a vr_view_interactor
		vr_view_ptr = dynamic_cast<vr_view_interactor*>(view_ptr);
		if (vr_view_ptr) {
			// configure vr event processing
			vr_view_ptr->set_event_type_flags(
				cgv::gui::VREventTypeFlags(
					cgv::gui::VRE_KEY +
					cgv::gui::VRE_THROTTLE +
					cgv::gui::VRE_STICK +
					cgv::gui::VRE_STICK_KEY +
					cgv::gui::VRE_POSE +
					cgv::gui::VRE_DEVICE +
					cgv::gui::VRE_STATUS
			));
			vr_view_ptr->enable_vr_event_debugging(false);
			// configure vr rendering
			vr_view_ptr->draw_action_zone(false);
			vr_view_ptr->draw_vr_kits(true);
			vr_view_ptr->enable_blit_vr_views(true);
			vr_view_ptr->set_blit_vr_view_width(200);
		}
	}

	if (!mesh_prog.build_program(ctx, "mesh.glpr", true))
		abort();
	if (!mesh_prog_smoothing.build_program(ctx, "mesh.glpr", true))
		abort();

	cgv::render::ref_box_renderer(ctx, 1);
	cgv::render::ref_sphere_renderer(ctx, 1);
	cgv::render::ref_rounded_cone_renderer(ctx, 1);
	return true;
}

void vr_mesh_view::clear(cgv::render::context& ctx)
{
	mesh_prog.destruct(ctx);
	mesh_prog_smoothing.destruct(ctx);
	cgv::render::ref_box_renderer(ctx, -1);
	cgv::render::ref_sphere_renderer(ctx, -1);
	cgv::render::ref_rounded_cone_renderer(ctx, -1);
}

void vr_mesh_view::init_frame(cgv::render::context& ctx)
{
	if (label_fbo.get_width() != label_resolution) {
		label_tex.destruct(ctx);
		label_fbo.destruct(ctx);
	}
	if (!label_fbo.is_created()) {
		label_tex.create(ctx, cgv::render::TT_2D, label_resolution, label_resolution);
		label_fbo.create(ctx, label_resolution, label_resolution);
		label_tex.set_min_filter(cgv::render::TF_LINEAR_MIPMAP_LINEAR);
		label_tex.set_mag_filter(cgv::render::TF_LINEAR);
		label_fbo.attach(ctx, label_tex);
		label_outofdate = true;
	}
	if (label_outofdate && label_fbo.is_complete(ctx)) {
		glPushAttrib(GL_COLOR_BUFFER_BIT);
		label_fbo.enable(ctx);
		label_fbo.push_viewport(ctx);
		ctx.push_pixel_coords();
		glClearColor(0.5f, 0.5f, 0.5f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT);

		glColor4f(label_color[0], label_color[1], label_color[2], 1);
		ctx.set_cursor(20, (int)ceil(label_size) + 20);
		ctx.enable_font_face(label_font_face, label_size);
		ctx.output_stream() << (animationmode ? animationmode_text : mesheditingmode_text) << label_text << "\n";
		ctx.output_stream().flush(); // make sure to flush the stream before change of font size or font face

		ctx.enable_font_face(label_font_face, 0.7f * label_size);
		/*for (size_t i = 0; i < intersection_points.size(); ++i) {
			ctx.output_stream()
				<< "box " << intersection_box_indices[i]
				<< " at (" << intersection_points[i]
				<< ") with controller " << intersection_controller_indices[i] << "\n";
		}*/
		ctx.output_stream().flush();

		ctx.pop_pixel_coords();
		label_fbo.pop_viewport(ctx);
		label_fbo.disable(ctx);
		glPopAttrib();
		label_outofdate = false;

		label_tex.generate_mipmaps(ctx);
	}
	if (have_new_mesh) {
		if (!M.get_positions().empty()) {
			if (!M.has_normals())
				M.compute_vertex_normals();

			MI.destruct(ctx);
			MI.construct(ctx, M);
			MI.bind(ctx, mesh_prog, true);
			MI.bind_wireframe(ctx, cgv::render::ref_rounded_cone_renderer(ctx).ref_prog(), true);
		}
	}
	have_new_mesh = false;
	if (have_new_smoothingMesh) {
		if (!smoothingMesh.get_positions().empty()) {
			if (!smoothingMesh.has_normals())
				smoothingMesh.compute_vertex_normals();
			MI_smoothing.destruct(ctx);
			MI_smoothing.construct(ctx, smoothingMesh);
			MI_smoothing.bind(ctx, mesh_prog_smoothing, true);
			MI_smoothing.bind_wireframe(ctx, cgv::render::ref_rounded_cone_renderer(ctx).ref_prog(), true);
		}
	}
	have_new_smoothingMesh = false;

	if (show_smoothing) {
		MI_smoothing.destruct(ctx);
		show_smoothing = false;
	}
		



	if (vr_view_ptr && vr_view_ptr->get_rendered_vr_kit() != 0 && vr_view_ptr->get_rendered_eye() == 0 && vr_view_ptr->get_rendered_vr_kit() == vr_view_ptr->get_current_vr_kit()) {
		vr::vr_kit* kit_ptr = vr_view_ptr->get_current_vr_kit();
		if (kit_ptr) {
			vr::vr_camera* camera_ptr = kit_ptr->get_camera();
			if (camera_ptr && camera_ptr->get_state() == vr::CS_STARTED) {
				uint32_t width = frame_width, height = frame_height, split = frame_split;
				if (shared_texture) {
					box2 tex_range;
					if (camera_ptr->get_gl_texture_id(camera_tex_id, width, height, undistorted, &tex_range.ref_min_pnt()(0))) {
						camera_aspect = (float)width / height;
						split = camera_ptr->get_frame_split();
						switch (split) {
						case vr::CFS_VERTICAL:
							camera_aspect *= 2;
							break;
						case vr::CFS_HORIZONTAL:
							camera_aspect *= 0.5f;
							break;
						}
					}
					else
						camera_tex_id = -1;
				}
				else {
					std::vector<uint8_t> frame_data;
					if (camera_ptr->get_frame(frame_data, width, height, undistorted, max_rectangle)) {
						camera_aspect = (float)width / height;
						split = camera_ptr->get_frame_split();
						switch (split) {
						case vr::CFS_VERTICAL:
							camera_aspect *= 2;
							break;
						case vr::CFS_HORIZONTAL:
							camera_aspect *= 0.5f;
							break;
						}
						cgv::data::data_format df(width, height, cgv::type::info::TI_UINT8, cgv::data::CF_RGBA);
						cgv::data::data_view dv(&df, frame_data.data());
						if (camera_tex.is_created()) {
							if (camera_tex.get_width() != width || camera_tex.get_height() != height)
								camera_tex.destruct(ctx);
							else
								camera_tex.replace(ctx, 0, 0, dv);
						}
						if (!camera_tex.is_created())
							camera_tex.create(ctx, dv);
					}
					else if (camera_ptr->has_error())
						cgv::gui::message(camera_ptr->get_last_error());
				}
				if (frame_width != width || frame_height != height) {
					frame_width = width;
					frame_height = height;

					center_left(0) = camera_centers[2](0) / frame_width;
					center_left(1) = camera_centers[2](1) / frame_height;
					center_right(0) = camera_centers[3](0) / frame_width;
					center_right(1) = camera_centers[3](1) / frame_height;

					update_member(&frame_width);
					update_member(&frame_height);
					update_member(&center_left(0));
					update_member(&center_left(1));
					update_member(&center_right(0));
					update_member(&center_right(1));
				}
				if (split != frame_split) {
					frame_split = split;
					update_member(&frame_split);
				}
			}
		}
	}
}

void vr_mesh_view::draw_surface(cgv::render::context& ctx, bool opaque_part)
{
	// remember current culling setting
	GLboolean is_culling = glIsEnabled(GL_CULL_FACE);
	GLint cull_face;
	glGetIntegerv(GL_CULL_FACE_MODE, &cull_face);

	// ensure that opengl culling is identical to shader program based culling
	if (cull_mode > 0) {
		glEnable(GL_CULL_FACE);
		glCullFace(cull_mode == cgv::render::CM_BACKFACE ? GL_BACK : GL_FRONT);
	}
	else
		glDisable(GL_CULL_FACE);

	// choose a shader program and configure it based on current settings
	cgv::render::shader_program& prog = mesh_prog;

	prog.set_uniform(ctx, "culling_mode", (int)cull_mode);
	prog.set_uniform(ctx, "map_color_to_material", (int)color_mapping);
	prog.set_uniform(ctx, "illumination_mode", (int)illumination_mode);
	// set default surface color for color mapping which only affects 
	// rendering if mesh does not have per Vector colors and color_mapping is on
	if (prog.get_color_index() != -1)
		prog.set_attribute(ctx, prog.get_color_index(), surface_color);

	// render the mesh from the Vector buffers with selected program
	if (!M.get_positions().empty())
		MI.draw_all(ctx, !opaque_part, opaque_part);
	if (!r_info.ref_draw_calls().empty())
		r_info.draw_all(ctx, !opaque_part, opaque_part);

	// recover opengl culling mode
	if (is_culling)
		glEnable(GL_CULL_FACE);
	else
		glDisable(GL_CULL_FACE);
	glCullFace(cull_face);
}
void vr_mesh_view::draw_surface_2(cgv::render::context& ctx, bool opaque_part) {

	// remember current culling setting
	/*GLboolean is_culling = glIsEnabled(GL_CULL_FACE);
	GLint cull_face;
	glGetIntegerv(GL_CULL_FACE_MODE, &cull_face);

	// ensure that opengl culling is identical to shader program based culling
	if (cull_mode > 0) {
		glEnable(GL_CULL_FACE);
		glCullFace(cull_mode == cgv::render::CM_BACKFACE ? GL_BACK : GL_FRONT);
	}
	else
		glDisable(GL_CULL_FACE);*/

	// choose a shader program and configure it based on current settings
	cgv::render::shader_program& prog = mesh_prog_smoothing;

	//prog.set_uniform(ctx, "culling_mode", (int)cull_mode);
	prog.set_uniform(ctx, "map_color_to_material", (int)color_mapping);
	//prog.set_uniform(ctx, "illumination_mode", (int)illumination_mode);
	// set default surface color for color mapping which only affects 
	// rendering if mesh does not have per Vector colors and color_mapping is on
	if (prog.get_color_index() != -1)
		prog.set_attribute(ctx, prog.get_color_index(), rgb(0.7f, 0.2f, 1.0f));
	MI_smoothing.draw_all(ctx, !opaque_part, opaque_part);

}
void vr_mesh_view::draw(cgv::render::context& ctx)
{
	// draw label
	if (label_tex.is_created()) {
		cgv::render::shader_program& prog = ctx.ref_default_shader_program(true);
		int pi = prog.get_position_index();
		int ti = prog.get_texcoord_index();
		vec3 p(0, 1.5f, 0);
		vec3 y = label_upright ? vec3(0, 1.0f, 0) : normalize(vr_view_ptr->get_view_up_dir_of_kit());
		vec3 x = normalize(cross(vec3(vr_view_ptr->get_view_dir_of_kit()), y));
		float w = 0.5f, h = 0.5f;
		std::vector<vec3> P;
		std::vector<vec2> T;
		P.push_back(p - 0.5f * w * x - 0.5f * h * y); T.push_back(vec2(0.0f, 0.0f));
		P.push_back(p + 0.5f * w * x - 0.5f * h * y); T.push_back(vec2(1.0f, 0.0f));
		P.push_back(p - 0.5f * w * x + 0.5f * h * y); T.push_back(vec2(0.0f, 1.0f));
		P.push_back(p + 0.5f * w * x + 0.5f * h * y); T.push_back(vec2(1.0f, 1.0f));
		cgv::render::attribute_array_binding::set_global_attribute_array(ctx, pi, P);
		cgv::render::attribute_array_binding::enable_global_array(ctx, pi);
		cgv::render::attribute_array_binding::set_global_attribute_array(ctx, ti, T);
		cgv::render::attribute_array_binding::enable_global_array(ctx, ti);
		prog.enable(ctx);
		label_tex.enable(ctx);
		ctx.set_color(rgb(1, 1, 1));
		glDrawArrays(GL_TRIANGLE_STRIP, 0, (GLsizei)P.size());
		label_tex.disable(ctx);
		prog.disable(ctx);
		cgv::render::attribute_array_binding::disable_global_array(ctx, pi);
		cgv::render::attribute_array_binding::disable_global_array(ctx, ti);
	}
	if (defined_path2.size() > 1) {
		//drawpath(ctx, path_list_1);
		drawpath(ctx, defined_path2);
	}
	if (vr_view_ptr) {
		if ((!shared_texture && camera_tex.is_created()) || (shared_texture && camera_tex_id != -1)) {
			if (vr_view_ptr->get_rendered_vr_kit() != 0 && vr_view_ptr->get_rendered_vr_kit() == vr_view_ptr->get_current_vr_kit()) {
				int eye = vr_view_ptr->get_rendered_eye();

				// compute billboard
				dvec3 vd = vr_view_ptr->get_view_dir_of_kit();
				dvec3 y = vr_view_ptr->get_view_up_dir_of_kit();
				dvec3 x = normalize(cross(vd, y));
				y = normalize(cross(x, vd));
				x *= camera_aspect * background_extent * background_distance;
				y *= background_extent * background_distance;
				vd *= background_distance;
				dvec3 eye_pos = vr_view_ptr->get_eye_of_kit(eye);
				std::vector<vec3> P;
				std::vector<vec2> T;
				P.push_back(eye_pos + vd - x - y);
				P.push_back(eye_pos + vd + x - y);
				P.push_back(eye_pos + vd - x + y);
				P.push_back(eye_pos + vd + x + y);
				double v_offset = 0.5 * (1 - eye);
				T.push_back(dvec2(0.0, 0.5 + v_offset));
				T.push_back(dvec2(1.0, 0.5 + v_offset));
				T.push_back(dvec2(0.0, v_offset));
				T.push_back(dvec2(1.0, v_offset));

				cgv::render::shader_program& prog = seethrough;
				cgv::render::attribute_array_binding::set_global_attribute_array(ctx, prog.get_position_index(), P);
				cgv::render::attribute_array_binding::set_global_attribute_array(ctx, prog.get_texcoord_index(), T);
				cgv::render::attribute_array_binding::enable_global_array(ctx, prog.get_position_index());
				cgv::render::attribute_array_binding::enable_global_array(ctx, prog.get_texcoord_index());

				GLint active_texture, texture_binding;
				if (shared_texture) {
					glGetIntegerv(GL_ACTIVE_TEXTURE, &active_texture);
					glGetIntegerv(GL_TEXTURE_BINDING_2D, &texture_binding);
					glActiveTexture(GL_TEXTURE0);
					glBindTexture(GL_TEXTURE_2D, camera_tex_id);
				}
				else
					camera_tex.enable(ctx, 0);
				prog.set_uniform(ctx, "texture", 0);
				prog.set_uniform(ctx, "seethrough_gamma", seethrough_gamma);
				prog.set_uniform(ctx, "use_matrix", use_matrix);

				// use of convenience function
				vr::configure_seethrough_shader_program(ctx, prog, frame_width, frame_height,
					vr_view_ptr->get_current_vr_kit(), *vr_view_ptr->get_current_vr_state(),
					0.01f, 2 * background_distance, eye, undistorted);

				prog.enable(ctx);
				ctx.set_color(rgba(1, 1, 1, 1));

				glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);

				prog.disable(ctx);

				if (shared_texture) {
					glActiveTexture(active_texture);
					glBindTexture(GL_TEXTURE_2D, texture_binding);
				}
				else
					camera_tex.disable(ctx);

				cgv::render::attribute_array_binding::disable_global_array(ctx, prog.get_position_index());
				cgv::render::attribute_array_binding::disable_global_array(ctx, prog.get_texcoord_index());
			}
		}
		if (vr_view_ptr) {
			std::vector<vec3> P;
			std::vector<float> R;
			std::vector<rgb> C;
			const vr::vr_kit_state* state_ptr = vr_view_ptr->get_current_vr_state();
			if (state_ptr) {
				for (int ci = 0; ci < 4; ++ci) if (state_ptr->controller[ci].status == vr::VRS_TRACKED) {
					vec3 ray_origin, ray_direction;
					state_ptr->controller[ci].put_ray(&ray_origin(0), &ray_direction(0));
					P.push_back(ray_origin);
					R.push_back(0.002f);
					P.push_back(ray_origin + ray_length * ray_direction);
					R.push_back(0.003f);
					rgb c(float(1 - ci), 0.5f * (int)state[ci], float(ci));
					C.push_back(c);
					C.push_back(c);
				}
			}
			if (P.size() > 0) {
				auto& cr = cgv::render::ref_rounded_cone_renderer(ctx);
				cr.set_render_style(cone_style);
				//cr.set_eye_position(vr_view_ptr->get_eye_of_kit());
				cr.set_position_array(ctx, P);
				cr.set_color_array(ctx, C);
				cr.set_radius_array(ctx, R);
				if (!cr.render(ctx, 0, P.size())) {
					cgv::render::shader_program& prog = ctx.ref_default_shader_program();
					int pi = prog.get_position_index();
					int ci = prog.get_color_index();
					cgv::render::attribute_array_binding::set_global_attribute_array(ctx, pi, P);
					cgv::render::attribute_array_binding::enable_global_array(ctx, pi);
					cgv::render::attribute_array_binding::set_global_attribute_array(ctx, ci, C);
					cgv::render::attribute_array_binding::enable_global_array(ctx, ci);
					glLineWidth(3);
					prog.enable(ctx);
					glDrawArrays(GL_LINES, 0, (GLsizei)P.size());
					prog.disable(ctx);
					cgv::render::attribute_array_binding::disable_global_array(ctx, pi);
					cgv::render::attribute_array_binding::disable_global_array(ctx, ci);
					glLineWidth(1);
				}
			}
		}
	}

	// draw intersection points
	if (!intersection_points.empty()) {
		auto& sr = cgv::render::ref_sphere_renderer(ctx);
		sr.set_position_array(ctx, intersection_points);
		sr.set_color_array(ctx, intersection_colors);
		sr.set_render_style(srs);
		sr.render(ctx, 0, intersection_points.size());
	}
	// render smoothing mesh
	if (MI_smoothing.is_constructed())
		draw_surface_2(ctx, true);
	// check if mesh is loaded
	if (MI.is_constructed()) {
		// draw mesh
		MI.draw_all(ctx, false, true);

		// draw bounding box
		if (show_bounding_box) {
			cgv::render::box_wire_renderer& box_render = cgv::render::ref_box_wire_renderer(ctx);
			box_render.init(ctx);
			visit_tree(aabb_tree.Root());
			box_render.set_box_array(ctx, boxes);
			//box_render.render(ctx, 0, boxes.size());

			if (box_render.validate_and_enable(ctx)) {
				glDrawArrays(GL_POINTS, 0, (GLsizei) boxes.size());
				box_render.disable(ctx);
			}
		}

		// draw wireframe
		if (show_wireframe) {
			cgv::render::rounded_cone_renderer& cr = ref_rounded_cone_renderer(ctx);
			cr.set_render_style(cone_style);
			if (cr.enable(ctx)) {
				MI.draw_wireframe(ctx);
				cr.disable(ctx);
			}
		}

		// draw vertices
		if (show_vertices) {
			cgv::render::sphere_renderer& sr = ref_sphere_renderer(ctx);
			if (vr_view_ptr)
				sr.set_y_view_angle(float(vr_view_ptr->get_y_view_angle()));
			glEnable(GL_BLEND);
			glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
			float tmp = sphere_style.radius_scale;
			sphere_style.radius_scale = 1;
			sr.set_render_style(sphere_style);
			sr.set_position_array(ctx, M.get_positions());
			if (M.has_colors())
				sr.set_color_array(ctx, *reinterpret_cast<const std::vector<rgb>*>(M.get_color_data_vector_ptr()));
			sr.render(ctx, 0, M.get_nr_positions());
			sphere_style.radius_scale = tmp;
			glDisable(GL_BLEND);
		}

		//ctx.pop_modelview_matrix();
	}

	


	// draw environment (if list isnt empty)
	if (!environment_boxes.empty() && show_environment_boxes) {
		cgv::render::box_renderer& renderer = cgv::render::ref_box_renderer(ctx);
		//renderer.set_render_style(style);
		renderer.set_box_array(ctx, environment_boxes);
		renderer.set_color_array(ctx, box_colors);
		renderer.render(ctx, 0, environment_boxes.size());
	}

	if (!r_info.ref_draw_calls().empty())
		r_info.draw_all(ctx, false, true);
}

void vr_mesh_view::finish_draw(cgv::render::context& ctx)
{
	return;
	if ((!shared_texture && camera_tex.is_created()) || (shared_texture && camera_tex_id != -1)) {
		cgv::render::shader_program& prog = ctx.ref_default_shader_program(true);
		glEnable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
		GLint active_texture, texture_binding;
		if (shared_texture) {
			glGetIntegerv(GL_ACTIVE_TEXTURE, &active_texture);
			glGetIntegerv(GL_TEXTURE_BINDING_2D, &texture_binding);
			glActiveTexture(GL_TEXTURE0);
			glBindTexture(GL_TEXTURE_2D, camera_tex_id);
		}
		else
			camera_tex.enable(ctx, 0);

		prog.set_uniform(ctx, "texture", 0);
		ctx.push_modelview_matrix();
		ctx.mul_modelview_matrix(cgv::math::translate4<double>(0, 3, 0));
		prog.enable(ctx);
		ctx.set_color(rgba(1, 1, 1, 0.8f));
		ctx.tesselate_unit_square();
		prog.disable(ctx);
		if (shared_texture) {
			glActiveTexture(active_texture);
			glBindTexture(GL_TEXTURE_2D, texture_binding);
		}
		else
			camera_tex.disable(ctx);
		ctx.pop_modelview_matrix();
		glDisable(GL_BLEND);
	}
}

void vr_mesh_view::create_gui() {
	add_decorator("mesh", "heading", "level=2");
	add_gui("file_name", file_name, "file_name",
		"open=true;title='open obj file';filter='mesh (obj):*.obj|all files:*.*';"
		"save=true;save_title='save obj file';w=140");

	align("\a");
	add_member_control(this, "scale", mesh_scale, "value_slider", "min=0.01;max=3;ticks=true;log=true");
	align("\b");

	align("\a");
	add_decorator("Smoothing", "heading", "level=3");
	connect_copy(add_button("apply")->click, rebind(this, &vr_mesh_view::applySmoothing));
	align("\b");
	
	if (begin_tree_node("mesh options", translate_vector, false, "options='w140';align=' '")) {
		align("\a");

		bool show = begin_tree_node("vertices", show_vertices, false, "options='w=100';align=' '");
		add_member_control(this, "show", show_vertices, "toggle", "w=42;shortcut='w'", " ");
		add_member_control(this, "", sphere_style.surface_color, "", "w=42");
		if (show) {
			align("\a");
			add_decorator("visible part", "heading");
			add_gui("style", sphere_style);
			add_decorator("invisible part", "heading");
			add_gui("style", sphere_hidden_style);
			align("\b");
			end_tree_node(show_vertices);
		}

		show = begin_tree_node("wireframe", show_wireframe, false, "options='w=100';align=' '");
		add_member_control(this, "show", show_wireframe, "toggle", "w=42;shortcut='w'", " ");
		add_member_control(this, "", cone_style.surface_color, "", "w=42");
		if (show) {
			align("\a");
			add_gui("style", cone_style);
			align("\b");
			end_tree_node(show_wireframe);
		}

		show = begin_tree_node("bounding_box", show_bounding_box, false, "options='w=100';align=' '");
		add_member_control(this, "show", show_bounding_box, "toggle", "w=42;shortcut='w'", " ");
		end_tree_node(show_bounding_box);

		show = begin_tree_node("surface", show_surface, false, "options='w=100';align=' '");
		add_member_control(this, "show", show_surface, "toggle", "w=42;shortcut='s'", " ");
		add_member_control(this, "", surface_color, "", "w=42");
		if (show) {
			align("\a");
			add_member_control(this, "cull mode", cull_mode, "dropdown", "enums='none,back,front'");
			if (begin_tree_node("color_mapping", color_mapping)) {
				align("\a");
				add_gui("color mapping", color_mapping, "bit_field_control",
					"enums='COLOR_FRONT=1,COLOR_BACK=2,OPACITY_FRONT=4,OPACITY_BACK=8'");
				align("\b");
				end_tree_node(color_mapping);
			}
			add_member_control(this, "surface color", surface_color);
			add_member_control(this, "illumination", illumination_mode, "dropdown", "enums='none,one sided,two sided'");
			// this is how to add a ui for the materials read from an obj material file
			std::vector<cgv::render::textured_material*>* materials = 0;
			if (!MI.ref_materials().empty())
				materials = &MI.ref_materials();
			else if (!r_info.ref_materials().empty())
				materials = &r_info.ref_materials();
			if (materials) {
				for (unsigned mi = 0; mi < materials->size(); ++mi) {
					if (begin_tree_node(materials->at(mi)->get_name(), *materials->at(mi))) {
						align("\a");
						add_gui("mat", static_cast<cgv::media::illum::textured_surface_material&>(*materials->at(mi)));
						align("\b");
						end_tree_node(*materials->at(mi));
					}
				}
			}
			align("\b");
			end_tree_node(show_surface);
		}


		end_tree_node(translate_vector);
	}
}

bool vr_mesh_view::read_mesh(const std::string& file_name)
{
	mesh_type tmp;
	size_t Vector_count = 0;
	if (cgv::utils::to_lower(cgv::utils::file::get_extension(file_name)) == "gltf") {
		fx::gltf::Document doc = fx::gltf::LoadFromText(file_name);
		if (get_context()) {
			cgv::render::context& ctx = *get_context();
			build_render_info(file_name, doc, ctx, r_info);
			r_info.bind(ctx, ctx.ref_surface_shader_program(true), true);
			cgv::render::extract_additional_information(doc, B, Vector_count);
		}
		else
			cgv::render::extract_mesh(file_name, doc, tmp);
	}
	else {
		if (!tmp.read(file_name))
			return false;
	}
	if (tmp.get_nr_positions() > 0) {
		M = tmp;
		B = M.compute_box();
		Vector_count = M.get_nr_positions();

		// initialize transformation matrices
		transformation_matrix.identity();
		mesh_rotation_matrix.identity();
		mesh_translation_vector.zeros();

		// scale the mesh
		mat3 scale_matrix;
		scale_matrix.identity();
		scale_matrix *= mesh_scale;
		M.transform(scale_matrix, mesh_translation_vector);

		//create HE_MESH and build bounding box
		he = generate_from_simple_mesh(M);
		build_aabbtree_from_triangles(he, aabb_tree);

		float volume = mesh_utils::volume(he);
		float surface = mesh_utils::surface(he);
		label_text = "Volume: " + std::to_string(volume) + "\nSurface: " + std::to_string(surface);
		label_outofdate = true;

	}
	sphere_style.radius = float(0.05 * sqrt(B.get_extent().sqr_length() / Vector_count));
	on_set(&sphere_style.radius);
	sphere_hidden_style.radius = sphere_style.radius;
	on_set(&sphere_hidden_style.radius);

	cone_style.radius = 0.5f * sphere_style.radius;
	return true;
}

void vr_mesh_view::scaleMesh(mesh_type& M, float scale) {

}

HE_Mesh* vr_mesh_view::generate_from_simple_mesh(mesh_type M) {
	auto newMesh = new HE_Mesh();

	if (M.get_positions().empty()) return nullptr; // mesh is empty, no conversion neccessary

	/// define index type
	typedef cgv::type::uint32_type idx_type;
	/// define index triple type
	typedef cgv::math::fvec<idx_type, 3> vec3i;

	// first (re)compute the normals to make sure they are calculated
	M.compute_vertex_normals();

	auto originalPositions = M.get_positions();
	std::vector<unsigned int> triangleBuffer;
	std::vector<idx_type> vectorIndices;
	std::vector<vec3i> uniqueTriples;

	M.merge_indices(vectorIndices, uniqueTriples, false, false);
	M.extract_triangle_element_buffer(vectorIndices, triangleBuffer);

	for (auto i = 0; i < triangleBuffer.size(); i += 3) {
		unsigned int vectorAIndex = uniqueTriples.at(triangleBuffer.at(i))[0];
		unsigned int vectorBIndex = uniqueTriples.at(triangleBuffer.at(i + 1))[0];
		unsigned int vectorCIndex = uniqueTriples.at(triangleBuffer.at(i + 2))[0];

		// adding the 3 vectors
		auto vectorA = newMesh->AddVector(vectorAIndex, originalPositions.at(vectorAIndex));
		auto vectorB = newMesh->AddVector(vectorBIndex, originalPositions.at(vectorBIndex));
		auto vectorC = newMesh->AddVector(vectorCIndex, originalPositions.at(vectorCIndex));

		auto face = newMesh->AddFace();

		// generating 3 half edges per triangle
		auto halfEdgeC = newMesh->AddHalfEdge(vectorC, vectorA, face);
		auto halfEdgeB = newMesh->AddHalfEdge(vectorB, vectorC, face, halfEdgeC);
		auto halfEdgeA = newMesh->AddHalfEdge(vectorA, vectorB, face, halfEdgeB);

		// closing the loop
		halfEdgeC->next = halfEdgeA;
	}

	// construct boundaries
	for (auto edge_it : *newMesh->GetHalfEdges()) {
		if (edge_it->twin == nullptr)
			newMesh->AddBoundary(edge_it);
	}
	uniqueTriples.clear();
	triangleBuffer.clear();
	vectorIndices.clear();

	return newMesh;
}

//push back the leaf node
void vr_mesh_view::visit_tree(AabbTree<triangle>::AabbNode* a)
{
	if (a->is_leaf() == true)
	{
		boxes.push_back(a->get_box());
	}

	if (a->is_leaf() == false)
	{
		visit_tree(a->left_child());
		visit_tree(a->right_child());
	}
}

// add translation vector to matrix
void vr_mesh_view::add_translation(vec3 v) {
	mat4 mat_translation;
	mat_translation.identity();
	mat_translation.set_col(3, vec4(v, 1));
	transformation_matrix = transformation_matrix * mat_translation;
}

// add translation vector to matrix
void vr_mesh_view::add_translation(mat3 r, vec3 v) {
	mat4 mat_translation;
	mat_translation.identity();
	mat_translation.set_col(3, vec4(v, 1));
	transformation_matrix = transformation_matrix * mat_translation;
	mesh_translation_vector = r * mesh_translation_vector + v;
}

// add rotation to matrix via angle and axis
void vr_mesh_view::add_rotation(float angle, vec3 axis) {
	mat4 rotationmatrix = rotate4(angle, axis);
	transformation_matrix = transformation_matrix * rotationmatrix;
}
// add rotation to matrix via angles
void vr_mesh_view::add_rotation(vec3 angles) {
	mat4 rotationmatrix = rotate4(angles);
	transformation_matrix = transformation_matrix * rotationmatrix;
}

// add rotation to matrix via 3x3 rotation matrix
void vr_mesh_view::add_rotation(mat3 r3) {
	mat4 r4;
	r4.identity();
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			r4(i, j) = r3(i, j);
		}
	}

	transformation_matrix = transformation_matrix * r4;
	mesh_rotation_matrix = r3 * mesh_rotation_matrix;
}

// add rotation to matrix via 3x3 rotation matrix
vec3 vr_mesh_view::global_to_local(vec3 pos) {
	pos = transpose(mesh_rotation_matrix) * (pos - mesh_translation_vector);
	return pos;
}

// returns pos in the local coordinate system
vec3 vr_mesh_view::local_to_global(vec3 pos) {
	pos = (mesh_rotation_matrix * pos) + mesh_translation_vector;
	return pos;
}

//updates Simple mesh from HE_Mesh
/*void vr_mesh_view::updateSimpleMesh() {
	auto originalPositions = M.get_positions();

	for (auto v : *he->GetVertices()) {
		//std::cout << M.position(v->originalIndex) << ", " << v->position << std::endl;;
		M.position(v->originalIndex) = v->position;
	}
	M.compute_vertex_normals();
	B = M.compute_box();
	have_new_mesh = true;
	post_redraw();
}*/

void vr_mesh_view::applySmoothing() {
	if (M.get_positions().empty()) return;
	//he = mesh_utils::smoothing_laplacian(he, transformation_matrix);

	std::vector<vec3> newPositions;

	for (HE_Vertex* v : *he->GetVertices()) {
		int number = 0;
		vec3 newpos = vec3(0, 0, 0);
		for (HE_Vertex* neighbor : he->GetNeighborVertices(v)) {
			number++;
			newpos += neighbor->position;
		}
		newpos /= number;
		newPositions.push_back(newpos);
	}
	int i = 0;
	for (HE_Vertex* v : *he->GetVertices()) {
		//vec4 n = vec4(newPositions[i], 1);
		//vec4 newpos = n * transformation_matrix;
		/*vec3 newpos = vr_mesh_view::local_to_global(newPositions[i]);
		v->position = vec3(newpos.x(), newpos.y(), newpos.z());
		M.position(v->originalIndex) = v->position;*/


		v->position = newPositions[i];
		//vec4 n = vec4(v->position, 1);
		//vec4 newpos = n * transformation_matrix;
		vec3 newpos = vr_mesh_view::local_to_global(newPositions[i]);
		M.position(v->originalIndex) = vec3(newpos.x(), newpos.y(), newpos.z());
		++i;

	}
	M.compute_vertex_normals();
	B = M.compute_box();
	have_new_mesh = true;
	post_redraw();
	//transformation_matrix.identity();	
	build_aabbtree_from_triangles(he, aabb_tree);
}

void vr_mesh_view::applySmoothingPoints() {
	std::vector<vec3> newPositions;
	for (HE_Vertex* v : smoothingpoints) {
		int number = 0;
		vec3 newpos = vec3(0, 0, 0);
		for (HE_Vertex* neighbor : he->GetNeighborVertices(v)) {
			number++;
			newpos += neighbor->position;
		}
		newpos /= number;
		newPositions.push_back(newpos);
	}
	int i = 0;
	for (HE_Vertex* v : smoothingpoints) {
		v->position = newPositions[i];
		//vec4 n = vec4(v->position, 1);
		//vec4 newpos = n * transformation_matrix;
		vec3 newpos = vr_mesh_view::local_to_global(newPositions[i]);
		M.position(v->originalIndex) = vec3(newpos.x(), newpos.y(), newpos.z());
		++i;

	}
	smoothingpoints.clear();
	M.compute_vertex_normals();
	B = M.compute_box();
	have_new_mesh = true;
	post_redraw();
	build_aabbtree_from_triangles(he, aabb_tree);
}

void vr_mesh_view::tessellation(const vec3& origin, const vec3& direction) {
	//global to local
	vec3 new_origin = global_to_local(origin);
	vec3 point_on_ray = origin + direction;
	vec3 new_point_on_ray = global_to_local(point_on_ray);
	vec3 new_dir = new_point_on_ray - new_origin;

	// create ray
	ray_intersection::ray tes_ray = ray_intersection::ray(new_origin, new_dir);
	float t = 0.0;

	if (ray_intersection::rayTreeIntersect(tes_ray, aabb_tree, t)) {
		vr_mesh_view::nr_tes_intersection++;
		vec3 tes_inter_point = ray_intersection::getIntersectionPoint(tes_ray, t);
		HE_Face* tes_face = ray_intersection::getIntersectedFace(tes_ray, he);
		//vec3 p1, p2, p3;
		//mesh_utils::getVerticesOfFace(he, tes_face, p1, p2, p3);

		auto tes_point = he->GetVerticesForFace(tes_face); //three vertices in the tes_face
		// add three faces to the original half edge mesh
		for (int i = 0; i < 3; i++) {
			unsigned int vectorAIndex = tes_point[i]->originalIndex;
			unsigned int vectorBIndex = tes_point[(i + 1) % 3]->originalIndex;
			unsigned int vectorCIndex = M.get_nr_positions() + nr_tes_intersection;

			// adding the 3 vectors
			auto vectorA = he->AddVector(vectorAIndex, tes_point[i]->position);
			auto vectorB = he->AddVector(vectorBIndex, tes_point[(i + 1) % 3]->position);
			auto vectorC = he->AddVector(vectorCIndex, tes_inter_point);

			auto face = he->AddFace();

			// generating 3 half edges per triangle
			auto halfEdgeC = he->AddHalfEdge(vectorC, vectorA, face);
			auto halfEdgeB = he->AddHalfEdge(vectorB, vectorC, face, halfEdgeC);
			auto halfEdgeA = he->AddHalfEdge(vectorA, vectorB, face, halfEdgeB);

			// closing the loop
			halfEdgeC->next = halfEdgeA;
		}
		//create a new normal 
		idx_type normal_idx = M.new_normal(vec3(0.0f, -1.0f, 0.0f));
		//create a global point vector
		vec4 trans_point = vec4(0);
		//transform the intersected point from local to global system
		//trans_point = transformation_matrix * vec4(tes_inter_point, 1.0);
		//create a new position in the simple mesh M
		//idx_type tes_pos_idx = M.new_position(vec3(trans_point[0], trans_point[1], trans_point[2]));
		idx_type tes_pos_idx = M.new_position(vr_mesh_view::local_to_global(tes_inter_point));
		// add new faces to simple mesh
		for (int i = 0; i < 3; i++) {
			//create a new face
			M.start_face();
			// tell the mesh to save a new corner (vertex) with the position and normal given as indices
			M.new_corner(tes_pos_idx, normal_idx);

			idx_type pos_idx;
			pos_idx = tes_point[i]->originalIndex;
			//create the second corner for each face
			M.new_corner(pos_idx, normal_idx);

			// create the last corner
			pos_idx = tes_point[(i + 1) % 3]->originalIndex;
			M.new_corner(pos_idx, normal_idx);

		}
		//output some useful information
		/*
		std::cout << tes_inter_point << std::endl;

		std::cout << "normal :" << M.normal(tes_point[0]->originalIndex) << std::endl;
		std::cout << "normal :" << M.normal(tes_point[1]->originalIndex) << std::endl;
		std::cout << "normal :" << M.normal(tes_point[2]->originalIndex) << std::endl;
		std::cout << "nr_face " << M.get_nr_faces() << std::endl;
		std::cout << "nr_normal " << M.get_nr_normals() << std::endl;
		std::cout << "nr_position " << M.get_nr_positions() << std::endl;
		std::cout << "begin_corner 0 " << M.begin_corner(0) << std::endl;
		std::cout << "end_corner 0 " << M.end_corner(0) << std::endl;
		std::cout << "begin_corner 11 " << M.begin_corner(11) << std::endl;
		std::cout << "end_corner 11" << M.end_corner(11) << std::endl;
		*/
		
		//compute the normals again
		M.compute_vertex_normals();
		B = M.compute_box();
		have_new_mesh = true;
		post_redraw();
		build_aabbtree_from_triangles(he, aabb_tree);
	}
	else {
		std::cout << "No intersection" << std::endl;
	}
}

void vr_mesh_view::vertex_manipulate(HE_Vertex* vertex, vec3 pos, vec3 last_pos) {

	if (he->changeVertexPos(vertex, vertex->position + pos - last_pos )) {
		M.position(vertex->originalIndex) = M.position(vertex->originalIndex) + local_to_global(pos) - local_to_global(last_pos);
		//std::cout << "Vertex is manipulated." << std::endl;
		post_redraw();
	}
	else
		std::cout << "Vertex position couldn't be manipulated." << std::endl;
}


void vr_mesh_view::drawpath(cgv::render::context& ctx,std::vector<vec3> path_list) {	
	auto& prog = ctx.ref_default_shader_program();
	int ci = prog.get_color_index();
	vec3 a = (1, 0, 0);
	color_list.push_back(a);
	cgv::render::attribute_array_binding::set_global_attribute_array(ctx, prog.get_position_index(), path_list);
	cgv::render::attribute_array_binding::enable_global_array(ctx, prog.get_position_index());
	cgv::render::attribute_array_binding::set_global_attribute_array(ctx, ci, color_list);
	cgv::render::attribute_array_binding::enable_global_array(ctx, ci);

	glLineWidth(3);
	prog.enable(ctx);
	glDrawArrays(GL_LINES, 0, (GLsizei)path_list.size());
	prog.disable(ctx);
	cgv::render::attribute_array_binding::disable_global_array(ctx, prog.get_position_index());
	cgv::render::attribute_array_binding::disable_global_array(ctx, ci);

}

void vr_mesh_view::show_selected_smoothing_faces(HE_Face* f) {
	std::cout << "show soothing faces" << std::endl;
	// add new faces to simple mesh#
	std::vector<HE_Vertex*> Fvertices = he->GetVerticesForFace(f);
	//create a new face
	smoothingMesh.start_face();
	for (int j = 0; j < Fvertices.size(); j++) {
		//vec4 point = transformation_matrix * vec4(Fvertices[j]->position, 1.0);
		vec3 point = vr_mesh_view::local_to_global(Fvertices[j]->position);
		idx_type pos_idx = smoothingMesh.new_position(vec3(point.x(), point.y(),point.z()));
		idx_type normal_idx = smoothingMesh.new_normal(vec3(0.0f, -1.0f, 0.0f));
		// tell the mesh to save a new corner (vertex) with the position and normal given as indices
		smoothingMesh.new_corner(pos_idx, normal_idx);
	}
	//compute the normals again
	smoothingMesh.compute_vertex_normals();
	B_smoothing = smoothingMesh.compute_box();
	have_new_smoothingMesh = true;
	post_redraw();
}
#include <cgv/base/register.h>

cgv::base::object_registration<vr_mesh_view> vr_test_reg("vr_mesh_view");


