#include <cgv/base/node.h>
#include <cgv/defines/quote.h>
#include <cgv/render/shader_program.h>
#include <cgv_reflect_types/media/color.h>
#include <cgv/render/drawable.h>
#include <cgv/render/clipped_view.h>
#include <cgv_gl/gl/gl.h>
#include <cgv_gl/sphere_renderer.h>
#include <cgv_gl/rounded_cone_renderer.h>
#include <cgv_gl/gl/mesh_render_info.h>
#include <cgv_gl/gl/gl_context.h>
#include <cgv/render/attribute_array_binding.h>
#include <cgv/gui/provider.h>
#include <cgv/gui/event_handler.h>
#include <cgv/media/color_scale.h>
#include <cgv/media/mesh/simple_mesh.h>
#include <cgv/gui/key_event.h>
#include <cgv/gui/mouse_event.h>
#include <cgv/gui/provider.h>
#include <cgv/gui/file_dialog.h>
#include <cgv/math/ftransform.h>
#include <cgv/math/geom.h>
#include <cgv/utils/scan.h>
#include <fstream>
#include <cgv_gl/gl/gltf_support.h>
#include "halfedgemesh.h"
#include "mesh_utils.h"
#include "aabb_tree.h"
#include "ray_intersection.h"
#include <cgv_gl/box_wire_renderer.h>

using namespace cgv::base;
using namespace cgv::signal;
using namespace cgv::gui;
using namespace cgv::math;
using namespace cgv::render;
using namespace cgv::utils;
using namespace cgv::media::illum;

class mesh_view : public node, public drawable, public event_handler, public provider
{
private:
	bool have_new_mesh;
public:
	typedef cgv::media::mesh::simple_mesh<float> mesh_type;
	typedef mesh_type::idx_type idx_type;
	typedef mesh_type::vec3i vec3i;

	bool in_picking;
	bool click_is_pick;
	double click_press_time;
	int click_button;
	int pick_point_index;

	std::vector<vec3> pick_points;
	std::vector<rgb> pick_colors;

	std::vector<vec3> sphere_positions;
	std::vector<rgba> sphere_colors;
	std::vector<float> sphere_radii;

	std::vector<vec4> planes;
	std::vector<rgba> plane_colors;

	cgv::render::view* view_ptr;
	bool show_vertices;
	cgv::render::sphere_render_style sphere_style;
	cgv::render::sphere_render_style sphere_hidden_style;

	bool show_wireframe;
	cgv::render::rounded_cone_render_style cone_style;

	bool show_surface;
	CullingMode cull_mode;
	ColorMapping color_mapping;
	rgb  surface_color;
	IlluminationMode illumination_mode;

	//bounding box related member
	bool show_bounding_box; 
	AabbTree<triangle> aabb_tree;
	std::vector<box3> boxes;

	std::string scene_file_name;
	std::string file_name;

	mesh_type M;
	box3 B;
	bool scene_box_outofdate;
	GLint max_clip_distances;
	int fst_clip_plane;
	int nr_clip_planes;

	vec3 translate_vector;
	vec3 rotation_angles;
	float scale_factor;

	cgv::render::render_info r_info;
	cgv::render::mesh_render_info mesh_info;
	cgv::render::shader_program mesh_prog;
	// mesh generation parameters
	int n, m;
	float a, b;
	float lb, ub;

	void apply_translation()
	{
		mat3 I;
		I.identity();
		M.transform(I, translate_vector);
		have_new_mesh = true;
		B = M.compute_box();
		scene_box_outofdate = true;
		post_redraw();
	}
	void apply_reflection()
	{
		mat3 I;
		I.identity();
		I = -I;
		M.transform(I, vec3(0.0f));
		M.revert_face_orientation();
		have_new_mesh = true;
		B = M.compute_box();
		scene_box_outofdate = true;
		post_redraw();
	}
	void revert_face_orientation()
	{
		M.revert_face_orientation();
		have_new_mesh = true;
		post_redraw();
	}
	void apply_rotation()
	{
		mat3 R = rotate3<float>(rotation_angles);
		M.transform(R,vec3(0.0f));
		have_new_mesh = true;
		B = M.compute_box();
		scene_box_outofdate = true;
		post_redraw();
	}
	void apply_scaling()
	{
		mat3 I;
		I.identity();
		I *= scale_factor;
		M.transform(I, vec3(0.0f));
		have_new_mesh = true;
		B = M.compute_box();
		scene_box_outofdate = true;
		post_redraw();
	}
public:
	mesh_view() : node("mesh_view")
	{
		click_is_pick = false;
		in_picking = false;
		click_press_time = 0;
		click_button = 0;
		pick_point_index = -1;
		view_ptr = 0;

		show_surface = true;
		cull_mode = CM_BACKFACE;
		color_mapping = cgv::render::CM_COLOR;
		surface_color = rgb(0.7f, 0.2f, 1.0f);
		illumination_mode = IM_ONE_SIDED;

		sphere_style.map_color_to_material = CM_COLOR_AND_OPACITY;
		sphere_style.surface_color = rgb(0.8f, 0.4f, 0.4f);
		sphere_hidden_style.percentual_halo_width = -50;
		sphere_hidden_style.halo_color_strength = 1.0f;
		sphere_hidden_style.halo_color = rgba(0.5f, 0.5f, 0.5f, 0.5f);
		show_vertices = true;

		show_wireframe = true;
		cone_style.surface_color = rgb(1.0f, 0.8f, 0.4f);

		show_bounding_box = false;

		have_new_mesh = false;
		scene_box_outofdate = false;

		fst_clip_plane = 0;
		nr_clip_planes = 0;

		translate_vector = vec3(0.0f);
		rotation_angles = vec3(0.0f);
		scale_factor = 1.0f;

		a = 1;
		b = 0.2f;
		lb = 0.01f;
		ub = 2.0f;
		n = m = 20;
	}
	std::string get_type_name() const
	{
		return "mesh_view";
	}
	void generate_dini_surface()
	{
		M.clear();
		// allocate per Vector colors of type rgb with float components
		M.ensure_colors(cgv::media::CT_RGB, (n + 1)*m);

		for (int i = 0; i <= n; ++i) {
			float y = (float)i / n;
			float v = (ub - lb)*y + lb;
			for (int j = 0; j < m; ++j) {
				float x = (float)j / m;
				float u = float(4.0f*M_PI)*x;
				// add new position to the mesh (function returns position index, which is i*m+j in our case)
				int vi = M.new_position(vec3(a*cos(u)*sin(v), a*sin(u)*sin(v), a*(cos(v) + log(tan(0.5f*v))) + b * u));
				// set color
				M.set_color(vi, rgb(x, y, 0.5f));
				// add quad connecting current Vector with previous ones
				if (i > 0) {
					int vi = ((i - 1) * m + j);
					int delta_j = -1;
					if (j == 0)
						delta_j = m - 1;
					M.start_face();
					M.new_corner(vi);
					M.new_corner(vi + m);
					M.new_corner(vi + m + delta_j);
					M.new_corner(vi + delta_j);
				}
			}
		}
		// compute surface normals at mesh vertices from quads
		M.compute_vertex_normals();

		have_new_mesh = true;
		post_redraw();
	}
	bool self_reflect(cgv::reflect::reflection_handler& rh)
	{
		return
			rh.reflect_member("show_bounding_box", show_bounding_box) &&
			rh.reflect_member("show_surface", show_surface) &&
			rh.reflect_member("cull_mode", (int&)cull_mode) &&
			rh.reflect_member("color_mapping", (int&)color_mapping) &&
			rh.reflect_member("surface_color", surface_color) &&
			rh.reflect_member("illumination_mode", (int&)illumination_mode) &&
			rh.reflect_member("show_vertices", show_vertices) &&
			rh.reflect_member("sphere_style", sphere_style) &&
			rh.reflect_member("sphere_hidden_style", sphere_hidden_style) &&
			rh.reflect_member("show_wireframe", show_wireframe) &&
			rh.reflect_member("cone_style", cone_style) &&
			rh.reflect_member("scene_file_name", scene_file_name) &&
			rh.reflect_member("file_name", file_name);
	}
	bool read_mesh(const std::string& file_name)
	{
		mesh_type tmp;
		size_t Vector_count = 0;
		if (cgv::utils::to_lower(cgv::utils::file::get_extension(file_name)) == "gltf") {
			fx::gltf::Document doc = fx::gltf::LoadFromText(file_name);
			if (get_context()) {
				cgv::render::context& ctx = *get_context();
				build_render_info(file_name, doc, ctx, r_info);
				r_info.bind(ctx, ctx.ref_surface_shader_program(true), true);
				extract_additional_information(doc, B, Vector_count);
			}
			else
				extract_mesh(file_name, doc, tmp);
		}
		else {
			if (!tmp.read(file_name))
				return false;
		}
		if (tmp.get_nr_positions() > 0) {
			M = tmp;
			B = M.compute_box();
			Vector_count = M.get_nr_positions();
			//create HE_MESH and build bounding box
			HE_Mesh* he = generate_from_simple_mesh(M);
			build_aabbtree_from_triangles(he, aabb_tree);
		}
		sphere_style.radius = float(0.05*sqrt(B.get_extent().sqr_length() / Vector_count));
		on_set(&sphere_style.radius);
		sphere_hidden_style.radius = sphere_style.radius;
		on_set(&sphere_hidden_style.radius);

		cone_style.radius = 0.5f*sphere_style.radius;
		if (cgv::utils::file::get_file_name(file_name) == "Max-Planck_lowres.obj")
			construct_mesh_colors();
		return true;
	}
	void on_set(void* member_ptr)
	{
		if (member_ptr == &file_name) {
			if (ref_tree_node_visible_flag(file_name)) {
				M.write(file_name);
			}
			else {
				if (read_mesh(file_name))
					have_new_mesh = true;
			}
		}
		if (member_ptr == &scene_file_name) {
			if (ref_tree_node_visible_flag(scene_file_name))
				write_scene(scene_file_name);
			else
				read_scene(scene_file_name);
		}
		update_member(member_ptr);
		post_redraw();
	}
	// a hack that adds Vector colors to a mesh and used for illustration purposes only
	void construct_mesh_colors()
	{
		if (M.has_colors())
			return;
		M.ensure_colors(cgv::media::CT_RGB, M.get_nr_positions());
		double int_part;
		for (unsigned i = 0; i < M.get_nr_positions(); ++i)
			M.set_color(i, cgv::media::color_scale(modf(20 * double(i) / (M.get_nr_positions() - 1), &int_part)));
	}
	bool on_pick(const mouse_event& me)
	{
		if (!view_ptr)
			return false;
		dvec3 pick_point;
		double window_z;
		if (!get_world_location(me.get_x(), me.get_y(), *view_ptr, pick_point, &window_z) ||
			window_z > 0.999)
			return false;
		// check whether to pick a previously defined point
		pick_point_index = -1;
		double pick_dist = 0;
		double pick_dist_threshold = sphere_style.radius*sphere_style.radius_scale;
		for (int i = 0; i < (int)pick_points.size(); ++i) {
			double dist = (pick_points[i] - vec3(pick_point)).length();
			if (dist < pick_dist_threshold) {
				if (pick_point_index == -1 || dist < pick_dist) {
					pick_dist = dist;
					pick_point_index = i;
				}
			}
		}
		if (pick_point_index == -1) {
			pick_point_index = pick_points.size();
			pick_points.push_back(pick_point);
			pick_colors.push_back(rgb(0, 1, 0));
			if (ref_tree_node_visible_flag(pick_points))
				post_recreate_gui();
			return true;
		}
		return false;
	}
	bool on_drag(const mouse_event& me)
	{
		if (!view_ptr)
			return false;
		if (pick_point_index == -1)
			return false;
		dvec3 pick_point;
		double window_z;
		if (!get_world_location(me.get_x(), me.get_y(), *view_ptr, pick_point, &window_z) ||
			window_z > 0.999) {
			pick_colors[pick_point_index] = rgb(0, 1, 0);
			if (ref_tree_node_visible_flag(pick_points))
				update_member(&pick_colors[pick_point_index]);

			pick_point_index = -1;
			post_redraw();
			return false;
		}
		pick_points[pick_point_index] = pick_point;
		if (ref_tree_node_visible_flag(pick_points)) {
			update_member(&pick_points[pick_point_index][0]);
			update_member(&pick_points[pick_point_index][1]);
			update_member(&pick_points[pick_point_index][2]);
		}
		post_redraw();
		return true;
	}
	void on_click(const mouse_event& me)
	{
		if (pick_point_index == -1)
			return;
		if (!view_ptr)
			return;
		dvec3 pick_point;
		if (!get_world_location(me.get_x(), me.get_y(), *view_ptr, pick_point))
			return;
		pick_points.erase(pick_points.begin() + pick_point_index);
		pick_colors.erase(pick_colors.begin() + pick_point_index);
		pick_point_index = -1;
		if (ref_tree_node_visible_flag(pick_points))
			post_recreate_gui();
		post_redraw();
	}
	bool read_scene(const std::string& scene_file_name) 
	{
		std::ifstream is(scene_file_name);
		if (is.fail())
			return false;

		sphere_positions.clear();
		sphere_colors.clear();
		sphere_radii.clear();
		planes.clear();
		plane_colors.clear();
		pick_points.clear();
		pick_colors.clear();
		pick_point_index = -1;
		post_recreate_gui();
		post_redraw();

		vec4 plane;
		vec3 pos;
		rgb  clr;
		rgba color;
		float radius;
		while (!is.eof()) {
			char buffer[2048];
			is.getline(&buffer[0], 2048);
			std::string line(buffer);
			if (line.empty())
				continue;
			std::stringstream ss(line, std::ios_base::in);
			char c;
			ss.get(c);
			switch (toupper(c)) {
			case 'M':
				 file_name = line.substr(3, line.size() - 4);
				 on_set(&file_name);
				 break;
			case 'S':
				ss >> pos >> radius >> color;
				if (!ss.fail()) {
					sphere_positions.push_back(pos);
					sphere_colors.push_back(color);
					sphere_radii.push_back(radius);
				}
				break;
			case 'P':
				ss >> plane >> color;
				if (!ss.fail()) {
					planes.push_back(plane);
					plane_colors.push_back(color);
				}
				break;
			case 'I':
				ss >> pos >> clr;
				if (!ss.fail()) {
					pick_points.push_back(pos);
					pick_colors.push_back(clr);
				}
				break;
			}
		}
		return !is.fail();
	}
	bool write_scene(const std::string& scene_file_name) const
	{
		std::ofstream os(scene_file_name);
		if (os.fail())
			return false;
		unsigned i;
		os << "m \"" << file_name << "\"" << std::endl;
		for (i = 0; i < sphere_positions.size(); ++i)
			os << "s " << sphere_positions[i] << " " << sphere_radii[i] << " " << sphere_colors[i] << std::endl;
		for (i = 0; i < planes.size(); ++i)
			os << "p " << planes[i] << " " << plane_colors[i] << std::endl;
		for (i = 0; i < pick_points.size(); ++i)
			os << "i " << pick_points[i] << " " << pick_colors[i] << std::endl;
		return true;
	}
	bool handle(event& e)
	{
		if (e.get_kind() == EID_KEY) {
			auto& ke = static_cast<key_event&>(e);
			if (ke.get_action() != KA_RELEASE) {
				switch (ke.get_key()) {
				case 'V': show_vertices = !show_vertices;  on_set(&show_vertices);  return true;
				case 'W': show_wireframe= !show_wireframe; on_set(&show_wireframe); return true;
				case 'F': show_surface = !show_surface;   on_set(&show_surface);   return true;
				case 'S' : add_sphere(ke.get_modifiers() == EM_SHIFT); return true;
				case 'O': add_oriented_plane(ke.get_modifiers() == EM_SHIFT); return true;
				case 'C': add_center(ke.get_modifiers() == EM_SHIFT); return true;
				case KEY_Back_Space : 
					if (ke.get_modifiers() == EM_SHIFT) {
						if (sphere_positions.empty())
							break;
						sphere_positions.pop_back();
						sphere_colors.pop_back();
						sphere_radii.pop_back();
						post_recreate_gui();
						post_redraw();
						return true;
					}
					else if (ke.get_modifiers() == EM_ALT) {
						if (planes.empty())
							break;
						planes.pop_back();
						plane_colors.pop_back();
						post_recreate_gui();
						post_redraw();
						return true;
					}
					else {
						if (!pick_points.empty()) {
							pick_points.pop_back();
							pick_colors.pop_back();
							pick_point_index = -1;
							if (ref_tree_node_visible_flag(pick_points))
								post_recreate_gui();
							post_recreate_gui();
							on_set(&pick_point_index);
							return true;
						}
					}
					break;
				}
			}
			return false;
		}
		if (e.get_kind() == EID_MOUSE) {
			auto& me = static_cast<mouse_event&>(e);
			
			switch (me.get_action()) {
			case MA_PRESS: 
				if (me.get_button() == MB_LEFT_BUTTON && me.get_modifiers() == EM_CTRL) {
					in_picking = true;
					click_is_pick = true;
					click_press_time = me.get_time();
					click_button = me.get_button();
					if (on_pick(me))
						click_is_pick = false;
					if (pick_point_index != -1) {
						pick_colors[pick_point_index] = rgb(1, 0, 0);
						if (ref_tree_node_visible_flag(pick_points))
							update_member(&pick_colors[pick_point_index]);
					}
					post_redraw();
					return true;
				}
				click_is_pick = false;
				break;
			case MA_RELEASE :
				if (me.get_button() == MB_LEFT_BUTTON && me.get_modifiers() == EM_CTRL) {
					if (click_is_pick) {
						click_is_pick = false;
						if (me.get_time() - click_press_time < 0.1)
							on_click(me);
					}
					else if (in_picking && pick_point_index != -1) {
						pick_colors[pick_point_index] = rgb(0, 1, 0);
						if (ref_tree_node_visible_flag(pick_points))
							update_member(&pick_colors[pick_point_index]);
						pick_point_index = -1;
						post_redraw();
					}
					in_picking = false;
					return true;
				}
				break;
			case MA_DRAG :
				if (in_picking) {
					on_drag(me);
					click_is_pick = false;
					return true;
				}
				break;
			}
			return false;
		}
		return false;
	}
	void stream_help(std::ostream& os)
	{
		os << "mesh_view: help is a secret" << std::endl;
	}
	vec3 get_center()
	{
		vec3 center(0);
		for (const auto& p : pick_points)
			center += p;
		center *= 1.0f / pick_points.size();
		return center;
	}
	void add_oriented_plane(bool collapse)
	{
		if (pick_points.size() < 3)
			return;
		vec3 normal(0);
		vec3 center = get_center();
		vec3 last_diff = pick_points.back() - center;
		for (const auto& p : pick_points) {
			vec3 diff = p - center;
			normal += cross(last_diff, diff);
			last_diff = diff;
		}
		normal.normalize();
		planes.push_back(vec4(normal, -dot(normal, center)));
		plane_colors.push_back(rgba(1, 1, 0, 0.75f));
		std::cout << "added plane: " << planes.back() << std::endl;
		post_recreate_gui();
		post_redraw();
		scene_box_outofdate = true;
	}
	void add_sphere(bool collapse)
	{
		if (pick_points.size() < 2)
			return;
		const vec3& center = pick_points[pick_points.size() - 2];
		const vec3& p = pick_points[pick_points.size() - 1];
		sphere_positions.push_back(center);
		sphere_radii.push_back((center-p).length());
		sphere_colors.push_back(rgba(0, 1, 1, 0.7f));
		if (collapse) {
			pick_points.pop_back();
			pick_points.pop_back();
			pick_colors.pop_back();
			pick_colors.pop_back();
			pick_point_index = -1;
		}
		post_recreate_gui();
		post_redraw();
		scene_box_outofdate = true;
	}
	void add_center(bool collapse)
	{
		if (pick_points.size() < 2)
			return;

		vec3 center = get_center();
		if (collapse) {
			pick_points.clear();
			pick_colors.clear();
		}
		pick_points.push_back(center);
		pick_colors.push_back(rgb(1,0,1));
		pick_point_index = -1;
		post_redraw();
	}
	void create_gui()
	{
		add_decorator("mesh", "heading", "level=2");
		add_gui("scene_file_name", scene_file_name, "file_name",
				"open=true;open_title='open scene file';filter='scene (scn):*.scn|all files:*.*';"
				"save=true;save_title='save scene file';w=140");
		add_gui("file_name", file_name, "file_name",
			"open=true;title='open obj file';filter='mesh (obj):*.obj|all files:*.*';"
			"save=true;save_title='save obj file';w=140");

		bool show = begin_tree_node("generate", a, false, "options='w=140';align=' '");
		connect_copy(add_button("generate", "w=52")->click, cgv::signal::rebind(this, &mesh_view::generate_dini_surface));
		if (show) {
			align("\a");
			add_member_control(this, "a", a, "value_slider", "min=0.1;max=10;ticks=true;log=true");
			add_member_control(this, "b", b, "value_slider", "min=0.1;max=10;ticks=true;log=true");
			add_member_control(this, "lb", lb, "value_slider", "min=0.001;step=0.0001;max=1;ticks=true;log=true");
			add_member_control(this, "ub", ub, "value_slider", "min=1;max=10;ticks=true;log=true");
			add_member_control(this, "n", n, "value_slider", "min=5;max=100;ticks=true;log=true");
			add_member_control(this, "m", m, "value_slider", "min=5;max=100;ticks=true;log=true");
			align("\b");
			end_tree_node(a);
		}
		if (begin_tree_node("debug", translate_vector, true)) {
			align("\a");
			add_decorator("mesh data structure", "heading", "level=3");
			connect_copy(add_button("generate mesh")->click, rebind(this, &mesh_view::debug_mesh_generation));
			align("\b");
		}
		if (begin_tree_node("transform", translate_vector, true)) {
			align("\a");
			add_decorator("translation", "heading", "level=3");
			add_gui("translate_vector", translate_vector, "", "options='min=-3;max=3;step=0.0001;ticks=true'");
			connect_copy(add_button("apply translation")->click, rebind(this, &mesh_view::apply_translation));
			add_decorator("rotation", "heading", "level=3");
			add_gui("rotation_angles", rotation_angles, "", "options='min=-180;max=180;step=0.1;ticks=true'");
			connect_copy(add_button("apply rotation")->click, rebind(this, &mesh_view::apply_rotation));
			add_decorator("scaling", "heading", "level=3");
			add_member_control(this, "scale_factor", scale_factor, "value_slider", "min=0.001;step=0.0001;max=1000;ticks=true;log=true");
			connect_copy(add_button("apply scaling")->click, rebind(this, &mesh_view::apply_scaling));
			add_decorator("reflection", "heading", "level=3");
			connect_copy(add_button("apply reflection")->click, rebind(this, &mesh_view::apply_reflection));
			connect_copy(add_button("revert face orientation")->click, rebind(this, &mesh_view::revert_face_orientation));
			align("\b");
			end_tree_node(translate_vector);
		}
		if (begin_tree_node("construct", pick_point_index, true)) {
			align("\a");
			unsigned i;
			connect_copy(add_button("add center", "w=96", " ")->click, rebind(this, &mesh_view::add_center, _c<bool>(false)));
			connect_copy(add_button("and collapse", "w=96")->click, rebind(this, &mesh_view::add_center, _c<bool>(true)));
			if (begin_tree_node("pick_points", pick_points)) {
				align("\a");
				for (i = 0; i < pick_points.size(); ++i) {
					if (begin_tree_node(std::string("pick_point_") + to_string(i), pick_points[i])) {
						align("\a");
						add_gui("position", pick_points[i], "", "options='min=-3;max=3;ticks=true'");
						add_member_control(this, "color", pick_colors[i]);
						align("\b");
						end_tree_node(pick_points[i]);
					}
					end_tree_node(pick_points);
				}
				align("\b");
				end_tree_node(pick_points);
			}
			connect_copy(add_button("add sphere", "w=96", " ")->click, rebind(this, &mesh_view::add_sphere, _c<bool>(false)));
			connect_copy(add_button("and collapse", "w=96")->click, rebind(this, &mesh_view::add_sphere, _c<bool>(true)));
			if (begin_tree_node("spheres", sphere_positions)) {
				align("\a");
				for (i = 0; i < sphere_positions.size(); ++i) {
					if (begin_tree_node(std::string("sphere_") + to_string(i), sphere_positions[i])) {
						align("\a");
						add_gui("center", sphere_positions[i], "", "options='min=-3;max=3;ticks=true'");
						add_member_control(this, "radius", sphere_radii[i], "value_slider", "min=0.00001;step=0.000001;max=10;ticks=true;log=true");
						add_member_control(this, "color", sphere_colors[i]);
						align("\b");
						end_tree_node(sphere_positions[i]);
					}
				}
				align("\b");
				end_tree_node(sphere_positions);
			}
			connect_copy(add_button("add plane", "w=96", " ")->click, rebind(this, &mesh_view::add_oriented_plane, _c<bool>(false)));
			connect_copy(add_button("and collapse", "w=96")->click, rebind(this, &mesh_view::add_oriented_plane, _c<bool>(true)));
			if (begin_tree_node("planes", planes)) {
				align("\a");
				add_member_control(this, "fst_clip_plane", fst_clip_plane, "value_slider", "min=0;ticks=true;max=0");
				add_member_control(this, "nr_clip_planes", nr_clip_planes, "value_slider", std::string("min=0;ticks=true;max=") + to_string(max_clip_distances - 1));
				for (i = 0; i < planes.size(); ++i) {
					if (begin_tree_node(std::string("plane_") + to_string(i), planes[i])) {
						align("\a");
						add_gui("normal", reinterpret_cast<vec3&>(planes[i]), "direction", "options='min=-1;max=1;ticks=true'");
						add_member_control(this, "d", planes[i][3], "value_slider", "min=-1;step=0.0000001;max=1;ticks=true");
						add_member_control(this, "color", plane_colors[i]);
						end_tree_node(planes[i]);
						align("\b");
					}
				}
				align("\b");
				end_tree_node(planes);
			}
			align("\b");
			end_tree_node(pick_point_index);
		}

		show = begin_tree_node("vertices", show_vertices, false, "options='w=100';align=' '");
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
			if (!mesh_info.ref_materials().empty())
				materials = &mesh_info.ref_materials();
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
		//add bounding_box button
		show = begin_tree_node("bounding_box", show_bounding_box, false, "options='w=100';align=' '");
		add_member_control(this, "show", show_bounding_box, "toggle", "w=42;shortcut='w'", " ");

	}
	bool init(context& ctx)
	{
		glGetIntegerv(GL_MAX_CLIP_DISTANCES, &max_clip_distances);
		std::cout << "max clip distances = " << max_clip_distances << std::endl;
		if (!view_ptr)
			view_ptr = find_view_as_node();
		if (!mesh_prog.build_program(ctx, "mesh.glpr", true))
			abort();
		ref_sphere_renderer(ctx, 1);
		ref_rounded_cone_renderer(ctx, 1);

		//ctx.set_bg_color(1.0f, 1.0f, 1.0f, 1.0f);
		return true;
	}
	void destruct(context& ctx)
	{
		mesh_prog.destruct(ctx);
		ref_sphere_renderer(ctx, -1);
		ref_rounded_cone_renderer(ctx, -1);
	}
	void compute_plane_points(const vec4& pln, std::vector<vec3>& P, std::vector<vec3>* N_ptr=0, std::vector<rgba>* C_ptr=0)
	{
		const vec3& nml = reinterpret_cast<const vec3&>(pln);
		vec3 tmp = nml;
		tmp[(fabs(tmp[0]) < fabs(tmp[1])) ? 0 : 1] += 3;
		mat3 F = build_orthogonal_frame(nml, tmp);
		const vec3& x = F.col(1);
		const vec3& y = F.col(2);
		box3 b; b.invalidate();
		int i;
		for (i = 0; i < 8; ++i)
			b.add_point(B.get_corner(i) * F);
		for (i = 0; i < 8; i += 2) {
			vec3 p = F * b.get_corner(i);
			float d = dot(pln, vec4(p, 1));
			p -= d * nml;
			P.push_back(p);
			if (N_ptr)
				N_ptr->push_back(nml);
			if (C_ptr)
				C_ptr->push_back(plane_colors[&pln - &planes[0]]);
		}

	}
	void init_frame(context& ctx)
	{
		if (have_new_mesh) {
			if (!M.get_positions().empty()) {
				// auto-compute mesh normals if not available
				if (!M.has_normals())
					M.compute_vertex_normals();
				// [re-]compute mesh render info
				mesh_info.destruct(ctx);

				mesh_info.construct(ctx, M);
				// bind mesh attributes to standard surface shader program
				mesh_info.bind(ctx, mesh_prog, true);
				mesh_info.bind_wireframe(ctx, ref_rounded_cone_renderer(ctx).ref_prog(), true);
			}
			// ensure that materials are presented in gui
			post_recreate_gui();
			scene_box_outofdate = true;
		}
		if (scene_box_outofdate) {
			// focus view on new mesh
			clipped_view* view_ptr = dynamic_cast<clipped_view*>(find_view_as_node());
			if (view_ptr) {
				box3 box = B;
				unsigned i;
				for (i = 0; i < sphere_positions.size(); ++i) {
					box.add_axis_aligned_box(box3(
						sphere_positions[i] - vec3(sphere_radii[i]),
						sphere_positions[i] + vec3(sphere_radii[i])));

				}
				for (i = 0; i < planes.size(); ++i) {
					std::vector<vec3> plane_points;
					compute_plane_points(planes[i], plane_points);
					for (const vec3& p : plane_points)
						box.add_point(p);
				}
				view_ptr->set_scene_extent(box);
				if (have_new_mesh) {
					view_ptr->set_focus(box.get_center());
					view_ptr->set_y_extent_at_focus(box.get_extent().length());
				}
				scene_box_outofdate = false;
			}
		}
		have_new_mesh = false;
	}
	void draw_surface(context& ctx, bool opaque_part)
	{
		// remember current culling setting
		GLboolean is_culling = glIsEnabled(GL_CULL_FACE);
		GLint cull_face;
		glGetIntegerv(GL_CULL_FACE_MODE, &cull_face);

		// ensure that opengl culling is identical to shader program based culling
		if (cull_mode > 0) {
			glEnable(GL_CULL_FACE);
			glCullFace(cull_mode == CM_BACKFACE ? GL_BACK : GL_FRONT);
		}
		else
			glDisable(GL_CULL_FACE);
		
		// choose a shader program and configure it based on current settings
		shader_program& prog = mesh_prog;
		// enable clip planes
		int nr_clip_distances = std::min(nr_clip_planes, (int)(planes.size() - fst_clip_plane));
		prog.set_uniform(ctx, "nr_clip_planes", nr_clip_distances);
		if (nr_clip_distances > 0)
			prog.set_uniform_array(ctx, "clip_planes", &planes[fst_clip_plane], nr_clip_distances);
		int i;
		for (i = 0; i < nr_clip_distances; ++i)
			glEnable(GL_CLIP_DISTANCE0 + i);

		prog.set_uniform(ctx, "culling_mode", (int)cull_mode);
		prog.set_uniform(ctx, "map_color_to_material", (int)color_mapping);
		prog.set_uniform(ctx, "illumination_mode", (int)illumination_mode);
		// set default surface color for color mapping which only affects 
		// rendering if mesh does not have per Vector colors and color_mapping is on
		if (prog.get_color_index() != -1)
			prog.set_attribute(ctx, prog.get_color_index(), surface_color);

		// render the mesh from the Vector buffers with selected program
		if (!M.get_positions().empty())
			mesh_info.draw_all(ctx, !opaque_part, opaque_part);
		if (!r_info.ref_draw_calls().empty())
			r_info.draw_all(ctx, !opaque_part, opaque_part);

		// recover opengl culling mode
		if (is_culling)
			glEnable(GL_CULL_FACE);
		else
			glDisable(GL_CULL_FACE);
		glCullFace(cull_face);
		for (i = 0; i < nr_clip_distances; ++i)
			glDisable(GL_CLIP_DISTANCE0 + i);
	}
	void draw(context& ctx)
	{
		if (!M.get_positions().empty()) {
			if (show_vertices) {
				sphere_renderer& sr = ref_sphere_renderer(ctx);
				if (view_ptr)
					sr.set_y_view_angle(float(view_ptr->get_y_view_angle()));
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
			if (show_wireframe) {
				rounded_cone_renderer& cr = ref_rounded_cone_renderer(ctx);
				cr.set_render_style(cone_style);
				if (cr.enable(ctx)) {
					mesh_info.draw_wireframe(ctx);
					cr.disable(ctx);
				}
			}
			//render bounding box
			if (show_bounding_box) {
				box_wire_renderer  box_render;
				box_render.init(ctx);
				visit_tree(aabb_tree.Root());
				box_render.set_box_array(ctx, boxes);

				if (box_render.validate_and_enable(ctx))
				{
					glDrawArrays(GL_POINTS, 0, (GLsizei)boxes.size());
					box_render.disable(ctx);
				}

			}
		}
		if (show_surface) {
			draw_surface(ctx, true);
		}
	}
	//push back the leaf node
	void visit_tree(AabbTree<triangle>::AabbNode* a)
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
	void draw_planes(context& ctx)
	{
		if (planes.empty())
			return;

		std::vector<vec3> P;
		std::vector<vec3> N;
		std::vector<rgba> C;
		for (const auto& pln : planes)
			compute_plane_points(pln, P, &N, &C);

		sphere_renderer& sr = ref_sphere_renderer(ctx);
		if (view_ptr)
			sr.set_y_view_angle(float(view_ptr->get_y_view_angle()));
		sr.set_render_style(sphere_style);
		sr.set_position_array(ctx, P);
		sr.set_color_array(ctx, C);
		sr.validate_and_enable(ctx);
		ctx.set_color(rgba(0, 0, 0, 0));
		glDrawArrays(GL_POINTS, 0, (GLsizei)P.size());
		sr.disable(ctx);

		auto& prog = ctx.ref_surface_shader_program();
		attribute_array_binding::set_global_attribute_array(ctx, prog.get_position_index(), P);
		attribute_array_binding::enable_global_array(ctx, prog.get_position_index());
		attribute_array_binding::set_global_attribute_array(ctx, prog.get_normal_index(), N);
		attribute_array_binding::enable_global_array(ctx, prog.get_normal_index());
		attribute_array_binding::set_global_attribute_array(ctx, prog.get_color_index(), C);
		attribute_array_binding::enable_global_array(ctx, prog.get_color_index());
		glDisable(GL_CULL_FACE);
		prog.enable(ctx);
		prog.set_uniform(ctx, "culling_mode", 0);
		prog.set_uniform(ctx, "map_color_to_material", 15);
		for (unsigned i = 0; i < planes.size(); ++i)
			glDrawArrays(GL_TRIANGLE_STRIP, 4 * i, 4);
		prog.disable(ctx);
		glEnable(GL_CULL_FACE);
		attribute_array_binding::disable_global_array(ctx, prog.get_position_index());
		attribute_array_binding::disable_global_array(ctx, prog.get_normal_index());
		attribute_array_binding::disable_global_array(ctx, prog.get_color_index());
	}
	void finish_frame(context& ctx)
	{
		if (show_surface)
			draw_surface(ctx, false);

		glEnable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

		draw_planes(ctx);

		sphere_renderer& sr = ref_sphere_renderer(ctx);
		if (view_ptr)
			sr.set_y_view_angle(float(view_ptr->get_y_view_angle()));
		sr.set_render_style(sphere_style);

		glDepthMask(GL_FALSE);
		if (!pick_points.empty()) {
			glDisable(GL_DEPTH_TEST);
			sr.set_render_style(sphere_hidden_style);
			sr.set_position_array(ctx, pick_points);
			sr.set_color_array(ctx, pick_colors);
			sr.validate_and_enable(ctx);
			ctx.set_color(rgba(0, 0, 0, 0));
			glDrawArrays(GL_POINTS, 0, (GLsizei)pick_points.size());
			sr.disable(ctx);
			sr.set_render_style(sphere_style);
			glEnable(GL_DEPTH_TEST);
		}
		if (!pick_points.empty()) {
			sr.set_position_array(ctx, pick_points);
			sr.set_color_array(ctx, pick_colors);
			sr.validate_and_enable(ctx);
			glDrawArrays(GL_POINTS, 0, (GLsizei)pick_points.size());
			sr.disable(ctx);
		}
		if (!sphere_positions.empty()) {
			float tmp = sphere_style.radius_scale;
			sphere_style.radius_scale = 1;
			sr.set_position_array(ctx, sphere_positions);
			sr.set_color_array(ctx, sphere_colors);
			sr.set_radius_array(ctx, sphere_radii);
			sr.render(ctx, 0, sphere_positions.size());
			sphere_style.radius_scale = tmp;
		}

		if (pick_point_index != -1) {
			glDisable(GL_BLEND);
			glDisable(GL_DEPTH_TEST);
			vec3 p = pick_points[pick_point_index];
			if (view_ptr)
				p += 1.5f*sphere_style.radius*sphere_style.radius_scale*vec3(view_ptr->get_view_up_dir());
			std::stringstream ss;
			ss << "[" << p << "]";
			ss.flush();

			ctx.set_color(rgb(0.1f, 0.1f, 0.1f));
			ctx.set_cursor(p.to_vec(), ss.str(), TA_BOTTOM, 0, 0);
			ctx.output_stream() << ss.str();
			ctx.output_stream().flush();

			ctx.set_color(rgb(0.9f, 0.9f, 0.9f));
			ctx.set_cursor(p.to_vec(), ss.str(), TA_BOTTOM, 1, -1);
			ctx.output_stream() << ss.str();
			ctx.output_stream().flush();

			glEnable(GL_DEPTH_TEST);
			glEnable(GL_BLEND);
		}
		glDepthMask(GL_TRUE);
		glDisable(GL_BLEND);
	}

	HE_Mesh* generate_from_simple_mesh(mesh_type M) {
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
			if(edge_it ->twin == nullptr)
				newMesh->AddBoundary(edge_it);
		}
		uniqueTriples.clear();
		triangleBuffer.clear();
		vectorIndices.clear();


		
		return newMesh;
	}

	void debug_mesh_generation() {
		auto generated_mesh = generate_from_simple_mesh(M);

		if (generated_mesh == nullptr) return;

		vec3 t,s;
		HE_Face* f;
		// TODO use newMesh for further tasks
		std::cout << "surface: " << mesh_utils::surface(generated_mesh) << std::endl;
		std::cout << "volume: " << mesh_utils::volume(generated_mesh) << std::endl;
		std::cout << "shortest distance to mesh from (0,0,0): " << mesh_utils::shortest_distance(vec3(0, 0, 0), generated_mesh,f,t) << std::endl;
		std::cout << "closest point: " << t << std::endl;
		std::cout << "AD shortest distance to mesh from (0,0,0): " << mesh_utils::shortest_distance_AD(vec3(0, 0, 0), aabb_tree, s) << std::endl;
		std::cout << "AD closest point: " << s << std::endl;
		

		return;

		for (auto face : *generated_mesh->GetFaces()) {
			std::cout << "Face: " << std::endl;
			for (auto vertex : generated_mesh->GetVerticesForFace(face)) {
				std::cout << "\t" << vertex->position.x() << ", " << vertex->position.y() << ", " << vertex->position.z() << "\n" << std::endl;
			}
		}

		delete generated_mesh;
	}
};
//use inverse translation mat4 to calculate the view position in local coordinate
	//return loacl viewposition
	vec3 view_translation(vec3 view_position, std::vector<vec3> point_path) {
		
		vec4 viewposition, new_view;
		viewposition[0] = view_position[0];
		viewposition[1] = view_position[1];
		viewposition[2] = view_position[2];
		viewposition[3] = 1.0;

		mesh_view::fill_animationpath(point_path);
			for (int i = 1; i < point_path.size(); ++i) {
				if (i < 1)
					break;
				mat4 mat_translation, inv_translation;
				mat_translation.identity();
				vec3 v = point_path[i] - point_path[i - 1];
				mesh_utils::shiftPositions(he, v);

				mat_translation(0,3) = v[0];
				mat_translation(1, 3) = v[1];
				mat_translation(2, 3) = v[2];
				mat_translation(3, 3) = 1;
				
				inv_translation = inv(mat_translation);
				for (int j = 0; j < 4;j++) 
				{
					float sum=0;
					for (int k = 0; k < 4; k++)
					{
						sum += inv_translation(j, k)*viewposition[k];
					}
					new_view[j] = sum;
					

				}	
				viewposition = new_view;
				
				
			}
			vec3 view;
			view[0] = viewposition[0];
			view[1] = viewposition[1];
			view[2] = viewposition[2];
		return view;
	}
#include <cgv/base/register.h>

/// register a factory to create new cubes
cgv::base::object_registration<mesh_view> mesh_view_fac("");
