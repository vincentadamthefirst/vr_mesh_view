#pragma once
#include <cgv/base/node.h>
#include <cgv/render/drawable.h>
#include <cgv/gui/provider.h>
#include <cgv/gui/event_handler.h>
#include <cgv_gl/box_renderer.h>
#include <cgv_gl/gl/mesh_render_info.h>
#include <cgv/render/shader_program.h>
#include <cgv_gl/rounded_cone_renderer.h>
#include <cgv/render/frame_buffer.h>

// necessary imports to render a mesh
#include <cgv_gl/gl/gltf_support.h>
#include <cgv/gui/file_dialog.h>
#include <cgv_gl/box_wire_renderer.h>

///@ingroup VR
///@{

/**@file
   example plugin for vr usage
*/

// these are the vr specific headers
#include <vr/vr_driver.h>
#include <cg_vr/vr_server.h>
#include <vr_view_interactor.h>
#include <vr_render_helpers.h>

#include "simple_csg.h"

class vr_mesh_view :
	public cgv::base::node,
	public cgv::render::drawable,
	public cgv::gui::event_handler,
	public cgv::gui::provider {
protected:
	// typedefs
	typedef cgv::media::mesh::simple_mesh<float> mesh_type;
	typedef mesh_type::idx_type idx_type;
	typedef mesh_type::vec3i vec3i;

	// different interaction states for the controllers
	enum InteractionState {
		IS_NONE,
		IS_OVER,
		IS_GRAB
	};

	/// The different possibilities to decorate the room, NONE = empty void, ROOM_TABLE = room with some objects
	/// ROOM_BOXES = room with floor and colored boxes around, ROOM_WALLS = room with floor, walls and ceiling
	enum DecorState {
		NONE = 0, ROOM_TABLE, ROOM_BOXES, ROOM_WALLS
	};

	// rendering style for boxes
	cgv::render::box_render_style style;

	cgv::render::rounded_cone_render_style cone_style;
	// sample for rendering a mesh
	double mesh_scale;
	dvec3 mesh_location;
	dquat mesh_orientation;

	// render information for mesh
	cgv::render::mesh_render_info MI;
	cgv::render::mesh_render_info MI_smoothing;

	// keep deadzone and precision vector for left controller
	cgv::gui::vr_server::vec_flt_flt left_deadzone_and_precision;
	// store handle to vr kit of which left deadzone and precision is configured
	void* last_kit_handle;

	// length of to be rendered rays
	float ray_length;

	// keep reference to vr_view_interactor
	vr_view_interactor* vr_view_ptr;

	// intersection points
	std::vector<vec3> intersection_points;
	std::vector<float> intersection_offsets;
	std::vector<rgb>  intersection_colors;
	std::vector<int>  intersection_box_indices;
	std::vector<int>  intersection_controller_indices;

	// state of current interaction with boxes for each controller
	InteractionState state[4];

	// render style for interaction
	cgv::render::sphere_render_style srs;
	cgv::render::sphere_render_style srs2;
	
	cgv::render::box_render_style movable_style;

	int nr_cameras;
	int frame_width, frame_height;
	int frame_split;
	float seethrough_gamma;
	mat4 camera_to_head_matrix[2];
	cgv::math::fmat<float, 4, 4> camera_projection_matrix[4];
	vec2 focal_lengths[4];
	vec2 camera_centers[4];
	cgv::render::texture camera_tex;
	cgv::render::shader_program seethrough;
	GLuint camera_tex_id;
	bool undistorted;
	bool shared_texture;
	bool max_rectangle;
	float camera_aspect;
	bool show_seethrough;
	bool use_matrix;
	float background_distance;
	float background_extent;
	vec2 extent_texcrd;
	vec2 center_left;
	vec2 center_right;

	// VA 16.06.2020
	// added fields for compatibility with mesh renderer

	// environment
	std::vector<box3> environment_boxes;
	std::vector<rgb> box_colors;

	// .obj file
	std::string file_name;

	//bounding box related member
	AabbTree<triangle> aabb_tree;
	std::vector<box3> boxes;

	// renderers & shaders
	cgv::render::render_info r_info;
	cgv::render::shader_program mesh_prog;
	cgv::render::shader_program mesh_prog_smoothing;

	// coloring & rendering information
	cgv::render::sphere_render_style sphere_style;
	cgv::render::sphere_render_style sphere_hidden_style;
	cgv::render::CullingMode cull_mode;
	cgv::render::ColorMapping color_mapping;
	cgv::render::IlluminationMode illumination_mode;
	rgb surface_color = rgb(0,0,1);

	// the mesh
	bool have_new_mesh;
	mesh_type M;
	box3 B;
	HE_Mesh* he;

	// renderer controls
	bool show_wireframe;
	bool show_bounding_box;
	bool show_vertices;
	bool show_surface;

	// environment rendering control
	DecorState decorState;

	// Simple CSG elements
	// if a sphere should be draw, used for when the user tries to construct an icosphere
	bool draw_icoSphere = false; 
	// set when beginning to construct an icosphere
	vec3 icoSphere_center = vec3(0, 0, 0);
	// the radius of the icosphere = distance(icoSphere_center, current controller position)
	float icoSphere_radius = 1.0f;
	// the selected CSG Operation
	CSG_Operation csg_op = CSG_Operation::CSG_UNION;
	// the selected number of subdivisions
	int icoSphere_subdivisions = 3;

	bool show_animationpath = false;

	bool destructSmoothingMesh;

	// gui object
	vec3 translate_vector;

	// transformation
	mat4 transformation_matrix;
	mat3 mesh_rotation_matrix;
	vec3 mesh_translation_vector;

	// vr control variables
	// if the controller buttons are pressed (based on oculus)
	bool bButtonIsPressed = false;
	bool yButtonIsPressed = false;

	//tessellation 
	bool rightButton1IsPressed = false;

	//vertex manipulation
	bool leftButton1IsPressed = false;
	bool isVertexPicked = false;

	bool rightButton3IsPressed = false;

	HE_Vertex* intersectedVertex;

	// positions only get written when y and b key (oculus) are held
	vec3 rightControllerPosition;
	vec3 leftControllerPosition;
	bool aniButton1IsPressed = false;

	std::vector<HE_Vertex*> smoothingpoints;
	const std::string mesheditingmode_text = "Mesh Editing Mode \nButtons:\nMenu: Change Mode\nL-Stick L: Tesselation\nL-Stick U: Vertex Manipulation\nL-Stick D: Select Smoothing Face\nL-Stick R: Apply Smoothing\n\nR-Stick U: Vertex Deletion\nR-Stick R: Recalculate Measurements\n";
	const std::string animationmode_text = "Animation Mode \nButtons:\nMenu: Change Mode\nTouch R-Stick: Draw Animation path \n+ translation\nR-Stick D: Go back to Origin\nTouch R-Stick and press R-Stick D: \nRestart Animation \nTouch L-Stick: Rotation and Translation\n";
	std::string label_text;
	int label_font_idx;
	bool label_upright;
	float label_size;
	rgb label_color;

	bool label_outofdate; // whether label texture is out of date
	unsigned label_resolution; // resolution of label texture
	cgv::render::texture label_tex; // texture used for offline rendering of label
	cgv::render::frame_buffer label_fbo; // fbo used for offline rendering of label

	// general font information
	std::vector<const char*> font_names;
	std::string font_enum_decl;

	// current font face used
	cgv::media::font::font_face_ptr label_font_face;
	cgv::media::font::FontFaceAttributes label_face_type;
	
	mesh_type smoothingMesh;
	box3 B_smoothing;

	bool have_new_smoothingMesh;

	bool animationmode = false;

	bool new_closest_point = false;

	vec3 closestPoint, referenceP;

	std::vector<vec3> color_list;
	int pathi = 0;

	bool rightButton2IsPressed = false;

	bool animation_start = false;
	vec3 mesh_centroid;

	std::vector<vec3> defined_path2;
	std::vector<vec3> defined_path;

public:
	void init_cameras(vr::vr_kit* kit_ptr);
	
	void start_camera();

	void stop_camera();

	/// compute intersection points of controller ray with movable boxes
	void compute_intersections(const vec3& origin, const vec3& direction, int ci, const rgb& color);
	/// register on device change events
	void on_device_change(void* kit_handle, bool attach);
	/// construct boxes that represent a room of dimensions w,d,h and wall width W
	void construct_room(DecorState decorState, float w, float d, float h, float W);
	/// construct boxes for environment
	void construct_environment(DecorState decorState, float s, float ew, float ed, float w, float d, float h);
	/// construct a scene with a table
	void build_scene(DecorState decorState, float w, float d, float h, float W);

public:
	vr_mesh_view();

	std::string get_type_name() { return "vr_mesh_view"; }

	void stream_help(std::ostream& os);

	void on_set(void* member_ptr);

	bool handle(cgv::gui::event& e);
	
	bool init(cgv::render::context& ctx);

	void clear(cgv::render::context& ctx);

	void init_frame(cgv::render::context& ctx);

	void draw(cgv::render::context& ctx);

	void finish_draw(cgv::render::context& ctx);

	void create_gui();

public:
	

	void visit_tree(AabbTree<triangle>::AabbNode* a);

	/// performs simple csg operation when the IcoSphere button is released
	void perform_simple_csg(CSG_Operation operation);
	/// reads a mesh from file
	bool read_main_mesh(const std::string& file_name);
	/// loads a new simple_mesh to be edited
	bool read_decor_meshes(const std::string& base_path);
	/// generates a HE_Mesh from a simple mesh
	HE_Mesh* generate_from_simple_mesh(mesh_type M);
	/// draws the surface of the main mesh
	void draw_surface(cgv::render::context& ctx, bool opaque_part);
	/// draws all elements into the scene that are static (not mesh related)
	void draw_room(cgv::render::context& ctx);
	/// draws a sphere for SimpleCSG (if needed)
	void draw_csgIcoSphere(cgv::render::context& ctx);

	void draw_surface_2(cgv::render::context& ctx, bool opaque_part);

	void add_translation(vec3 v);
	void add_translation(mat3 r, vec3 v);
	void add_rotation(float angle, vec3 axis);
	void add_rotation(vec3 angles);
	void add_rotation(mat3 rotation);
	vec3 global_to_local(vec3 pos);
	vec3 local_to_global(vec3 pos);

	void applySmoothing();

	void applySmoothingPoints();
	void tessellation(const vec3& origin, const vec3& direction);
	void vertex_deletion(const vec3& origin, const vec3& direction);
	void vertex_manipulate(HE_Vertex* vertex, vec3 pos, vec3 last_pos);
	void drawpath(cgv::render::context& ctx, std::vector<vec3> path_list);
	void build_simple_mesh_from_HE();

	void add_face_to_smoothingMesh(HE_Face* f);
	void update_measurements();
	void update_measurements(vec3 point, bool ad);

	void drawClosestPoint(cgv::render::context& ctx, vec3 point);

};

///@}
