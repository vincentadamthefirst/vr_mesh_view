#pragma once

#include <vector>
#include <cgv/math/fvec.h>
#include <cgv/media/axis_aligned_box.h>
#include <cgv/media/mesh/simple_mesh.h>
#include "halfedgemesh.h"
#include "triangle.h"

template <typename T>
class AabbTree {

public:
	typedef cgv::math::fvec<float, 3> vec3;
	//define primitive list type 
	typedef std::vector<T> primitive_li; 
	//define primitive list iterator type
	typedef typename primitive_li::iterator primitive_iterator;
	//define axis aligned bounding box type
	typedef typename cgv::media::axis_aligned_box<float, 3> box_type;

	primitive_li primitive_list;
	class AabbNode {
	protected:
		box_type box;
	public:
		AabbNode() {}
		AabbNode(const box_type& b) :box(b) {}
		box_type get_box() {
			return box;
		}
		virtual AabbNode* left_child() = 0;
		virtual AabbNode* right_child() = 0;
		virtual bool is_leaf() const = 0;
		virtual std::vector<vec3> get_triangle() = 0;
		virtual ~AabbNode() {}
	};
	class AabbLeafNode : public AabbNode {
		primitive_iterator primitives_begin, primitives_end;
	public:
		AabbLeafNode(const primitive_iterator& primitivesBegin,
			const primitive_iterator& primitivesEnd,
			const box_type& b) :
			primitives_begin(primitivesBegin), primitives_end(primitivesEnd), AabbNode(b){}
		//return always true because its a leaf node
		bool is_leaf() const
		{
			return true;
		}
		//returns the number primitives assosiated with the current leaf
		int get_nr_pri()
		{
			return (int)(primitives_end - primitives_begin);
		}
		//returns null pointer because of leaf node
		AabbNode* left_child() {
			return nullptr;
		}
		//returns null pointer because of leaf node
		AabbNode* right_child() {
			return nullptr;
		}
		std::vector<vec3> get_triangle() {
			return primitives_begin->get();
		}
	};
	class AabbSplitNode : public AabbNode {
		AabbNode* children[2];
	public:
		//default constructor
		AabbSplitNode()
		{
			children[0] = children[1] = nullptr;
		}
		//construct a split node from given Left and right child pointers and given bounding box b of the node
		AabbSplitNode(AabbNode* left_child, AabbNode* right_child, const box_type& b) : AabbNode(b)
		{
			children[0] = left_child;
			children[1] = right_child;
		}
		//return left child of this node which is a aabbnode type
		AabbNode* left_child() {
			return children[0];
		}
		//return left child of this node which is a aabbnode type
		AabbNode* right_child() {
			return children[1];
		}
		//destructor of node, recursively deleted whole subtree
		~AabbSplitNode()
		{
			if (left_child() != nullptr)
				delete left_child();
			if (right_child() != nullptr)
				delete right_child();
		}

		//returns always false because its a split node
		bool is_leaf() const
		{
			return false;
		}
		//return null because of split node
		std::vector<vec3> get_triangle() {
			return {};
		}
		
	
	};
private:
	
	AabbNode* root;

public:
	
	bool completed;
	int tree_depth = 0 ;
	AabbTree() {
		root = nullptr;
		completed = false;
	}
	//insert primitives to primitive list
	void insert(T& tri) {
		primitive_list.push_back(tri);
		completed = false;
	}
	//build aabb tree from root
	void pre_build() {
		box_type box = compute_box(primitive_list.begin(), primitive_list.end());
		if (root != nullptr) {
			delete root;
		}
		root = build(primitive_list.begin(), primitive_list.end(), box, 0);
		completed = true;
		std::cout << "depth: " << tree_depth << std::endl;
	}
	
	
	void clear()
	{
		primitive_list.clear();
		if (root != nullptr)
		{
			delete root;
			root = nullptr;
		}
		completed = false;
		tree_depth = 0;
	}
	bool is_completed() {
		return completed;
	}
	AabbNode* Root()
	{
		assert(is_completed());
		return root;
	}

protected:
	box_type compute_box(primitive_iterator begin, primitive_iterator end) {
			box_type box;
			for (auto i = begin; i != end; ++i) {
				box.add_axis_aligned_box(i->compute_box());
			}
			return box;
		}

	AabbNode* build(primitive_iterator pri_it_begin, primitive_iterator pri_it_end, box_type& box, int depth) {
			if (pri_it_end - pri_it_begin <= 1) {
				tree_depth = depth > tree_depth ? depth: tree_depth;
				return new AabbLeafNode(pri_it_begin, pri_it_end, box);
			}
			vec3 e = box.get_extent();
			int axis = 0;
			float max_extent = e[0];
			if (max_extent < e[1])
			{
				axis = 1;
				max_extent = e[1];
			}
			if (max_extent < e[2])
			{
				axis = 2;
				max_extent = e[2];
			}


			primitive_iterator mid = pri_it_begin + (pri_it_end - pri_it_begin) / 2;
			//nth_element funtion, in order to sort the triangle along the largest box extend by calculating the centorid of triangle
			std::nth_element(pri_it_begin, mid, pri_it_end, [&axis](const triangle& a, const triangle& b)
				{ return a.reference_point()[axis] < b.reference_point()[axis];});

			box_type box_1 = compute_box(pri_it_begin, mid);
			box_type box_2 = compute_box(mid, pri_it_end);

			return new AabbSplitNode(build(pri_it_begin, mid, box_1, depth + 1), build(mid, pri_it_end, box_2, depth + 1), box);
		}
};

void build_aabbtree_from_triangles(HE_Mesh* he, AabbTree<triangle >& tree);