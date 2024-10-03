#include "WalkMesh.hpp"

#include "read_write_chunk.hpp"

#include <glm/gtx/norm.hpp>
#include <glm/gtx/string_cast.hpp>

#include <iostream>
#include <fstream>
#include <algorithm>
#include <string>

WalkMesh::WalkMesh(std::vector< glm::vec3 > const &vertices_, std::vector< glm::vec3 > const &normals_, std::vector< glm::uvec3 > const &triangles_)
	: vertices(vertices_), normals(normals_), triangles(triangles_) {

	//construct next_vertex map (maps each edge to the next vertex in the triangle):
	next_vertex.reserve(triangles.size()*3);
	auto do_next = [this](uint32_t a, uint32_t b, uint32_t c) {
		std::__1::pair<std::__1::unordered_map<glm::uvec2, uint32_t>::iterator, bool> ret = next_vertex.insert(std::make_pair(glm::uvec2(a,b), c));
		assert(ret.second);
	};
	for (auto const &tri : triangles) {
		do_next(tri.x, tri.y, tri.z);
		do_next(tri.y, tri.z, tri.x);
		do_next(tri.z, tri.x, tri.y);
	}

	//DEBUG: are vertex normals consistent with geometric normals?
	// for (auto const &tri : triangles) {
		// glm::vec3 const &a = vertices[tri.x];
		// glm::vec3 const &b = vertices[tri.y];
		// glm::vec3 const &c = vertices[tri.z];
		// glm::vec3 out = glm::normalize(glm::cross(b-a, c-a));

		// float da = glm::dot(out, normals[tri.x]);
		// float db = glm::dot(out, normals[tri.y]);
		// float dc = glm::dot(out, normals[tri.z]);

		// assert(da > 0.1f && db > 0.1f && dc > 0.1f);
	// }
}

//project pt to the plane of triangle a,b,c and return the barycentric weights of the projected point:
glm::vec3 barycentric_weights(glm::vec3 const &a, glm::vec3 const &b, glm::vec3 const &c, glm::vec3 const &pt) {
	// projection part
	// glm::vec3 n = glm::normalize(glm::cross(b-a,c-a));
	// glm::vec3 proj_onto_normal = (glm::dot(pt, n) / glm::length(n)) * n;
	// glm::vec3 proj_onto_plane = pt-proj_onto_normal;
	// float dist_from_tri = glm::dot(a, n);
	// glm::vec3 p = proj_onto_plane + dist_from_tri * n;
	glm::vec3 p = pt;
	// barycentric weights part
	// cross products
	glm::vec3 ba_x_ca = glm::cross(b-a,c-a);
	glm::vec3 pxa = glm::cross(p-b,p-c);
	glm::vec3 pxb = glm::cross(p-c,p-a);
	glm::vec3 pxc = glm::cross(p-a,p-b);
	// find area of whole triangle using cross products
	float pa_area = glm::dot(pxa, ba_x_ca);
	float pb_area = glm::dot(pxb, ba_x_ca);
	float pc_area = glm::dot(pxc, ba_x_ca);
	float full_area = pa_area+pb_area+pc_area;
	float weight_a = pa_area / full_area;
	float weight_b = pb_area / full_area;
	float weight_c = pc_area / full_area;
	// inside outside testing
	// if (weight_a + weight_b + weight_c > 1.) {
	// 	float sum_a = std::abs(1. + weight_a - weight_b - weight_c);
	// 	float sum_b = std::abs(1. - weight_a + weight_b - weight_c);
	// 	float sum_c = std::abs(1. - weight_a - weight_b + weight_c);
	// 	float min_sum = std::min(sum_a, std::min(sum_b, sum_c));
	// 	if (min_sum == sum_a) weight_a = -weight_a;
	// 	else if (min_sum == sum_b) weight_b = -weight_b;
	// 	else if (min_sum == sum_c) weight_c = -weight_c;
	// }
	return glm::vec3(weight_a, weight_b, weight_c);
}

WalkPoint WalkMesh::nearest_walk_point(glm::vec3 const &world_point) const {
	assert(!triangles.empty() && "Cannot start on an empty walkmesh");

	WalkPoint closest;
	float closest_dis2 = std::numeric_limits< float >::infinity();

	for (auto const &tri : triangles) {
		//find closest point on triangle:

		glm::vec3 const &a = vertices[tri.x];
		glm::vec3 const &b = vertices[tri.y];
		glm::vec3 const &c = vertices[tri.z];

		//get barycentric coordinates of closest point in the plane of (a,b,c):
		glm::vec3 coords = barycentric_weights(a,b,c, world_point);

		//is that point inside the triangle?
		if (coords.x >= 0.0f && coords.y >= 0.0f && coords.z >= 0.0f) {
			//yes, point is inside triangle.
			float dis2 = glm::length2(world_point - to_world_point(WalkPoint(tri, coords)));
			if (dis2 < closest_dis2) {
				closest_dis2 = dis2;
				closest.indices = tri;
				closest.weights = coords;
			}
		} else {
			//check triangle vertices and edges:
			auto check_edge = [&world_point, &closest, &closest_dis2, this](uint32_t ai, uint32_t bi, uint32_t ci) {
				glm::vec3 const &a = vertices[ai];
				glm::vec3 const &b = vertices[bi];

				//find closest point on line segment ab:
				float along = glm::dot(world_point-a, b-a);
				float max = glm::dot(b-a, b-a);
				glm::vec3 pt;
				glm::vec3 coords;
				if (along < 0.0f) {
					pt = a;
					coords = glm::vec3(1.0f, 0.0f, 0.0f);
				} else if (along > max) {
					pt = b;
					coords = glm::vec3(0.0f, 1.0f, 0.0f);
				} else {
					float amt = along / max;
					pt = glm::mix(a, b, amt);
					coords = glm::vec3(1.0f - amt, amt, 0.0f);
				}

				float dis2 = glm::length2(world_point - pt);
				if (dis2 < closest_dis2) {
					closest_dis2 = dis2;
					closest.indices = glm::uvec3(ai, bi, ci);
					closest.weights = coords;
				}
			};
			check_edge(tri.x, tri.y, tri.z);
			check_edge(tri.y, tri.z, tri.x);
			check_edge(tri.z, tri.x, tri.y);
		}
	}
	assert(closest.indices.x < vertices.size());
	assert(closest.indices.y < vertices.size());
	assert(closest.indices.z < vertices.size());
	return closest;
}


void WalkMesh::walk_in_triangle(WalkPoint const &start, glm::vec3 const &step, WalkPoint *end_, float *time_) const {
	assert(end_);
	auto &end = *end_;

	assert(time_);
	auto &time = *time_;

	glm::vec3 step_coords;
	glm::vec3 w = start.weights;
	glm::vec3 a = vertices[start.indices[0]];
	glm::vec3 b = vertices[start.indices[1]];
	glm::vec3 c = vertices[start.indices[2]];
	glm::vec3 start_world = a*w[0] + b*w[1] + c*w[2];
	{ //project 'step' into a barycentric-coordinates direction:
		step_coords = barycentric_weights(a, b, c, start_world+step);
	}
	glm::vec3 v = step_coords - w;
	//if no edge is crossed, event will just be taking the whole step:
	time = 1.0f;
	end = start;

	//figure out which edge (if any) is crossed first.
	bool edge_a_crossed = step_coords[0] < 0.; 
	bool edge_b_crossed = step_coords[1] < 0.;
	bool edge_c_crossed = step_coords[2] < 0.;
	bool no_edge_crossed = !(edge_a_crossed || edge_b_crossed || edge_c_crossed);
	// set time and end appropriately.
	if (no_edge_crossed) {
		//   - *end will be the position after stepping
		//   - *remaining_step will be glm::vec3(0.0)
		end = {start.indices, step_coords};
		return;
	}
	glm::vec3 t = glm::vec3(std::numeric_limits<float>::infinity());
	if (edge_a_crossed && v[0] != 0.) {
		t[0] = w[0]/(-v[0]);
		time = t[0];
		glm::vec3 endw = (w + time*v);
		end.weights = glm::vec3(endw[1], endw[2], 0.);
		end.indices = glm::vec3(start.indices[1], start.indices[2], start.indices[0]);
	} if (edge_b_crossed && v[1] != 0.) {
		t[1] = w[1]/(-v[1]);
		if (t[1] < t[0]){
			time = t[1];
			glm::vec3 endw = (w + time*v);
			end.weights = glm::vec3(endw[2], endw[0], 0.);
			end.indices = glm::vec3(start.indices[2], start.indices[0], start.indices[1]);
		}
	} if (edge_c_crossed && v[2] != 0.) {
		t[2] = w[2]/(-v[2]);
		if (t[2] < t[0] && t[2] < t[1]){
			time = t[2];
			glm::vec3 endw = (w + time*v);
			end.weights = endw;
			end.weights[2] = 0.;
			end.indices = start.indices;
		}
	}
	//Remember: our convention is that when a WalkPoint is on an edge,
	// then wp.weights.z == 0.0f (so will likely need to re-order the indices)
}

bool WalkMesh::cross_edge(WalkPoint const &start, WalkPoint *end_, glm::quat *rotation_) const {
	assert(end_);
	auto &end = *end_;

	assert(rotation_);
	auto &rotation = *rotation_;

	assert(start.weights.z == 0.0f); //*must* be on an edge.
	glm::uvec2 edge = glm::uvec2(start.indices);
	// glm::vec3 normal_old = to_world_triangle_normal(start);
	//check if 'edge' is a non-boundary edge:
	if (next_vertex.count(glm::uvec2(edge.y, edge.x)) > 0){ 
		uint32_t next_vert = next_vertex.at(glm::uvec2(edge.y, edge.x));
		//it is!
		//make 'end' represent the same (world) point, but on triangle (edge.y, edge.x, [other point]):
		end.indices = glm::vec3(edge.y, edge.x, next_vert);
		end.weights = glm::vec3(start.weights[1], start.weights[0], 0.);
		glm::vec3 normal_new = to_world_triangle_normal(end);

		//make 'rotation' the rotation that takes (start.indices)'s normal to (end.indices)'s normal:
		rotation = glm::rotation(to_world_triangle_normal(start), normal_new);
		return true;
	} else {
		end = start;
		rotation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
		return false;
	}
}


WalkMeshes::WalkMeshes(std::string const &filename) {
	std::ifstream file(filename, std::ios::binary);

	std::vector< glm::vec3 > vertices;
	read_chunk(file, "p...", &vertices);

	std::vector< glm::vec3 > normals;
	read_chunk(file, "n...", &normals);

	std::vector< glm::uvec3 > triangles;
	read_chunk(file, "tri0", &triangles);

	std::vector< char > names;
	read_chunk(file, "str0", &names);

	struct IndexEntry {
		uint32_t name_begin, name_end;
		uint32_t vertex_begin, vertex_end;
		uint32_t triangle_begin, triangle_end;
	};

	std::vector< IndexEntry > index;
	read_chunk(file, "idxA", &index);

	if (file.peek() != EOF) {
		std::cerr << "WARNING: trailing data in walkmesh file '" << filename << "'" << std::endl;
	}

	//-----------------

	if (vertices.size() != normals.size()) {
		throw std::runtime_error("Mis-matched position and normal sizes in '" + filename + "'");
	}

	for (auto const &e : index) {
		if (!(e.name_begin <= e.name_end && e.name_end <= names.size())) {
			throw std::runtime_error("Invalid name indices in index of '" + filename + "'");
		}
		if (!(e.vertex_begin <= e.vertex_end && e.vertex_end <= vertices.size())) {
			throw std::runtime_error("Invalid vertex indices in index of '" + filename + "'");
		}
		if (!(e.triangle_begin <= e.triangle_end && e.triangle_end <= triangles.size())) {
			throw std::runtime_error("Invalid triangle indices in index of '" + filename + "'");
		}

		//copy vertices/normals:
		std::vector< glm::vec3 > wm_vertices(vertices.begin() + e.vertex_begin, vertices.begin() + e.vertex_end);
		std::vector< glm::vec3 > wm_normals(normals.begin() + e.vertex_begin, normals.begin() + e.vertex_end);

		//remap triangles:
		std::vector< glm::uvec3 > wm_triangles; wm_triangles.reserve(e.triangle_end - e.triangle_begin);
		for (uint32_t ti = e.triangle_begin; ti != e.triangle_end; ++ti) {
			if (!( (e.vertex_begin <= triangles[ti].x && triangles[ti].x < e.vertex_end)
			    && (e.vertex_begin <= triangles[ti].y && triangles[ti].y < e.vertex_end)
			    && (e.vertex_begin <= triangles[ti].z && triangles[ti].z < e.vertex_end) )) {
				throw std::runtime_error("Invalid triangle in '" + filename + "'");
			}
			wm_triangles.emplace_back(
				triangles[ti].x - e.vertex_begin,
				triangles[ti].y - e.vertex_begin,
				triangles[ti].z - e.vertex_begin
			);
		}
		
		std::string name(names.begin() + e.name_begin, names.begin() + e.name_end);

		auto ret = meshes.emplace(name, WalkMesh(wm_vertices, wm_normals, wm_triangles));
		if (!ret.second) {
			throw std::runtime_error("WalkMesh with duplicated name '" + name + "' in '" + filename + "'");
		}

	}
}

WalkMesh const &WalkMeshes::lookup(std::string const &name) const {
	auto f = meshes.find(name);
	if (f == meshes.end()) {
		throw std::runtime_error("WalkMesh with name '" + name + "' not found.");
	}
	return f->second;
}
