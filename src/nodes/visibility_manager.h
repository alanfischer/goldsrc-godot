#pragma once

#include <godot_cpp/classes/node.hpp>
#include <godot_cpp/variant/vector3.hpp>
#include <godot_cpp/variant/aabb.hpp>
#include <godot_cpp/variant/string.hpp>
#include <godot_cpp/variant/packed_int32_array.hpp>

#include <vector>

class GoldSrcBSP;

/// BSP PVS visibility manager for GoldSrc maps.
///
/// Manages observers (viewpoints with cached PVS) and tracked objects
/// (entities with AABB-derived leaf sets, or pre-baked leaf sets for
/// static geometry groups).
///
/// All handles are integers returned by the register/add methods.
/// Handles are stable until the corresponding unregister/remove call.
///
/// Usage:
///   1. Call setup() with the BSP path and scale factor.
///   2. Add observers (local camera, remote peers).
///   3. Register entities and leaf sets.
///   4. Each tick: call update_observer_position() and update_entity_aabb()
///      as needed. Query is_entity_visible_to_observer() for culling.
///   5. Call teardown() when done (e.g. on map unload).
class VisibilityManager : public godot::Node {
	GDCLASS(VisibilityManager, godot::Node)

protected:
	static void _bind_methods();

public:
	// Load BSP for query-only (no mesh build, no WADs needed).
	// Returns leaf count on success, 0 on failure.
	int setup(godot::String bsp_path, float scale_factor);
	int setup_from_data(godot::PackedByteArray pvs_blob, float scale_factor);
	void teardown();
	bool is_ready() const;
	int get_leaf_count() const;

	// --- Observers ---
	// An observer is a viewpoint (local camera or remote peer).
	// Its PVS is recomputed only when it crosses a BSP leaf boundary.
	// peer_id=0 is conventional for the local camera.
	// Returns observer handle; -1 on failure.
	int add_observer(int peer_id, godot::Vector3 pos);
	void remove_observer(int handle);
	void update_observer_position(int handle, godot::Vector3 pos);
	int get_observer_leaf(int handle) const;

	// Point/leaf visibility queries against an observer's PVS.
	bool is_leaf_visible_to_observer(int leaf, int observer_handle) const;
	bool is_position_visible_to_observer(godot::Vector3 pos, int observer_handle) const;

	// --- Entities ---
	// An entity has a world-space AABB. The leaf set is derived by pushing
	// the AABB through the BSP tree. Call update_entity_aabb() whenever
	// the entity moves.
	// Returns entity handle; -1 on failure.
	int register_entity(godot::AABB godot_aabb);
	void unregister_entity(int handle);
	void update_entity_aabb(int handle, godot::AABB godot_aabb);
	bool is_entity_visible_to_observer(int entity_handle, int observer_handle) const;

	// --- Leaf sets ---
	// For pre-baked leaf arrays (e.g. worldspawn spatial groups stamped
	// with pvs_leaves by the BSP builder). The set never changes.
	// Returns leaf set handle; -1 on failure.
	int register_leaf_set(godot::PackedInt32Array leaves);
	void unregister_leaf_set(int handle);
	bool is_leaf_set_visible_to_observer(int set_handle, int observer_handle) const;

private:
	struct Observer {
		int peer_id = 0;
		int leaf = -1;
		std::vector<bool> pvs;  // indexed by BSP leaf index; O(1) lookup
		bool valid = false;
	};

	struct Entity {
		std::vector<int> leaves;
		bool valid = false;
	};

	struct LeafSet {
		std::vector<int> leaves;
		bool valid = false;
	};

	GoldSrcBSP *bsp = nullptr;

	std::vector<Observer> observers;
	std::vector<int> observer_free;
	std::vector<Entity> entities;
	std::vector<int> entity_free;
	std::vector<LeafSet> leaf_sets;
	std::vector<int> leaf_set_free;

	void _update_observer_pvs(Observer &obs, int new_leaf);
	std::vector<int> _aabb_to_leaves(godot::AABB aabb) const;

	template<typename T>
	static int _alloc(std::vector<T> &pool, std::vector<int> &free_list);

	bool _is_leaf_set_visible(const std::vector<int> &leaves, int observer_handle) const;
};
