#include "visibility_manager.h"
#include "goldsrc_bsp.h"

#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/variant/utility_functions.hpp>

using namespace godot;

void VisibilityManager::_bind_methods() {
	ClassDB::bind_method(D_METHOD("setup", "bsp_path", "scale_factor"), &VisibilityManager::setup);
	ClassDB::bind_method(D_METHOD("setup_from_data", "pvs_blob", "scale_factor"), &VisibilityManager::setup_from_data);
	ClassDB::bind_method(D_METHOD("teardown"), &VisibilityManager::teardown);
	ClassDB::bind_method(D_METHOD("is_ready"), &VisibilityManager::is_ready);
	ClassDB::bind_method(D_METHOD("get_leaf_count"), &VisibilityManager::get_leaf_count);

	ClassDB::bind_method(D_METHOD("add_observer", "peer_id", "pos"), &VisibilityManager::add_observer);
	ClassDB::bind_method(D_METHOD("remove_observer", "handle"), &VisibilityManager::remove_observer);
	ClassDB::bind_method(D_METHOD("update_observer_position", "handle", "pos"), &VisibilityManager::update_observer_position);
	ClassDB::bind_method(D_METHOD("get_observer_leaf", "handle"), &VisibilityManager::get_observer_leaf);
	ClassDB::bind_method(D_METHOD("is_leaf_visible_to_observer", "leaf", "observer_handle"), &VisibilityManager::is_leaf_visible_to_observer);
	ClassDB::bind_method(D_METHOD("is_position_visible_to_observer", "pos", "observer_handle"), &VisibilityManager::is_position_visible_to_observer);

	ClassDB::bind_method(D_METHOD("register_entity", "aabb"), &VisibilityManager::register_entity);
	ClassDB::bind_method(D_METHOD("unregister_entity", "handle"), &VisibilityManager::unregister_entity);
	ClassDB::bind_method(D_METHOD("update_entity_aabb", "handle", "aabb"), &VisibilityManager::update_entity_aabb);
	ClassDB::bind_method(D_METHOD("is_entity_visible_to_observer", "entity_handle", "observer_handle"), &VisibilityManager::is_entity_visible_to_observer);

	ClassDB::bind_method(D_METHOD("register_leaf_set", "leaves"), &VisibilityManager::register_leaf_set);
	ClassDB::bind_method(D_METHOD("unregister_leaf_set", "handle"), &VisibilityManager::unregister_leaf_set);
	ClassDB::bind_method(D_METHOD("is_leaf_set_visible_to_observer", "set_handle", "observer_handle"), &VisibilityManager::is_leaf_set_visible_to_observer);
}


// --- Setup ---

int VisibilityManager::setup(String bsp_path, float scale_factor) {
	teardown();
	bsp = memnew(GoldSrcBSP);
	bsp->set_scale_factor(scale_factor);
	if (bsp->load_bsp(bsp_path) != Error::OK) {
		memdelete(bsp);
		bsp = nullptr;
		return 0;
	}
	return bsp->get_leaf_count();
}

int VisibilityManager::setup_from_data(PackedByteArray pvs_blob, float scale_factor) {
	teardown();
	bsp = memnew(GoldSrcBSP);
	bsp->set_scale_factor(scale_factor);
	if (bsp->load_bsp_from_data(pvs_blob) != Error::OK) {
		memdelete(bsp);
		bsp = nullptr;
		return 0;
	}
	return bsp->get_leaf_count();
}

void VisibilityManager::teardown() {
	if (bsp) {
		memdelete(bsp);
		bsp = nullptr;
	}
	observers.clear();
	observer_free.clear();
	entities.clear();
	entity_free.clear();
	leaf_sets.clear();
	leaf_set_free.clear();
}

bool VisibilityManager::is_ready() const {
	return bsp != nullptr;
}

int VisibilityManager::get_leaf_count() const {
	return bsp ? bsp->get_leaf_count() : 0;
}


// --- Handle allocation ---

template<typename T>
int VisibilityManager::_alloc(std::vector<T> &pool, std::vector<int> &free_list) {
	if (!free_list.empty()) {
		int idx = free_list.back();
		free_list.pop_back();
		pool[idx] = T{};
		return idx;
	}
	pool.push_back({});
	return (int)pool.size() - 1;
}


// --- Observers ---

void VisibilityManager::_update_observer_pvs(Observer &obs, int new_leaf) {
	if (new_leaf == obs.leaf) return;
	obs.leaf = new_leaf;
	int n = bsp->get_leaf_count();
	obs.pvs.assign(n, false);
	if (new_leaf >= 0) {
		PackedInt32Array arr = bsp->get_leaf_pvs(new_leaf);
		for (int i = 0; i < arr.size(); i++) {
			int l = arr[i];
			if (l >= 0 && l < n)
				obs.pvs[l] = true;
		}
	}
}

int VisibilityManager::add_observer(int peer_id, Vector3 pos) {
	int idx = _alloc(observers, observer_free);
	Observer &obs = observers[idx];
	obs.peer_id = peer_id;
	obs.valid = true;
	if (bsp) {
		obs.pvs.assign(bsp->get_leaf_count(), false);
		_update_observer_pvs(obs, bsp->point_to_leaf(pos));
	}
	return idx;
}

void VisibilityManager::remove_observer(int handle) {
	if (handle < 0 || handle >= (int)observers.size()) return;
	observers[handle] = Observer{};
	observer_free.push_back(handle);
}

void VisibilityManager::update_observer_position(int handle, Vector3 pos) {
	if (handle < 0 || handle >= (int)observers.size()) return;
	Observer &obs = observers[handle];
	if (!obs.valid || !bsp) return;
	_update_observer_pvs(obs, bsp->point_to_leaf(pos));
}

int VisibilityManager::get_observer_leaf(int handle) const {
	if (handle < 0 || handle >= (int)observers.size()) return -1;
	return observers[handle].leaf;
}

bool VisibilityManager::is_leaf_visible_to_observer(int leaf, int observer_handle) const {
	if (!bsp) return true;
	if (observer_handle < 0 || observer_handle >= (int)observers.size()) return true;
	const Observer &obs = observers[observer_handle];
	if (!obs.valid || obs.leaf < 0) return true;
	if (leaf < 0 || leaf >= (int)obs.pvs.size()) return false;
	return obs.pvs[leaf];
}

bool VisibilityManager::is_position_visible_to_observer(Vector3 pos, int observer_handle) const {
	if (!bsp) return true;
	return is_leaf_visible_to_observer(bsp->point_to_leaf(pos), observer_handle);
}


// --- Entities ---

std::vector<int> VisibilityManager::_aabb_to_leaves(AABB aabb) const {
	if (!bsp) return {};
	PackedInt32Array arr = bsp->get_leaves_in_aabb(aabb);
	std::vector<int> result(arr.size());
	for (int i = 0; i < arr.size(); i++)
		result[i] = arr[i];
	return result;
}

int VisibilityManager::register_entity(AABB aabb) {
	int idx = _alloc(entities, entity_free);
	Entity &e = entities[idx];
	e.valid = true;
	e.leaves = _aabb_to_leaves(aabb);
	return idx;
}

void VisibilityManager::unregister_entity(int handle) {
	if (handle < 0 || handle >= (int)entities.size()) return;
	entities[handle] = Entity{};
	entity_free.push_back(handle);
}

void VisibilityManager::update_entity_aabb(int handle, AABB aabb) {
	if (handle < 0 || handle >= (int)entities.size()) return;
	Entity &e = entities[handle];
	if (!e.valid) return;
	e.leaves = _aabb_to_leaves(aabb);
}

bool VisibilityManager::is_entity_visible_to_observer(int entity_handle, int observer_handle) const {
	if (!bsp) return true;
	if (entity_handle < 0 || entity_handle >= (int)entities.size()) return true;
	const Entity &e = entities[entity_handle];
	if (!e.valid) return false;
	return _is_leaf_set_visible(e.leaves, observer_handle);
}


// --- Leaf sets ---

int VisibilityManager::register_leaf_set(PackedInt32Array leaves) {
	int idx = _alloc(leaf_sets, leaf_set_free);
	LeafSet &ls = leaf_sets[idx];
	ls.valid = true;
	ls.leaves.resize(leaves.size());
	for (int i = 0; i < leaves.size(); i++)
		ls.leaves[i] = leaves[i];
	return idx;
}

void VisibilityManager::unregister_leaf_set(int handle) {
	if (handle < 0 || handle >= (int)leaf_sets.size()) return;
	leaf_sets[handle] = LeafSet{};
	leaf_set_free.push_back(handle);
}

bool VisibilityManager::is_leaf_set_visible_to_observer(int set_handle, int observer_handle) const {
	if (!bsp) return true;
	if (set_handle < 0 || set_handle >= (int)leaf_sets.size()) return true;
	const LeafSet &ls = leaf_sets[set_handle];
	if (!ls.valid) return false;
	return _is_leaf_set_visible(ls.leaves, observer_handle);
}


// --- Shared leaf-set visibility check ---

bool VisibilityManager::_is_leaf_set_visible(const std::vector<int> &leaves, int observer_handle) const {
	if (leaves.empty()) return true;  // no leaves computed → show safely
	if (observer_handle < 0 || observer_handle >= (int)observers.size()) return true;
	const Observer &obs = observers[observer_handle];
	if (!obs.valid || obs.leaf < 0) return true;
	int n = (int)obs.pvs.size();
	for (int leaf : leaves) {
		if (leaf >= 0 && leaf < n && obs.pvs[leaf])
			return true;
	}
	return false;
}
