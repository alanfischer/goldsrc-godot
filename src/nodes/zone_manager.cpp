#include "zone_manager.h"

#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/variant/utility_functions.hpp>

using namespace godot;

void ZoneManager::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_zone_map", "map"), &ZoneManager::set_zone_map);
	ClassDB::bind_method(D_METHOD("get_zone_map"), &ZoneManager::get_zone_map);
	ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "zone_map", PROPERTY_HINT_RESOURCE_TYPE, "ZoneMap"),
			"set_zone_map", "get_zone_map");

	ClassDB::bind_method(D_METHOD("register_observer", "id", "node"), &ZoneManager::register_observer);
	ClassDB::bind_method(D_METHOD("unregister_observer", "id"), &ZoneManager::unregister_observer);
	ClassDB::bind_method(D_METHOD("get_observer_zone", "id"), &ZoneManager::get_observer_zone);

	ClassDB::bind_method(D_METHOD("get_zone", "position"), &ZoneManager::get_zone);
	ClassDB::bind_method(D_METHOD("get_visible_zones", "zone"), &ZoneManager::get_visible_zones);
	ClassDB::bind_method(D_METHOD("is_visible", "from", "to"), &ZoneManager::is_visible);

	ClassDB::bind_method(D_METHOD("set_network_pvs_expand", "expand"), &ZoneManager::set_network_pvs_expand);
	ClassDB::bind_method(D_METHOD("get_network_pvs_expand"), &ZoneManager::get_network_pvs_expand);
	ADD_PROPERTY(PropertyInfo(Variant::INT, "network_pvs_expand"), "set_network_pvs_expand", "get_network_pvs_expand");

	ClassDB::bind_method(D_METHOD("set_render_pvs_expand", "expand"), &ZoneManager::set_render_pvs_expand);
	ClassDB::bind_method(D_METHOD("get_render_pvs_expand"), &ZoneManager::get_render_pvs_expand);
	ADD_PROPERTY(PropertyInfo(Variant::INT, "render_pvs_expand"), "set_render_pvs_expand", "get_render_pvs_expand");

	ADD_SIGNAL(MethodInfo("zone_changed",
			PropertyInfo(Variant::NIL, "observer_id"),
			PropertyInfo(Variant::INT, "from_zone"),
			PropertyInfo(Variant::INT, "to_zone")));
}

void ZoneManager::set_zone_map(const Ref<ZoneMap> &map) {
	zone_map = map;
}

Ref<ZoneMap> ZoneManager::get_zone_map() const {
	return zone_map;
}

void ZoneManager::register_observer(Variant id, Node3D *node) {
	for (auto &obs : observers) {
		if (obs.id == id) {
			obs.node = node;
			obs.last_zone = -2;
			return;
		}
	}
	Observer obs;
	obs.id = id;
	obs.node = node;
	observers.push_back(obs);
}

void ZoneManager::unregister_observer(Variant id) {
	for (int i = (int)observers.size() - 1; i >= 0; i--) {
		if (observers[i].id == id) {
			observers.erase(observers.begin() + i);
		}
	}
}

int ZoneManager::get_observer_zone(Variant id) const {
	for (const auto &obs : observers) {
		if (obs.id == id) {
			return obs.last_zone < -1 ? -1 : obs.last_zone;
		}
	}
	return -1;
}

int ZoneManager::get_zone(Vector3 position) const {
	if (zone_map.is_null()) return -1;
	return zone_map->get_zone(position);
}

PackedInt32Array ZoneManager::get_visible_zones(int zone) const {
	if (zone_map.is_null()) return PackedInt32Array();
	return zone_map->get_visible_zones(zone);
}

bool ZoneManager::is_visible(Vector3 from, Vector3 to) const {
	if (zone_map.is_null()) return true;
	int zone_from = zone_map->get_zone(from);
	int zone_to = zone_map->get_zone(to);
	if (zone_from == -1 || zone_to == -1) return true;
	return zone_map->are_zones_visible(zone_from, zone_to);
}

void ZoneManager::_physics_process(double /*delta*/) {
	if (zone_map.is_null()) return;
	for (auto &obs : observers) {
		if (!obs.node || !obs.node->is_inside_tree()) continue;
		int zone = zone_map->get_zone(obs.node->get_global_position());
		if (zone != obs.last_zone) {
			int from = obs.last_zone == -2 ? -1 : obs.last_zone;
			emit_signal("zone_changed", obs.id, from, zone);
			obs.last_zone = zone;
		}
	}
}

void ZoneManager::set_network_pvs_expand(int expand) { network_pvs_expand = expand; }
int ZoneManager::get_network_pvs_expand() const { return network_pvs_expand; }
void ZoneManager::set_render_pvs_expand(int expand) { render_pvs_expand = expand; }
int ZoneManager::get_render_pvs_expand() const { return render_pvs_expand; }
