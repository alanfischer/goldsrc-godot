#include "zone_map.h"

#include <godot_cpp/core/class_db.hpp>

using namespace godot;

void ZoneMap::_bind_methods() {
	ClassDB::bind_method(D_METHOD("get_zone_count"), &ZoneMap::get_zone_count);
	ClassDB::bind_method(D_METHOD("get_zone", "position"), &ZoneMap::get_zone);
	ClassDB::bind_method(D_METHOD("get_visible_zones", "zone"), &ZoneMap::get_visible_zones);
	ClassDB::bind_method(D_METHOD("are_zones_visible", "zone_a", "zone_b"), &ZoneMap::are_zones_visible);
}

int ZoneMap::get_zone_count() const {
	return 0;
}

int ZoneMap::get_zone(Vector3 /*position*/) const {
	return -1;
}

PackedInt32Array ZoneMap::get_visible_zones(int /*zone*/) const {
	return PackedInt32Array();
}

bool ZoneMap::are_zones_visible(int zone_a, int zone_b) const {
	PackedInt32Array visible = get_visible_zones(zone_a);
	for (int i = 0; i < visible.size(); i++) {
		if (visible[i] == zone_b) return true;
	}
	return false;
}
