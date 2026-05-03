#pragma once

#include <godot_cpp/classes/resource.hpp>
#include <godot_cpp/variant/vector3.hpp>
#include <godot_cpp/variant/packed_int32_array.hpp>

class ZoneMap : public godot::Resource {
	GDCLASS(ZoneMap, godot::Resource)

protected:
	static void _bind_methods();

public:
	virtual int get_zone_count() const;
	virtual int get_zone(godot::Vector3 position) const;
	virtual godot::PackedInt32Array get_visible_zones(int zone) const;
	virtual bool are_zones_visible(int zone_a, int zone_b) const;
};
