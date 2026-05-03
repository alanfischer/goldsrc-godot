#pragma once

#include <godot_cpp/classes/node.hpp>
#include <godot_cpp/classes/node3d.hpp>
#include <godot_cpp/variant/packed_int32_array.hpp>
#include <godot_cpp/variant/variant.hpp>
#include <godot_cpp/variant/vector3.hpp>
#include <vector>
#include "zone_map.h"

class ZoneManager : public godot::Node {
	GDCLASS(ZoneManager, godot::Node)

protected:
	static void _bind_methods();

public:
	void set_zone_map(const godot::Ref<ZoneMap> &map);
	godot::Ref<ZoneMap> get_zone_map() const;

	void register_observer(godot::Variant id, godot::Node3D *node);
	void unregister_observer(godot::Variant id);
	int get_observer_zone(godot::Variant id) const;

	int get_zone(godot::Vector3 position) const;
	godot::PackedInt32Array get_visible_zones(int zone) const;
	bool is_visible(godot::Vector3 from, godot::Vector3 to) const;

	void set_network_pvs_expand(int expand);
	int get_network_pvs_expand() const;
	void set_render_pvs_expand(int expand);
	int get_render_pvs_expand() const;

	void _physics_process(double delta) override;

private:
	godot::Ref<ZoneMap> zone_map;
	int network_pvs_expand = 0;
	int render_pvs_expand = 0;

	struct Observer {
		godot::Variant id;
		godot::Node3D *node = nullptr;
		int last_zone = -2; // -2 = uninitialized; forces zone_changed on first tick
	};
	std::vector<Observer> observers;
};
