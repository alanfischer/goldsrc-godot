#include "register_types.h"

#include <gdextension_interface.h>
#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/core/defs.hpp>
#include <godot_cpp/godot.hpp>

#include "nodes/goldsrc_wad.h"
#include "nodes/goldsrc_spr.h"
#include "nodes/goldsrc_bsp.h"
#include "nodes/goldsrc_mdl.h"

using namespace godot;

void initialize_goldsrc_module(ModuleInitializationLevel p_level) {
	if (p_level != MODULE_INITIALIZATION_LEVEL_SCENE) {
		return;
	}

	GDREGISTER_CLASS(GoldSrcWAD);
	GDREGISTER_CLASS(GoldSrcSPR);
	GDREGISTER_CLASS(GoldSrcBSP);
	GDREGISTER_CLASS(GoldSrcMDL);
}

void uninitialize_goldsrc_module(ModuleInitializationLevel p_level) {
	if (p_level != MODULE_INITIALIZATION_LEVEL_SCENE) {
		return;
	}
}

extern "C" {
GDExtensionBool GDE_EXPORT goldsrc_library_init(
		GDExtensionInterfaceGetProcAddress p_get_proc_address,
		GDExtensionClassLibraryPtr p_library,
		GDExtensionInitialization *r_initialization) {
	GDExtensionBinding::InitObject init_obj(p_get_proc_address, p_library, r_initialization);

	init_obj.register_initializer(initialize_goldsrc_module);
	init_obj.register_terminator(uninitialize_goldsrc_module);
	init_obj.set_minimum_library_initialization_level(MODULE_INITIALIZATION_LEVEL_SCENE);

	return init_obj.init();
}
}
