"""Builds the maps of the AirSim host project with Unreal's own tools:
/Game/Maps/Field (open ground, the recorded data's site) and /Game/Maps/Antwerp
(the same plus the city around Groenplaats).

Run by airsim/setup.sh (or by hand) through the editor's Python commandlet:

    LOCSIM_CITY_DIR=airsim/data/antwerp UnrealEditor-Cmd airsim/unreal/LocalizationSim.uproject \
        -run=pythonscript -script=airsim/maps/build_maps.py -unattended -nullrhi

Everything here is standard Unreal: the Interchange glTF importer brings in
the building tiles prepared by sim/tools/locsim_city.py (one .glb per city
district, metres from the scenario origin on Groenplaats), and the editor
scripting API places them with lights, sky and a ground slab, and saves
/Game/Maps/Antwerp. AirSim (external/Cosys-AirSim) spawns its vehicles at
the PlayerStart and reports GPS from OriginGeopoint in settings.json, which
must be the same point (51.2192 N, 4.4015 E).

Axes: Unreal's glTF importer puts glTF x (east) on Unreal X and glTF z
(south) on Unreal Y; AirSim wants Unreal X = north and Y = east (NED), so the
city is placed with a yaw of 90 degrees.
"""

import json
import os

import unreal

CITY_DIR = os.environ["LOCSIM_CITY_DIR"]
ASSETS = "/Game/Antwerp"
CITY_YAW = 90.0
GROUND_SIZE_M = 1000.0

levels = unreal.get_editor_subsystem(unreal.LevelEditorSubsystem)
actors = unreal.get_editor_subsystem(unreal.EditorActorSubsystem)


def new_map(path):
    """A fresh level (rebuilding replaces the previous one)."""
    if unreal.EditorAssetLibrary.does_asset_exist(path):
        unreal.EditorAssetLibrary.delete_asset(path)
    if not levels.new_level(path):
        raise RuntimeError("cannot create " + path)


def add_ground_and_sky():
    # Ground: a flat slab, top at z = 0 (the city model has buildings only).
    cube = unreal.load_asset("/Engine/BasicShapes/Cube")
    ground = actors.spawn_actor_from_object(cube, unreal.Vector(0, 0, -50), unreal.Rotator(0, 0, 0))
    ground.set_actor_label("ground")
    ground.set_actor_scale3d(unreal.Vector(GROUND_SIZE_M, GROUND_SIZE_M, 1.0))
    ground.static_mesh_component.set_material(0, unreal.load_asset("/Engine/BasicShapes/BasicShapeMaterial"))
    # Light and sky: engine actors, movable so AirSim's time of day and
    # weather can change them at runtime.
    sun = actors.spawn_actor_from_class(unreal.DirectionalLight, unreal.Vector(0, 0, 10000), unreal.Rotator(0, -40, 135))
    sun.set_actor_label("sun")
    sun.light_component.set_mobility(unreal.ComponentMobility.MOVABLE)
    sun.light_component.set_editor_property("atmosphere_sun_light", True)
    actors.spawn_actor_from_class(unreal.SkyAtmosphere, unreal.Vector(0, 0, 0), unreal.Rotator(0, 0, 0)).set_actor_label("sky")
    sky_light = actors.spawn_actor_from_class(unreal.SkyLight, unreal.Vector(0, 0, 500), unreal.Rotator(0, 0, 0))
    sky_light.set_actor_label("sky_light")
    sky_light.light_component.set_mobility(unreal.ComponentMobility.MOVABLE)
    sky_light.light_component.set_editor_property("real_time_capture", True)
    actors.spawn_actor_from_class(unreal.ExponentialHeightFog, unreal.Vector(0, 0, 0), unreal.Rotator(0, 0, 0)).set_actor_label("haze")
    # AirSim spawns its vehicles here, 1.5 m up (a vehicle starting inside the
    # ground slab would never collide with it).
    actors.spawn_actor_from_class(unreal.PlayerStart, unreal.Vector(0, 0, 150), unreal.Rotator(0, 0, 0)).set_actor_label("start")


def add_pole():
    # drone_localization's pole: a 2.9 m mast and the UWB anchor on top, 24 m
    # east of the origin, where the pole camera of settings/pole_*.json is
    # (Unreal: X north, Y east, cm; the engine's basic shapes are 1 m). For
    # looks only: no collision, so nothing flies into it by accident.
    for label, shape, z_cm, scale in (("pole_mast", "Cylinder", 145, (0.12, 0.12, 2.9)),
                                      ("pole_anchor", "Sphere", 312, (0.14, 0.14, 0.14))):
        actor = actors.spawn_actor_from_object(unreal.load_asset("/Engine/BasicShapes/" + shape),
                                               unreal.Vector(0, 2400, z_cm), unreal.Rotator(0, 0, 0))
        actor.set_actor_label(label)
        actor.set_actor_scale3d(unreal.Vector(*scale))
        actor.static_mesh_component.set_collision_profile_name("NoCollision")


def save(path):
    if not levels.save_current_level():
        raise RuntimeError("cannot save " + path)
    unreal.log_warning("LOCSIM map %s saved" % path)


# Field: open ground.
new_map("/Game/Maps/Field")
add_ground_and_sky()
add_pole()
save("/Game/Maps/Field")

# Antwerp: import the building tiles with Unreal's glTF importer.
new_map("/Game/Maps/Antwerp")
add_ground_and_sky()
add_pole()
manifest = json.load(open(os.path.join(CITY_DIR, "city.json")))
tasks = []
for name in manifest["files"]:
    task = unreal.AssetImportTask()
    task.filename = os.path.join(CITY_DIR, name)
    task.destination_path = ASSETS
    task.automated = True
    task.replace_existing = True
    task.save = True
    tasks.append(task)
unreal.AssetToolsHelpers.get_asset_tools().import_asset_tasks(tasks)
# Place them: the triangles are the collision (buildings are not boxes), so
# AirSim's vehicles collide with the real facades.
rotation = unreal.Rotator(roll=0.0, pitch=0.0, yaw=CITY_YAW)
for task in tasks:
    for path in task.imported_object_paths:
        mesh = unreal.load_asset(path)
        if not isinstance(mesh, unreal.StaticMesh):
            continue
        body = mesh.get_editor_property("body_setup")
        if body:
            body.set_editor_property("collision_trace_flag", unreal.CollisionTraceFlag.CTF_USE_COMPLEX_AS_SIMPLE)
        unreal.EditorAssetLibrary.save_loaded_asset(mesh)
        actor = actors.spawn_actor_from_object(mesh, unreal.Vector(0, 0, 0), rotation)
        actor.set_actor_label("city_" + mesh.get_name())
        actor.static_mesh_component.set_collision_profile_name("BlockAll")
save("/Game/Maps/Antwerp")

# Report what the vehicles will collide with.
for a in actors.get_all_level_actors():
    if isinstance(a, unreal.StaticMeshActor):
        c = a.static_mesh_component
        unreal.log_warning("LOCSIM %s collision %s profile %s" % (a.get_actor_label(), c.get_collision_enabled(), c.get_collision_profile_name()))
