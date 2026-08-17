"""Build the belief + occupancy octomap of the car doors from the captured clouds.

One-shot pipeline, not a node graph:

    1. doors_parts_prep  (Python)  -- dice the two door STLs into patch PCDs, the
                                      belief-map ground truth. Cached; a no-op unless
                                      a source STL or the patch grid changed.
    2. build_doors_octomap (C++)   -- belief map from those patches, then stamp every
                                      captured cloud onto it and print the per-patch /
                                      per-door coverage report.

Run it AFTER an inspection run has written clouds:

    ros2 launch doors_inspection doors_octomap.launch.py                 # sim data
    ros2 launch doors_inspection doors_octomap.launch.py mode:=real
    ros2 launch doors_inspection doors_octomap.launch.py res:=0.01 patches_y:=4
    ros2 launch doors_inspection doors_octomap.launch.py mode:=real snap:=true \
        snap_radius:=2 cloud_fix:=-0.740,-5.128,0.347,0.109,0.002,0.061

Use snap on REAL data. The builder credits a voxel only on an exact hit, and the
real clouds sit a measured 38 mm (median) off the model against a 20 mm voxel, so
without snap the report reads far below what the cameras actually captured -- the
6 Aug 2026 run scored 38.1% against sim's 86.2% while having MORE distinct points
at the door bottoms than sim did. See build_doors_octomap.cpp's header for the
root cause; snap is a reporting compensation, not a fix.

The two .ot files land next to the clouds they were built from, so a sim map and a
real map never overwrite each other:
    <data_dir>/beliefMap_doors_<mode>.ot
    <data_dir>/occupancyMap_doors_<mode>.ot
Inspect them with:  octovis <file>.ot
"""
import os

from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration

from doors_inspection.door_parts import (DEFAULT_DENSITY, DEFAULT_PARTS_DIR,
                                         DEFAULT_PATCHES_Y, DEFAULT_PATCHES_Z,
                                         ensure_parts)

# Where the doors executors write their captures (doors_inspection.launch.py's
# output_base_dir). <mode> picks the sim or the real subtree.
DEFAULT_DATA_ROOT = os.path.expanduser('~/colcon_ws/src/pcds/doors')

# Measured door misalignment, "roll,pitch,yaw,x,y,z" (deg, m) -- pass as cloud_fix
# on REAL data only. Fitted 7 Aug 2026 by point-to-plane ICP against the belief
# mesh over all 12 real captures (183 740 points, 20% trimmed): the doors lean
# 5.1 deg about world Y. Both doors fitted separately give the same angle (-5.04
# and -4.80), and fitting them separately does not beat this single global
# transform (median residual 18.5 / 17.9 vs 18.6 mm), so one transform is used.
# Median distance to the model: 41.5 mm -> 18.6 mm.
DOORS_FIT = '-0.740,-5.128,0.347,0.109,0.002,0.061'


def _bool(context, name):
    return LaunchConfiguration(name).perform(context).lower() in ('true', '1', 'yes')


def _launch_setup(context, *args, **kwargs):
    def cfg(name):
        return LaunchConfiguration(name).perform(context)

    # Belief-map input first, and synchronously: the builder must not start before the
    # patch PCDs exist, and doing it here rather than as a second process removes any
    # chance of a race.
    parts_dir = ensure_parts(
        parts_dir=cfg('parts_dir'),
        density=int(cfg('density')),
        patches_y=int(cfg('patches_y')),
        patches_z=int(cfg('patches_z')),
        force=_bool(context, 'force_parts'),
    )

    mode = cfg('mode')
    data_dir = cfg('data_dir') or os.path.join(DEFAULT_DATA_ROOT, f'{mode}_pcds')

    exe = os.path.join(get_package_prefix('doors_inspection'), 'lib',
                       'doors_inspection', 'build_doors_octomap')

    return [ExecuteProcess(
        cmd=[exe,
             '--mode', mode,
             '--parts_dir', parts_dir,
             '--data_dir', data_dir,
             # 1.0 because the door STLs are metres. The chassis builder divides by
             # 1000 for its millimetre CAD parts; passing that here would shrink the
             # doors to a 1.2 mm speck and the coverage report would read 0%.
             '--parts_scale', '1.0',
             '--res', cfg('res'),
             '--z_min', cfg('z_min'),
             '--use_ur', cfg('use_ur'),
             '--use_kawasaki', cfg('use_kawasaki'),
             '--snap', cfg('snap'),
             '--snap_radius', cfg('snap_radius'),
             '--cloud_fix', cfg('cloud_fix')],
        output='screen',
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'mode', default_value='sim',
            description="sim | real -- which capture subtree to build the map from."),
        DeclareLaunchArgument(
            'data_dir', default_value='',
            description='Override the capture directory; empty means '
                        '~/colcon_ws/src/pcds/doors/<mode>_pcds.'),
        DeclareLaunchArgument(
            'parts_dir', default_value=DEFAULT_PARTS_DIR,
            description='Cache of the belief-map patch PCDs (a build artefact).'),
        DeclareLaunchArgument(
            'density', default_value=str(DEFAULT_DENSITY),
            description='Belief-map surface samples per square metre.'),
        DeclareLaunchArgument(
            'patches_y', default_value=str(DEFAULT_PATCHES_Y),
            description='Patches across each door. 1 = one part per door.'),
        DeclareLaunchArgument(
            'patches_z', default_value=str(DEFAULT_PATCHES_Z),
            description='Patches up each door. 1 = one part per door.'),
        DeclareLaunchArgument(
            'force_parts', default_value='false',
            description='Re-sample the patch PCDs even if the cache looks current.'),
        DeclareLaunchArgument(
            'res', default_value='0.02',
            description='Octomap voxel size (m). Same as the chassis job.'),
        DeclareLaunchArgument(
            'z_min', default_value='-10.0',
            description='Drop captured points below this world Z.'),
        DeclareLaunchArgument(
            'use_ur', default_value='true', description='Include the UR captures.'),
        DeclareLaunchArgument(
            'use_kawasaki', default_value='true',
            description='Include the Kawasaki captures.'),
        DeclareLaunchArgument(
            'snap', default_value='false',
            description='true -> a captured point that misses the door by up to '
                        'snap_radius voxels still counts (nearest belief voxel wins). '
                        'Rescues real-data misalignment, but flatters the coverage '
                        'number, so it is off by default and the sim numbers stay '
                        'comparable with the chassis job.'),
        DeclareLaunchArgument(
            'snap_radius', default_value='1',
            description='Snap search radius in VOXELS (metres = snap_radius * res). '
                        '1 keeps the old 26-neighbour behaviour. Use 2 for real data: '
                        'the measured door misalignment is ~8 cm at the bottom and '
                        '1-2 cm at mid height, so +-2 cm never rescued the bottom. '
                        'Measured on the 6 Aug 2026 run at res 0.02 -- radius 0/1/2/4 '
                        'gives real 38.1/48.1/55.0/58.9%, sim 86.2/87.4/87.4/87.5%: '
                        'the gain saturates at 2 and sim barely moves, so this closes '
                        'the reporting gap without inflating the sim baseline.'),
        DeclareLaunchArgument(
            'cloud_fix', default_value='0,0,0,0,0,0',
            description='Rigid correction "roll,pitch,yaw,x,y,z" (deg, m, URDF ZYX '
                        'order) applied to the CAPTURED clouds before voxelising; the '
                        'belief map is untouched so the coverage denominator stays '
                        'fixed. All zeros = off. REAL DATA ONLY -- fitted by '
                        'point-to-plane ICP on the 6 Aug 2026 run, it drops sim from '
                        '86.2% to 28.1% because sim is already aligned. '
                        'Use DOORS_FIT (see module docstring) for the measured value.'),
        OpaqueFunction(function=_launch_setup),
    ])
