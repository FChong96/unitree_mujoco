#!/usr/bin/env python3
"""Convert the bundled A5-D description to explicit MJCF."""
import hashlib
import json
from pathlib import Path
import re
import xml.etree.ElementTree as ET

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
OUT = ROOT / 'unitree_robots/r1_a5_d'
DESCRIPTION = OUT / 'source'
SOURCE = DESCRIPTION / 'r1_a5_d.urdf.xacro'
MESH_DIRS = {folder: OUT / 'meshes' / folder for folder in ('a5', 'r1d', 'dex1_1')}


def origin(element):
    o = element.find('origin')
    return {'pos': o.get('xyz', '0 0 0'), 'euler': o.get('rpy', '0 0 0')} if o is not None else {}


def resolved_source():
    """This input only uses xacro:arg and $(arg ...); reject richer Xacro."""
    raw = SOURCE.read_text()
    tree = ET.fromstring(raw)
    defaults = {a.get('name'): a.get('default') for a in tree.findall('{http://www.ros.org/wiki/xacro}arg')}
    raw = re.sub(r'\$\(arg (\w+)\)', lambda m: defaults[m[1]], raw)
    tree = ET.fromstring(raw)
    for item in list(tree):
        if item.tag not in ('link', 'joint'):
            if item.tag not in ('material', 'ros2_control', '{http://www.ros.org/wiki/xacro}arg'):
                raise ValueError(f'Unsupported source element: {item.tag}')
            tree.remove(item)
    if '$' in ET.tostring(tree, encoding='unicode'):
        raise ValueError('Unsupported Xacro expression')
    return tree, defaults


def build(mobile=False):
    source, defaults = resolved_source()
    colors = {m.get('name'): m.find('color').get('rgba')
              for m in ET.parse(SOURCE).getroot().findall('material')}
    links = {x.get('name'): x for x in source.findall('link')}
    children = {}
    for joint in source.findall('joint'):
        children.setdefault(joint.find('parent').get('link'), []).append(joint)
    mj = ET.Element('mujoco', model='r1_a5_d_dex1_1')
    ET.SubElement(mj, 'compiler', angle='radian', eulerseq='XYZ', autolimits='true', fusestatic='false')
    ET.SubElement(mj, 'option', timestep='0.002', integrator='implicitfast', solver='Newton')
    ET.SubElement(mj, 'statistic', center='0 0 0.85', extent='1.7')
    visual = ET.SubElement(mj, 'visual')
    ET.SubElement(visual, 'global', azimuth='135', elevation='-15', offwidth='960', offheight='720')
    default = ET.SubElement(mj, 'default')
    ET.SubElement(default, 'joint', damping='0.2', armature='0.01')
    ET.SubElement(default, 'geom', friction='0.8 0.01 0.001', solref='0.01 1')
    asset = ET.SubElement(mj, 'asset')
    ET.SubElement(asset, 'texture', name='grid', type='2d', builtin='checker',
                  rgb1='0.2 0.23 0.26', rgb2='0.3 0.33 0.36', width='512', height='512')
    ET.SubElement(asset, 'material', name='floor', texture='grid', texrepeat='6 6', texuniform='true')
    world = ET.SubElement(mj, 'worldbody')
    ET.SubElement(world, 'light', pos='0 -2 3', dir='0 0 -1', directional='true')
    ET.SubElement(world, 'geom', name='floor', type='plane', size='0 0 0.1', material='floor',
                  contype='1', conaffinity='4')
    actuator = ET.SubElement(mj, 'actuator')
    equality = ET.SubElement(mj, 'equality')
    meshes = {}
    movable = []
    efforts = {}

    def body(parent, link_name, pose, joint=None):
        b = ET.SubElement(parent, 'body', name=link_name, **pose)
        link = links[link_name]
        inertial = link.find('inertial')
        if inertial is not None:
            inertia = inertial.find('inertia')
            ipose = origin(inertial)
            # fullinertia is expressed in the link frame; current source uses zero RPY.
            if any(float(x) != 0 for x in ipose.pop('euler', '0 0 0').split()):
                raise ValueError(f'Nonzero inertial RPY: {link_name}')
            ET.SubElement(b, 'inertial', **ipose, mass=inertial.find('mass').get('value'),
                          fullinertia=' '.join(inertia.get(x) for x in ('ixx', 'iyy', 'izz', 'ixy', 'ixz', 'iyz')))
        if mobile and link_name == 'base_link':
            ET.SubElement(b, 'freejoint', name='floating_base')
        if joint is not None and joint.get('type') != 'fixed':
            name = joint.get('name')
            movable.append(name)
            limit = joint.find('limit')
            attrs = dict(name=name, type='slide' if joint.get('type') == 'prismatic' else 'hinge',
                         axis=joint.find('axis').get('xyz'))
            if limit is not None and 'lower' in limit.attrib:
                attrs['range'] = f'{limit.get("lower")} {limit.get("upper")}'
            else:
                attrs['limited'] = 'false'
            ET.SubElement(b, 'joint', **attrs)
            if 'caster' not in name:
                effort = float(limit.get('effort'))
                efforts[name] = effort
                ET.SubElement(actuator, 'general', name=name, joint=name, biastype='affine',
                              gainprm='1', biasprm='0 0 0', forcerange=f'{-effort} {effort}', ctrllimited='false')
            mimic = joint.find('mimic')
            if mimic is not None:
                ET.SubElement(equality, 'joint', joint1=name, joint2=mimic.get('joint'),
                              polycoef='0 1 0 0 0', solref='0.002 1')
        for v in link.findall('visual'):
            mesh = v.find('geometry/mesh')
            if mesh is None:
                raise ValueError(f'Unsupported visual on {link_name}')
            path = Path(mesh.get('filename').split('/meshes/', 1)[1])
            filename = MESH_DIRS[path.parts[0]] / path.name
            key = path.as_posix().replace('/', '_').replace('.', '_')
            if key not in meshes:
                meshes[key] = filename
                ET.SubElement(asset, 'mesh', name=key, file='meshes/' + path.as_posix(),
                              scale=mesh.get('scale', '1 1 1'))
            color = v.find('material/color')
            material = v.find('material')
            ET.SubElement(b, 'geom', type='mesh', mesh=key, **origin(v),
                          rgba=color.get('rgba') if color is not None else colors.get(material.get('name') if material is not None else '', '0.7 0.7 0.7 1'),
                          contype='0', conaffinity='0', group='1', density='0')
        # Collision approximations are separate from the high resolution visual meshes.
        if link_name in ('Left_Wheel_link', 'Right_Wheel_link'):
            ET.SubElement(b, 'geom', name=link_name + '_contact', type='cylinder', size='0.065 0.024',
                          euler='1.57079632679 0 0', contype='4', conaffinity='1', group='3')
        if 'caster_Roll_link' in link_name:
            ET.SubElement(b, 'geom', name=link_name + '_contact', type='sphere', size='0.025',
                          contype='4', conaffinity='1', group='3')
        if '_dex1_finger_link_' in link_name:
            first = link_name.endswith('_1')
            ET.SubElement(b, 'geom', name=link_name + '_pad', type='box', size='0.012 0.0025 0.009',
                          pos=f'0.065 {-.0264 if first else .0264} {-.010658 if first else .010658}',
                          contype='2', conaffinity='8', group='3', rgba='0.15 0.15 0.15 1')
        if link_name.endswith('_tcp'):
            ET.SubElement(b, 'site', name=link_name, size='0.006', rgba='0.9 0.3 0.1 1')
        for child in children.get(link_name, []):
            body(b, child.find('child').get('link'), origin(child), child)

    body(world, 'base_link', {'pos': '0 0 0.16'})
    base = next(x for x in world.findall('body') if x.get('name') == 'base_link')
    ET.SubElement(base, 'site', name='imu', size='0.005', rgba='0 0 1 1')
    sensor = ET.SubElement(mj, 'sensor')
    for tag, name in [('framequat', 'imu_quat'), ('gyro', 'imu_gyro'), ('accelerometer', 'imu_acc')]:
        attrs = {'objtype': 'site', 'objname': 'imu'} if tag == 'framequat' else {'site': 'imu'}
        ET.SubElement(sensor, tag, name=name, **attrs)
    qpos = ([0, 0, .16, 1, 0, 0, 0] if mobile else [])
    qpos += [.25 if x.startswith('lift_') else .005 if '_dex1_finger_' in x else 0 for x in movable]
    ET.SubElement(ET.SubElement(mj, 'keyframe'), 'key', name='home', qpos=' '.join(map(str, qpos)))
    ET.indent(mj)
    OUT.mkdir(parents=True, exist_ok=True)
    for filename in meshes.values():
        if not filename.is_file():
            raise FileNotFoundError(filename)
    path = OUT / ('scene_mobile.xml' if mobile else 'scene.xml')
    ET.ElementTree(mj).write(path, encoding='utf-8', xml_declaration=True)
    return path, efforts, meshes, defaults


def main():
    fixed, efforts, meshes, defaults = build()
    build(mobile=True)
    assembly = json.loads((DESCRIPTION / 'assembly_sources.json').read_text())
    metadata = {
        'model': 'r1_a5_d_with_dual_dex1_1', 'kind': 'local_uncalibrated_component_assembly',
        'source': str(SOURCE.relative_to(ROOT)), 'source_sha256': hashlib.sha256(SOURCE.read_bytes()).hexdigest(),
        'component_sources': assembly, 'xacro_defaults': defaults, 'effort_limits': efforts,
        'mesh_sha256': {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest() for p in meshes.values()},
        'conversion': 'build_model.py: explicit link frames, full inertias, URDF axes and position limits',
        'assumptions': [
            'Default chassis fixed; scene_mobile.xml enables a free chassis.',
            'Lift stages coupled 1:1, initial extension 0.25 m each.',
            'Dex fingers coupled 1:1, stroke 0..0.041 m; raw 0..5.4 maps linearly (uncalibrated).',
            'Wheel/caster and fingertip contacts are approximate primitives; other visuals have no collision.',
            'No calibrated friction, backlash, delays, camera images, LiDAR or high level services.',
            'Motor control ranges use URDF effort; URDF velocity limits are not simulated drive speed limits.'
        ]}
    (OUT / 'model_sources.json').write_text(json.dumps(metadata, indent=2) + '\n')
    print(f'Generated {fixed.relative_to(ROOT)} and scene_mobile.xml')


if __name__ == '__main__':
    main()
