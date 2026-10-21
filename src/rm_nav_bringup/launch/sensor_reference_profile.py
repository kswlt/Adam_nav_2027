"""Nav2 parameters for translation-only sensor-reference tracking."""
import math


def configure(params, version, share):
    controller = params['controller_server']['ros__parameters']
    controller['odom_topic'] = '/sensor_reference/odom'
    controller['goal_checker'].update({'yaw_goal_tolerance': 2*math.pi, 'stateful': False})
    controller['FollowPath'] = {
        'plugin': 'rm_nav_control::TdtOmniController', 'translation_only': True,
        'map_bundle_version': version, 'environment_version': version+'/costmap',
        'sensor_health_topic': '/state/sensor_reference_healthy', 'max_speed': .3,
        'max_acceleration': .6, 'max_jerk': 3., 'input_timeout': .25}
    # All edges of this circumscribed polygon cover a 0.54 m circular envelope.
    r = .54/math.cos(math.pi/32)
    polygon = [[r*math.cos(2*math.pi*i/32), r*math.sin(2*math.pi*i/32)] for i in range(32)]
    for key in ('local_costmap', 'global_costmap'):
        params[key][key]['ros__parameters'].update({'robot_base_frame': 'nav_reference', 'footprint': str(polygon)})
        inflation=params[key][key]['ros__parameters']['inflation_layer']
        inflation['inflation_radius']=max(float(inflation.get('inflation_radius',0)),.65)
    params['bt_navigator']['ros__parameters'].update({
        'robot_base_frame': 'nav_reference', 'odom_topic': '/sensor_reference/odom',
        'navigators': ['navigate_to_pose'],
        'default_nav_to_pose_bt_xml': str(share/'config/sensor_reference_navigate.xml')})
    params['behavior_server']['ros__parameters'].update({
        'robot_base_frame': 'nav_reference', 'behavior_plugins': ['wait']})
    params['velocity_smoother']['ros__parameters'].update({
        'odom_topic': '/sensor_reference/odom', 'max_velocity': [.3, .3, 0.], 'min_velocity': [-.3, -.3, 0.]})
    collision = params['collision_monitor']['ros__parameters']
    collision['base_frame_id'] = 'nav_reference'
    collision['StopZone']['points'] = str([[v*1.15 for v in p] for p in polygon])
    return params
