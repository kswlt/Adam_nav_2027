"""Transport-independent admission of real ZLIO sessions and backend trust."""
import math


def parse_state(text):
    result = {}
    for field in text.split(';'):
        if '=' not in field:
            raise ValueError('Malformed Z-Localization status')
        key, value = field.split('=', 1)
        if key in result:
            raise ValueError('Duplicate Z-Localization status field')
        result[key] = value
    for key in ('status', 'pose_trusted', 'tf_frozen', 'tf_published', 'epoch', 'stamp_ns'):
        if key not in result:
            raise ValueError('Missing Z-Localization status field: '+key)
    result['epoch'] = int(result['epoch'])
    result['stamp_ns'] = int(result['stamp_ns'])
    return result


def trusted_for_navigation(state, mode, epoch, now_ns, timeout):
    return (mode == 'localization' and state.get('status') == 'NORMAL' and
            state.get('pose_trusted') == 'true' and state.get('tf_frozen') == 'false' and
            state.get('tf_published') == 'true' and state.get('epoch') == epoch and
            0 < state.get('stamp_ns', 0) and
            -.05 <= (now_ns-state['stamp_ns'])*1.e-9 <= timeout)


def rotation(q):
    import numpy as np
    x, y, z, w = q
    if not all(map(math.isfinite, q)) or abs(x*x+y*y+z*z+w*w-1.) > .001:
        raise ValueError('Non-unit sensor quaternion')
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
