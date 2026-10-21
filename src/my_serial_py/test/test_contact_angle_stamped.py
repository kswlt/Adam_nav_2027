import ast
from pathlib import Path


def test_serial_bridge_declares_auditable_angle_topic():
    text = Path(__file__).parents[1].joinpath('my_serial_py/serial_bridge.py').read_text()
    tree = ast.parse(text)
    source = ast.unparse(tree)
    assert 'contact_angle_stamped' in source
    assert 'host_monotonic_ns' in source
    assert 'semantics_verified = False' in source
