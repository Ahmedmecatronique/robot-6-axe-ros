"""Test du WebBridge (chemin Windows) contre un FAUX serveur rosbridge : vrai roslibpy, vrai WebSocket.

Le faux serveur parle le protocole rosbridge (JSON) : il accepte subscribe / advertise / publish / call_service,
pousse des messages de robot simulé et enregistre tout ce que l'IHM envoie. Cela valide le décodage des
messages, l'ordre des articulations, les commandes publiées et les appels de service — pas rosbridge lui-même.
"""
import asyncio
import base64
import json
import threading
import time

import numpy as np
import pytest

pytest.importorskip('roslibpy')
websockets = pytest.importorskip('websockets')

from robot_gui.web_bridge import WebBridge  # noqa: E402

PORT = 19090


class FakeRosbridge:
    def __init__(self, port=PORT):
        self.port = port
        self.subscribed = set()
        self.published = []            # [(topic, msg)] envoyés par l'IHM
        self.service_calls = []        # [(service, args)]
        self.loop = None
        self.ready = threading.Event()

    # ----- réponses de services simulées
    def service_values(self, service, args):
        if service == '/solve_ik':
            return {'reachable': True, 'near_singularity': True, 'message': 'Solution analytique (test)',
                    'joint_positions': [0.1] * 6, 'position_error': 0.0, 'orientation_error': 0.0,
                    'method': 'analytical'}
        return {'success': True, 'message': f'ok {service}'}

    async def handler(self, ws):
        pusher = asyncio.create_task(self.push_loop(ws))
        try:
            async for raw in ws:
                m = json.loads(raw)
                op = m.get('op')
                if op == 'subscribe':
                    self.subscribed.add(m['topic'])
                elif op == 'publish':
                    self.published.append((m['topic'], m['msg']))
                    if m['topic'] == '/gui/ping' and '/gui/pong' in self.subscribed:   # safety_node renvoie le ping
                        await ws.send(json.dumps({'op': 'publish', 'topic': '/gui/pong', 'msg': m['msg']}))
                elif op == 'call_service':
                    self.service_calls.append((m['service'], m.get('args')))
                    await ws.send(json.dumps({'op': 'service_response', 'service': m['service'], 'id': m.get('id'),
                                              'values': self.service_values(m['service'], m.get('args')),
                                              'result': True}))
        except websockets.ConnectionClosed:
            pass
        finally:
            pusher.cancel()

    async def push_loop(self, ws):
        """Pousse des mesures comme le ferait rosbridge : JointState (noms DANS LE DÉSORDRE), état, mode..."""
        jpg = base64.b64encode(b'\xff\xd8fake-jpeg\xff\xd9').decode()
        try:
            while True:
                t = time.time()
                names = ['joint_3', 'joint_1', 'joint_2', 'joint_6', 'joint_5', 'joint_4']   # ordre mélangé !
                vals = {'joint_1': 0.10, 'joint_2': 0.20, 'joint_3': 0.30, 'joint_4': 0.40, 'joint_5': 0.50,
                        'joint_6': 0.60}
                js = {'header': {'stamp': {'sec': int(t), 'nanosec': int((t % 1) * 1e9)}, 'frame_id': ''},
                      'name': names, 'position': [vals[n] for n in names],
                      'velocity': [0.0] * 6, 'effort': [0.0] * 6}
                msgs = {
                    '/joint_states': js,
                    '/joint_commands': {**js, 'position': [v + 0.01 for v in js['position']]},
                    '/stm32/status': {'data': [4, 63, 0, 0, 5, 1000, 8, 42]},
                    '/robot_mode': {'data': 'simulation'},
                    '/target_visible': {'data': True},
                    '/trajectory_active': {'data': False},
                    '/end_effector_pose': {'header': {}, 'pose': {'position': {'x': .3, 'y': 0., 'z': .5},
                                           'orientation': {'x': 0., 'y': 0., 'z': 0., 'w': 1.}}},
                    '/target_pose': {'header': {}, 'pose': {'position': {'x': .4, 'y': .1, 'z': .02},
                                     'orientation': {'x': 1., 'y': 0., 'z': 0., 'w': 0.}}},
                    '/camera/image_annotated': {'header': {}, 'format': 'jpeg', 'data': jpg},
                    '/bag/status': {'data': json.dumps({'recording': False, 'playing': False, 'current': '',
                                                        'names': ['session_a', 'session_b']})},
                    '/diagnostics': {'header': {}, 'status': [
                        {'name': 'arm6/test', 'level': 2, 'message': 'panne simulée', 'hardware_id': 'x',
                         'values': [{'key': 'k', 'value': 'v'}]}]},
                }
                for topic, msg in msgs.items():
                    if topic in self.subscribed:
                        await ws.send(json.dumps({'op': 'publish', 'topic': topic, 'msg': msg}))
                await asyncio.sleep(0.02)
        except (asyncio.CancelledError, websockets.ConnectionClosed):
            pass

    def run(self):
        async def main():
            async with websockets.serve(self.handler, '127.0.0.1', self.port):
                self.ready.set()
                await asyncio.Future()
        self.loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.loop)
        try:
            self.loop.run_until_complete(main())
        except asyncio.CancelledError:
            pass


@pytest.fixture(scope='module')
def bridge():
    server = FakeRosbridge()
    threading.Thread(target=server.run, daemon=True).start()
    assert server.ready.wait(5)
    b = WebBridge('127.0.0.1', server.port)
    deadline = time.time() + 10
    while time.time() < deadline and b.snapshot()['q_meas'] is None:
        time.sleep(0.05)
    b.server = server
    yield b
    b.shutdown()


def _wait(pred, timeout=5.0):
    t = time.time()
    while time.time() - t < timeout:
        if pred():
            return True
        time.sleep(0.03)
    return False


def test_receives_and_decodes_robot_state(bridge):
    # on attend que TOUS les flux soient arrivés (la fréquence n'est estimable qu'à partir du 2e message)
    assert _wait(lambda: (lambda s: s['js_rate'] > 5 and s['ee_T'] is not None and s['target_T'] is not None
                          and s['mode'] == 'simulation' and s['diagnostics'] and s['latency_ms'] is not None
                          and bridge.list_bags())(bridge.snapshot()))
    snap = bridge.snapshot()
    # JointState reçu avec les noms dans le désordre -> remis dans l'ordre joint_1..joint_6
    np.testing.assert_allclose(snap['q_meas'], [0.1, 0.2, 0.3, 0.4, 0.5, 0.6])
    np.testing.assert_allclose(snap['q_cmd'], [0.11, 0.21, 0.31, 0.41, 0.51, 0.61])
    assert snap['state'] == 4 and snap['mode'] == 'simulation' and snap['target_visible'] is True
    assert snap['js_rate'] > 5
    np.testing.assert_allclose(snap['ee_T'][:3, 3], [0.3, 0.0, 0.5])
    np.testing.assert_allclose(snap['target_T'][:3, 3], [0.4, 0.1, 0.02])
    assert bridge.image_jpeg()[1] == b'\xff\xd8fake-jpeg\xff\xd9'          # base64 décodé
    assert bridge.list_bags() == ['session_a', 'session_b']
    assert snap['diagnostics'][0]['level'] == 2
    assert snap['latency_ms'] is not None and abs(snap['latency_ms']) < 2000


def test_error_diagnostic_raises_alarm_event(bridge):
    assert _wait(lambda: any(e[0] == 'alarm' and 'panne simulée' in e[1] for e in _all_events(bridge)))


_EVENTS = []


def _all_events(bridge):
    _EVENTS.extend(bridge.drain_events())
    return _EVENTS


def test_publishes_joint_goal_in_ros_json_format(bridge):
    bridge.send_joint_goal([np.deg2rad([10, 20, 30, 40, 50, 60])])
    assert _wait(lambda: any(t == '/joint_goal' for t, _ in bridge.server.published))
    msg = [m for t, m in bridge.server.published if t == '/joint_goal'][-1]
    assert msg['joint_names'] == ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
    np.testing.assert_allclose(msg['points'][0]['positions'], np.deg2rad([10, 20, 30, 40, 50, 60]))


def test_estop_is_published_three_times(bridge):
    before = len([1 for t, _ in bridge.server.published if t == '/emergency_stop'])
    bridge.send_estop()
    assert _wait(lambda: len([1 for t, _ in bridge.server.published if t == '/emergency_stop']) - before >= 3)
    assert all(m == {'data': True} for t, m in bridge.server.published if t == '/emergency_stop')


def test_target_pose_and_gripper(bridge):
    from robot_kinematics.transforms import make_transform, rot_x
    bridge.send_target_pose(make_transform(rot_x(np.pi), [0.25, 0.0, 0.4]))
    bridge.send_gripper(1.0)
    assert _wait(lambda: any(t == '/target_pose' for t, _ in bridge.server.published)
                 and any(t == '/gripper_command' for t, _ in bridge.server.published))
    pose = [m for t, m in bridge.server.published if t == '/target_pose'][-1]
    assert pose['header']['frame_id'] == 'base_link'
    assert abs(pose['pose']['position']['z'] - 0.4) < 1e-9


def test_services_and_events(bridge):
    _all_events(bridge)
    bridge.call_home()
    bridge.call_enable(True)
    from robot_kinematics.transforms import make_transform, rot_x
    bridge.call_solve_ik(make_transform(rot_x(np.pi), [0.25, 0.0, 0.4]))
    assert _wait(lambda: any(e[0] == 'service' and e[1] == 'homing' and e[2] for e in _all_events(bridge)))
    assert _wait(lambda: any(e[0] == 'service' and e[1] == 'activation' for e in _all_events(bridge)))
    assert _wait(lambda: any(e[0] == 'ik_preview' and e[1] and e[1]['near_singularity'] for e in _all_events(bridge)))
    calls = dict(bridge.server.service_calls)
    assert calls['/robot/enable'] == {'enable': True}


def test_speed_scale_uses_set_parameters(bridge):
    bridge.set_speed_scale(0.25)
    assert _wait(lambda: any(s == '/trajectory_node/set_parameters' for s, _ in bridge.server.service_calls))
    args = [a for s, a in bridge.server.service_calls if s == '/trajectory_node/set_parameters'][-1]
    assert args['parameters'][0] == {'name': 'speed_scale', 'value': {'type': 3, 'double_value': 0.25}}


def test_rosbag_is_controlled_on_the_robot(bridge):
    ok, _ = bridge.bag_record_start()
    assert ok
    assert _wait(lambda: ('/bag/command', {'command': 'record_start', 'name': ''}) in bridge.server.service_calls)
    bridge.bag_replay_start('session_a')
    assert bridge.snapshot()['source'] == 'replay'
    assert _wait(lambda: ('/bag/command', {'command': 'play', 'name': 'session_a'}) in bridge.server.service_calls)
    assert not bridge.replay_finished()            # délai de grâce : ne s'arrête pas juste après le démarrage
    bridge.bag_replay_stop()
    assert bridge.snapshot()['source'] == 'live'


def test_pick_uses_trigger_service_and_status_events(bridge):
    bridge.pick_detected()
    assert _wait(lambda: any(s == '/pick/start' for s, _ in bridge.server.service_calls))
    bridge.feed_pick_status({'phase': 'approche', 'progress': 0.3, 'done': False})
    bridge.feed_pick_status({'phase': 'terminé', 'progress': 1.0, 'done': True, 'success': True, 'message': 'ok'})
    evs = bridge.drain_events()
    assert ('pick_feedback', 'approche', 0.3) in evs and ('pick_done', True, 'ok') in evs


def test_ping_pong_round_trip_through_rosbridge(bridge):
    bridge.send_ping()
    assert _wait(lambda: bridge.snapshot()['rtt_ms'] is not None)
    assert 0 <= bridge.snapshot()['rtt_ms'] < 1000
