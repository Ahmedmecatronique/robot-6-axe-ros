"""Noyau des ponts (sans ROS ni réseau) : historique en tampon anneau et mesure de latence par ping."""
import time

import numpy as np

from robot_gui.bridge_core import BridgeCore


class Dummy(BridgeCore):
    def __init__(self):
        super().__init__()
        self.sent = []

    def _publish(self, topic, msg_type, payload):
        self.sent.append((topic, msg_type, payload))

    def _call(self, service, srv_type, request, on_result):
        on_result(None)


def _feed(b, k):
    b.feed_joint_states('live', np.full(6, float(k)), np.zeros(6), np.zeros(6))


def test_history_before_wrap_is_ordered():
    b = Dummy()
    for k in range(100):
        _feed(b, k)
    t, q, c = b.history(1000)
    assert len(t) == 100
    np.testing.assert_array_equal(q[:, 0], np.arange(100))
    assert np.all(np.diff(t) >= 0) and t[-1] <= 0


def test_history_after_wrap_keeps_the_latest_samples_in_order():
    b = Dummy()
    n = b._hN
    for k in range(n + 500):                 # dépasse la capacité : le tampon boucle
        _feed(b, k)
    t, q, c = b.history(1e6)
    assert len(t) == n
    np.testing.assert_array_equal(q[:, 0], np.arange(500, n + 500))    # les plus récents, dans l'ordre
    assert np.all(np.diff(t) >= 0)


def test_history_window_selects_only_recent_samples():
    b = Dummy()
    for k in range(50):
        _feed(b, k)
    time.sleep(0.35)
    for k in range(50, 100):
        _feed(b, k)
    t, q, c = b.history(0.2)
    assert 45 <= len(t) <= 50                # seulement les échantillons récents
    assert q[-1, 0] == 99 and q[0, 0] >= 50
    assert np.all(t >= -0.21)


def test_history_tracks_command_and_resets_on_source_change():
    b = Dummy()
    b.feed_command('live', np.full(6, 5.0), np.zeros(6))
    _feed(b, 1)
    _, q, c = b.history(10)
    assert q[0, 0] == 1 and c[0, 0] == 5.0
    b.set_source('replay')
    assert len(b.history(10)[0]) == 0


def test_history_returns_copies_not_views_of_the_ring():
    b = Dummy()
    for k in range(10):
        _feed(b, k)
    _, q, _ = b.history(10)
    _feed(b, 999)
    assert q[-1, 0] == 9                     # l'ancien résultat n'est pas modifié par les écritures suivantes


def test_ping_pong_measures_round_trip():
    b = Dummy()
    b.send_ping()
    topic, msg_type, payload = b.sent[-1]
    assert topic == '/gui/ping' and msg_type == 'std_msgs/msg/Float64' and abs(payload['data'] - time.time()) < 1
    assert b.snapshot()['rtt_ms'] is None
    b.feed_pong(time.time() - 0.040)         # réponse arrivée 40 ms après l'envoi
    assert 35 < b.snapshot()['rtt_ms'] < 70
    b.feed_pong(time.time() + 10)            # horodatage aberrant (futur) : ignoré
    assert b.snapshot()['rtt_ms'] < 70
