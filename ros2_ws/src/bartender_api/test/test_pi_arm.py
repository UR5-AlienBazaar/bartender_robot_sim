"""Tests for pi_arm.py: request shaping and the never-raises contract.

urlopen is stubbed, so what is checked here is which request goes out and
what comes back -- in particular that every failure, however the bridge or
the network produced it, returns a dict carrying 'ok'. server.py reads
result.get('ok') to pick 200 or 502, so a body without it must still be one.
"""
import io
import json
import os
import sys
import urllib.error
import urllib.request

import pytest

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.abspath(__file__)), os.pardir))

from bartender_api import pi_arm                           # noqa: E402


class _Response:
    """The context-manager handle urlopen returns on success."""

    def __init__(self, payload):
        self.payload = payload

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        return False

    def read(self):
        return json.dumps(self.payload).encode('utf-8')


def _http_error(code, payload):
    """The bridge's error replies: a status plus a JSON body."""
    return urllib.error.HTTPError(
        'http://pi:8092/x', code, 'reason', {}, io.BytesIO(
            json.dumps(payload).encode('utf-8')))


@pytest.fixture
def seen(monkeypatch):
    """Capture requests instead of opening them; returns (calls, reply_with)."""
    calls = []
    box = {'reply': _Response({'ok': True})}

    def fake_urlopen(req, timeout=None):
        calls.append({'method': req.get_method(), 'url': req.full_url,
                      'body': req.data, 'timeout': timeout})
        if isinstance(box['reply'], Exception):
            raise box['reply']
        return box['reply']

    monkeypatch.setattr(urllib.request, 'urlopen', fake_urlopen)
    return calls, box


def test_move_posts_the_joints_as_json(seen):
    calls, _ = seen
    pi_arm.PiArm('http://10.42.0.200:8092').move([0.5, -1.2])
    (call,) = calls
    assert call['method'] == 'POST'
    assert call['url'] == 'http://10.42.0.200:8092/arm/move'
    assert json.loads(call['body']) == {'joints': [0.5, -1.2]}


def test_state_gets_with_no_body(seen):
    calls, _ = seen
    pi_arm.PiArm('http://10.42.0.200:8092').state()
    (call,) = calls
    assert call['method'] == 'GET'
    assert call['url'] == 'http://10.42.0.200:8092/arm/state'
    assert call['body'] is None


def test_a_trailing_slash_on_the_base_url_does_not_double_up(seen):
    calls, _ = seen
    pi_arm.PiArm('http://10.42.0.200:8092/').state()
    assert calls[0]['url'] == 'http://10.42.0.200:8092/arm/state'


def test_move_waits_no_longer_than_the_timeout(seen):
    calls, _ = seen
    pi_arm.PiArm('http://pi:8092').move([0.0])
    assert calls[0]['timeout'] == pi_arm.TIMEOUT_S


def test_a_200_body_comes_back_untouched(seen):
    _, box = seen
    box['reply'] = _Response({'ok': True, 'joints': [0.1]})
    assert pi_arm.PiArm('http://pi:8092').state() == {'ok': True,
                                                      'joints': [0.1]}


def test_a_bridge_error_body_keeps_its_reason(seen):
    """The bridge's 400 names the joints it refused; that beats 'HTTP 400'."""
    _, box = seen
    box['reply'] = _http_error(400, {'ok': False, 'error': 'bad joints'})
    assert pi_arm.PiArm('http://pi:8092').move([9.0]) == {
        'ok': False, 'error': 'bad joints'}


def test_a_bridge_error_without_ok_still_gets_one(seen):
    """The bridge's own error payloads do not all carry 'ok'.

        {'error': 'not found'}            (its 404)
        {'error': 'bad request body'}     (its 400 for unparseable JSON)

    The class contract is 'a dict with an ok field'; passing such a body
    through raw would break it, so the client fills it in.
    """
    _, box = seen
    box['reply'] = _http_error(400, {'error': 'joints must be finite'})
    result = pi_arm.PiArm('http://pi:8092').move([float('nan')])
    assert result == {'ok': False, 'error': 'joints must be finite'}


def test_an_unreadable_error_body_falls_back_to_the_status(seen):
    _, box = seen
    box['reply'] = urllib.error.HTTPError(
        'http://pi:8092/x', 500, 'reason', {}, io.BytesIO(b'not json'))
    result = pi_arm.PiArm('http://pi:8092').state()
    assert result == {'ok': False, 'error': 'pi arm bridge: HTTP 500'}


def test_a_non_dict_error_body_is_not_returned(seen):
    """A JSON list parses fine but cannot carry 'ok'; fall back instead."""
    _, box = seen
    box['reply'] = _http_error(500, ['not', 'a', 'dict'])
    result = pi_arm.PiArm('http://pi:8092').state()
    assert result == {'ok': False, 'error': 'pi arm bridge: HTTP 500'}


@pytest.mark.parametrize('failure', [
    urllib.error.URLError('connection refused'),
    OSError('timed out'),
    TimeoutError('timed out'),
])
def test_the_pi_being_gone_is_an_answer_not_an_exception(seen, failure):
    """urlopen failure modes all mean the same thing to the caller."""
    _, box = seen
    box['reply'] = failure
    result = pi_arm.PiArm('http://pi:8092').move([0.0])
    assert result['ok'] is False
    assert 'unreachable' in result['error']


@pytest.mark.parametrize('call', [
    lambda arm: arm.move([0.5]),
    lambda arm: arm.state(),
])
def test_every_answer_carries_ok(seen, call):
    """server.py switches on result.get('ok'); None would 502 a good reply."""
    _, box = seen
    for reply in [_Response({'ok': True}),
                  _http_error(503, {'ok': False, 'error': 'no joints yet'}),
                  _http_error(404, {'error': 'not found'}),
                  urllib.error.URLError('down')]:
        box['reply'] = reply
        assert 'ok' in call(pi_arm.PiArm('http://pi:8092'))
