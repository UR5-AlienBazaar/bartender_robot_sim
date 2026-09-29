"""HTTP client for the Pi's arm bridge (scripts/arm_http_bridge.py).

The VLM-facing routes POST /arm/move and GET /arm/state in server.py go
through this instead of speaking DDS across the network, for the same
reason CONTROL_API.md gives for the whole API: an HTTP hop is a pinned
contract, and a DDS client on the far side of a network is worse.

The bridge and the arm node it feeds (scripts/mab_arm_node.py,
scripts/run_arm.sh) live in this repo and are deployed to the rpi4 that
carries the CANdle; the bridge listens on the Pi's 8092.
"""
import json
import urllib.error
import urllib.request

TIMEOUT_S = 10.0


class PiArm:
    """Client for the rpi4 arm bridge.

    Never raises; every method returns a dict with an 'ok' field, matching
    the API's response shape.
    """

    def __init__(self, base_url):
        """Hold the bridge's base URL, e.g. http://10.42.0.200:8092."""
        self.base = base_url.rstrip('/')

    def _call(self, method, path, body=None):
        req = urllib.request.Request(
            self.base + path, method=method,
            data=json.dumps(body).encode('utf-8') if body is not None else None,
            headers={'Content-Type': 'application/json'})
        with urllib.request.urlopen(req, timeout=TIMEOUT_S) as resp:
            return json.load(resp)

    def _request(self, method, path, body=None):
        """_call plus the error shaping every answer shares.

        A bridge error body (a 400 naming the joints it refused, a 503 'no
        /joint_states yet') is more useful than the bare status, so it is
        parsed through -- but normalized, because some of the bridge's own
        error payloads carry only 'error', and the contract above promises
        'ok' on every dict this class returns.
        """
        try:
            return self._call(method, path, body)
        except urllib.error.HTTPError as exc:
            try:
                result = json.load(exc)
            except (ValueError, OSError):
                result = None
            if not isinstance(result, dict):
                return {'ok': False, 'error': f'pi arm bridge: HTTP {exc.code}'}
            result.setdefault('ok', False)
            return result
        except (urllib.error.URLError, OSError, ValueError) as exc:
            return {'ok': False, 'error': f'pi arm bridge unreachable: {exc}'}

    def move(self, joints):
        """Command absolute joint positions, in radians.

        Returns the bridge's answer, or {'ok': False, 'error': ...} when
        the Pi is unreachable.
        """
        return self._request('POST', '/arm/move', {'joints': list(joints)})

    def state(self):
        """Latest /joint_states from the Pi, or {'ok': False, ...}."""
        return self._request('GET', '/arm/state')
