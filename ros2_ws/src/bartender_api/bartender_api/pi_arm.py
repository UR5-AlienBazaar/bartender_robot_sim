"""HTTP client for the Pi's arm bridge (arm_http_bridge.py on the rpi4).

The VLM-facing routes POST /arm/move and GET /arm/state in server.py go
through this instead of speaking DDS across the network, for the same
reason CONTROL_API.md gives for the whole API: an HTTP hop is a pinned
contract, and a DDS client on the far side of a network is worse.

The bridge on the Pi is arm_http_bridge.py (pi_camera_publisher repo,
started by run_arm.sh) and listens on the Pi's 8092.
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

    def move(self, joints):
        """Command absolute joint positions, in radians.

        Returns the bridge's answer, or {'ok': False, 'error': ...} when
        the Pi is unreachable.
        """
        try:
            return self._call('POST', '/arm/move', {'joints': list(joints)})
        except urllib.error.HTTPError as exc:
            try:
                return json.load(exc)
            except (ValueError, OSError):
                return {'ok': False, 'error': f'pi arm bridge: HTTP {exc.code}'}
        except (urllib.error.URLError, OSError, ValueError) as exc:
            return {'ok': False, 'error': f'pi arm bridge unreachable: {exc}'}

    def state(self):
        """Latest /joint_states from the Pi, or {'ok': False, ...}."""
        try:
            return self._call('GET', '/arm/state')
        except urllib.error.HTTPError as exc:
            try:
                return json.load(exc)
            except (ValueError, OSError):
                return {'ok': False, 'error': f'pi arm bridge: HTTP {exc.code}'}
        except (urllib.error.URLError, OSError, ValueError) as exc:
            return {'ok': False, 'error': f'pi arm bridge unreachable: {exc}'}
