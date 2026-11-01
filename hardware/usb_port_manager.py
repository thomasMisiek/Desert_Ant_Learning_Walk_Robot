import os
import threading


_port_claims_lock = threading.Lock()
_port_claims = {}


def _stable_port_name(port):
    if port is None:
        return None

    try:
        return os.path.abspath(port)
    except Exception:
        return port


def _canonical_port(port):
    if port is None:
        return None

    try:
        return os.path.realpath(port)
    except Exception:
        return port


def claim_port(port, owner):
    stable_port = _stable_port_name(port)
    canonical_port = _canonical_port(port)

    if stable_port is None:
        return False

    with _port_claims_lock:
        for record in _port_claims.values():
            same_named_port = (
                record["stable_port"] == stable_port
            )

            same_physical_port = (
                canonical_port is not None
                and record["canonical_port"] == canonical_port
            )

            if same_named_port or same_physical_port:
                return record["owner"] == owner

        _port_claims[stable_port] = {
            "stable_port": stable_port,
            "canonical_port": canonical_port,
            "owner": owner,
        }

        return True


def release_port(port, owner):
    stable_port = _stable_port_name(port)

    if stable_port is None:
        return False

    with _port_claims_lock:
        record = _port_claims.get(
            stable_port
        )

        if record is None:
            return False

        if record["owner"] != owner:
            return False

        del _port_claims[stable_port]
        return True


def get_port_owner(port):
    stable_port = _stable_port_name(port)
    canonical_port = _canonical_port(port)

    if stable_port is None:
        return None

    with _port_claims_lock:
        for record in _port_claims.values():
            same_named_port = (
                record["stable_port"] == stable_port
            )

            same_physical_port = (
                canonical_port is not None
                and record["canonical_port"] == canonical_port
            )

            if same_named_port or same_physical_port:
                return record["owner"]

    return None


def is_port_claimed(port):
    return get_port_owner(port) is not None


def get_all_claims():
    with _port_claims_lock:
        claims = {}

        for stable_port, record in _port_claims.items():
            claims[stable_port] = record["owner"]

        return claims
