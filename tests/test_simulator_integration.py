"""Opt-in real SDK and simulator TCP connectivity check; no automated flight."""

import os
import socket

import pytest


@pytest.mark.integration
@pytest.mark.skipif(os.environ.get("UAV_SIMULATOR_CONNECTIVITY") != "1",
                    reason="Live SDK/simulator not enabled; mocked callbacks do not validate physical flight")
def test_real_sdk_and_simulator_connectivity():
    pytest.importorskip("udacidrone")
    from udacidrone.connection import MavlinkConnection
    assert callable(MavlinkConnection)
    with socket.create_connection((os.environ.get("UAV_SIMULATOR_HOST", "127.0.0.1"),
                                   int(os.environ.get("UAV_SIMULATOR_PORT", "5760"))), timeout=3):
        pass
