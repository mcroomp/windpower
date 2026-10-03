from __future__ import annotations

import numpy as np
import pytest

from scripts.linkhub_stress import run_stress
from tests.sitl.stack_infra import StackConfig, _static_stack


pytestmark = pytest.mark.sitl


def test_linkhub_stress_sitl(tmp_path, request) -> None:
    zeros = np.zeros(3)
    with _static_stack(
        tmp_path,
        pos=zeros,
        vel=zeros,
        rpy=zeros,
        accel_body=np.array([0.0, 0.0, -9.81]),
        gyro=zeros,
        test_name=request.node.name,
    ):
        result = run_stress(
            StackConfig.LINKHUB_URL,
            readers=32,
            duration=20.0,
            attitude_rate=100.0,
            send_burst=256,
            send_rounds=3,
            malformed_burst=96,
            timeout_requests=8,
        )

    assert result.sends_archived == 768
    assert result.follower_connections == 64
    assert result.readers == 32
    assert result.attitude_records >= 1_400
    assert result.attitude_rate_hz >= 70.0
