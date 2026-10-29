from unittest.mock import Mock

import pytest

from linkhub_client.client import LinkHubClient, LinkHubError, LinkHubMotorController


def test_motor_command_binds_device_identity():
    client = LinkHubClient.__new__(LinkHubClient)
    client._request_json = Mock(return_value={"running": True})

    client.motor_set("motor-2", 15, "forward", timeout_ms=2000)

    client._request_json.assert_called_once_with("PUT", "/v1/motor", {
        "device_id": "motor-2",
        "speed_percent": 15,
        "direction": "forward",
        "timeout_ms": 2000,
    })


def test_calibration_motor_retains_connected_device_for_renewal():
    client = Mock()
    client.motor_status.return_value = {"available": True, "connected": False}
    client.motor_reconnect.return_value = {
        "connected": True, "device": "BLDC B", "device_id": "motor-2",
    }
    motor = LinkHubMotorController(client, speed=15)
    assert motor.connect() == "BLDC B"
    client.motor_stop.assert_called_once_with()
    motor._renew()
    client.motor_set.assert_called_once_with("motor-2", 15, "forward", timeout_ms=2000)


def test_calibration_motor_rejects_missing_device_identity():
    client = Mock()
    client.motor_status.return_value = {"available": True, "connected": True}
    motor = LinkHubMotorController(client)
    with pytest.raises(LinkHubError, match="connected device ID"):
        motor.connect()
    with pytest.raises(LinkHubError, match="Connect the Bluetooth motor"):
        motor.start()
    assert not motor.enabled
    client.motor_set.assert_not_called()
