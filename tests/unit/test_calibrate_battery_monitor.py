from calibrate.repl import _cmd_battery


class _Session:
    def __init__(self, current: float):
        self.current = current
        self.writes: list[tuple[str, float]] = []

    def set_param(self, name: str, value: float) -> bool:
        self.writes.append((name, value))
        self.current = value
        return True

    def get_param(self, name: str) -> float | None:
        assert name == "BATT_MONITOR"
        return self.current


def test_battery_monitor_off_disables_and_warns_reboot_required(capsys):
    session = _Session(4.0)

    _cmd_battery(session, ["monitor", "off"])

    assert session.writes == [("BATT_MONITOR", 0)]
    output = capsys.readouterr().out
    assert "BATT_MONITOR = 0 (disabled)" in output
    assert "reboot-required" in output


def test_battery_monitor_on_defaults_to_analog_voltage_and_current(capsys):
    session = _Session(0.0)

    _cmd_battery(session, ["monitor", "on"])

    assert session.writes == [("BATT_MONITOR", 4)]
    assert "BATT_MONITOR = 4 (analog voltage + current)" in capsys.readouterr().out


def test_battery_monitor_on_accepts_explicit_monitor_type():
    session = _Session(0.0)

    _cmd_battery(session, ["monitor", "on", "3"])

    assert session.writes == [("BATT_MONITOR", 3)]


def test_battery_monitor_rejects_invalid_action_without_writing(capsys):
    session = _Session(4.0)

    _cmd_battery(session, ["monitor", "enable"])

    assert session.writes == []
    assert "Usage:" in capsys.readouterr().out
