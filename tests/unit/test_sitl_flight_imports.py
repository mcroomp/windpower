from unittest.mock import Mock, call

from linkhub_client.messages import NamedValueInt
from tests.sitl.flight.conftest import _send_anchor_location
from tests.sitl.stack_utils import HOME_ALT_M, HOME_LAT_DEG, HOME_LON_DEG


def test_flight_fixture_sends_anchor_from_sitl_home():
    gcs = Mock()
    _send_anchor_location(gcs, home_alt_m=25.0, log=Mock())

    assert gcs.send_message.call_args_list == [
        call(NamedValueInt(name="RAWES_LAT", value=round(HOME_LAT_DEG * 1e7))),
        call(NamedValueInt(name="RAWES_LON", value=round(HOME_LON_DEG * 1e7))),
        call(NamedValueInt(name="RAWES_AAL", value=round((HOME_ALT_M - 25.0) * 100))),
    ]
