# from helios.leg import Leg
# import numpy as np
# from pydantic import ValidationError


# def test_belongs_to_strip():
#     leg = Leg(serial_id=1)
#     assert leg.belongs_to_strip is False
#     leg.strip = "Some Strip Object"  # Replace with an actual ScanningStrip object
#     assert leg.belongs_to_strip is True

# def test_invalid_serial_id():
#     with pytest.raises(ValidationError):
#         Leg(serial_id=-1)  # NonNegativeInt should prevent negative serial_id