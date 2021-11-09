# flake8: noqa

from collections import defaultdict
from typing import Dict

from cereal import car
from selfdrive.car import dbc_dict

Ecu = car.CarParams.Ecu
NetworkLocation = car.CarParams.NetworkLocation
TransmissionType = car.CarParams.TransmissionType
GearShifter = car.CarState.GearShifter

class CarControllerParams:
  HCA_STEP = 2                   # HCA_01 message frequency 50Hz
  LDW_STEP = 10                  # LDW_02 message frequency 10Hz
  GRA_ACC_STEP = 3               # GRA_ACC_01 message frequency 33Hz

  GRA_VBP_STEP = 100             # Send ACC virtual button presses once a second
  GRA_VBP_COUNT = 16             # Send VBP messages for ~0.5s (GRA_ACC_STEP * 16)

  # Observed documented MQB limits: 3.00 Nm max, rate of change 5.00 Nm/sec.
  # Limiting rate-of-change based on real-world testing and Comma's safety
  # requirements for minimum time to lane departure.
  STEER_MAX = 300                # Max heading control assist torque 3.00 Nm
  STEER_DELTA_UP = 4             # Max HCA reached in 1.50s (STEER_MAX / (50Hz * 1.50))
  STEER_DELTA_DOWN = 10          # Min HCA reached in 0.60s (STEER_MAX / (50Hz * 0.60))
  STEER_DRIVER_ALLOWANCE = 80
  STEER_DRIVER_MULTIPLIER = 3    # weight driver torque heavily
  STEER_DRIVER_FACTOR = 1        # from dbc

class CANBUS:
  pt = 0
  cam = 2

class DBC_FILES:
  audi_b8 = "audi_b8"  # Used for Audi A5 B8 and B8.5

DBC = defaultdict(lambda: dbc_dict(DBC_FILES.audi_b8, None))  # type: Dict[str, Dict[str, str]]

BUTTON_STATES = {
  "accelCruise": False,
  "decelCruise": False,
  "cancel": False,
  "setCruise": False,
  "resumeCruise": False,
  "gapAdjustCruise": False
}

class CAR:
  AUDI_A5_B8 = "AUDI A5 B8"                         # Chassis 8T, Audi A5 B8/B8.5 and variants

# Control units on Extended CAN did not respond to UDS requests, so FW_VERSIONS could
# not be requested... using fingerprinting instead
#
# PLEASE DO NOT CONNECT PANDA TO THE CONVENIENCE CAN OF THE AUDI A5.
# The convenince CAN is not a high speed CAN with 500 kbps, instead it is a fault-
# tolerant CAN with only 100 kbps. If Panda gets connected, it will disturb the
# convenience CAN. So please, just connect Panda to the Extended CAN only.

FINGERPRINTS = {
  CAR.AUDI_A5_B8: [{
    64: 8, 134: 8, 159: 8, 256: 8, 257: 8, 259: 8, 260: 8, 261: 8, 262: 8, 264: 8, 265: 8, 266: 8, 267: 4, 268: 8, 279: 8, 286: 8, 294: 8, 523: 7, 543: 7, 776: 8, 778: 8, 779: 8, 780: 8, 782: 8, 785: 4, 810: 8, 901: 8, 914: 8, 919: 8, 959: 8, 960: 4, 994: 8, 1195: 1, 1215: 1, 1312: 8, 1318: 8, 1413: 8, 1440: 5, 1520: 5, 1536: 2, 1595: 3, 1596: 3, 1597: 8, 1605: 8, 1714: 8, 1716: 8, 1720: 8
  }],
}
