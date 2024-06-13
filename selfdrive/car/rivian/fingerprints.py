from cereal import car
from openpilot.selfdrive.car.rivian.values import CAR

Ecu = car.CarParams.Ecu

FW_VERSIONS = {
  CAR.RIVIAN_R1S: {
    (Ecu.eps, 0x730, None): [
      b'TeM3_E014p10_0.0.0 (16),E014.17.00',
    ],
    (Ecu.engine, 0x606, None): [
      b'\x01\x00\x05\x18A\x00\x02\x00\x00\x00\x00\x00\x00\x00\x00\x00\x000\x91',
    ],
  },
}
