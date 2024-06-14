from opendbc.can.packer import CANPacker
from openpilot.common.numpy_fast import clip
from openpilot.selfdrive.car import apply_std_steer_angle_limits
from openpilot.selfdrive.car.interfaces import CarControllerBase
from openpilot.selfdrive.car.rivian import riviancan
from openpilot.selfdrive.car.rivian.values import CarControllerParams


class CarController(CarControllerBase):
  def __init__(self, dbc_name, CP, VM):
    self.CP = CP
    self.frame = 0
    self.apply_angle_last = 0
    self.packer = CANPacker(dbc_name)

  def update(self, CC, CS, now_nanos):
    actuators = CC.actuators

    can_sends = []

    if CC.latActive:
      # Angular rate limit based on speed
      apply_angle = apply_std_steer_angle_limits(actuators.steeringAngleDeg, self.apply_angle_last, CS.out.vEgo, CarControllerParams)
      # To not fault the EPS
      apply_angle = clip(apply_angle, CS.out.steeringAngleDeg - 20, CS.out.steeringAngleDeg + 20)
    else:
      apply_angle = CS.out.steeringAngleDeg

    self.apply_angle_last = apply_angle
    can_sends.append(riviancan.create_steering_control(self.packer, self.frame, apply_angle, CC.latActive))

    # Longitudinal control
    if self.CP.openpilotLongitudinalControl:
      can_sends.append(riviancan.create_longitudinal_commands(self.packer, self.frame, actuators.accel, CS.acc_enabled))

    new_actuators = actuators.as_builder()
    new_actuators.steeringAngleDeg = self.apply_angle_last

    self.frame += 1
    return new_actuators, can_sends
