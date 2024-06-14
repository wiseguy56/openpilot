import copy
from cereal import car
from openpilot.common.conversions import Conversions as CV
from openpilot.selfdrive.car.interfaces import CarStateBase
from opendbc.can.can_define import CANDefine
from opendbc.can.parser import CANParser
from openpilot.selfdrive.car.rivian.values import DBC, GEAR_MAP, BUTTONS


class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    self.can_define = CANDefine(DBC[CP.carFingerprint]['pt'])
    self.button_states = {button.event_type: False for button in BUTTONS}
    self.acc_enabled = None

  def update(self, cp, cp_cam):
    ret = car.CarState.new_message()

    ret.vEgoRaw = cp.vl["ESPiB1"]["ESPiB1_VehicleSpeed"]
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
    ret.standstill = (ret.vEgo < 0.1)

    # Gas pedal
    pedal_status = cp.vl["VDM_PropStatus"]["VDM_AcceleratorPedalPosition"]
    ret.gas = pedal_status / 100.0
    ret.gasPressed = (pedal_status > 0)

    # Brake pedal
    ret.brake = 0
    ret.brakePressed = cp.vl["iBESP2"]["iBESP2_BrakePedalApplied"] == 1

    # Steering wheel
    epas_status = cp.vl["EPAS_AdasStatus"]
    ret.steeringAngleDeg = cp.vl["EPAS_AdasStatus"]["EPAS_InternalSas"]
    ret.steeringRateDeg = cp.vl["EPAS_AdasStatus"]["EPAS_SteeringAngleSpeed"]
    ret.steeringTorque = cp.vl["EPAS_SystemStatus"]["EPAS_TorsionBarTorque"]

    ret.steeringPressed = cp.vl["EPAS_SystemStatus"]["EPAS_HandsOnLevel"] > 0
    eac_status = self.can_define.dv["EPAS_AdasStatus"]["EPAS_EacStatus"].get(int(epas_status["EPAS_EacStatus"]),None)
    ret.steerFaultPermanent = eac_status in ["EPAS_EacStatus_Eac_Fault"]
    eac_error = self.can_define.dv["EPAS_AdasStatus"]["EPAS_EacErrorCode"].get(int(epas_status["EPAS_EacErrorCode"]), None)
    ret.steerFaultTemporary = eac_error not in ["EPAS_No_Err"]

    # Cruise state
    ret.cruiseState.enabled = cp_cam.vl["VDM_AdasSts"]["VDM_AdasInterfaceStatus"] == 1
    ret.cruiseState.speed = cp.vl["ESPiB1"]["ESPiB1_VehicleSpeed"] # todo
    ret.cruiseState.available = cp_cam.vl["VDM_AdasSts"]["VDM_AdasInterfaceStatus"] == 1
    ret.cruiseState.standstill = False  # This needs to be false, since we can resume from stop without sending anything special

    # Gear
    ret.gearShifter = GEAR_MAP[self.can_define.dv["VDM_PropStatus"]["VDM_Prndl_Status"].get(int(cp.vl["VDM_PropStatus"]["VDM_Prndl_Status"]), "VDM_Prndl_Status_Not_Defined")]

    # Buttons
    button_events = []
    # for button in BUTTONS:
    #   state = (cp.vl[button.can_addr][button.can_msg] in button.values)
    #   if self.button_states[button.event_type] != state:
    #     event = car.CarState.ButtonEvent.new_message()
    #     event.type = button.event_type
    #     event.pressed = state
    #     button_events.append(event)
    #   self.button_states[button.event_type] = state
    ret.buttonEvents = button_events

    # Doors
    ret.doorOpen = False

    # Blinkers
    ret.leftBlinker = False
    ret.rightBlinker = False

    # Seatbelt
    ret.seatbeltUnlatched = cp.vl["RCM_Status"]["RCM_Status_IND_WARN_BELT_DRIVER"] != 0

    # Blindspot
    ret.leftBlindspot = False
    ret.rightBlindspot = False

    # AEB
    ret.stockAeb = cp_cam.vl["ACM_AebRequest"]["ACM_EnableRequest"] == 1

    # Messages needed by carcontroller
    self.acc_enabled = copy.copy(cp_cam.vl["ACM_longitudinalRequest"]["ACM_longInterfaceEnable"])


    return ret

  @staticmethod
  def get_can_parser(CP):
    messages = [
      # sig_address, frequency
      ("ESPiB1", 50),
      ("VDM_PropStatus", 50),
      ("iBESP2", 50),
      ("EPAS_AdasStatus", 100),
      ("EPAS_SystemStatus", 100),
      ("RCM_Status", 8),
    ]

    return CANParser(DBC[CP.carFingerprint]["pt"], messages, 0)

  @staticmethod
  def get_cam_can_parser(CP):
    messages = [
      ("ACM_longitudinalRequest", 100),
      ("VDM_AdasSts", 100),
      ("ACM_AebRequest", 100)
    ]

    return CANParser(DBC[CP.carFingerprint]["pt"], messages, 2)
