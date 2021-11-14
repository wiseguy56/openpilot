import numpy as np
from cereal import car
from selfdrive.config import Conversions as CV
from selfdrive.car.interfaces import CarStateBase
from opendbc.can.parser import CANParser
from opendbc.can.can_define import CANDefine
from selfdrive.car.audi_b8.values import DBC_FILES, CANBUS, NetworkLocation, TransmissionType, GearShifter, BUTTON_STATES, CarControllerParams

class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    can_define = CANDefine(DBC_FILES.audi_b8)
    self.shifter_values = can_define.dv["TSK_01"]["GearPosition"]
    self.hca_status_values = can_define.dv["LH_EPS_03"]["EPS_HCA_Status"]
    self.buttonStates = BUTTON_STATES.copy()

  def update(self, pt_cp, cam_cp, ext_cp, trans_type):
    ret = car.CarState.new_message()
    # Update vehicle speed and acceleration from ABS wheel speeds.
    ret.wheelSpeeds.fl = pt_cp.vl["ESP_03"]["ESP_VL_Radgeschw"] * CV.KPH_TO_MS
    ret.wheelSpeeds.fr = pt_cp.vl["ESP_03"]["ESP_VR_Radgeschw"] * CV.KPH_TO_MS
    ret.wheelSpeeds.rl = pt_cp.vl["ESP_03"]["ESP_HL_Radgeschw"] * CV.KPH_TO_MS
    ret.wheelSpeeds.rr = pt_cp.vl["ESP_03"]["ESP_HR_Radgeschw"] * CV.KPH_TO_MS

    ret.vEgoRaw = float(np.mean([ret.wheelSpeeds.fl, ret.wheelSpeeds.fr, ret.wheelSpeeds.rl, ret.wheelSpeeds.rr]))
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
    ret.standstill = ret.vEgoRaw < 0.1

    # Update steering angle, rate, yaw rate, and driver input torque. VW send
    # the sign/direction in a separate signal so they must be recombined.
    ret.steeringAngleDeg = pt_cp.vl["LH_EPS_03"]["EPS_Berechneter_LW"] * (1, -1)[int(pt_cp.vl["LH_EPS_03"]["EPS_VZ_BLW"])]
    ret.steeringRateDeg = pt_cp.vl["LWI_01"]["LWI_Lenkradw_Geschw"] * (1, -1)[int(pt_cp.vl["LWI_01"]["LWI_VZ_Lenkradw_Geschw"])]
    ret.steeringTorque = pt_cp.vl["LH_EPS_03"]["EPS_Lenkmoment"] * (1, -1)[int(pt_cp.vl["LH_EPS_03"]["EPS_VZ_Lenkmoment"])]
    ret.steeringPressed = abs(ret.steeringTorque) > CarControllerParams.STEER_DRIVER_ALLOWANCE
    ret.yawRate = pt_cp.vl["ESP_02"]["ESP_Gierrate"] * (1, -1)[int(pt_cp.vl["ESP_02"]["ESP_VZ_Gierrate"])] * CV.DEG_TO_RAD

    # Verify EPS readiness to accept steering commands
    hca_status = self.hca_status_values.get(pt_cp.vl["LH_EPS_03"]["EPS_HCA_Status"])
    ret.steerError = hca_status in ["DISABLED", "FAULT"]
    ret.steerWarning = hca_status in ["INITIALIZING", "REJECTED"]

    # Update gas, brakes, and gearshift.
    ret.gas = pt_cp.vl["Motor_03"]["MO_Fahrpedalrohwert_01"] / 100.0
    ret.gasPressed = ret.gas > 0
    ret.brake = pt_cp.vl["ESP_05"]["ESP_Bremsdruck"] / 250.0  # FIXME: this is pressure in Bar, not sure what OP expects
    ret.brakePressed = bool(pt_cp.vl["ESP_05"]["ESP_Fahrer_bremst"])

    # Update gear and/or clutch position data.
#    if trans_type == TransmissionType.automatic:
#      ret.gearShifter = self.parse_gear_shifter(self.shifter_values.get(pt_cp.vl["Getriebe_11"]["GE_Fahrstufe"], None))
#    elif trans_type == TransmissionType.direct:
    if bool(pt_cp.vl["Gateway"]["BCM1_Rueckfahrlicht_Schalter"]):
      ret.gearShifter = GearShifter.reverse
    else:
      ret.gearShifter = self.parse_gear_shifter(self.shifter_values.get(pt_cp.vl["TSK_01"]["GearPosition"], None))
#    elif trans_type == TransmissionType.manual:
#      ret.clutchPressed = not pt_cp.vl["Motor_14"]["MO_Kuppl_schalter"]
#      if bool(pt_cp.vl["Gateway_72"]["BCM1_Rueckfahrlicht_Schalter"]):
#        ret.gearShifter = GearShifter.reverse
#      else:
#        ret.gearShifter = GearShifter.drive

    # Update door open status.
    ret.doorOpen = any([pt_cp.vl["Gateway"]["ZV_FT_offen"]])

    # Update seatbelt fastened status.
    ret.seatbeltUnlatched = pt_cp.vl["Airbag_02"]["AB_Gurtschloss_FA"] != 3

    # Update driver preference for metric. VW stores many different unit
    # preferences, including separate units for for distance vs. speed.
    # We use the speed preference for OP.
    self.displayMetricUnits = not pt_cp.vl["Kombi_01"]["KBI_MFA_v_Einheit_01"]

    # Consume blind-spot monitoring info/warning LED states, if available.
    # Infostufe: BSM LED on, Warnung: BSM LED flashing
#    if self.CP.enableBsm:
#      ret.leftBlindspot = bool(ext_cp.vl["SWA_01"]["SWA_Infostufe_SWA_li"]) or bool(ext_cp.vl["SWA_01"]["SWA_Warnung_SWA_li"])
#      ret.rightBlindspot = bool(ext_cp.vl["SWA_01"]["SWA_Infostufe_SWA_re"]) or bool(ext_cp.vl["SWA_01"]["SWA_Warnung_SWA_re"])

    # Consume factory LDW data relevant for factory SWA (Lane Change Assist)
    # and capture it for forwarding to the blind spot radar controller
#    self.ldw_stock_values = cam_cp.vl["LDW_02"] if self.CP.networkLocation == NetworkLocation.fwdCamera else {}

    # Stock FCW is considered active if the release bit for brake-jerk warning
    # is set. Stock AEB considered active if the partial braking or target
    # braking release bits are set.
    # Refer to VW Self Study Program 890253: Volkswagen Driver Assistance
    # Systems, chapter on Front Assist with Braking: Golf Family for all MQB
#    ret.stockFcw = bool(ext_cp.vl["ACC_10"]["AWV2_Freigabe"])
#    ret.stockAeb = bool(ext_cp.vl["ACC_10"]["ANB_Teilbremsung_Freigabe"]) or bool(ext_cp.vl["ACC_10"]["ANB_Zielbremsung_Freigabe"])

    # Update ACC radar status.
    accStatus = pt_cp.vl["TSK_02"]["TSK_Status_GRA_ACC_01"]
    if accStatus == 0:
      # ACC okay and enabled, but not currently engaged
      ret.cruiseState.available = True
      ret.cruiseState.enabled = False
    else:
      # ACC okay and enabled, currently engaged and regulating speed (3) or engaged with driver accelerating (4) or overrun (5)
      ret.cruiseState.available = True
      ret.cruiseState.enabled = True
#    else:
#      # ACC okay but disabled (1), or a radar visibility or other fault/disruption (6 or 7)
#      ret.cruiseState.available = False
#      ret.cruiseState.enabled = False

    # Update ACC setpoint. When the setpoint is zero or there's an error, the
    # radar sends a set-speed of ~90.69 m/s / 203mph.
    ret.cruiseState.speed = ext_cp.vl["ACC_02"]["ACC_Wunschgeschw"] * CV.KPH_TO_MS
    if ret.cruiseState.speed > 90:
      ret.cruiseState.speed = 0

    # Update control button states for turn signals and ACC controls.
    self.buttonStates["accelCruise"] = bool(pt_cp.vl["LS_01"]["LS_Tip_Hoch"])
    self.buttonStates["decelCruise"] = bool(pt_cp.vl["LS_01"]["LS_Tip_Runter"])
    self.buttonStates["cancel"] = bool(pt_cp.vl["LS_01"]["LS_Abbrechen"])
    self.buttonStates["setCruise"] = bool(pt_cp.vl["LS_01"]["LS_Tip_Setzen"])
    self.buttonStates["resumeCruise"] = bool(pt_cp.vl["LS_01"]["LS_Tip_Wiederaufnahme"])
    self.buttonStates["gapAdjustCruise"] = bool(pt_cp.vl["LS_01"]["LS_Verstellung_Zeitluecke"])
    ret.leftBlinker = bool(pt_cp.vl["Blinkmodi_01"]["BM_links"])
    ret.rightBlinker = bool(pt_cp.vl["Blinkmodi_01"]["BM_rechts"])

    # Read ACC hardware button type configuration info that has to pass thru
    # to the radar. Ends up being different for steering wheel buttons vs
    # third stalk type controls.
    self.graHauptschalter = pt_cp.vl["LS_01"]["LS_Hauptschalter"]
    self.graTypHauptschalter = pt_cp.vl["LS_01"]["LS_Typ_Hauptschalter"]
#    self.graButtonTypeInfo = pt_cp.vl["LS_01"]["GRA_ButtonTypeInfo"]
#    self.graTipStufe2 = pt_cp.vl["LS_01"]["GRA_Tip_Stufe_2"]
    # Pick up the LS_01 CAN message counter so we can sync to it for
    # later cruise-control button spamming.
    self.graMsgBusCounter = pt_cp.vl["LS_01"]["LS_01_BZ"]

    # Additional safety checks performed in CarInterface.
    self.parkingBrakeSet = bool(pt_cp.vl["TSK_01"]["KBI_Handbremse"])  # FIXME: need to include an EPB check as well
    ret.espDisabled = pt_cp.vl["ESP_01"]["ESP_Tastung_passiv"] != 0

    return ret

  @staticmethod
  def get_can_parser(CP):
    # this function generates lists for signal, messages and initial values
    signals = [
      # sig_name, sig_address, default
      ("LWI_Lenkradw_Geschw", "LWI_01", 0),         # Absolute steering rate
      ("LWI_VZ_Lenkradw_Geschw", "LWI_01", 0),      # Steering rate sign
      ("EPS_Berechneter_LW", "LH_EPS_03", 0),       # Absolute steering angle
      ("EPS_HCA_Status", "LH_EPS_03", 3),           # EPS HCA control status
      ("EPS_Lenkmoment", "LH_EPS_03", 0),           # Absolute driver torque input
      ("EPS_VZ_BLW", "LH_EPS_03", 0),               # Steering angle sign
      ("EPS_VZ_Lenkmoment", "LH_EPS_03", 0),        # Driver torque input sign
      ("ESP_Tastung_passiv", "ESP_01", 0),          # Stability control disabled
      ("ESP_Gierrate", "ESP_02", 0),                # Absolute yaw rate
      ("ESP_VZ_Gierrate", "ESP_02", 0),             # Yaw rate sign
      ("ESP_HL_Radgeschw", "ESP_03", 0),            # ABS wheel speed, rear left
      ("ESP_HR_Radgeschw", "ESP_03", 0),            # ABS wheel speed, rear right
      ("ESP_VL_Radgeschw", "ESP_03", 0),            # ABS wheel speed, front left
      ("ESP_VR_Radgeschw", "ESP_03", 0),            # ABS wheel speed, front right
      ("MO_Fahrpedalrohwert_01", "Motor_03", 0),    # Accelerator pedal value
      ("ESP_Bremsdruck", "ESP_05", 0),              # Brake pressure applied
      ("ESP_Fahrer_bremst", "ESP_05", 0),           # Brake pedal pressed
      ("KBI_Handbremse", "TSK_01", 0),              # Manual handbrake applied
      ("LS_01_BZ", "LS_01", 0),                     # GRA_ACC_01 CAN message counter
      ("LS_Abbrechen", "LS_01", 0),                 # ACC button, cancel
      ("LS_Hauptschalter", "LS_01", 0),             # ACC button, on/off
      ("LS_Tip_Hoch", "LS_01", 0),                  # ACC button, increase or accel
      ("LS_Tip_Runter", "LS_01", 0),                # ACC button, decrease or decel
      ("LS_Tip_Setzen", "LS_01", 0),                # ACC button, set
      ("LS_Tip_Wiederaufnahme", "LS_01", 0),        # ACC button, resume
      ("LS_Typ_Hauptschalter", "LS_01", 0),         # ACC main button type
      ("LS_Verstellung_Zeitluecke", "LS_01", 0),    # ACC button, time gap adj
      ("TSK_Status_GRA_ACC_01", "TSK_02", 0),       # ACC engagement status from drivetrain coordinator
      ("KBI_MFA_v_Einheit_01", "Kombi_01", 0),      # MPH vs KMH speed display
      ("BM_links", "Blinkmodi_01", 0),              # Left turn signal including comfort blink interval
      ("BM_rechts", "Blinkmodi_01", 0),             # Right turn signal including comfort blink interval
      ("AB_Gurtschloss_BF", "Airbag_02", 0),        # Seatbelt status, passenger
      ("AB_Gurtschloss_FA", "Airbag_02", 0),        # Seatbelt status, driver
      ("ZV_BT_offen", "Gateway", 0),                # Door open, passenger
      ("ZV_FT_offen", "Gateway", 0),                # Door open, driver
    ]

    checks = [
      # sig_address, frequency
      ("LWI_01", 100),      # From J500 Steering Assist with integrated sensors
      ("LH_EPS_03", 100),   # From J500 Steering Assist with integrated sensors
      ("ESP_01", 50),       # From J104 ABS/ESP controller
      ("ESP_02", 50),       # From J104 ABS/ESP controller
      ("ESP_03", 50),       # From J104 ABS/ESP controller
      ("ESP_05", 50),       # From J104 ABS/ESP controller
      ("Motor_03", 100),    # From J623 Engine control module
      ("TSK_01", 50),       # From J623 Engine control module
      ("TSK_02", 50),       # From J623 Engine control module
      ("LS_01", 33),        # From J533 CAN gateway (via LIN from steering wheel controls)
      ("Gateway", 5),       # From J533 CAN gateway (aggregated data)
      ("Airbag_02", 5),     # From J234 Airbag control module
      ("Kombi_01", 20),     # From J285 Instrument cluster
      ("Blinkmodi_01", 0),  # From J519 BCM (sent at 1Hz when no lights active, 50Hz when active)
    ]

#    if CP.transmissionType == TransmissionType.automatic:
#      signals += [("GE_Fahrstufe", "Getriebe_11", 0)]  # Auto trans gear selector position
#      checks += [("Getriebe_11", 20)]  # From J743 Auto transmission control module
#    elif CP.transmissionType == TransmissionType.direct:
    signals += [("GearPosition", "TSK_01", 0),  # EV gear selector position
                ("BCM1_Rueckfahrlicht_Schalter", "Gateway", 0)]  # Reverse light from BCM
    checks += [("TSK_01", 10)]  # From J??? unknown EV control module
#    elif CP.transmissionType == TransmissionType.manual:
#      signals += [("MO_Kuppl_schalter", "Motor_14", 0),  # Clutch switch
#                  ("BCM1_Rueckfahrlicht_Schalter", "Gateway_72", 0)]  # Reverse light from BCM
#      checks += [("Motor_14", 10)]  # From J623 Engine control module

    if CP.networkLocation == NetworkLocation.fwdCamera:
      # Radars are here on CANBUS.pt
      signals += MqbExtraSignals.fwd_radar_signals
      checks += MqbExtraSignals.fwd_radar_checks
      if CP.enableBsm:
        signals += MqbExtraSignals.bsm_radar_signals
        checks += MqbExtraSignals.bsm_radar_checks

    return CANParser(DBC_FILES.audi_b8, signals, checks, CANBUS.pt)

  @staticmethod
  def get_cam_can_parser(CP):

    signals = []
    checks = []

    if CP.networkLocation == NetworkLocation.fwdCamera:
      signals += [
        # sig_name, sig_address, default
#        ("LDW_SW_Warnung_links", "LDW_02", 0),      # Blind spot in warning mode on left side due to lane departure
#        ("LDW_SW_Warnung_rechts", "LDW_02", 0),     # Blind spot in warning mode on right side due to lane departure
#        ("LDW_Seite_DLCTLC", "LDW_02", 0),          # Direction of most likely lane departure (left or right)
#        ("LDW_DLC", "LDW_02", 0),                   # Lane departure, distance to line crossing
#        ("LDW_TLC", "LDW_02", 0),                   # Lane departure, time to line crossing
      ]
      checks += [
        # sig_address, frequency
#        ("LDW_02", 10)      # From R242 Driver assistance camera
      ]
    else:
      # Radars are here on CANBUS.cam
      signals += MqbExtraSignals.fwd_radar_signals
      checks += MqbExtraSignals.fwd_radar_checks
      if CP.enableBsm:
        signals += MqbExtraSignals.bsm_radar_signals
        checks += MqbExtraSignals.bsm_radar_checks

    return CANParser(DBC_FILES.audi_b8, signals, checks, CANBUS.cam)

class MqbExtraSignals:
  # Additional signal and message lists for optional or bus-portable controllers
  fwd_radar_signals = [
    ("ACC_Wunschgeschw", "ACC_02", 0),              # ACC set speed
#    ("AWV2_Freigabe", "ACC_10", 0),                 # FCW brake jerk release
#    ("ANB_Teilbremsung_Freigabe", "ACC_10", 0),     # AEB partial braking release
#    ("ANB_Zielbremsung_Freigabe", "ACC_10", 0),     # AEB target braking release
  ]
  fwd_radar_checks = [
#    ("ACC_10", 50),                                 # From J428 ACC radar control module
    ("ACC_02", 17),                                 # From J428 ACC radar control module
  ]
  bsm_radar_signals = [
#    ("SWA_Infostufe_SWA_li", "SWA_01", 0),          # Blind spot object info, left
#    ("SWA_Warnung_SWA_li", "SWA_01", 0),            # Blind spot object warning, left
#    ("SWA_Infostufe_SWA_re", "SWA_01", 0),          # Blind spot object info, right
#    ("SWA_Warnung_SWA_re", "SWA_01", 0),            # Blind spot object warning, right
  ]
  bsm_radar_checks = [
#    ("SWA_01", 20),                                 # From J1086 Lane Change Assist
  ]
