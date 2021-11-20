# CAN controls for Audi B8 platform.

def create_audi_b8_steering_control(packer, bus, apply_steer, idx, lkas_enabled):
  values = {
    "SET_ME_0X3": 0x0E,
    "Assist_Torque": abs(apply_steer),
    "Assist_Requested": lkas_enabled,
    "Assist_VZ": 1 if apply_steer < 0 else 0,
    "HCA_Available": 1,
    "HCA_Standby": not lkas_enabled,
    "HCA_Active": lkas_enabled,
    "SET_ME_0XFE": 0x00,
    "SET_ME_0X07": 0x00,
  }
  return packer.make_can_msg("HCA_01", bus, values, idx)

def create_audi_b8_acc_buttons_control(packer, bus, buttonStatesToSend, CS, idx):
  values = {
    "GRA_Hauptschalter": CS.graHauptschalter,
    "GRA_Abbrechen": buttonStatesToSend["cancel"],
    "GRA_Tip_Setzen": buttonStatesToSend["setCruise"],
    "GRA_Tip_Hoch": buttonStatesToSend["accelCruise"],
    "GRA_Tip_Runter": buttonStatesToSend["decelCruise"],
    "GRA_Tip_Wiederaufnahme": buttonStatesToSend["resumeCruise"],
    "GRA_Verstellung_Zeitluecke": 3 if buttonStatesToSend["gapAdjustCruise"] else 0,
    "GRA_Typ_Hauptschalter": CS.graTypHauptschalter,
#    "GRA_Codierung": 2,
#    "GRA_Tip_Stufe_2": CS.graTipStufe2,
#    "GRA_ButtonTypeInfo": CS.graButtonTypeInfo
  }
  return packer.make_can_msg("GRA_ACC_01", bus, values, idx)
