import solo_motor_controller as solo

solo_driver = solo.SoloMotorController('address')
print('firmware_version: ', solo_driver.get_firmware_version())

print('Command Mode: ',solo_driver.set_command_mode(True))