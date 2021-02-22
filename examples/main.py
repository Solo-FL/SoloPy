import solo_motor_controller as Solo

solo = Solo.SoloMotorController('address')

print('Firmware Version: ', solo.get_firmware_version())
print('Command Mode: ',solo.set_command_mode(True))
