import solo_motor_controller as Solo

solo = Solo.SoloMotorController(21)

#print('set_address: ', solo.set_address(21))

#print('set_command_mode: ', solo.set_command_mode(True))

#print('set_current_limit: ',solo.set_current_limit(21.21))

#print('set_torque_reference: ',solo.set_torque_reference(21.21))

#print('set_speed_reference: ',solo.set_speed_reference(2100))

#print('set_power_reference: ',solo.set_power_reference(21.21))

#print('set_identification: ',solo.set_identification(True))

##print('stop_system: ',solo.stop_system()) #need restart

#print('set_pwm_frequency: ',solo.set_pwm_frequency(21))

#print('set_speed_controller_Kp: ',solo.set_speed_controller_Kp(21.21))

#print('set_speed_controller_Ki: ',solo.set_speed_controller_Ki(21.21))

#print('set_direction: ',solo.set_direction(True))

#print('set_resistance: ',solo.set_resistance(21.21))

#print('set_inductance: ',solo.set_inductance(0.12))

#print('set_number_of_poles: ',solo.set_number_of_poles(21))

#print('set_encoder_lines: ',solo.set_encoder_lines(2121))

#print('set_speed_limit: ',solo.set_speed_limit(2121))

##print('reset_address: ',solo.reset_address())

#print('set_speed_control_mode: ',solo.set_speed_control_mode(2))

#print('reset_to_factory: ',solo.reset_to_factory())

#print('set_motor_type: ',solo.set_motor_type(2))

#print('set_control_mode: ',solo.set_control_mode(1))

#print('set_current_controller_kp: ',solo.set_current_controller_kp(21.21)) #need restart

#print('set_current_controller_ki: ',solo.set_current_controller_ki(21.21)) #need restart

#print('set_monitoring_mode: ',solo.set_monitoring_mode(2))

#print('set_magnetizing_current_reference: ',solo.set_magnetizing_current_reference(21.21))

#print('set_desired_position: ',solo.set_desired_position(2121))

#print('set_position_controller_kp: ',solo.set_position_controller_kp(21.21))

#print('set_position_controller_ki: ',solo.set_position_controller_ki(21.21))

##print('reset_position_to_zero: ',solo.reset_position_to_zero())

##print('overwrite_errors: ',solo.overwrite_errors())

#print('set_sog_normalBrushless_motor: ',solo.set_sog_normalBrushless_motor(21.21))

#print('set_sog_ultrafastBrushless_motor: ',solo.set_sog_ultrafastBrushless_motor(21.21))

#print('set_sog_dc_motor: ',solo.set_sog_dc_motor(21.21))

#print('set_sofg_normalBrushless_motor: ',solo.set_sofg_normalBrushless_motor(21.21))

#print('set_sofg_ultrafastBrushless_motor: ',solo.set_sofg_ultrafastBrushless_motor(21.21))

##print('set_uart_baudrate: ',solo.set_uart_baudrate(9600))

####################################### read ##########################################

#print('get_address: ',solo.get_address())

#print('get_voltage_a: ',solo.get_voltage_a())

#print('get_voltage_b: ',solo.get_voltage_b())

#print('get_current_a: ',solo.get_current_a())

#print('get_current_b: ',solo.get_current_b())

#print('get_bus_voltage: ',solo.get_bus_voltage())

#print('get_motor_current: ',solo.get_motor_current()) #??

#print('get_motor_voltage: ',solo.get_motor_voltage())

#print('get_speed_controller_kp: ',solo.get_speed_controller_kp())

#print('get_speed_controller_ki: ',solo.get_speed_controller_ki())

#print('get_Pwm_frequency: ',solo.get_Pwm_frequency())

#print('get_current_limit: ',solo.get_current_limit())

#print('get_quadrature_current: ',solo.get_quadrature_current()) #?? 8.27!

#print('get_direct_current: ',solo.get_direct_current()) #?? 5.025

#print('get_number_of_poles: ',solo.get_number_of_poles())

#print('get_encoder_lines: ',solo.get_encoder_lines())

#print('get_current_controller_kp: ',solo.get_current_controller_kp())

#print('get_current_controller_ki: ',solo.get_current_controller_ki()) #??

#print('get_temperature: ',solo.get_temperature())

#print('get_resistance: ',solo.get_resistance())

#print('get_inductance: ',solo.get_inductance())

#print('get_speed: ',solo.get_speed())

#print('get_motor_type: ',solo.get_motor_type()) #??

#print('get_speed_control_mode: ',solo.get_speed_control_mode())

#print('get_command_mode: ',solo.get_command_mode()) #??

#print('get_control_mode: ',solo.get_control_mode()) #??

#print('get_speed_limit: ',solo.get_speed_limit())

#print('get_position_controller_kp: ',solo.get_position_controller_kp())

#print('get_position_controller_ki: ',solo.get_position_controller_ki())

#print('get_encoder_position: ',solo.get_encoder_position())

#print('get_error_register: ',solo.get_error_register())

#print('get_firmware_version: ',solo.get_firmware_version())

#print('get_hardware_version: ',solo.get_hardware_version())

#print('get_torque_reference: ',solo.get_torque_reference())

#print('get_speed_reference: ',solo.get_speed_reference())

#print('get_magnetizing_current: ',solo.get_magnetizing_current())

#print('get_position_reference: ',solo.get_position_reference())

#print('get_power_reference: ',solo.get_power_reference())

#print('get_direction_rotation: ',solo.get_direction_rotation())

#print('get_sog_normalBrushless_motor: ',solo.get_sog_normalBrushless_motor())

#print('get_sog_ultraFastBrushless_motor: ',solo.get_sog_ultraFastBrushless_motor())

#print('get_sog_dc_motor: ',solo.get_sog_dc_motor())

#print('get_sofg_normalBrushless_motor: ',solo.get_sofg_normalBrushless_motor())

#print('get_sofg_ultraFastBrushless_motor: ',solo.get_sofg_ultraFastBrushless_motor())

print('get_uart_baudrate: ',solo.get_uart_baudrate())
