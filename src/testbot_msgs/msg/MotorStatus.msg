std_msgs/Header header
uint8 MOTOR_OK=0
uint8 MOTOR_OVERCURRENT=1
uint8 MOTOR_OVERTEMP=2
uint8 MOTOR_ERROR=3

uint8 left_motor_status    # Status code for left motor
uint8 right_motor_status   # Status code for right motor
float32 motor_voltage      # Current motor voltage
float32 motor_temperature  # Motor controller temperature
string status_message      # Additional status information