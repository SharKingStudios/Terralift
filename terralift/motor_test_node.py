#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8

# Try to import RPi.GPIO; if not available (e.g., WSL), fall back to SIM.
SIM_DEFAULT = False
try:
    import RPi.GPIO as GPIO  # provided by python3-rpi-lgpio on the Pi
except Exception:
    SIM_DEFAULT = True
    GPIO = None

class MotorTestNode(Node):
    """
    /fl_cmd (std_msgs/Int8):
      1  -> forward (after FL_POL applied)
     -1  -> backward
      0  -> stop

    Defaults match your ESP32 front-left:
      DIR=BCM16, PWM=BCM14, PWM=2000 Hz, duty=70%, FL_POL=-1
    """

    def __init__(self):
        super().__init__('motor_test_node')

        # ----- Parameters (defaults match your ESP32 mapping) -----
        self.declare_parameter('dir_pin', 17)              # BCM17
        self.declare_parameter('pwm_pin', 27)              # BCM27
        self.declare_parameter('pwm_hz', 2000)             # 2 kHz like ESP32
        self.declare_parameter('test_duty', 15.0)          # percent
        self.declare_parameter('dir_active_high', True)    # HIGH = "forward" before FL_POL
        self.declare_parameter('fl_pol', -1)               # ESP32 used FL_POL = -1
        self.declare_parameter('simulate', SIM_DEFAULT)    # force SIM in WSL

        self.dir_pin        = int(self.get_parameter('dir_pin').value)
        self.pwm_pin        = int(self.get_parameter('pwm_pin').value)
        self.pwm_hz         = int(self.get_parameter('pwm_hz').value)
        self.test_duty      = float(self.get_parameter('test_duty').value)
        self.dir_active_high= bool(self.get_parameter('dir_active_high').value)
        self.fl_pol         = int(self.get_parameter('fl_pol').value)
        self.simulate       = bool(self.get_parameter('simulate').value)

        self.pwm = None

        if self.simulate:
            self.get_logger().warn("SIMULATE=TRUE: running without GPIO.")
        else:
            try:
                GPIO.setmode(GPIO.BCM)
                GPIO.setwarnings(False)
                GPIO.setup(self.dir_pin, GPIO.OUT, initial=GPIO.LOW)
                GPIO.setup(self.pwm_pin, GPIO.OUT, initial=GPIO.LOW)
                self.pwm = GPIO.PWM(self.pwm_pin, self.pwm_hz)
                self.pwm.start(0.0)
                self.get_logger().info(
                    f"GPIO ready: DIR=BCM{self.dir_pin}, PWM=BCM{self.pwm_pin}, "
                    f"{self.pwm_hz} Hz, duty {self.test_duty:.1f}% when ON, "
                    f"FL_POL={self.fl_pol}, dir_active_high={self.dir_active_high}"
                )
            except Exception as e:
                self.get_logger().error(f"GPIO init failed, switching to SIM: {e}")
                self.simulate = True

        self.sub = self.create_subscription(Int8, 'fl_cmd', self.on_cmd, 10)
        self.get_logger().info("Listening on /fl_cmd (1=FWD, -1=BWD, 0=STOP)")

    def on_cmd(self, msg: Int8):
        raw = int(msg.data)
        # Apply front-left polarity exactly like ESP32: fl_cmd *= FL_POL
        effective = raw * self.fl_pol

        if self.simulate:
            if   effective > 0: self.get_logger().info("[SIM] FL: FORWARD")
            elif effective < 0: self.get_logger().info("[SIM] FL: BACKWARD")
            else:               self.get_logger().info("[SIM] FL: STOP")
            return

        if effective == 0:
            self._set_duty(0.0)
            self.get_logger().info("FL: STOP")
            return

        forward = (effective > 0)
        self._set_dir(forward)
        # tiny settle to avoid some drivers missing the change
        time.sleep(0.003)
        self._set_duty(self.test_duty)
        self.get_logger().info("FL: FORWARD" if forward else "FL: BACKWARD")

    def _set_dir(self, forward: bool):
        # "Forward" before FL_POL is dir_active_high; we already applied FL_POL above
        if self.dir_active_high:
            level = GPIO.HIGH if forward else GPIO.LOW
        else:
            level = GPIO.LOW if forward else GPIO.HIGH
        GPIO.output(self.dir_pin, level)

    def _set_duty(self, duty: float):
        duty = max(0.0, min(100.0, float(duty)))
        self.pwm.ChangeDutyCycle(duty)

    def destroy_node(self):
        if not self.simulate:
            try:
                self._set_duty(0.0)
                self.pwm.stop()
                GPIO.output(self.dir_pin, GPIO.LOW)
                GPIO.cleanup([self.dir_pin, self.pwm_pin])
            except Exception:
                pass
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    n = MotorTestNode()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
