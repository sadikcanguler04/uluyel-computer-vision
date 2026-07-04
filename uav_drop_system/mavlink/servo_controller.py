"""
Servo tetikleme.

old_docs/vision_servo_trigger.py::set_servo_pwm/move_servo_to_45/move_servo_home
davranışı `LegacyServoController` içinde birebir korunmuştur (tek servo,
tersinir "aç/kapa" debounce — bkz. proje_spec_uluyel_v2.md §2.4).

`PayloadLock` ve `DualPayloadServoController`, iki bağımsız servo için
HAZIRLIK (prep) amaçlıdır. Mission-scoped "bir kez ve asla tekrar değil"
kilidini, eski `servo_is_active`'in tersinir debounce'undan bilinçli olarak
ayırır: bir payload bir kez bırakıldığında, `PayloadLock.reset()`
çağrılmadan (yalnızca yeni görev başlangıcında / IDLE durumunda) bir daha
asla tekrar bırakılamaz.

`config.LEGACY_SINGLE_SERVO_MODE = True` iken main.py yalnızca
`LegacyServoController`'ı kullanır; `DualPayloadServoController` sahada
gerçek servo kanal/PWM değerleri doğrulanıp config.py'de doldurulana kadar
etkin çalışma yoluna dahil edilmez.
"""


def set_servo_pwm(master, target_system, target_component, servo_no, pwm):
    """old_docs/vision_servo_trigger.py::set_servo_pwm'in taşınmış hâli."""
    from pymavlink import mavutil

    print(f"[SERVO] Servo {servo_no} -> PWM {pwm}")

    master.mav.command_long_send(
        target_system,
        target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
        0,
        servo_no,
        pwm,
        0,
        0,
        0,
        0,
        0
    )


class LegacyServoController:
    """Eski çalışan tek-servo davranışı (SERVO_NO=8, tersinir debounce)."""

    def __init__(self, master, target_system, target_component, servo_no, home_pwm, active_pwm):
        self.master = master
        self.target_system = target_system
        self.target_component = target_component
        self.servo_no = servo_no
        self.home_pwm = home_pwm
        self.active_pwm = active_pwm

        self.servo_is_active = False

    def _send(self, pwm):
        set_servo_pwm(self.master, self.target_system, self.target_component, self.servo_no, pwm)

    def move_to_active(self):
        if not self.servo_is_active:
            print("[ACTION] Hedef doğrulandı. Servo 45 derece pozisyona gidiyor.")
            self._send(self.active_pwm)
            self.servo_is_active = True

    def move_home(self):
        if self.servo_is_active:
            print("[ACTION] Hedef uzun süre kayıp. Servo başlangıç pozisyonuna dönüyor.")
            self._send(self.home_pwm)
            self.servo_is_active = False

    def force_home(self):
        """Başlangıçta / kapanışta koşulsuz olarak home pozisyonuna gönderir."""
        self._send(self.home_pwm)
        self.servo_is_active = False


class PayloadLock:
    """Mission-scoped one-shot kilit: bir kez release edilen payload asla tekrar edilemez."""

    def __init__(self):
        self.blue_payload_available = True
        self.red_payload_available = True

    def is_available(self, color):
        if color == "BLUE":
            return self.blue_payload_available
        if color == "RED":
            return self.red_payload_available
        raise ValueError(f"Bilinmeyen payload rengi: {color}")

    def consume(self, color):
        """Payload'ı kalıcı olarak tüketir. Zaten tüketilmişse False döner."""
        if not self.is_available(color):
            return False

        if color == "BLUE":
            self.blue_payload_available = False
        elif color == "RED":
            self.red_payload_available = False
        else:
            raise ValueError(f"Bilinmeyen payload rengi: {color}")

        return True

    def reset(self):
        """Yalnızca yeni görev başlangıcında (IDLE) çağrılmalıdır."""
        self.blue_payload_available = True
        self.red_payload_available = True


class DualPayloadServoController:
    """
    YENİ mimari (prep): renk bazlı iki bağımsız servo + mission-scoped one-shot kilit.

    config.SERVO_A_BLUE_PAYLOAD_NO / SERVO_B_RED_PAYLOAD_NO ve ilgili PWM
    değerleri sahada doğrulanıp config.py'de doldurulmadan bu sınıf
    kullanılamaz (kanal numarası tahmin edilmez, açıkça hata verir).
    """

    def __init__(self, master, target_system, target_component, config_module, payload_lock=None):
        required = [
            "SERVO_A_BLUE_PAYLOAD_NO",
            "SERVO_B_RED_PAYLOAD_NO",
            "SERVO_A_HOME_PWM",
            "SERVO_A_RELEASE_PWM",
            "SERVO_B_HOME_PWM",
            "SERVO_B_RELEASE_PWM",
        ]
        missing = [name for name in required if getattr(config_module, name, None) is None]

        if missing:
            raise RuntimeError(
                "DualPayloadServoController kullanılamaz: şu config değerleri henüz "
                f"sahada doğrulanmadı ve None: {missing}. "
                "config.LEGACY_SINGLE_SERVO_MODE=True kaldıkça bu sınıf devreye girmez."
            )

        self.master = master
        self.target_system = target_system
        self.target_component = target_component
        self.config = config_module
        self.payload_lock = payload_lock if payload_lock is not None else PayloadLock()

    def _send(self, servo_no, pwm):
        set_servo_pwm(self.master, self.target_system, self.target_component, servo_no, pwm)

    def release(self, target_color):
        """
        target_color: tespit edilen HEDEFİN rengi ("RED" veya "BLUE").
        Kırmızı hedef -> mavi yük (Servo A), mavi hedef -> kırmızı yük (Servo B).
        Zaten bırakılmış bir payload için tekrar tetiklemez, False döner.
        """
        if target_color == "RED":
            payload_color = "BLUE"
            servo_no = self.config.SERVO_A_BLUE_PAYLOAD_NO
            release_pwm = self.config.SERVO_A_RELEASE_PWM
        elif target_color == "BLUE":
            payload_color = "RED"
            servo_no = self.config.SERVO_B_RED_PAYLOAD_NO
            release_pwm = self.config.SERVO_B_RELEASE_PWM
        else:
            raise ValueError(f"Bilinmeyen hedef rengi: {target_color}")

        if not self.payload_lock.consume(payload_color):
            print(f"[SERVO] {payload_color} payload zaten bırakılmış, tekrar tetiklenmedi.")
            return False

        self._send(servo_no, release_pwm)
        return True
