"""
mavlink.servo_controller testleri — PayloadLock, LegacyServoController ve
DualPayloadServoController. Gerçek donanım gerekmez; `set_servo_pwm`'in
gönderdiği MAVLink komutları sahte (fake) bir `master.mav` nesnesiyle
kaydedilir.
"""

from types import SimpleNamespace

import pytest

import config
from mavlink.servo_controller import DualPayloadServoController, PayloadLock, set_servo_pwm


class _RecordingMav:
    def __init__(self):
        self.sent = []

    def command_long_send(self, target_system, target_component, command, confirmation,
                           p1, p2, p3, p4, p5, p6, p7):
        self.sent.append((command, p1, p2))


class _FakeMaster:
    def __init__(self):
        self.mav = _RecordingMav()


def test_payload_lock_defaults_available():
    lock = PayloadLock()
    assert lock.blue_payload_available is True
    assert lock.red_payload_available is True


def test_payload_lock_consume_is_one_shot():
    lock = PayloadLock()

    assert lock.consume("BLUE") is True
    assert lock.blue_payload_available is False
    assert lock.consume("BLUE") is False  # ikinci kez tüketilemez


def test_payload_lock_reset_restores_availability():
    lock = PayloadLock()
    lock.consume("RED")
    lock.reset()

    assert lock.red_payload_available is True


def test_dual_payload_servo_controller_constructs_with_real_config():
    """
    config.py'deki SERVO_A/B_* değerleri artık (kullanıcı isteğiyle) dolu
    placeholder değerler — bu yüzden DualPayloadServoController artık
    RuntimeError vermeden kurulabilmeli.
    """
    master = _FakeMaster()
    controller = DualPayloadServoController(master, target_system=1, target_component=1, config_module=config)

    assert controller is not None


def test_dual_payload_servo_controller_rejects_incomplete_config():
    fake_cfg = SimpleNamespace(
        SERVO_A_BLUE_PAYLOAD_NO=8,
        SERVO_B_RED_PAYLOAD_NO=None,  # eksik
        SERVO_A_HOME_PWM=1000,
        SERVO_A_RELEASE_PWM=1250,
        SERVO_B_HOME_PWM=1000,
        SERVO_B_RELEASE_PWM=1250,
    )

    with pytest.raises(RuntimeError):
        DualPayloadServoController(_FakeMaster(), 1, 1, fake_cfg)


def test_release_red_target_triggers_servo_a_with_blue_payload():
    master = _FakeMaster()
    controller = DualPayloadServoController(master, 1, 1, config)

    released = controller.release("RED")

    assert released is True
    assert controller.payload_lock.blue_payload_available is False
    assert controller.payload_lock.red_payload_available is True

    from pymavlink import mavutil
    assert master.mav.sent == [
        (mavutil.mavlink.MAV_CMD_DO_SET_SERVO, config.SERVO_A_BLUE_PAYLOAD_NO, config.SERVO_A_RELEASE_PWM)
    ]


def test_release_blue_target_triggers_servo_b_with_red_payload():
    master = _FakeMaster()
    controller = DualPayloadServoController(master, 1, 1, config)

    released = controller.release("BLUE")

    assert released is True
    assert controller.payload_lock.red_payload_available is False
    assert controller.payload_lock.blue_payload_available is True

    from pymavlink import mavutil
    assert master.mav.sent == [
        (mavutil.mavlink.MAV_CMD_DO_SET_SERVO, config.SERVO_B_RED_PAYLOAD_NO, config.SERVO_B_RELEASE_PWM)
    ]


def test_release_same_payload_twice_is_denied():
    master = _FakeMaster()
    controller = DualPayloadServoController(master, 1, 1, config)

    assert controller.release("RED") is True
    assert controller.release("RED") is False  # mavi yük zaten tüketildi
    assert len(master.mav.sent) == 1  # ikinci denemede PWM gönderilmedi


def test_set_servo_pwm_sends_expected_command():
    master = _FakeMaster()
    set_servo_pwm(master, target_system=1, target_component=1, servo_no=8, pwm=1250)

    from pymavlink import mavutil
    assert master.mav.sent == [(mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 8, 1250)]
