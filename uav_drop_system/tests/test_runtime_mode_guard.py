"""
Codex review takibi: LEGACY_SINGLE_SERVO_MODE guard'i ve tek-servo
varsayilanlarinin donanim gerekmeden dogrulanmasi.

- config.LEGACY_SINGLE_SERVO_MODE varsayilan olarak True olmali (eski
  davranisin degismeden calismaya devam etmesi bunu gerektirir).
- main._check_runtime_mode(cfg), LEGACY_SINGLE_SERVO_MODE=False oldugunda
  acik bir RuntimeError vermeli; True oldugunda sessizce gecmeli. Bu
  fonksiyon main.py'de picamera2/pymavlink gerektiren hicbir seyi
  tetiklemeden cagrilabilir (import main, hicbir hardware baglantisi
  acmaz).
- LegacyServoController, config.SERVO_NO/SERVO_HOME_PWM/SERVO_45_PWM
  degerlerini oldugu gibi kullanmali (8 / 1000 / 1250).
- main.py'nin aktif calisma yolu (kaynak metni), DualPayloadServoController'i
  hic import etmemeli/kullanmamali - dual-servo mimarisi hazir ama devrede
  degil.
"""

import ast
from types import SimpleNamespace

import pytest

import config
import main
from mavlink.servo_controller import LegacyServoController


def test_legacy_single_servo_mode_default_is_true():
    assert config.LEGACY_SINGLE_SERVO_MODE is True


def test_check_runtime_mode_raises_when_legacy_mode_disabled():
    fake_cfg = SimpleNamespace(LEGACY_SINGLE_SERVO_MODE=False)

    with pytest.raises(RuntimeError, match="LEGACY_SINGLE_SERVO_MODE=True"):
        main._check_runtime_mode(fake_cfg)


def test_check_runtime_mode_passes_when_legacy_mode_enabled():
    fake_cfg = SimpleNamespace(LEGACY_SINGLE_SERVO_MODE=True)

    # Hata firlatmamali.
    main._check_runtime_mode(fake_cfg)


def test_check_runtime_mode_passes_with_real_config():
    # Gercek config.py ile de (varsayilan True) guard sorunsuz gecmeli.
    main._check_runtime_mode(config)


def test_legacy_servo_controller_uses_config_defaults():
    servo = LegacyServoController(
        master=None,
        target_system=None,
        target_component=None,
        servo_no=config.SERVO_NO,
        home_pwm=config.SERVO_HOME_PWM,
        active_pwm=config.SERVO_45_PWM,
    )

    assert servo.servo_no == 8
    assert servo.home_pwm == 1000
    assert servo.active_pwm == 1250
    assert servo.servo_is_active is False


def test_main_does_not_import_or_instantiate_dual_payload_controller():
    """
    Statik (AST tabanli) kontrol: main.py, DualPayloadServoController'i ne
    import ediyor ne de olusturuyor olmali — bu, dual-servo mimarisinin
    aktif calisma yoluna yanlislikla dahil edilmedigini dogrular (bkz.
    config.LEGACY_SINGLE_SERVO_MODE). Docstring/yorumlarda adin gecmesi
    (aciklama amacli) burada sorun degil; yalnizca gercek import/kullanim
    kontrol ediliyor.
    """
    with open(main.__file__, "r", encoding="utf-8") as f:
        source = f.read()

    tree = ast.parse(source, filename=main.__file__)

    imported_names = set()

    for node in ast.walk(tree):
        if isinstance(node, (ast.Import, ast.ImportFrom)):
            for alias in node.names:
                imported_names.add(alias.asname or alias.name)

    assert "DualPayloadServoController" not in imported_names

    instantiations = [
        node for node in ast.walk(tree)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Name)
        and node.func.id == "DualPayloadServoController"
    ]

    assert instantiations == []
