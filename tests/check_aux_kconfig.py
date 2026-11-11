#!/usr/bin/env python3
"""Resolve AUX options without changing the project's saved sdkconfig."""
from pathlib import Path
from tempfile import TemporaryDirectory

from esp_kconfiglib import Kconfig


root = Path(__file__).resolve().parents[1]
with TemporaryDirectory(prefix="glove-aux-kconfig-") as directory:
    top = Path(directory, "Kconfig")
    top.write_text(
        'config IDF_TARGET_ESP32C3\n    bool "C3"\n'
        'config IDF_TARGET_ESP32S3\n    bool "S3"\n'
        f'source "{root / "main/Kconfig.projbuild"}"\n'
    )
    for target, board, limit in (("ESP32C3", "ESP32_C3", 3), ("ESP32S3", "ESP32_S3", 8)):
        config = Kconfig(str(top), warn=False)
        config.syms[f"IDF_TARGET_{target}"].set_value("y")
        config.syms[f"TARGET_BOARD_XIAO_{board}"].set_value("y")
        count = config.syms["AUX_ADC_CHANNEL_COUNT"]
        assert count.str_value == str(limit)
        for index in range(8):
            pin = config.syms[f"AUX_ADC_GPIO_{index}"]
            assert bool(pin.visibility) == (index < limit)
        if target == "ESP32C3":
            # Old eight-channel configurations cannot make extra C3 inputs visible.
            count.set_value("8")
            config.syms["AUX_ADC_GPIO_7"].set_value("9")
            assert count.str_value == "3"
            assert not config.syms["AUX_ADC_GPIO_7"].visibility
            pin = config.syms["AUX_ADC_GPIO_0"]
            pin.set_value("48")
            assert pin.str_value == "-1"
        for active in range(1, limit + 1):
            count.set_value(str(active))
            assert count.str_value == str(active)
            for index in range(8):
                assert bool(config.syms[f"AUX_ADC_GPIO_{index}"].visibility) == (index < active)
        config.syms["ADC_DUMMY"].set_value("y")
        assert count.str_value == str(limit)
        assert all(not config.syms[f"AUX_ADC_GPIO_{i}"].visibility for i in range(8))
        config.syms["AUX_ADC_ENABLED"].set_value("n")
        assert not count.visibility
        assert count.str_value == ""
        print(f"PASS: {target} AUX count, pin visibility, dummy and disabled options")
