<!-- Copilot / AI agent instructions for esp_CO2_thread_sensor -->

Purpose
-------
This file tells an AI coding assistant how the repo is organized and the specific patterns, build steps, and integration points to follow so changes are immediately useful and safe.

Big picture (what this project is)
---------------------------------
- ESP-IDF C project for an ESP32C6-based device that reads an SCD41 CO2 sensor over I2C and runs an OpenThread (Thread mesh) stack. The device:
  - Initializes NVS and I2C, starts SCD41 periodic measurement (I2C addr 0x62).
  - Runs OpenThread in a dedicated FreeRTOS task and exposes the OT CLI when configured.
  - Logs sensor readings periodically from `main.c`.

Primary files and components to inspect
--------------------------------------
- `main/main.c` — single-file app logic: I2C init (`i2c_master_init()`), SCD41 helpers (`scd41_start_measurement()` and `scd41_read_measurement()`), OpenThread task (`ot_task_worker`) and `app_main()` loop. Use this as the canonical example for sensor and OT integration.
- `main/CMakeLists.txt` — component registration and target override (`set(IDF_TARGET "esp32c6")`).
- `CMakeLists.txt` (root) — standard ESP-IDF project bootstrap (uses $IDF_PATH tooling).
- `sdkconfig`, `sdkconfig.defaults` — project build-time config flags (OpenThread features like `CONFIG_OPENTHREAD_AUTO_START`, CLI options, log level). Prefer reading these rather than hardcoding behavior.
- `main/esp_ot_config.h` — OpenThread-related compile-time glue (check before modifying OT behavior).

Build / flash / debug (concrete commands)
---------------------------------------
Assume ESP-IDF tools are installed and `$IDF_PATH` is set.
- Set target (optional):
  - idf.py set-target esp32c6
- Build the firmware:
  - idf.py build
- Flash to device and monitor logs (replace /dev/tty.usbserial-XXXX with your serial):
  - idf.py -p /dev/tty.usbserial-XXXX flash monitor
  - Use Ctrl-] to exit monitor.
- View logs only (no flash):
  - idf.py -p /dev/tty.usbserial-XXXX monitor

Project-specific conventions & patterns
-------------------------------------
- Single-purpose `main.c` — prefer adding small helper functions or new components alongside existing patterns (e.g., follow `i2c_write`/`i2c_read` helper style).
- FreeRTOS threading: long-running subsystems (OpenThread) are launched in their own tasks (`xTaskCreate(ot_task_worker, ...)`) with their own stack size and priority. New subsystems should follow this model.
- Hardware pin constants are in `main.c` (I2C pins: SDA=21, SCL=22). If adding new hardware features, add constants near the top of `main.c` or in a new header under `main/`.
- Config-first behavior: feature toggles use `sdkconfig` flags (e.g., `CONFIG_OPENTHREAD_CLI`). Prefer enabling/disabling via sdkconfig options, not by editing runtime code.

Integration points & external deps
---------------------------------
- Depends on ESP-IDF tree (look under `esp-idf/` in the workspace). OpenThread integration uses the `esp_openthread_*` APIs and `esp_netif` glue.
- I2C driver (`driver/i2c.h`) and SCD41 commands are in `main.c`. SCD41 is polled every 2 seconds in `app_main()`.
- NVS is required by OpenThread — `nvs_flash_init()` is called at startup.

Safety and small-change rules for AI agents
-----------------------------------------
- Never modify `sdkconfig` without confirming: changes alter build-time behavior. Prefer suggesting edits and asking the human to apply them.
- Keep default PSKd and secrets out of commits. `JOINER_PSKD` is currently a literal in `main.c` — do not change to a real secret in repo; instead, suggest environment or secure storage approaches.
- Preserve `IDF_TARGET` in `main/CMakeLists.txt` unless the user asks to retarget hardware.

Quick examples (use these as templates)
-------------------------------------
- Add a new polled sensor: copy `i2c_master_init()` and `scd41_read_measurement()` pattern, use `vTaskDelay(pdMS_TO_TICKS(x))` for delays and `ESP_LOGI/W` macros for logs.
- Add OpenThread CLI command: follow `esp_openthread_cli_init()` usage in `ot_task_worker` and place command registration in `main/`.

What I might ask you next
-------------------------
- Serial port to use for flashing/monitoring on your machine.
- Any CI flasher scripts or private tooling not present in the repo.
- Whether we should remove the hardcoded `JOINER_PSKD` or wire it to secure storage.

If anything here is incomplete or you want additional rules (commit message format, CI, tests), tell me what to add and I will update this file.
