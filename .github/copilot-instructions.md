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
- `main/main.c` — app entry point, initializes tasks, glue between sensors and OpenThread.
- `main/openthread.c` — OpenThread initialization, state callbacks, optional joiner logic.
- `main/openthread_manager.c` — manages OpenThread task launch and mainloop.
- `main/sensors.c` — I2C init, SCD41 start/read helpers, sensor polling task.
- `main/CMakeLists.txt` — component registration and target override (`set(IDF_TARGET "esp32c6")`).
- `CMakeLists.txt` (root) — standard ESP-IDF project bootstrap (uses $IDF_PATH tooling).
- `sdkconfig`, `sdkconfig.defaults` — project build-time config flags (OpenThread features like `CONFIG_OPENTHREAD_AUTO_START`, CLI options, log level). Prefer reading these rather than hardcoding behavior.

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
- Single-purpose `main.c` — prefer adding small helper functions or new components alongside existing patterns.
- FreeRTOS threading: long-running subsystems (OpenThread) are launched in their own tasks (`xTaskCreate(ot_task_worker, ...)`) with stack size ~8 KB and priority 5.
- Hardware pin constants are in `main.c` (I2C pins: SDA=21, SCL=22). New hardware features go near the top of `main.c` or in a new header under `main/`.
- Config-first behavior: feature toggles use `sdkconfig` flags. Prefer enabling/disabling via sdkconfig, not by editing runtime code.
- Sensor polling loops should use `vTaskDelay(pdMS_TO_TICKS(x))` for timing; avoid busy loops.

Integration points & external deps
---------------------------------
- Depends on ESP-IDF tree (look under `esp-idf/` in the workspace). OpenThread integration uses `esp_openthread_*` APIs and `esp_netif` glue.
- I2C driver (`driver/i2c.h`) and SCD41 commands are in `sensors.c`. SCD41 is polled every 2 seconds.
- NVS is required by OpenThread — `nvs_flash_init()` is called at startup.

Safety and small-change rules for AI agents
-----------------------------------------
- **Do not modify any files under `components/` unless explicitly instructed.**
- Never modify `sdkconfig` without confirming: changes alter build-time behavior.
- Keep default PSKd and secrets out of commits; use environment variables or NVS storage for secrets.
- Always check return codes from `esp_err_t` functions (i2c_init, scd4x_read_measurement, etc.).
- Log meaningful messages on failures; avoid crashing tasks.
- Follow existing I2C bus recovery patterns for safety.
- Use C11 for C files and C++17 for C++ files.

Code organization guidelines
---------------------------
- New sensor drivers go in `main/sensors/`, one file per device if possible.
- OpenThread helpers go in `main/openthread/`.
- `main.c` should only initialize tasks and glue subsystems together.
- Do not hardcode delays or magic numbers; use `pdMS_TO_TICKS(x)` for FreeRTOS delays.
- Follow existing logging conventions (`ESP_LOGI/W/E`).

Quick examples (use these as templates)
-------------------------------------
- Add a new polled sensor: copy `i2c_master_init()` and `scd41_read_measurement()` pattern, use `vTaskDelay(pdMS_TO_TICKS(x))` for delays and `ESP_LOGI/W` macros for logs.
- Add OpenThread CLI command: follow `esp_openthread_cli_init()` usage in `openthread.c` and place command registration in `main/`.

What I might ask you next
-------------------------
- Serial port to use for flashing/monitoring on your machine.
- Any CI flasher scripts or private tooling not present in the repo.
- Whether we should remove the hardcoded `JOINER_PSKD` or wire it to secure storage.
