/**
 * PersistentConfig.h
 * ==================
 * Non-volatile storage for user-confirmed configuration (task: "changes
 * should be persisted only after the user confirms the edit").
 *
 * Mechanism: ESP32 NVS via the Arduino `Preferences` library — the simplest
 * appropriate persistence on this platform (no filesystem, no manual
 * partition handling; the default Arduino ESP32 partition table already
 * contains an NVS partition).
 *
 * Contract:
 *   - load() reads the stored values into a RobotConfig at boot; missing
 *     keys fall back to the compiled-in defaults (the robot must boot and
 *     run WITHOUT any prior calibration or saved config — calibration is
 *     never a boot-time dependency).
 *   - save() is ONLY called when the user confirms an edit in the UI.
 *   - The stored set is deliberately minimal: PID gains, integral flag,
 *     target position, forward velocity, system ON/OFF state.
 */

#ifndef PERSISTENT_CONFIG_H
#define PERSISTENT_CONFIG_H

#include "../config/RobotConfig.h"

class PersistentConfig {
public:
    /**
     * Load saved values over `cfg` (in place). Keys that were never saved
     * keep the compiled-in defaults. Returns false only if NVS could not
     * be opened at all — in that case the defaults simply remain.
     */
    static bool load(RobotConfig& cfg);

    /// Write the current controller-relevant values to NVS.
    /// Call ONLY on user-confirmed edits (UI SELECT on a value screen).
    static bool save(const RobotConfig& cfg);

    /// Erase ALL saved values (UI "RESET" -> factory defaults on next boot).
    static bool clear();
};

#endif // PERSISTENT_CONFIG_H
