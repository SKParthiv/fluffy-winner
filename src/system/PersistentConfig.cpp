/**
 * PersistentConfig.cpp — see header. Thin wrapper over Preferences/NVS.
 * Kept as its own module so the storage choice can change without any
 * UI or controller code knowing about it.
 */

#include "PersistentConfig.h"

#if defined(ARDUINO_ARCH_ESP32)
#include <Preferences.h>

static const char* kNamespace = "fluffy";

bool PersistentConfig::load(RobotConfig& cfg) {
    Preferences prefs;
    if (!prefs.begin(kNamespace, true /*read-only*/)) {
        return false;  // no stored config yet: defaults stay in place
    }

    // getFloat(key, default) returns the default when the key is absent,
    // which is exactly the "boot without prior calibration" requirement.
    cfg.controller.gains.kp =
        prefs.getFloat("kp", cfg.controller.gains.kp);
    cfg.controller.gains.ki =
        prefs.getFloat("ki", cfg.controller.gains.ki);
    cfg.controller.gains.kd =
        prefs.getFloat("kd", cfg.controller.gains.kd);
    cfg.controller.integralEnabled =
        prefs.getBool("integ", cfg.controller.integralEnabled);
    cfg.controller.targetPosition =
        prefs.getFloat("target", cfg.controller.targetPosition);
    cfg.controller.forwardVelocity =
        prefs.getFloat("vfwd", cfg.controller.forwardVelocity);

    prefs.end();
    return true;
}

bool PersistentConfig::save(const RobotConfig& cfg) {
    Preferences prefs;
    if (!prefs.begin(kNamespace, false /*read-write*/)) {
        return false;
    }
    prefs.putFloat("kp", cfg.controller.gains.kp);
    prefs.putFloat("ki", cfg.controller.gains.ki);
    prefs.putFloat("kd", cfg.controller.gains.kd);
    prefs.putBool("integ", cfg.controller.integralEnabled);
    prefs.putFloat("target", cfg.controller.targetPosition);
    prefs.putFloat("vfwd", cfg.controller.forwardVelocity);
    prefs.end();
    return true;
}

bool PersistentConfig::clear() {
    Preferences prefs;
    if (!prefs.begin(kNamespace, false)) return false;
    prefs.clear();   // remove every key in this namespace
    prefs.end();
    return true;
}

#else  // host build: persistence is an ESP32 feature, stub it out

bool PersistentConfig::load(RobotConfig&) { return false; }
bool PersistentConfig::save(const RobotConfig&) { return false; }
bool PersistentConfig::clear() { return false; }

#endif
