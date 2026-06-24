/*
 * Name: CalibrationManager.h
 * Use: Declaration for the calibration state machine used for ESC and IMU calibration.
 * Version: 4.0.0
 * Created by: Durvesh Pathak dp676@cornell.edu
 */

/**
 * ============================================================
 *  CalibrationManager.h  —  Application Calibration Manager (v4.0)
 *  Adafruit HUZZAH32 Feather (ESP32-WROOM-32E)
 *  Central calibration state machine for ESC + IMU calibration
 *
 *  Goal:
 *    - Web UI and RC controller use the same calibration path
 *    - No calibration logic runs directly inside web handlers
 *    - Calibration only runs when safe/disarmed
 *  Created By - Durvesh Pathak
 * ============================================================
 **/

#pragma once
#ifndef CALIBRATION_MANAGER_H
#define CALIBRATION_MANAGER_H

#include <Arduino.h>
#include "../IMU/MPU9250.h"

// ─────────────────────────────────────────────────────────────
// Calibration modes
// ─────────────────────────────────────────────────────────────
enum class CalibrationMode : uint8_t {
    NONE = 0,

    ESC,              // ESC calibration
    GYRO_BIAS,        // Static gyro bias calibration
    ACCEL_6_FACE,     // Six-face accelerometer calibration
    MAG_MINMAX,       // Magnetometer hard/soft iron min/max calibration
    IMU_ALL_GUIDED    // Guided gyro + accel + mag sequence
};

// ─────────────────────────────────────────────────────────────
// Calibration state
// ─────────────────────────────────────────────────────────────
enum class CalibrationState : uint8_t {
    IDLE = 0,

    REQUESTED,
    WAITING_FOR_SAFE,
    WAITING_FOR_STILLNESS,
    WAITING_FOR_USER_STEP,
    COLLECTING,
    COMPUTING,
    SAVING,
    DONE,
    FAILED,
    CANCELLED
};

// ─────────────────────────────────────────────────────────────
// Calibration request source
// ─────────────────────────────────────────────────────────────
enum class CalibrationSource : uint8_t {
    NONE = 0,
    WEB,
    RC
};

// ─────────────────────────────────────────────────────────────
// Calibration status for telemetry / web UI
// ─────────────────────────────────────────────────────────────
struct CalibrationStatus {
    CalibrationMode mode;
    CalibrationState state;
    CalibrationSource source;

    uint32_t runId;
    uint32_t startedMs;
    uint32_t updatedMs;

    uint8_t step;
    uint8_t totalSteps;

    uint16_t samplesCollected;
    uint16_t validSamples;
    uint16_t rejectedSamples;

    float progress;     // 0.0 to 1.0
    float quality;      // 0.0 to 1.0

    bool active;
    bool success;
    bool safeToRun;
    bool requiresUserConfirm;
    bool canCancel;

    char message[96];
    char error[96];
};

// ─────────────────────────────────────────────────────────────
// CalibrationManager class
// ─────────────────────────────────────────────────────────────
class CalibrationManager {
public:
    struct AccelPose {
        const char* label;
        uint8_t axis;
        int8_t sign;
    };

    static constexpr uint8_t ACCEL_POSE_COUNT = 6;

    CalibrationManager();

    void begin(MPU9250& imu);
    void reset();

    bool request(CalibrationMode mode, CalibrationSource source);
    void cancel();
    void confirmStep();
    void setSafety(bool safeToRun);
    void attachMotorOutputs(void (*writeMotors)(float, float, float, float),
                            void (*motorsOff)()) {
        _writeMotors = writeMotors;
        _motorsOff = motorsOff;
    }

    void update();

    CalibrationStatus status() const;

    bool isActive() const {
        return _status.active;
    }

    bool shouldBlockFlight() const {
        return _status.active;
    }

    bool ownsMotors() const {
        return _ownsMotors;
    }

private:
    CalibrationStatus _status;
    uint32_t _nextRunId = 1;

    MPU9250* _imu = nullptr;
    void (*_writeMotors)(float, float, float, float) = nullptr;
    void (*_motorsOff)() = nullptr;
    bool _ownsMotors = false;

    // ── Gyro calibration working data ────────────────────────
    uint32_t _gyroStartMs = 0;
    uint32_t _lastSampleMs = 0;

    double _gyroSumX = 0.0;
    double _gyroSumY = 0.0;
    double _gyroSumZ = 0.0;

    float _gyroMinX = 0.0f;
    float _gyroMaxX = 0.0f;
    float _gyroMinY = 0.0f;
    float _gyroMaxY = 0.0f;
    float _gyroMinZ = 0.0f;
    float _gyroMaxZ = 0.0f;

    uint16_t _gyroTargetSamples = 0;

    // ── Accel 6-face calibration working data ────────────────
    float _accelPlus[3]  = {0.0f, 0.0f, 0.0f};
    float _accelMinus[3] = {0.0f, 0.0f, 0.0f};

    bool _accelGotPlus[3]  = {false, false, false};
    bool _accelGotMinus[3] = {false, false, false};

    uint8_t _accelPoseIndex = 0;
    uint16_t _accelTargetSamples = 120;

    // ── Magnetometer min/max calibration working data ────────
    uint32_t _magStartMs = 0;
    uint32_t _magDurationMs = 30000;
    uint32_t _lastMagPrintMs = 0;

    float _magMinX = 0.0f;
    float _magMaxX = 0.0f;
    float _magMinY = 0.0f;
    float _magMaxY = 0.0f;
    float _magMinZ = 0.0f;
    float _magMaxZ = 0.0f;

    uint16_t _magValidSamples = 0;

    bool _confirmRequested = false;

    // ── Internal helpers ─────────────────────────────────────
    void _startGyroCalibration();
    void _updateGyroCalibration();

    void _startAccelCalibration();
    void _captureAccelPose();
    void _finishAccelCalibration();

    void _startMagCalibration();
    void _updateMagCalibration();

    void _finishFullCalibration();
};

#endif // CALIBRATION_MANAGER_H
