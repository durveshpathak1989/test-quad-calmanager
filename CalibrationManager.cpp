/**
 * ============================================================
 *  CalibrationManager.cpp  —  Application Calibration Manager (v4.0)
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

#include "CalibrationManager.h"
#include "../DebugConfig/DebugConfig.h"

// Physical pose list.
// axis: 0=X, 1=Y, 2=Z
// sign: expected sensor sign for that pose
static const CalibrationManager::AccelPose ACCEL_POSES[CalibrationManager::ACCEL_POSE_COUNT] = {
    { "NOSE UP",       0, +1 },
    { "NOSE DOWN",     0, -1 },
    { "LEFT SIDE UP",  1, +1 },
    { "RIGHT SIDE UP", 1, -1 },
    { "FLAT TOP UP",   2, +1 },
    { "UPSIDE DOWN",   2, -1 }
};

CalibrationManager::CalibrationManager()
{
    reset();
}

void CalibrationManager::begin(MPU9250& imu)
{
    _imu = &imu;
    reset();
}

void CalibrationManager::reset()
{
    _status.mode = CalibrationMode::NONE;
    _status.state = CalibrationState::IDLE;
    _status.source = CalibrationSource::NONE;

    _status.step = 0;
    _status.totalSteps = 0;

    _status.samplesCollected = 0;
    _status.validSamples = 0;
    _status.rejectedSamples = 0;

    _status.progress = 0.0f;
    _status.quality = 0.0f;

    _status.active = false;
    _status.success = false;
    _status.safeToRun = false;
    _status.requiresUserConfirm = false;
    _status.canCancel = false;

    _gyroStartMs = 0;
    _lastSampleMs = 0;

    _gyroSumX = 0.0;
    _gyroSumY = 0.0;
    _gyroSumZ = 0.0;

    _gyroMinX = _gyroMaxX = 0.0f;
    _gyroMinY = _gyroMaxY = 0.0f;
    _gyroMinZ = _gyroMaxZ = 0.0f;

    _gyroTargetSamples = 0;

    for (int i = 0; i < 3; i++) {
        _accelPlus[i] = 0.0f;
        _accelMinus[i] = 0.0f;
        _accelGotPlus[i] = false;
        _accelGotMinus[i] = false;
    }

    _accelPoseIndex = 0;
    _accelTargetSamples = 120;

    _magStartMs = 0;
    _magDurationMs = 30000;
    _lastMagPrintMs = 0;

    _magMinX = _magMaxX = 0.0f;
    _magMinY = _magMaxY = 0.0f;
    _magMinZ = _magMaxZ = 0.0f;
    _magValidSamples = 0;

    _confirmRequested = false;

    _status.error[0] = '\0';

    snprintf(_status.message, sizeof(_status.message),
             "Calibration idle");
}

bool CalibrationManager::request(CalibrationMode mode, CalibrationSource source)
{
    if (_status.active) {
        snprintf(_status.message, sizeof(_status.message),
                 "Calibration already active");
        return false;
    }

    _status.mode = mode;
    _status.state = CalibrationState::REQUESTED;
    _status.source = source;

    _status.runId = _nextRunId++;
    _status.startedMs = millis();
    _status.updatedMs = _status.startedMs;

    _status.step = 0;
    _status.totalSteps = 0;

    _status.samplesCollected = 0;
    _status.validSamples = 0;
    _status.rejectedSamples = 0;

    _status.progress = 0.0f;
    _status.quality = 0.0f;

    _status.active = true;
    _status.success = false;
    _status.safeToRun = false;
    _status.requiresUserConfirm = false;
    _status.canCancel = true;

    _confirmRequested = false;

    _status.error[0] = '\0';

    snprintf(_status.message, sizeof(_status.message),
             "Calibration requested");

    return true;
}

void CalibrationManager::cancel()
{
    _status.state = CalibrationState::CANCELLED;
    _status.active = false;
    _status.success = false;
    _status.canCancel = false;
    _status.requiresUserConfirm = false;

    _confirmRequested = false;

    snprintf(_status.message, sizeof(_status.message),
             "Calibration cancelled");
}

void CalibrationManager::confirmStep()
{
    _confirmRequested = true;
}

void CalibrationManager::setSafety(bool safeToRun)
{
    _status.safeToRun = safeToRun;
}

void CalibrationManager::update()
{
    if (!_status.active) {
        return;
    }

    _status.updatedMs = millis();

    if (_status.state == CalibrationState::REQUESTED) {
        _status.progress = 0.0f;
        _status.requiresUserConfirm = false;
        _status.canCancel = true;

        switch (_status.mode) {
            case CalibrationMode::GYRO_BIAS:
            case CalibrationMode::ACCEL_6_FACE:
            case CalibrationMode::MAG_MINMAX:
                _status.totalSteps = 1;
                break;

            case CalibrationMode::IMU_ALL_GUIDED:
                _status.totalSteps = 3;
                break;

            default:
                _status.totalSteps = 1;
                break;
        }

        _status.state = CalibrationState::WAITING_FOR_SAFE;

        snprintf(_status.message, sizeof(_status.message),
                 "[Cal Manager] Waiting for safe state");
    }

    else if (_status.state == CalibrationState::WAITING_FOR_SAFE) {
        if (_status.safeToRun) {
            _status.state = CalibrationState::WAITING_FOR_USER_STEP;
            _status.requiresUserConfirm = true;

            snprintf(_status.message, sizeof(_status.message),
                     "[Cal Manager] Safe — flip SWC to start");
        }
    }

    else if (_status.state == CalibrationState::WAITING_FOR_USER_STEP) {
        if (_confirmRequested) {

            if (_status.mode == CalibrationMode::GYRO_BIAS) {
                _startGyroCalibration();
            }

            else if (_status.mode == CalibrationMode::ACCEL_6_FACE) {
                if (_status.step == 0) {
                    _startAccelCalibration();
                } else if (_status.step == 2) {
                    _captureAccelPose();
                }
            }

            else if (_status.mode == CalibrationMode::MAG_MINMAX) {
                _startMagCalibration();
            }

            else if (_status.mode == CalibrationMode::IMU_ALL_GUIDED) {
                if (_status.step == 0) {
                    _startGyroCalibration();
                } else if (_status.step == 2) {
                    _captureAccelPose();
                }
            }

            else {
                _status.state = CalibrationState::DONE;
                _status.active = false;
                _status.success = true;
                _status.progress = 1.0f;
                _status.requiresUserConfirm = false;
                _status.canCancel = false;

                snprintf(_status.message, sizeof(_status.message),
                         "Calibration flow complete");
            }
        }
    }

    else if (_status.state == CalibrationState::COLLECTING) {
        if (_status.step == 1) {
            _updateGyroCalibration();
        } else if (_status.step == 3) {
            _updateMagCalibration();
        }
    }

    _confirmRequested = false;
}

// ─────────────────────────────────────────────────────────────
// Gyro calibration
// ─────────────────────────────────────────────────────────────
void CalibrationManager::_startGyroCalibration()
{
    _gyroStartMs = millis();
    _lastSampleMs = 0;

    _gyroSumX = 0.0;
    _gyroSumY = 0.0;
    _gyroSumZ = 0.0;

    _gyroMinX =  9999.0f;
    _gyroMaxX = -9999.0f;
    _gyroMinY =  9999.0f;
    _gyroMaxY = -9999.0f;
    _gyroMinZ =  9999.0f;
    _gyroMaxZ = -9999.0f;

    _gyroTargetSamples = 500;

    _status.state = CalibrationState::COLLECTING;
    _status.step = 1;
    _status.totalSteps = (_status.mode == CalibrationMode::IMU_ALL_GUIDED) ? 3 : 1;

    _status.samplesCollected = 0;
    _status.validSamples = 0;
    _status.rejectedSamples = 0;

    _status.progress = 0.0f;
    _status.requiresUserConfirm = false;

    snprintf(_status.message, sizeof(_status.message),
             "[Cal Manager] Gyro: keep flat and still");
}

void CalibrationManager::_updateGyroCalibration()
{
    if (_imu == nullptr) {
        _status.state = CalibrationState::FAILED;
        _status.active = false;
        _status.success = false;
        snprintf(_status.error, sizeof(_status.error),
                 "IMU pointer is null");
        snprintf(_status.message, sizeof(_status.message),
                 "Gyro calibration failed");
        return;
    }

    MPU_RawData raw;
    if (!_imu->readRaw(raw)) {
        _status.rejectedSamples++;
        return;
    }

    float gx = (float)raw.gx * GYRO_SCALE_500DPS;
    float gy = (float)raw.gy * GYRO_SCALE_500DPS;
    float gz = (float)raw.gz * GYRO_SCALE_500DPS;

    _gyroSumX += gx;
    _gyroSumY += gy;
    _gyroSumZ += gz;

    if (gx < _gyroMinX) _gyroMinX = gx;
    if (gx > _gyroMaxX) _gyroMaxX = gx;

    if (gy < _gyroMinY) _gyroMinY = gy;
    if (gy > _gyroMaxY) _gyroMaxY = gy;

    if (gz < _gyroMinZ) _gyroMinZ = gz;
    if (gz > _gyroMaxZ) _gyroMaxZ = gz;

    _status.samplesCollected++;
    _status.validSamples++;

    _status.progress =
        (float)_status.samplesCollected / (float)_gyroTargetSamples;

    if (_status.samplesCollected < _gyroTargetSamples) {
        return;
    }

    float avgX = (float)(_gyroSumX / _status.samplesCollected);
    float avgY = (float)(_gyroSumY / _status.samplesCollected);
    float avgZ = (float)(_gyroSumZ / _status.samplesCollected);

    float rangeX = _gyroMaxX - _gyroMinX;
    float rangeY = _gyroMaxY - _gyroMinY;
    float rangeZ = _gyroMaxZ - _gyroMinZ;

    DBG_PRINTF("[CAL] Gyro range: X=%.4f Y=%.4f Z=%.4f dps\n",
                  rangeX, rangeY, rangeZ);

    const float GYRO_RANGE_LIMIT_DPS = 3.0f;

    if (rangeX > GYRO_RANGE_LIMIT_DPS ||
        rangeY > GYRO_RANGE_LIMIT_DPS ||
        rangeZ > GYRO_RANGE_LIMIT_DPS) {

        _status.state = CalibrationState::FAILED;
        _status.active = false;
        _status.success = false;
        _status.canCancel = false;
        _status.quality = 0.0f;

        snprintf(_status.error, sizeof(_status.error),
                 "Gyro moved/noisy: %.2f %.2f %.2f dps",
                 rangeX, rangeY, rangeZ);

        snprintf(_status.message, sizeof(_status.message),
                 "Gyro rejected — keep drone still");

        DBG_PRINTF("[CAL][FAIL] %s\n", _status.error);
        return;
    }

    float worstRange = max(rangeX, max(rangeY, rangeZ));
    float gyroQuality = 1.0f - (worstRange / GYRO_RANGE_LIMIT_DPS);
    gyroQuality = constrain(gyroQuality, 0.0f, 1.0f);

    _imu->cal.gx_b = avgX;
    _imu->cal.gy_b = avgY;
    _imu->cal.gz_b = avgZ;
    _imu->cal.valid = true;

    _status.quality = gyroQuality;
    _status.progress = 1.0f;

    DBG_PRINTF("[CAL] Gyro done: X=%+.4f Y=%+.4f Z=%+.4f dps quality=%.2f\n",
                  avgX, avgY, avgZ, gyroQuality);

    if (_status.mode == CalibrationMode::IMU_ALL_GUIDED) {
        _startAccelCalibration();
        return;
    }

    _finishFullCalibration();

    snprintf(_status.message, sizeof(_status.message),
             "Gyro done: %.4f %.4f %.4f q=%.2f",
             avgX, avgY, avgZ, _status.quality);
}

// ─────────────────────────────────────────────────────────────
// Accel 6-face calibration
// ─────────────────────────────────────────────────────────────
void CalibrationManager::_startAccelCalibration()
{
    _accelPoseIndex = 0;

    for (int i = 0; i < 3; i++) {
        _accelPlus[i] = 0.0f;
        _accelMinus[i] = 0.0f;
        _accelGotPlus[i] = false;
        _accelGotMinus[i] = false;
    }

    _status.state = CalibrationState::WAITING_FOR_USER_STEP;
    _status.step = 2;
    _status.totalSteps = (_status.mode == CalibrationMode::IMU_ALL_GUIDED) ? 3 : 1;

    _status.samplesCollected = 0;
    _status.validSamples = 0;
    _status.rejectedSamples = 0;

    _status.progress = 0.0f;
    _status.requiresUserConfirm = true;

    snprintf(_status.message, sizeof(_status.message),
             "Accel 1/6: %s — flip SWC",
             ACCEL_POSES[_accelPoseIndex].label);

    DBG_PRINTF("[CAL] Accel pose 1/6: %s — flip SWC when still\n",
                  ACCEL_POSES[_accelPoseIndex].label);
}

void CalibrationManager::_captureAccelPose()
{
    if (_imu == nullptr) {
        _status.state = CalibrationState::FAILED;
        _status.active = false;
        _status.success = false;
        snprintf(_status.error, sizeof(_status.error), "IMU pointer is null");
        snprintf(_status.message, sizeof(_status.message), "Accel failed");
        return;
    }

    const AccelPose& pose = ACCEL_POSES[_accelPoseIndex];

    double axSum = 0.0;
    double aySum = 0.0;
    double azSum = 0.0;
    uint16_t good = 0;

    for (uint16_t i = 0; i < _accelTargetSamples; i++) {
        MPU_RawData raw;
        if (_imu->readRaw(raw)) {
            axSum += (float)raw.ax * ACCEL_SCALE_8G;
            aySum += (float)raw.ay * ACCEL_SCALE_8G;
            azSum += (float)raw.az * ACCEL_SCALE_8G;
            good++;
        } else {
            _status.rejectedSamples++;
        }

        delay(2);
    }

    if (good < (_accelTargetSamples / 2)) {
        _status.state = CalibrationState::FAILED;
        _status.active = false;
        _status.success = false;
        snprintf(_status.error, sizeof(_status.error),
                 "Accel capture failed: %u samples", good);
        snprintf(_status.message, sizeof(_status.message),
                 "Accel calibration failed");
        return;
    }

    float ax = (float)(axSum / good);
    float ay = (float)(aySum / good);
    float az = (float)(azSum / good);

    float axisValue[3] = { ax, ay, az };

    uint8_t axis = pose.axis;
    int8_t sign = pose.sign;
    float value = axisValue[axis];

    DBG_PRINTF("[CAL] Accel pose %u/6 %s: ax=%+.4f ay=%+.4f az=%+.4f g\n",
                  (unsigned)(_accelPoseIndex + 1),
                  pose.label,
                  ax, ay, az);

    DBG_PRINTF("[CAL] Routed to sensor %c%c = %+.4f g\n",
                  sign > 0 ? '+' : '-',
                  axis == 0 ? 'X' : axis == 1 ? 'Y' : 'Z',
                  value);

    if ((sign > 0 && value < 0.5f) ||
        (sign < 0 && value > -0.5f)) {

        DBG_PRINTLN(F("[CAL][WARN] Accel sign/magnitude unexpected."));
    }

    if (sign > 0) {
        _accelPlus[axis] = value;
        _accelGotPlus[axis] = true;
    } else {
        _accelMinus[axis] = value;
        _accelGotMinus[axis] = true;
    }

    _status.samplesCollected += good;
    _status.validSamples += good;

    _accelPoseIndex++;

    if (_accelPoseIndex >= ACCEL_POSE_COUNT) {
        _finishAccelCalibration();
        return;
    }

    _status.state = CalibrationState::WAITING_FOR_USER_STEP;
    _status.step = 2;
    _status.requiresUserConfirm = true;
    _status.progress = (float)_accelPoseIndex / (float)ACCEL_POSE_COUNT;

    snprintf(_status.message, sizeof(_status.message),
             "Accel %u/6: %s — flip SWC",
             (unsigned)(_accelPoseIndex + 1),
             ACCEL_POSES[_accelPoseIndex].label);

    DBG_PRINTF("[CAL] Next accel pose %u/6: %s\n",
                  (unsigned)(_accelPoseIndex + 1),
                  ACCEL_POSES[_accelPoseIndex].label);
}

void CalibrationManager::_finishAccelCalibration()
{
    bool accelOk = true;

    for (int axis = 0; axis < 3; axis++) {
        if (!_accelGotPlus[axis] || !_accelGotMinus[axis]) {
            accelOk = false;
        }
    }

    if (!accelOk) {
        _status.state = CalibrationState::FAILED;
        _status.active = false;
        _status.success = false;
        _status.canCancel = false;

        snprintf(_status.error, sizeof(_status.error),
                 "Accel incomplete");

        snprintf(_status.message, sizeof(_status.message),
                 "Accel calibration failed");

        DBG_PRINTLN(F("[CAL][FAIL] Accel calibration incomplete."));
        return;
    }

    _imu->cal.ax_b = (_accelPlus[0] + _accelMinus[0]) / 2.0f;
    _imu->cal.ay_b = (_accelPlus[1] + _accelMinus[1]) / 2.0f;
    _imu->cal.az_b = (_accelPlus[2] + _accelMinus[2]) / 2.0f;

    float hx = (_accelPlus[0] - _accelMinus[0]) / 2.0f;
    float hy = (_accelPlus[1] - _accelMinus[1]) / 2.0f;
    float hz = (_accelPlus[2] - _accelMinus[2]) / 2.0f;

    _imu->cal.ax_s = fabsf(hx) > 0.01f ? 1.0f / hx : 1.0f;
    _imu->cal.ay_s = fabsf(hy) > 0.01f ? 1.0f / hy : 1.0f;
    _imu->cal.az_s = fabsf(hz) > 0.01f ? 1.0f / hz : 1.0f;

    _imu->cal.valid = true;

    DBG_PRINTF("[CAL] Accel bias:  X=%+.4f Y=%+.4f Z=%+.4f g\n",
                  _imu->cal.ax_b, _imu->cal.ay_b, _imu->cal.az_b);

    DBG_PRINTF("[CAL] Accel scale: X=%+.4f Y=%+.4f Z=%+.4f\n",
                  _imu->cal.ax_s, _imu->cal.ay_s, _imu->cal.az_s);

    _status.progress = 1.0f;
    _status.quality = 1.0f;

    if (_status.mode == CalibrationMode::IMU_ALL_GUIDED) {
        _startMagCalibration();
        return;
    }

    _finishFullCalibration();
}

// ─────────────────────────────────────────────────────────────
// Mag min/max calibration
// ─────────────────────────────────────────────────────────────
void CalibrationManager::_startMagCalibration()
{
    if (_imu == nullptr) {
        _status.state = CalibrationState::FAILED;
        _status.active = false;
        _status.success = false;
        snprintf(_status.error, sizeof(_status.error), "IMU pointer is null");
        snprintf(_status.message, sizeof(_status.message), "Mag failed");
        return;
    }


    _magStartMs = millis();
    _lastMagPrintMs = 0;
    _magDurationMs = 30000;

    _magMinX =  999999.0f;
    _magMaxX = -999999.0f;
    _magMinY =  999999.0f;
    _magMaxY = -999999.0f;
    _magMinZ =  999999.0f;
    _magMaxZ = -999999.0f;

    _magValidSamples = 0;

    _status.state = CalibrationState::COLLECTING;
    _status.step = 3;
    _status.totalSteps = (_status.mode == CalibrationMode::IMU_ALL_GUIDED) ? 3 : 1;

    _status.samplesCollected = 0;
    _status.validSamples = 0;
    _status.rejectedSamples = 0;

    _status.progress = 0.0f;
    _status.requiresUserConfirm = false;

    snprintf(_status.message, sizeof(_status.message),
             "Mag: rotate all axes 30 sec");

    DBG_PRINTLN(F("[CAL] Mag calibration started. Rotate through all axes."));
}

void CalibrationManager::_updateMagCalibration()
{
    if (_imu == nullptr) {
        _status.state = CalibrationState::FAILED;
        _status.active = false;
        _status.success = false;
        snprintf(_status.error, sizeof(_status.error), "IMU pointer is null");
        snprintf(_status.message, sizeof(_status.message), "Mag failed");
        return;
    }

    uint32_t now = millis();
    uint32_t elapsed = now - _magStartMs;

    MPU_RawData raw;
    if (_imu->readRaw(raw)) {
        if (raw.magOk) {
            float mx = (float)raw.mx * MAG_SCALE_16BIT * _imu->cal.mag_asa_x;
            float my = (float)raw.my * MAG_SCALE_16BIT * _imu->cal.mag_asa_y;
            float mz = (float)raw.mz * MAG_SCALE_16BIT * _imu->cal.mag_asa_z;

            float magNorm = sqrtf(mx * mx + my * my + mz * mz);

            if (magNorm > 1.0f) {
                if (mx < _magMinX) _magMinX = mx;
                if (mx > _magMaxX) _magMaxX = mx;

                if (my < _magMinY) _magMinY = my;
                if (my > _magMaxY) _magMaxY = my;

                if (mz < _magMinZ) _magMinZ = mz;
                if (mz > _magMaxZ) _magMaxZ = mz;

                _magValidSamples++;
                _status.validSamples++;
            } else {
                _status.rejectedSamples++;
            }
        } else {
            _status.rejectedSamples++;
        }

        _status.samplesCollected++;
    }

    _status.progress = constrain((float)elapsed / (float)_magDurationMs,
                                 0.0f, 1.0f);

    if (now - _lastMagPrintMs >= 5000) {
        _lastMagPrintMs = now;

        DBG_PRINTF("[CAL] Mag progress %.0f%% samples=%u\n",
                      _status.progress * 100.0f,
                      _magValidSamples);
    }

    if (elapsed < _magDurationMs) {
        return;
    }

    if (_magValidSamples < 50) {
        _status.state = CalibrationState::FAILED;
        _status.active = false;
        _status.success = false;
        _status.canCancel = false;

        snprintf(_status.error, sizeof(_status.error),
                 "Mag failed: %u valid samples",
                 _magValidSamples);

        snprintf(_status.message, sizeof(_status.message),
                 "Mag calibration failed");

        DBG_PRINTF("[CAL][FAIL] %s\n", _status.error);
        return;
    }

    float rx = _magMaxX - _magMinX;
    float ry = _magMaxY - _magMinY;
    float rz = _magMaxZ - _magMinZ;

    _imu->cal.mx_b = (_magMaxX + _magMinX) / 2.0f;
    _imu->cal.my_b = (_magMaxY + _magMinY) / 2.0f;
    _imu->cal.mz_b = (_magMaxZ + _magMinZ) / 2.0f;

    float avgRadius = (rx + ry + rz) / 3.0f;

    _imu->cal.mx_s = rx > 0.1f ? avgRadius / rx : 1.0f;
    _imu->cal.my_s = ry > 0.1f ? avgRadius / ry : 1.0f;
    _imu->cal.mz_s = rz > 0.1f ? avgRadius / rz : 1.0f;

    _imu->cal.valid = true;

    DBG_PRINTF("[CAL] Mag range: X=%.2f Y=%.2f Z=%.2f uT\n",
                  rx, ry, rz);

    DBG_PRINTF("[CAL] Mag bias:  X=%+.2f Y=%+.2f Z=%+.2f uT\n",
                  _imu->cal.mx_b, _imu->cal.my_b, _imu->cal.mz_b);

    DBG_PRINTF("[CAL] Mag scale: X=%+.4f Y=%+.4f Z=%+.4f\n",
                  _imu->cal.mx_s, _imu->cal.my_s, _imu->cal.mz_s);

    _status.quality = constrain((float)_magValidSamples / 300.0f, 0.0f, 1.0f);
    _status.progress = 1.0f;

    _finishFullCalibration();
}

// ─────────────────────────────────────────────────────────────
// Save + finish
// ─────────────────────────────────────────────────────────────
void CalibrationManager::_finishFullCalibration()
{
    if (_imu == nullptr) {
        _status.state = CalibrationState::FAILED;
        _status.active = false;
        _status.success = false;
        snprintf(_status.error, sizeof(_status.error), "IMU pointer is null");
        snprintf(_status.message, sizeof(_status.message), "Save failed");
        return;
    }

    _status.state = CalibrationState::SAVING;
    _status.requiresUserConfirm = false;
    _status.canCancel = false;

    _imu->cal.valid = true;
    _imu->saveCalibration();

    DBG_PRINTLN(F("[CAL] Full saved calibration:"));
    _imu->printCalibration();

    _status.state = CalibrationState::DONE;
    _status.active = false;
    _status.success = true;
    _status.canCancel = false;
    _status.requiresUserConfirm = false;
    _status.progress = 1.0f;

    snprintf(_status.message, sizeof(_status.message),
             "Calibration complete quality=%.2f",
             _status.quality);
}

CalibrationStatus CalibrationManager::status() const
{
    return _status;
}
