// attitude_transform.h - independent quaternion/DCM/Euler conversions.
// REFERENCE IMPLEMENTATION, not used by the flight firmware (PlatformIO
// doesn't link it because nothing includes it). It is the independent
// 321-sequence DCM check that the NWU sign convention in
// flight_kinematics.h was verified against - keep it for re-verifying that
// convention if the IMU mounting or AHRS library ever changes.

#pragma once
#include <Arduino.h>

struct Quaternion {
    float q1;
    float q2;
    float q3;
    float q4;
};

struct EulerAngles {
    float roll;
    float pitch;
    float yaw;
};

struct DCM {
    float dcm[3][3];
};

void quaternionToDCM(const Quaternion& q, DCM& dcm, bool scalarFirst = false);
void DCMToQuaternion(const DCM& dcm, Quaternion& q, bool scalarFirst = false);
void eulerToDCM(const byte order_third, const byte order_second, const byte order_first, const EulerAngles& euler, DCM& dcm);
void DCMToEuler(const int seq, const DCM& dcm, EulerAngles& outAngles);
void EulerToQuaternion(const byte order_third, const byte order_second, const byte order_first, const EulerAngles& euler, Quaternion& q, bool scalarFirst = false);
void QuaternionToEuler(const int seq, const Quaternion& q, EulerAngles& euler, bool scalarFirst = false);

extern DCM dcm_input;
extern Quaternion quaternion_input;
extern EulerAngles euler_input;

extern DCM dcm_output;
extern Quaternion quaternion_output;
extern EulerAngles euler_output;
