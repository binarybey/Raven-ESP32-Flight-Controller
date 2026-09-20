#include "attitude_transform.h"

DCM dcm_input;
Quaternion quaternion_input;
EulerAngles euler_input;

DCM dcm_output;
Quaternion quaternion_output;
EulerAngles euler_output;

void quaternionToDCM(const Quaternion& q, DCM& dcm, bool scalarFirst) {
    float q1 = scalarFirst ? q.q2 : q.q1;
    float q2 = scalarFirst ? q.q3 : q.q2;
    float q3 = scalarFirst ? q.q4 : q.q3;
    float q4 = scalarFirst ? q.q1 : q.q4;

    dcm.dcm[0][0] = (q4 * q4 + q1 * q1 - q2 * q2 - q3 * q3);
    dcm.dcm[0][1] = 2 * (q1 * q2 + q3 * q4);
    dcm.dcm[0][2] = 2 * (q1 * q3 - q2 * q4);

    dcm.dcm[1][0] = 2 * (q1 * q2 - q3 * q4);
    dcm.dcm[1][1] = (q4 * q4 - q1 * q1 + q2 * q2 - q3 * q3);
    dcm.dcm[1][2] = 2 * (q2 * q3 + q1 * q4);

    dcm.dcm[2][0] = 2 * (q1 * q3 + q2 * q4);
    dcm.dcm[2][1] = 2 * (q2 * q3 - q1 * q4);
    dcm.dcm[2][2] = (q4 * q4 - q1 * q1 - q2 * q2 + q3 * q3);
}

void DCMToQuaternion(const DCM& dcm, Quaternion& q, bool scalarFirst) {
    float q1_sq_temp = 0.25f * (1.0f + dcm.dcm[0][0] - dcm.dcm[1][1] - dcm.dcm[2][2]);
    float q2_sq_temp = 0.25f * (1.0f - dcm.dcm[0][0] + dcm.dcm[1][1] - dcm.dcm[2][2]);
    float q3_sq_temp = 0.25f * (1.0f - dcm.dcm[0][0] - dcm.dcm[1][1] + dcm.dcm[2][2]);
    float q4_sq_temp = 0.25f * (1.0f + dcm.dcm[0][0] + dcm.dcm[1][1] + dcm.dcm[2][2]);

    float deciding_term = std::max<float>(std::max<float>(q1_sq_temp,q2_sq_temp),std::max<float>(q3_sq_temp,q4_sq_temp));

    float q1;
    float q2;
    float q3;
    float q4;

    if(deciding_term == q1_sq_temp) {
        q1 = sqrtf(q1_sq_temp);
        q2 = (dcm.dcm[0][1] + dcm.dcm[1][0]) / (4.0f * sqrtf(q1_sq_temp));
        q3 = (dcm.dcm[2][0] + dcm.dcm[0][2]) / (4.0f * sqrtf(q1_sq_temp));
        q4 = (dcm.dcm[1][2] - dcm.dcm[2][1]) / (4.0f * sqrtf(q1_sq_temp));
    }
    else if(deciding_term == q2_sq_temp){
        q1 = (dcm.dcm[0][1] + dcm.dcm[1][0]) / (4.0f * sqrtf(q2_sq_temp));
        q2 = sqrtf(q2_sq_temp);
        q3 = (dcm.dcm[1][2] + dcm.dcm[2][1]) / (4.0f * sqrtf(q2_sq_temp));
        q4 = (dcm.dcm[2][0] - dcm.dcm[0][2]) / (4.0f * sqrtf(q2_sq_temp));
    }
    else if(deciding_term == q3_sq_temp){
        q1 = (dcm.dcm[2][0] + dcm.dcm[0][2]) / (4.0f * sqrtf(q3_sq_temp));
        q2 = (dcm.dcm[1][2] + dcm.dcm[2][1]) / (4.0f * sqrtf(q3_sq_temp));
        q3 = sqrtf(q3_sq_temp);
        q4 = (dcm.dcm[0][1] - dcm.dcm[1][0]) / (4.0f * sqrtf(q3_sq_temp));
    }
    else if(deciding_term == q4_sq_temp){
        q1 = (dcm.dcm[1][2] - dcm.dcm[2][1]) / (4.0f * sqrtf(q4_sq_temp));
        q2 = (dcm.dcm[2][0] - dcm.dcm[0][2]) / (4.0f * sqrtf(q4_sq_temp));
        q3 = (dcm.dcm[0][1] - dcm.dcm[1][0]) / (4.0f * sqrtf(q4_sq_temp));
        q4 = sqrtf(q4_sq_temp);
    }

    q.q1 = scalarFirst ? q2 : q1;
    q.q2 = scalarFirst ? q3 : q2;
    q.q3 = scalarFirst ? q4 : q3;
    q.q4 = scalarFirst ? q1 : q4;
}

void eulerToDCM(const byte order_third, const byte order_second, const byte order_first, const EulerAngles& euler, DCM& dcm) {
    // e.g. 3-2-1 standard order
    DCM R[3];
    DCM intermediate;

    R[0].dcm[0][0] = 1.0f;
    R[0].dcm[0][1] = 0.0f;
    R[0].dcm[0][2] = 0.0f;
    R[0].dcm[1][0] = 0.0f;
    R[0].dcm[2][0] = 0.0f;
    R[0].dcm[1][1] = cosf(euler.roll);
    R[0].dcm[1][2] = sinf(euler.roll);
    R[0].dcm[2][1] = sinf(euler.roll) * -1.0f;
    R[0].dcm[2][2] = cosf(euler.roll);

    R[1].dcm[1][1] = 1.0f;
    R[1].dcm[0][1] = 0.0f;
    R[1].dcm[1][0] = 0.0f;
    R[1].dcm[2][1] = 0.0f;
    R[1].dcm[1][2] = 0.0f;
    R[1].dcm[0][0] = cosf(euler.pitch);
    R[1].dcm[0][2] = sinf(euler.pitch) * -1.0f;
    R[1].dcm[2][0] = sinf(euler.pitch);
    R[1].dcm[2][2] = cosf(euler.pitch);
    
    R[2].dcm[2][2] = 1.0f;
    R[2].dcm[0][2] = 0.0f;
    R[2].dcm[1][2] = 0.0f;
    R[2].dcm[2][1] = 0.0f;
    R[2].dcm[2][0] = 0.0f;
    R[2].dcm[0][0] = cosf(euler.yaw);
    R[2].dcm[0][1] = sinf(euler.yaw);
    R[2].dcm[1][0] = sinf(euler.yaw) * -1.0f;
    R[2].dcm[1][1] = cosf(euler.yaw);

    for(int i=0;i<3;i++){
        for(int j=0;j<3;j++){
        intermediate.dcm[i][j] = 0.0f;
            for(int k = 0; k < 3; k++){
            intermediate.dcm[i][j] += R[order_third-1].dcm[i][k] * R[order_second-1].dcm[k][j];
            }
        }
    }

    for(int i = 0; i < 3; i++) {
        for(int j = 0; j < 3; j++) {
        dcm.dcm[i][j] = 0.0f;
            for(int k = 0; k < 3; k++){
            dcm.dcm[i][j] += intermediate.dcm[i][k] * R[order_first-1].dcm[k][j];
            }
        }
    }
}

void DCMToEuler(const int seq, const DCM& dcm, EulerAngles& outAngles) {
    float pivot;
    switch (seq)
    {
    case 321:
        pivot = dcm.dcm[0][2];
        if (pivot > 1.0f) pivot = 1.0f; else if (pivot < -1.0f) pivot = -1.0f;
        outAngles.pitch = asinf(-1.0f * pivot);
        outAngles.yaw = atan2f(dcm.dcm[0][1], dcm.dcm[0][0]);
        outAngles.roll = atan2f(dcm.dcm[1][2], dcm.dcm[2][2]);
        break;

    case 312:
        pivot = dcm.dcm[1][2];
        if (pivot > 1.0f) pivot = 1.0f; else if (pivot < -1.0f) pivot = -1.0f;
        outAngles.roll = asinf(pivot);
        outAngles.pitch = atan2f(-1.0f * dcm.dcm[0][2], dcm.dcm[2][2]);
        outAngles.yaw = atan2f(-1.0f * dcm.dcm[1][0], dcm.dcm[1][1]);
        break;

    case 213:
        pivot = dcm.dcm[2][1];
        if (pivot > 1.0f) pivot = 1.0f; else if (pivot < -1.0f) pivot = -1.0f;
        outAngles.roll = asinf(-1.0f * pivot);
        outAngles.pitch = atan2f(dcm.dcm[2][0], dcm.dcm[2][2]);
        outAngles.yaw = atan2f(dcm.dcm[0][1], dcm.dcm[1][1]);
        break;
    case 231:
        pivot = dcm.dcm[0][1];
        if (pivot > 1.0f) pivot = 1.0f; else if (pivot < -1.0f) pivot = -1.0f;
        outAngles.roll = atan2f(-1.0f * dcm.dcm[2][1], dcm.dcm[1][1]);
        outAngles.pitch = atan2f(-1.0f * dcm.dcm[0][2], dcm.dcm[0][0]);
        outAngles.yaw = asinf(pivot);
        break;
        
    case 123:
        pivot = dcm.dcm[2][0];
        if (pivot > 1.0f) pivot = 1.0f; else if (pivot < -1.0f) pivot = -1.0f;
        outAngles.roll = atan2f(-1.0f * dcm.dcm[2][1], dcm.dcm[2][2]);
        outAngles.pitch = asinf(pivot);
        outAngles.yaw = atan2f(-1.0f * dcm.dcm[1][0], dcm.dcm[0][0]);
        break;

    case 132:
        pivot = dcm.dcm[1][0];
        if (pivot > 1.0f) pivot = 1.0f; else if (pivot < -1.0f) pivot = -1.0f;
        outAngles.roll = atan2f(dcm.dcm[1][2], dcm.dcm[1][1]);
        outAngles.pitch = atan2f(dcm.dcm[2][0], dcm.dcm[0][0]);
        outAngles.yaw = asinf(-1.0f * pivot);
        break;    

    default:
        break;
    }
}

void EulerToQuaternion(const byte order_third, const byte order_second, const byte order_first, const EulerAngles& euler, Quaternion& q, bool scalarFirst) {
    DCM temporaryDCM;
    eulerToDCM(order_third, order_second, order_first, euler, temporaryDCM);
    DCMToQuaternion(temporaryDCM, q, scalarFirst);
}

void QuaternionToEuler(const int seq, const Quaternion& q, EulerAngles& euler, bool scalarFirst) {
    DCM temporaryDCM;
    quaternionToDCM(q, temporaryDCM, scalarFirst);
    DCMToEuler(seq, temporaryDCM, euler);
}


// 3-1-2
// roll = asinf(C_23)
// pitch = atan2f(-C_13, C_33)
// yaw = atan2f(-C_21, C_22)

// 3-2-1
// roll = atan2f(C_23, C_33)
// pitch = asinf(-C_13)
// yaw = atan2f(C_12, C_11)

// 2-1-3
// roll = asinf(-C_32)
// pitch = atan2f(C_31, C_33)
// yaw = atan2f(C_12, C_22)

// 2-3-1
// roll = atan2f(-C_32, C_22)
// pitch = atan2f(-C_13, C_11)
// yaw = asinf(C_12)

// 1-2-3
// roll = atan2f(-C_32, C_33)
// pitch = asinf(C_31)
// yaw = atan2f(-C_21, C_11)

// 1-3-2
// roll = atan2f(C_23, C_22)
// pitch = atan2f(C_31, C_11)
// yaw = asinf(-C_21)