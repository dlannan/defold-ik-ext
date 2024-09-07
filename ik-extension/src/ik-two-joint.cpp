#include <dmsdk/dlib/vmath.h>

#include <math.h>
#define min(a,b)            (((a) < (b)) ? (a) : (b))
#define max(a,b)            (((a) > (b)) ? (a) : (b))

float clamp(float x, float upper, float lower)
{
    return min(upper, max(x, lower));
}

dmVMath::Quat quat_inv(dmVMath::Quat q)
{
    return dmVMath::Quat(-q[0], -q[1], -q[2], q[3]);
}

dmVMath::Vector3 rotateVector(dmVMath::Quat q, dmVMath::Vector3 vec)
{
    // t = 2q x v
    float tx = 2. * (q[1] * vec[2] - q[2] * vec[1]);
    float ty = 2. * (q[2] * vec[0] - q[0] * vec[2]);
    float tz = 2. * (q[0] * vec[1] - q[1] * vec[0]);

    // v + w t + q x t
    vec[0] = vec[0] + q[3] * tx + q[1] * tz - q[2] * ty;
    vec[1] = vec[1] + q[3] * ty + q[2] * tx - q[0] * tz;
    vec[2] = vec[2] + q[3] * tz + q[0] * ty - q[1] * tx;
    return vec;
}

dmVMath::Quat fromAngleAxis( float angle, dmVMath::Vector3 axis )
{
    float x = axis[0];
    float y = axis[1]; 
    float z = axis[2];
    
    dmVMath::Quat ret;

    float halfAngle = angle * 0.5;

    float sin_2 = sinf(halfAngle);
    float cos_2 = cosf(halfAngle);

    float sin_norm = sin_2 / sqrtf(x * x + y * y + z * z);

    ret[3] = cos_2;
    ret[0] = x * sin_norm;
    ret[1] = y * sin_norm;
    ret[2] = z * sin_norm;

    return ret;
}


// hip_pos - hip joint
// knee_pos - knee join
// heel_pos - heel joint
// t - target point
// eps - minimum extension lenght
// hip_gr - hip rotation
// knee_gr - knee rotation
// hip_lr - hip local rotation
// knee_lr - knee local rotation

const bool use_stable_extension = false;

void two_joint_ik(
    dmVMath::Vector3 hip_pos, dmVMath::Vector3 knee_pos, dmVMath::Vector3 heel_pos, dmVMath::Vector3 t, float eps,
    dmVMath::Quat hip_gr, dmVMath::Quat knee_gr, dmVMath::Quat &hip_lr, dmVMath::Quat &knee_lr) {
  
    float lab = length(knee_pos - hip_pos);
    float lcb = length(knee_pos - heel_pos);
    float lat = clamp(length(t - hip_pos), eps, lab + lcb - eps);

    float ac_ab_0 = acos(clamp(dot(normalize(heel_pos - hip_pos), normalize(knee_pos - hip_pos)), -1, 1));
    float ba_bc_0 = acos(clamp(dot(normalize(hip_pos - knee_pos), normalize(heel_pos - knee_pos)), -1, 1));
    float ac_at_0 = acos(clamp(dot(normalize(heel_pos - hip_pos), normalize(t - hip_pos)), -1, 1));

    float ac_ab_1 = acos(clamp((lcb*lcb-lab*lab-lat*lat) / (-2*lab*lat), -1, 1));
    float ba_bc_1 = acos(clamp((lat*lat-lab*lab-lcb*lcb) / (-2*lab*lcb), -1, 1));

    dmVMath::Vector3 axis0;
    if (use_stable_extension) {
        dmVMath::Vector3 d = rotateVector(knee_gr,  dmVMath::Vector3(0, 0, 1));
        axis0 = normalize(cross(heel_pos - hip_pos, d));
    } else {
        axis0 = normalize(cross(heel_pos - hip_pos, knee_pos - hip_pos));
    }
    dmVMath::Vector3 axis1 = normalize(cross(heel_pos - hip_pos, t - hip_pos));    
    
    dmVMath::Quat r0 = fromAngleAxis(ac_ab_1 - ac_ab_0, rotateVector(quat_inv(hip_gr), axis0));
    dmVMath::Quat r1 = fromAngleAxis(ba_bc_1 - ba_bc_0, rotateVector(quat_inv(knee_gr), axis0));
    dmVMath::Quat r2 = fromAngleAxis(ac_at_0, rotateVector(quat_inv(hip_gr), axis1));

    hip_lr = hip_lr * r0 * r2;
    knee_lr = knee_lr * r1;
}