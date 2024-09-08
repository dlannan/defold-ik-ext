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

dmVMath::Quat quat_mul(dmVMath::Quat q, dmVMath::Quat p)
{
    return dmVMath::Quat(
        p[3]*q[3] - p[0]*q[0] - p[1]*q[1] - p[2]*q[2],
        p[3]*q[0] + p[0]*q[3] - p[1]*q[2] + p[2]*q[1],
        p[3]*q[1] + p[0]*q[2] + p[1]*q[3] - p[2]*q[0],
        p[3]*q[2] - p[0]*q[1] + p[1]*q[0] + p[2]*q[3]);
}

dmVMath::Quat quat_from_angle_axis(float angle, dmVMath::Vector3 axis)
{
    float c = cosf(angle / 2.0f);
    float s = sinf(angle / 2.0f);
    return dmVMath::Quat(c, s * axis.getX(), s * axis.getY(), s * axis.getZ());
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
        dmVMath::Vector3 d = dmVMath::Rotate(knee_gr,  dmVMath::Vector3(0, 0, 1));
        axis0 = normalize(cross(heel_pos - hip_pos, d));
    } else {
        axis0 = normalize(cross(heel_pos - hip_pos, knee_pos - hip_pos));
    }
    dmVMath::Vector3 axis1 = normalize(cross(heel_pos - hip_pos, t - hip_pos));    
    
    dmVMath::Quat r0 = quat_from_angle_axis(ac_ab_1 - ac_ab_0, dmVMath::Rotate(quat_inv(hip_gr), axis0));
    dmVMath::Quat r1 = quat_from_angle_axis(ba_bc_1 - ba_bc_0, dmVMath::Rotate(quat_inv(knee_gr), axis0));
    dmVMath::Quat r2 = quat_from_angle_axis(ac_at_0, dmVMath::Rotate(quat_inv(hip_gr), axis1));

    hip_lr = quat_mul(hip_lr, quat_mul(r0, r2));
    knee_lr = quat_mul(knee_lr, r1);
}