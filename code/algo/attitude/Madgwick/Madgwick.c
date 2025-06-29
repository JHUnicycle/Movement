#include "Madgwick.h"

void MadgwickAHRS_init(float sample_rate)
{
    // 初始四元数 (单位四元数)
    s_Madgwick_info.q0 = 1.0f;
    s_Madgwick_info.q1 = 0.0f;
    s_Madgwick_info.q2 = 0.0f;
    s_Madgwick_info.q3 = 0.0f;

    // 滤波器参数设置
    s_Madgwick_info.beta_static = 0.1f;                 // 静止时信任加速度计 (较高beta)
    s_Madgwick_info.beta_dynamic = 0.025f;              // 运动时信任陀螺仪 (较低beta)
    s_Madgwick_info.beta = s_Madgwick_info.beta_static; // 初始处于静止状态

    // 传感器零偏初始化
    s_Madgwick_info.g_bias_x = 0.0f;
    s_Madgwick_info.g_bias_y = 0.0f;
    s_Madgwick_info.g_bias_z = 0.0f;

    s_Madgwick_info.acc_bias_x = 0.0f;
    s_Madgwick_info.acc_bias_y = 0.0f;
    s_Madgwick_info.acc_bias_z = 0.0f;

    // 重力参考值 (m/s²)
    s_Madgwick_info.gravity_ref = 9.80665f; // 标准重力加速度

    // 运动检测阈值
    s_Madgwick_info.acc_threshold_static = 0.02f; // 1.02g
    s_Madgwick_info.acc_threshold_motion = 0.05f; // 1.05g

    // 设置采样率
    s_Madgwick_info.sample_rate = sample_rate;
    s_Madgwick_info.inv_sample_rate = 1.0f / sample_rate;

    // 初始化标志
    s_Madgwick_info.is_initialized = 0;
    s_Madgwick_info.init_count = 0; // 使用计数器替代时间判断
}

// 四元数归一化
void normalizeQuaternion(float *q0, float *q1, float *q2, float *q3)
{
    float norm = sqrtf(*q0 * *q0 + *q1 * *q1 + *q2 * *q2 + *q3 * *q3);
    if (norm < 1e-6f)
        return;
    float inv_norm = 1.0f / norm;
    *q0 *= inv_norm;
    *q1 *= inv_norm;
    *q2 *= inv_norm;
    *q3 *= inv_norm;
}

// 更新姿态解算器 (主要函数)
void MadgwickAHRS_update(struct IMU_DATA *imu_data)
{ // 加速度计值 (m/s²)
    // 如果未初始化，直接返回
    if (!s_Madgwick_info.is_initialized)
    {
        return;
    }

    // 使用固定的采样周期
    float dt = s_Madgwick_info.inv_sample_rate;

    float gx = imu_data->gyro.x;
    float gy = imu_data->gyro.y;
    float gz = imu_data->gyro.z;

    float ax = imu_data->acc.x;
    float ay = imu_data->acc.y;
    float az = imu_data->acc.z;

    // 加速度计校准
    ax -= s_Madgwick_info.acc_bias_x;
    ay -= s_Madgwick_info.acc_bias_y;
    az -= s_Madgwick_info.acc_bias_z;

    // 陀螺仪校准
    gx -= s_Madgwick_info.g_bias_x;
    gy -= s_Madgwick_info.g_bias_y;
    gz -= s_Madgwick_info.g_bias_z;

    // 计算加速度幅度 (用于运动检测)
    float acc_norm = sqrtf(ax * ax + ay * ay + az * az);
    float acc_delta = fabsf(acc_norm - s_Madgwick_info.gravity_ref) / s_Madgwick_info.gravity_ref;

    // 运动状态检测和beta值自适应
    if (acc_delta < s_Madgwick_info.acc_threshold_static)
    {
        // 静止状态
        s_Madgwick_info.beta = s_Madgwick_info.beta_static;

        // 静态时在线校准陀螺零偏 (非常保守的更新)
        s_Madgwick_info.g_bias_x = 0.999f * s_Madgwick_info.g_bias_x + 0.001f * gx;
        s_Madgwick_info.g_bias_y = 0.999f * s_Madgwick_info.g_bias_y + 0.001f * gy;
        s_Madgwick_info.g_bias_z = 0.999f * s_Madgwick_info.g_bias_z + 0.001f * gz;
    }
    else if (acc_delta > s_Madgwick_info.acc_threshold_motion)
    {
        // 检测到显著运动状态
        s_Madgwick_info.beta = s_Madgwick_info.beta_dynamic;
    }
    // 中间状态保持当前beta不变

    // 归一化加速度计测量值
    float ax_n = ax / acc_norm;
    float ay_n = ay / acc_norm;
    float az_n = az / acc_norm;

    // 计算预测的重力方向 (基于当前四元数)
    float vx = 2.0f * (s_Madgwick_info.q1 * s_Madgwick_info.q3 - s_Madgwick_info.q0 * s_Madgwick_info.q2);
    float vy = 2.0f * (s_Madgwick_info.q0 * s_Madgwick_info.q1 + s_Madgwick_info.q2 * s_Madgwick_info.q3);
    float vz = s_Madgwick_info.q0 * s_Madgwick_info.q0 - s_Madgwick_info.q1 * s_Madgwick_info.q1 - s_Madgwick_info.q2 * s_Madgwick_info.q2 + s_Madgwick_info.q3 * s_Madgwick_info.q3;

    // 计算预测和测量值之间的误差
    float error_x = ay_n * vz - az_n * vy;
    float error_y = az_n * vx - ax_n * vz;
    float error_z = ax_n * vy - ay_n * vx;

    // 计算梯度
    float gradient_x = error_x;
    float gradient_y = error_y;
    float gradient_z = error_z;
    float gradient_norm = sqrtf(gradient_x * gradient_x + gradient_y * gradient_y + gradient_z * gradient_z);

    if (gradient_norm > 1e-6f)
    {
        // 归一化梯度 (避免零除)
        float inv_gradient_norm = 1.0f / gradient_norm;
        gradient_x *= inv_gradient_norm;
        gradient_y *= inv_gradient_norm;
        gradient_z *= inv_gradient_norm;
    }

    // 应用beta系数调整反馈强度
    gradient_x *= s_Madgwick_info.beta;
    gradient_y *= s_Madgwick_info.beta;
    gradient_z *= s_Madgwick_info.beta;

    // 创建陀螺仪四元数
    float gx_quat = 0.5f * (gx + gradient_x);
    float gy_quat = 0.5f * (gy + gradient_y);
    float gz_quat = 0.5f * (gz + gradient_z);

    // 四元数导数 (四元数乘法)
    float qDot0 = -s_Madgwick_info.q1 * gx_quat - s_Madgwick_info.q2 * gy_quat - s_Madgwick_info.q3 * gz_quat;
    float qDot1 = s_Madgwick_info.q0 * gx_quat + s_Madgwick_info.q2 * gz_quat - s_Madgwick_info.q3 * gy_quat;
    float qDot2 = s_Madgwick_info.q0 * gy_quat - s_Madgwick_info.q1 * gz_quat + s_Madgwick_info.q3 * gx_quat;
    float qDot3 = s_Madgwick_info.q0 * gz_quat + s_Madgwick_info.q1 * gy_quat - s_Madgwick_info.q2 * gx_quat;

    // 积分四元数
    s_Madgwick_info.q0 += qDot0 * dt;
    s_Madgwick_info.q1 += qDot1 * dt;
    s_Madgwick_info.q2 += qDot2 * dt;
    s_Madgwick_info.q3 += qDot3 * dt;

    // 四元数归一化 (防止发散)
    normalizeQuaternion(&s_Madgwick_info.q0, &s_Madgwick_info.q1, &s_Madgwick_info.q2, &s_Madgwick_info.q3);
}

// 获取重力校正的加速度 (移除重力分量)
void MadgwickAHRS_getLinearAccel(float ax, float ay, float az,
                                 float *lin_x, float *lin_y, float *lin_z)
{
    // 计算基于姿态的重力分量
    float gx = 2.0f * (s_Madgwick_info.q1 * s_Madgwick_info.q3 - s_Madgwick_info.q0 * s_Madgwick_info.q2);
    float gy = 2.0f * (s_Madgwick_info.q0 * s_Madgwick_info.q1 + s_Madgwick_info.q2 * s_Madgwick_info.q3);
    float gz = s_Madgwick_info.q0 * s_Madgwick_info.q0 - s_Madgwick_info.q1 * s_Madgwick_info.q1 - s_Madgwick_info.q2 * s_Madgwick_info.q2 + s_Madgwick_info.q3 * s_Madgwick_info.q3;

    // 从测量中移除重力分量
    *lin_x = ax - gx * s_Madgwick_info.gravity_ref;
    *lin_y = ay - gy * s_Madgwick_info.gravity_ref;
    *lin_z = az - gz * s_Madgwick_info.gravity_ref;
}

// 加速度计校准和姿态初始化函数
int MadgwickAHRS_calibrate(struct IMU_DATA *imu_data)
{
    static float acc_sum_x = 0, acc_sum_y = 0, acc_sum_z = 0;

    if (s_Madgwick_info.is_initialized)
    {
        return 1; // 已经初始化完成
    }

    float ax = imu_data->acc.x;
    float ay = imu_data->acc.y;
    float az = imu_data->acc.z;

    // 加速度采集和累加
    acc_sum_x += ax;
    acc_sum_y += ay;
    acc_sum_z += az;
    s_Madgwick_info.init_count++;

    // 校准完成条件
    if (s_Madgwick_info.init_count >= 1)
    {
        // 计算平均加速度作为零偏
        s_Madgwick_info.acc_bias_x = acc_sum_x / s_Madgwick_info.init_count;
        s_Madgwick_info.acc_bias_y = acc_sum_y / s_Madgwick_info.init_count;
        s_Madgwick_info.acc_bias_z = (acc_sum_z / s_Madgwick_info.init_count) - s_Madgwick_info.gravity_ref;

        // 使用校准后的加速度计算初始姿态
        float ax_cal = (acc_sum_x / s_Madgwick_info.init_count) - s_Madgwick_info.acc_bias_x;
        float ay_cal = (acc_sum_y / s_Madgwick_info.init_count) - s_Madgwick_info.acc_bias_y;
        float az_cal = (acc_sum_z / s_Madgwick_info.init_count) - s_Madgwick_info.acc_bias_z;

        float roll = atan2f(ay_cal, az_cal);
        float pitch = atan2f(-ax_cal, sqrtf(ay_cal * ay_cal + az_cal * az_cal));

        // 转换为四元数
        float cy = cosf(pitch * 0.5f);
        float sy = sinf(pitch * 0.5f);
        float cr = cosf(roll * 0.5f);
        float sr = sinf(roll * 0.5f);

        s_Madgwick_info.q0 = cy * cr;
        s_Madgwick_info.q1 = cy * sr;
        s_Madgwick_info.q2 = sy * cr;
        s_Madgwick_info.q3 = sy * sr;

        s_Madgwick_info.is_initialized = 1;

        // 重置静态变量供下次使用
        acc_sum_x = acc_sum_y = acc_sum_z = 0;

        return 1; // 校准完成
    }

    return 0; // 校准进行中
}

float MadgwickAHRS_get_pitch()
{
    float sinp = 2.0f * (s_Madgwick_info.q0 * s_Madgwick_info.q2 - s_Madgwick_info.q3 * s_Madgwick_info.q1);
    float pitch = (fabsf(sinp) >= 1.0f) ? copysignf(PI_2, sinp) : asinf(sinp);
    return pitch;
}

float MadgwickAHRS_get_roll()
{
    float roll = atan2f(2.0f * (s_Madgwick_info.q0 * s_Madgwick_info.q1 + s_Madgwick_info.q2 * s_Madgwick_info.q3),
                        1.0f - 2.0f * (s_Madgwick_info.q1 * s_Madgwick_info.q1 + s_Madgwick_info.q2 * s_Madgwick_info.q2));
    return roll;
}

float MadgwickAHRS_get_yaw()
{
    float yaw = atan2f(2.0f * (s_Madgwick_info.q0 * s_Madgwick_info.q3 + s_Madgwick_info.q1 * s_Madgwick_info.q2),
                       1.0f - 2.0f * (s_Madgwick_info.q2 * s_Madgwick_info.q2 + s_Madgwick_info.q3 * s_Madgwick_info.q3));
    return yaw;
}