#include <stdio.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <nuttx/sensors/cxd5602pwbimu.h>
#include <arch/board/cxd56_cxd5602pwbimu.h>
#include <stdbool.h>


#define CXD5602PWBIMU_DEVPATH      "/dev/imu0"
#define MAX_NFIFO (1)

#define GRAVITY_AMOUNT 9.80665f
#define EARTH_ROTATION_SPEED_AMOUNT 7.2921159e-5

#define MESUREMENT_FREQUENCY 1920
#define GYRO_NOISE_DENSITY (1.0e-3 * M_PI / 180.0f)
#define ACCEL_NOISE_DENSITY (14.0e-6 * GRAVITY_AMOUNT)
#define GYRO_NOISE_AMOUNT (GYRO_NOISE_DENSITY * sqrt(MESUREMENT_FREQUENCY))
#define ACCEL_NOISE_AMOUNT (ACCEL_NOISE_DENSITY * sqrt(MESUREMENT_FREQUENCY))
#define ACCEL_BIAS_DRIFT (4.43e-6 * GRAVITY_AMOUNT * 3.0f)
#define GYRO_BIAS_DRIFT (0.39f * M_PI / 180.0f)
// 観測ノイズの分散
#define GYRO_OBSERVATION_NOISE_VARIANCE (GYRO_NOISE_AMOUNT * GYRO_NOISE_AMOUNT)
#define ACCEL_OBSERVATION_NOISE_VARIANCE (ACCEL_NOISE_AMOUNT * ACCEL_NOISE_AMOUNT)
// プロセスノイズの分散
#define PROCESS_NOISE_VARIANCE (1.0e-7)

#define LIST_SIZE 8
#define SIGMA_K (LIST_SIZE / 8.0f)

// Madgwick Filterの重み
#define ACC_MADGWICK_FILTER_WEIGHT 0.11f //(sqrt(3.0f / 4.0f) * GYRO_NOISE_AMOUNT / ACCEL_NOISE_AMOUNT)
#define GYRO_MADGWICK_FILTER_WEIGHT 0.00000001f

static cxd5602pwbimu_data_t g_data[MAX_NFIFO];
int devfd;
int ret;

// グローバル変数（ゼロ速度補正用）
float biased_velocity = 0.0;
int zero_velocity_counter = 0;

bool is_initialized = false;
int current_list_num = 0;
float estimated_acceleration_x;
float estimated_acceleration_y;
float estimated_acceleration_z;
float estimated_rotation_speed_x;
float estimated_rotation_speed_y;
float estimated_rotation_speed_z;
float mesuared_acceleration_x[LIST_SIZE];
float mesuared_acceleration_y[LIST_SIZE];
float mesuared_acceleration_z[LIST_SIZE];
float mesuared_rotation_speed_x[LIST_SIZE];
float mesuared_rotation_speed_y[LIST_SIZE];
float mesuared_rotation_speed_z[LIST_SIZE];
float quaternion[4] = {1.0, 0.0, 0.0, 0.0};
float velocity[3] = {0.0, 0.0, 0.0};
float position[3] = {0.0, 0.0, 0.0};
float current_gravity[3] = {0.0, 0.0, 0.0};
int old_timestamp = -1;
int calibrate_counter = 0;

// クォータニオン微分関数：qは長さ4、omegaは長さ3、dqdtに結果を出力
void diff_quaternion(const float q[4], const float omega[3], float dqdt[4])
{
  float w = q[0], x = q[1], y = q[2], z = q[3];
  float omega_x = omega[0], omega_y = omega[1], omega_z = omega[2];

  dqdt[0] = 0.5 * (-x * omega_x - y * omega_y - z * omega_z);
  dqdt[1] = 0.5 * (w * omega_x + y * omega_z - z * omega_y);
  dqdt[2] = 0.5 * (w * omega_y - x * omega_z + z * omega_x);
  dqdt[3] = 0.5 * (w * omega_z + x * omega_y - y * omega_x);
}

// RK4法によるクォータニオン更新
void runge_kutta_update(const float q[4], const float omega[3], float h, float q_next[4])
{
  float k1[4], k2[4], k3[4], k4[4];
  float q1[4], q2[4], q3[4];
  int i;

  diff_quaternion(q, omega, k1);
  for (i = 0; i < 4; i++)
  {
    q1[i] = q[i] + (h / 2.0) * k1[i];
  }
  diff_quaternion(q1, omega, k2);
  for (i = 0; i < 4; i++)
  {
    q2[i] = q[i] + (h / 2.0) * k2[i];
  }
  diff_quaternion(q2, omega, k3);
  for (i = 0; i < 4; i++)
  {
    q3[i] = q[i] + h * k3[i];
  }
  diff_quaternion(q3, omega, k4);
  for (i = 0; i < 4; i++)
  {
    q_next[i] = q[i] + (h / 6.0) * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]);
  }
  // 正規化
  float norm = 0.0;
  for (i = 0; i < 4; i++)
  {
    norm += q_next[i] * q_next[i];
  }
  norm = sqrt(norm);
  for (i = 0; i < 4; i++)
  {
    q_next[i] /= norm;
  }
}

// 3次元ベクトルの外積
void cross_product(const float a[3], const float b[3], float result[3])
{
  result[0] = a[1] * b[2] - a[2] * b[1];
  result[1] = a[2] * b[0] - a[0] * b[2];
  result[2] = a[0] * b[1] - a[1] * b[0];
}

// RK4法による3次元ベクトルの更新
void update_vector_rk4(const float v[3], const float omega[3], float h, float v_next[3])
{
  float k1[3], k2[3], k3[3], k4[3];
  float temp[3];
  int i;

  cross_product(omega, v, k1);
  for (i = 0; i < 3; i++)
    temp[i] = v[i] + (h / 2.0) * k1[i];
  cross_product(omega, temp, k2);
  for (i = 0; i < 3; i++)
    temp[i] = v[i] + (h / 2.0) * k2[i];
  cross_product(omega, temp, k3);
  for (i = 0; i < 3; i++)
    temp[i] = v[i] + h * k3[i];
  cross_product(omega, temp, k4);
  for (i = 0; i < 3; i++)
    v_next[i] = v[i] + (h / 6.0) * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]);
}

// ゼロ速度補正関数
bool zero_velocity_correction(float target_velocity[3], float dt)
{
  biased_velocity += ACCEL_BIAS_DRIFT * dt;
  if (fabs(target_velocity[0]) < biased_velocity &&
      fabs(target_velocity[1]) < biased_velocity &&
      fabs(target_velocity[2]) < biased_velocity)
  {
    zero_velocity_counter++;
  }
  else
  {
    zero_velocity_counter = 0;
  }
  if (zero_velocity_counter > MESUREMENT_FREQUENCY)
  {
    biased_velocity = 0.0;
    zero_velocity_counter = 0;
    return true;
  }
  return false;
}

// 現在の時刻におけるフィルタ結果を計算（x: 入力配列、n: データ数、kernel: ガウスカーネル、K: カーネルサイズ）
float apply_causal_gaussian_filter(const float *x, int list_num)
{
  static float kernel[LIST_SIZE];
  static bool is_kernel_initialized = false;
  if (!is_kernel_initialized)
  {
    float sum = 0.0;
    for (int i = 0; i < LIST_SIZE; i++)
    {
      kernel[i] = (1.0 / (sqrt(2.0 * M_PI) * SIGMA_K)) * exp(-(i * i) / (2.0 * SIGMA_K * SIGMA_K));
      sum += kernel[i];
    }
    // 正規化
    for (int i = 0; i < LIST_SIZE; i++)
    {
      kernel[i] /= sum;
    }

    is_kernel_initialized = true;
  }

  float y_current = 0.0;
  // 最新のデータが x[n-1] とし、過去方向にカーネルを適用
  for (int i = 0; i < LIST_SIZE; i++)
  {
    int idx = LIST_SIZE - 1 - i;
    int list_idx = (list_num + idx) % LIST_SIZE;
    y_current += kernel[i] * x[list_idx];
  }
  return y_current;
}

// 回転行列を使って3次元ベクトルに回転を適用する関数
void apply_rotation(const float *q, const float *v, float result[3])
{
  // クォータニオンの逆を計算
  float q_conjugate[4] = {q[0], -q[1], -q[2], -q[3]};

  // 入力ベクトルをクォータニオン形式に変換
  float v_quat[4] = {0.0f, v[0], v[1], v[2]};

  // q * v_quat を計算
  float temp[4];
  temp[0] = q[0] * v_quat[0] - q[1] * v_quat[1] - q[2] * v_quat[2] - q[3] * v_quat[3];
  temp[1] = q[0] * v_quat[1] + q[1] * v_quat[0] + q[2] * v_quat[3] - q[3] * v_quat[2];
  temp[2] = q[0] * v_quat[2] - q[1] * v_quat[3] + q[2] * v_quat[0] + q[3] * v_quat[1];
  temp[3] = q[0] * v_quat[3] + q[1] * v_quat[2] - q[2] * v_quat[1] + q[3] * v_quat[0];

  // temp * q_conjugate を計算
  float rotated[4];
  rotated[0] = temp[0] * q_conjugate[0] - temp[1] * q_conjugate[1] - temp[2] * q_conjugate[2] - temp[3] * q_conjugate[3];
  rotated[1] = temp[0] * q_conjugate[1] + temp[1] * q_conjugate[0] + temp[2] * q_conjugate[3] - temp[3] * q_conjugate[2];
  rotated[2] = temp[0] * q_conjugate[2] - temp[1] * q_conjugate[3] + temp[2] * q_conjugate[0] + temp[3] * q_conjugate[1];
  rotated[3] = temp[0] * q_conjugate[3] + temp[1] * q_conjugate[2] - temp[2] * q_conjugate[1] + temp[3] * q_conjugate[0];

  // 結果をベクトル形式に変換
  result[0] = rotated[1];
  result[1] = rotated[2];
  result[2] = rotated[3];
}

bool imu_data_initialize(cxd5602pwbimu_data_t dat)
{
  static int initialize_counter = 0;
  static float initialize_acceleration_x_sum = 0.0;
  static float initialize_acceleration_y_sum = 0.0;
  static float initialize_acceleration_z_sum = 0.0;
  static float initialize_rotation_speed_x_sum = 0.0;
  static float initialize_rotation_speed_y_sum = 0.0;
  static float initialize_rotation_speed_z_sum = 0.0;
  if (initialize_counter < MESUREMENT_FREQUENCY)
  {
    initialize_acceleration_x_sum += dat.ax;
    initialize_acceleration_y_sum += dat.ay;
    initialize_acceleration_z_sum += dat.az;
    initialize_rotation_speed_x_sum += dat.gx;
    initialize_rotation_speed_y_sum += dat.gy;
    initialize_rotation_speed_z_sum += dat.gz;
    initialize_counter++;

    return false;
  }
  else
  {
    estimated_acceleration_x = initialize_acceleration_x_sum / MESUREMENT_FREQUENCY;
    estimated_acceleration_y = initialize_acceleration_y_sum / MESUREMENT_FREQUENCY;
    estimated_acceleration_z = initialize_acceleration_z_sum / MESUREMENT_FREQUENCY;
    float average_acceleration_norm = sqrt(estimated_acceleration_x * estimated_acceleration_x +
                                           estimated_acceleration_y * estimated_acceleration_y +
                                           estimated_acceleration_z * estimated_acceleration_z);
    estimated_rotation_speed_x = initialize_rotation_speed_x_sum / MESUREMENT_FREQUENCY;
    estimated_rotation_speed_y = initialize_rotation_speed_y_sum / MESUREMENT_FREQUENCY;
    estimated_rotation_speed_z = initialize_rotation_speed_z_sum / MESUREMENT_FREQUENCY;
    float dot_product = estimated_acceleration_x * estimated_rotation_speed_x +
                        estimated_acceleration_y * estimated_rotation_speed_y +
                        estimated_acceleration_z * estimated_rotation_speed_z;
    float earth_rotation_speed_x = estimated_rotation_speed_x - dot_product * estimated_acceleration_x / average_acceleration_norm / average_acceleration_norm;
    float earth_rotation_speed_y = estimated_rotation_speed_y - dot_product * estimated_acceleration_y / average_acceleration_norm / average_acceleration_norm;
    float earth_rotation_speed_z = estimated_rotation_speed_z - dot_product * estimated_acceleration_z / average_acceleration_norm / average_acceleration_norm;
    float earth_speed_norm = sqrt(earth_rotation_speed_x * earth_rotation_speed_x +
                                  earth_rotation_speed_y * earth_rotation_speed_y +
                                  earth_rotation_speed_z * earth_rotation_speed_z);
    initialize_counter = 0;
    initialize_acceleration_x_sum = 0.0f;
    initialize_acceleration_y_sum = 0.0f;
    initialize_acceleration_z_sum = 0.0f;
    initialize_rotation_speed_x_sum = 0.0f;
    initialize_rotation_speed_y_sum = 0.0f;
    initialize_rotation_speed_z_sum = 0.0f;

    for (int i = 0; i < LIST_SIZE; i++)
    {
      mesuared_acceleration_x[i] = estimated_acceleration_x;
      mesuared_acceleration_y[i] = estimated_acceleration_y;
      mesuared_acceleration_z[i] = estimated_acceleration_z;
      mesuared_rotation_speed_x[i] = estimated_rotation_speed_x;
      mesuared_rotation_speed_y[i] = estimated_rotation_speed_y;
      mesuared_rotation_speed_z[i] = estimated_rotation_speed_z;
    }

    current_gravity[0] = estimated_acceleration_x;
    current_gravity[1] = estimated_acceleration_y;
    current_gravity[2] = estimated_acceleration_z;

    // 単位ベクトル計算
    float z_axis_x = estimated_acceleration_x / average_acceleration_norm;
    float z_axis_y = estimated_acceleration_y / average_acceleration_norm;
    float z_axis_z = estimated_acceleration_z / average_acceleration_norm;
    float y_axis_x = earth_rotation_speed_x / earth_speed_norm;
    float y_axis_y = earth_rotation_speed_y / earth_speed_norm;
    float y_axis_z = earth_rotation_speed_z / earth_speed_norm;
    float x_axis_x = y_axis_y * z_axis_z - y_axis_z * z_axis_y;
    float x_axis_y = y_axis_z * z_axis_x - y_axis_x * z_axis_z;
    float x_axis_z = y_axis_x * z_axis_y - y_axis_y * z_axis_x;
    float trace = x_axis_x + y_axis_y + z_axis_z;
    if (trace > 0)
    {
      float s = sqrt(trace + 1.0) * 2.0;
      quaternion[0] = 0.25 * s;
      quaternion[1] = (y_axis_z - z_axis_y) / s;
      quaternion[2] = (z_axis_x - x_axis_z) / s;
      quaternion[3] = (x_axis_y - y_axis_x) / s;
    }
    else if (x_axis_x > y_axis_y && x_axis_x > z_axis_z)
    {
      float s = sqrt(1.0 + x_axis_x - y_axis_y - z_axis_z) * 2.0;
      quaternion[0] = (y_axis_z - z_axis_y) / s;
      quaternion[1] = 0.25 * s;
      quaternion[2] = (x_axis_y + y_axis_x) / s;
      quaternion[3] = (z_axis_x + x_axis_z) / s;
    }
    else if (y_axis_y > z_axis_z)
    {
      float s = sqrt(1.0 + y_axis_y - x_axis_x - z_axis_z) * 2.0;
      quaternion[0] = (z_axis_x - x_axis_z) / s;
      quaternion[1] = (x_axis_y + y_axis_x) / s;
      quaternion[2] = 0.25 * s;
      quaternion[3] = (y_axis_z + z_axis_y) / s;
    }
    else
    {
      float s = sqrt(1.0 + z_axis_z - x_axis_x - y_axis_y) * 2.0;
      quaternion[0] = (x_axis_y - y_axis_x) / s;
      quaternion[1] = (z_axis_x + x_axis_z) / s;
      quaternion[2] = (y_axis_z + z_axis_y) / s;
      quaternion[3] = 0.25 * s;
    }
    return true;
  }
  return false;
}

void update(cxd5602pwbimu_data_t dat)
{
  float dt = 1.0 / MESUREMENT_FREQUENCY;
  if (old_timestamp == -1)
  {
    old_timestamp = dat.timestamp;
  }
  else
  {
    dt = (dat.timestamp - old_timestamp) / 19200000.0f;
    old_timestamp = dat.timestamp;
  }

  mesuared_acceleration_x[current_list_num] = dat.ax;
  mesuared_acceleration_y[current_list_num] = dat.ay;
  mesuared_acceleration_z[current_list_num] = dat.az;
  mesuared_rotation_speed_x[current_list_num] = dat.gx;
  mesuared_rotation_speed_y[current_list_num] = dat.gy;
  mesuared_rotation_speed_z[current_list_num] = dat.gz;

  // ガウスフィルタを適用
  estimated_acceleration_x = apply_causal_gaussian_filter(mesuared_acceleration_x, current_list_num);
  estimated_acceleration_y = apply_causal_gaussian_filter(mesuared_acceleration_y, current_list_num);
  estimated_acceleration_z = apply_causal_gaussian_filter(mesuared_acceleration_z, current_list_num);
  estimated_rotation_speed_x = apply_causal_gaussian_filter(mesuared_rotation_speed_x, current_list_num);
  estimated_rotation_speed_y = apply_causal_gaussian_filter(mesuared_rotation_speed_y, current_list_num);
  estimated_rotation_speed_z = apply_causal_gaussian_filter(mesuared_rotation_speed_z, current_list_num);

  // 内積を引く
  float acceleration_norm = sqrt(estimated_acceleration_x * estimated_acceleration_x +
                                 estimated_acceleration_y * estimated_acceleration_y +
                                 estimated_acceleration_z * estimated_acceleration_z);
  float dot_product = estimated_acceleration_x * estimated_rotation_speed_x +
                      estimated_acceleration_y * estimated_rotation_speed_y +
                      estimated_acceleration_z * estimated_rotation_speed_z;
  float earth_rotation_speed_x = estimated_rotation_speed_x - dot_product * estimated_acceleration_x / acceleration_norm / acceleration_norm;
  float earth_rotation_speed_y = estimated_rotation_speed_y - dot_product * estimated_acceleration_y / acceleration_norm / acceleration_norm;
  float earth_rotation_speed_z = estimated_rotation_speed_z - dot_product * estimated_acceleration_z / acceleration_norm / acceleration_norm;
  float earth_rotation_speed_norm = sqrt(earth_rotation_speed_x * earth_rotation_speed_x +
                                         earth_rotation_speed_y * earth_rotation_speed_y +
                                         earth_rotation_speed_z * earth_rotation_speed_z);

  // キャリブレーション条件のチェック
  if (acceleration_norm < GRAVITY_AMOUNT + ACCEL_NOISE_AMOUNT * 40 &&
      acceleration_norm > GRAVITY_AMOUNT - ACCEL_NOISE_AMOUNT * 40 &&
      earth_rotation_speed_norm < EARTH_ROTATION_SPEED_AMOUNT + GYRO_NOISE_AMOUNT * 2.0 &&
      earth_rotation_speed_norm > EARTH_ROTATION_SPEED_AMOUNT / 2.0)
  {
    if (calibrate_counter < MESUREMENT_FREQUENCY / 30)
    {
      calibrate_counter++;
    }
    else
    {
      float normalized_acceleration_x = estimated_acceleration_x / acceleration_norm;
      float normalized_acceleration_y = estimated_acceleration_y / acceleration_norm;
      float normalized_acceleration_z = estimated_acceleration_z / acceleration_norm;
      float normalized_earth_rotation_speed_x = earth_rotation_speed_x / earth_rotation_speed_norm;
      float normalized_earth_rotation_speed_y = earth_rotation_speed_y / earth_rotation_speed_norm;
      float normalized_earth_rotation_speed_z = earth_rotation_speed_z / earth_rotation_speed_norm;

      // f(q, acc)
      float f_q_acc[3] = {
          2 * (quaternion[1] * quaternion[3] - quaternion[0] * quaternion[2]) - normalized_acceleration_x,
          2 * (quaternion[0] * quaternion[1] + quaternion[2] * quaternion[3]) - normalized_acceleration_y,
          2 * (0.5 - quaternion[1] * quaternion[1] - quaternion[2] * quaternion[2]) - normalized_acceleration_z};

      // J(q, acc)
      float j_q_acc[3][4] = {
          {2 * quaternion[2], -2 * quaternion[3], 2 * quaternion[0], -2 * quaternion[1]},
          {2 * quaternion[1], 2 * quaternion[0], 2 * quaternion[3], 2 * quaternion[2]},
          {0, -4 * quaternion[1], -4 * quaternion[2], 0}};
      float step_acc[4];
      for (int i = 0; i < 4; i++)
      {
        step_acc[i] = 0.0;
        for (int j = 0; j < 3; j++)
        {
          step_acc[i] += j_q_acc[i][j] * f_q_acc[j];
        }
      }
      float step_acc_norm = sqrt(step_acc[0] * step_acc[0] +
                                 step_acc[1] * step_acc[1] +
                                 step_acc[2] * step_acc[2] +
                                 step_acc[3] * step_acc[3]);

      float f_q_gyro[3] = {
          2 * (quaternion[1] * quaternion[2] + quaternion[0] * quaternion[3]) - normalized_earth_rotation_speed_x,
          2 * (0.5 - quaternion[0] * quaternion[0] - quaternion[2] * quaternion[2]) - normalized_earth_rotation_speed_y,
          2 * (quaternion[2] * quaternion[3] - quaternion[0] * quaternion[1]) - normalized_earth_rotation_speed_z};
      // J(q, gyro)
      float j_q_gyro[3][4] = {
          {2 * quaternion[3], 2 * quaternion[2], 2 * quaternion[1], 2 * quaternion[0]},
          {2 * quaternion[0], -2 * quaternion[1], 2 * quaternion[2], -2 * quaternion[3]},
          {-2 * quaternion[1], -2 * quaternion[0], -2 * quaternion[3], -2 * quaternion[2]}};
      float step_gyro[4];
      for (int i = 0; i < 4; i++)
      {
        step_gyro[i] = 0.0;
        for (int j = 0; j < 3; j++)
        {
          step_gyro[i] += j_q_gyro[i][j] * f_q_gyro[j];
        }
      }
      float step_gyro_norm = sqrt(step_gyro[0] * step_gyro[0] +
                                  step_gyro[1] * step_gyro[1] +
                                  step_gyro[2] * step_gyro[2] +
                                  step_gyro[3] * step_gyro[3]);
      for (int i = 0; i < 4; i++)
      {
        quaternion[i] -= step_acc[i] * ACC_MADGWICK_FILTER_WEIGHT / step_acc_norm * dt;
        quaternion[i] -= step_gyro[i] * GYRO_MADGWICK_FILTER_WEIGHT / step_gyro_norm * dt;
      }

      // 正規化
      float quaternion_norm = sqrt(quaternion[0] * quaternion[0] +
                                   quaternion[1] * quaternion[1] +
                                   quaternion[2] * quaternion[2] +
                                   quaternion[3] * quaternion[3]);
      for (int i = 0; i < 4; i++)
      {
        quaternion[i] /= quaternion_norm;
      }

      current_gravity[0] = estimated_acceleration_x;
      current_gravity[1] = estimated_acceleration_y;
      current_gravity[2] = estimated_acceleration_z;

      velocity[0] = 0.0;
      velocity[1] = 0.0;
      velocity[2] = 0.0;
      calibrate_counter = 0;
    }
  }
  else
  {
    calibrate_counter = 0;
  }

  // クォータニオン更新（更新結果は返り値として受け取る）
  float estimated_rotation_speed[3] = {estimated_rotation_speed_x, estimated_rotation_speed_y, estimated_rotation_speed_z};
  float estimated_rotation_speed_minus[3] = {-estimated_rotation_speed_x, -estimated_rotation_speed_y, -estimated_rotation_speed_z};
  runge_kutta_update(quaternion, estimated_rotation_speed, dt, quaternion);
  update_vector_rk4(current_gravity, estimated_rotation_speed_minus, dt, current_gravity);

  float acceleration[3] = {estimated_acceleration_x - current_gravity[0], estimated_acceleration_y - current_gravity[1], estimated_acceleration_z - current_gravity[2]};
  apply_rotation(quaternion, acceleration, acceleration);
  estimated_acceleration_x = acceleration[0];
  estimated_acceleration_y = acceleration[1];
  estimated_acceleration_z = acceleration[2];

  // 速度・位置の更新（単純オイラー積分）
  velocity[0] += (estimated_acceleration_x)*dt;
  velocity[1] += (estimated_acceleration_y)*dt;
  velocity[2] += (estimated_acceleration_z)*dt;
  position[0] += velocity[0] * dt;
  position[1] += velocity[1] * dt;
  position[2] += velocity[2] * dt;

  if (zero_velocity_correction(velocity, dt))
  {
    velocity[0] = 0.0;
    velocity[1] = 0.0;
    velocity[2] = 0.0;
  }

  current_list_num = (current_list_num + 1) % LIST_SIZE;

  return;
}

static int start_sensing(int fd, int rate, int adrange, int gdrange,
                         int nfifos)
{
  cxd5602pwbimu_range_t range;

  ioctl(fd, SNIOC_SSAMPRATE, rate);
  range.accel = adrange;
  range.gyro = gdrange;
  ioctl(fd, SNIOC_SDRANGE, (unsigned long)(uintptr_t)&range);
  ioctl(fd, SNIOC_SFIFOTHRESH, nfifos);
  ioctl(fd, SNIOC_ENABLE, 1);

  return 0;
}

static int drop_50msdata(int fd, int samprate)
{
  int cnt = samprate / 20; /* data size of 50ms */

  cnt = ((cnt + MAX_NFIFO - 1) / MAX_NFIFO) * MAX_NFIFO;
  if (cnt == 0) cnt = MAX_NFIFO;

  while (cnt)
    {
      read(fd, g_data, sizeof(g_data[0]) * MAX_NFIFO);
      cnt -= MAX_NFIFO;
    }

  return 0;
}

void setup() 
{
  Serial.begin( 115200 );

  board_cxd5602pwbimu_initialize(5);

  devfd = open(CXD5602PWBIMU_DEVPATH, O_RDONLY);

  start_sensing(devfd, MESUREMENT_FREQUENCY, 8, 2000, MAX_NFIFO);
  drop_50msdata(devfd, MESUREMENT_FREQUENCY);
  // dump_data(devfd);
  bool is_initialized = false;
  while (!is_initialized)
  {
    ret = read(devfd, g_data, sizeof(g_data[0]) * MAX_NFIFO);
    if(ret == sizeof(g_data[0]) * MAX_NFIFO) 
    {
      for (int i = 0; i < MAX_NFIFO; i++)
      {
        is_initialized = imu_data_initialize(g_data[i]);
      }
    }
  }
}

void loop() {
  static int execute_counter = 0;
  ret = read(devfd, g_data, sizeof(g_data[0]) * MAX_NFIFO);
  if(ret == sizeof(g_data[0]) * MAX_NFIFO) {
    for(int i=0; i < MAX_NFIFO; i++)
    {
      update(g_data[i]);

      execute_counter++;
      if (execute_counter >= MESUREMENT_FREQUENCY / 30)
      {
        Serial.printf( "%08x,%08x,%08x,%08x,"
             "%08x,%08x,%08x,%08x,"
             "%08x,%08x,%08x,%08x,"
             "%08x,%08x,%08x,"
             "%08x,%08x,%08x\n",
             (unsigned int)g_data[i].timestamp,
             *(unsigned int *)&g_data[i].temp,
             *(unsigned int *)&estimated_rotation_speed_x,
             *(unsigned int *)&estimated_rotation_speed_y,
             *(unsigned int *)&estimated_rotation_speed_z,
             *(unsigned int *)&estimated_acceleration_x,
             *(unsigned int *)&estimated_acceleration_y,
             *(unsigned int *)&estimated_acceleration_z,
             *(unsigned int *)&quaternion[0],
             *(unsigned int *)&quaternion[1],
             *(unsigned int *)&quaternion[2],
             *(unsigned int *)&quaternion[3],
             *(unsigned int *)&velocity[0],
             *(unsigned int *)&velocity[1],
             *(unsigned int *)&velocity[2],
             *(unsigned int *)&position[0],
             *(unsigned int *)&position[1],
             *(unsigned int *)&position[2]);
        execute_counter = 0;
      }
    }
  }
}