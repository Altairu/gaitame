// Spresense Multi-IMU を用いた自己位置推定例（Arduino IDE用）
//
// センサ初期化、運動状態の推定（クォータニオンによる姿勢推定、単純オイラー積分による速度・位置計算）を行い，
// Serial出力により推定結果（姿勢：roll, pitch, yaw，速度，位置）を表示する。
// ※ 実際の環境に応じてパラメータやフィルタ処理は調整してください。

#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <poll.h>
#include <errno.h>
#include <math.h>
#include <sys/ioctl.h>
#include <stdint.h>
#include <Arduino.h> // Arduino環境ではこのヘッダで millis() が提供される

// PWBIMU用ヘッダ
#include <nuttx/sensors/cxd5602pwbimu.h>

extern "C" int board_cxd5602pwbimu_initialize(int);

#define IMU_RATE 240    // サンプリングレート [Hz]
#define IMU_ADRANGE 4   // 加速度レンジ [G]
#define IMU_GDRANGE 500 // ジャイロレンジ [dps]
#define IMU_FIFO 1      // FIFO設定

// 測定周波数等のパラメータ
#define MESUREMENT_FREQUENCY IMU_RATE
#define LIST_SIZE 8
#define SIGMA_K (LIST_SIZE / 8.0f)
#define ACC_MADGWICK_FILTER_WEIGHT 0.11f
#define GYRO_MADGWICK_FILTER_WEIGHT 0.00000001f
#define GRAVITY_AMOUNT 9.80665f
#define ACCEL_NOISE_AMOUNT (14.0e-6 * GRAVITY_AMOUNT)
#define ACCEL_BIAS_DRIFT (4.43e-6 * GRAVITY_AMOUNT * 3.0f)
#define EARTH_ROTATION_SPEED_AMOUNT 7.2921159e-5
#define GYRO_NOISE_AMOUNT (1.0e-3 * M_PI / 180.0f)

// センサファイルディスクリプタ
static int fd;

// 時間管理用
static unsigned long last_time = 0;

// 推定状態（グローバル変数）
static float quaternion[4] = {1.0f, 0.0f, 0.0f, 0.0f};
static float velocity[3] = {0.0f, 0.0f, 0.0f};
static float position[3] = {0.0f, 0.0f, 0.0f};

// センサデータフィルタリング用リングバッファ
static float acc_buf_x[LIST_SIZE] = {0};
static float acc_buf_y[LIST_SIZE] = {0};
static float acc_buf_z[LIST_SIZE] = {0}; // Z軸用リングバッファ
static int buf_index = 0;

// 内部計算用一時変数
static int init_counter = 0;
static bool is_initialized = false;
static int zero_velocity_counter = 0;
static float biased_velocity = 0.0f;

// 初期加速度補正用変数
static float initial_acc[2] = {0.0f, 0.0f}; // X, Y軸の初期加速度

// 初期重力加速度補正用変数
static float gravity[3] = {0.0f, 0.0f, 0.0f}; // X, Y, Z軸の重力加速度

// ----- クォータニオン微分・更新等の関数 -----
void diff_quaternion(const float q[4], const float omega[3], float dqdt[4])
{
  float w = q[0], x = q[1], y = q[2], z = q[3];
  dqdt[0] = 0.5f * (-x * omega[0] - y * omega[1] - z * omega[2]);
  dqdt[1] = 0.5f * (w * omega[0] + y * omega[2] - z * omega[1]);
  dqdt[2] = 0.5f * (w * omega[1] - x * omega[2] + z * omega[0]);
  dqdt[3] = 0.5f * (w * omega[2] + x * omega[1] - y * omega[0]);
}

// Runge-Kutta 4次法によるクォータニオン更新
void runge_kutta_update(const float q[4], const float omega[3], float h, float q_next[4])
{
  float k1[4], k2[4], k3[4], k4[4], temp[4];
  diff_quaternion(q, omega, k1);
  for (int i = 0; i < 4; i++)
  {
    temp[i] = q[i] + (h / 2.0f) * k1[i];
  }
  diff_quaternion(temp, omega, k2);
  for (int i = 0; i < 4; i++)
  {
    temp[i] = q[i] + (h / 2.0f) * k2[i];
  }
  diff_quaternion(temp, omega, k3);
  for (int i = 0; i < 4; i++)
  {
    temp[i] = q[i] + h * k3[i];
  }
  diff_quaternion(temp, omega, k4);
  for (int i = 0; i < 4; i++)
  {
    q_next[i] = q[i] + (h / 6.0f) * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]);
  }
  // 正規化
  float norm = 0.0f;
  for (int i = 0; i < 4; i++)
  {
    norm += q_next[i] * q_next[i];
  }
  norm = sqrtf(norm);
  for (int i = 0; i < 4; i++)
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

// RK4法による3次元ベクトル更新（例：重心方向更新用）
void update_vector_rk4(const float v[3], const float omega[3], float h, float v_next[3])
{
  float k1[3], k2[3], k3[3], k4[3], temp[3];
  cross_product(omega, v, k1);
  for (int i = 0; i < 3; i++)
    temp[i] = v[i] + (h / 2.0f) * k1[i];
  cross_product(omega, temp, k2);
  for (int i = 0; i < 3; i++)
    temp[i] = v[i] + (h / 2.0f) * k2[i];
  cross_product(omega, temp, k3);
  for (int i = 0; i < 3; i++)
    temp[i] = v[i] + h * k3[i];
  cross_product(omega, temp, k4);
  for (int i = 0; i < 3; i++)
    v_next[i] = v[i] + (h / 6.0f) * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]);
}

// カジュアルなガウスフィルタ：最新の LIST_SIZE 個の値にカーネルを適用
float apply_causal_gaussian_filter(const float *x, int current)
{
  static float kernel[LIST_SIZE];
  static bool kernel_initialized = false;
  if (!kernel_initialized)
  {
    float sum = 0.0f;
    for (int i = 0; i < LIST_SIZE; i++)
    {
      kernel[i] = (1.0f / (sqrtf(2.0f * M_PI) * SIGMA_K)) * expf(-((i * i)) / (2.0f * SIGMA_K * SIGMA_K));
      sum += kernel[i];
    }
    for (int i = 0; i < LIST_SIZE; i++)
    {
      kernel[i] /= sum;
    }
    kernel_initialized = true;
  }
  float y = 0.0f;
  for (int i = 0; i < LIST_SIZE; i++)
  {
    int idx = (current + LIST_SIZE - i) % LIST_SIZE;
    y += kernel[i] * x[idx];
  }
  return y;
}

// クォータニオンを使ってベクトルに回転を適用する
void apply_rotation(const float *q, const float *v, float result[3])
{
  float q_conj[4] = {q[0], -q[1], -q[2], -q[3]};
  float vq[4] = {0.0f, v[0], v[1], v[2]};
  float temp[4];
  temp[0] = q[0] * vq[0] - q[1] * vq[1] - q[2] * vq[2] - q[3] * vq[3];
  temp[1] = q[0] * vq[1] + q[1] * vq[0] + q[2] * vq[3] - q[3] * vq[2];
  temp[2] = q[0] * vq[2] - q[1] * vq[3] + q[2] * vq[0] + q[3] * vq[1];
  temp[3] = q[0] * vq[3] + q[1] * vq[2] - q[2] * vq[1] + q[3] * vq[0];
  float r[4];
  r[0] = temp[0] * q_conj[0] - temp[1] * q_conj[1] - temp[2] * q_conj[2] - temp[3] * q_conj[3];
  r[1] = temp[0] * q_conj[1] + temp[1] * q_conj[0] + temp[2] * q_conj[3] - temp[3] * q_conj[2];
  r[2] = temp[0] * q_conj[2] - temp[1] * q_conj[3] + temp[2] * q_conj[0] + temp[3] * q_conj[1];
  r[3] = temp[0] * q_conj[3] + temp[1] * q_conj[2] - temp[2] * q_conj[1] + temp[3] * q_conj[0];
  result[0] = r[1];
  result[1] = r[2];
  result[2] = r[3];
}

// 重力補正の強化
void apply_gravity_correction(float acc[3], const float gravity[3], float corrected_acc[3])
{
  for (int i = 0; i < 3; i++)
  {
    corrected_acc[i] = acc[i] - gravity[i];
  }
}

// Yaw（θz）のみを考慮したクォータニオン更新
void update_yaw_quaternion(const float q[4], float yaw_rate, float dt, float q_next[4])
{
  float delta_yaw = yaw_rate * dt;
  float half_delta_yaw = delta_yaw / 2.0f;
  float cos_half_yaw = cosf(half_delta_yaw);
  float sin_half_yaw = sinf(half_delta_yaw);

  // 現在のクォータニオンにYaw回転を適用
  q_next[0] = cos_half_yaw * q[0] - sin_half_yaw * q[3];
  q_next[1] = q[1];
  q_next[2] = q[2];
  q_next[3] = sin_half_yaw * q[0] + cos_half_yaw * q[3];
}

// Yaw（θz）のみを考慮したクォータニオン更新
void update_yaw(float yaw_rate, float dt, float &yaw)
{
  yaw += yaw_rate * dt;
  if (yaw > M_PI)
    yaw -= 2 * M_PI;
  else if (yaw < -M_PI)
    yaw += 2 * M_PI;
}

// 加速度を世界座標系に変換（Yaw角のみを考慮）
void transform_acceleration(float acc_x, float acc_y, float yaw, float &world_acc_x, float &world_acc_y)
{
  world_acc_x = acc_x * cosf(yaw) - acc_y * sinf(yaw);
  world_acc_y = acc_x * sinf(yaw) + acc_y * cosf(yaw);
}

// 加速度を補正する関数
void correct_acceleration(float &acc_x, float &acc_y, float yaw)
{
  // 初期加速度をYaw角に基づいて補正
  float corrected_x = acc_x - (initial_acc[0] * cosf(yaw) - initial_acc[1] * sinf(yaw));
  float corrected_y = acc_y - (initial_acc[0] * sinf(yaw) + initial_acc[1] * cosf(yaw));
  acc_x = corrected_x;
  acc_y = corrected_y;
}

// 重力加速度を補正する関数
void remove_gravity(float &acc_x, float &acc_y, float yaw)
{
  // 重力加速度をYaw角に基づいて補正
  float gravity_x = gravity[0] * cosf(yaw) - gravity[1] * sinf(yaw);
  float gravity_y = gravity[0] * sinf(yaw) + gravity[1] * cosf(yaw);
  acc_x -= gravity_x;
  acc_y -= gravity_y;
}

// 重力加速度を除去する関数
void remove_gravity_with_orientation(float acc[3], const float q[4], float corrected_acc[3])
{
  // 推定された姿勢（クォータニオン）を使用して重力加速度を除去
  float gravity[3] = {0.0f, 0.0f, GRAVITY_AMOUNT}; // 重力加速度ベクトル（世界座標系）
  float rotated_gravity[3];
  apply_rotation(q, gravity, rotated_gravity); // 重力ベクトルをローカル座標系に変換

  for (int i = 0; i < 3; i++)
  {
    corrected_acc[i] = acc[i] - rotated_gravity[i]; // 重力加速度を除去
  }
}

// センサデータから自己位置推定の更新を行う
bool update_state(cxd5602pwbimu_data_t dat)
{
  unsigned long now = millis();
  float dt = (now - last_time) / 1000.0f;
  last_time = now;
  if (dt <= 0)
    dt = 1.0f / IMU_RATE;

  // ラジアン変換（dps -> rad/s）
  const float PI_F = static_cast<float>(M_PI); // M_PIをfloat型にキャスト
  float omega[3] = {dat.gx * (PI_F / 180.0f), dat.gy * (PI_F / 180.0f), dat.gz * (PI_F / 180.0f)};

  // クォータニオンの更新（姿勢推定）
  float new_quaternion[4];
  runge_kutta_update(quaternion, omega, dt, new_quaternion);
  for (int i = 0; i < 4; i++)
  {
    quaternion[i] = new_quaternion[i];
  }

  // 加速度データのフィルタリング
  acc_buf_x[buf_index] = dat.ax;
  acc_buf_y[buf_index] = dat.ay;
  acc_buf_z[buf_index] = dat.az; // Z軸データを追加
  float f_ax = apply_causal_gaussian_filter(acc_buf_x, buf_index);
  float f_ay = apply_causal_gaussian_filter(acc_buf_y, buf_index);
  float f_az = apply_causal_gaussian_filter(acc_buf_z, buf_index); // Z軸フィルタリング
  buf_index = (buf_index + 1) % LIST_SIZE;

  // 初期キャリブレーション：一定期間静止状態と仮定
  if (!is_initialized)
  {
    init_counter++;
    if (init_counter >= MESUREMENT_FREQUENCY)
    {
      gravity[0] = f_ax; // 初期X軸重力加速度を記録
      gravity[1] = f_ay; // 初期Y軸重力加速度を記録
      gravity[2] = f_az; // Z軸重力加速度を記録
      is_initialized = true;
    }
    return false;
  }

  // 重力加速度を除去
  float acc[3] = {f_ax, f_ay, f_az};
  float corrected_acc[3];
  remove_gravity_with_orientation(acc, quaternion, corrected_acc);

  // デバッグ用出力
  Serial.printf("Debug: Corrected Acc: X=%.6f, Y=%.6f, Z=%.6f\n", corrected_acc[0], corrected_acc[1], corrected_acc[2]);

  // 加速度を世界座標系に変換
  float world_acc_x, world_acc_y, world_acc_z;
  apply_rotation(quaternion, corrected_acc, corrected_acc); // 全軸を変換
  world_acc_x = corrected_acc[0];
  world_acc_y = corrected_acc[1];
  world_acc_z = corrected_acc[2];

  // デバッグ用出力
  Serial.printf("Debug: World Acc: X=%.6f, Y=%.6f, Z=%.6f\n", world_acc_x, world_acc_y, world_acc_z);

  // 速度・位置更新（オイラー積分）
  velocity[0] += world_acc_x * dt;
  velocity[1] += world_acc_y * dt;
  velocity[2] += world_acc_z * dt;

  position[0] += velocity[0] * dt * 1000.0f; // mm単位に変換
  position[1] += velocity[1] * dt * 1000.0f; // mm単位に変換
  position[2] += velocity[2] * dt * 1000.0f; // Z軸の位置更新（mm単位に変換）

  // デバッグ用シリアル出力
  Serial.printf("Debug: AccZ=%.6f, VelZ=%.6f, PosZ=%.6f\n", corrected_acc[2], velocity[2], position[2]);

  // 簡易ゼロ速度補正
  biased_velocity += ACCEL_BIAS_DRIFT * dt;
  if (fabs(velocity[0]) < biased_velocity &&
      fabs(velocity[1]) < biased_velocity)
  {
    zero_velocity_counter++;
  }
  else
  {
    zero_velocity_counter = 0;
  }
  if (zero_velocity_counter > MESUREMENT_FREQUENCY)
  {
    velocity[0] = velocity[1] = 0.0f;
    zero_velocity_counter = 0;
    biased_velocity = 0.0f;
  }

  return true;
}

// --------------------------------------------------------
// setup(): センサ初期化、初期設定
// --------------------------------------------------------
void setup()
{
  Serial.begin(115200);
  while (!Serial)
  {
  } // シリアル待ち

  int ret = board_cxd5602pwbimu_initialize(5);
  if (ret < 0)
  {
    Serial.println("ERROR: Failed to initialize CXD5602PWBIMU.");
    return;
  }
  fd = open("/dev/imu0", O_RDONLY);
  if (fd < 0)
  {
    Serial.println("ERROR: Could not open /dev/imu0");
    return;
  }
  ret = ioctl(fd, SNIOC_SSAMPRATE, IMU_RATE);
  if (ret)
  {
    Serial.print("ERROR: Set sampling rate failed. ");
    Serial.println(errno);
    return;
  }
  cxd5602pwbimu_range_t range;
  range.accel = IMU_ADRANGE;
  range.gyro = IMU_GDRANGE;
  ret = ioctl(fd, SNIOC_SDRANGE, (unsigned long)(uintptr_t)&range);
  if (ret)
  {
    Serial.print("ERROR: Set dynamic range failed. ");
    Serial.println(errno);
    return;
  }
  ret = ioctl(fd, SNIOC_SFIFOTHRESH, IMU_FIFO);
  if (ret)
  {
    Serial.print("ERROR: Set FIFO failed. ");
    Serial.println(errno);
    return;
  }
  ret = ioctl(fd, SNIOC_ENABLE, 1);
  if (ret)
  {
    Serial.print("ERROR: Enable failed. ");
    Serial.println(errno);
    return;
  }

  last_time = millis();
  Serial.println("Start Self-Position Estimation.");
}

// --------------------------------------------------------
// loop(): センサデータ取得，自己位置推定，結果出力
// --------------------------------------------------------
void loop()
{
  static int execute_counter = 0;
  struct pollfd pfd;
  pfd.fd = fd;
  pfd.events = POLLIN;
  int pret = poll(&pfd, 1, 1); // ポーリングのタイムアウトを1msに短縮
  if (pret > 0)
  {
    cxd5602pwbimu_data_t data;
    int r = read(fd, &data, sizeof(data));
    if (r == sizeof(data))
    {
      if (update_state(data))
      {
        execute_counter++;
        if (execute_counter >= MESUREMENT_FREQUENCY / 30)
        {
          // データをリアルタイムで送信
          Serial.printf("%.6f,%.6f,%.6f,%.6f," // クォータニオン
                        "%.6f,%.6f,%.6f,"      // 加速度 [m/s^2]
                        "%.6f,%.6f,%.6f,"      // 角速度 [rad/s]
                        "%.6f,%.6f,%.6f,"      // 位置 [mm]
                        "%.6f,%.6f,%.6f\n",    // 速度 [mm/s]
                        quaternion[0], quaternion[1], quaternion[2], quaternion[3],
                        data.ax, data.ay, data.az,
                        data.gx, data.gy, data.gz,
                        position[0], position[1], position[2],
                        velocity[0], velocity[1], velocity[2]);
          execute_counter = 0;
        }
      }
    }
  }
}
