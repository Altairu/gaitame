#include <Arduino.h>
#include <stdio.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <nuttx/sensors/cxd5602pwbimu.h>
#include <arch/board/cxd56_cxd5602pwbimu.h>
#include <unistd.h>
#include <math.h>

// ===== 定数 =====
#define CXD5602PWBIMU_DEVPATH   "/dev/imu0"
#define MESUREMENT_FREQUENCY    1920      // サンプリング周波数[Hz]
#define CALIBRATION_DURATION_MS 500       // キャリブレーション期間[ms]
#define MAX_NFIFO               1

// ===== グローバル変数 =====
int   devfd = -1;
float q[4]         = {1.0f, 0.0f, 0.0f, 0.0f};   // 姿勢クォータニオン
float gyro_bias[3] = {0.0f, 0.0f, 0.0f};         // ジャイロバイアス
float measured_gravity = 9.80665f;               // 実測重力加速度
float vel[3] = {0.0f, 0.0f, 0.0f};               // 推定速度 (ワールド座標)
float pos[3] = {0.0f, 0.0f, 0.0f};               // 推定位置 (ワールド座標)
int   old_timestamp = -1;

// ===== ヘルパー関数 =====
// クォータニオン微分
void diff_quaternion(const float q[4], const float omega[3], float dqdt[4]) {
  float w=q[0], x=q[1], y=q[2], z=q[3];
  float ox=omega[0], oy=omega[1], oz=omega[2];
  dqdt[0] = 0.5f * (-x*ox - y*oy - z*oz);
  dqdt[1] = 0.5f * ( w*ox + y*oz - z*oy);
  dqdt[2] = 0.5f * ( w*oy - x*oz + z*ox);
  dqdt[3] = 0.5f * ( w*oz + x*oy - y*ox);
}

// RK4によるクォータニオン更新
void runge_kutta_update(const float q[4], const float omega[3],
                        float dt, float q_next[4]) {
  float k1[4], k2[4], k3[4], k4[4], tmp[4];
  diff_quaternion(q, omega, k1);
  for(int i=0;i<4;i++) tmp[i] = q[i] + dt*k1[i]/2.0f;
  diff_quaternion(tmp, omega, k2);
  for(int i=0;i<4;i++) tmp[i] = q[i] + dt*k2[i]/2.0f;
  diff_quaternion(tmp, omega, k3);
  for(int i=0;i<4;i++) tmp[i] = q[i] + dt*k3[i];
  diff_quaternion(tmp, omega, k4);

  float norm = 0.0f;
  for(int i=0;i<4;i++){
    q_next[i] = q[i] + dt*(k1[i] + 2*k2[i] + 2*k3[i] + k4[i]) / 6.0f;
    norm += q_next[i]*q_next[i];
  }
  norm = sqrtf(norm);
  for(int i=0;i<4;i++) q_next[i] /= norm;
}

// ベクトル回転 (q ⊗ v ⊗ q*)
void apply_rotation(const float q[4], const float v[3], float out[3]) {
  float w=q[0], x=q[1], y=q[2], z=q[3];
  float vx=v[0], vy=v[1], vz=v[2];
  out[0] = (1 - 2*y*y - 2*z*z)*vx + (2*x*y - 2*w*z)*vy + (2*x*z + 2*w*y)*vz;
  out[1] = (2*x*y + 2*w*z)*vx + (1 - 2*x*x - 2*z*z)*vy + (2*y*z - 2*w*x)*vz;
  out[2] = (2*x*z - 2*w*y)*vx + (2*y*z + 2*w*x)*vy + (1 - 2*x*x - 2*y*y)*vz;
}

// ===== IMU 初期設定 & FIFOクリア =====
int start_sensing(int fd, int rate, int adrange, int gyrange, int nfifo) {
  cxd5602pwbimu_range_t range;
  ioctl(fd, SNIOC_SSAMPRATE, rate);
  range.accel = adrange;
  range.gyro  = gyrange;
  ioctl(fd, SNIOC_SDRANGE, (unsigned long)(uintptr_t)&range);
  ioctl(fd, SNIOC_SFIFOTHRESH, nfifo);
  ioctl(fd, SNIOC_ENABLE, 1);
  return 0;
}

int drop_50msdata(int fd, int samprate) {
  int cnt = samprate / 20;
  cxd5602pwbimu_data_t buf;
  while(cnt--) {
    read(fd, &buf, sizeof(buf));
  }
  return 0;
}

// ===== 初期キャリブレーション =====
void calibrate_initial() {
  int N = MESUREMENT_FREQUENCY * CALIBRATION_DURATION_MS / 1000;
  double axs=0, ays=0, azs=0;
  double gxs=0, gys=0, gzs=0;
  cxd5602pwbimu_data_t d;
  for(int i=0; i<N; i++){
    read(devfd, &d, sizeof(d));
    axs += d.ax;  ays += d.ay;  azs += d.az;
    gxs += d.gx;  gys += d.gy;  gzs += d.gz;
  }
  // ジャイロバイアス算出
  gyro_bias[0] = gxs / N;
  gyro_bias[1] = gys / N;
  gyro_bias[2] = gzs / N;
  // 重力大きさ算出
  float axm = axs/N, aym = ays/N, azm = azs/N;
  measured_gravity = sqrtf(axm*axm + aym*aym + azm*azm);
  // 初期クォータニオン（roll,pitchのみ）
  float roll  = atan2f(aym, azm);
  float pitch = atan2f(-axm, sqrtf(aym*aym + azm*azm));
  float cy=1, sy=0;  // yaw=0
  float cr=cosf(roll/2), sr=sinf(roll/2);
  float cp=cosf(pitch/2), sp=sinf(pitch/2);
  q[0] = cr*cp*cy + sr*sp*sy;
  q[1] = sr*cp*cy - cr*sp*sy;
  q[2] = cr*sp*cy + sr*cp*sy;
  q[3] = cr*cp*sy - sr*sp*cy;

  old_timestamp = d.timestamp;
}

// ===== Arduino setup =====
void setup() {
  Serial.begin(115200);
  board_cxd5602pwbimu_initialize(5);

  devfd = open(CXD5602PWBIMU_DEVPATH, O_RDONLY);
  if (devfd < 0) {
    Serial.println("IMU open fail");
    while (1);
  }

  start_sensing(devfd, MESUREMENT_FREQUENCY, 8, 2000, MAX_NFIFO);
  drop_50msdata(devfd, MESUREMENT_FREQUENCY);
  calibrate_initial();
}

// ===== Arduino loop =====
void loop() {
  static int cnt = 0;
  cxd5602pwbimu_data_t d;
  if (read(devfd, &d, sizeof(d)) == sizeof(d)) {
    // Δt計算
    float dt = (old_timestamp < 0)
      ? 1.0f / MESUREMENT_FREQUENCY
      : (d.timestamp - old_timestamp) / 19200000.0f;
    old_timestamp = d.timestamp;

    // ジャイロバイアス補正
    float omega[3] = {
      d.gx - gyro_bias[0],
      d.gy - gyro_bias[1],
      d.gz - gyro_bias[2]
    };

    // クォータニオン更新
    float qn[4];
    runge_kutta_update(q, omega, dt, qn);
    memcpy(q, qn, sizeof(q));

    // 位置推定：加速度→ワールド回転→重力引き→積分
    float acc_body[3]  = { d.ax, d.ay, d.az };
    float acc_world[3];
    apply_rotation(q, acc_body, acc_world);
    acc_world[2] -= measured_gravity;  // 重力成分除去
    for (int i=0; i<3; i++) {
      vel[i] += acc_world[i] * dt;
      pos[i] += vel[i]       * dt;
    }

    // 30Hzでシリアル出力
    if (++cnt >= MESUREMENT_FREQUENCY/30) {
      Serial.printf(
        "%08x,%08x,"              // timestamp,temp
        "%08x,%08x,%08x,"         // gx,gy,gz
        "%08x,%08x,%08x,"         // ax,ay,az
        "%08x,%08x,%08x,%08x,"    // qw,qx,qy,qz
        "%08x,%08x,%08x,"         // vx,vy,vz
        "%08x,%08x,%08x\n",       // px,py,pz
        (unsigned int)d.timestamp,
        *(unsigned int*)&d.temp,
        *(unsigned int*)&omega[0],
        *(unsigned int*)&omega[1],
        *(unsigned int*)&omega[2],
        *(unsigned int*)&d.ax,
        *(unsigned int*)&d.ay,
        *(unsigned int*)&d.az,
        *(unsigned int*)&q[0],
        *(unsigned int*)&q[1],
        *(unsigned int*)&q[2],
        *(unsigned int*)&q[3],
        *(unsigned int*)&vel[0],
        *(unsigned int*)&vel[1],
        *(unsigned int*)&vel[2],
        *(unsigned int*)&pos[0],
        *(unsigned int*)&pos[1],
        *(unsigned int*)&pos[2]
      );
      cnt = 0;
    }
  }
}
