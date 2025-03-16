// Spresense Multi-IMU Sample for Arduino IDE

#include <stdio.h>
#include <sys/ioctl.h>
#include <time.h>
#include <inttypes.h>
#include <fcntl.h>
#include <unistd.h>
#include <signal.h>
#include <poll.h>
#include <errno.h>
#include <math.h>

// PWBIMU用ヘッダファイル
#include <nuttx/sensors/cxd5602pwbimu.h>

extern "C" int board_cxd5602pwbimu_initialize(int);

//----------------------------------------
// IMU設定定数
//----------------------------------------
#define IMU_RATE 60     // サンプリングレート: 60Hz
#define IMU_ADRANGE 4   // 加速度レンジ: 4G
#define IMU_GDRANGE 500 // ジャイロレンジ: 500dps
#define IMU_FIFO 1      // FIFO設定値

static int fd; // センサデバイスファイルディスクリプタ

// グローバル変数：姿勢推定用のオイラー角（単位: degree）および時間管理
static unsigned long last_time = 0;
static float roll = 0.0, pitch = 0.0, yaw = 0.0;

//----------------------------------------
// setup() 関数: センサの初期化と設定、初期時間の設定
//----------------------------------------
void setup()
{
  int ret = 0;

  // センサ初期化
  ret = board_cxd5602pwbimu_initialize(5);
  if (ret < 0)
  {
    printf("ERROR: Failed to initialize CXD5602PWBIMU.\n");
    return;
  }

  // IMUデバイス "/dev/imu0" を読み取り専用でオープン
  fd = open("/dev/imu0", O_RDONLY);
  if (fd < 0)
  {
    printf("ERROR: Could not open /dev/imu0\n");
    return;
  }

  // サンプリングレートの設定
  ret = ioctl(fd, SNIOC_SSAMPRATE, IMU_RATE);
  if (ret)
  {
    printf("ERROR: Set sampling rate failed. %d\n", errno);
    return;
  }

  // 加速度およびジャイロのレンジ設定
  cxd5602pwbimu_range_t range;
  range.accel = IMU_ADRANGE;
  range.gyro = IMU_GDRANGE;
  ret = ioctl(fd, SNIOC_SDRANGE, (unsigned long)(uintptr_t)&range);
  if (ret)
  {
    printf("ERROR: Set dynamic range failed. %d\n", errno);
    return;
  }

  // FIFOの設定
  ret = ioctl(fd, SNIOC_SFIFOTHRESH, IMU_FIFO);
  if (ret)
  {
    printf("ERROR: Set FIFO failed. %d\n", errno);
    return;
  }

  // センサ出力の有効化
  ret = ioctl(fd, SNIOC_ENABLE, 1);
  if (ret)
  {
    printf("ERROR: Enable failed. %d\n", errno);
    return;
  }

  // ※ 必要に応じて、バッファのゴミデータ除去処理を追加可能

  // CSVフォーマットのヘッダ（センサ値）を出力
  // 例: "ax,ay,az,gx,gy,gz"
  printf("ax,ay,az,gx,gy,gz\n");

  // 姿勢推定用ヘッダ出力
  // 例: "roll,pitch,yaw"
  printf("roll,pitch,yaw\n");

  // 初期時間の記録（Arduino の millis() を仮定）
  last_time = millis();
}

//----------------------------------------
// loop() 関数: センサデータの取得、姿勢推定、および出力
//----------------------------------------
void loop()
{
  struct pollfd fds[1];
  fds[0].fd = fd;
  fds[0].events = POLLIN;

  // 最大1000ミリ秒（1秒）間待機
  int ret = poll(fds, 1, 1000);
  if (ret > 0)
  {
    cxd5602pwbimu_data_t data;
    ret = read(fd, &data, sizeof(data));
    if (ret == sizeof(data))
    {
      // センサデータをCSV形式で出力（加速度とジャイロ）
      printf("%f,%f,%f,%f,%f,%f\n",
             data.ax, data.ay, data.az,
             data.gx, data.gy, data.gz);

      // 時間差(dt)の計算（秒単位）
      unsigned long now = millis();
      float dt = (now - last_time) / 1000.0;
      last_time = now;

      // ジャイロは degree/sec で出力されるので、そのまま積分すると roll, pitch, yaw は degree になるはずですが、
      // もしラジアンで扱われている場合、出力時に (180/π) を掛けて度に変換します。
      roll += data.gx * dt;
      pitch += data.gy * dt;
      yaw += data.gz * dt;
      // yaw を 0〜360度に正規化（roll, pitch はそのまま出力）
      float yaw_deg = fmod((yaw * (180.0 / M_PI)) + 360, 360);

      // 推定した姿勢（roll, pitch, yaw）を degree に変換して表示
      printf("%f,%f,%f\n", roll * (180.0 / M_PI), pitch * (180.0 / M_PI), yaw_deg);
    }
  }
}

//----------------------------------------
// main() 関数: setup() と loop() の呼び出し
//----------------------------------------
int main(void)
{
  setup();
  while (1)
  {
    loop();
  }
  return 0;
}
