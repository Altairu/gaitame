
<!-- Noto Sans フォント読み込み＆MathJax v3（amsパッケージ有効化） -->
<link href="https://fonts.googleapis.com/css2?family=Noto+Sans&display=swap" rel="stylesheet">
<style>
  body {
    font-family: 'Noto Sans', sans-serif;
  }
</style>

<script>
  window.MathJax = {
    tex: {
      inlineMath: [['$', '$'], ['\\(', '\\)']],    /* インライン：$…$、\(...\) */
      displayMath: [['$$', '$$'], ['\\[', '\\]']], /* 表示式：$$…$$、\[...\] */
      packages: {'[+]': ['ams']}                   /* amsmath, amsfonts 等を追加 */
    },
    svg: {
      fontCache: 'global'
    }
  };
</script>
<script async src="https://cdn.jsdelivr.net/npm/mathjax@3/es5/tex-svg.js"></script>


## 1. ジャイロバイアス補正

```cpp
// 生のジャイロ値からキャリブレーションで求めたバイアスを引く
float omega[3] = {
  d.gx - gyro_bias[0],
  d.gy - gyro_bias[1],
  d.gz - gyro_bias[2]
};
```

対応数式：

$$
\omega_x = g_x - b_{g,x},\quad
\omega_y = g_y - b_{g,y},\quad
\omega_z = g_z - b_{g,z}.
$$


## 2. クォータニオンの微分

```cpp
void diff_quaternion(const float q[4], const float omega[3], float dqdt[4]) {
  float w=q[0], x=q[1], y=q[2], z=q[3];
  float ox=omega[0], oy=omega[1], oz=omega[2];
  dqdt[0] = 0.5f * (-x*ox - y*oy - z*oz);
  dqdt[1] = 0.5f * ( w*ox + y*oz - z*oy);
  dqdt[2] = 0.5f * ( w*oy - x*oz + z*ox);
  dqdt[3] = 0.5f * ( w*oz + x*oy - y*ox);
}
```

対応微分方程式：

$$
\dot q = \tfrac12
\begin{bmatrix}
-\,q_1 & -\,q_2 & -\,q_3\\
\;q_0 & -\,q_3 & \;q_2\\
\;q_3 & \;q_0 & -\,q_1\\
-\,q_2 & \;q_1 & \;q_0
\end{bmatrix}
\begin{bmatrix}\omega_x\\\omega_y\\\omega_z\end{bmatrix}.
$$


## 3. RK4 によるクォータニオン更新

```cpp
void runge_kutta_update(const float q[4], const float omega[3],
                        float dt, float q_next[4]) {
  float k1[4], k2[4], k3[4], k4[4], tmp[4];

  diff_quaternion(q,       omega, k1);
  for(int i=0;i<4;i++) tmp[i] = q[i] + dt*k1[i]/2.0f;
  diff_quaternion(tmp,     omega, k2);
  for(int i=0;i<4;i++) tmp[i] = q[i] + dt*k2[i]/2.0f;
  diff_quaternion(tmp,     omega, k3);
  for(int i=0;i<4;i++) tmp[i] = q[i] + dt*k3[i];
  diff_quaternion(tmp,     omega, k4);

  float norm = 0.0f;
  for(int i=0;i<4;i++){
    q_next[i] = q[i] + dt*(k1[i] + 2*k2[i] + 2*k3[i] + k4[i]) / 6.0f;
    norm    += q_next[i]*q_next[i];
  }
  norm = sqrtf(norm);
  for(int i=0;i<4;i++) q_next[i] /= norm;
}
```

対応数式：

$$
\begin{aligned}
k_1 &= f(q_n,\omega),\\
k_2 &= f\bigl(q_n + \tfrac{\Delta t}{2}k_1,\omega\bigr),\\
k_3 &= f\bigl(q_n + \tfrac{\Delta t}{2}k_2,\omega\bigr),\\
k_4 &= f\bigl(q_n + \Delta t\,k_3,\omega\bigr),\\
\tilde q_{n+1}
&= q_n + \frac{\Delta t}{6}\,(k_1 + 2k_2 + 2k_3 + k_4),\\
q_{n+1}
&= \dfrac{\tilde q_{n+1}}{\|\tilde q_{n+1}\|}.
\end{aligned}
$$


## 4. ボディ→ワールド座標変換

```cpp
void apply_rotation(const float q[4], const float v[3], float out[3]) {
  float w=q[0], x=q[1], y=q[2], z=q[3];
  float vx=v[0], vy=v[1], vz=v[2];
  out[0] = (1 - 2*y*y - 2*z*z)*vx + (2*x*y - 2*w*z)*vy + (2*x*z + 2*w*y)*vz;
  out[1] = (2*x*y + 2*w*z)*vx + (1 - 2*x*x - 2*z*z)*vy + (2*y*z - 2*w*x)*vz;
  out[2] = (2*x*z - 2*w*y)*vx + (2*y*z + 2*w*x)*vy + (1 - 2*x*x - 2*y*y)*vz;
}
```

回転行列 \(R(q)\)：

$$
R(q)=
\begin{bmatrix}
1-2(y^2+z^2) & 2(xy - wz) & 2(xz + wy)\\
2(xy + wz) & 1-2(x^2+z^2) & 2(yz - wx)\\
2(xz - wy) & 2(yz + wx) & 1-2(x^2+y^2)
\end{bmatrix},
\quad
\mathbf{a}_{world} = R(q)\,\mathbf{a}_{body}.
$$


## 5. 重力成分の除去

```cpp
// acc_world[2] から measured_gravity を引く
acc_world[2] -= measured_gravity;
```

$$
\mathbf{a}'_{world}
=
\begin{bmatrix}
a_x \\ a_y \\ a_z - g
\end{bmatrix},\quad
g = \text{measured\_gravity}.
$$


## 6. 速度・位置の積分（オイラー法）

```cpp
for (int i=0; i<3; i++) {
  vel[i] += acc_world[i] * dt;  // 速度更新
  pos[i] += vel[i]       * dt;  // 位置更新
}
```

$$
\begin{aligned}
\mathbf{v}_{n+1} &= \mathbf{v}_n + \mathbf{a}'_{world}\,\Delta t,\\
\mathbf{p}_{n+1} &= \mathbf{p}_n + \mathbf{v}_n\,\Delta t.
\end{aligned}
$$
