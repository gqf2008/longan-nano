//!超声波测距传感器

use embedded_hal::blocking::delay::DelayUs;
use embedded_hal::digital::v2::{InputPin, OutputPin};

#[derive(Debug, Copy, Clone)]
pub struct Distance(f64);

impl Distance {
    pub fn cm(&self) -> f64 {
        self.0 / 10.0
    }
    pub fn mm(&self) -> f64 {
        self.0
    }
}

pub struct HcSr04<Triger, Echo, Delay> {
    trig: Triger,
    echo: Echo,
    delay: Delay,
    timer: MonoTimer,
}

impl<Trig, Echo, Delay> HcSr04<Trig, Echo, Delay>
where
    Trig: OutputPin,
    Echo: InputPin,
    Delay: DelayUs<u32>,
{
    pub fn new(mut trig: Trig, echo: Echo, delay: Delay, core_frequency: u64) -> Self {
        trig.set_low().ok();
        HcSr04 {
            trig,
            echo,
            delay,
            timer: MonoTimer {
                frequency: core_frequency,
            },
        }
    }

    pub fn measure(&mut self) -> Distance {
        self.delay.delay_us(50000u32);
        let sum = self.measure1();
        Distance(sum)
    }

    fn measure1(&mut self) -> f64 {
        //发送信号
        self.trig.set_high().ok();
        self.delay.delay_us(20u32);
        self.trig.set_low().ok();
        // let t0 = riscv::register::mcycle::read64();
        // let clocks = (us * (self.core_frequency as u64)) / 1_000_000;
        // while riscv::register::mcycle::read64().wrapping_sub(t0) <= clocks {}
        //let start_wait = self.timer.now();
        //等高电平
        while let Ok(true) = self.echo.is_low() {
            // if start_wait.elapsed() > self.timer.frequency() * 5 {
            //     return Err(Error::Timeout);
            // }
        }
        //等低电平（高电平持续的时间就是信号往返的时间）
        let start_instant = self.timer.now();
        //crate::sprintln!("start {}", start_instant.now);
        while let Ok(true) = self.echo.is_high() {
            // if start_instant.elapsed() > self.timer.frequency().0 {
            //     return Err(Error::Timeout);
            // }
        }
        let ticks = start_instant.elapsed() as f64;
        // crate::sprintln!("elapsed {}", ticks);
        ticks / self.timer.frequency() as f64 * 170.0 * 1000.0
    }
}

#[derive(Clone, Copy)]
pub struct MonoTimer {
    frequency: u64,
}

impl MonoTimer {
    /// Creates a new `Monotonic` timer
    pub fn new(frequency: u64) -> Self {
        MonoTimer { frequency }
    }

    /// Returns the frequency at which the monotonic timer is operating at
    pub fn frequency(self) -> u64 {
        self.frequency
    }

    /// Returns an `Instant` corresponding to "now"
    pub fn now(self) -> Instant {
        Instant {
            now: riscv::register::mcycle::read64(),
        }
    }
}

/// A measurement of a monotonically non-decreasing clock
#[derive(Clone, Copy)]
pub struct Instant {
    now: u64,
}

impl Instant {
    /// Ticks elapsed since the `Instant` was created
    pub fn elapsed(self) -> u64 {
        riscv::register::mcycle::read64().wrapping_sub(self.now)
    }
}

//标量卡尔曼滤波器
#[derive(Debug, Default)]
struct KalmanScalar {
    x: f64,    // 系统的状态量
    A: f64,    // x(n)=A*x(n-1)+u(n),u(n)~N(0,q)
    H: f64,    // z(n)=H*x(n)+w(n),w(n)~N(0,r)
    q: f64,    // 预测过程噪声协方差
    r: f64,    // 测量过程噪声协方差
    p: f64,    // 估计误差协方差
    gain: f64, //卡尔曼增益
}

impl KalmanScalar {
    /**
     *@x：待测量的初始值
     *@p：后验状态估计值误差的方差的初始值
     */
    fn new(x: f64, p: f64, predict_q: f64, new_measured_q: f64) -> Self {
        Self {
            x: x,              //待测量的初始值，如有中值一般设成中值
            p: p,              //后验状态估计值误差的方差的初始值（不要为0问题不大）
            q: predict_q,      //预测（过程）噪声方差 影响收敛速率，可以根据实际需求给出
            r: new_measured_q, //测量（观测）噪声方差 可以通过实验手段获得
            A: 1.0,
            H: 1.0,
            gain: 0.0,
        }
    }
    /**
     *@new_measured；测量值
     *返回滤波后的值
     */
    fn filter(&mut self, new_measured: f64) -> f64 {
        /* Predict */
        self.x = self.A * self.x; //%x的先验估计由上一个时间点的后验估计值和输入信息给出
        self.p = self.A * self.A * self.p + self.q; /*计算先验均方差 p(n|n-1)=A^2*p(n-1|n-1)+q */
        /* Correct */
        self.gain = self.p * self.H / (self.p * self.H * self.H + self.r);
        self.x = self.x + self.gain * (new_measured - self.H * self.x); //利用残余的信息改善对x(t)的估计，给出后验估计，这个值也就是输出
        self.p = (1.0 - self.gain * self.H) * self.p; //%计算后验均方差
        self.x
    }
}

// /**
//  * 	x = A*x + w; w~N(0,Q)
//  * 	y = C*x + v; v~N(0,R)
//  */
//  typedef struct scalar_kalman_s
//  {
//      float A, C;
//      float A2, C2; 	// A,C 的平方

//      float Q, R;
//      float K, P;

//      float x;
//  }scalar_kalman_t;

// /**
// * 	状态方程: x = A*x + w; w~N(0,Q)
// * 	测量方程: y = C*x + v; v~N(0,R)
//  */
//  float scalar_kalman(scalar_kalman_t *kalman, float y)
//  {
//      // 状态预测
//      kalman->x = kalman->A * kalman->x;
//      // 误差协方差预测
//      kalman->P = kalman->A2 * kalman->P + kalman->Q;
//      // 计算卡尔曼滤波增益
//      kalman->K = kalman->P * kalman->C / (kalman->C2 * kalman->P + kalman->R);

//      // 状态估计校正
//      kalman->x = kalman->x + kalman->K * (y - kalman->C * kalman->x);
//      // 误差协方差估计校正
//      kalman->P = (1 - kalman->K * kalman->C) * kalman->P;

//      return kalman->C * kalman->x; 	// 输出滤波后的y
//  }

//  void scalar_kalman_init(scalar_kalman_t *kalman, float A, float C, float Q, float R)
//  {
//      kalman->A = A;
//      kalman->A2 = A * A;
//      kalman->C = C;
//      kalman->C2 = C * C;

//      kalman->Q = Q;
//      kalman->R = R;

//      kalman->x = 0;
//      kalman->P = Q;
//      kalman->K = 1;
//  }

///////////////////////////////////////////////

// #include "stm32f10x.h"
// #include "filter.h"
// #include <stdio.h>

// static float angle,angle_dot;
// const  float Q_angle = 0.002, Q_gyro = 0.002, R_angle = 0.5, dt = 0.01;
// static float P[2][2]-{
//                       { 1 , 0 },
//                       { 0 , 1 }
//                      };
// static float Pdot[4] = { 0 , 0 , 0 , 0};
// const  u8    C_0 = 1;
// static float q_bias , angle_err , PCt_0 , PCt_1 , E , K_0 , K_1 , t_0 , t_1;

// /* float angle_m 加速度计计算角度
//    float gyro_m  陀螺仪角速度
//    float *angle_f 融合后的角度
//    float *angle_dot_f 融合后的角速度

//    Q_angle 加速度计 过程噪声协方差(仿真整定)
//    Q_gyro  陀螺仪   过程噪声协方差(仿真整定)
//    R_angle 加速度计 测量方程协方差(对测量设备进行数据测量后利用matlab进行协方差计算)
//    P    角度协方差矩阵
//    Pdot 角速度协方差矩阵
//    dt   微分因子
//    q_bias 陀螺仪偏移量
//    C0 测量矩阵
//    K0 K1 卡尔曼增益矩阵(2 * 1)元素
//    PCt_0 PCt_1 t_0 t_1 中间计算变量
//    angle_err 过程测量误差值
//    void kalman_fliter(float angle_m, float gyro_m, float *angle_f , *angle_dot_f)
// */
// void kalman_filter(float angle_m, float gyro_m, float *angle_f, float *angle_dot_f)
// {
// 	angle += (gyro_m - q_bias) * dt;

// 	Pdot[0] = Q_angle - P[0][1] - P[1][0];
// 	Pdot[1] = -P[1][1];
// 	Pdot[2] = -P[1][1];
// 	Pdot[3] = Q_gyro;

// 	P[0][0] += Pdot[0] * dt;
// 	P[0][1] += Pdot[1] * dt;
// 	P[1][0] += Pdot[2] * dt;
// 	P[1][1] += Pdot[3] * dt;

// 	angle_err = angle_m - angle;

// 	PCt_0 = C_0 * P[0][0];
// 	PCt_1 = C_0 * P[1][0];

// 	E = R_angle + C_0 * PCt_0;

// 	K_0 = PCt_0 / E;
// 	K_1 = PCt_1 / E;

// 	t_0 = PCt_0;
// 	t_1 = C_0 * P[0][1];

// 	P[0][0] -= K_0 * t_0;
// 	P[0][1] -= K_0 * t_1;
// 	P[1][0] -= K_1 * t_0;
// 	P[1][1] -= K_1 * t_1;

// 	angle  += K_0 * angle_err;
// 	q_bias += K_1 * angle_err;
// 	angle_dot = gyro_m - q_bias;

// 	*angle_f        = angle;
// 	*angle_dot_f = angle_dot;
// }
