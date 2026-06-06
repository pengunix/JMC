use socketcan::{CanFrame, CanSocket, Id, StandardId, ExtendedId};
use embedded_can::{ErrorKind::Bit, Frame};
use tokio::sync::{mpsc, watch};
use super::motor_param::{Limits, MotorModel};

pub struct MotorState {
    pub q: f32,
    pub dq: f32,
    pub tau: f32,
    pub temp: f32,
    pub error: u32, // define in MotorError
}

pub struct Motor {
    pub model: MotorModel,
    pub limits: Limits,
    pub slave_id: u32,
    pub master_id: u32,
    pub use_extended_id: bool,

    pub(super) can_tx: mpsc::Sender<CanFrame>,
    pub(super) state_rx: watch::Receiver<MotorState>,
}
pub trait MotorControl {
    /// float 与 int 互转
    fn float_to_int(&self, value: f32, min: f32, max: f32, bits: u8) -> u16 {
        let clamped = value.clamp(min, max);
        let scale = (1 << bits) - 1;
        ((clamped - min) / (max - min) * scale as f32).round() as u16
    }
    /// float 与 int 互转
    fn int_to_float(&self, value: u16, min: f32, max: f32, bits: u8) -> f32 {
        let scale = (1 << bits) - 1;
        (value as f32 / scale as f32) * (max - min) + min
    }
    /// 解析CAN帧数据为电机状态
    fn parse_frame(&self, frame: &CanFrame) -> Option<MotorState>;
    /// 使能电机
    async fn enable(&mut self) -> Result<(), String>;

    /// 失能电机
    async fn disable(&mut self) -> Result<(), String>;

    /// 设置参数
    async fn set_param(&mut self, reg: u16, value: u32) -> Result<(), String>;

    /// 设置电机模式
    async fn set_run_mode(&mut self) -> Result<(), String>;

    /// Mit控制
    async fn control_mit(
        &mut self,
        kp: f32,
        kd: f32,
        q: f32,
        qd: f32,
        tau: f32,
    ) -> Result<(), String>;
}

impl Motor {
    pub fn new(
        model: MotorModel,
        slave_id: u32,
        master_id: u32,
        use_extended_id: bool,
        can_tx: mpsc::Sender<CanFrame>,
        state_rx: watch::Receiver<MotorState>,
    ) -> Self {
        let limits = model.limits();
        Motor {
            model,
            limits: limits,
            slave_id,
            master_id,
            use_extended_id,
            can_tx,
            state_rx,
        }
    }

    pub fn subscribe_state(&self) -> watch::Receiver<MotorState> {
        self.state_rx.clone()
    }


}
