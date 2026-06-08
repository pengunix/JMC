use async_trait::async_trait;
use socketcan::CanFrame;
use tokio::sync::{mpsc, watch};
use super::motor_param::{Limits, MotorModel};

pub struct MotorState {
    pub q: f32,
    pub dq: f32,
    pub tau: f32,
    pub temp: f32,
    pub error: u32,
}

pub struct Motor {
    pub joint_name: String,
    pub model: MotorModel,
    pub limits: Limits,
    pub slave_id: u32,
    pub master_id: u32,
    pub use_extended_id: bool,

    pub(super) can_tx: mpsc::Sender<CanFrame>,
    pub(super) state_tx: watch::Sender<MotorState>,
    pub(super) state_rx: watch::Receiver<MotorState>,
}

#[async_trait]
pub trait MotorControl: Send {
    fn slave_id(&self) -> u32;
    fn master_id(&self) -> u32;
    fn use_extended_id(&self) -> bool;
    fn model(&self) -> &MotorModel;
    fn limits(&self) -> &Limits;
    fn subscribe_state(&self) -> watch::Receiver<MotorState>;
    fn state_tx(&self) -> &watch::Sender<MotorState>;

    fn push_state(&self, state: MotorState) {
        let _ = self.state_tx().send(state);
    }

    fn float_to_int(&self, value: f32, min: f32, max: f32, bits: u8) -> u16 {
        let clamped = value.clamp(min, max);
        let scale = (1 << bits) - 1;
        ((clamped - min) / (max - min) * scale as f32).round() as u16
    }

    fn int_to_float(&self, value: u16, min: f32, max: f32, bits: u8) -> f32 {
        let scale = (1 << bits) - 1;
        (value as f32 / scale as f32) * (max - min) + min
    }

    fn parse_frame(&self, frame: &CanFrame) -> Option<MotorState>;
    async fn enable(&mut self) -> Result<(), String>;
    async fn disable(&mut self) -> Result<(), String>;
    async fn set_param(&mut self, reg: u16, value: u32) -> Result<(), String>;
    async fn set_run_mode(&mut self) -> Result<(), String>;
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
        joint_name:String,
        model: MotorModel,
        slave_id: u32,
        master_id: u32,
        use_extended_id: bool,
        can_tx: mpsc::Sender<CanFrame>,
    ) -> Self {
        let limits = model.limits();
        let (state_tx, state_rx) = watch::channel(MotorState {
            q: 0.0,
            dq: 0.0,
            tau: 0.0,
            temp: 0.0,
            error: 0,
        });
        Motor {
            joint_name,
            model,
            limits,
            slave_id,
            master_id,
            use_extended_id,
            can_tx,
            state_tx,
            state_rx,
        }
    }

    pub fn subscribe_state(&self) -> watch::Receiver<MotorState> {
        self.state_rx.clone()
    }
}
