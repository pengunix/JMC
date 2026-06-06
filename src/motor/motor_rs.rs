use super::motor::Motor;
use super::motor::MotorControl;
use embedded_can::Frame;
use socketcan::{CanFrame, ExtendedId, Id, StandardId};

/// 29位ID段： bit 21~16
#[repr(u32)]
enum MotorError {
    NotCalibrated = (1 << 21),
    OverLoad = (1 << 20),
    EncoderFault = (1 << 19),
    OverTemp = (1 << 18),
    PhaseCurrentFault = (1 << 17),
}
/// 29位ID段：22~23
#[repr(u8)]
enum MotorMode {
    Reset = 0,
    Cali = 1,
    Motor = 2,
}

#[repr(u32)]
enum RunMode {
    MoveControl = 0,
    PPMode = 1,
    VelMode = 2,
    TauMode = 3,
    SetZeroMode = 4,
    CSPMode = 5,
}
pub struct MotorRS {
    pub motor: Motor,
}

impl MotorControl for MotorRS {
    fn parse_frame(&self, frame: &CanFrame) -> Option<super::motor::MotorState> {
        let data = frame.data();
        let extend_id = match frame.id() {
            Id::Standard(id) => ExtendedId::new(id.as_raw() as u32).unwrap(),
            Id::Extended(id) => id,
        };
        if data.len() < 8 {
            return None;
        }
        let q_int = ((data[0] as u16) << 8) | (data[1] as u16);
        let dq_int = ((data[2] as u16) << 8) | (data[3] as u16);
        let tau_int = ((data[4] as u16) << 8) | (data[5] as u16);
        let temp_int = ((data[6] as u16) << 8) | (data[7] as u16);

        Some(super::motor::MotorState {
            q: self.int_to_float(q_int, -self.motor.limits.q_max, self.motor.limits.q_max, 16),
            dq: self.int_to_float(
                dq_int,
                -self.motor.limits.dq_max,
                self.motor.limits.dq_max,
                16,
            ),
            tau: self.int_to_float(
                tau_int,
                -self.motor.limits.tau_max,
                self.motor.limits.tau_max,
                16,
            ),
            temp: temp_int as f32 / 10.0,
            error: extend_id.as_raw() & 0x3FFFFF, // 错误信息在ID的低22位
        })
    }

    async fn enable(&mut self) -> Result<(), String> {
        let id = if self.motor.use_extended_id {
            Id::Extended(
                ExtendedId::new(self.motor.slave_id | (self.motor.master_id << 8) | (0x3 << 24))
                    .unwrap(),
            )
        } else {
            Id::Standard(StandardId::new(self.motor.slave_id as u16).unwrap())
        };

        let frame = CanFrame::new(id, &[0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0]).unwrap();
        self.motor
            .can_tx
            .send(frame)
            .await
            .map_err(|e| e.to_string())
    }

    async fn disable(&mut self) -> Result<(), String> {
        let id = if self.motor.use_extended_id {
            Id::Extended(
                ExtendedId::new(self.motor.slave_id | (self.motor.master_id << 8) | (0x4 << 24))
                    .unwrap(),
            )
        } else {
            Id::Standard(StandardId::new(self.motor.slave_id as u16).unwrap())
        };

        let frame = CanFrame::new(id, &[0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0]).unwrap();
        self.motor
            .can_tx
            .send(frame)
            .await
            .map_err(|e| e.to_string())
    }

    async fn set_param(&mut self, reg: u16, value: u32) -> Result<(), String>{
        let id = if self.motor.use_extended_id {
            Id::Extended(
                ExtendedId::new(self.motor.slave_id | (self.motor.master_id << 8) | (0x12 << 24))
                    .unwrap(),
            )
        } else {
            Id::Standard(StandardId::new(self.motor.slave_id as u16).unwrap())
        };

        let frame = CanFrame::new(
            id,
            &[
                (reg & 0xFF) as u8,
                ((reg >> 8) & 0xFF) as u8,
                0x0,
                0x0,
                (value & 0xFF) as u8,
                ((value >> 8) & 0xFF) as u8,
                ((value >> 16) & 0xFF) as u8,
                ((value >> 24) & 0xFF) as u8,
            ],
        ).unwrap();
        self.motor
            .can_tx
            .send(frame)
            .await
            .map_err(|e| e.to_string())
    }

    async fn set_run_mode(&mut self) -> Result<(), String> {
        self.set_param(0x7005, RunMode::MoveControl as u32).await
    }

    async fn control_mit(
        &mut self,
        kp: f32,
        kd: f32,
        q: f32,
        qd: f32,
        tau: f32,
    ) -> Result<(), String> {
        let kp_int = self.float_to_int(kp, 0.0, self.motor.limits.kp_max, 16);
        let kd_int = self.float_to_int(kd, 0.0, self.motor.limits.kd_max, 16);
        let q_int = self.float_to_int(q, -self.motor.limits.q_max, self.motor.limits.q_max, 16);
        let dq_int = self.float_to_int(qd, -self.motor.limits.dq_max, self.motor.limits.dq_max, 16);
        let tau_int = self.float_to_int(
            tau,
            -self.motor.limits.tau_max,
            self.motor.limits.tau_max,
            16,
        );

        let id = if self.motor.use_extended_id {
            Id::Extended(
                ExtendedId::new(self.motor.slave_id | ((tau_int as u32) << 8) | (0x1 << 24))
                    .unwrap(),
            )
        } else {
            Id::Standard(StandardId::new(self.motor.slave_id as u16).unwrap())
        };

        let frame = CanFrame::new(
            id,
            &[
                ((q_int >> 8) & 0xFF) as u8,
                (q_int & 0xFF) as u8,
                ((dq_int >> 8) & 0xFF) as u8,
                (dq_int & 0xFF) as u8,
                ((kp_int >> 8) & 0xFF) as u8,
                (kp_int & 0xFF) as u8,
                ((kd_int >> 8) & 0xFF) as u8,
                (kd_int & 0xFF) as u8,
            ],
        )
        .unwrap();
        self.motor
            .can_tx
            .send(frame)
            .await
            .map_err(|e| e.to_string())
    }
}
