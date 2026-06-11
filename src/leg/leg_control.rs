use futures_util::{SinkExt, StreamExt};
use futures_util::stream::{SplitSink, SplitStream};
use socketcan::{CanFrame, EmbeddedFrame, Id, SocketOptions, id::ERR_MASK_ALL, tokio::CanSocket};
use std::sync::Arc;
use tokio::sync::{Mutex, mpsc};

use crate::motor::motor::{Motor, MotorControl};
use crate::motor::motor_param::MotorModel;
use crate::motor::motor_rs::MotorRS;

pub struct MotorSpec {
    pub joint_name: String,
    pub model: MotorModel,
    pub slave_id: u32,
    pub master_id: u32,
}

pub struct LegControl {
    motors: Vec<Arc<Mutex<Box<dyn MotorControl + Send>>>>,
    frame_tx: mpsc::Sender<CanFrame>,
    frame_rx: mpsc::Receiver<CanFrame>,
    sink: SplitSink<CanSocket, CanFrame>,
    stream: SplitStream<CanSocket>,
}

impl LegControl {
    pub fn new(iface: &str, motor_specs: Vec<MotorSpec>) -> std::io::Result<Self> {
        let can = CanSocket::open(iface)?;
        can.set_error_filter(ERR_MASK_ALL)
            .map_err(|e| std::io::Error::new(std::io::ErrorKind::Other, e.to_string()))?;
        let (sink, stream) = can.split();
        let (frame_tx, frame_rx) = mpsc::channel::<CanFrame>(64);

        let tx = frame_tx.clone();
        let motors: Vec<Arc<Mutex<Box<dyn MotorControl + Send>>>> = motor_specs
            .into_iter()
            .map(|spec| {
                Arc::new(Mutex::new(
                    Box::new(MotorRS {
                        motor: Motor::new(
                            spec.joint_name,
                            spec.model,
                            spec.slave_id,
                            spec.master_id,
                            true,
                            tx.clone(),
                        ),
                    }) as Box<dyn MotorControl + Send>,
                ))
            })
            .collect();

        Ok(LegControl {
            motors,
            frame_tx,
            frame_rx,
            sink,
            stream,
        })
    }

    pub fn motor(&self, index: usize) -> &Arc<Mutex<Box<dyn MotorControl + Send>>> {
        &self.motors[index]
    }

    pub fn motor_count(&self) -> usize {
        self.motors.len()
    }

    pub fn motors(&self) -> &[Arc<Mutex<Box<dyn MotorControl + Send>>>] {
        &self.motors
    }

    pub fn frame_tx(&self) -> mpsc::Sender<CanFrame> {
        self.frame_tx.clone()
    }

    pub async fn enable_all_motors(&self) -> Result<(), String> {
        for m_arc in self.motors.iter() {
            m_arc.lock().await.enable().await?;
        }
        Ok(())
    }

    pub fn spawn_tasks(self) -> (tokio::task::JoinHandle<()>, tokio::task::JoinHandle<()>) {
        let LegControl {
            motors,
            frame_tx: _frame_tx,
            frame_rx,
            sink,
            stream,
        } = self;

        let write_task = tokio::spawn(async move {
            let mut sink = sink;
            let mut frame_rx = frame_rx;
            while let Some(frame) = frame_rx.recv().await {
                if let Err(e) = sink.send(frame).await {
                    eprintln!("Error sending CAN frame: {}", e);
                }
            }
        });

        let read_task = tokio::spawn(async move {
            let mut stream = stream;
            while let Some(Ok(frame)) = stream.next().await {
                match frame.id() {
                    Id::Standard(id) => println!(
                        "Received CAN frame: ID={:?}, Data={:02X?}",
                        id.as_raw(),
                        frame.data()
                    ),
                    Id::Extended(id) => {
                        let frame_master = id.as_raw() & 0xFF;
                        // 一个腿上只有四个电机直接遍历
                        for motor in &motors {
                            let m = motor.lock().await;
                            if frame_master == m.master_id() as u32 {
                                if let Some(state) = m.parse_frame(&frame) {
                                    m.push_state(state);
                                }
                            }
                        }
                    }
                }
            }
        });

        (write_task, read_task)
    }
}
