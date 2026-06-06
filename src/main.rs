use console_subscriber::ConsoleLayer;
use futures_timer::Delay;
use futures_util::{SinkExt, StreamExt};
use socketcan::{
    CanFrame, EmbeddedFrame, Id, Result, SocketOptions, id::ERR_MASK_ALL, tokio::CanSocket,
};
use std::env;
use std::f32::consts::PI;
use std::sync::Arc;
use tokio::sync::{Mutex, mpsc, watch};
use tokio::time::{Duration, Instant, interval};
mod motor;
use crate::motor::motor::{MotorControl, MotorState};
use motor::motor::Motor;
use motor::motor_param::MotorModel;
use motor::motor_rs::MotorRS;

async fn test_main() {
    let iface = env::args().nth(1).unwrap_or_else(|| "can0".into());
    let socketcan = CanSocket::open(&iface).unwrap();
    socketcan.set_error_filter(ERR_MASK_ALL).unwrap();
    let (mut sink, mut stream) = socketcan.split();

    let (frame_tx, mut frame_rx) = mpsc::channel::<CanFrame>(64);
    let write_task = tokio::spawn(async move {
        while let Some(frame) = frame_rx.recv().await {
            // println!("id {:02X?}, data {:02X?}", frame.id(), frame.data());
            if let Err(e) = sink.send(frame).await {
                eprintln!("Error sending CAN frame: {}", e);
            }
        }
    });

    let (motor_1_tx, motor_1_rx) = watch::channel::<MotorState>(MotorState {
        q: 0.0,
        dq: 0.0,
        tau: 0.0,
        temp: 0.0,
        error: 0,
    });
    let mut motor_1 = Arc::new(Mutex::new(MotorRS {
        motor: Motor::new(
            MotorModel::RS06,
            0x01,
            0x0B,
            true,
            frame_tx.clone(),
            motor_1_rx,
        ),
    }));
    let (motor_2_tx, motor_2_rx) = watch::channel::<MotorState>(MotorState {
        q: 0.0,
        dq: 0.0,
        tau: 0.0,
        temp: 0.0,
        error: 0,
    });
    let mut motor_2 = Arc::new(Mutex::new(MotorRS {
        motor: Motor::new(
            MotorModel::RS06,
            0x02,
            0x0C,
            true,
            frame_tx.clone(),
            motor_2_rx,
        ),
    }));
    let (motor_3_tx, motor_3_rx) = watch::channel::<MotorState>(MotorState {
        q: 0.0,
        dq: 0.0,
        tau: 0.0,
        temp: 0.0,
        error: 0,
    });
    let mut motor_3 = Arc::new(Mutex::new(MotorRS {
        motor: Motor::new(
            MotorModel::RS03,
            0x03,
            0x0D,
            true,
            frame_tx.clone(),
            motor_3_rx,
        ),
    }));
    let (motor_4_tx, motor_4_rx) = watch::channel::<MotorState>(MotorState {
        q: 0.0,
        dq: 0.0,
        tau: 0.0,
        temp: 0.0,
        error: 0,
    });
    let mut motor_4 = Arc::new(Mutex::new(MotorRS {
        motor: Motor::new(
            MotorModel::RS02,
            0x04,
            0x0E,
            true,
            frame_tx.clone(),
            motor_4_rx,
        ),
    }));

    let motor_1_read = motor_1.clone();
    let motor_2_read = motor_2.clone();
    let motor_3_read = motor_3.clone();
    let motor_4_read = motor_4.clone();
    let read_task = tokio::spawn(async move {
        while let Some(Ok(frame)) = stream.next().await {
            match frame.id() {
                Id::Standard(id) => println!(
                    "Received CAN frame: ID={:?}, Data={:02X?}",
                    id.as_raw(),
                    frame.data()
                ),
                Id::Extended(id) => {
                    {
                        let mut motor_1r = motor_1_read.lock().await;
                        if id.as_raw() & 0xFF == motor_1r.motor.master_id {
                            motor_1_tx.send(motor_1r.parse_frame(&frame).unwrap());
                        }
                    }
                    {
                        let mut motor_2r = motor_2_read.lock().await;
                        if id.as_raw() & 0xFF == motor_2r.motor.master_id {
                            motor_2_tx.send(motor_2r.parse_frame(&frame).unwrap());
                        }
                    }
                    {
                        let mut motor_3r = motor_3_read.lock().await;
                        if id.as_raw() & 0xFF == motor_3r.motor.master_id {
                            motor_3_tx.send(motor_3r.parse_frame(&frame).unwrap());
                        }
                    }
                    {
                        let mut motor_4r = motor_4_read.lock().await;
                        if id.as_raw() & 0xFF == motor_4r.motor.master_id {
                            motor_4_tx.send(motor_4r.parse_frame(&frame).unwrap());
                        }
                    }
                }
            }
        }
    });

    let motor_1_main = motor_1.clone();
    let motor_2_main = motor_2.clone();
    let motor_3_main = motor_3.clone();
    let motor_4_main = motor_4.clone();
    let main_task = tokio::spawn(async move {
        let amplitude: f32 = 5.0; // 振幅
        let frequency: f32 = 0.1; // Hz，每秒0.1个周期

        let control_period = Duration::from_millis(1); // 100Hz 控制频率
        let mut ticker = interval(control_period);
        let start = Instant::now();
        let mut init_pos: f32 = 0.0;

        {
            let mut motor_1w = motor_1_main.lock().await;
            motor_1w.enable().await;
            motor_1w.set_run_mode().await;

            let mut motor_2w = motor_2_main.lock().await;
            motor_2w.enable().await;
            motor_2w.set_run_mode().await;

            let mut motor_3w = motor_3_main.lock().await;
            motor_3w.enable().await;
            motor_3w.set_run_mode().await;

            let mut motor_4w = motor_4_main.lock().await;
            motor_4w.enable().await;
            motor_4w.set_run_mode().await;
        }
        Delay::new(Duration::from_secs(2)).await;

        loop {
            ticker.tick().await;
            let elapsed = start.elapsed().as_secs_f32();
            let pos = amplitude * (2.0 * PI * frequency * elapsed).sin() + init_pos;
            {
                let mut motor_1w = motor_1_main.lock().await;
                let _ = motor_1w.control_mit(1.0, 0.1, pos, 0.0, 0.0).await;
                print!("pos1: {:.2} ", motor_1w.motor.subscribe_state().borrow().q)
            }
            {
                let mut motor_2w = motor_2_main.lock().await;
                let _ = motor_2w.control_mit(1.0, 0.1, pos, 0.0, 0.0).await;
                print!("pos2: {:.2} ", motor_2w.motor.subscribe_state().borrow().q)
            }
            {
                let mut motor_3w = motor_3_main.lock().await;
                let _ = motor_3w.control_mit(1.0, 0.1, pos, 0.0, 0.0).await;
                print!("pos3: {:.2} ", motor_3w.motor.subscribe_state().borrow().q)
            }
            {
                let mut motor_4w = motor_4_main.lock().await;
                let _ = motor_4w.control_mit(1.0, 0.1, pos, 0.0, 0.0).await;
                println!("pos2: {:.2} ", motor_4w.motor.subscribe_state().borrow().q)
            }

        }
    });

    tokio::try_join!(write_task, read_task, main_task);
}

fn main() -> Result<()> {
    console_subscriber::ConsoleLayer::builder()
        .retention(std::time::Duration::from_secs(60))
        .server_addr(([127, 0, 0, 1], 9090))
        .init();
    let rt = tokio::runtime::Builder::new_multi_thread()
        .enable_all()
        .build()
        .unwrap();
    rt.block_on(test_main());
    Ok(())
}
