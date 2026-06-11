use ros2_client::{Node, MessageTypeName, Name};
use std::sync::Arc;
use std::time::Duration;
use tokio::sync::Mutex;
use crate::leg::yaml_loader::MotorCmdHandle;

#[derive(Debug, Clone, serde::Deserialize)]
struct MotorCommandMsg {
    pos: Vec<f64>,
    vel: Vec<f64>,
    kp: Vec<f64>,
    kd: Vec<f64>,
    tau: Vec<f64>,
}

pub async fn subscriber_task(node: &mut Node, cmd_handles: Vec<MotorCmdHandle>) {
    let topic = node
        .create_topic(
            &Name::new("/", "dm_cmd").unwrap(),
            MessageTypeName::new("dm_msgs", "Command"),
            &ros2_client::DEFAULT_SUBSCRIPTION_QOS,
        )
        .unwrap();

    let subscriber = node
        .create_subscription::<MotorCommandMsg>(&topic, None)
        .unwrap();

    let spinner = node.spinner().unwrap();
    let _spawned_spinner = tokio::spawn(async move { spinner.spin().await });

    let last_cmd: Arc<Mutex<Option<MotorCommandMsg>>> = Arc::new(Mutex::new(None));

    // Background task: receive ROS messages into the shared buffer
    let rx_buf = last_cmd.clone();
    tokio::spawn(async move {
        loop {
            let (msg, _info) = subscriber.async_take().await.unwrap();
            *rx_buf.lock().await = Some(msg);
        }
    });

    // 100Hz control loop: send commands periodically so motors keep reporting state
    let mut ticker = tokio::time::interval(Duration::from_millis(1));
    loop {
        ticker.tick().await;
        let cmd = last_cmd.lock().await.clone();

        for (i, handle) in cmd_handles.iter().enumerate() {
            let (q_cmd, dq_cmd, tau_cmd, kp, kd) = match &cmd {
                Some(msg) if i < msg.pos.len() => (
                    msg.pos[i] * handle.direction as f64 + handle.cmd_zero_offset as f64,
                    msg.vel[i] * handle.direction as f64,
                    msg.tau[i] * handle.direction as f64,
                    msg.kp[i] as f32,
                    msg.kd[i] as f32,
                ),
                _ => (0.0, 0.0, 0.0, 0.0, 0.0),
            };

            let mut motor = handle.motor.lock().await;
            let _ = motor
                .control_mit(kp, kd, q_cmd as f32, dq_cmd as f32, tau_cmd as f32)
                .await;
        }
    }
}
