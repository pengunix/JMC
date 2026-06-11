use crate::motor::motor::MotorState;
use ros2_client::{Node, MessageTypeName, Name};
use serde::ser::{Serialize, SerializeStruct, Serializer};
use std::time::Duration;
use tokio::sync::watch;

struct MotorStateMsg {
    joint_names: Vec<String>,
    pos: Vec<f64>,
    vel: Vec<f64>,
    tau: Vec<f64>,
}

impl Serialize for MotorStateMsg {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        let mut s = serializer.serialize_struct("MotorState", 4)?;
        s.serialize_field("joint_names", &self.joint_names)?;
        s.serialize_field("pos", &self.pos)?;
        s.serialize_field("vel", &self.vel)?;
        s.serialize_field("tau", &self.tau)?;
        s.end()
    }
}

/// (direction, zero_offset) per motor
pub type Transform = (f32, f32);

pub async fn publisher_task(
    node: &mut Node,
    joint_names: Vec<String>,
    states: Vec<watch::Receiver<MotorState>>,
    transforms: Vec<Transform>,
) {
    let topic = node
        .create_topic(
            &Name::new("/", "dm_states").unwrap(),
            MessageTypeName::new("dm_msgs", "State"),
            &ros2_client::DEFAULT_SUBSCRIPTION_QOS,
        )
        .unwrap();

    let publisher = node.create_publisher(&topic, None).unwrap();

    let mut ticker = tokio::time::interval(Duration::from_millis(1));
    loop {
        ticker.tick().await;
        let mut pos = Vec::with_capacity(states.len());
        let mut vel = Vec::with_capacity(states.len());
        let mut tau = Vec::with_capacity(states.len());

        for (rx, (direction, zero_offset)) in states.iter().zip(transforms.iter()) {
            let raw = rx.borrow();
            pos.push((raw.q * direction + zero_offset) as f64);
            vel.push((raw.dq * direction) as f64);
            tau.push((raw.tau * direction) as f64);
        }

        let state = MotorStateMsg {
            joint_names: joint_names.clone(),
            pos,
            vel,
            tau,
        };
        let _ = publisher.async_publish(&state).await;
    }
}
