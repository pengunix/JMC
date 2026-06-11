use ros2_client::{Context, NodeName, NodeOptions};
use tokio::sync::watch;
use crate::motor::motor::MotorState;
use super::publisher::{self, Transform};
use super::subscriber;
use crate::leg::yaml_loader::MotorCmdHandle;

pub fn spawn_task(
    joint_names: Vec<String>,
    state_rxs: Vec<watch::Receiver<MotorState>>,
    transforms: Vec<Transform>,
    cmd_handles: Vec<MotorCmdHandle>,
) -> (tokio::task::JoinHandle<()>, tokio::task::JoinHandle<()>) {
    let pub_ctx = Context::new().unwrap();
    let mut pub_node = pub_ctx
        .new_node(
            NodeName::new("/rustdds", "rustdds_publisher").unwrap(),
            NodeOptions::new().enable_rosout(true),
        )
        .unwrap();

    let sub_ctx = Context::new().unwrap();
    let mut sub_node = sub_ctx
        .new_node(
            NodeName::new("/rustdds", "rustdds_subscriber").unwrap(),
            NodeOptions::new().enable_rosout(true),
        )
        .unwrap();

    let pub_task = tokio::spawn(async move {
        publisher::publisher_task(&mut pub_node, joint_names, state_rxs, transforms).await
    });
    let sub_task = tokio::spawn(async move {
        subscriber::subscriber_task(&mut sub_node, cmd_handles).await
    });

    (pub_task, sub_task)
}
