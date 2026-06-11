use socketcan::Result;

mod motor;
mod leg;
mod ros2;

async fn test_main() -> anyhow::Result<()> {
    let system = leg::yaml_loader::load_and_init("conf/motors.yaml").await?;

    // Enable all motors
    for handle in &system.cmd_handles {
        let mut m = handle.motor.lock().await;
        let _ = m.enable().await;
        let _ = m.set_run_mode().await;
    }
    println!("All motors enabled");

    // Build transform pairs for publisher: (direction, zero_offset)
    let transforms: Vec<_> = system
        .state_transforms
        .iter()
        .map(|t| (t.direction, t.zero_offset))
        .collect();

    let state_rxs: Vec<_> = system
        .state_transforms
        .into_iter()
        .map(|t| t.state_rx)
        .collect();

    let (pub_task, sub_task) = ros2::ros2::spawn_task(
        system.joint_names,
        state_rxs,
        transforms,
        system.cmd_handles,
    )
;

    let _ = tokio::try_join!(pub_task, sub_task);
    Ok(())
}

fn main() -> Result<()> {
    let rt = tokio::runtime::Builder::new_multi_thread()
        .enable_all()
        .build()
        .unwrap();
    rt.block_on(async {
        if let Err(e) = test_main().await {
            eprintln!("Error: {}", e);
        }
    });
    Ok(())
}
