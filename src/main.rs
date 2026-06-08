use futures_timer::Delay;
use socketcan::Result;
use std::env;
use std::f32::consts::PI;
use tokio::time::{Duration, Instant, interval};

mod motor;
mod leg;
use crate::motor::motor::MotorControl;
use leg::leg_control::LegControl;

async fn test_main() {
    let iface = env::args().nth(1).unwrap_or_else(|| "can0".into());
    let leg = LegControl::new(&iface).unwrap();

    let m1 = leg.motor(0).clone();
    let m2 = leg.motor(1).clone();
    let m3 = leg.motor(2).clone();
    let m4 = leg.motor(3).clone();

    let (write_task, read_task) = leg.spawn_tasks();

    let main_task = tokio::spawn(async move {
        let amplitude: f32 = 5.0;
        let frequency: f32 = 0.1;

        let control_period = Duration::from_millis(1);
        let mut ticker = interval(control_period);
        let start = Instant::now();
        let init_pos: f32 = 0.0;

        {
            println!("Enabling motors...");
            let mut m1w = m1.lock().await;
            let _ = m1w.enable().await;
            let _ = m1w.set_run_mode().await;

            let mut m2w = m2.lock().await;
            let _ = m2w.enable().await;
            let _ = m2w.set_run_mode().await;

            let mut m3w = m3.lock().await;
            let _ = m3w.enable().await;
            let _ = m3w.set_run_mode().await;

            let mut m4w = m4.lock().await;
            let _ = m4w.enable().await;
            let _ = m4w.set_run_mode().await;
        }
        Delay::new(Duration::from_secs(2)).await;

        loop {
            ticker.tick().await;
            let elapsed = start.elapsed().as_secs_f32();
            let pos = amplitude * (2.0 * PI * frequency * elapsed).sin() + init_pos;
            {
                let mut m1w = m1.lock().await;
                let _ = m1w.control_mit(1.0, 0.1, pos, 0.0, 0.0).await;
                print!("pos1: {:.2} ", m1w.subscribe_state().borrow().q)
            }
            {
                let mut m2w = m2.lock().await;
                let _ = m2w.control_mit(1.0, 0.1, pos, 0.0, 0.0).await;
                print!("pos2: {:.2} ", m2w.subscribe_state().borrow().q)
            }
            {
                let mut m3w = m3.lock().await;
                let _ = m3w.control_mit(1.0, 0.1, pos, 0.0, 0.0).await;
                print!("pos3: {:.2} ", m3w.subscribe_state().borrow().q)
            }
            {
                let mut m4w = m4.lock().await;
                let _ = m4w.control_mit(1.0, 0.1, pos, 0.0, 0.0).await;
                println!("pos4: {:.2} ", m4w.subscribe_state().borrow().q)
            }

            Delay::new(Duration::from_millis(1)).await;
        }
    });

    let _ = tokio::try_join!(write_task, read_task, main_task);
}

fn main() -> Result<()> {
    let rt = tokio::runtime::Builder::new_current_thread()
        .enable_all()
        .build()
        .unwrap();
    rt.block_on(test_main());
    Ok(())
}
