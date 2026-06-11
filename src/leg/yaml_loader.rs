use anyhow::{Context, Result};
use serde::Deserialize;
use std::collections::HashMap;
use std::sync::Arc;
use tokio::sync::{Mutex, watch};

use super::leg_control::{LegControl, MotorSpec};
use crate::motor::motor::{MotorControl, MotorState};
use crate::motor::motor_param::MotorModel;

#[derive(Deserialize)]
struct GlobalConfig {
    zero_position: f32,
}

#[derive(Deserialize)]
struct MotorDef {
    motor_type: String,
    slave_id: u32,
    master_id: u32,
}

#[derive(Deserialize)]
struct LegDef {
    name: String,
    can_port: String,
    motors: Vec<String>,
    directions: Vec<i32>,
    zero_position_indices: usize,
    zero_position_sign: i32,
}

#[derive(Deserialize)]
struct YamlConfig {
    global: GlobalConfig,
    motors: HashMap<String, MotorDef>,
    legs: HashMap<String, LegDef>,
}

#[derive(Clone, Debug)]
pub struct MotorCommand {
    pub q: f64,
    pub dq: f64,
    pub kp: f64,
    pub kd: f64,
    pub tau: f64,
}

impl Default for MotorCommand {
    fn default() -> Self {
        MotorCommand {
            q: 0.0,
            dq: 0.0,
            kp: 0.0,
            kd: 0.0,
            tau: 0.0,
        }
    }
}

/// Per-motor transform config for publishing state
pub struct MotorStateTransform {
    pub state_rx: watch::Receiver<MotorState>,
    pub direction: f32,
    /// zero_offset to ADD after direction multiply (always -zero_position for zero_idx motor)
    pub zero_offset: f32,
    pub joint_name: String,
}

/// Per-motor handle for command dispatch
pub struct MotorCmdHandle {
    pub motor: Arc<Mutex<Box<dyn MotorControl + Send>>>,
    pub direction: f32,
    /// zero_offset for command: zero_position * zero_position_sign (for zero_idx motor)
    pub cmd_zero_offset: f32,
}

pub struct MotorSystem {
    pub state_transforms: Vec<MotorStateTransform>,
    pub cmd_handles: Vec<MotorCmdHandle>,
    pub joint_names: Vec<String>,
    /// CAN read/write task handles — kept alive to keep tasks running
    _leg_tasks: Vec<(tokio::task::JoinHandle<()>, tokio::task::JoinHandle<()>)>,
}

fn string_to_motor_type(s: &str) -> Result<MotorModel> {
    match s {
        "DM4310" => Ok(MotorModel::DM4310),
        "DM4310_48V" => Ok(MotorModel::DM4310_48V),
        "DM4340" => Ok(MotorModel::DM4340),
        "DM4340_48V" => Ok(MotorModel::DM4340_48V),
        "DM6006" => Ok(MotorModel::DM6006),
        "DM8006" => Ok(MotorModel::DM8006),
        "DM8009" => Ok(MotorModel::DM8009),
        "DM10010L" => Ok(MotorModel::DM10010L),
        "DM10010" => Ok(MotorModel::DM10010),
        "DMH3510" => Ok(MotorModel::DMH3510),
        "DMH6215" => Ok(MotorModel::DMH6215),
        "DMG6220" => Ok(MotorModel::DMG6220),
        "RS00" => Ok(MotorModel::RS00),
        "RS01" => Ok(MotorModel::RS01),
        "RS02" => Ok(MotorModel::RS02),
        "RS03" => Ok(MotorModel::RS03),
        "RS04" => Ok(MotorModel::RS04),
        "RS05" => Ok(MotorModel::RS05),
        "RS06" => Ok(MotorModel::RS06),
        _ => Err(anyhow::anyhow!("Unknown motor type: {}", s)),
    }
}

fn parse_config(config_path: &str) -> Result<YamlConfig> {
    let content =
        std::fs::read_to_string(config_path).context("Failed to read config file")?;
    let config: YamlConfig =
        serde_yaml::from_str(&content).context("Failed to parse YAML config")?;
    Ok(config)
}

pub async fn load_and_init(config_path: &str) -> Result<MotorSystem> {
    let config = parse_config(config_path)?;
    let zero_position = config.global.zero_position;

    let mut state_transforms = Vec::new();
    let mut cmd_handles = Vec::new();
    let mut joint_names = Vec::new();
    let mut leg_tasks = Vec::new();

    for (_leg_name, leg_def) in &config.legs {

        let motor_specs: Vec<MotorSpec> = leg_def
            .motors
            .iter()
            .map(|m_name| {
                let mdef = config
                    .motors
                    .get(m_name)
                    .with_context(|| format!("Motor '{}' not found", m_name))?;
                Ok(MotorSpec {
                    joint_name: m_name.clone(),
                    model: string_to_motor_type(&mdef.motor_type)?,
                    slave_id: mdef.slave_id,
                    master_id: mdef.master_id,
                })
            })
            .collect::<Result<Vec<_>>>()?;

        let leg = LegControl::new(&leg_def.can_port, motor_specs)?;

        // Clone motor Arcs and get raw state receivers before spawn_tasks consumes leg
        let motor_count = leg.motor_count();
        let motor_arcs: Vec<Arc<Mutex<Box<dyn MotorControl + Send>>>> =
            (0..motor_count).map(|i| leg.motor(i).clone()).collect();

        let raw_state_rxs: Vec<watch::Receiver<MotorState>> = {
            let mut rxs = Vec::new();
            for m in &motor_arcs {
                rxs.push(m.lock().await.subscribe_state());
            }
            rxs
        };

        // Spawn CAN read/write tasks (consumes LegControl)
        let tasks = leg.spawn_tasks();
        leg_tasks.push(tasks);

        // Build transforms and handles for each motor in this leg
        for i in 0..motor_count {
            let direction = leg_def.directions[i] as f32;
            let is_zero_idx = leg_def.zero_position_indices == i;

            // Publishing: pos = raw * direction - zero_position (for zero_idx)
            let zero_offset = if is_zero_idx { -zero_position } else { 0.0 };

            // Command: motor_cmd = ros_cmd * direction + zero_position * zero_sign
            let cmd_zero_offset = if is_zero_idx {
                zero_position * leg_def.zero_position_sign as f32
            } else {
                0.0
            };

            let joint_name = leg_def.motors[i].clone();

            state_transforms.push(MotorStateTransform {
                state_rx: raw_state_rxs[i].clone(),
                direction,
                zero_offset,
                joint_name: joint_name.clone(),
            });

            cmd_handles.push(MotorCmdHandle {
                motor: motor_arcs[i].clone(),
                direction,
                cmd_zero_offset,
            });

            joint_names.push(joint_name);
        }
    }

    Ok(MotorSystem {
        state_transforms,
        cmd_handles,
        joint_names,
        _leg_tasks: leg_tasks,
    })
}
