//! The ROS side of the blackboard: a node that serves the global parameters read-only through
//! the standard parameter services and publishes them latched, together with the field maps.
//!
//! The node runs in its own thread, so neither waiting for the middleware nor spinning blocks
//! the orchestrator. It is only created after [`RosBlackboard::start`], because the middleware
//! router may be one of the components the orchestrator starts.

use std::{
    collections::HashMap,
    sync::{mpsc, Arc, Mutex},
    time::Duration,
};

use futures::{executor::LocalPool, task::LocalSpawnExt};
use indexmap::IndexMap;
use r2r::{
    bitbots_msgs::msg::ParameterBlackboard, nav_msgs::msg::OccupancyGrid, rcl_interfaces,
    Parameter, ParameterValue, QosProfile, RosParams,
};

use crate::{
    api::{GlobalValue, RosState},
    blackboard::{Blackboard, FieldMap},
};

/// How long to wait before retrying a failed node setup.
const SETUP_RETRY: Duration = Duration::from_secs(5);
const SPIN_TIMEOUT: Duration = Duration::from_millis(50);

/// Late subscribers receive the last message.
const LATCHED: QosProfile = QosProfile::default()
    .reliable()
    .transient_local()
    .keep_last(1);

enum Command {
    Start,
    Publish(Arc<Blackboard>),
}

/// Handle of the blackboard node thread.
pub struct RosBlackboard {
    tx: mpsc::Sender<Command>,
    state: Arc<Mutex<RosState>>,
    started: bool,
}

impl RosBlackboard {
    /// Spawn the thread; the node is created on [`RosBlackboard::start`]. `log` receives
    /// messages about the node for the orchestrator's log.
    pub fn spawn(node_name: String, topic: String, log: impl Fn(&str) + Send + 'static) -> Self {
        let (tx, rx) = mpsc::channel();
        let state = Arc::new(Mutex::new(RosState::WaitingFor(Vec::new())));
        let thread_state = state.clone();
        std::thread::Builder::new()
            .name("blackboard".into())
            .spawn(move || run(rx, thread_state, &node_name, &topic, &log))
            .expect("cannot spawn the blackboard thread");
        Self {
            tx,
            state,
            started: false,
        }
    }

    pub fn state(&self) -> RosState {
        self.state.lock().unwrap().clone()
    }

    /// Report what the node still waits for before it is started.
    pub fn set_waiting_for(&self, components: Vec<String>) {
        if !self.started {
            *self.state.lock().unwrap() = RosState::WaitingFor(components);
        }
    }

    /// Create the node; later calls have no effect.
    pub fn start(&mut self) {
        if !self.started {
            self.started = true;
            let _ = self.tx.send(Command::Start);
        }
    }

    /// Publish this state once the node runs, replacing what was published before.
    pub fn publish(&self, blackboard: Arc<Blackboard>) {
        let _ = self.tx.send(Command::Publish(blackboard));
    }
}

fn run(
    rx: mpsc::Receiver<Command>,
    state: Arc<Mutex<RosState>>,
    node_name: &str,
    topic: &str,
    log: &dyn Fn(&str),
) {
    let mut latest: Option<Arc<Blackboard>> = None;
    loop {
        match rx.recv() {
            Ok(Command::Start) => break,
            Ok(Command::Publish(b)) => latest = Some(b),
            Err(_) => return,
        }
    }

    let mut node = loop {
        *state.lock().unwrap() = RosState::Starting;
        match BlackboardNode::new(node_name, topic) {
            Ok(node) => break node,
            Err(e) => {
                let msg = format!("{e:#}");
                log(&format!(
                    "setting up the ROS node failed, retrying in {SETUP_RETRY:?}: {msg}"
                ));
                *state.lock().unwrap() = RosState::Failed(msg);
                std::thread::sleep(SETUP_RETRY);
                for cmd in rx.try_iter() {
                    if let Command::Publish(b) = cmd {
                        latest = Some(b);
                    }
                }
            }
        }
    };
    *state.lock().unwrap() = RosState::Running;
    log(&format!("ROS node {node_name} is running"));

    let mut published: Option<Arc<Blackboard>> = None;
    loop {
        node.spin();
        loop {
            match rx.try_recv() {
                Ok(Command::Publish(b)) => latest = Some(b),
                Ok(Command::Start) => {}
                Err(mpsc::TryRecvError::Empty) => break,
                Err(mpsc::TryRecvError::Disconnected) => return,
            }
        }
        let Some(current) = &latest else {
            continue;
        };
        if published.as_ref().is_some_and(|p| Arc::ptr_eq(p, current)) {
            continue;
        }
        match node.publish(current) {
            Ok(()) => log(&format!(
                "published {} global parameters and {} field maps",
                current.parameters.len(),
                current.maps.len()
            )),
            Err(e) => log(&format!("publishing failed: {e:#}")),
        }
        published = Some(current.clone());
    }
}

/// Serves the global parameters through the parameter services and rejects changes, since
/// the orchestrator owns them.
struct ReadOnlyParams {
    values: Arc<Mutex<IndexMap<String, ParameterValue>>>,
}

impl RosParams for ReadOnlyParams {
    fn register_parameters(
        &mut self,
        _prefix: &str,
        _param: Option<Parameter>,
        params: &mut IndexMap<String, Parameter>,
    ) -> r2r::Result<()> {
        for (name, value) in self.values.lock().unwrap().iter() {
            params.insert(name.clone(), Parameter::new(value.clone()));
        }
        Ok(())
    }

    fn get_parameter(&mut self, param_name: &str) -> r2r::Result<ParameterValue> {
        self.values
            .lock()
            .unwrap()
            .get(param_name)
            .cloned()
            .ok_or_else(|| r2r::Error::InvalidParameterName {
                name: param_name.to_string(),
            })
    }

    fn set_parameter(&mut self, param_name: &str, param_val: &ParameterValue) -> r2r::Result<()> {
        self.check_parameter(param_name, param_val)
    }

    fn check_parameter(&self, param_name: &str, _param_val: &ParameterValue) -> r2r::Result<()> {
        Err(r2r::Error::ParameterValueConv {
            name: param_name.to_string(),
            msg: "the global parameters are read-only, override them through the \
                  bitbots_orchestrate API"
                .to_string(),
        })
    }
}

struct BlackboardNode {
    node: r2r::Node,
    pool: LocalPool,
    clock: r2r::Clock,
    state_publisher: r2r::Publisher<ParameterBlackboard>,
    map_publishers: HashMap<String, r2r::Publisher<OccupancyGrid>>,
    values: Arc<Mutex<IndexMap<String, ParameterValue>>>,
}

impl BlackboardNode {
    fn new(node_name: &str, topic: &str) -> anyhow::Result<Self> {
        let ctx = r2r::Context::create()?;
        let mut node = r2r::Node::create(ctx, node_name, "")?;
        let values = Arc::new(Mutex::new(IndexMap::new()));
        let params = Arc::new(Mutex::new(ReadOnlyParams {
            values: values.clone(),
        }));
        // No events are emitted, as all changes are rejected.
        let (handler, _events) = node.make_derived_parameter_handler(params)?;
        let pool = LocalPool::new();
        pool.spawner().spawn_local(handler)?;
        let state_publisher = node.create_publisher::<ParameterBlackboard>(topic, LATCHED)?;
        Ok(Self {
            node,
            pool,
            clock: r2r::Clock::create(r2r::ClockType::SystemTime)?,
            state_publisher,
            map_publishers: HashMap::new(),
            values,
        })
    }

    fn spin(&mut self) {
        self.node.spin_once(SPIN_TIMEOUT);
        self.pool.run_until_stalled();
    }

    fn publish(&mut self, blackboard: &Blackboard) -> anyhow::Result<()> {
        let values: IndexMap<String, ParameterValue> = blackboard
            .parameters
            .iter()
            .map(|(name, (value, _))| (name.clone(), to_ros(value)))
            .collect();
        {
            // The list service reads the node's parameters, the get service our values.
            let mut params = self.node.params.lock().unwrap();
            params.clear();
            for (name, value) in &values {
                params.insert(name.clone(), Parameter::new(value.clone()));
            }
        }
        *self.values.lock().unwrap() = values;

        let stamp = r2r::Clock::to_builtin_time(&self.clock.get_now()?);
        self.state_publisher.publish(&ParameterBlackboard {
            stamp: stamp.clone(),
            parameters: blackboard
                .parameters
                .iter()
                .map(|(name, (value, _))| rcl_interfaces::msg::Parameter {
                    name: name.clone(),
                    value: to_msg(value),
                })
                .collect(),
        })?;

        for map in &blackboard.maps {
            if !self.map_publishers.contains_key(&map.topic) {
                let publisher = self.node.create_publisher(&map.topic, LATCHED)?;
                self.map_publishers.insert(map.topic.clone(), publisher);
            }
            self.map_publishers[&map.topic].publish(&occupancy_grid(map, stamp.clone()))?;
        }
        Ok(())
    }
}

fn occupancy_grid(map: &FieldMap, stamp: r2r::builtin_interfaces::msg::Time) -> OccupancyGrid {
    let mut grid = OccupancyGrid::default();
    grid.header.frame_id = map.frame_id.clone();
    grid.header.stamp = stamp.clone();
    grid.info.map_load_time = stamp;
    grid.info.resolution = map.resolution as f32;
    grid.info.width = map.width;
    grid.info.height = map.height;
    grid.info.origin.position.x = -(map.width as f64) / 2.0 * map.resolution;
    grid.info.origin.position.y = -(map.height as f64) / 2.0 * map.resolution;
    grid.info.origin.orientation.w = 1.0;
    grid.data = map.data.clone();
    grid
}

fn to_ros(value: &GlobalValue) -> ParameterValue {
    match value.clone() {
        GlobalValue::Bool(v) => ParameterValue::Bool(v),
        GlobalValue::Integer(v) => ParameterValue::Integer(v),
        GlobalValue::Double(v) => ParameterValue::Double(v),
        GlobalValue::String(v) => ParameterValue::String(v),
        GlobalValue::BoolArray(v) => ParameterValue::BoolArray(v),
        GlobalValue::IntegerArray(v) => ParameterValue::IntegerArray(v),
        GlobalValue::DoubleArray(v) => ParameterValue::DoubleArray(v),
        GlobalValue::StringArray(v) => ParameterValue::StringArray(v),
    }
}

/// The message form of a value, with the type ids of rcl_interfaces/ParameterType.
fn to_msg(value: &GlobalValue) -> rcl_interfaces::msg::ParameterValue {
    use rcl_interfaces::msg::ParameterType as T;
    let mut msg = rcl_interfaces::msg::ParameterValue::default();
    match value.clone() {
        GlobalValue::Bool(v) => {
            msg.type_ = T::PARAMETER_BOOL as u8;
            msg.bool_value = v;
        }
        GlobalValue::Integer(v) => {
            msg.type_ = T::PARAMETER_INTEGER as u8;
            msg.integer_value = v;
        }
        GlobalValue::Double(v) => {
            msg.type_ = T::PARAMETER_DOUBLE as u8;
            msg.double_value = v;
        }
        GlobalValue::String(v) => {
            msg.type_ = T::PARAMETER_STRING as u8;
            msg.string_value = v;
        }
        GlobalValue::BoolArray(v) => {
            msg.type_ = T::PARAMETER_BOOL_ARRAY as u8;
            msg.bool_array_value = v;
        }
        GlobalValue::IntegerArray(v) => {
            msg.type_ = T::PARAMETER_INTEGER_ARRAY as u8;
            msg.integer_array_value = v;
        }
        GlobalValue::DoubleArray(v) => {
            msg.type_ = T::PARAMETER_DOUBLE_ARRAY as u8;
            msg.double_array_value = v;
        }
        GlobalValue::StringArray(v) => {
            msg.type_ = T::PARAMETER_STRING_ARRAY as u8;
            msg.string_array_value = v;
        }
    }
    msg
}
