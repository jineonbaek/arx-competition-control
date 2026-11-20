#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};



// Corresponds to arx_chassis_interfaces__msg__Command

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Command {

    // This member is not documented.
    #[allow(missing_docs)]
    pub request_id: std::string::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub scheduler_instance: std::string::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub client_id: std::string::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub operation: std::string::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub kind: std::string::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub source_id: std::string::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub task_id: std::string::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub generation: u64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub seq: u64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub issued_at: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub ttl_s: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub sdk: [f64; 3],

}



impl Default for Command {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::Command::default())
  }
}

impl rosidl_runtime_rs::Message for Command {
  type RmwMsg = super::msg::rmw::Command;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        request_id: msg.request_id.as_str().into(),
        scheduler_instance: msg.scheduler_instance.as_str().into(),
        client_id: msg.client_id.as_str().into(),
        operation: msg.operation.as_str().into(),
        kind: msg.kind.as_str().into(),
        source_id: msg.source_id.as_str().into(),
        task_id: msg.task_id.as_str().into(),
        generation: msg.generation,
        seq: msg.seq,
        issued_at: msg.issued_at,
        ttl_s: msg.ttl_s,
        sdk: msg.sdk,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        request_id: msg.request_id.as_str().into(),
        scheduler_instance: msg.scheduler_instance.as_str().into(),
        client_id: msg.client_id.as_str().into(),
        operation: msg.operation.as_str().into(),
        kind: msg.kind.as_str().into(),
        source_id: msg.source_id.as_str().into(),
        task_id: msg.task_id.as_str().into(),
      generation: msg.generation,
      seq: msg.seq,
      issued_at: msg.issued_at,
      ttl_s: msg.ttl_s,
        sdk: msg.sdk,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      request_id: msg.request_id.to_string(),
      scheduler_instance: msg.scheduler_instance.to_string(),
      client_id: msg.client_id.to_string(),
      operation: msg.operation.to_string(),
      kind: msg.kind.to_string(),
      source_id: msg.source_id.to_string(),
      task_id: msg.task_id.to_string(),
      generation: msg.generation,
      seq: msg.seq,
      issued_at: msg.issued_at,
      ttl_s: msg.ttl_s,
      sdk: msg.sdk,
    }
  }
}


// Corresponds to arx_chassis_interfaces__msg__State

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct State {

    // This member is not documented.
    #[allow(missing_docs)]
    pub scheduler_instance: std::string::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub ready: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub driver_instance: std::string::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub active: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub client_id: std::string::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub source_id: std::string::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub task_id: std::string::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub generation: u64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub output_seq: u64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub expires_at: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub sdk: [f64; 3],


    // This member is not documented.
    #[allow(missing_docs)]
    pub maintenance: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub applied_sdk: [f64; 3],


    // This member is not documented.
    #[allow(missing_docs)]
    pub applied_seq: u64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub reason: std::string::String,

}



impl Default for State {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::State::default())
  }
}

impl rosidl_runtime_rs::Message for State {
  type RmwMsg = super::msg::rmw::State;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        scheduler_instance: msg.scheduler_instance.as_str().into(),
        ready: msg.ready,
        driver_instance: msg.driver_instance.as_str().into(),
        active: msg.active,
        client_id: msg.client_id.as_str().into(),
        source_id: msg.source_id.as_str().into(),
        task_id: msg.task_id.as_str().into(),
        generation: msg.generation,
        output_seq: msg.output_seq,
        expires_at: msg.expires_at,
        sdk: msg.sdk,
        maintenance: msg.maintenance,
        applied_sdk: msg.applied_sdk,
        applied_seq: msg.applied_seq,
        reason: msg.reason.as_str().into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        scheduler_instance: msg.scheduler_instance.as_str().into(),
      ready: msg.ready,
        driver_instance: msg.driver_instance.as_str().into(),
      active: msg.active,
        client_id: msg.client_id.as_str().into(),
        source_id: msg.source_id.as_str().into(),
        task_id: msg.task_id.as_str().into(),
      generation: msg.generation,
      output_seq: msg.output_seq,
      expires_at: msg.expires_at,
        sdk: msg.sdk,
      maintenance: msg.maintenance,
        applied_sdk: msg.applied_sdk,
      applied_seq: msg.applied_seq,
        reason: msg.reason.as_str().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      scheduler_instance: msg.scheduler_instance.to_string(),
      ready: msg.ready,
      driver_instance: msg.driver_instance.to_string(),
      active: msg.active,
      client_id: msg.client_id.to_string(),
      source_id: msg.source_id.to_string(),
      task_id: msg.task_id.to_string(),
      generation: msg.generation,
      output_seq: msg.output_seq,
      expires_at: msg.expires_at,
      sdk: msg.sdk,
      maintenance: msg.maintenance,
      applied_sdk: msg.applied_sdk,
      applied_seq: msg.applied_seq,
      reason: msg.reason.to_string(),
    }
  }
}


// Corresponds to arx_chassis_interfaces__msg__Result

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Result {

    // This member is not documented.
    #[allow(missing_docs)]
    pub request_id: std::string::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub accepted: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub reason: std::string::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub output_seq: u64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub state: super::msg::State,

}



impl Default for Result {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::Result::default())
  }
}

impl rosidl_runtime_rs::Message for Result {
  type RmwMsg = super::msg::rmw::Result;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        request_id: msg.request_id.as_str().into(),
        accepted: msg.accepted,
        reason: msg.reason.as_str().into(),
        output_seq: msg.output_seq,
        state: super::msg::State::into_rmw_message(std::borrow::Cow::Owned(msg.state)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        request_id: msg.request_id.as_str().into(),
      accepted: msg.accepted,
        reason: msg.reason.as_str().into(),
      output_seq: msg.output_seq,
        state: super::msg::State::into_rmw_message(std::borrow::Cow::Borrowed(&msg.state)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      request_id: msg.request_id.to_string(),
      accepted: msg.accepted,
      reason: msg.reason.to_string(),
      output_seq: msg.output_seq,
      state: super::msg::State::from_rmw_message(msg.state),
    }
  }
}


// Corresponds to arx_chassis_interfaces__msg__Output

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Output {

    // This member is not documented.
    #[allow(missing_docs)]
    pub scheduler_instance: std::string::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub driver_instance: std::string::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub output_seq: u64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub issued_at: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub ttl_s: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub sdk: [f64; 3],


    // This member is not documented.
    #[allow(missing_docs)]
    pub reason: std::string::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub source_id: std::string::String,

}



impl Default for Output {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::Output::default())
  }
}

impl rosidl_runtime_rs::Message for Output {
  type RmwMsg = super::msg::rmw::Output;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        scheduler_instance: msg.scheduler_instance.as_str().into(),
        driver_instance: msg.driver_instance.as_str().into(),
        output_seq: msg.output_seq,
        issued_at: msg.issued_at,
        ttl_s: msg.ttl_s,
        sdk: msg.sdk,
        reason: msg.reason.as_str().into(),
        source_id: msg.source_id.as_str().into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        scheduler_instance: msg.scheduler_instance.as_str().into(),
        driver_instance: msg.driver_instance.as_str().into(),
      output_seq: msg.output_seq,
      issued_at: msg.issued_at,
      ttl_s: msg.ttl_s,
        sdk: msg.sdk,
        reason: msg.reason.as_str().into(),
        source_id: msg.source_id.as_str().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      scheduler_instance: msg.scheduler_instance.to_string(),
      driver_instance: msg.driver_instance.to_string(),
      output_seq: msg.output_seq,
      issued_at: msg.issued_at,
      ttl_s: msg.ttl_s,
      sdk: msg.sdk,
      reason: msg.reason.to_string(),
      source_id: msg.source_id.to_string(),
    }
  }
}


