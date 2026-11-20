#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};


#[link(name = "arx_chassis_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__arx_chassis_interfaces__msg__Command() -> *const std::ffi::c_void;
}

#[link(name = "arx_chassis_interfaces__rosidl_generator_c")]
extern "C" {
    fn arx_chassis_interfaces__msg__Command__init(msg: *mut Command) -> bool;
    fn arx_chassis_interfaces__msg__Command__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<Command>, size: usize) -> bool;
    fn arx_chassis_interfaces__msg__Command__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<Command>);
    fn arx_chassis_interfaces__msg__Command__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<Command>, out_seq: *mut rosidl_runtime_rs::Sequence<Command>) -> bool;
}

// Corresponds to arx_chassis_interfaces__msg__Command
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Command {

    // This member is not documented.
    #[allow(missing_docs)]
    pub request_id: rosidl_runtime_rs::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub scheduler_instance: rosidl_runtime_rs::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub client_id: rosidl_runtime_rs::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub operation: rosidl_runtime_rs::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub kind: rosidl_runtime_rs::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub source_id: rosidl_runtime_rs::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub task_id: rosidl_runtime_rs::String,


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
    unsafe {
      let mut msg = std::mem::zeroed();
      if !arx_chassis_interfaces__msg__Command__init(&mut msg as *mut _) {
        panic!("Call to arx_chassis_interfaces__msg__Command__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for Command {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { arx_chassis_interfaces__msg__Command__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { arx_chassis_interfaces__msg__Command__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { arx_chassis_interfaces__msg__Command__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for Command {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for Command where Self: Sized {
  const TYPE_NAME: &'static str = "arx_chassis_interfaces/msg/Command";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__arx_chassis_interfaces__msg__Command() }
  }
}


#[link(name = "arx_chassis_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__arx_chassis_interfaces__msg__State() -> *const std::ffi::c_void;
}

#[link(name = "arx_chassis_interfaces__rosidl_generator_c")]
extern "C" {
    fn arx_chassis_interfaces__msg__State__init(msg: *mut State) -> bool;
    fn arx_chassis_interfaces__msg__State__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<State>, size: usize) -> bool;
    fn arx_chassis_interfaces__msg__State__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<State>);
    fn arx_chassis_interfaces__msg__State__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<State>, out_seq: *mut rosidl_runtime_rs::Sequence<State>) -> bool;
}

// Corresponds to arx_chassis_interfaces__msg__State
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct State {

    // This member is not documented.
    #[allow(missing_docs)]
    pub scheduler_instance: rosidl_runtime_rs::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub ready: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub driver_instance: rosidl_runtime_rs::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub active: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub client_id: rosidl_runtime_rs::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub source_id: rosidl_runtime_rs::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub task_id: rosidl_runtime_rs::String,


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
    pub reason: rosidl_runtime_rs::String,

}



impl Default for State {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !arx_chassis_interfaces__msg__State__init(&mut msg as *mut _) {
        panic!("Call to arx_chassis_interfaces__msg__State__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for State {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { arx_chassis_interfaces__msg__State__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { arx_chassis_interfaces__msg__State__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { arx_chassis_interfaces__msg__State__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for State {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for State where Self: Sized {
  const TYPE_NAME: &'static str = "arx_chassis_interfaces/msg/State";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__arx_chassis_interfaces__msg__State() }
  }
}


#[link(name = "arx_chassis_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__arx_chassis_interfaces__msg__Result() -> *const std::ffi::c_void;
}

#[link(name = "arx_chassis_interfaces__rosidl_generator_c")]
extern "C" {
    fn arx_chassis_interfaces__msg__Result__init(msg: *mut Result) -> bool;
    fn arx_chassis_interfaces__msg__Result__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<Result>, size: usize) -> bool;
    fn arx_chassis_interfaces__msg__Result__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<Result>);
    fn arx_chassis_interfaces__msg__Result__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<Result>, out_seq: *mut rosidl_runtime_rs::Sequence<Result>) -> bool;
}

// Corresponds to arx_chassis_interfaces__msg__Result
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Result {

    // This member is not documented.
    #[allow(missing_docs)]
    pub request_id: rosidl_runtime_rs::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub accepted: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub reason: rosidl_runtime_rs::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub output_seq: u64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub state: super::super::msg::rmw::State,

}



impl Default for Result {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !arx_chassis_interfaces__msg__Result__init(&mut msg as *mut _) {
        panic!("Call to arx_chassis_interfaces__msg__Result__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for Result {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { arx_chassis_interfaces__msg__Result__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { arx_chassis_interfaces__msg__Result__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { arx_chassis_interfaces__msg__Result__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for Result {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for Result where Self: Sized {
  const TYPE_NAME: &'static str = "arx_chassis_interfaces/msg/Result";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__arx_chassis_interfaces__msg__Result() }
  }
}


#[link(name = "arx_chassis_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__arx_chassis_interfaces__msg__Output() -> *const std::ffi::c_void;
}

#[link(name = "arx_chassis_interfaces__rosidl_generator_c")]
extern "C" {
    fn arx_chassis_interfaces__msg__Output__init(msg: *mut Output) -> bool;
    fn arx_chassis_interfaces__msg__Output__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<Output>, size: usize) -> bool;
    fn arx_chassis_interfaces__msg__Output__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<Output>);
    fn arx_chassis_interfaces__msg__Output__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<Output>, out_seq: *mut rosidl_runtime_rs::Sequence<Output>) -> bool;
}

// Corresponds to arx_chassis_interfaces__msg__Output
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Output {

    // This member is not documented.
    #[allow(missing_docs)]
    pub scheduler_instance: rosidl_runtime_rs::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub driver_instance: rosidl_runtime_rs::String,


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
    pub reason: rosidl_runtime_rs::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub source_id: rosidl_runtime_rs::String,

}



impl Default for Output {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !arx_chassis_interfaces__msg__Output__init(&mut msg as *mut _) {
        panic!("Call to arx_chassis_interfaces__msg__Output__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for Output {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { arx_chassis_interfaces__msg__Output__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { arx_chassis_interfaces__msg__Output__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { arx_chassis_interfaces__msg__Output__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for Output {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for Output where Self: Sized {
  const TYPE_NAME: &'static str = "arx_chassis_interfaces/msg/Output";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__arx_chassis_interfaces__msg__Output() }
  }
}


