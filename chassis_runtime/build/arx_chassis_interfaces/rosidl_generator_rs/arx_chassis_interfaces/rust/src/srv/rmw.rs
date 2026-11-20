#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};



#[link(name = "arx_chassis_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__arx_chassis_interfaces__srv__Manage_Request() -> *const std::ffi::c_void;
}

#[link(name = "arx_chassis_interfaces__rosidl_generator_c")]
extern "C" {
    fn arx_chassis_interfaces__srv__Manage_Request__init(msg: *mut Manage_Request) -> bool;
    fn arx_chassis_interfaces__srv__Manage_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<Manage_Request>, size: usize) -> bool;
    fn arx_chassis_interfaces__srv__Manage_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<Manage_Request>);
    fn arx_chassis_interfaces__srv__Manage_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<Manage_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<Manage_Request>) -> bool;
}

// Corresponds to arx_chassis_interfaces__srv__Manage_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Manage_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub operation: rosidl_runtime_rs::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub scheduler_instance: rosidl_runtime_rs::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub client_id: rosidl_runtime_rs::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub profile: rosidl_runtime_rs::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub resume_key: rosidl_runtime_rs::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub output_seq: u64,

}



impl Default for Manage_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !arx_chassis_interfaces__srv__Manage_Request__init(&mut msg as *mut _) {
        panic!("Call to arx_chassis_interfaces__srv__Manage_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for Manage_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { arx_chassis_interfaces__srv__Manage_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { arx_chassis_interfaces__srv__Manage_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { arx_chassis_interfaces__srv__Manage_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for Manage_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for Manage_Request where Self: Sized {
  const TYPE_NAME: &'static str = "arx_chassis_interfaces/srv/Manage_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__arx_chassis_interfaces__srv__Manage_Request() }
  }
}


#[link(name = "arx_chassis_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__arx_chassis_interfaces__srv__Manage_Response() -> *const std::ffi::c_void;
}

#[link(name = "arx_chassis_interfaces__rosidl_generator_c")]
extern "C" {
    fn arx_chassis_interfaces__srv__Manage_Response__init(msg: *mut Manage_Response) -> bool;
    fn arx_chassis_interfaces__srv__Manage_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<Manage_Response>, size: usize) -> bool;
    fn arx_chassis_interfaces__srv__Manage_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<Manage_Response>);
    fn arx_chassis_interfaces__srv__Manage_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<Manage_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<Manage_Response>) -> bool;
}

// Corresponds to arx_chassis_interfaces__srv__Manage_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Manage_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub accepted: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub reason: rosidl_runtime_rs::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub client_id: rosidl_runtime_rs::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub state: super::super::msg::rmw::State,

}



impl Default for Manage_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !arx_chassis_interfaces__srv__Manage_Response__init(&mut msg as *mut _) {
        panic!("Call to arx_chassis_interfaces__srv__Manage_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for Manage_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { arx_chassis_interfaces__srv__Manage_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { arx_chassis_interfaces__srv__Manage_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { arx_chassis_interfaces__srv__Manage_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for Manage_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for Manage_Response where Self: Sized {
  const TYPE_NAME: &'static str = "arx_chassis_interfaces/srv/Manage_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__arx_chassis_interfaces__srv__Manage_Response() }
  }
}






#[link(name = "arx_chassis_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__arx_chassis_interfaces__srv__Manage() -> *const std::ffi::c_void;
}

// Corresponds to arx_chassis_interfaces__srv__Manage
#[allow(missing_docs, non_camel_case_types)]
pub struct Manage;

impl rosidl_runtime_rs::Service for Manage {
    type Request = Manage_Request;
    type Response = Manage_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__arx_chassis_interfaces__srv__Manage() }
    }
}


