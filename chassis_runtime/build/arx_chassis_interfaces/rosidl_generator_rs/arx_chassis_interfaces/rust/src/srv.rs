#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};




// Corresponds to arx_chassis_interfaces__srv__Manage_Request

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Manage_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub operation: std::string::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub scheduler_instance: std::string::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub client_id: std::string::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub profile: std::string::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub resume_key: std::string::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub output_seq: u64,

}



impl Default for Manage_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::Manage_Request::default())
  }
}

impl rosidl_runtime_rs::Message for Manage_Request {
  type RmwMsg = super::srv::rmw::Manage_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        operation: msg.operation.as_str().into(),
        scheduler_instance: msg.scheduler_instance.as_str().into(),
        client_id: msg.client_id.as_str().into(),
        profile: msg.profile.as_str().into(),
        resume_key: msg.resume_key.as_str().into(),
        output_seq: msg.output_seq,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        operation: msg.operation.as_str().into(),
        scheduler_instance: msg.scheduler_instance.as_str().into(),
        client_id: msg.client_id.as_str().into(),
        profile: msg.profile.as_str().into(),
        resume_key: msg.resume_key.as_str().into(),
      output_seq: msg.output_seq,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      operation: msg.operation.to_string(),
      scheduler_instance: msg.scheduler_instance.to_string(),
      client_id: msg.client_id.to_string(),
      profile: msg.profile.to_string(),
      resume_key: msg.resume_key.to_string(),
      output_seq: msg.output_seq,
    }
  }
}


// Corresponds to arx_chassis_interfaces__srv__Manage_Response

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Manage_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub accepted: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub reason: std::string::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub client_id: std::string::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub state: super::msg::State,

}



impl Default for Manage_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::Manage_Response::default())
  }
}

impl rosidl_runtime_rs::Message for Manage_Response {
  type RmwMsg = super::srv::rmw::Manage_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        accepted: msg.accepted,
        reason: msg.reason.as_str().into(),
        client_id: msg.client_id.as_str().into(),
        state: super::msg::State::into_rmw_message(std::borrow::Cow::Owned(msg.state)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      accepted: msg.accepted,
        reason: msg.reason.as_str().into(),
        client_id: msg.client_id.as_str().into(),
        state: super::msg::State::into_rmw_message(std::borrow::Cow::Borrowed(&msg.state)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      accepted: msg.accepted,
      reason: msg.reason.to_string(),
      client_id: msg.client_id.to_string(),
      state: super::msg::State::from_rmw_message(msg.state),
    }
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


