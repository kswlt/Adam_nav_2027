#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};


#[link(name = "pb_rm_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__pb_rm_interfaces__msg__Buff() -> *const std::ffi::c_void;
}

#[link(name = "pb_rm_interfaces__rosidl_generator_c")]
extern "C" {
    fn pb_rm_interfaces__msg__Buff__init(msg: *mut Buff) -> bool;
    fn pb_rm_interfaces__msg__Buff__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<Buff>, size: usize) -> bool;
    fn pb_rm_interfaces__msg__Buff__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<Buff>);
    fn pb_rm_interfaces__msg__Buff__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<Buff>, out_seq: *mut rosidl_runtime_rs::Sequence<Buff>) -> bool;
}

// Corresponds to pb_rm_interfaces__msg__Buff
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// 机器人增益和底盘能量数据 (裁判系统串口协议 V1.7.0 0x0204)

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Buff {
    /// 机器人回血增益（百分比，值为 10 表示每秒恢复血量上限的 10%）
    pub recovery_buff: u8,

    /// 机器人射击热量冷却倍率（直接值，值为 5 表示 5 倍冷却）
    pub cooling_buff: u8,

    /// 机器人防御增益（百分比，值为 50 表示 50% 防御增益）
    pub defence_buff: u8,

    /// 机器人负防御增益（百分比，值为 30 表示 -30% 防御增益）
    pub vulnerability_buff: u8,

    /// 机器人攻击增益（百分比，值为 50 表示 50% 攻击增益）
    pub attack_buff: u16,

    /// 机器人剩余能量值反馈，以 16 进制标识机器人剩余能量值比例，仅在机器人剩余能量小于 50% 时反馈，其余默认反馈 0x32。
    pub remaining_energy: u8,

}



impl Default for Buff {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !pb_rm_interfaces__msg__Buff__init(&mut msg as *mut _) {
        panic!("Call to pb_rm_interfaces__msg__Buff__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for Buff {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { pb_rm_interfaces__msg__Buff__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { pb_rm_interfaces__msg__Buff__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { pb_rm_interfaces__msg__Buff__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for Buff {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for Buff where Self: Sized {
  const TYPE_NAME: &'static str = "pb_rm_interfaces/msg/Buff";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__pb_rm_interfaces__msg__Buff() }
  }
}


#[link(name = "pb_rm_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__pb_rm_interfaces__msg__EventData() -> *const std::ffi::c_void;
}

#[link(name = "pb_rm_interfaces__rosidl_generator_c")]
extern "C" {
    fn pb_rm_interfaces__msg__EventData__init(msg: *mut EventData) -> bool;
    fn pb_rm_interfaces__msg__EventData__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<EventData>, size: usize) -> bool;
    fn pb_rm_interfaces__msg__EventData__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<EventData>);
    fn pb_rm_interfaces__msg__EventData__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<EventData>, out_seq: *mut rosidl_runtime_rs::Sequence<EventData>) -> bool;
}

// Corresponds to pb_rm_interfaces__msg__EventData
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// 场地事件数据 (裁判系统串口协议 V1.7.0 0x0101)

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct EventData {
    /// 己方与兑换区不重叠的补给区的占领状态，1 为已占领
    pub non_overlapping_supply_zone: u8,

    /// 己方与兑换区重叠的补给区的占领状态，1 为已占领
    pub overlapping_supply_zone: u8,

    /// 己方补给区的占领状态，1 为已占领（仅 RMUL 适用）
    pub supply_zone: u8,

    /// 己方小能量机关的激活状态，1 为已激活
    pub small_energy: u8,

    /// 己方大能量机关的激活状态，1 为已激活
    pub big_energy: u8,

    /// 己方中央高地的占领状态，1 为被己方占领，2 为被对方占领
    pub central_highland: u8,

    /// 己方梯形高地的占领状态，1 为被己方占领，2 为被对方占领
    pub trapezoidal_highland: u8,

    /// 中心增益点的占领情况，
    /// 0 为未被占领，1 为被己方占领，2 为被对方占领，3 为被双方占领（仅 RMUL 适用）
    pub center_gain_zone: u8,

}

impl EventData {
    /// Constants for the occupation and activation states
    /// Not occupied or not activated
    pub const UNOCCUPIED: u8 = 0;

    /// Occupied or activated by friendly side
    pub const OCCUPIED_FRIEND: u8 = 1;

    /// Occupied or activated by enemy side
    pub const OCCUPIED_ENEMY: u8 = 2;

    /// Occupied or activated by both sides
    pub const OCCUPIED_BOTH: u8 = 3;

}


impl Default for EventData {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !pb_rm_interfaces__msg__EventData__init(&mut msg as *mut _) {
        panic!("Call to pb_rm_interfaces__msg__EventData__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for EventData {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { pb_rm_interfaces__msg__EventData__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { pb_rm_interfaces__msg__EventData__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { pb_rm_interfaces__msg__EventData__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for EventData {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for EventData where Self: Sized {
  const TYPE_NAME: &'static str = "pb_rm_interfaces/msg/EventData";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__pb_rm_interfaces__msg__EventData() }
  }
}


#[link(name = "pb_rm_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__pb_rm_interfaces__msg__GameRobotHP() -> *const std::ffi::c_void;
}

#[link(name = "pb_rm_interfaces__rosidl_generator_c")]
extern "C" {
    fn pb_rm_interfaces__msg__GameRobotHP__init(msg: *mut GameRobotHP) -> bool;
    fn pb_rm_interfaces__msg__GameRobotHP__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<GameRobotHP>, size: usize) -> bool;
    fn pb_rm_interfaces__msg__GameRobotHP__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<GameRobotHP>);
    fn pb_rm_interfaces__msg__GameRobotHP__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<GameRobotHP>, out_seq: *mut rosidl_runtime_rs::Sequence<GameRobotHP>) -> bool;
}

// Corresponds to pb_rm_interfaces__msg__GameRobotHP
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// 机器人血量数据 (裁判系统串口协议 V1.7.0 0x0003)

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct GameRobotHP {
    /// 红 1 英雄机器人血量。若该机器人未上场或者被罚下，则血量为 0
    pub red_1_robot_hp: u16,

    /// 红 2 工程机器人血量
    pub red_2_robot_hp: u16,

    /// 红 3 步兵机器人血量
    pub red_3_robot_hp: u16,

    /// 红 4 步兵机器人血量
    pub red_4_robot_hp: u16,

    /// 红 7 哨兵机器人血量
    pub red_7_robot_hp: u16,

    /// 红方前哨站血量
    pub red_outpost_hp: u16,

    /// 红方基地血量
    pub red_base_hp: u16,

    /// 蓝 1 英雄机器人血
    pub blue_1_robot_hp: u16,

    /// 蓝 2 工程机器人血量
    pub blue_2_robot_hp: u16,

    /// 蓝 3 步兵机器人血量
    pub blue_3_robot_hp: u16,

    /// 蓝 4 步兵机器人血量
    pub blue_4_robot_hp: u16,

    /// 蓝 7 哨兵机器人血量
    pub blue_7_robot_hp: u16,

    /// 蓝方前哨站血量
    pub blue_outpost_hp: u16,

    /// 蓝方基地血量
    pub blue_base_hp: u16,

}



impl Default for GameRobotHP {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !pb_rm_interfaces__msg__GameRobotHP__init(&mut msg as *mut _) {
        panic!("Call to pb_rm_interfaces__msg__GameRobotHP__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for GameRobotHP {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { pb_rm_interfaces__msg__GameRobotHP__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { pb_rm_interfaces__msg__GameRobotHP__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { pb_rm_interfaces__msg__GameRobotHP__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for GameRobotHP {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for GameRobotHP where Self: Sized {
  const TYPE_NAME: &'static str = "pb_rm_interfaces/msg/GameRobotHP";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__pb_rm_interfaces__msg__GameRobotHP() }
  }
}


#[link(name = "pb_rm_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__pb_rm_interfaces__msg__GameStatus() -> *const std::ffi::c_void;
}

#[link(name = "pb_rm_interfaces__rosidl_generator_c")]
extern "C" {
    fn pb_rm_interfaces__msg__GameStatus__init(msg: *mut GameStatus) -> bool;
    fn pb_rm_interfaces__msg__GameStatus__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<GameStatus>, size: usize) -> bool;
    fn pb_rm_interfaces__msg__GameStatus__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<GameStatus>);
    fn pb_rm_interfaces__msg__GameStatus__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<GameStatus>, out_seq: *mut rosidl_runtime_rs::Sequence<GameStatus>) -> bool;
}

// Corresponds to pb_rm_interfaces__msg__GameStatus
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// 比赛状态数据 (裁判系统串口协议 V1.7.0 0x0001)

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct GameStatus {

    // This member is not documented.
    #[allow(missing_docs)]
    pub game_type: u8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub game_progress: u8,

    /// 当前阶段剩余时间，单位：秒
    pub stage_remain_time: i32,

}

impl GameStatus {
    /// constants for game progress
    /// 未开始比赛
    pub const NOT_START: u8 = 0;

    /// 准备阶段
    pub const PREPARATION: u8 = 1;

    /// 十五秒裁判系统自检阶段
    pub const SELF_CHECKING: u8 = 2;

    /// 五秒倒计时
    pub const COUNT_DOWN: u8 = 3;

    /// 比赛中
    pub const RUNNING: u8 = 4;

    /// 比赛结算中
    pub const GAME_OVER: u8 = 5;

}


impl Default for GameStatus {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !pb_rm_interfaces__msg__GameStatus__init(&mut msg as *mut _) {
        panic!("Call to pb_rm_interfaces__msg__GameStatus__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for GameStatus {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { pb_rm_interfaces__msg__GameStatus__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { pb_rm_interfaces__msg__GameStatus__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { pb_rm_interfaces__msg__GameStatus__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for GameStatus {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for GameStatus where Self: Sized {
  const TYPE_NAME: &'static str = "pb_rm_interfaces/msg/GameStatus";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__pb_rm_interfaces__msg__GameStatus() }
  }
}


#[link(name = "pb_rm_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__pb_rm_interfaces__msg__GroundRobotPosition() -> *const std::ffi::c_void;
}

#[link(name = "pb_rm_interfaces__rosidl_generator_c")]
extern "C" {
    fn pb_rm_interfaces__msg__GroundRobotPosition__init(msg: *mut GroundRobotPosition) -> bool;
    fn pb_rm_interfaces__msg__GroundRobotPosition__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<GroundRobotPosition>, size: usize) -> bool;
    fn pb_rm_interfaces__msg__GroundRobotPosition__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<GroundRobotPosition>);
    fn pb_rm_interfaces__msg__GroundRobotPosition__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<GroundRobotPosition>, out_seq: *mut rosidl_runtime_rs::Sequence<GroundRobotPosition>) -> bool;
}

// Corresponds to pb_rm_interfaces__msg__GroundRobotPosition
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// 地面机器人位置数据 (裁判系统串口协议 V1.7.0 0x020B)

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct GroundRobotPosition {
    /// 己方英雄机器人位置
    pub hero_position: geometry_msgs::msg::rmw::Point,

    /// 己方工程机器人位置
    pub engineer_position: geometry_msgs::msg::rmw::Point,

    /// 己方 3 号步兵机器人位置
    pub standard_3_position: geometry_msgs::msg::rmw::Point,

    /// 己方 4 号步兵机器人位置
    pub standard_4_position: geometry_msgs::msg::rmw::Point,

}



impl Default for GroundRobotPosition {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !pb_rm_interfaces__msg__GroundRobotPosition__init(&mut msg as *mut _) {
        panic!("Call to pb_rm_interfaces__msg__GroundRobotPosition__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for GroundRobotPosition {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { pb_rm_interfaces__msg__GroundRobotPosition__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { pb_rm_interfaces__msg__GroundRobotPosition__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { pb_rm_interfaces__msg__GroundRobotPosition__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for GroundRobotPosition {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for GroundRobotPosition where Self: Sized {
  const TYPE_NAME: &'static str = "pb_rm_interfaces/msg/GroundRobotPosition";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__pb_rm_interfaces__msg__GroundRobotPosition() }
  }
}


#[link(name = "pb_rm_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__pb_rm_interfaces__msg__RfidStatus() -> *const std::ffi::c_void;
}

#[link(name = "pb_rm_interfaces__rosidl_generator_c")]
extern "C" {
    fn pb_rm_interfaces__msg__RfidStatus__init(msg: *mut RfidStatus) -> bool;
    fn pb_rm_interfaces__msg__RfidStatus__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<RfidStatus>, size: usize) -> bool;
    fn pb_rm_interfaces__msg__RfidStatus__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<RfidStatus>);
    fn pb_rm_interfaces__msg__RfidStatus__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<RfidStatus>, out_seq: *mut rosidl_runtime_rs::Sequence<RfidStatus>) -> bool;
}

// Corresponds to pb_rm_interfaces__msg__RfidStatus
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// 机器人 RFID 模块状态 (裁判系统串口协议 V1.7.0 0x0209)

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct RfidStatus {
    /// 己方基地增益点
    pub base_gain_point: bool,

    /// 己方中央高地增益点
    pub central_highland_gain_point: bool,

    /// 对方中央高地增益点
    pub enemy_central_highland_gain_point: bool,

    /// 己方梯形高地增益点
    pub friendly_trapezoidal_highland_gain_point: bool,

    /// 对方梯形高地增益点
    pub enemy_trapezoidal_highland_gain_point: bool,

    /// 己方地形跨越增益点（飞坡）（靠近己方一侧飞坡前）
    pub friendly_fly_ramp_front_gain_point: bool,

    /// 己方地形跨越增益点（飞坡）（靠近己方一侧飞坡后）
    pub friendly_fly_ramp_back_gain_point: bool,

    /// 对方地形跨越增益点（飞坡）（靠近对方一侧飞坡前）
    pub enemy_fly_ramp_front_gain_point: bool,

    /// 对方地形跨越增益点（飞坡）（靠近对方一侧飞坡后）
    pub enemy_fly_ramp_back_gain_point: bool,

    /// 己方地形跨越增益点（中央高地下方）
    pub friendly_central_highland_lower_gain_point: bool,

    /// 己方地形跨越增益点（中央高地上方）
    pub friendly_central_highland_upper_gain_point: bool,

    /// 对方地形跨越增益点（中央高地下方）
    pub enemy_central_highland_lower_gain_point: bool,

    /// 对方地形跨越增益点（中央高地上方）
    pub enemy_central_highland_upper_gain_point: bool,

    /// 己方地形跨越增益点（公路下方）
    pub friendly_highway_lower_gain_point: bool,

    /// 己方地形跨越增益点（公路上方）
    pub friendly_highway_upper_gain_point: bool,

    /// 对方地形跨越增益点（公路下方）
    pub enemy_highway_lower_gain_point: bool,

    /// 对方地形跨越增益点（公路上方）
    pub enemy_highway_upper_gain_point: bool,

    /// 己方堡垒增益点
    pub friendly_fortress_gain_point: bool,

    /// 己方前哨站增益点
    pub friendly_outpost_gain_point: bool,

    /// 己方与兑换区不重叠的补给区/RMUL 补给区
    pub friendly_supply_zone_non_exchange: bool,

    /// 己方与兑换区重叠的补给区
    pub friendly_supply_zone_exchange: bool,

    /// 己方大资源岛增益点
    pub friendly_big_resource_island: bool,

    /// 对方大资源岛增益点
    pub enemy_big_resource_island: bool,

    /// 中心增益点（仅 RMUL 适用）
    pub center_gain_point: bool,

}

impl RfidStatus {
    /// const for RFID status
    /// RFID card not detected
    pub const NOT_DETECTED: u8 = 0;

    /// RFID card detected
    pub const DETECTED: u8 = 1;

}


impl Default for RfidStatus {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !pb_rm_interfaces__msg__RfidStatus__init(&mut msg as *mut _) {
        panic!("Call to pb_rm_interfaces__msg__RfidStatus__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for RfidStatus {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { pb_rm_interfaces__msg__RfidStatus__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { pb_rm_interfaces__msg__RfidStatus__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { pb_rm_interfaces__msg__RfidStatus__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for RfidStatus {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for RfidStatus where Self: Sized {
  const TYPE_NAME: &'static str = "pb_rm_interfaces/msg/RfidStatus";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__pb_rm_interfaces__msg__RfidStatus() }
  }
}


#[link(name = "pb_rm_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__pb_rm_interfaces__msg__RobotStatus() -> *const std::ffi::c_void;
}

#[link(name = "pb_rm_interfaces__rosidl_generator_c")]
extern "C" {
    fn pb_rm_interfaces__msg__RobotStatus__init(msg: *mut RobotStatus) -> bool;
    fn pb_rm_interfaces__msg__RobotStatus__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<RobotStatus>, size: usize) -> bool;
    fn pb_rm_interfaces__msg__RobotStatus__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<RobotStatus>);
    fn pb_rm_interfaces__msg__RobotStatus__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<RobotStatus>, out_seq: *mut rosidl_runtime_rs::Sequence<RobotStatus>) -> bool;
}

// Corresponds to pb_rm_interfaces__msg__RobotStatus
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// 机器人性能体系数据 (裁判系统串口协议 V1.7.0 0x0201)

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct RobotStatus {
    /// 本机器人 ID
    pub robot_id: u8,

    /// 机器人等级
    pub robot_level: u8,

    /// 机器人当前血量
    pub current_hp: u16,

    /// 机器人血量上限
    pub maximum_hp: u16,

    /// 机器人枪口热量每秒冷却值
    pub shooter_barrel_cooling_value: u16,

    /// 机器人枪口热量上限
    pub shooter_barrel_heat_limit: u16,

    /// 实时底盘缓冲能量和射击热量数 (裁判系统串口协议 V1.7.0 0x0202)
    /// 第 1 个 17mm 发射机构的枪口热量
    pub shooter_17mm_1_barrel_heat: u16,

    /// 本机机器人位置数据 (裁判系统串口协议 V1.7.0 0x0203)
    /// 本机器人姿态
    pub robot_pos: geometry_msgs::msg::rmw::Pose,

    /// 当扣血原因为装甲模块被弹丸攻击、受撞击、离线或测速模块离线时，
    /// 数值为装甲模块或测速模块的 ID 编号；当其他原因导致扣血时，该数值为 0
    pub armor_id: u8,

    /// 血量变化类型
    pub hp_deduction_reason: u8,

    /// 允许发弹量 (裁判系统串口协议 V1.7.0 0x0208)
    /// 17mm 弹丸剩余发射次数
    pub projectile_allowance_17mm: u16,

    /// 剩余金币数量
    pub remaining_gold_coin: u16,

    /// 血量是否下降（上位机二次处理）
    pub is_hp_deduced: bool,

}

impl RobotStatus {
    /// 伤害状态数据 (裁判系统串口协议 V1.7.0 0x0206)
    /// const for hp_deduction_reason
    /// 装甲模块被弹丸攻击导致扣血
    pub const ARMOR_HIT: u8 = 0;

    /// 裁判系统重要模块离线导致扣血
    pub const SYSTEM_OFFLINE: u8 = 1;

    /// 射击初速度超限导致扣血
    pub const OVER_SHOOT_SPEED: u8 = 2;

    /// 枪口热量超限导致扣血
    pub const OVER_HEAT: u8 = 3;

    /// 底盘功率超限导致扣血
    pub const OVER_POWER: u8 = 4;

    /// 装甲模块受到撞击导致扣血
    pub const ARMOR_COLLISION: u8 = 5;

}


impl Default for RobotStatus {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !pb_rm_interfaces__msg__RobotStatus__init(&mut msg as *mut _) {
        panic!("Call to pb_rm_interfaces__msg__RobotStatus__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for RobotStatus {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { pb_rm_interfaces__msg__RobotStatus__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { pb_rm_interfaces__msg__RobotStatus__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { pb_rm_interfaces__msg__RobotStatus__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for RobotStatus {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for RobotStatus where Self: Sized {
  const TYPE_NAME: &'static str = "pb_rm_interfaces/msg/RobotStatus";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__pb_rm_interfaces__msg__RobotStatus() }
  }
}


#[link(name = "pb_rm_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__pb_rm_interfaces__msg__Gimbal() -> *const std::ffi::c_void;
}

#[link(name = "pb_rm_interfaces__rosidl_generator_c")]
extern "C" {
    fn pb_rm_interfaces__msg__Gimbal__init(msg: *mut Gimbal) -> bool;
    fn pb_rm_interfaces__msg__Gimbal__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<Gimbal>, size: usize) -> bool;
    fn pb_rm_interfaces__msg__Gimbal__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<Gimbal>);
    fn pb_rm_interfaces__msg__Gimbal__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<Gimbal>, out_seq: *mut rosidl_runtime_rs::Sequence<Gimbal>) -> bool;
}

// Corresponds to pb_rm_interfaces__msg__Gimbal
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// msg for Gimbal, pitch and yaw.

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Gimbal {

    // This member is not documented.
    #[allow(missing_docs)]
    pub pitch: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub yaw: f32,

    /// Only used in velocity control type
    pub pitch_min_range: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub pitch_max_range: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub yaw_min_range: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub yaw_max_range: f32,

}



impl Default for Gimbal {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !pb_rm_interfaces__msg__Gimbal__init(&mut msg as *mut _) {
        panic!("Call to pb_rm_interfaces__msg__Gimbal__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for Gimbal {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { pb_rm_interfaces__msg__Gimbal__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { pb_rm_interfaces__msg__Gimbal__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { pb_rm_interfaces__msg__Gimbal__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for Gimbal {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for Gimbal where Self: Sized {
  const TYPE_NAME: &'static str = "pb_rm_interfaces/msg/Gimbal";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__pb_rm_interfaces__msg__Gimbal() }
  }
}


#[link(name = "pb_rm_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__pb_rm_interfaces__msg__GimbalCmd() -> *const std::ffi::c_void;
}

#[link(name = "pb_rm_interfaces__rosidl_generator_c")]
extern "C" {
    fn pb_rm_interfaces__msg__GimbalCmd__init(msg: *mut GimbalCmd) -> bool;
    fn pb_rm_interfaces__msg__GimbalCmd__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<GimbalCmd>, size: usize) -> bool;
    fn pb_rm_interfaces__msg__GimbalCmd__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<GimbalCmd>);
    fn pb_rm_interfaces__msg__GimbalCmd__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<GimbalCmd>, out_seq: *mut rosidl_runtime_rs::Sequence<GimbalCmd>) -> bool;
}

// Corresponds to pb_rm_interfaces__msg__GimbalCmd
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct GimbalCmd {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::rmw::Header,

    /// control type
    pub yaw_type: u8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub pitch_type: u8,

    /// control dada
    pub position: super::super::msg::rmw::Gimbal,


    // This member is not documented.
    #[allow(missing_docs)]
    pub velocity: super::super::msg::rmw::Gimbal,

}

impl GimbalCmd {
    /// constants for control type
    /// position control, set position by absolute angle
    pub const ABSOLUTE_ANGLE: u8 = 1;

    /// velocity control, set velocity
    pub const VELOCITY: u8 = 2;

}


impl Default for GimbalCmd {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !pb_rm_interfaces__msg__GimbalCmd__init(&mut msg as *mut _) {
        panic!("Call to pb_rm_interfaces__msg__GimbalCmd__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for GimbalCmd {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { pb_rm_interfaces__msg__GimbalCmd__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { pb_rm_interfaces__msg__GimbalCmd__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { pb_rm_interfaces__msg__GimbalCmd__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for GimbalCmd {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for GimbalCmd where Self: Sized {
  const TYPE_NAME: &'static str = "pb_rm_interfaces/msg/GimbalCmd";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__pb_rm_interfaces__msg__GimbalCmd() }
  }
}


#[link(name = "pb_rm_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__pb_rm_interfaces__msg__Models() -> *const std::ffi::c_void;
}

#[link(name = "pb_rm_interfaces__rosidl_generator_c")]
extern "C" {
    fn pb_rm_interfaces__msg__Models__init(msg: *mut Models) -> bool;
    fn pb_rm_interfaces__msg__Models__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<Models>, size: usize) -> bool;
    fn pb_rm_interfaces__msg__Models__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<Models>);
    fn pb_rm_interfaces__msg__Models__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<Models>, out_seq: *mut rosidl_runtime_rs::Sequence<Models>) -> bool;
}

// Corresponds to pb_rm_interfaces__msg__Models
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Models {

    // This member is not documented.
    #[allow(missing_docs)]
    pub chassis: rosidl_runtime_rs::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub gimbal: rosidl_runtime_rs::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub shoot: rosidl_runtime_rs::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub arm: rosidl_runtime_rs::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub custom_controller: rosidl_runtime_rs::String,

}



impl Default for Models {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !pb_rm_interfaces__msg__Models__init(&mut msg as *mut _) {
        panic!("Call to pb_rm_interfaces__msg__Models__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for Models {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { pb_rm_interfaces__msg__Models__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { pb_rm_interfaces__msg__Models__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { pb_rm_interfaces__msg__Models__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for Models {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for Models where Self: Sized {
  const TYPE_NAME: &'static str = "pb_rm_interfaces/msg/Models";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__pb_rm_interfaces__msg__Models() }
  }
}


#[link(name = "pb_rm_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__pb_rm_interfaces__msg__RobotStateInfo() -> *const std::ffi::c_void;
}

#[link(name = "pb_rm_interfaces__rosidl_generator_c")]
extern "C" {
    fn pb_rm_interfaces__msg__RobotStateInfo__init(msg: *mut RobotStateInfo) -> bool;
    fn pb_rm_interfaces__msg__RobotStateInfo__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<RobotStateInfo>, size: usize) -> bool;
    fn pb_rm_interfaces__msg__RobotStateInfo__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<RobotStateInfo>);
    fn pb_rm_interfaces__msg__RobotStateInfo__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<RobotStateInfo>, out_seq: *mut rosidl_runtime_rs::Sequence<RobotStateInfo>) -> bool;
}

// Corresponds to pb_rm_interfaces__msg__RobotStateInfo
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct RobotStateInfo {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::rmw::Header,


    // This member is not documented.
    #[allow(missing_docs)]
    pub models: super::super::msg::rmw::Models,

}



impl Default for RobotStateInfo {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !pb_rm_interfaces__msg__RobotStateInfo__init(&mut msg as *mut _) {
        panic!("Call to pb_rm_interfaces__msg__RobotStateInfo__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for RobotStateInfo {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { pb_rm_interfaces__msg__RobotStateInfo__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { pb_rm_interfaces__msg__RobotStateInfo__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { pb_rm_interfaces__msg__RobotStateInfo__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for RobotStateInfo {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for RobotStateInfo where Self: Sized {
  const TYPE_NAME: &'static str = "pb_rm_interfaces/msg/RobotStateInfo";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__pb_rm_interfaces__msg__RobotStateInfo() }
  }
}


