#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};



// Corresponds to pb_rm_interfaces__msg__Buff
/// 机器人增益和底盘能量数据 (裁判系统串口协议 V1.7.0 0x0204)

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::Buff::default())
  }
}

impl rosidl_runtime_rs::Message for Buff {
  type RmwMsg = super::msg::rmw::Buff;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        recovery_buff: msg.recovery_buff,
        cooling_buff: msg.cooling_buff,
        defence_buff: msg.defence_buff,
        vulnerability_buff: msg.vulnerability_buff,
        attack_buff: msg.attack_buff,
        remaining_energy: msg.remaining_energy,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      recovery_buff: msg.recovery_buff,
      cooling_buff: msg.cooling_buff,
      defence_buff: msg.defence_buff,
      vulnerability_buff: msg.vulnerability_buff,
      attack_buff: msg.attack_buff,
      remaining_energy: msg.remaining_energy,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      recovery_buff: msg.recovery_buff,
      cooling_buff: msg.cooling_buff,
      defence_buff: msg.defence_buff,
      vulnerability_buff: msg.vulnerability_buff,
      attack_buff: msg.attack_buff,
      remaining_energy: msg.remaining_energy,
    }
  }
}


// Corresponds to pb_rm_interfaces__msg__EventData
/// 场地事件数据 (裁判系统串口协议 V1.7.0 0x0101)

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::EventData::default())
  }
}

impl rosidl_runtime_rs::Message for EventData {
  type RmwMsg = super::msg::rmw::EventData;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        non_overlapping_supply_zone: msg.non_overlapping_supply_zone,
        overlapping_supply_zone: msg.overlapping_supply_zone,
        supply_zone: msg.supply_zone,
        small_energy: msg.small_energy,
        big_energy: msg.big_energy,
        central_highland: msg.central_highland,
        trapezoidal_highland: msg.trapezoidal_highland,
        center_gain_zone: msg.center_gain_zone,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      non_overlapping_supply_zone: msg.non_overlapping_supply_zone,
      overlapping_supply_zone: msg.overlapping_supply_zone,
      supply_zone: msg.supply_zone,
      small_energy: msg.small_energy,
      big_energy: msg.big_energy,
      central_highland: msg.central_highland,
      trapezoidal_highland: msg.trapezoidal_highland,
      center_gain_zone: msg.center_gain_zone,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      non_overlapping_supply_zone: msg.non_overlapping_supply_zone,
      overlapping_supply_zone: msg.overlapping_supply_zone,
      supply_zone: msg.supply_zone,
      small_energy: msg.small_energy,
      big_energy: msg.big_energy,
      central_highland: msg.central_highland,
      trapezoidal_highland: msg.trapezoidal_highland,
      center_gain_zone: msg.center_gain_zone,
    }
  }
}


// Corresponds to pb_rm_interfaces__msg__GameRobotHP
/// 机器人血量数据 (裁判系统串口协议 V1.7.0 0x0003)

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::GameRobotHP::default())
  }
}

impl rosidl_runtime_rs::Message for GameRobotHP {
  type RmwMsg = super::msg::rmw::GameRobotHP;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        red_1_robot_hp: msg.red_1_robot_hp,
        red_2_robot_hp: msg.red_2_robot_hp,
        red_3_robot_hp: msg.red_3_robot_hp,
        red_4_robot_hp: msg.red_4_robot_hp,
        red_7_robot_hp: msg.red_7_robot_hp,
        red_outpost_hp: msg.red_outpost_hp,
        red_base_hp: msg.red_base_hp,
        blue_1_robot_hp: msg.blue_1_robot_hp,
        blue_2_robot_hp: msg.blue_2_robot_hp,
        blue_3_robot_hp: msg.blue_3_robot_hp,
        blue_4_robot_hp: msg.blue_4_robot_hp,
        blue_7_robot_hp: msg.blue_7_robot_hp,
        blue_outpost_hp: msg.blue_outpost_hp,
        blue_base_hp: msg.blue_base_hp,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      red_1_robot_hp: msg.red_1_robot_hp,
      red_2_robot_hp: msg.red_2_robot_hp,
      red_3_robot_hp: msg.red_3_robot_hp,
      red_4_robot_hp: msg.red_4_robot_hp,
      red_7_robot_hp: msg.red_7_robot_hp,
      red_outpost_hp: msg.red_outpost_hp,
      red_base_hp: msg.red_base_hp,
      blue_1_robot_hp: msg.blue_1_robot_hp,
      blue_2_robot_hp: msg.blue_2_robot_hp,
      blue_3_robot_hp: msg.blue_3_robot_hp,
      blue_4_robot_hp: msg.blue_4_robot_hp,
      blue_7_robot_hp: msg.blue_7_robot_hp,
      blue_outpost_hp: msg.blue_outpost_hp,
      blue_base_hp: msg.blue_base_hp,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      red_1_robot_hp: msg.red_1_robot_hp,
      red_2_robot_hp: msg.red_2_robot_hp,
      red_3_robot_hp: msg.red_3_robot_hp,
      red_4_robot_hp: msg.red_4_robot_hp,
      red_7_robot_hp: msg.red_7_robot_hp,
      red_outpost_hp: msg.red_outpost_hp,
      red_base_hp: msg.red_base_hp,
      blue_1_robot_hp: msg.blue_1_robot_hp,
      blue_2_robot_hp: msg.blue_2_robot_hp,
      blue_3_robot_hp: msg.blue_3_robot_hp,
      blue_4_robot_hp: msg.blue_4_robot_hp,
      blue_7_robot_hp: msg.blue_7_robot_hp,
      blue_outpost_hp: msg.blue_outpost_hp,
      blue_base_hp: msg.blue_base_hp,
    }
  }
}


// Corresponds to pb_rm_interfaces__msg__GameStatus
/// 比赛状态数据 (裁判系统串口协议 V1.7.0 0x0001)

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::GameStatus::default())
  }
}

impl rosidl_runtime_rs::Message for GameStatus {
  type RmwMsg = super::msg::rmw::GameStatus;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        game_type: msg.game_type,
        game_progress: msg.game_progress,
        stage_remain_time: msg.stage_remain_time,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      game_type: msg.game_type,
      game_progress: msg.game_progress,
      stage_remain_time: msg.stage_remain_time,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      game_type: msg.game_type,
      game_progress: msg.game_progress,
      stage_remain_time: msg.stage_remain_time,
    }
  }
}


// Corresponds to pb_rm_interfaces__msg__GroundRobotPosition
/// 地面机器人位置数据 (裁判系统串口协议 V1.7.0 0x020B)

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct GroundRobotPosition {
    /// 己方英雄机器人位置
    pub hero_position: geometry_msgs::msg::Point,

    /// 己方工程机器人位置
    pub engineer_position: geometry_msgs::msg::Point,

    /// 己方 3 号步兵机器人位置
    pub standard_3_position: geometry_msgs::msg::Point,

    /// 己方 4 号步兵机器人位置
    pub standard_4_position: geometry_msgs::msg::Point,

}



impl Default for GroundRobotPosition {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::GroundRobotPosition::default())
  }
}

impl rosidl_runtime_rs::Message for GroundRobotPosition {
  type RmwMsg = super::msg::rmw::GroundRobotPosition;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        hero_position: geometry_msgs::msg::Point::into_rmw_message(std::borrow::Cow::Owned(msg.hero_position)).into_owned(),
        engineer_position: geometry_msgs::msg::Point::into_rmw_message(std::borrow::Cow::Owned(msg.engineer_position)).into_owned(),
        standard_3_position: geometry_msgs::msg::Point::into_rmw_message(std::borrow::Cow::Owned(msg.standard_3_position)).into_owned(),
        standard_4_position: geometry_msgs::msg::Point::into_rmw_message(std::borrow::Cow::Owned(msg.standard_4_position)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        hero_position: geometry_msgs::msg::Point::into_rmw_message(std::borrow::Cow::Borrowed(&msg.hero_position)).into_owned(),
        engineer_position: geometry_msgs::msg::Point::into_rmw_message(std::borrow::Cow::Borrowed(&msg.engineer_position)).into_owned(),
        standard_3_position: geometry_msgs::msg::Point::into_rmw_message(std::borrow::Cow::Borrowed(&msg.standard_3_position)).into_owned(),
        standard_4_position: geometry_msgs::msg::Point::into_rmw_message(std::borrow::Cow::Borrowed(&msg.standard_4_position)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      hero_position: geometry_msgs::msg::Point::from_rmw_message(msg.hero_position),
      engineer_position: geometry_msgs::msg::Point::from_rmw_message(msg.engineer_position),
      standard_3_position: geometry_msgs::msg::Point::from_rmw_message(msg.standard_3_position),
      standard_4_position: geometry_msgs::msg::Point::from_rmw_message(msg.standard_4_position),
    }
  }
}


// Corresponds to pb_rm_interfaces__msg__RfidStatus
/// 机器人 RFID 模块状态 (裁判系统串口协议 V1.7.0 0x0209)

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::RfidStatus::default())
  }
}

impl rosidl_runtime_rs::Message for RfidStatus {
  type RmwMsg = super::msg::rmw::RfidStatus;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        base_gain_point: msg.base_gain_point,
        central_highland_gain_point: msg.central_highland_gain_point,
        enemy_central_highland_gain_point: msg.enemy_central_highland_gain_point,
        friendly_trapezoidal_highland_gain_point: msg.friendly_trapezoidal_highland_gain_point,
        enemy_trapezoidal_highland_gain_point: msg.enemy_trapezoidal_highland_gain_point,
        friendly_fly_ramp_front_gain_point: msg.friendly_fly_ramp_front_gain_point,
        friendly_fly_ramp_back_gain_point: msg.friendly_fly_ramp_back_gain_point,
        enemy_fly_ramp_front_gain_point: msg.enemy_fly_ramp_front_gain_point,
        enemy_fly_ramp_back_gain_point: msg.enemy_fly_ramp_back_gain_point,
        friendly_central_highland_lower_gain_point: msg.friendly_central_highland_lower_gain_point,
        friendly_central_highland_upper_gain_point: msg.friendly_central_highland_upper_gain_point,
        enemy_central_highland_lower_gain_point: msg.enemy_central_highland_lower_gain_point,
        enemy_central_highland_upper_gain_point: msg.enemy_central_highland_upper_gain_point,
        friendly_highway_lower_gain_point: msg.friendly_highway_lower_gain_point,
        friendly_highway_upper_gain_point: msg.friendly_highway_upper_gain_point,
        enemy_highway_lower_gain_point: msg.enemy_highway_lower_gain_point,
        enemy_highway_upper_gain_point: msg.enemy_highway_upper_gain_point,
        friendly_fortress_gain_point: msg.friendly_fortress_gain_point,
        friendly_outpost_gain_point: msg.friendly_outpost_gain_point,
        friendly_supply_zone_non_exchange: msg.friendly_supply_zone_non_exchange,
        friendly_supply_zone_exchange: msg.friendly_supply_zone_exchange,
        friendly_big_resource_island: msg.friendly_big_resource_island,
        enemy_big_resource_island: msg.enemy_big_resource_island,
        center_gain_point: msg.center_gain_point,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      base_gain_point: msg.base_gain_point,
      central_highland_gain_point: msg.central_highland_gain_point,
      enemy_central_highland_gain_point: msg.enemy_central_highland_gain_point,
      friendly_trapezoidal_highland_gain_point: msg.friendly_trapezoidal_highland_gain_point,
      enemy_trapezoidal_highland_gain_point: msg.enemy_trapezoidal_highland_gain_point,
      friendly_fly_ramp_front_gain_point: msg.friendly_fly_ramp_front_gain_point,
      friendly_fly_ramp_back_gain_point: msg.friendly_fly_ramp_back_gain_point,
      enemy_fly_ramp_front_gain_point: msg.enemy_fly_ramp_front_gain_point,
      enemy_fly_ramp_back_gain_point: msg.enemy_fly_ramp_back_gain_point,
      friendly_central_highland_lower_gain_point: msg.friendly_central_highland_lower_gain_point,
      friendly_central_highland_upper_gain_point: msg.friendly_central_highland_upper_gain_point,
      enemy_central_highland_lower_gain_point: msg.enemy_central_highland_lower_gain_point,
      enemy_central_highland_upper_gain_point: msg.enemy_central_highland_upper_gain_point,
      friendly_highway_lower_gain_point: msg.friendly_highway_lower_gain_point,
      friendly_highway_upper_gain_point: msg.friendly_highway_upper_gain_point,
      enemy_highway_lower_gain_point: msg.enemy_highway_lower_gain_point,
      enemy_highway_upper_gain_point: msg.enemy_highway_upper_gain_point,
      friendly_fortress_gain_point: msg.friendly_fortress_gain_point,
      friendly_outpost_gain_point: msg.friendly_outpost_gain_point,
      friendly_supply_zone_non_exchange: msg.friendly_supply_zone_non_exchange,
      friendly_supply_zone_exchange: msg.friendly_supply_zone_exchange,
      friendly_big_resource_island: msg.friendly_big_resource_island,
      enemy_big_resource_island: msg.enemy_big_resource_island,
      center_gain_point: msg.center_gain_point,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      base_gain_point: msg.base_gain_point,
      central_highland_gain_point: msg.central_highland_gain_point,
      enemy_central_highland_gain_point: msg.enemy_central_highland_gain_point,
      friendly_trapezoidal_highland_gain_point: msg.friendly_trapezoidal_highland_gain_point,
      enemy_trapezoidal_highland_gain_point: msg.enemy_trapezoidal_highland_gain_point,
      friendly_fly_ramp_front_gain_point: msg.friendly_fly_ramp_front_gain_point,
      friendly_fly_ramp_back_gain_point: msg.friendly_fly_ramp_back_gain_point,
      enemy_fly_ramp_front_gain_point: msg.enemy_fly_ramp_front_gain_point,
      enemy_fly_ramp_back_gain_point: msg.enemy_fly_ramp_back_gain_point,
      friendly_central_highland_lower_gain_point: msg.friendly_central_highland_lower_gain_point,
      friendly_central_highland_upper_gain_point: msg.friendly_central_highland_upper_gain_point,
      enemy_central_highland_lower_gain_point: msg.enemy_central_highland_lower_gain_point,
      enemy_central_highland_upper_gain_point: msg.enemy_central_highland_upper_gain_point,
      friendly_highway_lower_gain_point: msg.friendly_highway_lower_gain_point,
      friendly_highway_upper_gain_point: msg.friendly_highway_upper_gain_point,
      enemy_highway_lower_gain_point: msg.enemy_highway_lower_gain_point,
      enemy_highway_upper_gain_point: msg.enemy_highway_upper_gain_point,
      friendly_fortress_gain_point: msg.friendly_fortress_gain_point,
      friendly_outpost_gain_point: msg.friendly_outpost_gain_point,
      friendly_supply_zone_non_exchange: msg.friendly_supply_zone_non_exchange,
      friendly_supply_zone_exchange: msg.friendly_supply_zone_exchange,
      friendly_big_resource_island: msg.friendly_big_resource_island,
      enemy_big_resource_island: msg.enemy_big_resource_island,
      center_gain_point: msg.center_gain_point,
    }
  }
}


// Corresponds to pb_rm_interfaces__msg__RobotStatus
/// 机器人性能体系数据 (裁判系统串口协议 V1.7.0 0x0201)

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
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
    pub robot_pos: geometry_msgs::msg::Pose,

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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::RobotStatus::default())
  }
}

impl rosidl_runtime_rs::Message for RobotStatus {
  type RmwMsg = super::msg::rmw::RobotStatus;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        robot_id: msg.robot_id,
        robot_level: msg.robot_level,
        current_hp: msg.current_hp,
        maximum_hp: msg.maximum_hp,
        shooter_barrel_cooling_value: msg.shooter_barrel_cooling_value,
        shooter_barrel_heat_limit: msg.shooter_barrel_heat_limit,
        shooter_17mm_1_barrel_heat: msg.shooter_17mm_1_barrel_heat,
        robot_pos: geometry_msgs::msg::Pose::into_rmw_message(std::borrow::Cow::Owned(msg.robot_pos)).into_owned(),
        armor_id: msg.armor_id,
        hp_deduction_reason: msg.hp_deduction_reason,
        projectile_allowance_17mm: msg.projectile_allowance_17mm,
        remaining_gold_coin: msg.remaining_gold_coin,
        is_hp_deduced: msg.is_hp_deduced,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      robot_id: msg.robot_id,
      robot_level: msg.robot_level,
      current_hp: msg.current_hp,
      maximum_hp: msg.maximum_hp,
      shooter_barrel_cooling_value: msg.shooter_barrel_cooling_value,
      shooter_barrel_heat_limit: msg.shooter_barrel_heat_limit,
      shooter_17mm_1_barrel_heat: msg.shooter_17mm_1_barrel_heat,
        robot_pos: geometry_msgs::msg::Pose::into_rmw_message(std::borrow::Cow::Borrowed(&msg.robot_pos)).into_owned(),
      armor_id: msg.armor_id,
      hp_deduction_reason: msg.hp_deduction_reason,
      projectile_allowance_17mm: msg.projectile_allowance_17mm,
      remaining_gold_coin: msg.remaining_gold_coin,
      is_hp_deduced: msg.is_hp_deduced,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      robot_id: msg.robot_id,
      robot_level: msg.robot_level,
      current_hp: msg.current_hp,
      maximum_hp: msg.maximum_hp,
      shooter_barrel_cooling_value: msg.shooter_barrel_cooling_value,
      shooter_barrel_heat_limit: msg.shooter_barrel_heat_limit,
      shooter_17mm_1_barrel_heat: msg.shooter_17mm_1_barrel_heat,
      robot_pos: geometry_msgs::msg::Pose::from_rmw_message(msg.robot_pos),
      armor_id: msg.armor_id,
      hp_deduction_reason: msg.hp_deduction_reason,
      projectile_allowance_17mm: msg.projectile_allowance_17mm,
      remaining_gold_coin: msg.remaining_gold_coin,
      is_hp_deduced: msg.is_hp_deduced,
    }
  }
}


// Corresponds to pb_rm_interfaces__msg__Gimbal
/// msg for Gimbal, pitch and yaw.

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::Gimbal::default())
  }
}

impl rosidl_runtime_rs::Message for Gimbal {
  type RmwMsg = super::msg::rmw::Gimbal;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        pitch: msg.pitch,
        yaw: msg.yaw,
        pitch_min_range: msg.pitch_min_range,
        pitch_max_range: msg.pitch_max_range,
        yaw_min_range: msg.yaw_min_range,
        yaw_max_range: msg.yaw_max_range,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      pitch: msg.pitch,
      yaw: msg.yaw,
      pitch_min_range: msg.pitch_min_range,
      pitch_max_range: msg.pitch_max_range,
      yaw_min_range: msg.yaw_min_range,
      yaw_max_range: msg.yaw_max_range,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      pitch: msg.pitch,
      yaw: msg.yaw,
      pitch_min_range: msg.pitch_min_range,
      pitch_max_range: msg.pitch_max_range,
      yaw_min_range: msg.yaw_min_range,
      yaw_max_range: msg.yaw_max_range,
    }
  }
}


// Corresponds to pb_rm_interfaces__msg__GimbalCmd

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct GimbalCmd {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::Header,

    /// control type
    pub yaw_type: u8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub pitch_type: u8,

    /// control dada
    pub position: super::msg::Gimbal,


    // This member is not documented.
    #[allow(missing_docs)]
    pub velocity: super::msg::Gimbal,

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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::GimbalCmd::default())
  }
}

impl rosidl_runtime_rs::Message for GimbalCmd {
  type RmwMsg = super::msg::rmw::GimbalCmd;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Owned(msg.header)).into_owned(),
        yaw_type: msg.yaw_type,
        pitch_type: msg.pitch_type,
        position: super::msg::Gimbal::into_rmw_message(std::borrow::Cow::Owned(msg.position)).into_owned(),
        velocity: super::msg::Gimbal::into_rmw_message(std::borrow::Cow::Owned(msg.velocity)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Borrowed(&msg.header)).into_owned(),
      yaw_type: msg.yaw_type,
      pitch_type: msg.pitch_type,
        position: super::msg::Gimbal::into_rmw_message(std::borrow::Cow::Borrowed(&msg.position)).into_owned(),
        velocity: super::msg::Gimbal::into_rmw_message(std::borrow::Cow::Borrowed(&msg.velocity)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      header: std_msgs::msg::Header::from_rmw_message(msg.header),
      yaw_type: msg.yaw_type,
      pitch_type: msg.pitch_type,
      position: super::msg::Gimbal::from_rmw_message(msg.position),
      velocity: super::msg::Gimbal::from_rmw_message(msg.velocity),
    }
  }
}


// Corresponds to pb_rm_interfaces__msg__Models

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Models {

    // This member is not documented.
    #[allow(missing_docs)]
    pub chassis: std::string::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub gimbal: std::string::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub shoot: std::string::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub arm: std::string::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub custom_controller: std::string::String,

}



impl Default for Models {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::Models::default())
  }
}

impl rosidl_runtime_rs::Message for Models {
  type RmwMsg = super::msg::rmw::Models;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        chassis: msg.chassis.as_str().into(),
        gimbal: msg.gimbal.as_str().into(),
        shoot: msg.shoot.as_str().into(),
        arm: msg.arm.as_str().into(),
        custom_controller: msg.custom_controller.as_str().into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        chassis: msg.chassis.as_str().into(),
        gimbal: msg.gimbal.as_str().into(),
        shoot: msg.shoot.as_str().into(),
        arm: msg.arm.as_str().into(),
        custom_controller: msg.custom_controller.as_str().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      chassis: msg.chassis.to_string(),
      gimbal: msg.gimbal.to_string(),
      shoot: msg.shoot.to_string(),
      arm: msg.arm.to_string(),
      custom_controller: msg.custom_controller.to_string(),
    }
  }
}


// Corresponds to pb_rm_interfaces__msg__RobotStateInfo

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct RobotStateInfo {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::Header,


    // This member is not documented.
    #[allow(missing_docs)]
    pub models: super::msg::Models,

}



impl Default for RobotStateInfo {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::RobotStateInfo::default())
  }
}

impl rosidl_runtime_rs::Message for RobotStateInfo {
  type RmwMsg = super::msg::rmw::RobotStateInfo;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Owned(msg.header)).into_owned(),
        models: super::msg::Models::into_rmw_message(std::borrow::Cow::Owned(msg.models)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Borrowed(&msg.header)).into_owned(),
        models: super::msg::Models::into_rmw_message(std::borrow::Cow::Borrowed(&msg.models)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      header: std_msgs::msg::Header::from_rmw_message(msg.header),
      models: super::msg::Models::from_rmw_message(msg.models),
    }
  }
}


