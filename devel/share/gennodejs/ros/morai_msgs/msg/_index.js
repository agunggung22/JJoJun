
"use strict";

let Obstacle = require('./Obstacle.js');
let SyncModeSetGear = require('./SyncModeSetGear.js');
let FaultInjection_Tire = require('./FaultInjection_Tire.js');
let RobotState = require('./RobotState.js');
let CollisionData = require('./CollisionData.js');
let MoraiSimProcHandle = require('./MoraiSimProcHandle.js');
let DillyCmd = require('./DillyCmd.js');
let ManipulatorControl = require('./ManipulatorControl.js');
let IntersectionStatus = require('./IntersectionStatus.js');
let PRCtrlCmd = require('./PRCtrlCmd.js');
let Obstacles = require('./Obstacles.js');
let FaultInjection_Controller = require('./FaultInjection_Controller.js');
let Transforms = require('./Transforms.js');
let UGVServeSkidCtrlCmd = require('./UGVServeSkidCtrlCmd.js');
let MoraiSrvResponse = require('./MoraiSrvResponse.js');
let SaveSensorData = require('./SaveSensorData.js');
let ReplayInfo = require('./ReplayInfo.js');
let GhostMessage = require('./GhostMessage.js');
let SkidSteer6wUGVStatus = require('./SkidSteer6wUGVStatus.js');
let VelocityCmd = require('./VelocityCmd.js');
let ScenarioLoad = require('./ScenarioLoad.js');
let TOF = require('./TOF.js');
let SyncModeCmdResponse = require('./SyncModeCmdResponse.js');
let SyncModeInfo = require('./SyncModeInfo.js');
let IntscnTL = require('./IntscnTL.js');
let DdCtrlCmd = require('./DdCtrlCmd.js');
let SensorPosControl = require('./SensorPosControl.js');
let SVADC = require('./SVADC.js');
let ObjectStatusList = require('./ObjectStatusList.js');
let SetTrafficLight = require('./SetTrafficLight.js');
let NpcGhostInfo = require('./NpcGhostInfo.js');
let FaultStatusInfo_Overall = require('./FaultStatusInfo_Overall.js');
let CMDConveyor = require('./CMDConveyor.js');
let SyncModeAddObject = require('./SyncModeAddObject.js');
let GVDirectCmd = require('./GVDirectCmd.js');
let ShipCtrlCmd = require('./ShipCtrlCmd.js');
let WheelControl = require('./WheelControl.js');
let Conveyor = require('./Conveyor.js');
let MultiPlayEventResponse = require('./MultiPlayEventResponse.js');
let FaultStatusInfo_Sensor = require('./FaultStatusInfo_Sensor.js');
let GetTrafficLightStatus = require('./GetTrafficLightStatus.js');
let SyncModeRemoveObject = require('./SyncModeRemoveObject.js');
let ObjectStatus = require('./ObjectStatus.js');
let RobotOutput = require('./RobotOutput.js');
let ObjectStatusExtended = require('./ObjectStatusExtended.js');
let SyncModeCmd = require('./SyncModeCmd.js');
let FaultStatusInfo = require('./FaultStatusInfo.js');
let GVStateCmd = require('./GVStateCmd.js');
let SkidSteer6wUGVCtrlCmd = require('./SkidSteer6wUGVCtrlCmd.js');
let WoowaDillyStatus = require('./WoowaDillyStatus.js');
let PREvent = require('./PREvent.js');
let Lamps = require('./Lamps.js');
let IntersectionControl = require('./IntersectionControl.js');
let RadarDetection = require('./RadarDetection.js');
let ObjectStatusListExtended = require('./ObjectStatusListExtended.js');
let GeoVector3Message = require('./GeoVector3Message.js');
let EgoDdVehicleStatus = require('./EgoDdVehicleStatus.js');
let CtrlCmd = require('./CtrlCmd.js');
let EgoVehicleStatus = require('./EgoVehicleStatus.js');
let MoraiTLInfo = require('./MoraiTLInfo.js');
let SyncModeCtrlCmd = require('./SyncModeCtrlCmd.js');
let VehicleSpecIndex = require('./VehicleSpecIndex.js');
let VehicleSpec = require('./VehicleSpec.js');
let NpcGhostCmd = require('./NpcGhostCmd.js');
let PRStatus = require('./PRStatus.js');
let SkateboardCtrlCmd = require('./SkateboardCtrlCmd.js');
let ShipState = require('./ShipState.js');
let SyncModeResultResponse = require('./SyncModeResultResponse.js');
let EventInfo = require('./EventInfo.js');
let MoraiSimProcStatus = require('./MoraiSimProcStatus.js');
let FaultInjection_Response = require('./FaultInjection_Response.js');
let MultiEgoSetting = require('./MultiEgoSetting.js');
let MoraiTLIndex = require('./MoraiTLIndex.js');
let ExternalForce = require('./ExternalForce.js');
let MapSpec = require('./MapSpec.js');
let SkateboardStatus = require('./SkateboardStatus.js');
let MultiPlayEventRequest = require('./MultiPlayEventRequest.js');
let TrafficLight = require('./TrafficLight.js');
let FaultInjection_Sensor = require('./FaultInjection_Sensor.js');
let DillyCmdResponse = require('./DillyCmdResponse.js');
let SyncModeScenarioLoad = require('./SyncModeScenarioLoad.js');
let RadarDetections = require('./RadarDetections.js');
let WaitForTick = require('./WaitForTick.js');
let VehicleCollision = require('./VehicleCollision.js');
let ERP42Info = require('./ERP42Info.js');
let VehicleCollisionData = require('./VehicleCollisionData.js');
let GPSMessage = require('./GPSMessage.js');
let MapSpecIndex = require('./MapSpecIndex.js');
let FaultStatusInfo_Vehicle = require('./FaultStatusInfo_Vehicle.js');
let EgoVehicleStatusExtended = require('./EgoVehicleStatusExtended.js');
let WaitForTickResponse = require('./WaitForTickResponse.js');

module.exports = {
  Obstacle: Obstacle,
  SyncModeSetGear: SyncModeSetGear,
  FaultInjection_Tire: FaultInjection_Tire,
  RobotState: RobotState,
  CollisionData: CollisionData,
  MoraiSimProcHandle: MoraiSimProcHandle,
  DillyCmd: DillyCmd,
  ManipulatorControl: ManipulatorControl,
  IntersectionStatus: IntersectionStatus,
  PRCtrlCmd: PRCtrlCmd,
  Obstacles: Obstacles,
  FaultInjection_Controller: FaultInjection_Controller,
  Transforms: Transforms,
  UGVServeSkidCtrlCmd: UGVServeSkidCtrlCmd,
  MoraiSrvResponse: MoraiSrvResponse,
  SaveSensorData: SaveSensorData,
  ReplayInfo: ReplayInfo,
  GhostMessage: GhostMessage,
  SkidSteer6wUGVStatus: SkidSteer6wUGVStatus,
  VelocityCmd: VelocityCmd,
  ScenarioLoad: ScenarioLoad,
  TOF: TOF,
  SyncModeCmdResponse: SyncModeCmdResponse,
  SyncModeInfo: SyncModeInfo,
  IntscnTL: IntscnTL,
  DdCtrlCmd: DdCtrlCmd,
  SensorPosControl: SensorPosControl,
  SVADC: SVADC,
  ObjectStatusList: ObjectStatusList,
  SetTrafficLight: SetTrafficLight,
  NpcGhostInfo: NpcGhostInfo,
  FaultStatusInfo_Overall: FaultStatusInfo_Overall,
  CMDConveyor: CMDConveyor,
  SyncModeAddObject: SyncModeAddObject,
  GVDirectCmd: GVDirectCmd,
  ShipCtrlCmd: ShipCtrlCmd,
  WheelControl: WheelControl,
  Conveyor: Conveyor,
  MultiPlayEventResponse: MultiPlayEventResponse,
  FaultStatusInfo_Sensor: FaultStatusInfo_Sensor,
  GetTrafficLightStatus: GetTrafficLightStatus,
  SyncModeRemoveObject: SyncModeRemoveObject,
  ObjectStatus: ObjectStatus,
  RobotOutput: RobotOutput,
  ObjectStatusExtended: ObjectStatusExtended,
  SyncModeCmd: SyncModeCmd,
  FaultStatusInfo: FaultStatusInfo,
  GVStateCmd: GVStateCmd,
  SkidSteer6wUGVCtrlCmd: SkidSteer6wUGVCtrlCmd,
  WoowaDillyStatus: WoowaDillyStatus,
  PREvent: PREvent,
  Lamps: Lamps,
  IntersectionControl: IntersectionControl,
  RadarDetection: RadarDetection,
  ObjectStatusListExtended: ObjectStatusListExtended,
  GeoVector3Message: GeoVector3Message,
  EgoDdVehicleStatus: EgoDdVehicleStatus,
  CtrlCmd: CtrlCmd,
  EgoVehicleStatus: EgoVehicleStatus,
  MoraiTLInfo: MoraiTLInfo,
  SyncModeCtrlCmd: SyncModeCtrlCmd,
  VehicleSpecIndex: VehicleSpecIndex,
  VehicleSpec: VehicleSpec,
  NpcGhostCmd: NpcGhostCmd,
  PRStatus: PRStatus,
  SkateboardCtrlCmd: SkateboardCtrlCmd,
  ShipState: ShipState,
  SyncModeResultResponse: SyncModeResultResponse,
  EventInfo: EventInfo,
  MoraiSimProcStatus: MoraiSimProcStatus,
  FaultInjection_Response: FaultInjection_Response,
  MultiEgoSetting: MultiEgoSetting,
  MoraiTLIndex: MoraiTLIndex,
  ExternalForce: ExternalForce,
  MapSpec: MapSpec,
  SkateboardStatus: SkateboardStatus,
  MultiPlayEventRequest: MultiPlayEventRequest,
  TrafficLight: TrafficLight,
  FaultInjection_Sensor: FaultInjection_Sensor,
  DillyCmdResponse: DillyCmdResponse,
  SyncModeScenarioLoad: SyncModeScenarioLoad,
  RadarDetections: RadarDetections,
  WaitForTick: WaitForTick,
  VehicleCollision: VehicleCollision,
  ERP42Info: ERP42Info,
  VehicleCollisionData: VehicleCollisionData,
  GPSMessage: GPSMessage,
  MapSpecIndex: MapSpecIndex,
  FaultStatusInfo_Vehicle: FaultStatusInfo_Vehicle,
  EgoVehicleStatusExtended: EgoVehicleStatusExtended,
  WaitForTickResponse: WaitForTickResponse,
};
