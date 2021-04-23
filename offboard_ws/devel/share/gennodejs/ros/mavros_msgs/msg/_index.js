
"use strict";

let Param = require('./Param.js');
let DebugValue = require('./DebugValue.js');
let LandingTarget = require('./LandingTarget.js');
let Vibration = require('./Vibration.js');
let ESCStatus = require('./ESCStatus.js');
let HilGPS = require('./HilGPS.js');
let ESCStatusItem = require('./ESCStatusItem.js');
let TimesyncStatus = require('./TimesyncStatus.js');
let WaypointList = require('./WaypointList.js');
let Waypoint = require('./Waypoint.js');
let WaypointReached = require('./WaypointReached.js');
let HilStateQuaternion = require('./HilStateQuaternion.js');
let BatteryStatus = require('./BatteryStatus.js');
let VFR_HUD = require('./VFR_HUD.js');
let EstimatorStatus = require('./EstimatorStatus.js');
let OpticalFlowRad = require('./OpticalFlowRad.js');
let GlobalPositionTarget = require('./GlobalPositionTarget.js');
let RTCM = require('./RTCM.js');
let WheelOdomStamped = require('./WheelOdomStamped.js');
let Thrust = require('./Thrust.js');
let HilSensor = require('./HilSensor.js');
let RCIn = require('./RCIn.js');
let LogEntry = require('./LogEntry.js');
let MountControl = require('./MountControl.js');
let PlayTuneV2 = require('./PlayTuneV2.js');
let OnboardComputerStatus = require('./OnboardComputerStatus.js');
let ESCInfoItem = require('./ESCInfoItem.js');
let Altitude = require('./Altitude.js');
let GPSRTK = require('./GPSRTK.js');
let CamIMUStamp = require('./CamIMUStamp.js');
let StatusText = require('./StatusText.js');
let LogData = require('./LogData.js');
let CommandCode = require('./CommandCode.js');
let State = require('./State.js');
let GPSRAW = require('./GPSRAW.js');
let RadioStatus = require('./RadioStatus.js');
let HilControls = require('./HilControls.js');
let OverrideRCIn = require('./OverrideRCIn.js');
let VehicleInfo = require('./VehicleInfo.js');
let HomePosition = require('./HomePosition.js');
let ExtendedState = require('./ExtendedState.js');
let RTKBaseline = require('./RTKBaseline.js');
let FileEntry = require('./FileEntry.js');
let Trajectory = require('./Trajectory.js');
let ParamValue = require('./ParamValue.js');
let Mavlink = require('./Mavlink.js');
let CompanionProcessStatus = require('./CompanionProcessStatus.js');
let ActuatorControl = require('./ActuatorControl.js');
let ManualControl = require('./ManualControl.js');
let ESCInfo = require('./ESCInfo.js');
let RCOut = require('./RCOut.js');
let ADSBVehicle = require('./ADSBVehicle.js');
let PositionTarget = require('./PositionTarget.js');
let HilActuatorControls = require('./HilActuatorControls.js');
let AttitudeTarget = require('./AttitudeTarget.js');

module.exports = {
  Param: Param,
  DebugValue: DebugValue,
  LandingTarget: LandingTarget,
  Vibration: Vibration,
  ESCStatus: ESCStatus,
  HilGPS: HilGPS,
  ESCStatusItem: ESCStatusItem,
  TimesyncStatus: TimesyncStatus,
  WaypointList: WaypointList,
  Waypoint: Waypoint,
  WaypointReached: WaypointReached,
  HilStateQuaternion: HilStateQuaternion,
  BatteryStatus: BatteryStatus,
  VFR_HUD: VFR_HUD,
  EstimatorStatus: EstimatorStatus,
  OpticalFlowRad: OpticalFlowRad,
  GlobalPositionTarget: GlobalPositionTarget,
  RTCM: RTCM,
  WheelOdomStamped: WheelOdomStamped,
  Thrust: Thrust,
  HilSensor: HilSensor,
  RCIn: RCIn,
  LogEntry: LogEntry,
  MountControl: MountControl,
  PlayTuneV2: PlayTuneV2,
  OnboardComputerStatus: OnboardComputerStatus,
  ESCInfoItem: ESCInfoItem,
  Altitude: Altitude,
  GPSRTK: GPSRTK,
  CamIMUStamp: CamIMUStamp,
  StatusText: StatusText,
  LogData: LogData,
  CommandCode: CommandCode,
  State: State,
  GPSRAW: GPSRAW,
  RadioStatus: RadioStatus,
  HilControls: HilControls,
  OverrideRCIn: OverrideRCIn,
  VehicleInfo: VehicleInfo,
  HomePosition: HomePosition,
  ExtendedState: ExtendedState,
  RTKBaseline: RTKBaseline,
  FileEntry: FileEntry,
  Trajectory: Trajectory,
  ParamValue: ParamValue,
  Mavlink: Mavlink,
  CompanionProcessStatus: CompanionProcessStatus,
  ActuatorControl: ActuatorControl,
  ManualControl: ManualControl,
  ESCInfo: ESCInfo,
  RCOut: RCOut,
  ADSBVehicle: ADSBVehicle,
  PositionTarget: PositionTarget,
  HilActuatorControls: HilActuatorControls,
  AttitudeTarget: AttitudeTarget,
};
