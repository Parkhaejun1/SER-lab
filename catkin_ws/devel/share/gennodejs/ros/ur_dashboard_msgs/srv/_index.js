
"use strict";

let IsProgramRunning = require('./IsProgramRunning.js')
let AddToLog = require('./AddToLog.js')
let Popup = require('./Popup.js')
let GetSafetyMode = require('./GetSafetyMode.js')
let GetLoadedProgram = require('./GetLoadedProgram.js')
let IsInRemoteControl = require('./IsInRemoteControl.js')
let GetRobotMode = require('./GetRobotMode.js')
let GetProgramState = require('./GetProgramState.js')
let IsProgramSaved = require('./IsProgramSaved.js')
let Load = require('./Load.js')
let RawRequest = require('./RawRequest.js')

module.exports = {
  IsProgramRunning: IsProgramRunning,
  AddToLog: AddToLog,
  Popup: Popup,
  GetSafetyMode: GetSafetyMode,
  GetLoadedProgram: GetLoadedProgram,
  IsInRemoteControl: IsInRemoteControl,
  GetRobotMode: GetRobotMode,
  GetProgramState: GetProgramState,
  IsProgramSaved: IsProgramSaved,
  Load: Load,
  RawRequest: RawRequest,
};
