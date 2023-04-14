// package frc.robot.util;

// import com.ctre.phoenix.motorcontrol.StatusFrame;

// import frc.robot.RobotMap;
// import frc.robot.subsystems.Climber;
// import harkerrobolib.wrappers.HSFalcon;

// public class CANFrameManager {

//     private static void deactivateMotor(HSFalcon falcon) {
//         for (StatusFrame frame : StatusFrame.values())
//             falcon.setStatusFramePeriod(frame, RobotMap.MAX_CAN_FRAME_PERIOD);
//     }

//     private static void activateMotor(HSFalcon falcon) {
//         deactivateMotor(falcon);
//         falcon.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, fastCANFrame);
//         falcon.setStatusFramePeriod(StatusFrame.Status_4_AinTempVbat, slowCANFrame);
//     }
// }
