package frc.robot.util;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import frc.robot.RobotMap;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.MotorPositionSystem.MotorPositionSystemBuilder;
import frc.robot.util.MotorVelocitySystem.MotorVelocitySystemBuilder;
import harkerrobolib.wrappers.HSFalcon;

public class SwerveModule implements Sendable {
  private HSFalcon rotation;
  private HSFalcon drive;

  private CANCoder canCoder;

  private int swerveID;

  private MotorPositionSystem rotationSystem;
  private MotorVelocitySystem driveSystem;

  private static final double ROTATION_MOTOR_CURRENT_CONTINUOUS = 25;
  private static final double ROTATION_MOTOR_CURRENT_PEAK = 40;
  private static final double ROTATION_MOTOR_CURRENT_PEAK_DUR = 0.1;

  private static final double DRIVE_MOTOR_CURRENT_CONTINUOUS = 40;
  private static final double DRIVE_MOTOR_CURRENT_PEAK = 80;
  private static final double DRIVE_MOTOR_CURRENT_PEAK_DUR = 0.1;

  private static final double DRIVE_kS = 0.3;
  private static final double DRIVE_kV = 2.2819; // 2.2819
  private static final double DRIVE_kA = 0.3621;
  private static final double DRIVE_kD = 1.3;

  private static final double ROTATION_kS = 0.40104;
  private static final double ROTATION_kV = 0.0057859;
  private static final double ROTATION_kA = 0.00016558;

  private static final double DRIVE_MAX_ERROR = 2;

  private static final double ROTATION_MAX_VEL_ERROR = 0.13;
  private static final double ROTATION_MAX_POS_ERROR = 0.1;

  private static final double WHEEL_DIAMETER = 4.0;
  private static final double ROTATION_GEAR_RATIO = 12.8;
  private static final double DRIVE_GEAR_RATIO = 6.75;
  private static final double DRIVE_FALCON_TO_MPS =
      Conversions.ENCODER_TO_WHEEL_SPEED / DRIVE_GEAR_RATIO * WHEEL_DIAMETER;
  private static final double ROT_MOTOR_TO_DEG = Conversions.ENCODER_TO_DEG / ROTATION_GEAR_RATIO;

  public SwerveModule(int swerveID) {
    this.swerveID = swerveID;
    rotation =
        new HSFalconBuilder()
            .invert(Drivetrain.ROTATION_INVERTS[swerveID])
            .supplyLimit(
                ROTATION_MOTOR_CURRENT_PEAK,
                ROTATION_MOTOR_CURRENT_CONTINUOUS,
                ROTATION_MOTOR_CURRENT_PEAK_DUR)
            .velocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_100Ms)
            .build(RobotMap.ROTATION_IDS[swerveID], RobotMap.CANBUS);
    SendableRegistry.addLW(
        rotation, "Drivetrain/" + swerveIDToName(swerveID) + " Module", "Rotation Motor");
    drive =
        new HSFalconBuilder()
            .invert(Drivetrain.DRIVE_INVERTS[swerveID])
            .supplyLimit(
                DRIVE_MOTOR_CURRENT_PEAK,
                DRIVE_MOTOR_CURRENT_CONTINUOUS,
                DRIVE_MOTOR_CURRENT_PEAK_DUR)
            .build(RobotMap.TRANSLATION_IDS[swerveID], RobotMap.CANBUS);
    SendableRegistry.addLW(
        drive, "Drivetrain/" + swerveIDToName(swerveID) + " Module", "Drive Motor");
    canCoder = new CANCoder(RobotMap.CANCODER_IDS[swerveID], RobotMap.CANBUS);
    driveSystem =
        new MotorVelocitySystemBuilder()
            .constants(DRIVE_kV, DRIVE_kA, DRIVE_kS, DRIVE_kD)
            .unitConversionFactor(DRIVE_FALCON_TO_MPS)
            .maxError(DRIVE_MAX_ERROR)
            .build(drive)
            .init();
    rotationSystem =
        new MotorPositionSystemBuilder()
            .constants(ROTATION_kV, ROTATION_kA, ROTATION_kS)
            .unitConversionFactor(ROT_MOTOR_TO_DEG)
            .maxError(ROTATION_MAX_POS_ERROR, ROTATION_MAX_VEL_ERROR)
            .build(rotation)
            .init();
    SendableRegistry.addLW(
        rotationSystem, "Drivetrain/" + swerveIDToName(swerveID) + " Module", "Rotation System");
    SendableRegistry.addLW(
        driveSystem, "Drivetrain/" + swerveIDToName(swerveID) + " Module", "Drive System");
  }

  public void initLiveWindow() {}

  public void setAngleAndDrive(
      double rotationAngle, double driveOutput, boolean drivePercentOutput) {
    rotationSystem.set(rotationAngle);
    if (drivePercentOutput) {
      drive.set(ControlMode.PercentOutput, driveOutput / Drivetrain.MAX_TRANSLATION_VEL);
    } else {
      driveSystem.set(driveOutput);
    }
  }

  public void zeroDriveEncoders() {
    drive.setSelectedSensorPosition(0);
  }

  public void setAngleAndDrive(double rotationAngle, double driveOutput) {
    setAngleAndDrive(rotationAngle, driveOutput, false);
  }

  public double getCurrentAngle() {
    return rotationSystem.getPosition();
  }

  public double getCurrentAngleVelocity() {
    return rotationSystem.getVelocity();
  }

  public Rotation2d getCurrentRotation() {
    return Rotation2d.fromDegrees(getCurrentAngle());
  }

  public double getCurrentSpeed() {
    return drive.getSelectedSensorVelocity() * DRIVE_FALCON_TO_MPS;
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(getCurrentSpeed(), getCurrentRotation());
  }

  public void setRotationOffset() {
    double position = canCoder.getAbsolutePosition() - Drivetrain.CANCODER_OFFSETS[swerveID];
    rotation.setSelectedSensorPosition(
        position / ROT_MOTOR_TO_DEG); // DEGREE.to(TALONFX, position * ROTATION_GEAR_RATIO));
    canCoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, RobotMap.MAX_CAN_FRAME_PERIOD);
  }

  public HSFalcon getDriveMotor() {
    return drive;
  }

  public HSFalcon getRotationMotor() {
    return rotation;
  }

  public CANCoder getCanCoder() {
    return canCoder;
  }

  public static String swerveIDToName(int swerveID) {
    String output = "";
    if (swerveID < 2) output += "Front ";
    else output += "Back ";
    if (swerveID % 2 == 0) output += "Left";
    else output += "Right";
    return output;
  }

  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("SwerveModule");
  }
}
