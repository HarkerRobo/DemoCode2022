package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.util.HSFalconBuilder;
import frc.robot.util.MotorPositionSystem;
import frc.robot.util.MotorPositionSystem.MotorPositionSystemBuilder;
import harkerrobolib.wrappers.HSFalcon;

public class Climber extends SubsystemBase {
  private static Climber instance;

  private HSFalcon left;
  private HSFalcon right;
  private DoubleSolenoid climber;
  private DigitalInput rightLimitSwitch;
  private DigitalInput leftLimitSwitch;

  private static final double CURRENT_CONTINUOUS = 40;
  private static final double CURRENT_PEAK = 45;
  private static final double CURRENT_PEAK_DUR = 0.5;

  private static final boolean RIGHT_INVERT = false;

  private static final boolean LEFT_INVERT = true;

  private static final double kS = 0.126;
  private static final double kV = 0.165;
  private static final double kA = 0.019;
  private static final double kG = -0.35;
  private static final double POS_MAX_ERROR = 15.0;
  private static final double VEL_MAX_ERROR = 5.0;
  private static final double unitConversion = 1.0 / 2048.0;
  private static final double MAX_CONTROL_EFFORT = 10.0;

  public static final double UP_AND_BACK_HEIGHT = 118000 / 2048.0;
  public static final double UP_HEIGHT = 112500 / 2048.0;
  public static final double MID_HEIGHT = 48000 / 2048.0;
  public static final double DOWN_HEIGHT = 0.1;

  private MotorPositionSystem leftPositionSys;
  private MotorPositionSystem rightPositionSys;

  private Climber() {
    right =
        new HSFalconBuilder()
            .invert(RIGHT_INVERT)
            .voltageComp(MAX_CONTROL_EFFORT)
            // .supplyLimit(CURRENT_PEAK, CURRENT_CONTINUOUS, CURRENT_PEAK_DUR)
            .build(RobotMap.RIGHT_CLIMBER, RobotMap.CANBUS);
    addChild("Right Motor", right);
    left =
        new HSFalconBuilder()
            .invert(LEFT_INVERT)
            // .supplyLimit(CURRENT_PEAK, CURRENT_CONTINUOUS, CURRENT_PEAK_DUR)
            .voltageComp(MAX_CONTROL_EFFORT)
            .build(RobotMap.LEFT_CLIMBER, RobotMap.CANBUS);
    addChild("Left Motor", left);
    climber =
        new DoubleSolenoid(
            PneumaticsModuleType.REVPH, RobotMap.CLIMBER_FORWARD, RobotMap.CLIMBER_BACKWARD);
    rightPositionSys =
        new MotorPositionSystemBuilder()
            .maxError(POS_MAX_ERROR, VEL_MAX_ERROR)
            .elevatorGravityConstant(kG)
            .constants(kV, kA, kS)
            .unitConversionFactor(unitConversion)
            .build(right)
            .init();
    addChild("Right Position System", rightPositionSys);
    leftPositionSys =
        new MotorPositionSystemBuilder()
            .maxError(POS_MAX_ERROR, VEL_MAX_ERROR)
            .elevatorGravityConstant(kG)
            .constants(kV, kA, kS)
            .unitConversionFactor(unitConversion)
            .build(left)
            .init();
    addChild("Left Position System", leftPositionSys);
    rightLimitSwitch = new DigitalInput(RobotMap.CLIMBER_RIGHT_LIMIT_SWTICH);
    leftLimitSwitch = new DigitalInput(RobotMap.CLIMBER_LEFT_LIMIT_SWITCH);
  }

  public boolean leftLimitSwitch() {
    return !leftLimitSwitch.get();
  }

  public boolean rightLimitSwitch() {
    return !rightLimitSwitch.get();
  }

  public boolean limitSwitchHit() {
    return !rightLimitSwitch.get() && !leftLimitSwitch.get();
  }

  public void setRightClimberPos(double pos) {
    rightPositionSys.set(pos);
  }

  public void setLeftClimberPos(double pos) {
    leftPositionSys.set(pos);
  }

  public void setClimberForward() {
    climber.set(DoubleSolenoid.Value.kReverse);
  }

  public void setClimberBackward() {
    climber.set(DoubleSolenoid.Value.kForward);
  }

  public void turnOffMotors() {
    right.set(ControlMode.PercentOutput, 0);
    left.set(ControlMode.PercentOutput, 0);
  }

  public double getRightClimberPos() {
    return right.getSelectedSensorPosition();
  }

  public double getLeftClimberPos() {
    return left.getSelectedSensorPosition();
  }

  public double getRightClimberVel() {
    return right.getSelectedSensorVelocity();
  }

  public double getLeftClimberVel() {
    return left.getSelectedSensorVelocity();
  }

  public void setLeftPercentOutput(double output) {
    left.set(ControlMode.PercentOutput, output);
  }

  public void setRightPercentOutput(double output) {
    right.set(ControlMode.PercentOutput, output);
  }

  public HSFalcon getRightClimber() {
    return right;
  }

  public HSFalcon getLeftClimber() {
    return left;
  }

  public MotorPositionSystem getRightPositionSystem() {
    return rightPositionSys;
  }

  public MotorPositionSystem getLeftPositionSystem() {
    return leftPositionSys;
  }

  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Climber");
    builder.addStringProperty("Solenoid Value", () -> climber.get().name(), null);
    builder.addBooleanProperty("Left Limit Switch Hit", () -> !leftLimitSwitch.get(), null);
    builder.addBooleanProperty("Right Limit Switch Hit", () -> !rightLimitSwitch.get(), null);
  }

  public static Climber getInstance() {
    if (instance == null) {
      instance = new Climber();
    }
    return instance;
  }

  public void setBothClimberPos(double position) {
    setLeftClimberPos(position);
    setRightClimberPos(position);
  }
}
