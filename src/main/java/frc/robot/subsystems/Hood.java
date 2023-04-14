package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.util.Conversions;
import frc.robot.util.HSFalconBuilder;
import frc.robot.util.InterpolatingTreeMap;
import frc.robot.util.MotorPositionSystem;
import frc.robot.util.MotorPositionSystem.MotorPositionSystemBuilder;
import harkerrobolib.wrappers.HSFalcon;

public class Hood extends SubsystemBase {
  private static Hood instance;

  private HSFalcon hood;

  private DigitalInput limitSwitch;

  private static final boolean INVERT = true;

  private static final double CURRENT_CONTINUOUS = 10;
  private static final double CURRENT_PEAK = 10;
  private static final double CURRENT_PEAK_DUR = 0.05;

  private static final double STALLING_CURRENT = 12;
  private static final double RANGE = 32.25;

  private static final double kS = 0.1; // .32;
  private static final double kV = 0.05998;
  private static final double kA = 0.00523;
  private static final double kG = 0.2;

  private static final double GEAR_RATIO = 180;
  private static final double FALCON_TO_DEG = Conversions.ENCODER_TO_DEG / GEAR_RATIO;

  private static final double MAX_POS_ERROR = 0.3;
  private static final double MAX_VEL_ERROR = 0.2;
  private static final double MAX_VOLTAGE = 10;

  private InterpolatingTreeMap hoodVals;

  private boolean isHoodZeroed;

  private MotorPositionSystem positionSystem;

  private Hood() {
    hood =
        new HSFalconBuilder()
            .statorLimit(CURRENT_PEAK, CURRENT_CONTINUOUS, CURRENT_PEAK_DUR)
            .invert(INVERT)
            .build(RobotMap.HOOD_ID, RobotMap.CANBUS);
    addChild("Hood Motor", hood);
    positionSystem =
        new MotorPositionSystemBuilder()
            .constants(kV, kA, kS)
            .elevatorGravityConstant(kG)
            .unitConversionFactor(FALCON_TO_DEG)
            .maxVoltage(MAX_VOLTAGE)
            .maxError(MAX_POS_ERROR, MAX_VEL_ERROR)
            .build(hood)
            .init();
    addChild("Hood Position System", positionSystem);
    isHoodZeroed = false;
    initMotors();
  }

  public void initMotors() {
    // hood.configForwardSoftLimitEnable(true);
    // hood.configForwardSoftLimitThreshold(5.0);
  }

  public void setHoodPosition(double position) {
    positionSystem.set(position);
  }

  public void setHoodPercentOutput(double percentOutput) {
    hood.set(ControlMode.PercentOutput, percentOutput);
  }

  public void setHoodEncoderZero() {
    hood.setSelectedSensorPosition(0);
    isHoodZeroed = true;
  }

  public double getHoodPosition() {
    return positionSystem.getPosition();
  }

  public double getHoodVelocity() {
    return positionSystem.getVelocity();
  }

  public double calculateHoodPosition() {
    return 20;
  }

  public boolean isHoodStalling() {
    return hood.getStatorCurrent() > STALLING_CURRENT;
  }

  public boolean isHoodZeroed() {
    return isHoodZeroed;
  }

  public static Hood getInstance() {
    if (instance == null) instance = new Hood();
    return instance;
  }

  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Hood");
    builder.addBooleanProperty("Is Zeroed", () -> isHoodZeroed, null);
    builder.addBooleanProperty("Limit Switch Hit", () -> !limitSwitch.get(), null);
  }
}
