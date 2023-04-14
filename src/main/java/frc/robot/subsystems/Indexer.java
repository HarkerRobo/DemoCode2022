package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.util.Conversions;
import frc.robot.util.HSFalconBuilder;
import frc.robot.util.MotorVelocitySystem;
import frc.robot.util.MotorVelocitySystem.MotorVelocitySystemBuilder;
import harkerrobolib.wrappers.HSFalcon;

public class Indexer extends SubsystemBase {
  private static Indexer instance;
  private HSFalcon top;
  private HSFalcon bottom;
  private DigitalInput topProximity;
  private DigitalInput bottomProximity;
  // private ColorSensor colorSensor;

  private static final double CURRENT_CONTINUOUS = 30;
  private static final double CURRENT_PEAK = 60;
  private static final double CURRENT_PEAK_DUR = 0.2;
  private static final boolean TOP_INVERT = false;
  private static final boolean BOTTOM_INVERT = false;

  private static final double TOP_GEAR_RATIO = 5.0;
  private static final double BOTTOM_GEAR_RATIO = 1.5;
  private static final double TOP_WHEEL_DIAMETER = 4.0;
  private static final double BOTTOM_WHEEL_DIAMETER = 3.0;
  private static final double TOP_FALCON_TO_CARGO_SPEED =
      Conversions.ENCODER_TO_WHEEL_SPEED / TOP_GEAR_RATIO * TOP_WHEEL_DIAMETER;
  private static final double BOTTOM_FALCON_TO_CARGO_SPEED =
      Conversions.ENCODER_TO_WHEEL_SPEED / BOTTOM_GEAR_RATIO * BOTTOM_WHEEL_DIAMETER;

  private static final double TOP_kS = 0.062268;
  private static final double TOP_kV = 2.5235;
  private static final double TOP_kA = 0.034295;
  private static final double BOTTOM_kS = 0.017587;
  private static final double BOTTOM_kV = 1.014;
  private static final double BOTTOM_kA = 0.082487;

  private static final double MAX_BOTTOM_ERROR = 21;
  private static final double MAX_TOP_ERROR = 21;

  private static MotorVelocitySystem topSystem;
  private static MotorVelocitySystem bottomSystem;

  private Debouncer indexerBalls;

  private Indexer() {
    top =
        new HSFalconBuilder()
            .invert(TOP_INVERT)
            .statorLimit(CURRENT_PEAK, CURRENT_CONTINUOUS, CURRENT_PEAK_DUR)
            .build(RobotMap.INDEXER_TOP, RobotMap.CANBUS);
    addChild("Top Motor", top);
    bottom =
        new HSFalconBuilder()
            .invert(BOTTOM_INVERT)
            .statorLimit(CURRENT_PEAK, CURRENT_CONTINUOUS, CURRENT_PEAK_DUR)
            .build(RobotMap.INDEXER_BOTTOM, RobotMap.CANBUS);
    addChild("Bottom Motor", bottom);
    // topSystem =
    //     new MotorVelocitySystemBuilder()
    //         .constants(TOP_kV, TOP_kA, TOP_kS)
    //         .maxError(MAX_TOP_ERROR)
    //         .unitConversionFactor(TOP_FALCON_TO_CARGO_SPEED)
    //         .build(top)
    //         .init();
    // bottomSystem =
    //     new MotorVelocitySystemBuilder()
    //         .constants(BOTTOM_kV, BOTTOM_kA, BOTTOM_kS)
    //         .maxError(MAX_BOTTOM_ERROR)
    //         .unitConversionFactor(BOTTOM_FALCON_TO_CARGO_SPEED)
    //         .build(bottom)
    //         .init();
    // addChild("Top System", topSystem);
    // addChild("Bottom System", bottomSystem);
    topProximity = new DigitalInput(RobotMap.TOP_PROXIMITY);
    bottomProximity = new DigitalInput(RobotMap.BOTTOM_PROXIMITY);
    indexerBalls = new Debouncer(0.25, DebounceType.kRising);
    // colorSensor = new ColorSensor(RobotMap.COLOR_A, RobotMap.COLOR_B, RobotMap.COLOR_PROXIMITY);
    // colorSensor.setColor(DriverStation.getAlliance().equals(Alliance.Red) ? true : false);
  }

  public void setTopOutput(double topOutput) {
    top.set(ControlMode.PercentOutput, topOutput);
    // topSystem.set(topOutput);
  }

  public boolean isEmpty() {
    return indexerBalls.calculate(!isBallInTop() && !isBallInBottom());
  }

  public void setBottomOutput(double bottomOutput) {
    bottom.set(ControlMode.PercentOutput, bottomOutput);
    // bottomSystem.set(bottomOutput);
  }

  public void setBothOutput(double output) {
    setTopOutput(output);
    setBottomOutput(output);
  }

  public void turnOffMotors() {
    top.set(ControlMode.PercentOutput, 0);
    bottom.set(ControlMode.PercentOutput, 0);
  }

  public boolean isBallInTop() {
    return !topProximity.get();
  }

  public boolean isBallInBottom() {
    return !bottomProximity.get();
  }

  public double getTopMPS() {
    return top.getSelectedSensorVelocity() * TOP_FALCON_TO_CARGO_SPEED; // change
  }

  public double getBottomMPS() {
    return bottom.getSelectedSensorVelocity() * BOTTOM_FALCON_TO_CARGO_SPEED; // change
  }

  public boolean isRightColor() {
    // if (!colorSensor.isFunctioning()) return true;
    // return colorSensor.isRightColor();
    return true;
  }

  public static Indexer getInstance() {
    if (instance == null) {
      instance = new Indexer();
    }
    return instance;
  }

  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Indexer");
    builder.addBooleanProperty("top proximity", () -> isBallInTop(), null);

    // builder.addBooleanProperty("Color is red", () -> colorSensor.isRed(), null);
    // builder.addBooleanProperty(
    //     "Color sensor is functioning", () -> colorSensor.isFunctioning(), null);
  }
}
