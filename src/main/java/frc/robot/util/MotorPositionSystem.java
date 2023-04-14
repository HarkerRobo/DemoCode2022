package frc.robot.util;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.RobotMap;

public class MotorPositionSystem extends MotorVelocitySystem {
  private double kG, kD, positionSetpoint, maxPosError;
  public boolean arm;

  private MotorPositionSystem(
      BaseMotorController motor,
      double kS,
      double kV,
      double kA,
      double unitConversion,
      double maxPosError,
      double maxVelError,
      double maxVoltage,
      double kG,
      boolean arm) {
    super(motor, kS, kV, kA, 0, unitConversion, maxVelError, maxVoltage);
    this.kG = kG;
    this.arm = arm;
    this.maxPosError = maxPosError;
  }

  public MotorPositionSystem init() {
    positionSetpoint = 0.0;
    super.init();
    return this;
  }

  public void configConstants() {
    super.configConstants();
    motor.config_kD(RobotMap.SLOT_INDEX, kD);
  }

  public void calculateConstants() {
    if (maxPosError < 0.000001) maxPosError = 0.001;
    Matrix<N1, N2> gains =
        new LinearQuadraticRegulator<>(
                LinearSystemId.identifyPositionSystem(kV, kA),
                VecBuilder.fill(maxPosError, maxError),
                VecBuilder.fill(maxVoltage),
                RobotMap.TALON_FX_LOOP)
            .getK();
    gains.times(1023.0 / maxVoltage * unitConversion);
    kP = gains.get(0, 0);
    kD = gains.get(0, 1);
    kF = kV * 1023.0 / maxVoltage * unitConversion;
    configConstants();
  }

  public void set(double position) {
    positionSetpoint = position;
    velocitySetpoint = 0.0;
    double ff = Math.signum(getPositionError()) * kS;
    if (arm) ff += kG * Math.cos(Math.toRadians(getPosition()));
    else ff += kG;
    motor.set(
        ControlMode.Position,
        position / unitConversion,
        DemandType.ArbitraryFeedForward,
        ff / maxVoltage);
  }

  public double getVelocitySetpoint() {
    return 0.0;
  }

  public double getVelocity() {
    return super.getVelocity() * Conversions.SECOND_TO_CTRE_SECOND;
  }

  public double getPosition() {
    return motor.getSelectedSensorPosition() * unitConversion;
  }

  public double getPositionError() {
    return positionSetpoint - getPosition();
  }

  public double getPositionSetpoint() {
    return positionSetpoint;
  }

  public boolean atSetpoint() {
    return super.atSetpoint() && Math.abs(getPositionError()) < maxPosError;
  }

  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.setSmartDashboardType("MotorPositionSystem");
    builder.addDoubleProperty("Vel Setpoint", () -> getVelocitySetpoint(), null);
    builder.addDoubleProperty("Position", () -> getPosition(), null);
    builder.addDoubleProperty("Pos Setpoint", () -> getPositionSetpoint(), (a) -> set(a));
    builder.addDoubleProperty("Pos Error", () -> getPositionError(), null);
    builder.addDoubleProperty(
        "kD",
        () -> kD,
        (a) -> {
          kD = a;
          calculateConstants();
        });
    builder.addDoubleProperty("kG", () -> kG, (a) -> kG = a);
    builder.addBooleanProperty("isArm", () -> arm, (a) -> arm = a);
  }

  public static class MotorPositionSystemBuilder {
    private double kV,
        kA,
        kS,
        maxPosError,
        unitConversion = 1.0,
        maxVoltage = RobotMap.MAX_MOTOR_VOLTAGE,
        maxVelError,
        kG = 0.0;
    private boolean arm;

    public MotorPositionSystemBuilder maxError(double posError, double velError) {
      this.maxPosError = posError;
      this.maxVelError = velError;
      return this;
    }

    public MotorPositionSystemBuilder elevatorGravityConstant(double kG) {
      this.kG = kG;
      this.arm = false;
      return this;
    }

    public MotorPositionSystemBuilder armGravityConstant(double kG) {
      this.kG = kG;
      this.arm = true;
      return this;
    }

    public MotorPositionSystemBuilder constants(double kV, double kA, double kS) {
      this.kV = kV;
      this.kA = kA;
      this.kS = kS;
      return this;
    }

    public MotorPositionSystemBuilder unitConversionFactor(double factor) {
      unitConversion = factor;
      return this;
    }

    public MotorPositionSystemBuilder maxVoltage(double voltage) {
      maxVoltage = voltage;
      return this;
    }

    public MotorPositionSystem build(BaseMotorController motor) {
      return new MotorPositionSystem(
          motor, kS, kV, kA, unitConversion, maxPosError, maxVelError, maxVoltage, kG, arm);
    }
  }
}
