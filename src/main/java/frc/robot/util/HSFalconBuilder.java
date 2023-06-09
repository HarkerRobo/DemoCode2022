package frc.robot.util;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;
import frc.robot.RobotMap;
import harkerrobolib.wrappers.HSFalcon;

public class HSFalconBuilder {
  private NeutralMode neutralMode = NeutralMode.Brake;
  private boolean invert = false;
  private SensorVelocityMeasPeriod velocityMeasPeriod = SensorVelocityMeasPeriod.Period_100Ms;
  private int velocityWindow = 1;
  private int voltageFilter = 16;
  private int fastCANFrame = (int) (1000 * RobotMap.ROBOT_LOOP);
  private int slowCANFrame = 2 * fastCANFrame;
  private StatorCurrentLimitConfiguration stator;
  private SupplyCurrentLimitConfiguration supply;
  private double voltageComp = RobotMap.MAX_MOTOR_VOLTAGE;

  public HSFalconBuilder neutralMode(NeutralMode neutralMode) {
    this.neutralMode = neutralMode;
    return this;
  }

  public HSFalconBuilder invert(boolean invert) {
    this.invert = invert;
    return this;
  }

  public HSFalconBuilder velocityMeasurementPeriod(SensorVelocityMeasPeriod period) {
    velocityMeasPeriod = period;
    return this;
  }

  public HSFalconBuilder voltageFilter(int voltageFilter) {
    this.voltageFilter = voltageFilter;
    return this;
  }

  public HSFalconBuilder statorLimit(double peak, double sustained, double peakdur) {
    stator = new StatorCurrentLimitConfiguration(true, sustained, peak, peakdur);
    supply = null;
    return this;
  }

  public HSFalconBuilder supplyLimit(double peak, double sustained, double peakdur) {
    supply = new SupplyCurrentLimitConfiguration(true, sustained, peak, peakdur);
    stator = null;
    return this;
  }

  public HSFalconBuilder canFramePeriods(int fast, int slow) {
    fastCANFrame = fast;
    slowCANFrame = slow;
    return this;
  }

  public HSFalconBuilder velocityWindow(int window) {
    velocityWindow = window;
    return this;
  }

  public HSFalconBuilder voltageComp(double voltageComp) {
    this.voltageComp = voltageComp;
    return this;
  }

  public HSFalcon build(int deviceID, String canbus) {
    HSFalcon falcon = new HSFalcon(deviceID, canbus);
    falcon.configFactoryDefault();
    falcon.setNeutralMode(neutralMode);
    falcon.setInverted(invert);
    falcon.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    falcon.configVelocityMeasurementPeriod(velocityMeasPeriod);
    falcon.configVelocityMeasurementWindow(velocityWindow);
    falcon.configVoltageMeasurementFilter(voltageFilter);
    if (stator != null) falcon.configStatorCurrentLimit(stator);
    if (supply != null) falcon.configSupplyCurrentLimit(supply);
    for (StatusFrame frame : StatusFrame.values())
      falcon.setStatusFramePeriod(frame, RobotMap.MAX_CAN_FRAME_PERIOD);
    falcon.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, fastCANFrame);
    falcon.setStatusFramePeriod(StatusFrame.Status_4_AinTempVbat, slowCANFrame);
    falcon.configVoltageCompSaturation(voltageComp);
    falcon.enableVoltageCompensation(true);
    falcon.configClosedLoopPeriod(RobotMap.SLOT_INDEX, 1);
    return falcon;
  }

  public HSFalcon build(int deviceID) {
    return build(deviceID, "rio");
  }
}
