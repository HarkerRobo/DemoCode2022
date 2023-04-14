package frc.robot.util;

public final class Conversions {
  public static final double FEET_TO_METER = 0.305;
  public static final double INCH_TO_METER = 0.0254;
  public static final double ENCODER_TO_WHEEL_SPEED = 10.0 / 2048.0 * Math.PI * INCH_TO_METER;
  public static final double SECOND_TO_CTRE_SECOND = 10.0;
  public static final double ENCODER_TO_DEG = 360.0 / 2048.0;
}
