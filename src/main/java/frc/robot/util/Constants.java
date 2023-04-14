package frc.robot.util;

import edu.wpi.first.math.geometry.Translation2d;

public final class Constants {

  public static final double FIELD_LENGTH = 54.0 * Conversions.FEET_TO_METER;
  public static final double FIELD_WIDTH = FIELD_LENGTH / 2.0;
  public static final double HUB_RADIUS = 427.0 / 16.0 / 12.0 * Conversions.FEET_TO_METER;
  public static final Translation2d HUB_LOCATION =
      new Translation2d(FIELD_LENGTH / 2.0, FIELD_WIDTH / 2.0);
  public static final double HUB_HEIGHT = 2.64;
}
