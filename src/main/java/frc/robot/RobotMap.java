package frc.robot;

public final class RobotMap {
  public static final String CANBUS = "BINGCHILLING";
  public static final double ROBOT_LOOP = 0.01;
  public static final double TALON_FX_LOOP = 0.001;
  public static final double MAX_MOTOR_VOLTAGE = 10.0;
  public static final int MAX_CAN_FRAME_PERIOD = 255;
  public static final int SLOT_INDEX = 0;

  public static final String LIMELIGHT_NAME = "gloworm";

  public static final int DEFAULT_LOOP_ID = 0;
  public static final int DEFAULT_SLOT_ID = 0;

  public static final int[] TRANSLATION_IDS = {13, 4, 2, 6}; // FL, FR, BL, BR
  public static final int[] ROTATION_IDS = {1, 5, 3, 7};
  public static final int[] CANCODER_IDS = {0, 2, 1, 3};
  public static final int PIGEON_ID = 0;

  public static final int INTAKE_FORWARD = 7;
  public static final int INTAKE_BACKWARD = 3;
  public static final int INTAKE_MOTOR = 8;

  public static final int INDEXER_TOP = 10;
  public static final int INDEXER_BOTTOM = 9;

  public static final int HOOD_ID = 14;

  public static final int TOP_PROXIMITY = 7;
  public static final int BOTTOM_PROXIMITY = 6;

  public static final int COLOR_PROXIMITY = 0;
  public static final int COLOR_A = 0;
  public static final int COLOR_B = 0;

  public static final int SHOOTER_MASTER = 11;
  public static final int SHOOTER_FOLLOWER = 12;

  public static final int RIGHT_CLIMBER = 15;
  public static final int LEFT_CLIMBER = 16;
  public static final int CLIMBER_FORWARD = 2;
  public static final int CLIMBER_BACKWARD = 6;
  public static final int CLIMBER_RIGHT_LIMIT_SWTICH = 4;
  public static final int CLIMBER_LEFT_LIMIT_SWITCH = 5;

  public static final int OPERATOR_ID = 1;
  public static final int DRIVER_ID = 0;
}
