package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.sensors.PigeonIMU_StatusFrame;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.util.InterpolatingTreeMap;
import frc.robot.util.SwerveModule;

public class Drivetrain extends SubsystemBase {
  public static Drivetrain instance;

  public static final boolean[] ROTATION_INVERTS = {false, false, false, false};
  public static final boolean[] DRIVE_INVERTS = {true, true, true, false};

  public static final double[] CANCODER_OFFSETS = {
    149.719, 178.330078, 109.951172, 32.255859
  }; // in deg

  private static final double DT_WIDTH = 0.5461; // 0.93345 bumper to bumper
  private static final double DT_LENGTH = 0.5969; // 0.88265

  public static final double MAX_TRANSLATION_VEL = 4.0; // in m/s
  public static final double MAX_ACCELERATION = 2.0; // m/s^2
  public static final double MAX_ROTATION_VEL = 1.5 * Math.PI; // in rad/s
  public static final double MAX_ROTATION_ACCELERATION = 1.0 * Math.PI;

  private static final Matrix<N3, N1> ODOMETRY_STATE_STDEV = VecBuilder.fill(1.0, 1.0, 1.0);
  private static final Matrix<N1, N1> ODOMETRY_ENCODER_STDEV = VecBuilder.fill(1.0);
  private static final Matrix<N3, N1> ODOMETRY_VISION_STDEV = VecBuilder.fill(1.0, 1.0, 1.0);

  private SwerveModule[] swerveModules;
  private SwerveDriveKinematics kinematics;
  private SwerveDrivePoseEstimator poseEstimator;
  private Pigeon2 pigeon;

  public static final double LIMELIGHT_KP = 0.12;
  public static final double LIMELIGHT_KI = 0.01;
  public static final double LIMELIGHT_KD = 0.000000;

  private InterpolatingTreeMap headingHistory;
  private static final int MAX_HISTORY_SIZE = 500;

  private static ProfiledPIDController HUB_LOOP =
      new ProfiledPIDController(LIMELIGHT_KP, LIMELIGHT_KI, LIMELIGHT_KD, new Constraints(4, 3.5));

  private static final boolean PIGEON_UP = false;

  private Drivetrain() {
    swerveModules =
        new SwerveModule[] {
          new SwerveModule(0), new SwerveModule(1), new SwerveModule(2), new SwerveModule(3)
        };
    for (int i = 0; i < 4; i++) {
      addChild(SwerveModule.swerveIDToName(i) + " Module", swerveModules[i]);
    }

    HUB_LOOP.setGoal(0);

    kinematics =
        new SwerveDriveKinematics(
            new Translation2d(DT_LENGTH / 2, DT_WIDTH / 2),
            new Translation2d(DT_LENGTH / 2, -DT_WIDTH / 2),
            new Translation2d(-DT_LENGTH / 2, DT_WIDTH / 2),
            new Translation2d(-DT_LENGTH / 2, -DT_WIDTH / 2));

    pigeon = new Pigeon2(RobotMap.PIGEON_ID, RobotMap.CANBUS);
    for (PigeonIMU_StatusFrame frame : PigeonIMU_StatusFrame.values())
      pigeon.setStatusFramePeriod(frame, RobotMap.MAX_CAN_FRAME_PERIOD);
    pigeon.setStatusFramePeriod(
        PigeonIMU_StatusFrame.CondStatus_9_SixDeg_YPR, (int) (1000 * RobotMap.ROBOT_LOOP));

    poseEstimator =
        new SwerveDrivePoseEstimator(
            getRobotRotation(),
            new Pose2d(0, 0, getRobotRotation()),
            kinematics,
            ODOMETRY_STATE_STDEV,
            ODOMETRY_ENCODER_STDEV,
            ODOMETRY_VISION_STDEV);
    headingHistory = new InterpolatingTreeMap();
  }

  public void updatePoseEstimator() {
    poseEstimator.update(
        getRobotRotation(),
        swerveModules[0].getState(),
        swerveModules[1].getState(),
        swerveModules[2].getState(),
        swerveModules[3].getState());
    // headingHistory.put(
    //     Timer.getFPGATimestamp(),
    // poseEstimator.getEstimatedPosition().getRotation().getRadians());
    // if (headingHistory.size() >= MAX_HISTORY_SIZE) headingHistory.pollFirstEntry();
    // poseEstimator.addVisionMeasurement(
    //     new Pose2d(
    //         PhotonVisionLimelight.robotToField(),
    //         new Rotation2d(
    //             headingHistory.get(
    //                 Timer.getFPGATimestamp() - PhotonVisionLimelight.lastMeasurementLatency()))),
    //     Timer.getFPGATimestamp() - PhotonVisionLimelight.lastMeasurementLatency());
  }

  public void setAngleAndDrive(ChassisSpeeds chassisSpeeds) {
    setAngleAndDrive(kinematics.toSwerveModuleStates(chassisSpeeds));
  }

  public void setAngleAndDrive(SwerveModuleState[] states) {
    for (int i = 0; i < 4; i++) {
      double driveOutput = states[i].speedMetersPerSecond;
      double angle = states[i].angle.getDegrees();
      double diff = angle - swerveModules[i].getCurrentAngle();
      diff = diff % 360.0;
      if (Math.abs(diff) > 270.0) {
        diff -= Math.signum(diff) * 360.0;
      } else if (Math.abs(diff) > 90.0) {
        diff -= Math.signum(diff) * 180.0;
        driveOutput = -driveOutput;
      }
      swerveModules[i].setAngleAndDrive(diff + swerveModules[i].getCurrentAngle(), driveOutput);
    }
  }

  public boolean isAligned() {
    return false;
    // Pose2d currentPose = poseEstimator.getEstimatedPosition();
    // double dist = currentPose.getTranslation().getDistance(FieldConstants.HUB_LOCATION);
    // double threshold =
    //     Math.toDegrees(Math.atan(Shooter.CUSTOM_RADIUS / (dist + FieldConstants.HUB_RADIUS)));
    // Translation2d diff = FieldConstants.HUB_LOCATION.minus(currentPose.getTranslation());
    // double angleToHub = Math.toDegrees(Math.atan2(diff.getY(), diff.getX()));
    // return Math.abs(getRobotHeading() - angleToHub) <= threshold;
  }

  public void setDrivetrainOffset() {
    for (int i = 0; i < 4; i++) swerveModules[i].setRotationOffset();
  }

  public void setNeutralMode(NeutralMode mode) {
    for (int i = 0; i < 4; i++) {
      swerveModules[i].getDriveMotor().setNeutralMode(mode);
      swerveModules[i].getRotationMotor().setNeutralMode(mode);
    }
  }

  public Rotation2d getRobotRotation() {
    return Rotation2d.fromDegrees(getRobotHeading());
  }

  public Pose2d getPoseEstimatorPose2d() {
    return poseEstimator.getEstimatedPosition();
  }

  public SwerveDriveKinematics getKinematics() {
    return kinematics;
  }

  public ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(
        swerveModules[0].getState(),
        swerveModules[1].getState(),
        swerveModules[2].getState(),
        swerveModules[3].getState());
  }

  public SwerveModule getSwerveModule(int id) {
    return swerveModules[id];
  }

  public double getRobotHeading() {
    return (PIGEON_UP) ? -pigeon.getYaw() : pigeon.getYaw();
  }

  public void setPose(Pose2d pose) {
    for (int i = 0; i < 4; i++) {
      swerveModules[i].zeroDriveEncoders();
    }
    pigeon.setYaw(pose.getRotation().getDegrees());
    poseEstimator.resetPosition(pose, pose.getRotation());
  }

  public void zeroPigeon() {
    pigeon.setYaw(0);
    pigeon.setAccumZAngle(0);
  }

  public Pigeon2 getPigeon() {
    return pigeon;
  }

  public static Drivetrain getInstance() {
    if (instance == null) instance = new Drivetrain();
    return instance;
  }

  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Drivetrain");
    builder.addDoubleProperty("Robot Heading", () -> getRobotHeading(), null);
  }
}
