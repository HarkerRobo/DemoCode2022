package frc.robot.commands.drivetrain;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.OI;
import frc.robot.RobotMap;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Shooter;
import harkerrobolib.commands.IndefiniteCommand;
import harkerrobolib.util.MathUtil;

public class SwerveManual extends IndefiniteCommand {
  public static final double SPEED_MULTIPLIER = 0.5;
  public static final double MIN_OUTPUT = 0.0001;
  private static final double PIGEON_KP = 0.0003;
  private static final double MAX_ACCELERATION = Drivetrain.MAX_ACCELERATION + 6;

  private double vx;
  private double prevvx;
  private double vy;
  private double prevvy;
  private double omega;

  private double pigeonAngle;
  private boolean holdingPigeonAngle;

  private boolean aligningWithHub;

  // private static SwerveControllerLoop HUB_LOOP = new SwerveControllerLoop();

  public SwerveManual() {
    addRequirements(Drivetrain.getInstance());
    vx = vy = omega = prevvx = prevvy = 0;
    aligningWithHub = false;
    holdingPigeonAngle = false;
    pigeonAngle = Drivetrain.getInstance().getRobotHeading();
  }

  public void initialize() {
    Drivetrain.getInstance().setDrivetrainOffset();
  }

  public void execute() {
    prevvx = vx;
    prevvy = vy;
    vx =
        MathUtil.mapJoystickOutput(
            OI.getInstance().getDriverGamepad().getLeftY(), OI.DEFAULT_DEADBAND);
    vy =
        MathUtil.mapJoystickOutput(
            OI.getInstance().getDriverGamepad().getLeftX(), OI.DEFAULT_DEADBAND);
    omega =
        -MathUtil.mapJoystickOutput(
            OI.getInstance().getDriverGamepad().getRightX(), OI.DEFAULT_DEADBAND);
    squareInputs();
    scaleToDrivetrainSpeeds();
    limitAcceleration();
    adjustPigeon();
    if (Math.abs(omega) <= MIN_OUTPUT && magnitude(vx, vy) <= MIN_OUTPUT) {
      omega = MIN_OUTPUT;
      vx = vy = 0;
      // ChassisSpeeds speeds =
      //     ChassisSpeeds.fromFieldRelativeSpeeds(
      //         vx, vy, omega, Drivetrain.getInstance().getRobotRotation());
      // var states = Drivetrain.getInstance().getKinematics().toSwerveModuleStates(speeds);
      // for (int i = 0; i < 4; i++) states[i].angle =
      // states[i].angle.plus(Rotation2d.fromDegrees(90));
      // Drivetrain.getInstance().setAngleAndDrive(states);
    }
    
    Drivetrain.getInstance()
        .setAngleAndDrive(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                vx, -vy, omega, Drivetrain.getInstance().getRobotRotation()));
  }

  private double magnitude(double x, double y) {
    return Math.sqrt(x * x + y * y);
  }

  public void limitAcceleration() {
    double xAcc = (vx - prevvx) / RobotMap.ROBOT_LOOP;
    double yAcc = (vy - prevvy) / RobotMap.ROBOT_LOOP;
    double magAcc = magnitude(xAcc, yAcc);
    if (magAcc > MAX_ACCELERATION) {
      double scale = MAX_ACCELERATION / magAcc;
      vx = prevvx + xAcc * scale * RobotMap.ROBOT_LOOP;
      vy = prevvy + yAcc * scale * RobotMap.ROBOT_LOOP;
    }
  }

  public void adjustPigeon() {
    if (Math.abs(omega) < MIN_OUTPUT) {
      omega = -PIGEON_KP * (Drivetrain.getInstance().getRobotHeading() - pigeonAngle);
    } else {
      pigeonAngle = Drivetrain.getInstance().getRobotHeading();
    }

    SmartDashboard.putNumber("pigeonangle", Drivetrain.getInstance().getRobotHeading());
    SmartDashboard.putNumber("held pigeonangle", pigeonAngle);
  }

  public void squareInputs() {
    vx *= Math.abs(vx);
    vy *= Math.abs(vy);
    omega *= Math.abs(omega);
  }

  public void scaleToDrivetrainSpeeds() {
    vx *= SPEED_MULTIPLIER * Drivetrain.MAX_TRANSLATION_VEL;
    vy *= SPEED_MULTIPLIER * Drivetrain.MAX_TRANSLATION_VEL;
    omega *= SPEED_MULTIPLIER * Drivetrain.MAX_ROTATION_VEL;
  }
}
