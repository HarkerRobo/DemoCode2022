// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.drivetrain.SwerveManual;
import frc.robot.commands.hood.HoodManual;
import frc.robot.commands.indexer.IndexerManual;
import frc.robot.commands.intake.IntakeManual;
import frc.robot.commands.shooter.ShooterManual;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final Field2d FIELD = new Field2d();
  private Notifier coastDrivetrainNotifier;

  public Robot() {
    super();
  }

  public Robot(double period) {
    super(period);
    coastDrivetrainNotifier =
        new Notifier(
            () -> {
              if (isDisabled()) Drivetrain.getInstance().setNeutralMode(NeutralMode.Coast);
            });
  }
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    CommandScheduler.getInstance().setDefaultCommand(Drivetrain.getInstance(), new SwerveManual());
    CommandScheduler.getInstance().setDefaultCommand(Intake.getInstance(), new IntakeManual());
    SmartDashboard.putNumber("angle", 0.0);
    CommandScheduler.getInstance().setDefaultCommand(Shooter.getInstance(), new ShooterManual());
    CommandScheduler.getInstance().setDefaultCommand(Indexer.getInstance(), new IndexerManual());
    CommandScheduler.getInstance().setDefaultCommand(Hood.getInstance(), new HoodManual());
    SmartDashboard.putNumber("angle to hub", 0);
    SmartDashboard.putData("Field", FIELD);
    NetworkTableInstance.getDefault().setUpdateRate(RobotMap.ROBOT_LOOP);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    // SmartDashboard.putNumber("Robot loop", counter);
    Drivetrain.getInstance().updatePoseEstimator();
    FIELD.setRobotPose(Drivetrain.getInstance().getPoseEstimatorPose2d());
    SmartDashboard.putNumber(
        "current pose x",
        Drivetrain.getInstance().getPoseEstimatorPose2d().getTranslation().getX());
    SmartDashboard.putNumber(
        "current pose y",
        Drivetrain.getInstance().getPoseEstimatorPose2d().getTranslation().getY());
    SmartDashboard.putNumber(
        "current pose rotatoin",
        Drivetrain.getInstance().getPoseEstimatorPose2d().getRotation().getDegrees());
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    Drivetrain.getInstance().setNeutralMode(NeutralMode.Brake);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    Drivetrain.getInstance().setNeutralMode(NeutralMode.Brake);
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
    coastDrivetrainNotifier.startSingle(3.0);
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
