package frc.robot.commands.shooter;

import frc.robot.OI;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.State;
import harkerrobolib.commands.IndefiniteCommand;

public class ShooterManual extends IndefiniteCommand {

  private double smartdashboard;

  public ShooterManual() {
    addRequirements(Shooter.getInstance());
  }

  public void execute() {
    double possibleShooterSpeed = Shooter.getInstance().calculateShooterSpeed();
    // SmartDashboard.putNumber("speed", smartdashboard);
    updateShooterState(possibleShooterSpeed);
    if (Shooter.getInstance().getState() != State.IDLE) {
      Shooter.getInstance().set(possibleShooterSpeed);
    } else Shooter.getInstance().turnOffMotors();
  }

  private void updateShooterState(double nextSpeed) {
    if (OI.getInstance().getDriverGamepad().getButtonY().get()) {
      switch (Shooter.getInstance().getState()) {
        case IDLE:
          Shooter.getInstance().setState(State.REVVING);
          break;
        case REVVING:
          if (Shooter.getInstance().atSpeed(nextSpeed)) // && Drivetrain.getInstance().isAligned())
          Shooter.getInstance().setState(State.SHOOTING);
          break;
        case SHOOTING:
          if (!(Shooter.getInstance()
              .atSpeed(nextSpeed))) // && Drivetrain.getInstance().isAligned()))
          Shooter.getInstance().setState(State.REVVING);
      }
    } else {
      Shooter.getInstance().setState(State.IDLE);
    }
  }

  public void end(boolean interrupted) {
    Shooter.getInstance().turnOffMotors();
  }
}
