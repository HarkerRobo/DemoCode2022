package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

public class SetClimberPos extends CommandBase {
  private double position;

  public SetClimberPos(double pos) {
    addRequirements(Climber.getInstance());
    position = pos;
  }

  public void execute() {
    Climber.getInstance().setBothClimberPos(position);
  }

  public boolean isFinished() {
    return Climber.getInstance().getLeftPositionSystem().atSetpoint()
        && Climber.getInstance().getRightPositionSystem().atSetpoint();
  }

  public void end(boolean interrupted) {
    Climber.getInstance().turnOffMotors();
  }
}
