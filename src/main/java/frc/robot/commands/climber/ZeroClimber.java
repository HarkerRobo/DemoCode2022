package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

public class ZeroClimber extends CommandBase {
  private static final double ZERO_SPEED = -0.27;

  public ZeroClimber() {
    addRequirements(Climber.getInstance());
  }

  public void execute() {
    if (!Climber.getInstance().leftLimitSwitch())
      Climber.getInstance().setLeftPercentOutput(ZERO_SPEED);
    else {
      Climber.getInstance().setLeftPercentOutput(0);
    }
    if (!Climber.getInstance().rightLimitSwitch())
      Climber.getInstance().setRightPercentOutput(ZERO_SPEED);
    else {
      Climber.getInstance().setRightPercentOutput(0);
    }
  }

  public boolean isFinished() {
    return Climber.getInstance().limitSwitchHit();
  }

  public void end(boolean interrupted) {
    if (!interrupted) {
      Climber.getInstance().setLeftPercentOutput(0);
      Climber.getInstance().setRightPercentOutput(0);
      Climber.getInstance().getLeftClimber().setSelectedSensorPosition(0);
      Climber.getInstance().getRightClimber().setSelectedSensorPosition(0);
    }
  }
}
