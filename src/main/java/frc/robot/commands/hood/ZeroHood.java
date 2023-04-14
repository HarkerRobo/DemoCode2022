package frc.robot.commands.hood;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Hood;

public class ZeroHood extends CommandBase {
  private static final double ZERO_SPEED = -0.1;

  public ZeroHood() {
    addRequirements(Hood.getInstance());
  }

  public void execute() {
    Hood.getInstance().setHoodPercentOutput(ZERO_SPEED);
  }

  public boolean isFinished() {
    return Hood.getInstance().isHoodStalling();
  }

  public void end(boolean interrupted) {
    if (!interrupted) {
      Hood.getInstance().setHoodPercentOutput(0);
      Hood.getInstance().setHoodEncoderZero();
    }
  }
}
