package frc.robot.commands.hood;

import frc.robot.subsystems.Hood;
import frc.robot.util.InterpolatingTreeMap;
import harkerrobolib.commands.IndefiniteCommand;

public class HoodManual extends IndefiniteCommand {

  public HoodManual() {
    addRequirements(Hood.getInstance());
  }

  public void execute() {
    Hood.getInstance().setHoodPosition(Hood.getInstance().calculateHoodPosition());
  }

  public void end(boolean interrupted) {
    Hood.getInstance().setHoodPosition(0);
  }
}
