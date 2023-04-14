package frc.robot.commands.intake;

import frc.robot.subsystems.Intake;
import harkerrobolib.commands.IndefiniteCommand;

public class IntakeAuton extends IndefiniteCommand {
  public IntakeAuton() {
    addRequirements(Intake.getInstance());
  }

  public void execute() {
    Intake.getInstance().setState(Intake.State.INTAKE);
    Intake.getInstance().actOnState(IntakeManual.INTAKE_SPEED);
  }

  public void end(boolean interrupted) {
    Intake.getInstance().setState(Intake.State.NEUTRAL);
    Intake.getInstance().actOnState(0);
  }
}
