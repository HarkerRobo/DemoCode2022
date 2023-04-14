package frc.robot.commands.intake;

import frc.robot.OI;
import frc.robot.subsystems.Intake;
import harkerrobolib.commands.IndefiniteCommand;

public class IntakeManual extends IndefiniteCommand {

  public static final double INTAKE_SPEED = 3.0; // meters per second

  public IntakeManual() {
    addRequirements(Intake.getInstance());
  }

  public void execute() {
    if (OI.getInstance().getDriverGamepad().getButtonTriggerLeft().get())
      Intake.getInstance().setState(Intake.State.OUTTAKE);
    else if (OI.getInstance().getDriverGamepad().getButtonTriggerRight().get())
      Intake.getInstance().setState(Intake.State.INTAKE);
    else Intake.getInstance().setState(Intake.State.NEUTRAL);
    Intake.getInstance().actOnState(INTAKE_SPEED);
  }

  public void end(boolean interrupted) {
    Intake.getInstance().setState(Intake.State.NEUTRAL);
    Intake.getInstance().actOnState(INTAKE_SPEED);
  }
}
