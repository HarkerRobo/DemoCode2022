package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.OI;
import frc.robot.subsystems.Climber;

public class ClimberStages {
  private static class WaitForDriverA extends WaitUntilCommand {
    public WaitForDriverA() {
      super(() -> OI.getInstance().getDriverGamepad().getButtonAState());
      addRequirements(Climber.getInstance());
    }
  }

  private static class WaitForDriverB extends WaitUntilCommand {
    public WaitForDriverB() {
      super(() -> OI.getInstance().getDriverGamepad().getButtonBState());
      addRequirements(Climber.getInstance());
    }
  }

  public static final SequentialCommandGroup STAGE_1 =
      new SequentialCommandGroup(
          new ZeroClimber(),
          new SetClimberPos(Climber.UP_HEIGHT),
          new WaitForDriverB(),
          new SetClimberPos(Climber.DOWN_HEIGHT));

  public static SequentialCommandGroup STAGE_2_AND_3() {
    return new SequentialCommandGroup(
        new SetClimberPos(Climber.MID_HEIGHT),
        new WaitForDriverA(),
        new InstantCommand(() -> Climber.getInstance().setClimberBackward(), Climber.getInstance()),
        new WaitForDriverB(),
        new SetClimberPos(Climber.UP_HEIGHT),
        new WaitForDriverA(),
        new InstantCommand(() -> Climber.getInstance().setClimberForward(), Climber.getInstance()),
        new WaitForDriverB(),
        new SetClimberPos(Climber.DOWN_HEIGHT));
  }

  public static final SequentialCommandGroup ALL_STAGES =
      new SequentialCommandGroup(
          STAGE_1, new WaitForDriverB(), STAGE_2_AND_3(), new WaitForDriverB(), STAGE_2_AND_3());
}
