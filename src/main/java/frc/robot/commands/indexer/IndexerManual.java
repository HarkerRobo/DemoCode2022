package frc.robot.commands.indexer;

import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import harkerrobolib.commands.IndefiniteCommand;

public class IndexerManual extends IndefiniteCommand {
  public static final double BOTTOM_INDEXER_SPEED = 0.6; // 6.0; // meters per second
  public static final double TOP_INDEXER_SPEED = 0.3; // 2.0;
  public static final double INDEXER_OUTTAKE_SPEED = -0.6; // -16; // meters per second
  // private Notifier topIndexerProximity;

  public IndexerManual() {
    addRequirements(Indexer.getInstance());
    // topIndexerProximity = new Notifier(()->{if (Indexer.getInstance().isBallInTop())
    // Indexer.getInstance().setTopOutput(-0.1);});
  }

  public void execute() {
    if (Shooter.getInstance().getState() == Shooter.State.SHOOTING) {
      Indexer.getInstance().setBothOutput(BOTTOM_INDEXER_SPEED);
      return;
    }
    switch (Intake.getInstance().getState()) {
      case INTAKE:
        if (!Indexer.getInstance().isRightColor())
          Indexer.getInstance().setBottomOutput(INDEXER_OUTTAKE_SPEED);
        else {
          if (Indexer.getInstance().isBallInTop()) Indexer.getInstance().setTopOutput(0);
          else Indexer.getInstance().setTopOutput(TOP_INDEXER_SPEED);
          if (Indexer.getInstance().isBallInBottom())
            if (Indexer.getInstance().isBallInTop()) Indexer.getInstance().setBottomOutput(0);
            else Indexer.getInstance().setBottomOutput(BOTTOM_INDEXER_SPEED);
          else Indexer.getInstance().setBottomOutput(BOTTOM_INDEXER_SPEED);
        }
        break;
      case OUTTAKE:
        Indexer.getInstance().setBothOutput(INDEXER_OUTTAKE_SPEED);
        break;
      case NEUTRAL:
        Indexer.getInstance().setBothOutput(0);
        break;
    }
  }

  public void end(boolean interrupted) {
    Indexer.getInstance().setBothOutput(0);
  }
}
