package frc.robot.subsystems.Transport;

import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Shooter.Shooter;

public class TransportSim implements TransportIO {
  private Intake intake = Intake.getInstance();
  private Shooter shooter = Shooter.getInstance();

  public TransportSim() {

  }

  @Override
  public void updateInputs(TransportIOInputs inputs) {
    inputs.hasNote = intake.simHasNote();
  }

  @Override
  public void set(double speed) {
    // shoots note
    if (speed > 0.35) { 
      intake.obtainNoteFromIntakeSim();
      shooter.shootNote();
    }
  }
}
