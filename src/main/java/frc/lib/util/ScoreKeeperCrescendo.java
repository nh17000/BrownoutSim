package frc.lib.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

public class ScoreKeeperCrescendo {
  private static final ScoreKeeperCrescendo SCOREKEEPER = new ScoreKeeperCrescendo();

  public static ScoreKeeperCrescendo getInstance() {
    return SCOREKEEPER;
  }

  public enum Goal {
    Speaker,
    Amp,
    Trap
  }

  public enum EndgameState {
    None,
    Parked,
    Onstage,
    OnstageSpotlit,
  }

  private int score, notesInAmp;
  private boolean autoAmplifyEnabled;
  private double lastAmplication;

  private ScoreKeeperCrescendo() {
    score = 0;
    notesInAmp = 0;
    autoAmplifyEnabled = true;
    lastAmplication = 0.0;
  }

  public int scoreGoal(Goal goal) {
    if (DriverStation.isAutonomous()) {
      switch (goal) {
        case Speaker: 
          score += 5; break;
        case Amp: 
          score += 2; 
          notesInAmp++; break;
        default: break;
      }
    } else {
      switch (goal) {
        case Speaker: 
          if (autoAmplifyEnabled) { amplify(Timer.getFPGATimestamp()); }
          score += isAmplified() ? 5 : 1; break;
        case Amp: 
          score += 1; 
          notesInAmp++; break;
        case Trap: 
          score += 5; break;
      }
    }
    return score;
  }

  public int setEndgame(EndgameState state) {
    return setEndgame(state, false);
  }

  public int setEndgame(EndgameState state, boolean harmonized) {
    switch (state) {
      case Parked: 
        score += 1; break;
      case Onstage: 
        score += 3; break;
      case OnstageSpotlit: 
        score += 4; break;
      default: break;
    }

    if (harmonized) score += 2;

    return score;
  }

  public void amplify(double timestamp) {
    if (canAmplify()) { lastAmplication = timestamp; }
  }

  public boolean canAmplify() {
    return notesInAmp >= 2 && !isAmplified();
  }
  
  public boolean isAmplified() {
    return Timer.getFPGATimestamp() - lastAmplication < 13; // 10s + 3s leeway
  }

  public int getScore() {
    return score;
  }

  public void toggleAutoAmplify(boolean enabled) {
    autoAmplifyEnabled = enabled;
  }
}
