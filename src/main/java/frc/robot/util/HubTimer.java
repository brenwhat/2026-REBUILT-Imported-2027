package frc.robot.util;

import java.util.Optional;

import org.wpilib.driverstation.DriverStation;
import org.wpilib.driverstation.DriverStation.Alliance;
import org.wpilib.smartdashboard.SmartDashboard;
import org.wpilib.command2.InstantCommand;
import org.wpilib.command2.SequentialCommandGroup;
import org.wpilib.command2.WaitCommand;

public class HubTimer {
    public boolean autoWin;

    private final int shift1End = 105;
    private final int shift2End = 81;
    private final int shift3End = 56;
    private final int shift4End = 31;

public HubTimer() {

}

public boolean isHubActive() {
  Optional<Alliance> alliance = DriverStation.getAlliance();
  // If we have no alliance, we cannot be enabled, therefore no hub.
  if (alliance.isEmpty()) {
    return false;
  }
  // Hub is always enabled in autonomous.
  if (DriverStation.isAutonomousEnabled()) {
    return true;
  }
  // At this point, if we're not teleop enabled, there is no hub.
  if (!DriverStation.isTeleopEnabled()) {
    return false;
  }

  // We're teleop enabled, compute.
  double matchTime = DriverStation.getMatchTime();
  String gameData = DriverStation.getGameSpecificMessage();
  // If we have no game data, we cannot compute, assume hub is active, as its likely early in teleop.
  if (gameData.isEmpty()) {
    return true;
  }
  boolean redInactiveFirst = false;
  switch (gameData.charAt(0)) {
    case 'R' -> redInactiveFirst = true;
    case 'B' -> redInactiveFirst = false;
    default -> {
      // If we have invalid game data, assume hub is active.
      return true;
    }
  }

  // Shift was is active for blue if red won auto, or red if blue won auto.
  boolean shift1Active = switch (alliance.get()) {
    case Red -> !redInactiveFirst;
    case Blue -> redInactiveFirst;
  };

  if (matchTime > 130) {
    // Transition shift, hub is active.
    return true;
  } else if (matchTime > 105) {
    // Shift 1
    return shift1Active;
  } else if (matchTime > 80) {
    // Shift 2
    return !shift1Active;
  } else if (matchTime > 55) {
    // Shift 3
    return shift1Active;
  } else if (matchTime > 30) {
    // Shift 4
    return !shift1Active;
  } else {
    // End game, hub always active.
    return true;
  }
}

public int getShiftInterval(int matchTime) {
    if (matchTime > 105) {return shift1End;}
    else if (matchTime > 80) {return shift2End;}
    else if (matchTime > 55) {return shift3End;}
    else {return shift4End;}
}

public int getRemainingHubShift() {
    int remainingTime;
    int matchTime = (int) DriverStation.getMatchTime();
    if (matchTime > 130) {return 0;}
    int shiftEnd = getShiftInterval(matchTime);
    remainingTime = matchTime - shiftEnd;
    return remainingTime;
}

}