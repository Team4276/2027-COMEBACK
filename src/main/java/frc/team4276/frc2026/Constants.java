// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team4276.frc2026;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY,
  }

  public static Mode getMode() {
    return mode;
  }

  public static enum RobotType {
    COMPBOT,
    SIMBOT
  }

  public static Mode mode = Mode.SIM;

  public static RobotType getType() {
    return switch (mode) {
      case REAL -> RobotType.COMPBOT;
      case REPLAY -> RobotType.COMPBOT;
      case SIM -> RobotType.SIMBOT;
    };
  }

  public static final boolean isTuning = true;

  public static final boolean isSim = (mode == Mode.SIM);

  public static final boolean isDemo = false;

  /** Checks whether the correct robot is selected when deploying. */
  public static class CheckDeploy {
    public static void main(String... args) {
      if (getType() == RobotType.SIMBOT) {
        System.err.println("Cannot deploy, invalid robot selected: " + getType());
        System.exit(1);
      }
    }
  }
}
