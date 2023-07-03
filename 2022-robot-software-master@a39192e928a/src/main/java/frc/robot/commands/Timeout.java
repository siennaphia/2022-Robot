
package frc.robot.commands;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.utilities.Util;

/**
 * Used in command groups to wait between commands.
 */

public class Timeout extends CommandBase {
    private final double timeoutValue;
    private long endTime = 0;

    public Timeout(double timeoutValue) {
        this.timeoutValue = timeoutValue;
    }

    @Override
    public void initialize() {
        Util.logf("Timeout cmd started. Seconds=%.1f\n", timeoutValue);
        endTime = RobotController.getFPGATime() + (int) timeoutValue * 1000000;

    }

    @Override
    public boolean isFinished() {
        if (Robot.climber.getAbortClimb()) {
            return true;
        }
        return RobotController.getFPGATime() >= endTime;
    }

    public void end() {
        Util.logf("Timeout cmd Completed. Seconds=%.1f\n", timeoutValue);
    }
}