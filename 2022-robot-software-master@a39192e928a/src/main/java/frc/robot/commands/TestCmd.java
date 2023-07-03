
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.utilities.Util;

public class TestCmd extends CommandBase {
    double joy;

    public TestCmd() {
        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        joy = -1.0;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if (joy > 1)
            return true;
        Util.logf("Test Ramp joy:%.2f ramp:%.2f\n", joy, Robot.drivetrain.rampExp(joy, 0, "Test"));
        joy += .05;
        return false;
    }
}
