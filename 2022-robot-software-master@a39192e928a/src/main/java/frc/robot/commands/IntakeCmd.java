package frc.robot.commands;

import static frc.robot.utilities.Util.logf;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.Joysticks;

public class IntakeCmd extends CommandBase {

    public enum IntakeMode {
        IN, OUT, LIFTBALL, PREPARESHOOT, REVERSEINTAKE, FORWARDINTAKE
    };

    private IntakeMode mode = IntakeMode.IN;

    public IntakeCmd(IntakeMode mode) {
        // Use requires() here to declare subsystem dependencies
        // addRequirements(Robot.intake);
        this.mode = mode;
        //logf("Intake setup mode:%s\n", mode);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        if (Robot.intake == null) {
            logf("Intake not setup mode:%s\n", mode);
            return; // If no intake definned return
        }
        logf("Intake will run mode:%s\n", mode);
        switch (mode) {
            case IN:
                Robot.intake.beaterBarIn(false); 
                break;
            case OUT:
                Robot.intake.beaterBarOut();
                break;
            case LIFTBALL:
                //Robot.intake.liftBall();
                break;
            case PREPARESHOOT:
                Robot.intake.ballInStart();
                break;
            case REVERSEINTAKE:
                Robot.intake.intakeOut();
                break;
            case FORWARDINTAKE:
                Robot.intake.intakeIn();
                //Robot.compressor.stop();
                break;
        }
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
        if (mode == IntakeMode.PREPARESHOOT) {
            if (!Joysticks.rightJoy.getRawButton(2)) {
                if (Robot.config.intake)
                    Robot.intake.ballInStop();
                return true;
            }
            return false;
        }
        if (mode == IntakeMode.REVERSEINTAKE) {
            if (Joysticks.operator.getRawButton(7)) {
                return false;
            } else {
                Robot.intake.stopIntake();
            }
        }
        if (mode == IntakeMode.FORWARDINTAKE) {
            if (Joysticks.operator.getRawButton(6)) {
                return false;
            } else {
                Robot.intake.stopIntake();
                //Robot.compressor.restart();

            }
        }
        return true;
    }
}
