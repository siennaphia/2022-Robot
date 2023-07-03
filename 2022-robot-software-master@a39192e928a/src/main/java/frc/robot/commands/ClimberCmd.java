package frc.robot.commands;

import static frc.robot.utilities.Util.logf;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class ClimberCmd extends CommandBase {

    // private int delay = 0;
    private int pneumaticDelayTicks = 5; // 5 mean 5 times the 20 milli second loop

    public enum ClimbMode {
        IDLE, HOME, DISTANCE, STATIC_UP, STATIC_DOWN, DYNAMIC_UP, DYNAMIC_DOWN,
        BAR_RELEASE_OUT, BAR_RELEASE_IN, ABORT_CLIMB, CLEAR_ABORT_FLAG
    };

    private double distance;
    private ClimbMode mode = ClimbMode.IDLE;
    private int delayTicks = 0;
    private int delay = 0;
    // private double delaySeconds;
    private double requestedSpeed = 0;

    public ClimberCmd(ClimbMode mode, double distance, double speed, double secondsToDelay) {
        // Use requires() here to declare subsystem dependencies
        // addRequirements(Robot.intake);
        this.mode = mode;
        this.delayTicks = (int) (secondsToDelay * 50);
        this.distance = distance;
        // this.delaySeconds = secondsToDelay;
        this.requestedSpeed = speed;
    }

    public ClimberCmd(ClimbMode mode) {
        this.mode = mode;
        this.distance = 0;
        this.delayTicks = pneumaticDelayTicks;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        if (Robot.climber == null) {
            logf("Climber not setup mode:%s dist:%.2f\n", mode, distance);
            return; // If no climber return
        }
        if (Robot.climber.getAbortClimb()  && mode != ClimbMode.CLEAR_ABORT_FLAG)   {
            return;
        }
       
        logf("Start Climber mode:%s distance:%.2f req speed:%.2f delay:%d\n", mode, distance, requestedSpeed, delay);
        Robot.climber.setRequestedSpeed(requestedSpeed);
        delay = delayTicks;
        switch (mode) {
            case IDLE:
                break;
            case HOME:
                Robot.climber.homeClimber();
                Robot.climber.setAbortClimb(false);
                break;
            case DISTANCE:
                Robot.climber.setDistanceFromCommand(distance);
                break;
            case STATIC_UP:
                Robot.climber.staticUp();
                delay = pneumaticDelayTicks;
                break;
            case STATIC_DOWN:
                Robot.climber.staticDown();
                delay = pneumaticDelayTicks;
                break;
            case DYNAMIC_UP:
                Robot.climber.dynamicUp();
                delay = pneumaticDelayTicks;
                break;
            case DYNAMIC_DOWN:
                Robot.climber.dynamicDown();
                delay = pneumaticDelayTicks;
                break;
            case BAR_RELEASE_IN:
                Robot.climber.barReleaseIn();
                delay = pneumaticDelayTicks;
                break;
            case BAR_RELEASE_OUT:
                Robot.climber.barReleaseOut();
                delay = pneumaticDelayTicks;
                break;
            case ABORT_CLIMB:
                Robot.climber.setAbortClimb(true);
                break;
            case CLEAR_ABORT_FLAG:
                Robot.climber.setAbortClimb(false);
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
        logf("End Climber CMD End mode:%s dist:%.2f\n", mode, distance);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // See if time has exceeded the set time -- used mainly for safety
        // if (delay > 0) {
        // delay--;
        // return delay <= 0;
        // }

        if (Robot.climber.getAbortClimb() || !Robot.config.climber)   {
            return true;
        }

        if (mode == ClimbMode.DISTANCE) {
            if (Robot.climber.atDistance()) {
                return true;
            }
            return false;
        }
        if (mode == ClimbMode.HOME) {
            return Robot.climber.isHome();
        }
        return true;
    }

}
