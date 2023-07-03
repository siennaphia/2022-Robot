package frc.robot.commands;

import static frc.robot.utilities.Util.logf;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class ShooterCmd extends CommandBase {

    public enum ShooterMode {
        HOMETurret, HOMEDeflector, RUNNING, STOPPED, SHOOT, TURRETToAngle, SET_SHOOTER_SPEED4, SET_SHOOTER_SPEED5,
        SET_SHOOTER_SPEED6, SHOOTER_OFF, SET_SHOOTER_VELOCITY, REVERSESHOOTER
    };

    private ShooterMode mode = ShooterMode.STOPPED;

    enum State {
        IDLE, HOMINGTURRET, START_SHOOT, INTAKE_REVERSING, WAIT_FOR_SPEED, LIFTING_BALL, MOVE_FRONT_BALL_IN,
        MOVE_FRONT_TO_REAR_AT_START, MOVE_BALL_TO_REAR_AFTER_SHOOT, LET_REAR_SETTLE
    };

    private State state = State.IDLE;
    private int delay = 0; // used to do timing for error time out
    private int overallDelay = 500;
    private boolean intakeReversed = false;
    private boolean sensorsValid = false;
    private double requestedTurretAngle = 0;
    private double data; // Input data received type of data depends on the mode

    public ShooterCmd(ShooterMode mode) {
        // Use requires() here to declare subsystem dependencies
        // addRequirements(Robot.intake);
        this.mode = mode;
        this.requestedTurretAngle = 0;
        // logf("Shooter setup mode:%s\n", mode);
    }

    public ShooterCmd(double turretAngle) {
        // Use requires() here to declare subsystem dependencies
        // addRequirements(Robot.intake);
        this.mode = ShooterMode.TURRETToAngle;
        this.requestedTurretAngle = turretAngle;
        // logf("Shooter setup mode:%s requested angle:%.2f\n", mode,
        // this.requestedTurretAngle);
    }

    public ShooterCmd(ShooterMode mode, double data) {
        this.mode = mode;
        this.data = data;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        if (Robot.shooter == null) {
            logf("!!!!! Shooter not setup mode:%s\n", mode);
            return; // If no shooter return
        }
        logf("Shooter mode:%s\n", mode);
        overallDelay = 500;
        switch (mode) {
            case HOMEDeflector:
                break;
            case HOMETurret:
                state = State.HOMINGTURRET;
                delay = 400; // Give turret 4 seconds or 4 * 50 loops
                overallDelay = 500;
                Robot.shooter.setTurretSpeed(-.2);
                break;
            case RUNNING:
                break;
            case STOPPED:
                break;
            case SHOOT:
                state = State.START_SHOOT;
                // If after 5 seconds the shoot sequence is not complete something is very wrong
                overallDelay = 50 * 5;
                intakeReversed = false;
                sensorsValid = Robot.distanceSensors.rearBallValid() && Robot.distanceSensors.frontBallValid();
                Robot.compressor.stop();
                Robot.intake.beaterBarIn(false);
                break;
            case TURRETToAngle:
                Robot.shooter.setTurretAngle(requestedTurretAngle);
                break;
            case SET_SHOOTER_SPEED4:
                Robot.shooter.setShooterSpeed(0.33); // change to .33 only after a test shot
                break;
            case SET_SHOOTER_SPEED5:
                Robot.shooter.setShooterSpeed(0.55);
                break;
            case SET_SHOOTER_SPEED6:
                Robot.shooter.setShooterSpeed(0.65);
                break;
            case SHOOTER_OFF:
                Robot.shooter.setShooterSpeed(0);
                break;
            case SET_SHOOTER_VELOCITY:
                if (data == 0) {
                    Robot.shooter.stopShooter();
                    break;
                }
                Robot.shooter.setShooterVelocity(data);
                logf("Backspin Ratio %.2f", Robot.config.backspinRatio);
                break;
            case REVERSESHOOTER:
                Robot.shooter.shooterOut();
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
        logf("Shoot sequence ended state:%s overAllDelay:%d\n", state, overallDelay);
        state = State.IDLE;
        if (Robot.compressor != null)
            Robot.compressor.restart();
        if (Robot.intake != null)
            Robot.intake.stopIntake();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        overallDelay--;
        if (overallDelay < 0) {
            logf("!!!!!! Overall shoot or homeing sequnce did not complete in allocated time state:%s\n", state);
            return true;
        }
        boolean rearBall = Robot.distanceSensors.rearBallPresent();
        boolean frontBall = Robot.distanceSensors.frontBallPresent();
        switch (state) {
            case IDLE:
                return true;
            case HOMINGTURRET:
                delay--;
                if (delay < 0) {
                    logf("????? Turret did not reacch home in allocated time\n");
                    return true;
                }
                if (Robot.shooter.getTurretHomed()) {
                    logf("Turret is home\n");
                    return true;
                }
                return false;
            case START_SHOOT:
                // See if ball sensors valid and working
                if (!sensorsValid) {
                    logf("!!!!! Ball Sensors not valid -- results will not be correct");
                }
                // Check to see if balls present
                if (!rearBall && !frontBall) {
                    logf("!!!!! No balls present -- try when balls present\n");
                    return true;
                }
                // If no rear ball try to move front ball to rear
                if (!rearBall) {
                    state = State.MOVE_FRONT_TO_REAR_AT_START;
                    delay = 50; //
                    Robot.intake.intakeIn();
                    logf("No Rear move front to rear\n");
                    return false;
                }
                // If front ball move it away from the rear ball with a quick reverse
                if (Robot.distanceSensors.frontBallPresent() && !intakeReversed) {
                    Robot.intake.intakeOut();
                    state = State.INTAKE_REVERSING;
                    delay = 0;
                    return false;
                }
                // Make sure shooter speed is valid is not wait a bit for the speed to come up
                if (!isShootSpeedValid()) {
                    state = State.WAIT_FOR_SPEED;
                    delay = 10;
                    return false;
                }
                // Ball is present at rear, front ball moved out of the way, shooter speed OK
                // Everything seems OK so lift ball
                Robot.intake.liftBallForCommand();
                state = State.LIFTING_BALL;
                delay = 10;
                return false;
            // Wait while intake reverses to move front ball away from rear
            case INTAKE_REVERSING:
                delay--;
                if (delay < 0) {
                    // logf("Stop intake reverse\n");
                    Robot.intake.stopIntake();
                    intakeReversed = true;
                    state = State.START_SHOOT;
                    return false;
                }
                return false;
            // Wait a bit for the shooter to come up to speed
            case WAIT_FOR_SPEED:
                delay--;
                if (delay < 0) {
                    logf("!!!!! Shooter not up to speed in allocated time\n");
                }
                if (isShootSpeedValid()) {
                    state = State.START_SHOOT;
                }
                return false;
            // Wait for ball lifter to go up
            case LIFTING_BALL:
                delay--;
                if (delay < 0) {
                    // logf("Drop Ball Lifter\n");
                    Robot.intake.dropBallLifter();
                    logf("Drop Ball Lifter State f:%b r:%b Volts f:%.2f r:%.2f\n", frontBall, rearBall,
                            Robot.frontBallVoltage,
                            Robot.rearBallVoltage);
                    if (rearBall) {
                        logf("!!!!! Rear ball present after lift -- possible bad\n");
                    }
                    if (frontBall) {
                        state = State.MOVE_FRONT_BALL_IN;
                        Robot.intake.intakeIn();
                        delay = 300;
                        return false;
                    } else {
                        logf("No Ball in magazine so shoot sequence complete\n");
                        return true;
                    }
                }
                return false;
            // If no ball in rear at start and front ball is present try and move it to rear
            case MOVE_FRONT_TO_REAR_AT_START:
                delay--;
                if (delay < 0) {
                    logf("!!!!! Front Ball unable to move to rear in allocated time\n");
                    return true;
                }
                if (rearBall) {
                    state = State.LET_REAR_SETTLE;
                    delay = 25;
                    // Robot.intake.stopIntake();
                    return false;
                }
                return false;
            case LET_REAR_SETTLE:
                delay--;
                if (delay == 12) {
                    Robot.intake.stopIntake();
                }
                if (delay < 0) {
                    if (rearBall) {
                        state = State.START_SHOOT;
                        logf("Rear Ball Settled\n");
                        return false;
                    }
                }
                return false;
            case MOVE_FRONT_BALL_IN:
                delay--;
                if (delay < 0) {
                    logf("!!!! Move Front Ball in failed");
                    return true;
                }
                if (rearBall) {
                    logf("Ball Made it to rear position f:%b r:%b Volts f:%.2f r:%.2f\n", frontBall, rearBall,
                            Robot.frontBallVoltage, Robot.rearBallVoltage);
                    state = State.MOVE_BALL_TO_REAR_AFTER_SHOOT;
                    delay = 10;
                    return false;
                }
                return false;
            case MOVE_BALL_TO_REAR_AFTER_SHOOT:
                delay--;
                if (delay < 0) {
                    return true;
                }
        }
        return true;
    }

    void setShooterSpeed(double speed) {
        Robot.shooter.setShooterSpeed(speed);
    }

    boolean isShootSpeedValid() {
        double tolerance = 800; // The varation of allowed speed
        double speed = Robot.shooter.getShooterSpeed();
        double requestedVelocity = Robot.shooter.getRequestedVelocity();
        if (Math.abs(speed - requestedVelocity) <= tolerance) {
            return true;
        }

        // Shooter req .33 peed:28165 29365 32503 31439
        // Shooter req .55 speed:54144 52960
        // Shooter req .60 speed 57296
        // Shooter req .70 speed 66760 66496
        // Shooter req .80 speed 73984

        // Shooter data
        // .55 Shoots 88.5"
        // .60 shoots 105"

        logf("Shoot speed too low speed:%.2f requested:%.2f\n", speed, requestedVelocity);
        return false;

    }
}
