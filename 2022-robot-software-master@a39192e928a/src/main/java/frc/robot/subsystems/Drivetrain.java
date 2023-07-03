package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Config.RobotType;

import static frc.robot.utilities.Util.logf;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

public class Drivetrain extends SubsystemBase {

    private FXMotor rightMotor;
    private FXMotor leftMotor;

    private double rightSpeed = 0;
    private double leftSpeed = 0;
    private boolean brakeMode = true;
    private Double targetAngle = null;
    private double rightJoy;
    private double leftJoy;
    private double yaw;
    private boolean showDriveLog = false;
    private double rightStart = 0;
    private double leftStart = 0;
    private PID positionPID;

    public Drivetrain() {
        rightMotor = new FXMotor("Right", Robot.config.driveRight, Robot.config.driveRightFollow, true);
        leftMotor = new FXMotor("Left", Robot.config.driveLeft, Robot.config.driveLeftFollow, true);
        rightMotor.setBrakeMode(brakeMode);
        leftMotor.setBrakeMode(brakeMode);
        rightMotor.zeroEncoder();
        leftMotor.zeroEncoder();
        rightMotor.setInverted(true);
        leftMotor.setSensorPhase(true);
        rightMotor.setSensorPhase(false);
        if (Robot.config.getRobotType() == RobotType.Competition) {
            // PID for competion robot
            positionPID = new PID("Drive", .4, 0, .02, 0, 0, -1, 1, false);
        } else {
            // PID for Mini Fast test robot
            positionPID = new PID("Drive", 15, 0, 0, 0, 0, -1, 1, false);
        }
        rightMotor.setPositionPID(positionPID, FeedbackDevice.CTRE_MagEncoder_Relative); // set pid for SRX
        leftMotor.setPositionPID(positionPID, FeedbackDevice.CTRE_MagEncoder_Relative); // set pid for SRX
    }

    public void setSpeed(double right, double left) {
        setRightMotor(right);
        setLeftMotor(left);
    }

    public void setLeftMotor(double speed) {
        if (Math.abs(speed) < .05) {
            speed = 0;
        }
        leftMotor.setSpeed(speed);
        leftSpeed = speed;
    }

    public void forcePercentMode() {
        rightMotor.forcePercentMode();
        leftMotor.forcePercentMode();
    }

    public void setRightMotor(double speed) {
        if (Math.abs(speed) < .05)
            speed = 0;
        rightMotor.setSpeed(speed);
        rightSpeed = speed;
    }

    public double getLeftMotorSpeed() {
        return leftSpeed;
    }

    public double getRightMotorSpeed() {
        return rightSpeed;
    }

    boolean turboMode;

    @Override
    public void periodic() {
        yaw = Robot.yaw;
        turboMode = Robot.oi.turboMode();
        if (Robot.oi.driveStraightPressed()) {
            logf("Drive Straight start target yaw:%.2f\n", yaw);
            targetAngle = Robot.yaw;
        }
        if (Robot.oi.driveStraightReleased()) {
            logf("Drive Straight finish target:%.2f yaw:%.2f\n", targetAngle, yaw);
            targetAngle = null;
        }
        double sensitivity = .7;

        if (turboMode) {
            sensitivity = 1.0;
        }

        rightJoy = Robot.oi.rightJoySpeed() * sensitivity;
        leftJoy = Robot.oi.leftJoySpeed() * sensitivity;

        // Insert logic for null zone here

        // Insert logic for acceration and dealeftrlation here

        if (targetAngle != null) {
            // If Drive straight active make adjustments
            driveStraight();
        }
        // Adjust the ramp rate to make it easier to drive
        // Ramping will also put less strain on the motors
        rightJoy = ramp(rightJoy, rightSpeed, "Right");
        leftJoy = ramp(leftJoy, leftSpeed, "Left");
        showDriveLog = Math.abs(rightJoy) > .06 || Math.abs(leftJoy) > .06;
        if (showDriveLog && Robot.count % 100 == 0 && (Math.abs(rightJoy) > .05 || Math.abs(leftJoy) > .05)) {
            logf("Pos L:%d R:%d Joy L:%.2f R:%.2f\n", leftMotor.getPos(), rightMotor.getPos(), leftJoy,
                    rightJoy);
        }
        setLeftMotor(leftJoy);
        setRightMotor(rightJoy);
    }

    void driveStraight() {
        double error = Robot.yaw - targetAngle;
        if (error > 10) {
            logf("!!!!! DS error too positive diff:%.1f yaw:%.1f target:%.3f\n", error,
                    yaw, targetAngle);
            error = 5;
        }
        if (error < -10) {
            logf("!!!!! DS error too negative diff:%.1f yaw:%.1f target:%.3f\n", error,
                    yaw, targetAngle);
            error = -5;
        }
        // Adjsut speed if too fast
        double averageJoy = (rightJoy + leftJoy) / 2;
        // If turbo mode ignore speed limit
        if (!turboMode) {
            if (averageJoy > .5)
                averageJoy = .5;
            if (averageJoy < -.5)
                averageJoy = -.5;
        }
        double factor = error * Math.abs(averageJoy) * 0.035; // Was 0.045
        // Log drive straight data every 2.5 seconds
        if (Robot.count % 50 == 0) {
            logf("Drive Straight targ:%.2f yaw:%.2f err:%.2f avg:%.2f factor:%.2f Fr Dist:%.2f\n",
                    targetAngle, yaw, error,
                    averageJoy, factor, Robot.frontDistance);
        }
        leftJoy = averageJoy - factor;
        rightJoy = averageJoy + factor;
    }

    private boolean rampTest = false;

    public double ramp(double joy, double speed, String side) {
        if (rampTest) {
            return rampExp(joy, speed, side);
        }
        double error = Math.abs(speed - joy);
        if (error < .1)
            return joy;
        if (turboMode) {
            if (joy != speed) {
                if (joy > speed) {
                    joy = speed + Robot.config.rampUp;
                } else {
                    joy = speed - Robot.config.rampDown;
                }
            }
        } else {
            if (joy != speed) {
                if (joy > speed) {
                    joy = speed + Robot.config.rampUp;
                } else {
                    joy = speed - Robot.config.rampDown;
                }
            }
        }

        return joy;
    }

    public double rampExp(double joy, double speed, String side) {
        double error = Math.abs(speed - joy);
        if (error < .1)
            return joy;
        boolean joyIsNegative = false;
        joyIsNegative = joy < 0;
        joy = 0.2 * (Math.exp(1.75 * (Math.abs(joy)) - 1));
        return (joyIsNegative ? -joy : joy);
    }

    public double getLeftEncoder() {
        return leftMotor.getPos();
    }

    public double getRightEncoder() {
        return rightMotor.getPos();
    }

    // Set the initial position of the robot wheels used in later getDistance Inches
    public void setInitialPosition() {
        rightStart = getRightEncoder();
        leftStart = getLeftEncoder();
    }

    // Get the distance the robot has traveled in inches
    public double getDistanceInches() {
        double ticks = (Math.abs(rightStart - getRightEncoder()) + Math.abs(leftStart - getLeftEncoder())) / 2.0;
        return ticks / Robot.config.driveTicksPerInch;
    }

    public double rightDistInches() {
        return (getRightEncoder() - rightStart) / Robot.config.driveTicksPerInch;
    }

    public double leftDistInches() {
        return (getLeftEncoder() - leftStart) / Robot.config.driveTicksPerInch;
    }

    public void setBrakeMode(boolean brakeMode) {
        rightMotor.setBrakeMode(brakeMode);
        leftMotor.setBrakeMode(brakeMode);
    }

    public void setDefaultBrakeMode() {
        setBrakeMode(Robot.config.defaultBrakeMode);
    }

    public void resetEncoders() {
        rightMotor.zeroEncoder();
        leftMotor.zeroEncoder();
    }

    public void setPosition(int rightTicks, int leftTicks) {
        rightMotor.setPos(rightMotor.getPos() + rightTicks);
        leftMotor.setPos(leftMotor.getPos() + leftTicks);
    }

}
