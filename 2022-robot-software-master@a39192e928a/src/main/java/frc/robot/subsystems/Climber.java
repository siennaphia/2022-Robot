// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.utilities.Util.logf;

import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

// Robot Labs on 3/7/2021
// Tirgger/buttons  -- red and goes out
// Trigger -- green and goes in
// 

// Hone on 3/8/22 
// ID 6 is Right Climb but on the right side of robot
// ID 7 is Left Climb but on the left side of robot
// reverse limit switches on top fire when arms are homed
// forward limit switches on bottom fire when arms extended 
// One button Green and goes out - extend  -- encoders go more negative
// Two buttons Red and goes in - retract -- encoders go more positive
// Home goes out??

public class Climber extends SubsystemBase {
    private ClimbMotor rightLiftMotor;
    private ClimbMotor leftLiftMotor;
    private int lastPov = -1;
    private int homeCount = 0;
    private boolean climberHomed = false;
    private int ticks;
    // private double lastTime = 0;
    private double requestPosition = 0;
    private double currentRequestPos = 0;

    private double velocity = 5.0; // in inches per second
    private final double numberOfTicks = 5; // we issue a move command every 5 ticks (20ms x 5 = 100ms)
    private double deltaPosition = 0; // position increment after numberOfTicks at velocity

    private PneumaticHub pHub;
    private Solenoid staticUp;
    private Solenoid staticDown;
    private Solenoid dynamicUp;
    private Solenoid dynamicDown;
    private Solenoid barReleaseIn;
    private Solenoid barReleaseOut;
    private boolean armed = false;
    private boolean atPosition = false;
    private boolean abortClimb = false; // Set when user wants to abort the climb
    private double requestedSpeed = 0;

    enum ClimbDirection {
        BAR_UP, BAR_DOWN // BAR_UP go positive, BAR_DOWN go negitive
    };

    ClimbDirection direction = ClimbDirection.BAR_DOWN;

    public Climber() {
        logf("Start of Climber Subsystem ticks per inch:%.2f\n", Robot.config.climberTicksPerIn);
        rightLiftMotor = new ClimbMotor("Right Climb", "R Clb", Robot.config.rightClimbMotorId);
        leftLiftMotor = new ClimbMotor("Left Climb", "L Clb", Robot.config.leftClimbMotorId);
        rightLiftMotor.setSensorPhase(true);
        rightLiftMotor.setInverted(false);
        leftLiftMotor.setSensorPhase(true);
        leftLiftMotor.setInverted(true);

        pHub = Robot.pHub;
        staticUp = pHub.makeSolenoid(7);
        staticUp.setPulseDuration(0.15);
        staticUp.set(false);

        staticDown = pHub.makeSolenoid(6);
        staticDown.setPulseDuration(0.15);
        staticDown.set(false);
        staticDown.startPulse();

        dynamicUp = pHub.makeSolenoid(4);
        dynamicUp.setPulseDuration(0.15);
        dynamicUp.set(false);

        dynamicDown = pHub.makeSolenoid(5);
        dynamicDown.setPulseDuration(0.15);
        dynamicDown.set(false);
        // Move both bars down at the start
        dynamicDown.startPulse();
        staticDown.startPulse();

        barReleaseIn = pHub.makeSolenoid(15);
        barReleaseIn.setPulseDuration(0.15);
        barReleaseIn.set(false);

        barReleaseOut = pHub.makeSolenoid(14);
        barReleaseOut.setPulseDuration(0.15);
        barReleaseOut.set(false);

    }

    @Override
    public void periodic() {
        if (Robot.count == 5) {
            // Should move the dynamic bar down at the start of the
            dynamicDown.startPulse();
        }
        if (Robot.count == 7) {
            // Should move the static bar down at the start of the
            staticDown.startPulse();
        }
        if (Robot.count == 9) {
            barReleaseIn.startPulse();
        }
        // This method will be called once per scheduler run
        boolean homeRight = rightLiftMotor.getForwardLimit();
        boolean homeLeft = leftLiftMotor.getForwardLimit();
        if (homeLeft) {
            leftLiftMotor.zeroEncoder();
            leftLiftMotor.setSpeed(0);
        }
        if (homeRight) {
            rightLiftMotor.zeroEncoder();
            rightLiftMotor.setSpeed(0);
        }
        if (homeCount > 0) {
            homeCount--;
            if (homeCount > 0) {
                if (Robot.count % 10 == 0) {
                    logf("Climber being homed count:%d limit L:%b R:%b\n", homeCount, homeLeft, homeRight);
                }
                if (homeRight && homeLeft) {
                    logf("Climber is homed count:%d limit L:%b R:%b\n", homeCount, homeLeft, homeRight);
                    climberHomed = true;
                    lastPov = -2; // Fix for home POV problem
                    homeCount = 0;
                }
            }
            return;
        }
        // Logic to manually control the climber motors
        double joyStickSpeed = -Joysticks.operator.getRawAxis(2);
        boolean reverse = Joysticks.operator.getRawButton(5);
        int pov = Joysticks.operator.getPOV();
        double speed = (reverse) ? joyStickSpeed : -joyStickSpeed;
        speed *= .5;
        if (pov == -1) {
            rightLiftMotor.setSpeed(speed);
            leftLiftMotor.setSpeed(speed);
            // lastPov = -2;
        }
        if (Math.abs(joyStickSpeed) > .05) {
            logf("Manual Control Joy:%.2f distance:%.2f Enc:<L:%d,R:%d> For:<L:%b,R:%b> Rev:<L:%b,R:%b>\n",
                    joyStickSpeed, currentDistance(), leftLiftMotor.getPosition(), rightLiftMotor.getPosition(),
                    leftLiftMotor.getForwardLimit(), rightLiftMotor.getForwardLimit(), leftLiftMotor.getReverseLimit(),
                    rightLiftMotor.getReverseLimit());
            return; // Avoid PID Logic if joy stick is activated
        }
        pov = -1;  // TODO  Disable POV cases until move to Smart Dash
        // Logic to manaully control the height of the climber motors
        if (pov != -1 && pov != lastPov) {
            switch (pov) {
                case 0:
                    requestPosition = 1.0;
                    break;
                case 90:
                    requestPosition = 8;
                    break;
                case 180:
                    requestPosition = 12;
                    break;
                case 270:
                    requestPosition = 29; // Was 26
                    break;
            }
            requestedSpeed = 15;
            armed = true;
            currentRequestPos = currentDistance();
            direction = (requestPosition - currentDistance()) < 0 ? ClimbDirection.BAR_DOWN : ClimbDirection.BAR_UP;
            lastPov = pov;
            // lastTime = RobotController.getFPGATime();

            velocity = requestedSpeed;
            logf("Set new climber req pos:%.2f actual pos:%.2f direction:%s homed:%b velocity:%.2f\n", requestPosition,
                    currentRequestPos, direction, climberHomed, velocity);
        }

        velocity = requestedSpeed;

        // Do a PID at for every x number of inches -- determined by the rate
        // double time = RobotController.getFPGATime() - lastTime;
        if ((Robot.count % numberOfTicks == 0) && armed) { // ...every numberOfTicks, execute a small move
            // how close am I?...
            double leftToGo = (currentRequestPos - requestPosition) * (direction == ClimbDirection.BAR_DOWN ? 1 : -1);
            if (leftToGo > 0.0) { // not there yet...
                // lastTime = RobotController.getFPGATime(); // Consider adding cyletime to rate
                deltaPosition = velocity * (numberOfTicks * 0.020);
                currentRequestPos = currentRequestPos
                        + (direction == ClimbDirection.BAR_DOWN ? -deltaPosition : +deltaPosition);
                setDistanceInches(currentRequestPos);

                logf("Desired pos %.2f, current pos:%.2f left to go:%.2f pitch:%.2f\n",
                        currentRequestPos, currentDistance(), leftToGo, Robot.yawNavX.getPitch());

            } else {
                logf("Got there\n");
                atPosition = true;
                armed = false;
            }
        }

        if (getAbortClimb()) {
            armed = false;
        }

        // int diff = rightLiftMotor.getPosition() - leftLiftMotor.getPosition();
        if (Robot.count % 15 == 0) {
            // Update Smart Dashboard every second
            // SmartDashboard.putNumber("Climb Diff", diff);
            SmartDashboard.putNumber("Climb Dist", currentDistance());
            // SmartDashboard.putNumber("Homed", climberHomed ? 1 : 0);
        }

        if (Robot.count % 100 == 0) { // Print critical parameters every 100 times through the loop which is 5 seconds
            // double leftCurrent = leftLiftMotor.getMotorCurrent();
            // double rightCurrent = rightLiftMotor.getMotorCurrent();
            // logf("Status -- Desired pos %.2f, current pos:%.2f tick diff:%d current
            // <L:%.2f,R:%.2f> power <L:%.2f,R:%.2f>\n",
            // currentRequestPos, currentDistance(), diff, leftCurrent, rightCurrent,
            // leftCurrent * leftLiftMotor.getMotorVoltage(), rightCurrent *
            // rightLiftMotor.getMotorVoltage());
        }

    }

    public void setDistanceFromCommand(double inches) {
        requestPosition = inches;
        armed = true;
        atPosition = false;
        currentRequestPos = currentDistance();
        direction = (requestPosition - currentDistance()) < 0 ? ClimbDirection.BAR_DOWN : ClimbDirection.BAR_UP;
    }

    public void setDistanceInches(double inches) {
        ticks = (int) (-inches * Robot.config.climberTicksPerIn);
        // lastTicks = ticks;
        rightLiftMotor.setDistance(ticks);
        leftLiftMotor.setDistance(ticks);
    }

    public boolean atDistance() {
        return atPosition;
    }

    public void homeClimber() {
        logf("Start Homing of climber\n");
        requestPosition = 0;
        rightLiftMotor.setSpeed(.9);
        leftLiftMotor.setSpeed(.9);
        homeCount = 450;
        climberHomed = false;
    }

    public boolean isHome() {
        return climberHomed;
    }

    public void staticUp() {
        staticUp.startPulse();
    }

    public void staticDown() {
        staticDown.startPulse();
    }

    public void dynamicUp() {
        dynamicUp.startPulse();
    }

    public void dynamicDown() {
        dynamicDown.startPulse();
    }

    public void barReleaseIn() {
        barReleaseIn.startPulse();
    }

    public void barReleaseOut() {
        barReleaseOut.startPulse();
    }

    public double currentDistance() {
        int pos = (leftLiftMotor.getPosition() + rightLiftMotor.getPosition()) / 2;
        return -pos / Robot.config.climberTicksPerIn;
    }

    public void setRequestedSpeed(double speed) {
        logf("SetRequested Speed:%.2f\n", speed);
        requestedSpeed = speed;
    }

    public void setAbortClimb(boolean flag) {
        abortClimb = flag;
        logf("!!!!!!!!!!!!!!  Abort Climb:%b !!!!!!!!!!!!\n", abortClimb);
    }

    public boolean getAbortClimb() {
        return abortClimb;
    }

}