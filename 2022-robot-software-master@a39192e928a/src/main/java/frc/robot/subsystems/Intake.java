// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Robot;
import static frc.robot.utilities.Util.logf;

public class Intake extends SubsystemBase {

    private SRXMotor rightMotor;
    private SRXMotor leftMotor;
    private SRXMotor beaterMotor;
    private PneumaticHub pHub;
    private Solenoid lifterSolenoidForward;
    private Solenoid lifterSolenoidReverse;
    private Solenoid beaterSolenoidForward;
    private Solenoid beaterSolenoidReverse;
    //private int ballLifterCount = 0;
    public boolean beaterBarOut = false; 

    public Intake() {
       // logf("Start of the Intake Subsystem\n");
        rightMotor = new SRXMotor("Right", Robot.config.rightIntakeID, -1, true);
        rightMotor.setInverted(true);

        leftMotor = new SRXMotor("Left", Robot.config.leftIntakeID, -1, true);

        beaterMotor = new SRXMotor("Beater", Robot.config.beaterBarID, -1, true);
        beaterMotor.setInverted(false);
        // Test with setting brake mode
        rightMotor.setBrakeMode(true);
        leftMotor.setBrakeMode(true);

        //pHub = new PneumaticHub();
        pHub = Robot.pHub;
        lifterSolenoidForward = pHub.makeSolenoid(1);
        lifterSolenoidReverse = pHub.makeSolenoid(0);
        lifterSolenoidForward.setPulseDuration(0.15);
        lifterSolenoidReverse.setPulseDuration(0.15);
        lifterSolenoidForward.set(false);
        lifterSolenoidReverse.set(false);

        beaterSolenoidForward = pHub.makeSolenoid(3);
        beaterSolenoidReverse = pHub.makeSolenoid(2);
        beaterSolenoidForward.setPulseDuration(0.15);
        beaterSolenoidReverse.setPulseDuration(0.15);
        beaterSolenoidForward.set(false);
        beaterSolenoidReverse.set(false);
        // Raise beater bar at the start
        beaterSolenoidForward.startPulse();
    }

    @Override
    public void periodic() {
        // Reset the lifter at count == 4 at startup
        if(Robot.count == 4) {
            lifterSolenoidReverse.startPulse();
        }
        SmartDashboard.putBoolean( "beaterOut", beaterBarOut);
        // Wait to drop ball lifter
        // if (ballLifterCount > 0) {
        //     ballLifterCount -= 1;
        //     if (ballLifterCount == 0) {
        //         lifterSolenoidReverse.startPulse();
        //         logf("Retract ball lift\n");
        //     }
        // }
    }

    public void beaterBarOut() {
        logf("Deploy Bar and start intake and beater bar motors\n");
        beaterBarOut = true; 
        beaterSolenoidReverse.startPulse();
        beaterMotor.setSpeed(Robot.config.beaterBarSpeed);
        rightMotor.setSpeed(Robot.config.intakeSpeed);
        leftMotor.setSpeed(Robot.config.intakeSpeed);
    }

    public void ballInStart() {
        rightMotor.setSpeed(Robot.config.intakeSpeed);
        leftMotor.setSpeed(Robot.config.intakeSpeed);
        if (Robot.shooter != null) {
            Robot.shooter.setShooterSpeed(Robot.config.shooterDefaultSpeed);
        }
    }

    public void ballInStop() {
        logf("Stop intake and shooter motors\n");
        rightMotor.setSpeed(0);
        leftMotor.setSpeed(0);
        beaterSolenoidForward.startPulse();
        if (Robot.shooter != null) {
            Robot.shooter.setShooterSpeed(0.0);
        }
    }

    public void beaterBarIn(boolean stopShooter) {
        beaterBarOut = false; 
        logf("Retract Bar and stop motors\n");
        beaterSolenoidForward.startPulse();
        beaterMotor.setSpeed(0);
        rightMotor.setSpeed(0);
        leftMotor.setSpeed(0);
        if (Robot.shooter != null && stopShooter) {
            Robot.shooter.setShooterSpeed(0.0);
        }
    }

    // public void liftBall() {
    //     // Handle the ball lifter plunger
    //     ballLifterCount = Robot.config.ballLiftDelay;
    //     lifterSolenoidForward.startPulse();
    //     logf("Start ball lift\n");
    // }

    public void liftBallForCommand() {
        // Handle the ball lifter plunger
        lifterSolenoidForward.startPulse();
        logf("Start ball lift for command\n");
    }

    public void dropBallLifter() {
        lifterSolenoidReverse.startPulse();
    }

    public void intakeOut() {
        // Reverse the intake to spit out balls
        logf("Start intake reverse\n");
        rightMotor.setSpeed(-1);
        leftMotor.setSpeed(-1);
    }

    public void intakeIn() {
        // Move balls in
        logf("Start intake forward\n");
        rightMotor.setSpeed(Robot.config.intakeSpeed);
        leftMotor.setSpeed(Robot.config.intakeSpeed);
    }

    public void intakeInShooting() {
        // Move balls in
        logf("Start intake forward\n");
        rightMotor.setSpeed(Robot.config.intakeSpeedShooting);
        leftMotor.setSpeed(Robot.config.intakeSpeedShooting);
    }

    public void stopIntake() {
        logf("Stop Intake -- Intake\n");
        rightMotor.setSpeed(0);
        leftMotor.setSpeed(0);
    }
}
