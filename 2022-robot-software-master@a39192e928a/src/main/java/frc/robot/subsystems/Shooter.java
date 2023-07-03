package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.commands.groups.Test;

import static frc.robot.utilities.Util.logf;
import static frc.robot.utilities.Util.round2;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

// 

// 
public class Shooter extends SubsystemBase {
  private SRXMotor shooterMotor;
  private double lastSpeed = 0;
  // private SRXMotor deflectorMotor;
  private SRXMotor turretMotor;
  private SRXMotor backspinMotor;
  private double turretAngle = 0;
  private double lastTurretAngle = 0;
  private boolean turretHomed = false;
  private double lastTurretSpeed = 0;
  private PID turretPID;
  private PID shooterPID; // PID for shooter velocity
  private PID backspinPID; // PID for backspin wheel velocity
  private int turretTicks = 0;
  private double turretTotalAngle = 111.5;
  private boolean turretHomingAtStart = false;
  private double lastShooterVelocity;

  // Current threshold to trigger current limit
  private int kPeakCurrentAmps = 10;
  // Duration after current exceed Peak Current to trigger current limit
  private int kPeakTimeMs = 0;
  // Current to mantain once current limit has been triggered
  private int kContinCurrentAmps = 5;

  public Shooter() {
    // logf("Start of Shooter Subsystem\n");
    // targetingLight = new DigitalOutput(1);

    shooterMotor = new SRXMotor("Shooter", Robot.config.shooterID, -1, true);
    backspinMotor = new SRXMotor("Backspin", Robot.config.backspinID, -1, true);

    // shooterPID = new PID("Shooter", 0.045, .00007, .7, 0, 0, -1, 1, false); //
    // Seems to work still a lot of oscillations

    shooterPID = new PID("Shooter", .35, .00002, .5, 0, 0, -1, 1, false); // With overshoot 0.00007 .7
    shooterMotor.setSensorPhase(false);
    shooterMotor.setVelocityPID(shooterPID);
    shooterMotor.setBrakeMode(false);

    if (Robot.config.backspinID > 0) {
      backspinPID = new PID("Backspin", .35, 0.00002, 0.5, 0, 0, -1, -1, false);
      backspinMotor.setSensorPhase(true);
      backspinMotor.setVelocityPID(backspinPID);
      backspinMotor.setBrakeMode(false);
    }

    // shooterMotor.setCurrentLimit(kPeakCurrentAmps, kContinCurrentAmps,
    // kPeakTimeMs);

    // Set parameters for the turret motor
    turretMotor = new SRXMotor("Turret", Robot.config.turretID, -1, true);
    turretMotor.enableLimitSwitch(true, true);
    turretMotor.setBrakeMode(false);
    turretMotor.zeroEncoder();
    turretPID = new PID("Turret", 0.3, 0, 0, 0, 0, -1, 1, false); // Setup pid
    turretMotor.setPositionPID(turretPID, FeedbackDevice.QuadEncoder); // set pid for SRX
    turretMotor.setCurrentLimit(kPeakCurrentAmps, kContinCurrentAmps, kPeakTimeMs);
    turretMotor.setSensorPhase(true);
    turretHomed = false;

  }

  @Override
  // This method will be called once per scheduler run
  // Make sure that you declare this subsystem in RobotContainer.java
  public void periodic() {
    if (Robot.count == 50) {
      Robot.visionLight.targetLight(false);
    }
    if (Robot.count == 200) {
      logf("----  Schedule Test Command  ----\n");
      Command t = new Test();
      new ScheduleCommand(t);
    }

    // Get shooter test speed from Right joystick buttons
    double speed = -1;
    double[] ar = { 0, .33, .55, .6, .7};
    for (int i = 7; i <= 11; i++) {
      boolean button = Joysticks.rightJoy.getRawButton(i);
      if (button) {
        speed = ar[i - 7];
      }
    }

    // If requesting a speed on the shooter set target light (green light) to on
    if (speed != -1) {
      Robot.visionLight.targetLight(speed > 0);
      setShooterSpeed(speed);
    }

    // Perform homing actions for turret motor at 2 seonds after start up
    if (Robot.count == 100 && !turretHomed && Robot.config.AutoHome) {
      setTurretSpeed(-0.2);
      turretHomingAtStart = true;
    }
    if (turretHomed && turretHomingAtStart) {
      turretHomingAtStart = false;
      setTurretAngle(0);
      logf("!!!!! Turret Automatically homed\n");
    }

    if (turretMotor.errorCode == ErrorCode.OK) {
      turretAngle = range(-Joysticks.operator.getRawAxis(4), -1, 1, -30, 30);
      boolean rightLimit = turretMotor.getForwardLimitSwitch();
      boolean leftLimit = turretMotor.getReverseLimitSwitch();
      if (leftLimit) {
        turretMotor.zeroEncoder();
        turretHomed = true;
      }
      double angle = -turretMotor.getPos() / Robot.config.turretTicksPerDegree + turretTotalAngle / 2;
      if (Robot.count % 50 == 17) {
        SmartDashboard.putNumber("Turret Ang", round2(angle));
      }

      double joy = Joysticks.operator.getRawAxis(4);
      if (Math.abs(joy) < .2) {
        joy = 0;
      }

      // TODO setTurretSpeed(joy) test
      // turretAngle = joy * 20;

      if (turretAngle != lastTurretAngle) {
        turretTicks = (int) (turretAngle * Robot.config.turretTicksPerDegree);
        // turretMotor.getMotor().set(ControlMode.Position, turretTicks);

        logf("Turret Angle:%.2f Requested Ticks:%d actual Ticks:%d homed:%b limit:<L:%b,R:%b>\n", angle, turretTicks,
            turretMotor.getPos(), turretHomed, leftLimit, rightLimit);
        lastTurretAngle = turretAngle;
      }
    }
    if (Robot.count % 500 == Robot.config.shooterID * 9 && getShooterSpeed() > 0) {
      logf("Shooter speed:%.2f req:%.2f turret pos:%d req:%d angle:%.2f\n",
          getShooterSpeed(), lastSpeed,
          turretMotor.getPos(), turretTicks, getTurrentAngle());
    }
    if (Robot.count % 15 == 6) {
      SmartDashboard.putNumber("Sh Sp", getShooterSpeed());
      if (Robot.config.backspinID > 0) {
        SmartDashboard.putNumber("Bs Sp", getBackSpinSpeed());
      }
    }
  }

  public void setShooterSpeed(double speed) {
    if (lastSpeed == speed) {
      return;
    }
    logf("New Shooter speed:%.2f last:%.2f\n", speed, lastSpeed);
    shooterMotor.setSpeed(speed);
    lastSpeed = speed;
  }

  public void setShooterVelocity(double velocity) {
    lastShooterVelocity = velocity;
    shooterMotor.setVelocity(velocity);
    if (Robot.config.backspinID > 0) {
      Robot.config.backspinRatio = SmartDashboard.getNumber("Backspin Ratio", 1);
      SmartDashboard.putNumber("Backspin Ratio", Robot.config.backspinRatio);
      logf("------ Back Spin Ratio %.2f\n", Robot.config.backspinRatio);
      backspinMotor.setVelocity(velocity * Robot.config.backspinRatio);
    }
  }

  public double getShooterSpeed() {
    return shooterMotor.getActualSpeed();
  }

  public double getBackSpinSpeed() {
    return backspinMotor.getActualSpeed();
  }

  double range(double val, double fromMin, double fromMax, double toMin, double toMax) {
    return (val - fromMin) * (toMax - toMin) / (fromMax - fromMin) + toMin;
  }

  public boolean getTurretHomed() {
    return turretHomed;
  }

  public double getTurrentAngle() {
    double angle = -turretMotor.getPos() / Robot.config.turretTicksPerDegree + turretTotalAngle / 2;
    return angle;
  }

  public void setTurretSpeed(double speed) {
    if (speed == lastTurretSpeed) {
      // return;
    }
    logf("Set new turret speed:%.2f\n", speed);
    turretMotor.getMotor().set(ControlMode.PercentOutput, speed);
    lastTurretSpeed = speed;
  }

  public double getRequestedSpeed() {
    return lastTurretSpeed;
  }

  public double getRequestedVelocity() {
    return lastShooterVelocity;
  }

  public void setTurretAngle(double angle) {
    // if (angle != lastTurretAngle) {
    turretTicks = (int) ((-angle + turretTotalAngle / 2) * Robot.config.turretTicksPerDegree);
    // logf("Set turret angle:%.2f ticks:%d\n", angle, turretTicks);
    turretMotor.getMotor().set(ControlMode.Position, turretTicks);
    // }
  }

  public void stopShooter() {
    shooterMotor.stopMotor();
    if (Robot.config.backspinID > 0) {
      backspinMotor.stopMotor();
    }
  }

  public void shooterOut() {
    logf("Start shooter reverse\n");
    shooterMotor.setVelocity(-10020); //arbitrary rpm to spit out ball if stuck
  }

}