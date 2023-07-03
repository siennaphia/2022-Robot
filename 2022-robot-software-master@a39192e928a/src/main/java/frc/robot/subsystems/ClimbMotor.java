package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

public class ClimbMotor extends SubsystemBase {
    private SRXMotor motor;
    private PID pid;

    // Current threshold to trigger current limit
    private int kPeakCurrentAmps = 40;
    // Duration after current exceed Peak Current to trigger current limit
    private int kPeakTimeMs = 0;
    // Current to mantain once current limit has been triggered
    private int kContinCurrentAmps = 20;

    private String name;
    private String shortName;
    public double lastSpeed = 0;
    private int id;

    /** Creates a new subsystem. */
    public ClimbMotor(String name, String shortName, int id) {
        this.name = name;
        this.id = id;
        this.shortName = shortName;
        motor = new SRXMotor(name, id, -1, true);
        motor.enableLimitSwitch(true, true);
        motor.setBrakeMode(true);
        motor.zeroEncoder();
        pid = new PID(this.name, Robot.config.climberP, .003, 0.1, 0, 0, -1, 1, false); 
        // pid = new PID(name, 1, 0, 0, 0, 0, 0, -1, 1, false); // Setup pid
        motor.setPositionPID(pid, FeedbackDevice.QuadEncoder); // set pid for SRX
        motor.setCurrentLimit(kPeakCurrentAmps, kContinCurrentAmps, kPeakTimeMs);
        //logf("Climb Pid:%s for name:%s short:%s\n", pid.getPidData(), this.name, this.shortName);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        // The cliombing motors are controlled by Climber.java
        if (Robot.count % 25 == id * 2) {
            SmartDashboard.putNumber(shortName + " Pos", motor.getPos());
            //SmartDashboard.putNumber(shortName + " Cur", Util.round2(motor.getMotorCurrent()));
        }
        // if (Robot.count % 500 == id * 5) // id * 5 ensures that logs do not happen in same loop
        //     logf("%s speed %.2f Pos:%d Cur:%.2f VoltS:%.2f Limit F:%b R:%b\n", name, lastSpeed, motor.getPosFast(),
        //             motor.getMotorCurrent(), motor.getMotorVoltage(),
        //             motor.getForwardLimitSwitch(), motor.getReverseLimitSwitch());
    }

    void setDistance(int ticks) {
        motor.getMotor().set(ControlMode.Position, ticks);
    }

    void setSpeed(double speed) {
        motor.setSpeed(speed);
        lastSpeed = speed;
    }

    int getPosition() {
        return motor.getPos();
    }

    void zeroEncoder() {
        motor.zeroEncoder();
    }

    void setSensorPhase(boolean val) {
        motor.setSensorPhase(val);
    }

    void setInverted(boolean val) {
        motor.setInverted(val);
    }

    boolean getForwardLimit() {
        return motor.getForwardLimitSwitch();
    }

    boolean getReverseLimit() {
        return motor.getReverseLimitSwitch();
    }

    double getActualSpeed() {
        return motor.getActualSpeed();
    }

    double getMotorCurrent() {
       return motor.getMotorCurrent();
    }

    double getMotorVoltage() {
        return motor.getMotorVoltage();
    }
}