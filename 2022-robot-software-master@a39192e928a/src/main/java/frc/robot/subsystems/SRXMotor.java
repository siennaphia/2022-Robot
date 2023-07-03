package frc.robot.subsystems;

import static frc.robot.Robot.count;
import static frc.robot.Robot.logging;
import static frc.robot.utilities.Util.logf;
import static frc.robot.utilities.Util.round2;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

// setSensorPhase() should change the direction reported in the getSelectedSensor*() methods 
// (but not the SensorCollection methods).
// It should also change the direction reported for "PID0" in a self-test snapshot, 
// but not the position reported by "Quad/MagEnc(rel)" 

public class SRXMotor {
    private TalonSRX motor;
    private TalonSRX followMotor;
    private String name;
    private int id;
    private int followId;
    private double lastSpeed = 0;
    private int lastPos = 0;
    boolean myLogging = false;
    public ErrorCode errorCode;
    public ErrorCode errorCodeFollow;

    private boolean sensorPhase = false;
    private boolean motorInvert = false;
    private FeedbackDevice feedBackDevice = FeedbackDevice.QuadEncoder;

    SRXMotor(String name, int id, int followId, boolean logging) {
        this.name = name;
        this.id = id;
        this.followId = followId;
        myLogging = logging;
        motor = new TalonSRX(this.id);
        errorCode = motor.configFactoryDefault();
        if (errorCode != ErrorCode.OK) {
            logf("????????? Motor %s Error: %s ??????????\n", name, errorCode);
        }
        if (followId > 0) {
            followMotor = new TalonSRX(followId);
            errorCode = followMotor.configFactoryDefault();
            followMotor.follow(motor);
            if (errorCode != ErrorCode.OK) {
                logf("????????? Follow Motor %s Error: %s ??????????\n", name, errorCode);
                followId = -followId;
            }

        }

        motor.getSensorCollection().setQuadraturePosition(0, 0);
        if (followId > 0)
            logf("Created %s motor ids:<%d,%d> firmware:<%d,%d> voltage:<%.1f,%.1f>\n", name, id, followId,
                    motor.getFirmwareVersion(), followMotor.getFirmwareVersion(), motor.getBusVoltage(),
                    followMotor.getBusVoltage());
        else
            logf("Created %s motor id:%d firmware:%d voltage:%.1f\n", name, id, motor.getFirmwareVersion(),
                    motor.getBusVoltage());

    }

    public int getPos() {
        return (int) motor.getSelectedSensorPosition();
    }

    void enableLimitSwitch(boolean forward, boolean reverse) {
        if (forward)
            motor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
        if (reverse)
            motor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
    }

    boolean getForwardLimitSwitch() {
        return motor.getSensorCollection().isFwdLimitSwitchClosed();
    }

    boolean getReverseLimitSwitch() {
        return motor.getSensorCollection().isRevLimitSwitchClosed();
    }

    void setBrakeMode(boolean mode) {
        motor.setNeutralMode(mode ? NeutralMode.Brake : NeutralMode.Coast);
        if (followId > 0)
            followMotor.setNeutralMode(mode ? NeutralMode.Brake : NeutralMode.Coast);
    }

    void setPos(double position) {
        // logf("!!!! SetPos %s to %.1f\n", name, position);
        motor.set(ControlMode.Position, position);
    }

    void setVelocity(double velocity) {
        logf("!!!! Set Velocity for %s to %.0f\n", name, velocity);
        motor.set(ControlMode.Velocity, velocity);
    }

    public void setInverted(boolean invert) {
        motor.setInverted(invert);
        if (followId > 0) {
            followMotor.setInverted(invert);
        }
    }

    public double getLastSpeed() {
        return lastSpeed;
    }

    public double getActualSpeed() {
        return motor.getSelectedSensorVelocity(0);
    }

    public TalonSRX getMotor() {
        return motor;
    }

    void periodic() {
        if (!Robot.config.showMotorData)
            return;
        if (count % 50 == 0 && logging) {
            logPeriodic();
        }
        if (count % 500 == 0)
            updateSmart();
    }

    public void logPeriodic() {
        int pos = motor.getSensorCollection().getQuadraturePosition();
        if (pos != lastPos) {
            lastPos = pos;
            if (myLogging) {
                if (followId > 0) {
                    logf("%s motor sp:%.2f cur:%.2f temp:%.2f vel:%.2f pos:%.0f\n", name, motor.getMotorOutputPercent(),
                            motor.getStatorCurrent(), motor.getTemperature(), motor.getSelectedSensorVelocity(), pos);
                    logf("%s follow sp:%.2f cur:%.2f temp:%.2f vel:%d pos:%d\n", name,
                            followMotor.getMotorOutputPercent(), followMotor.getStatorCurrent(),
                            followMotor.getTemperature(), followMotor.getSelectedSensorVelocity(),
                            followMotor.getSensorCollection().getQuadraturePosition());
                } else {
                    logf("%s motor sp:%.2f cur:%.2f temp:%.2f vel:%d pos:%d\n", name, motor.getMotorOutputPercent(),
                            motor.getStatorCurrent(), motor.getTemperature(), motor.getSelectedSensorVelocity(),
                            motor.getSensorCollection().getQuadraturePosition());
                }
            }
        }

    }

    public void setCurrentLimit(int peakAmps, int continousAmps, int durationMilliseconds) {
        /*
         * Peak Current and Duration must be exceeded before current limit is activated.
         * When activated, current will be limited to Continuous Current. Set Peak
         * Current params to 0 if desired behavior is to immediately current-limit.
         */
        // talon.configPeakCurrentLimit(35, 10); /* 35 A */
        // talon.configPeakCurrentDuration(200, 10); /* 200ms */
        // talon.configContinuousCurrentLimit(30, 10); /* 30
        motor.configPeakCurrentLimit(peakAmps, Robot.config.kTimeoutMs);
        motor.configPeakCurrentDuration(durationMilliseconds, Robot.config.kTimeoutMs);
        motor.configContinuousCurrentLimit(continousAmps, Robot.config.kTimeoutMs);
        motor.enableCurrentLimit(true); // Honor initial setting
    }

    public void updateSmart() {
        SmartDashboard.putNumber(name + " Pos", (int) motor.getSensorCollection().getQuadraturePosition());
        SmartDashboard.putNumber(name + " Cur", round2(motor.getStatorCurrent()));
    }

    public void setSpeed(double speed) {
        if (speed != lastSpeed) {
            motor.set(ControlMode.PercentOutput, speed);
            lastSpeed = speed;
        }
    }

    public void stopMotor() {
        motor.set(ControlMode.PercentOutput, 0);
    }

    void zeroEncoder() {
        motor.setSelectedSensorPosition(0);
    }

    void setEncoderPosition(double position) {
        motor.getSensorCollection().setQuadraturePosition((int) position, Robot.config.kTimeoutMs);
    }

    void setPositionPID(PID pid, FeedbackDevice feedBack) {
        feedBackDevice = feedBack;
        setPositionPID(motor, 0, pid);
        PIDToSRX(motor, pid, 0, Robot.config.kTimeoutMs);
    }

    void setVelocityPID(PID pid) {  // tod at some point fix this name
        PIDToSRX(motor, pid, 0, Robot.config.kTimeoutMs);
    }

    double getMotorVoltage() {
        return motor.getMotorOutputVoltage();
    }

    public void PIDToSRX(TalonSRX srx, PID pid, int slot, int timeout) {
        srx.config_kP(slot, pid.kP, timeout);
        srx.config_kI(slot, pid.kI, timeout);
        srx.config_kD(slot, pid.kD, timeout);
        srx.config_kF(slot, pid.kFF, timeout);
        srx.config_IntegralZone(slot, (int) pid.kIz, timeout);
        srx.configAllowableClosedloopError(slot, pid.allowableCloseLoopError, timeout);
        srx.configMaxIntegralAccumulator(slot, pid.maxIntegralAccumulation, timeout);
        logf("Setup %s PID for %s slot %d %s\n", pid.name, name, slot, pid.getPidData());
    }

    public double getError() {
        return motor.getClosedLoopError(0);
    }

    public void logMotorVCS() {
        if (Math.abs(lastSpeed) > .02) {
            logf("%s\n", getMotorsVCS(motor));
            if (followId > 0) {
                logf("%s\n", getMotorsVCS(followMotor));
            }
        }
    }

    public String getMotorsVCS(TalonSRX motor) {
        if (Math.abs(lastSpeed) > .02) {
            double bussVoltage = motor.getBusVoltage();
            double outputVoltage = motor.getMotorOutputVoltage();
            double supplyCurrent = motor.getSupplyCurrent();
            double statorCurrent = motor.getStatorCurrent();
            return String.format("%s motor volts<%.2f:%.2f> cur<%.2f:%.2f> power<%.2f:%.2f> sp:%.3f", name, bussVoltage,
                    outputVoltage, supplyCurrent, statorCurrent, bussVoltage * supplyCurrent,
                    outputVoltage * statorCurrent, lastSpeed);
        }
        return name + "Not Running";
    }

    public double getMotorCurrent() {
        return motor.getStatorCurrent();
    }

    public void setSensorPhase(boolean phase) {
        sensorPhase = phase;
        motor.setSensorPhase(phase);
    }

    private void setPositionPID(TalonSRX talon, int pidIdx, PID pid) {
        // Config the sensor used for Primary PID and sensor direction
        talon.configSelectedFeedbackSensor(feedBackDevice, pidIdx, Robot.config.kTimeoutMs);

        // Ensure sensor is positive when output is positive
        talon.setSensorPhase(sensorPhase);

        // Set based on what direction you want forward/positive to be.
        // This does not affect sensor phase.

        // TODO will need to do for follow motor -- take out and check climber code
        talon.setInverted(motorInvert);

        /* Config the peak and nominal outputs, 12V means full */
        talon.configNominalOutputForward(0, Robot.config.kTimeoutMs);
        talon.configNominalOutputReverse(0, Robot.config.kTimeoutMs);
        talon.configPeakOutputForward(pid.kMaxOutput, Robot.config.kTimeoutMs);
        talon.configPeakOutputReverse(pid.kMinOutput, Robot.config.kTimeoutMs);

        // Config the allowable closed-loop error, Closed-Loop output will be neutral
        // within this range. See Table in Section 17.2.1 for native units per rotation.
        talon.configAllowableClosedloopError(0, pidIdx, Robot.config.kTimeoutMs);
    }
}