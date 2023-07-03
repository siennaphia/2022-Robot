package frc.robot;

import static frc.robot.utilities.Util.logf;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;

public class Config {

    // Default parameters should defaults for the competition Robot

    public enum RobotType {
        MiniSRX, MiniFast, Competition
    };

    // Type of Robot
    public static RobotType robotType = RobotType.Competition;

    // Pneumatic Control Modules Parameters
    public int pcmHubID = 1;

    // Vision Parameters
    public boolean LimeLight = true;
    public boolean opensightVision = false;
    public boolean ledRing = false;

    public enum DriveType {
        None, Tank, OperatorTank,
    };

    // Drive Parameters
    public int driveRight = 2;
    public int driveLeft = 3;
    public int driveRightFollow = 4;
    public int driveLeftFollow = 5;
    public DriveType driveType = DriveType.OperatorTank;
    public double driveTicksPerInch = 987;
    public boolean defaultBrakeMode = true;
    public double wheelBase = 15.5; // Wheel base for mini
    public double wheelDiameter = 6.0; // Wheel diameter for MINI
    public double driveTicksPerRevolution = 2000; // Value for mini
    public double throttleRate = 1.5; // for the throttle power curve
    public boolean showMotorData = false;
    public double rampUp = .05;
    public double rampUpTurbo = .05;
    public double rampDown = .05;
    public double rampDownTurbo = .05;
    public double miniSpeedFactor = 1.0;

    // Climber Parameters
    public boolean climber = true;
    public double climberTicksPerIn = 260 - 10;
    public int rightClimbMotorId = 6;
    public int leftClimbMotorId = 7;
    public double climberP = .6 + 1 + .5 + 2 + 1; // Added .5 3/3/2022 -- added another 1 on 3/15

    // Miscellaneous Parameter
    public boolean ultraSonicDistance = true;
    public boolean ultra1030Front = false;
    public boolean ultra1030Rear = false;
    public int kTimeoutMs = 30; // default timeout used for messages to the SRX
    public boolean enableCompressor = true;
    public boolean cameraServer = true;
    public boolean joysticksEnabled = true;
    public boolean operatorPadEnabled = true;
    public boolean neoPixelsActive = false;
    public double deadZone = 0.085;
    public boolean powerHubToDashBoard = false;
    public int blinkerChannel = 13;

    // Shooter parameters
    public boolean shooter = true;
    public boolean shooterVelocityPID = true;
    public double deflectorTicksPerDegree = 25;
    public double turretTicksPerDegree = 6000;
    public int shooterID = 12;
    public int turretID = 11;
    public int backspinID = 14;
    public int deflectorID = 10;
    public int revRPM = 5676;
    public double shooterDefaultSpeed = .75;
    public double backspinRatio = 1.10;
    public boolean AutoHome = true;
    public double ShooterSpeedPIDLow = 20000;
    public double ShooterSpeedPIDHigh = 33000; // Was 66000 slow it down a bit

    // Intake parameters
    public boolean intake = true;
    public int beaterBarID = 15;
    public int rightIntakeID = 16;
    public int leftIntakeID = 17;
    public int ballLiftDelay = 20;
    public double beaterBarSpeed = 0.75; // Changed from .85 used at WPB, 0.75
    public double intakeSpeed = 0.5; // Changed from 0.6 bc wheels pushing up due to speed
    public double intakeSpeedShooting = 0.9;

    // Misc parameters
    public boolean lidar = false;
    public boolean pigeon = false;
    public boolean BNO055Connected = false;
    public boolean PowerDistributionHub = true;
    public boolean PneumaticHUB = true;
    public boolean PhotonVision = false;
    public boolean ShowOnSmart = false;
    public boolean BlinkTarget = false;

    // Shoot Command parameters
    public double conveyorSpeed = 0.7;
    public double highShootSpeedTop = 1.0;
    public double highShootSpeedBottom = 0.6;
    public double lowShootSpeedTop = 0.3;
    public double lowShootSpeedBottom = 0.3;
    public final double CERTAIN_SPEED = 0.6;

    Config() {
        if (isMini()) {
            robotType = RobotType.MiniFast;
        }
        logf("Start of Robot Config for %s\n", robotType);
        switch (robotType) {
            case MiniSRX:
                joysticksEnabled = false;
                break;
            case MiniFast:
                driveRight = 2;
                driveLeft = 3;
                driveRightFollow = -7;
                driveLeftFollow = -11;
                driveTicksPerInch = (50 * 12) / 16;
                climberP = .1;
                climber = false;
                intake = false;
                PowerDistributionHub = false;
                shooter = false;
                cameraServer = false;
                enableCompressor = false;
                PhotonVision = false;
                BlinkTarget = false;
                AutoHome = false;
                ultra1030Front = true;
                PneumaticHUB = false;
                joysticksEnabled = false;
                miniSpeedFactor = .5;
                break;
            case Competition:
                AutoHome = false;
                // climber = false;
                ultra1030Front = true;
                beaterBarSpeed = 0.75;
                BlinkTarget = true;
                // if (!isBackspin()) {
                // backspinID = -1;
                // }
                break;
        }
    }

    public RobotType getRobotType() {
        return robotType;
    }

    boolean isMini() {
        String fileName = "/home/lvuser/deploy/mini.txt";
        File fin = new File(fileName);
        return fin.exists();
    }

    boolean isBackspin() {
        String fileName = "/home/lvuser/deploy/backspin.txt";
        File fin = new File(fileName);
        return fin.exists();
    }

    void readConfig() {
        String fileName = "/home/lvuser/deploy/parameters.txt";
        File fin = new File(fileName);
        if (!fin.exists()) {
            logf("File %s not found default values assumed", fileName);
            return;
        }
        try {
            BufferedReader br = new BufferedReader(new FileReader(fin));
            String line = null;
            while ((line = br.readLine()) != null) {
                line = line.trim();
                if (!line.startsWith("#")) {
                    String[] ar = line.split("=");
                    if (ar.length >= 2) {
                        String value = ar[1].trim();
                        if (value.contains("#")) {
                            value = value.split("#")[0].trim();
                        }
                        setVariable(ar[0].trim(), value);
                    }
                }
            }
            br.close();
        } catch (FileNotFoundException ex) {
            logf("Unable to open file: %s\n", fin.getName());
        } catch (IOException ex) {
            logf("Error reading file: %s\n", fin.getName());
        }
    }

    boolean setVariable(String variable, String value) {
        try {
            switch (variable.toLowerCase()) {
                case "robottype":
                    robotType = RobotType.valueOf(value);
                    break;
                default:
                    logf("???? Warning Variable:%s not found in parms.txt file\n", variable);
                    return false;
            }
        } catch (Exception ex) {
            logf("Unable to convert %s with a value of %s\n", variable, value);
            return false;
        }
        logf("Variable:%s value:%s\n", variable, value);
        return true;
    }
}