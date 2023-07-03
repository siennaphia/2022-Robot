// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.utilities.Util.logf;
import static frc.robot.utilities.Util.round2;
import static frc.robot.utilities.Util.splashScreen;
//import static frc.robot.utilities.Util.normalizeAngle;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.groups.LowerHub;
import frc.robot.commands.groups.Test;
import frc.robot.commands.groups.TwoShootCenter;
import frc.robot.commands.groups.TwoShootCenterLow;
import frc.robot.commands.groups.TwoShootLeft;
import frc.robot.commands.groups.TwoShootLeftLow;
import frc.robot.commands.groups.TwoShootRight;
import frc.robot.commands.groups.TwoShootRightLow;
import frc.robot.commands.groups.UpperHub;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CompressorController;
import frc.robot.subsystems.DistanceSensors;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Joysticks;
import frc.robot.subsystems.Lidar;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.PhotonVision;
import frc.robot.subsystems.PowerHub;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.VisionLight;
import frc.robot.subsystems.YawProvider;
import frc.robot.utilities.MinMaxAvg;

/**
 * Things to improve
 * - Faster Climb with rear Ultrasonic
 * - More autonomous cases -- shoot 3 balls
 * - Grab From Human player station
 * - Fix second ball problem
 * - Add shooter velocity PID and speed check
 * - Shoot to high goal using limelight vision -- very low priority
 * - Limelight create piplines - Andrew & Keanu
 * - When in shoot position use wheel encoders
 * - Riley Stop targeting when too close to wall
 * - Add Ramp back in -- fails see test results
 * 
 * 
 * 
 * Things to test
 * - Nudges
 * - Ball Collect
 * - Accuracy of shooter velocity pid
 * - Test all drive staight cases
 * - Test ultra sonic sensors
 * - Collect ball using vision added logic to do it only when a target exists.
 * - Try for less swing on climb
 * - Climb abort
 * - Faster Drive add turbo button with turbo button
 * - Test 2 ball collect and shoot
 * 
 * HW Improvements
 * - Improve beater bar -- move it out??
 * 
 * Things to purchase / bring
 * - Mr. Sparky blades ------- <><><><><><
 * - More fusses esp the little ones - ???
 * - LAN coupler
 * - USB Breakouts
 * - Router
 * - Second set of cylinders, find pancakes?
 * - Clevises for all clyinders
 * - Motors for all cases - shooter, intake, climbers, drive
 * 
 */

public class Robot extends TimedRobot {
  public static Config config = new Config();
  // Subsystems:
  // public static DrivetrainSRX drivetrain;
  public static Drivetrain drivetrain;
  // public static Limelight limelight; // = new Limelight();
  public static long count = 1;
  public static boolean logging = false;

  private String version = "2.0";
  public static double frontBallVoltage = 0;
  public static double rearBallVoltage = 0;
  public static double frontDistance = 0;
  public static double rearDistance = 0;
  public static double yaw; // Robots yaw as determined by the active Yaw sensor
  private static long lastMemory;
  public static Joysticks joysticks = new Joysticks();
  public static OI oi;
  public static YawProvider yawNavX;
  private long lastGC = 0;
  private MinMaxAvg loopData = new MinMaxAvg();
  private MinMaxAvg gcData = new MinMaxAvg();

  public static PowerHub powerHub;
  public static Climber climber;
  public static Shooter shooter;
  public static Intake intake;
  public static Limelight limeLight;
  public static CompressorController compressor;
  public static DistanceSensors distanceSensors;
  public static Lidar lidar;
  public static long longestLoopTime = 0;
  public static PneumaticHub pHub;
  public static PhotonVision photonvision;
  public static VisionLight visionLight;
  public static Alliance alliance;
  public static int location;

  // A chooser for autonomous commands
  SendableChooser<Command> autonomousChooser = new SendableChooser<>();

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    alliance = DriverStation.getAlliance();
    location = DriverStation.getLocation();
    logf("Alliance:%s Location:%d Free Mem at start %d\n", alliance.toString(), location, getMemory());

    splashScreen(version);

    drivetrain = new Drivetrain();
    oi = new OI();

    yawNavX = new YawProvider();

    if (Robot.config.PneumaticHUB)
      pHub = new PneumaticHub();

    if (config.PowerDistributionHub) {
      powerHub = new PowerHub();
    }
    if (config.climber) {
      climber = new Climber();
    }
    if (config.shooter) {
      shooter = new Shooter();
    }
    if (config.intake) {
      intake = new Intake();
    }
    if (config.ultraSonicDistance) {
      distanceSensors = new DistanceSensors();
    }
    if (config.enableCompressor) {
      compressor = new CompressorController();
    }

    if (config.cameraServer) {
      CameraServer.startAutomaticCapture();
    }

    if (config.PhotonVision) {
      photonvision = new PhotonVision();
    }

    if (config.LimeLight) {
      limeLight = new Limelight();
      limeLight.setLimelightPipeline();
    }

    if (config.BlinkTarget)
      visionLight = new VisionLight();

    // Add commands to the autonomous command chooser
    autonomousChooser.setDefaultOption("Lower Hub One Shot", new LowerHub());
    autonomousChooser.addOption("Upper Hub One shot", new UpperHub());
    autonomousChooser.addOption("HIGH RIGHT Two Shot", new TwoShootRight());
    autonomousChooser.addOption("HIGH LEFT Two Shot", new TwoShootLeft());
    autonomousChooser.addOption("HIGH CENTER Two Shot", new TwoShootCenter());
    autonomousChooser.addOption("LOW RIGHT Two Shot", new TwoShootRightLow());
    autonomousChooser.addOption("LOW LEFT Two Shot", new TwoShootLeftLow());
    autonomousChooser.addOption("LOW CENTER Two Shot", new TwoShootCenterLow());
    autonomousChooser.addOption("Test Autonomous", new Test());

    // Put the chooser on the dashboard
    SmartDashboard.putData((Sendable) autonomousChooser);

    // logf("Free Mem at end %d\n", getMemory());

  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like diagnostics that you want ran during disabled, autonomous,
   * teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.

    // Note loop stat time
    long loopStart = RobotController.getFPGATime();
    // Get the current robot yaw
    yaw = yawNavX.yaw();
    // Put Yaw on smart dashbaord every 0.5 seconds
    if (count % 25 == 16) {
      SmartDashboard.putNumber("Yaw", round2(yaw));
    }

    // if(count == 30) {
    // for(double d = 179; d <= 181; d+=.1) {
    // logf("d:%.2f r:%.2f\n", d, normalizeAngle(d));
    // }
    // }

    // if(count == 50) {
    // for(double d = -181; d <= -179; d+=.1) {
    // logf("d:%.2f r:%.2f\n", d, normalizeAngle(d));
    // }
    // }

    // TODO should do only every x times
    frontDistance = distanceSensors.getFrontDistance();
    rearDistance = distanceSensors.getRearDistance();

    // Run the tasks
    CommandScheduler.getInstance().run();

    // Show longest loop time over 15 millisecond every second
    long loopTime = RobotController.getFPGATime() - loopStart;
    longestLoopTime = Math.max(longestLoopTime, loopTime);
    // Every second reset max loop time
    if (count % 50 == 16) {
      // Show loop time in milli-seconds
      loopData.AddData(longestLoopTime / 1000.0);
      longestLoopTime = 0;
    }

    // Determine GC Rate
    long mem = getMemory();
    if (mem > lastMemory) {
      // logf("!!!!!! Garbage collection mem:%d last:%d last:%.2f\n", mem, lastMemory,
      // (count - lastGC) * .02);
      gcData.AddData((count - lastGC) * .02);
      lastGC = count;
    }
    lastMemory = mem;

    // Log GC and Max loop data every minute
    if (count % (50 * 60) == 210) {
      logf("Loop Count %s  Garage Collection %s\n", loopData.Show(true), gcData.Show(true));
    }
    //if (config.BlinkTarget && (count % 50 == 0)) {
      //visionLight.toggleTargetingLight();
    //}
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    logf("Robot was disabled\n");
    if (shooter != null) {
      shooter.stopShooter(); //stop shooter from spinning on enable if spinning during disable
      shooter.setShooterVelocity(0); //redundency to ensure velocity number reflects motor
    }
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {

    drivetrain.resetEncoders();
    Command cmd = autonomousChooser.getSelected();
    // schedule the autonomous command
    if (autonomousChooser != null) {
      cmd.schedule();
    }
  }

  @Override
  public void teleopInit() {
    logf("Start of Teleop\n");
    count = 0;  
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    count += 1; // Main counter used to time things in the robot
    if ((count <= 200) && (count % 50 == 10)) {
      if (limeLight != null) {
        limeLight.setLimelightPipeline();
      }
    }
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }

  public Command getAutonomousCommand() {
    return autonomousChooser.getSelected();
  }

  public long getMemory() {
    return Runtime.getRuntime().freeMemory();
  }

}
