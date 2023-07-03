package frc.robot;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.ClimberCmd;
import frc.robot.commands.ClimberCmd.ClimbMode;
import frc.robot.commands.DriveStraight;
import frc.robot.commands.DriveStraight.DriveMode;
import frc.robot.commands.IntakeCmd;
import frc.robot.commands.IntakeCmd.IntakeMode;
import frc.robot.commands.ShooterCmd;
import frc.robot.commands.TestCmd;
import frc.robot.commands.ShooterCmd.ShooterMode;
import frc.robot.commands.TurnTo;
import frc.robot.commands.TurnTo.TurnMode;
import frc.robot.commands.ZeroYaw;
import frc.robot.commands.groups.HomeTurret;
import frc.robot.commands.groups.LiftCmd1Bar;
import frc.robot.commands.groups.LiftCmd2Bar;
import frc.robot.commands.groups.LiftCmd3Bar;
import frc.robot.commands.groups.PrepareClimber;
import frc.robot.subsystems.Joysticks;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {

  static enum Action {
    PRESSED, RELEASED
  }

  private ArrayList<ButtonHandler> buttons = new ArrayList<ButtonHandler>();

  OI() {
    // Joystick buttons
    Joystick left = Robot.joysticks.getLeftJoy();
    Joystick right = Robot.joysticks.getRightJoy();
    Joystick op = Robot.joysticks.getOperatorJoy();

    // Left Joy actions
    new ButtonHandler(left, 3, Action.PRESSED, new TurnTo(TurnMode.RELATIVE, -90), "Nudge 90 Left");
    new ButtonHandler(left, 4, Action.PRESSED, new TurnTo(TurnMode.RELATIVE, 90), "Nudge 90 Right");
    new ButtonHandler(left, 5, Action.PRESSED, new TurnTo(TurnMode.RELATIVE, -2, true), "Nudge 2 Left");
    new ButtonHandler(left, 6, Action.PRESSED, new TurnTo(TurnMode.RELATIVE, 2, true), "Nudge 2 Right");
    new ButtonHandler(left, 7, Action.PRESSED, new LiftCmd3Bar(), "Automatic Lift");
    new ButtonHandler(left, 8, Action.PRESSED, new LiftCmd2Bar(), "Automatic Lift");
    new ButtonHandler(left, 9, Action.PRESSED, new LiftCmd1Bar(), "Automatic Lift");
    new ButtonHandler(left, 10, Action.PRESSED, new PrepareClimber(), "Prepare Climber");
    new ButtonHandler(left, 12, Action.PRESSED, new ClimberCmd(ClimbMode.ABORT_CLIMB), "Abort Climb");
    new ButtonHandler(left, 11, Action.PRESSED, new ClimberCmd(ClimbMode.CLEAR_ABORT_FLAG), "Clear Abort Climb Flag");

    // Right Joy actions
    new ButtonHandler(right, 1, Action.PRESSED, new ShooterCmd(ShooterMode.SHOOT), "Shoot Command");
    // TODO was speed was .4 for competition robot
    new ButtonHandler(right, 2, Action.PRESSED, new DriveStraight(DriveMode.DRIVE_TO_BALL, 120, .4, 10),
        "Drive to ball");
    new ButtonHandler(right, 3, Action.PRESSED, new ClimberCmd(ClimbMode.DYNAMIC_UP), "Dynamic UP");
    new ButtonHandler(right, 4, Action.PRESSED, new ClimberCmd(ClimbMode.DYNAMIC_DOWN), "Dynamic Down");
    new ButtonHandler(right, 5, Action.PRESSED, new ClimberCmd(ClimbMode.STATIC_UP), "Static UP");
    new ButtonHandler(right, 6, Action.PRESSED, new ClimberCmd(ClimbMode.STATIC_DOWN), "Static Down");
    new ButtonHandler(right, 12, Action.PRESSED, new ShooterCmd(ShooterMode.REVERSESHOOTER), "Reverse Shooter"); //reverse shooter wheel in case of jam

    // Operator actions
    new ButtonHandler(op, 1, Action.PRESSED, new IntakeCmd(IntakeMode.OUT), "Intake Out");
    new ButtonHandler(op, 2, Action.PRESSED, new IntakeCmd(IntakeMode.IN), "Intake In");
    new ButtonHandler(op, 3, Action.PRESSED, new ZeroYaw(), "Zero Yaw");
    new ButtonHandler(op, 4, Action.PRESSED, new ClimberCmd(ClimbMode.HOME), "Home Climber");
    // button 5 is being used to manually control the speed of the dynamic climbing arms 
    new ButtonHandler(op, 6, Action.PRESSED, new IntakeCmd(IntakeMode.FORWARDINTAKE), "Forward Intake");
    new ButtonHandler(op, 7, Action.PRESSED, new IntakeCmd(IntakeMode.REVERSEINTAKE), "Reverse Intake");
    new ButtonHandler(op, 8, Action.PRESSED, new PrepareClimber(), "Prepare Climber");
    new ButtonHandler(op, 9, Action.PRESSED, new ShooterCmd(ShooterMode.SHOOT), "Shoot Command");
    // TDOD take speed from config
    new POVButton(op, 0)
        .whenPressed(new ShooterCmd(ShooterMode.SET_SHOOTER_VELOCITY, Robot.config.ShooterSpeedPIDHigh));
    new POVButton(op, 90)
        .whenPressed(new ShooterCmd(ShooterMode.SET_SHOOTER_VELOCITY, Robot.config.ShooterSpeedPIDLow));
    new POVButton(op, 180).whenPressed(new ShooterCmd(ShooterMode.SET_SHOOTER_VELOCITY, 0));
  

    // Smart Dashboard buttons primarly for testing
    if (Robot.config.ShowOnSmart) {
      SmartDashboard.putData("Turn Left 90", new TurnTo(TurnMode.RELATIVE, -90));
      SmartDashboard.putData("Turn Right 90", new TurnTo(TurnMode.RELATIVE, 90));
      SmartDashboard.putData("Turn Left 45", new TurnTo(TurnMode.RELATIVE, -45));
      SmartDashboard.putData("Turn Right 45", new TurnTo(TurnMode.RELATIVE, 45));
      SmartDashboard.putData("Turn Left 25", new TurnTo(TurnMode.RELATIVE, -25));
      SmartDashboard.putData("Turn Right 25", new TurnTo(TurnMode.RELATIVE, 25));
      SmartDashboard.putData("Turn Abs 0", new TurnTo(TurnMode.ABSOLUTE, 0));
      SmartDashboard.putData("Turn Abs 180", new TurnTo(TurnMode.ABSOLUTE, 180));
      SmartDashboard.putData("Turn G CW", new TurnTo(TurnMode.GRID_CW));
      SmartDashboard.putData("Turn G CCW", new TurnTo(TurnMode.GRID_CCW));
      SmartDashboard.putData("For 3'", new DriveStraight(DriveMode.RELATIVE_INCHES,
          36, .3, 5));
      SmartDashboard.putData("Back 3'", new DriveStraight(DriveMode.RELATIVE_INCHES, -36, .3, 5, true));
      SmartDashboard.putData("For 1'", new DriveStraight(DriveMode.RELATIVE_INCHES,
          12, .3, 5, true));
      SmartDashboard.putData("Back 1'", new DriveStraight(DriveMode.RELATIVE_INCHES, -12, .3, 5, true));
      SmartDashboard.putData("For 3'|.3|30D", new DriveStraight(DriveMode.RELATIVE_INCHES, false, 30, 36, .3, 5));
      SmartDashboard.putData("For 1'|.3|30D", new DriveStraight(DriveMode.RELATIVE_INCHES, false, 30, 12, .3, 5));
      SmartDashboard.putData("BarRelOut", new ClimberCmd(ClimbMode.BAR_RELEASE_OUT));
      SmartDashboard.putData("BarRelIn", new ClimberCmd(ClimbMode.BAR_RELEASE_IN));
      SmartDashboard.putData("Home Turret 1", new ShooterCmd(ShooterMode.HOMETurret));
      SmartDashboard.putData("Home Turret - Group", new HomeTurret());

    }

    SmartDashboard.putData("Turret 10", new ShooterCmd(10));
    SmartDashboard.putData("Turret -10", new ShooterCmd(-10));
    SmartDashboard.putData("Turret 0", new ShooterCmd(0));
    SmartDashboard.putData("Test Command", new TestCmd());
    SmartDashboard.putData("Shoot Vel 0", new ShooterCmd(ShooterMode.SET_SHOOTER_VELOCITY, 0));
    SmartDashboard.putData("Shoot Vel 1000", new ShooterCmd(ShooterMode.SET_SHOOTER_VELOCITY, 1000));
    SmartDashboard.putData("Shoot Vel 20000", new ShooterCmd(ShooterMode.SET_SHOOTER_VELOCITY, 20000));
    SmartDashboard.putData("Shoot Vel 30000", new ShooterCmd(ShooterMode.SET_SHOOTER_VELOCITY, 30000));
    SmartDashboard.putData("For 2'|.25|0D", new DriveStraight(DriveMode.RELATIVE_INCHES, 24, .25, 10));

    Robot.config.backspinRatio = SmartDashboard.getNumber("Backspin Ratio", Robot.config.backspinRatio);
    SmartDashboard.putNumber("Backspin Ratio", Robot.config.backspinRatio);

    SmartDashboard.putData("Back 2'|.25|0D", new DriveStraight(DriveMode.RELATIVE_INCHES, -24, .25, 10));
    SmartDashboard.putData("For to 19 in'|.25|0D", new DriveStraight(DriveMode.INCHES_FROM_FRONT_SENSOR, 19, .25, 10));

    // TODO remove these
    SmartDashboard.putData("Turn -179", new TurnTo(TurnMode.RELATIVE, -179));
    SmartDashboard.putData("Turn +179", new TurnTo(TurnMode.RELATIVE, +179));

    // SmartDashboard.putData("Turn -180", new TurnTo(TurnMode.RELATIVE, -180));
    // SmartDashboard.putData("Turn +180", new TurnTo(TurnMode.RELATIVE, +180));
    // SmartDashboard.putData("Turn +190", new TurnTo(TurnMode.RELATIVE, +190));
    // SmartDashboard.putData("Turn -190", new TurnTo(TurnMode.RELATIVE, -190));

  }

  // Left Joy actions
  public boolean driveStraightPressed() {
    if (Joysticks.leftJoy == null)
      return Joysticks.operator.getRawButtonPressed(5);
    return Joysticks.leftJoy.getTriggerPressed();
  }

  public boolean driveStraightReleased() {
    if (Joysticks.leftJoy == null)
      return Joysticks.operator.getRawButtonPressed(5);
    return Joysticks.leftJoy.getTriggerReleased();
  }

  public double driveStraightSpeed() {
    if (Joysticks.leftJoy == null)
      return (Joysticks.operator.getRawAxis(5) + Joysticks.operator.getRawAxis(1)) / 2;
    return (Joysticks.rightJoy.getY() + Joysticks.leftJoy.getY()) / 2;
  }

  public double leftJoySpeed() {
    if (Joysticks.rightJoy == null)
      return -Joysticks.operator.getRawAxis(1)* Robot.config.miniSpeedFactor; ;
    return -Joysticks.leftJoy.getY();
  }

  // Right Joy Actions

  public double rightJoySpeed() {
    if (Joysticks.rightJoy == null)
      return -Joysticks.operator.getRawAxis(5) * Robot.config.miniSpeedFactor;
    return -Joysticks.rightJoy.getY();
  }

  public boolean ballTrackActive() {
    return Joysticks.rightJoy.getRawButton(2);
  }

  // Operator actions
  public boolean clearStickyAndLogCurrents() {
    return Joysticks.operator.getRawButtonPressed(7);
  }

  public boolean beaterBarOut() {
    return Joysticks.operator.getRawButtonPressed(1);
  }

  public boolean beaterBarIn() {
    return Joysticks.operator.getRawButtonPressed(0);
  }

  public boolean turboMode() {
    if (Joysticks.leftJoy == null)
      return false;
    return Joysticks.leftJoy.getPOV() == 0;
  }

  class ButtonHandler {
    int port;
    Joystick joystick;
    int buttonNumber;
    Action act;
    String name;

    private ButtonHandler(Joystick joystick, int buttonNumber, Action act, CommandBase cmd, String name) {
      if (joystick == null)
        return;
      this.joystick = joystick;
      this.buttonNumber = buttonNumber;
      this.act = act;
      this.name = name;
      port = joystick.getPort();
      buttons.add(this);
      JoystickButton button = new JoystickButton(joystick, buttonNumber);
      if (act == Action.PRESSED)
        button.whenPressed(cmd);
      if (act == Action.RELEASED)
        button.whenReleased(cmd);
      // todo took out button.close();
    }

    String getData() {
      return "Button:" + name + " Port:" + port + " Button:" + buttonNumber + " Action:" + act;
    }
  }
}