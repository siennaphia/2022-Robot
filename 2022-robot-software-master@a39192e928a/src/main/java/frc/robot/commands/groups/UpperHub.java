package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;
import frc.robot.commands.DisplayLog;
import frc.robot.commands.DriveStraight;
import frc.robot.commands.ShooterCmd;
import frc.robot.commands.DriveStraight.DriveMode;
import frc.robot.commands.ShooterCmd.ShooterMode;

public class UpperHub extends SequentialCommandGroup {
    public UpperHub() {
        addCommands(
                new DisplayLog("Start Upper Hub Autonomous"),
                new ShooterCmd(ShooterMode.SET_SHOOTER_VELOCITY, Robot.config.ShooterSpeedPIDHigh),
                new WaitCommand(2),
                new ShooterCmd(ShooterMode.SHOOT),
                new WaitCommand(1.0),
                new DriveStraight(DriveMode.RELATIVE_INCHES, -96, 0.5, 10),
                new ShooterCmd(ShooterMode.SET_SHOOTER_VELOCITY, 0),
                new DisplayLog("End Upper Hub Autonomous"));
    }
}