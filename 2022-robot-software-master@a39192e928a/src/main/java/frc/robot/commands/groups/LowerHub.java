package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;
import frc.robot.commands.DisplayLog;
import frc.robot.commands.DriveStraight;
import frc.robot.commands.ShooterCmd;
import frc.robot.commands.DriveStraight.DriveMode;
import frc.robot.commands.ShooterCmd.ShooterMode;

public class LowerHub extends SequentialCommandGroup {
    public LowerHub() {
        addCommands(
                new DisplayLog("Start Lower Hub Autonomous"),
                new ShooterCmd(ShooterMode.SET_SHOOTER_VELOCITY, Robot.config.ShooterSpeedPIDLow),
                new WaitCommand(1.5),
                new ShooterCmd(ShooterMode.SHOOT),
                new WaitCommand(1.0),
                new DriveStraight(DriveMode.RELATIVE_INCHES, -96, 0.5, 10),
                new ShooterCmd(ShooterMode.SET_SHOOTER_VELOCITY, 0),
                new DisplayLog("End Lower Hub Autonomous"));
    }
}
