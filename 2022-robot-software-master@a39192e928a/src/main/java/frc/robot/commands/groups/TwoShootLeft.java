package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;
import frc.robot.commands.DisplayLog;
import frc.robot.commands.DriveStraight;
import frc.robot.commands.IntakeCmd;
import frc.robot.commands.ShooterCmd;
import frc.robot.commands.TurnTo;
import frc.robot.commands.ZeroYaw;
import frc.robot.commands.DriveStraight.DriveMode;
import frc.robot.commands.IntakeCmd.IntakeMode;
import frc.robot.commands.ShooterCmd.ShooterMode;
import frc.robot.commands.TurnTo.TurnMode;

public class TwoShootLeft extends SequentialCommandGroup {
    public  TwoShootLeft() {
        addCommands(
                new DisplayLog("Start Two Shoot"),
                new ZeroYaw(),
                new IntakeCmd(IntakeMode.OUT),
                new WaitCommand(1.5),
                new DriveStraight(DriveMode.RELATIVE_INCHES, 54, 0.25, 10),
                new WaitCommand(1.0),
                new TurnTo(TurnMode.RELATIVE, -179.5),
                new IntakeCmd(IntakeMode.IN),
                new ShooterCmd(ShooterMode.SET_SHOOTER_VELOCITY, Robot.config.ShooterSpeedPIDHigh) ,
                new DriveStraight(DriveMode.RELATIVE_INCHES, 90-4.5, 0.3, 10),
                new DriveStraight(DriveMode.RELATIVE_INCHES, 4.5+4.5, 0.2, 10),
                new WaitCommand(.2),
                new ShooterCmd(ShooterMode.SHOOT),
                new WaitCommand(1.5),
                //new IntakeCmd(IntakeMode.FORWARDINTAKE),           
                //new WaitCommand(2.0),
                new ShooterCmd(ShooterMode.SHOOT),
                new WaitCommand(.5),
                new ShooterCmd(ShooterMode.SET_SHOOTER_VELOCITY, 0),
                new IntakeCmd(IntakeMode.IN),
                new DisplayLog("End Two Shoot Left"));
    }
}