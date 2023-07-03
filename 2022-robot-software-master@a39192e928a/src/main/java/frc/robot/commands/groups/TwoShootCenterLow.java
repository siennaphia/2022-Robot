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

public class TwoShootCenterLow extends SequentialCommandGroup {
    public TwoShootCenterLow() {
        addCommands(
                new DisplayLog("Start Two Shoot Center"),
                new ZeroYaw(),
                new IntakeCmd(IntakeMode.OUT),
                new WaitCommand(0.5),
                new DriveStraight(DriveMode.RELATIVE_INCHES, 54, 0.25, 10),
                new WaitCommand(0.5),
                new TurnTo(TurnMode.RELATIVE, -135.5), // spin around
                new IntakeCmd(IntakeMode.IN),
                new WaitCommand(0.5),
                new DriveStraight(DriveMode.RELATIVE_INCHES, 85, 0.3, 10), // the 85 goes 74 so theres a 11inch linear
                                                                           // difference
                new WaitCommand(1.0),
                new TurnTo(TurnMode.RELATIVE, -85.0),
                new WaitCommand(1.0),
                new ShooterCmd(ShooterMode.SET_SHOOTER_VELOCITY, Robot.config.ShooterSpeedPIDLow),
                new DriveStraight(DriveMode.RELATIVE_INCHES, 59 - 4.5 - 6, 0.3, 10),
                new DriveStraight(DriveMode.RELATIVE_INCHES, 4.5, 0.2, 10), // goes slow up to the tarmac to avoid it
                                                                            // falling out
                new WaitCommand(0.3),
                // 1 new IntakeCmd(IntakeMode.FORWARDINTAKE),
                // 1 new IntakeCmd(IntakeMode.FORWARDINTAKE),
                new ShooterCmd(ShooterMode.SHOOT),
                // 1 new IntakeCmd(IntakeMode.FORWARDINTAKE),
                // 1 new IntakeCmd(IntakeMode.FORWARDINTAKE),
                new WaitCommand(1.5),
                new ShooterCmd(ShooterMode.SHOOT),
                new WaitCommand(1.5),
                // 1 new IntakeCmd(IntakeMode.FORWARDINTAKE),
                // new WaitCommand(0.7),
                new ShooterCmd(ShooterMode.SHOOT),
                new WaitCommand(.75),
                new ShooterCmd(ShooterMode.SET_SHOOTER_VELOCITY, 0),
                new IntakeCmd(IntakeMode.IN),
                new DisplayLog("End Two Shoot Center"));
    }
}