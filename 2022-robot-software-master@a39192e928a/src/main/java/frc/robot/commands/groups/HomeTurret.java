package frc.robot.commands.groups;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DisplayLog;
import frc.robot.commands.ShooterCmd;
import frc.robot.commands.ShooterCmd.ShooterMode;


public class HomeTurret extends SequentialCommandGroup {
  double standardDelay = 10;
  double pneumaticDelay = .1;

  public HomeTurret() {
        addCommands( 
          new DisplayLog("Start Home Turret"),
          new ShooterCmd(ShooterMode.HOMETurret),
          new ShooterCmd(0),
          new DisplayLog("End Home Turret")
        );
  }
}