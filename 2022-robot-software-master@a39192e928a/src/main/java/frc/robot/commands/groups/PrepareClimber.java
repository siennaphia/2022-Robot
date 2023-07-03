
package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.ClimberCmd;
import frc.robot.commands.DisplayLog;
import frc.robot.commands.ClimberCmd.ClimbMode;

public class PrepareClimber extends SequentialCommandGroup {

    double standardDelay = 10;
    double pneumaticDelay = .1;

    public PrepareClimber() {
        addCommands(
                new DisplayLog("Start Prepare Climber"),
                // Home and park
                new ClimberCmd(ClimbMode.HOME),
                new ClimberCmd(ClimbMode.STATIC_UP),  // Should remove 
                new ClimberCmd(ClimbMode.DYNAMIC_UP),
                new ClimberCmd(ClimbMode.BAR_RELEASE_IN),
                new WaitCommand(.5),
                new ClimberCmd(ClimbMode.STATIC_DOWN),
                new ClimberCmd(ClimbMode.DISTANCE, 12, 10.0, standardDelay), // go up to clear the bar
                new DisplayLog("End Prepare Climbe"));
    }
}
