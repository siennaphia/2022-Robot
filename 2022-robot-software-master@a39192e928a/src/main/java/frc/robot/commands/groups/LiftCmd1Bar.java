
package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.commands.ClimberCmd;
import frc.robot.commands.DisplayLog;
import frc.robot.commands.Timeout;
import frc.robot.commands.ClimberCmd.ClimbMode;

public class LiftCmd1Bar extends SequentialCommandGroup {

    double standardDelay = 10;
    double pneumaticDelay = .1;
public  LiftCmd1Bar() {
    addCommands( 
    new DisplayLog("Start Auto Climb 1 Bar"),
    new ClimberCmd(ClimbMode.DISTANCE, .5, 7.0, standardDelay),  // lift robot slowly
    new Timeout(0.5),
    new ClimberCmd(ClimbMode.STATIC_UP),
    new Timeout(1.0),
    new ClimberCmd(ClimbMode.DISTANCE, 2.0, 5.0, standardDelay),  // raise lifters to clear bar - used to be 8 (changed to match cmd2), 4/11 change distance to touch bar
    new DisplayLog("End Auto Climb 1 Bar"),
    new ClimberCmd(ClimbMode.CLEAR_ABORT_FLAG)
    );
}
}