package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.commands.ClimberCmd;
import frc.robot.commands.DisplayLog;
import frc.robot.commands.Timeout;
import frc.robot.commands.ClimberCmd.ClimbMode;

public class LiftCmd2Bar extends SequentialCommandGroup {
    double standardDelay = 10;
  double pneumaticDelay = .1;

  public LiftCmd2Bar() {
        addCommands( 
          new DisplayLog("Start Auto Climb 2 Bar"),
          // Home and park
         // new ClimberCmd(ClimbMode.HOME),
          //new ClimberCmd(ClimbMode.STATIC_DOWN),
          //new ClimberCmd(ClimbMode.DYNAMIC_DOWN),
          //new ClimberCmd(ClimbMode.BAR_RELEASE_IN),
          //new ClimberCmd(ClimbMode.DISTANCE, 12, 10.0, standardDelay),  // go up to clear the bar
         // new ClimberCmd(ClimbMode.DYNAMIC_UP),
          //new Timeout(1.0),
          new ClimberCmd(ClimbMode.DISTANCE, .5, 6.0, standardDelay),  // lift robot slowly
          new Timeout(.1),
          new ClimberCmd(ClimbMode.STATIC_UP),
          new Timeout(1.3),
          new ClimberCmd(ClimbMode.DISTANCE, 4, 24.0, standardDelay),  // raise lifters to clear bar
          new ClimberCmd(ClimbMode.DYNAMIC_DOWN),
          new Timeout(0.5),
          new ClimberCmd(ClimbMode.DISTANCE,34, 24.0, standardDelay),  // "reach for the stars"
          new Timeout(.1), //on climb 3, its 1.1
          new ClimberCmd(ClimbMode.DYNAMIC_UP),
          new Timeout(1.0),
          new ClimberCmd(ClimbMode.DISTANCE, 18, 3.0, standardDelay), //Try change to 18 3 if release to early
          new Timeout(.5),  // engage and lift from high bar
          new ClimberCmd(ClimbMode.STATIC_DOWN),
          new Timeout(1.0),
          new ClimberCmd(ClimbMode.DISTANCE, 13, 3.0,  standardDelay),  // bring the robot closer to the high bar // try 13 3 if release to early
          new ClimberCmd(ClimbMode.BAR_RELEASE_OUT),
          // new ClimberCmd(ClimbMode.DISTANCE, 12, standardDelay),
          new ClimberCmd(ClimbMode.DISTANCE, 0.5, 6.0, standardDelay),    //complete lift
          new ClimberCmd(ClimbMode.BAR_RELEASE_IN),
          new Timeout(1.0),
          new ClimberCmd(ClimbMode.STATIC_UP),
          new Timeout(1.0),
          new ClimberCmd(ClimbMode.DISTANCE, 1.5, 3.0,  standardDelay),  // Change fromm 3 to 1.5 on 3/20/22
          new DisplayLog("End Auto Climb 2 Bar"),
          new ClimberCmd(ClimbMode.CLEAR_ABORT_FLAG)
        );
    
    }
    }