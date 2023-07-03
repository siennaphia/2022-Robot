package frc.robot.commands.groups;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ClimberCmd;
import frc.robot.commands.DisplayLog;
import frc.robot.commands.Timeout;
import frc.robot.commands.ClimberCmd.ClimbMode;

public class LiftCmd3Bar extends SequentialCommandGroup {
  double standardDelay = 10;
  double pneumaticDelay = .1;

  public LiftCmd3Bar() {
        addCommands( 

          new DisplayLog("Start Auto Climb 3 Bars"),
          // Home and park
          //new ClimberCmd(ClimbMode.HOME),
          //new ClimberCmd(ClimbMode.STATIC_DOWN),
          //new ClimberCmd(ClimbMode.DYNAMIC_DOWN),
          //new ClimberCmd(ClimbMode.BAR_RELEASE_IN),
          //new ClimberCmd(ClimbMode.DISTANCE, 12, 18.0, standardDelay),  // go up to clear the bar
          //new ClimberCmd(ClimbMode.DYNAMIC_UP),
          //new Timeout(2.0),
          new ClimberCmd(ClimbMode.DISTANCE, .5, 6.0, standardDelay),  // lift robot slowly
          new Timeout(.5),
          new ClimberCmd(ClimbMode.STATIC_UP),
          new Timeout(1.0),  // Change .5 to 1.0 for test on 3/20
          new ClimberCmd(ClimbMode.DISTANCE, 4, 18.0, standardDelay),  // raise lifters to clear bar
          new ClimberCmd(ClimbMode.DYNAMIC_DOWN),
          new ClimberCmd(ClimbMode.DISTANCE,32, 18.0, standardDelay),  // "reach for the stars"
          new Timeout(.5),
          new ClimberCmd(ClimbMode.DYNAMIC_UP),
          new Timeout(1.0),
          new ClimberCmd(ClimbMode.DISTANCE, 18, 3.0, standardDelay),  // engage and lift from high bar
          new ClimberCmd(ClimbMode.STATIC_DOWN),
          new Timeout(.5),
          new ClimberCmd(ClimbMode.DISTANCE, 13, 3.0,  standardDelay),  // bring the robot closer to the high bar
          new ClimberCmd(ClimbMode.BAR_RELEASE_OUT),
          new Timeout(.5),
          new ClimberCmd(ClimbMode.BAR_RELEASE_IN),
          // new ClimberCmd(ClimbMode.DISTANCE, 12, standardDelay),
          new ClimberCmd(ClimbMode.DISTANCE,0.5, 6.0, standardDelay),    //complete lift
        
          new ClimberCmd(ClimbMode.STATIC_UP),
          new Timeout(1.0),  // Change .5 to 1.0 for test on 3/20
          // Go to 3rd bar from 2nd 
          new ClimberCmd(ClimbMode.DISTANCE, 6, 10.0, standardDelay),  // raise lifters to clear high bar
          new Timeout(2), //new wait to avoid the swing //ws .5
          new ClimberCmd(ClimbMode.DYNAMIC_DOWN),
          new ClimberCmd(ClimbMode.DISTANCE,30, 15.0, standardDelay),  // reach to next bar
          new Timeout(.5),
          new ClimberCmd(ClimbMode.DYNAMIC_UP),
          new Timeout(1.0),
          new ClimberCmd(ClimbMode.DISTANCE, 18, 3.0, standardDelay),  // engage terrestrial bar
          new ClimberCmd(ClimbMode.STATIC_DOWN),
          new Timeout(0.5),
          new ClimberCmd(ClimbMode.DISTANCE, 13, 3.0,  standardDelay),  // bring the robot closer to the terrestrial bar
         
          new ClimberCmd(ClimbMode.BAR_RELEASE_OUT),
          new Timeout(0.5),  
          new ClimberCmd(ClimbMode.BAR_RELEASE_IN),
          new Timeout(0.5),   
          new ClimberCmd(ClimbMode.DISTANCE,.0, 6.0, standardDelay),  // Why 
          

          new ClimberCmd(ClimbMode.STATIC_UP),
          new Timeout(1.0),  
          // Add to get off bar
          new ClimberCmd(ClimbMode.DISTANCE, 2, 3.0,  standardDelay),
          // After competion new ClimberCmd(ClimbMode.DISTANCE,8, 7, standardDelay),
          new DisplayLog("End Auto Climb"),
          new ClimberCmd(ClimbMode.CLEAR_ABORT_FLAG)
        );
  }
}