package frc.robot.subsystems;

import static frc.robot.utilities.Util.logf;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Robot;

public class Joysticks {
    public static Joystick leftJoy;
    public static Joystick rightJoy;
    public static Joystick operator;
    private static boolean joysEnabled;
    private static boolean operatorEnabled;

    public Joysticks() {
        joysEnabled = Robot.config.joysticksEnabled;
        if (joysEnabled) {
            leftJoy = new Joystick(0);
            rightJoy = new Joystick(1);
        }
        operatorEnabled = Robot.config.operatorPadEnabled;
        if (operatorEnabled) {
            operator = new Joystick(2);
        }
       
        String joyName0 = DriverStation.getJoystickName(0);
        String joyName1 = DriverStation.getJoystickName(1);
        String operatorName = DriverStation.getJoystickName(2);
        logf("Joysticks Enabled:%b <%s,%s> Operator Enabled:%b %s\n", joysEnabled, joyName0, joyName1, operatorEnabled,
                operatorName);
        // boolean joyCorrect = joyName1.equals(joyName0) && joyName0.equals("Logitech
        // Extreme 3D");
        // Robot.neoPixelControl.setLed(NeoPixelControl.LedAssignment.JoysticksCorrect,
        // joyCorrect ? NeoPixelControl.LedType.GREEN : NeoPixelControl.LedType.RED);
        // boolean opCorrect = operatorName.equals("Controller (Gamepad F310)");
        // Robot.neoPixelControl.setLed(NeoPixelControl.LedAssignment.OperatorCorrect,
        // opCorrect ? NeoPixelControl.LedType.GREEN : NeoPixelControl.LedType.RED);
    }

    public Joystick getRightJoy() {
        return rightJoy;
    }

    public Joystick getLeftJoy() {
        return leftJoy;
    }

    public Joystick getOperatorJoy() {
        return operator;
    }
}