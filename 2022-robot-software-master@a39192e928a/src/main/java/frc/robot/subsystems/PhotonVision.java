package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import static frc.robot.utilities.Util.logf;

public class PhotonVision extends SubsystemBase {

    // Change this to match the name of your camera
    PhotonCamera camera = new PhotonCamera("photonvision");
    double angle = 0;
    double lastAngle = 0;
    boolean noCamera = true;


    public PhotonVision() {
    }

    
    @Override
    public void periodic() {
        PhotonPipelineResult result = null;
        try {
            result = camera.getLatestResult();
        } catch (Exception e) {
            if (Robot.count % 500 == 0)
                logf("!!!!!!!!!!!  No Vision Camera\n");
            return;
        }
        boolean pressed = Joysticks.rightJoy.getRawButtonPressed(2);
        if (result.hasTargets() && Robot.shooter.getTurretHomed() && pressed) {
            // Calculate angle to target
            angle = result.getBestTarget().getYaw();
            double turretAngle = Robot.shooter.getTurrentAngle();
            double newTurretAngle = turretAngle + angle;
            if (Math.abs(newTurretAngle) > 25) {
                Robot.shooter.setTurretAngle(0);
                logf("!!!!!! Turret Angle out side range new:%.2f vision angle:%.2f turret angle%.2f\n", newTurretAngle,
                        angle, turretAngle);
                return;
            }
            logf("Angle from targeting camera angle:%.2f turret angle:%.2f new angle:%.2f\n", angle,
                    lastAngle, newTurretAngle);
            Robot.shooter.setTurretAngle(newTurretAngle);
            lastAngle = newTurretAngle;
        } else {
            // If we have no targets do nothing
            if (pressed) {
                logf("!!!!! targets:%b homed:%b \n", result.hasTargets(), Robot.shooter.getTurretHomed());
            }
        }
    }

    void changePipeline(int id) {
        // Change pipeline to new pipe line
        camera.setPipelineIndex(id);
    }
}
