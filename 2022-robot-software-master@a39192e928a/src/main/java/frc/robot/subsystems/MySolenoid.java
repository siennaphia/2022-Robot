/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.utilities.Util;

public class MySolenoid extends SubsystemBase {
    // This class creates solenoid objects for things like LEDs
    // that are controlled by a Pneumatic Control Module (PCM)
    // It will warn you if the PCM does not exist

    int id;
    int channelA;
    int channelB;
    String name;

    private PneumaticHub pHub;
    private Solenoid solenoidA = null;
    private Solenoid solenoidB = null;

    // Create a single channel solenoid -- used for controlling things like leds
    public MySolenoid(String name, int id, int channelA, int channelB) {
        if (Robot.config.pcmHubID <= 0) {
            return;
        }
        if (id != Robot.config.pcmHubID) {
            solenoidA = null;
            Util.logf("No solenoid configuration for id:%d %s channel:%d\n", id, name, channelA);
            return;
        }
        try {
            //pHub = new PneumaticHub();
            pHub = Robot.pHub; 
            solenoidA = pHub.makeSolenoid(channelA);
            this.id = id;
            this.channelA = channelA;
            this.channelB = channelB;
            this.name = name;
            if (channelB > 0) {
                Util.loginfo("Create a double solenoid for %s id:%d channels:<%d,%d>\n", name, id, channelA, channelB);
                solenoidB = pHub.makeSolenoid(channelA);
            } else {
                Util.loginfo("Create a single solenoid for %s id:%d channel:%d\n", name, id, channelA);
            }
        } catch (Exception e) {
            solenoidA = null;
            solenoidB = null;
            Util.logf("*** Error unable to create a solenoid for %s\n", name);
        }
    }

    @Override
    public void periodic() {
    }

    public boolean getA() {
        if (solenoidA != null)
            return solenoidA.get();
        else
            return false;
    }

    public void setA(boolean value) {
        if (solenoidA != null)
            solenoidA.set(value);
    }

    public boolean getB() {
        if (solenoidB != null)
            return solenoidB.get();
        else
            return false;
    }

    public void setB(boolean value) {
        if (solenoidB != null)
            solenoidB.set(value);
    }
}
