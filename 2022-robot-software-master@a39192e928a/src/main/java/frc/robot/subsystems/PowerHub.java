package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import static frc.robot.utilities.Util.logf;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

public class PowerHub extends SubsystemBase {
  /** Creates a new ReplaceMeCommand. */
  private static final int PDH_CAN_ID = 1;
  private static final int NUM_PDH_CHANNELS = 24;

  PowerDistribution powerHub = new PowerDistribution(PDH_CAN_ID, ModuleType.kRev);

  public PowerHub() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the susystem scheduled.
  @Override
  public void periodic() {
    // Clear sticky faults if requested
    if (Robot.oi.clearStickyAndLogCurrents()) {
      Robot.visionLight.toggleTargetingLight();
      // Clear sticky faults and Log Data from power hub
      powerHub.clearStickyFaults();
      String s = String.format("Volt:%.2f Cur:%.2f ", powerHub.getVoltage(), powerHub.getTotalCurrent());
      for (int i = 0; i < NUM_PDH_CHANNELS; i++) {
        double current = powerHub.getCurrent(i);
        if (current > 0) {
          s += String.format("%d:%.1f,", i, current);
        }
      }
      logf("%s\n", s);
    }
    if (!Robot.config.powerHubToDashBoard)
      return;
    long second = Robot.count % 100;
    if (second == 90 && Robot.config.ShowOnSmart)
      SmartDashboard.putNumber("Voltage", powerHub.getVoltage());
    if (second == 95 && Robot.config.ShowOnSmart)
      SmartDashboard.putNumber("Total Current", powerHub.getTotalCurrent());

    /**
     * Get the currents of each channel of the PDH and display them on
     * Shuffleboard.
     */
    int channel = (int) ((second + 1) / 4);
    if (channel < NUM_PDH_CHANNELS) {
      double current = powerHub.getCurrent(channel);
      if (current > 0.1 && Robot.config.ShowOnSmart) {
        SmartDashboard.putNumber(("Ch" + String.valueOf(channel) + " Current"), current);
      }
    }
  }
}
