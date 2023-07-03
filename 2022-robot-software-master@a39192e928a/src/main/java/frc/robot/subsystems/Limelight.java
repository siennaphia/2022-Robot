// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.utilities.Util;

import static frc.robot.utilities.Util.round2;

/**
 * The Limelight subsystem is a light that is lime green. If you look at it at a
 * certain angle, you will go blind, so read this code with caution.
 */
public class Limelight extends SubsystemBase {
  private final NetworkTable limeTable = NetworkTableInstance.getDefault().getTable("limelight");

  private final NetworkTableEntry txEntry = limeTable.getEntry("tx");
  private final NetworkTableEntry tyEntry = limeTable.getEntry("ty");
  private final NetworkTableEntry tvEntry = limeTable.getEntry("tv");
  private final NetworkTableEntry taEntry = limeTable.getEntry("ta");
  private final NetworkTableEntry tplEntry = limeTable.getEntry("pipeline");
  private int resetCount = 0;

  public double tx;
  public double ty;
  public boolean tv;
  public double ta;

  public double previousAngle = Double.MAX_VALUE;

  public double previousTx = tx;
  public double deltaTx = 0;
  

  public Limelight() {
    Util.logf("-------- Start Limelight %s\n", Robot.alliance);

  }

  @Override
  public void periodic() {
    tx = round2(txEntry.getDouble(0));
    ty = round2(tyEntry.getDouble(0));
    tv = tvEntry.getDouble(0) == 1;
    ta = round2(taEntry.getDouble(0));
    deltaTx = Math.abs(tx - previousTx);
    previousTx = tx;
    if (Robot.count % 15 == 0) {
      SmartDashboard.putNumber("TX", tx);
      SmartDashboard.putNumber("TY", ty);

      SmartDashboard.putNumber("TA", ta);
      SmartDashboard.putNumber("TV", tv ? 1 : 0);
      SmartDashboard.putBoolean("TVB", tv);
      SmartDashboard.putNumber("LLPl", getLimelightPipeline());
    }
    resetCount--;
    if(resetCount == 0) {
      resetSnapshot();
    }

  }

  private final double TARGET_HEIGHT = 20;
  private final double MOUNTING_HEIGHT = 8 * 12;
  private final double MOUNTING_ANGLE = 40;

  public double getEstimatedDistance() {
    // Formula: tan(a1 + a2) = (h2 - h1) / d
    return (TARGET_HEIGHT - MOUNTING_HEIGHT) / Math.tan(MOUNTING_ANGLE + ty);
  }

  public void setLimelightPipeline() {
    int pipeline;
    Robot.alliance = DriverStation.getAlliance();
    if (Robot.alliance == Alliance.Red) {
      pipeline = 1;
    } else if (Robot.alliance == Alliance.Blue) {
      pipeline = 2;
    } else {
      pipeline = 0;
    }
    int prevPipeline = getLimelightPipeline();
    Util.logf("-----------  Set Limelight pipeline robot alliance:%s new:%d prev:%d\n", Robot.alliance,  pipeline,
        prevPipeline);
    limeTable.getEntry("pipeline").setNumber(pipeline);

  }

  public int getLimelightPipeline() {
    return (int) tplEntry.getDouble(-1);
  }

  public void takeSnapshot() {
    limeTable.getEntry("snapshot").setNumber(1);
    resetCount = 10;
  }

  public void resetSnapshot() {
    limeTable.getEntry("snapshot").setNumber(0);
  }
}
