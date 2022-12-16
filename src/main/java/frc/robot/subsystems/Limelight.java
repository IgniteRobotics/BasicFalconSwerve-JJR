// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Arrays;

import com.igniterobotics.robotbase.calc.Measurement;
import com.igniterobotics.robotbase.calc.Units;
import com.igniterobotics.robotbase.calc.Measurement.Unit;
import com.igniterobotics.robotbase.reporting.ReportingBoolean;
import com.igniterobotics.robotbase.reporting.ReportingLevel;
import com.igniterobotics.robotbase.reporting.ReportingNumber;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {
  private NetworkTableInstance nTableInstance = NetworkTableInstance.getDefault();
  private NetworkTable netTable = nTableInstance.getTable("limelight");
  private NetworkTableEntry ty = netTable.getEntry("ty");
  private NetworkTableEntry tx = netTable.getEntry("tx");
  private NetworkTableEntry tv = netTable.getEntry("tv");
  private NetworkTableEntry camMode = netTable.getEntry("camMode");
  private NetworkTableEntry ledMode = netTable.getEntry("ledMode");

  private ReportingNumber tyReporter = new ReportingNumber("Limelight ty", ReportingLevel.TEST);
  private ReportingNumber txReporter = new ReportingNumber("Limelight tx", ReportingLevel.TEST);
  private ReportingNumber distanceReporter = new ReportingNumber("Limelight distance", ReportingLevel.COMPETITON);
  private ReportingBoolean tvReporter = new ReportingBoolean("Limelight tv", ReportingLevel.TEST);

  //camera height from floor
  private final double h1 = 0.7112;
  //target height from floor
  private final double h2 = 2.61;
  //camera setback angle in degrees
  private final Measurement a1 = new Measurement(Unit.DEGREES, 35);

  private final int windowSize = 5;
  private final double[] rollingWindow = new double[windowSize];
  private int index = 0;

  private double snapshotDistance;

  /** Creates a new Limelight. */
  public Limelight() {}

  @Override
  public void periodic() {
    tyReporter.set(getTy());
    txReporter.set(getTx());
    tvReporter.set(getTv());
    distanceReporter.set(getDistanceAverage());
    appendWindow(getDistance());
  }

  public double getTy() {
    return ty.getDouble(-1);
  }

  public double getTx() {
    return tx.getDouble(-1);
  }

  public boolean getTv() {
    return tv.getDouble(-1) > 0;
  }

  public void setCamMode(boolean isCamMode) {
    camMode.setNumber(isCamMode ? 1 : 0);
  }

  public void setLed(boolean on) {
    ledMode.setNumber(on ? 3 : 1);
  }

  public double getDistance() {
    if(!getTv()) return 0;

    return (h2 - h1) / Math.tan(Units.degToRad(getTy()) + a1.getAsRadians());
  }

  public double getXOffset(){
    if(!getTv()) return 0;
    
    return ( (getDistance()/Math.sin(Units.degToRad(90))) * Math.sin(Units.degToRad(getTx())));
  }

  public double getYOffset(){
    if(!getTv()) return 0;
    
    return ( (getDistance()/Math.sin(Units.degToRad(90))) * Math.sin(Units.degToRad(90 - getTx())));
  }

  public double getDistanceAverage() {
    return Arrays.stream(rollingWindow).sum() / windowSize;
  }

  private void appendWindow(double data) {
    rollingWindow[index] = data;
    index++;
    if(index >= windowSize) {
      index = 0;
    }
  }

  public void snapshotDistance() {
    this.snapshotDistance = getDistance();
  }

  public double getSnapshotDistance() {
    return snapshotDistance;
  }
}
