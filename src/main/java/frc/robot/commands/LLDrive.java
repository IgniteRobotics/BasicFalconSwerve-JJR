// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LLDrive extends CommandBase {
  private double rotation;
  private Translation2d translation;
  private boolean fieldRelative;
  private boolean openLoop;
  
  private Swerve s_Swerve;
  private Limelight s_Limelight;

  //ratio of y offset to x offset for scaling the curvature.
  private double ratio = 0;

  //do we have a target?
  private boolean targetFound = false;

  /** Creates a new LLDrive. */
  public LLDrive(Swerve s_Swerve, Limelight limelight, boolean fieldRelative, boolean openLoop) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.s_Swerve = s_Swerve;
    addRequirements(s_Swerve);

    this.s_Limelight = limelight;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.ratio = 0;
    this.targetFound = false;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!targetFound) {
      if (s_Limelight.getTv()){
        targetFound = true;
        ratio = s_Limelight.getTy() / s_Limelight.getTx();
      } else {
        //skip the rest if you haven't found a target.
        //this starts driving in the same iteration we find a target.
        return;
      }
    }
    double yAxis = s_Limelight.getTy();
    double xAxis = s_Limelight.getTx();

    SmartDashboard.putNumber("raw xOffset", xAxis);
    SmartDashboard.putNumber("raw yOffset", yAxis);

    translation = new Translation2d(yAxis - (ratio * xAxis), xAxis).times(Constants.Swerve.maxSpeed);
    s_Swerve.drive(translation, 0, fieldRelative, openLoop);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    translation = new Translation2d(0,0);
    s_Swerve.drive(translation, 0, false, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
