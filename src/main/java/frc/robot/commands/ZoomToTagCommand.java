// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ZoomToTagCommand extends CommandBase {
  private Translation2d translation;
  private boolean fieldRelative = true;
  private boolean openLoop = false;
  
  private Swerve s_Swerve;
  private Limelight s_Limelight;

  //ratio of y offset to x offset for scaling the curvature.
  private double ratio = 0;

  //do we have a target?
  private boolean targetFound = false;

  private ProfiledPIDController yController = 
    new ProfiledPIDController(
      2.7, 0.0, 0, 
      new TrapezoidProfile.Constraints(this.maxSpeed, this.maxAccel)
    );

  private ProfiledPIDController xController = 
    new ProfiledPIDController(
      1.35, 0.0, 0, 
      new TrapezoidProfile.Constraints(this.maxSpeed,this.maxAccel)
    );


  //defaults for distance and offset tolerances (in meters)
  private double yTolerance = 1;
  private double xTolerance = 0.25;

  private double maxSpeed = Constants.Swerve.maxSpeed;
  private double maxAccel = Constants.Swerve.maxSpeed/2;

  /** Creates a new LLDrive. */
  public ZoomToTagCommand(Swerve s_Swerve, Limelight limelight, boolean fieldRelative, boolean openLoop) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.s_Swerve = s_Swerve;
    addRequirements(s_Swerve);

    this.s_Limelight = limelight;
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.yTolerance = SmartDashboard.getNumber("Zoom Y Tolerance", 1);
    this.xTolerance = SmartDashboard.getNumber("Zoom X Tolerance", 0.25);

    this.yController.setConstraints(new TrapezoidProfile.Constraints(
      SmartDashboard.getNumber("Zoom Speed", maxSpeed), 
      SmartDashboard.getNumber("Zoom Accel", maxAccel)
      ));

    this.xController.setConstraints(new TrapezoidProfile.Constraints(
      SmartDashboard.getNumber("Zoom Speed", maxSpeed), 
      SmartDashboard.getNumber("Zoom Accel", maxAccel)
      ));

    this.ratio = 0;
    this.targetFound = false;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!targetFound) {
      if (s_Limelight.getTv()){
        targetFound = true;
        //scale the xoffset up if it's smaller than the Y.
        //otherwise it's 1.
        if (s_Limelight.getYOffset() > s_Limelight.getXOffset()) {
          ratio = s_Limelight.getYOffset() / s_Limelight.getXOffset();
        } else {
          ratio = 1;
        }
      } else {
        //skip the rest if you haven't found a target.
        //this starts driving in the same iteration we find a target.
        return;
      }
    }

    
    double yAxis = s_Limelight.getYOffset();
    double xAxis = s_Limelight.getXOffset();

    SmartDashboard.putNumber("raw xOffset", xAxis);
    SmartDashboard.putNumber("raw yOffset", yAxis);

    xAxis = xController.calculate(xAxis, this.xTolerance);
    //TRYING changing the PID controller instead.
    //ratio and fix sign on yAxis movement
    // yAxis = Math.copySign(
    //   Math.max((Math.abs(yAxis) - Math.abs(xAxis)*ratio), 0), 
    //   yAxis);
    yAxis = yController.calculate(yAxis, this.yTolerance);
    

    translation = new Translation2d(yAxis, xAxis);
    s_Swerve.drive(translation, 0, fieldRelative, openLoop);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    translation = new Translation2d(0,0);
    s_Swerve.drive(translation, 0, fieldRelative, openLoop);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return yController.atGoal() && xController.atGoal();
  }
}
