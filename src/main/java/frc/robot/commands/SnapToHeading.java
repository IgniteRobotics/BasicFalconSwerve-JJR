// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SnapToHeading extends ProfiledPIDCommand {
  private Swerve m_Swerve;
  private double m_heading;
  /** Creates a new SnapToHeading. */
  public SnapToHeading(Swerve swerve, double heading) {
    super(
        // The ProfiledPIDController used by the command
        new ProfiledPIDController(
            // The PID gains
            1,
            0,
            0.1,
            // The motion profile constraints
            new TrapezoidProfile.Constraints(Constants.Swerve.maxAngularVelocity, Constants.Swerve.maxAngularVelocity)),
        // This should return the measurement
        swerve::getHeadingRadians,
        // This should return the goal (can also be a constant)
        heading,
        // This uses the output
        (output, setpoint) -> {
          System.out.println(output);
          swerve.drive(new Translation2d(0,0), output, false, false);
        }
        );
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_Swerve = swerve;
    this.m_heading = heading;
    addRequirements(m_Swerve);
    // Configure additional PID options by calling `getController` here.
    getController().setTolerance(0.1);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}
