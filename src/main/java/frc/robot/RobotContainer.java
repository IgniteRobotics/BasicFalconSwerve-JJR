// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  /* Controllers */
  private final Joystick driver = new Joystick(0);

  /* Drive Controls */
  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;
  private final boolean fieldRelative = true;
  private final boolean openLoop = false;

  /* Driver Buttons */
  private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
  private final JoystickButton spinLeft = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
  private final JoystickButton spinRight = new JoystickButton(driver, XboxController.Button.kRightBumper.value);
  private final JoystickButton driverAButton = new JoystickButton(driver, XboxController.Button.kA.value);

  private POVButton driverDpadUp = new POVButton(driver, 0);
  private POVButton driverDpadDown = new POVButton(driver, 180);
  private POVButton driverDpadLeft = new POVButton(driver, 270);
  private POVButton driverDpadRight = new POVButton(driver, 90);

  /* Subsystems */
  private final Swerve s_Swerve = new Swerve();
  private final Limelight s_Limelight = new Limelight();


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    
    s_Swerve.setDefaultCommand(new TeleopSwerve(s_Swerve, driver, translationAxis, strafeAxis, rotationAxis, fieldRelative, openLoop));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    /* Driver Buttons */
    zeroGyro.whenPressed(new InstantCommand(() -> s_Swerve.zeroIMU()));

    Translation2d spinLeftOffset = new Translation2d(-Constants.Swerve.wheelBase / 2.0, Constants.Swerve.trackWidth / 2.0);
    Translation2d spinRightOffset = new Translation2d(-Constants.Swerve.wheelBase / 2.0, -Constants.Swerve.trackWidth / 2.0);
    
    spinLeft.whileHeld(new SpinCommand(s_Swerve, spinLeftOffset, -Constants.Swerve.maxAngularVelocity));
    spinRight.whileHeld(new SpinCommand(s_Swerve, spinRightOffset, Constants.Swerve.maxAngularVelocity));

    driverAButton.whileHeld(new ZoomToTagCommand(s_Swerve, s_Limelight, fieldRelative, openLoop));
    driverDpadUp.whenPressed(new SnapToHeading(s_Swerve, 0));
    driverDpadDown.whenPressed(new SnapToHeading(s_Swerve, 180));
    driverDpadLeft.whenPressed(new SnapToHeading(s_Swerve, 90));
    driverDpadRight.whenPressed(new SnapToHeading(s_Swerve, 270));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new exampleAuto(s_Swerve);
  }
}
