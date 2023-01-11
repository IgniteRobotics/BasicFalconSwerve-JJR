package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class SpinCommand extends CommandBase {
    
    private Swerve s_Swerve;
    private Joystick controller;
    private int translationAxis;
    private int strafeAxis;
    
    private Supplier<Double> centerOffsetX;
    private Supplier<Double> centerOffsetY;
    private double rotationSpeed;
    
    public SpinCommand(Swerve s_Swerve, Joystick controller, int translationAxis, int strafeAxis, Supplier<Double> centerOffsetX, Supplier<Double> centerOffsetY, double rotationSpeed) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.controller = controller;
        this.translationAxis = translationAxis;
        this.strafeAxis = strafeAxis;
        this.centerOffsetX = centerOffsetX;
        this.centerOffsetY = centerOffsetY;
        this.rotationSpeed = rotationSpeed;
    }

    @Override
    public void initialize() {
        // Decide where to move center based on spin direction
        double offsetY = rotationSpeed < 0 ? -centerOffsetY.get() : centerOffsetY.get();

        s_Swerve.setRotationCenterOffset(new Translation2d(centerOffsetX.get(), offsetY));
    }

    @Override
    public void execute() {
        double yAxis = -controller.getRawAxis(translationAxis);
        double xAxis = -controller.getRawAxis(strafeAxis);
        
        yAxis = (Math.abs(yAxis) < Constants.stickDeadband) ? 0 : yAxis;
        xAxis = (Math.abs(xAxis) < Constants.stickDeadband) ? 0 : xAxis;
        
        Translation2d translation = new Translation2d(yAxis, xAxis).times(Constants.Swerve.maxSpeed);
        s_Swerve.drive(translation, rotationSpeed, true, false);
    }

    @Override
    public void end(boolean interrupted) {
        s_Swerve.resetRotationCenterOffset();
    }
}