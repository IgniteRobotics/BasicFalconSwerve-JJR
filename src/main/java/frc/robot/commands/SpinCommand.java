package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;

public class SpinCommand extends CommandBase {
    
    private Swerve s_Swerve;
    
    private Translation2d centerOffset;
    private double rotationSpeed;
    
    public SpinCommand(Swerve s_Swerve, Translation2d centerOffset, double rotationSpeed) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.centerOffset = centerOffset;
        this.rotationSpeed = rotationSpeed;
    }

    @Override
    public void initialize() {
        s_Swerve.setRotationCenterOffset(centerOffset);
    }

    @Override
    public void execute() {
        Translation2d translation = new Translation2d(0, 0);
        s_Swerve.drive(translation, rotationSpeed, true, false);
    }

    @Override
    public void end(boolean interrupted) {
        s_Swerve.resetRotationCenterOffset();
    }
}