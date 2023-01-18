package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;

public class AutoBalance extends CommandBase {
    
    private double balanceAngleThresholdDegrees  = 5;    

    private Swerve s_Swerve;

    public AutoBalance(Swerve s_Swerve) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);
    }

    @Override
    public void execute() {
        double pitchAngleDegrees = s_Swerve.getPitch();
        double rollAngleDegrees = s_Swerve.getRoll();
        
        Translation2d balanceCorrection = new Translation2d(0, 0);

        if (Math.abs(pitchAngleDegrees) <= Math.abs(balanceAngleThresholdDegrees) ||
            Math.abs(rollAngleDegrees)  <= Math.abs(balanceAngleThresholdDegrees)) {
            
            double pitchAngleRadians = Math.toRadians(pitchAngleDegrees);
            double rollAngleRadians = Math.toRadians(rollAngleDegrees);

            double xAxisRate = Math.sin(pitchAngleRadians) * -1;
            double yAxisRate = Math.sin(rollAngleRadians) * -1;
            
            balanceCorrection = new Translation2d(xAxisRate, yAxisRate);
        }

        SmartDashboard.putNumber("Gyro Pitch", pitchAngleDegrees);
        SmartDashboard.putNumber("Gyro Roll", rollAngleDegrees);
        SmartDashboard.putNumber("Correction X", balanceCorrection.getX());
        SmartDashboard.putNumber("Correction Y", balanceCorrection.getY());

        s_Swerve.drive(balanceCorrection, 0, true, false);
    }

    @Override
    public void end(boolean interrupted) {
        s_Swerve.drive(new Translation2d(0, 0), 0, true, false);
    }
}