package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;

public class AutoBalance extends CommandBase {
    
    private Supplier<Double> balanceAngleThresholdDegrees;    
    private double speed;
 
    private Swerve s_Swerve;

    public AutoBalance(Swerve s_Swerve, Supplier<Double> threshold, double speed) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.balanceAngleThresholdDegrees = threshold;
        this.speed = speed;
    }

    @Override
    public void execute() {
        double pitchAngleDegrees = s_Swerve.getPitch();
        double rollAngleDegrees = s_Swerve.getRoll();
        
        Translation2d balanceCorrection = new Translation2d(0, 0);

        if (Math.abs(pitchAngleDegrees) >= Math.abs(balanceAngleThresholdDegrees.get()) ||
            Math.abs(rollAngleDegrees)  >= Math.abs(balanceAngleThresholdDegrees.get())) {
            
            double pitchAngleRadians = Math.toRadians(pitchAngleDegrees);
            double rollAngleRadians = Math.toRadians(rollAngleDegrees);

            double xAxisRate = Math.sin(rollAngleRadians) * -speed;
            double yAxisRate = Math.sin(pitchAngleRadians) * speed;
            
            balanceCorrection = new Translation2d(xAxisRate, yAxisRate);
        }

        SmartDashboard.putNumber("Gyro Pitch", pitchAngleDegrees);
        SmartDashboard.putNumber("Gyro Roll", rollAngleDegrees);
        SmartDashboard.putNumber("Correction X", balanceCorrection.getX());
        SmartDashboard.putNumber("Correction Y", balanceCorrection.getY());

        s_Swerve.drive(balanceCorrection, 0, false, false);
    }

    @Override
    public void end(boolean interrupted) {
        s_Swerve.drive(new Translation2d(0, 0), 0, false, false);
    }
}