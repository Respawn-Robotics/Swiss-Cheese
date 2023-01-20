package frc.robot.commands;

import frc.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.LimelightSubsystem;

public class FollowTape extends CommandBase {    
    private Swerve s_Swerve;    
    double KpAim = .025; //controls overshoot of aim
    double min_command = 0.35; //controls minimum voltage of aim
    double KpDistance = -0.1235;
    LimelightSubsystem LimelightSubsystem = new LimelightSubsystem();
    double tx;
    double ty;

    public FollowTape(Swerve s_Swerve) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);
    }

    @Override
    public void execute() {
        tx = LimelightSubsystem.getTX();
        ty = LimelightSubsystem.getTY();

        Double heading_error = -tx;
        Double steering_adjust = 0.0;
        Double driving_adjust = 0.0;
            
            if(LimelightSubsystem.getV() == 0){
                steering_adjust = 1.5;
                s_Swerve.drive(new Translation2d(0,0), steering_adjust, false, true);
            }else if(LimelightSubsystem.getV() == 1){
                if (tx > 1.0)
                {
                        steering_adjust = KpAim*heading_error - min_command;
                }
                else if (tx < -1.0)
                {
                        steering_adjust = KpAim*heading_error + min_command;
                }
                driving_adjust = KpDistance * ty;
                s_Swerve.drive(new Translation2d(driving_adjust,0), steering_adjust, false, true);
        }
    }
}