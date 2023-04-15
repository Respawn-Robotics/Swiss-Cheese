package frc.robot.commands;

import frc.robot.subsystems.Swerve;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Vision;

public class FollowTape extends CommandBase {    
    private Swerve s_Swerve;    
    double KpAim = .075; //controls overshoot of aim
    double min_command = 0.275; //controls minimum voltage of aim
    double KpDistance = 0.1;
    Vision LimelightSubsystem = new Vision();
    double tx;
    double ty;
    double ta;

    public FollowTape(Swerve s_Swerve) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);
    }

    @Override
    public void execute() {
        tx = LimelightSubsystem.getTX();
        ty = LimelightSubsystem.getTY();
        ta = LimelightSubsystem.getTA();

        Double steering_adjust = 0.0;
        Double driving_adjust = 0.0;
            
        if(LimelightSubsystem.isTargetValid()){
            if(ta < 1.56 && ty < 0){
                System.out.println(ty);
                driving_adjust = 1.0;
                s_Swerve.drive(new Translation2d(driving_adjust,0), steering_adjust, false, true);
            }
        }
    }

@Override
    public boolean isFinished(){
        return (ta > 1.56 || ty >= 0); //End when angle is less than one
    }
}