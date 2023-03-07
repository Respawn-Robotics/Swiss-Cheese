package frc.robot.commands;

import frc.robot.subsystems.Swerve;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class FixOrientation extends CommandBase {    
    private Swerve s_Swerve;    
    public double yaw;
    double KpAim = .075;
    Double steering_adjust = 0.0;
    double desiredAngle = 180;


    public FixOrientation(Swerve s_Swerve) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);
    }

    @Override
    public void initialize(){

    }

    public void levelRobot(){
        s_Swerve.gyro.getPitch();
        
    }

    
    @Override
    public void execute() {
        steering_adjust = KpAim * (desiredAngle - yaw);
        s_Swerve.drive(new Translation2d(0,0), steering_adjust, false, true);
    }
}