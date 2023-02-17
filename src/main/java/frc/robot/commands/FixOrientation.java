package frc.robot.commands;

import frc.robot.subsystems.Swerve;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class FixOrientation extends CommandBase {    
    private Swerve s_Swerve;    
    public double yaw;
    double KpAim = .075;
    Double steering_adjust = 0.0;
    double min_command = 0.275;


    public FixOrientation(Swerve s_Swerve) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);
    }

    @Override
    public void initialize(){

    }

    
    @Override
    public void execute() {
        steering_adjust = KpAim* -yaw - min_command;
        s_Swerve.drive(new Translation2d(0,0), steering_adjust, false, true);
    }
}