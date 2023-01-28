package frc.robot.commands;

import frc.robot.subsystems.Swerve;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class FixOrientation extends CommandBase {    
    private Swerve s_Swerve;    
    public double yaw;
    public double yawError;

    public FixOrientation(Swerve s_Swerve) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);
    }
    @Override
    public void initialize(){
        //yaw = s_Swerve.gyro.getYaw();
        //yawError = (yaw%360)%180;
    }

    
    @Override
    public void execute() {
        yaw = s_Swerve.gyro.getAbsoluteCompassHeading();
        System.out.println(yaw);
        //System.out.println(yawError);

        // Double heading_error = ;
        // double KpAim = .075;

        // if (tx > 1.0)
        // {
        //         steering_adjust = KpAim*heading_error - min_command;
        // }
        // else if (tx < -1.0)
        // {
        //         steering_adjust = KpAim*heading_error + min_command;
        // }
        
    }
}