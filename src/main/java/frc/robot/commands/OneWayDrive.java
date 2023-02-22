package frc.robot.commands;

import frc.robot.subsystems.Swerve;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class OneWayDrive extends CommandBase {    
    private Swerve s_Swerve;    

    public OneWayDrive(Swerve s_Swerve) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);
    }

    public void LeftDrive() {
        
            s_Swerve.drive(new Translation2d(-1,0), 0, true, false);
        
    }

    public void RightDrive() {
        
            s_Swerve.drive(new Translation2d(1,0), 0, true, false);
        
    }

    public void FrontDrive() {
        
            s_Swerve.drive(new Translation2d(0,1), 0, true, false);
        
    }

    public void BackDrive() {
        
            s_Swerve.drive(new Translation2d(0,-1), 0, true, false);
        
    }

    @Override
    public void execute() {

    }
}