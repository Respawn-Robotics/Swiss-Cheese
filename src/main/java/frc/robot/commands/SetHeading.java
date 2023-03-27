package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;

public class SetHeading extends CommandBase {
    private Swerve s_Swerve;
    private double currentYaw,adjust;
    private double kPHeading = 0.1;
    private double kIHeading = 0;
    private double kDHeading = 0;
    private int desiredHeading;

    public SetHeading(Swerve s_Swerve, int desiredHeading){
        this.s_Swerve = s_Swerve;
        this.desiredHeading = desiredHeading;
        addRequirements(s_Swerve);
    }

    @Override
    public void initialize(){
    }

    
    @Override
    public void execute() {
        PIDController headingPID = new PIDController(kPHeading, kIHeading, kDHeading);
        headingPID.enableContinuousInput(-180, 180);
        this.currentYaw = s_Swerve.gyro.getYaw();
        
        adjust = headingPID.calculate(s_Swerve.gyro.getYaw(), desiredHeading);
       
            s_Swerve.drive(new Translation2d(0,0), adjust*.5, false, false);
        
    }

    @Override
    public void end(boolean interuptted){

    }

    @Override
    public boolean isFinished(){
        return Math.abs(currentYaw) == desiredHeading; //End when angle is less than one
    }
}