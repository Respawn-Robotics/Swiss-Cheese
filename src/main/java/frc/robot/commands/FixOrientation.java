package frc.robot.commands;

import frc.robot.subsystems.Swerve;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class FixOrientation extends CommandBase {    
    private Swerve s_Swerve;  

    private double currentAngle;
    private double error;
    private double drivePower;
    private double kPLeveler = .04;

    public FixOrientation(Swerve s_Swerve) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);
    }

    @Override
    public void initialize(){
    }

    
    @Override
    public void execute() {
         //PIDLevel();  
        this.currentAngle = s_Swerve.gyro.getRoll();

        error = 0 - currentAngle;
        drivePower = -Math.min(kPLeveler * error, 1);

        s_Swerve.drive(new Translation2d(drivePower,0), 0, false, false);
        System.out.println(drivePower);
        System.out.println(currentAngle);
    }

    @Override
    public void end(boolean interuptted){
        s_Swerve.setX(); //End command, sets X mode on wheels
    }

    @Override
    public boolean isFinished(){
        return Math.abs(error) < .5; //End when angle is less than one
    }
}