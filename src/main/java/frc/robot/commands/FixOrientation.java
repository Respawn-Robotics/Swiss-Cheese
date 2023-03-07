package frc.robot.commands;

import frc.robot.subsystems.Swerve;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class FixOrientation extends CommandBase {    
    public Swerve s_Swerve;    
    public double yaw;
    public double angle;
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
        if(s_Swerve.gyro.getRoll() > 10 ){
            System.out.println("TooMuch");
            s_Swerve.drive(new Translation2d(-.65,0), 0, false, true);
        }else if(s_Swerve.gyro.getRoll() < -10){
            System.out.println("TooLittle");
            s_Swerve.drive(new Translation2d(.65,0), 0, false, true);
        }else{
            System.out.println("right");
            s_Swerve.setX();
        }
    }

    
    @Override
    public void execute() {
        
        // steering_adjust = KpAim * (desiredAngle - yaw);
        // s_Swerve.drive(new Translation2d(0,0), steering_adjust, false, true);
        levelRobot();
    }
}