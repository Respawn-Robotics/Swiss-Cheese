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
    public Boolean engaged = false;


    public FixOrientation(Swerve s_Swerve) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);
    }

    @Override
    public void initialize(){

    }

    public void levelRobot(){
        if((s_Swerve.gyro.getRoll() > 3.5) && (!engaged) ){
            s_Swerve.drive(new Translation2d(-.55,0), 0, false, true);
            System.out.println("Toomuch" + engaged);
        }else if((s_Swerve.gyro.getRoll() < -3.5) && (!engaged)){
            s_Swerve.drive(new Translation2d(.55,0), 0, false, true);
            System.out.println("TooLittle" +engaged);
        }else{
            s_Swerve.setX();
            engaged = true;
            System.out.println("right" + engaged);
        }
    }

    
    @Override
    public void execute() {
        
        // steering_adjust = KpAim * (desiredAngle - yaw);
        // s_Swerve.drive(new Translation2d(0,0), steering_adjust, false, true);
        levelRobot();
    }
}