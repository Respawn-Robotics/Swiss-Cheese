package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class FixOrientation extends CommandBase {    
    public Swerve s_Swerve;    
    public double currentAngle;
    double KpLevel = .05;
    Double robotAdjust = 0.0;
    double desiredAngle = 0;
    public Boolean engaged = false;


    public FixOrientation(Swerve s_Swerve) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);
    }

    @Override
    public void initialize(){
        engaged = false;
    }

    public void levelRobot(){
        if((s_Swerve.gyro.getRoll() > 3.25) && (!engaged) ){
            s_Swerve.drive(new Translation2d(-.55,0), 0, false, true);
            System.out.println("Toomuch" + engaged);
        }else if((s_Swerve.gyro.getRoll() < -3.25) && (!engaged)){
            s_Swerve.drive(new Translation2d(.55,0), 0, false, true);
            System.out.println("TooLittle" +engaged);
        }else{
            s_Swerve.setX();
            engaged = true;
            System.out.println("right" + engaged);
        }
    }

    public void PIDLevel(){
        currentAngle = -s_Swerve.gyro.getRoll();
        robotAdjust = KpLevel*(currentAngle-desiredAngle);
        s_Swerve.drive(new Translation2d(robotAdjust,0), 0, false, false);
    }

    
    @Override
    public void execute() {
        
        PIDLevel();
        //levelRobot();
    }
}