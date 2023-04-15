package frc.robot.commands;

import frc.robot.subsystems.Swerve;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class FixOrientation extends CommandBase {    
    private Swerve s_Swerve;  
    private PIDController controller, controller2;

    // private double currentAngle;
    // private double error;
    // private double drivePower;
    // private double kPLeveler = .04;

    public FixOrientation(Swerve s_Swerve) {
        this.s_Swerve = s_Swerve;
        controller = new PIDController(1, 0, 0);
        controller.setTolerance(9.5);
        controller.setSetpoint(0);
        controller2 = new PIDController(1, 0, 0);
        controller2.setTolerance(1);
        controller2.setSetpoint(0);
        addRequirements(s_Swerve);
    }

    @Override
    public void initialize(){
    }

    
    @Override
    public void execute() {
         //PIDLevel();  
        SmartDashboard.putBoolean("At Tolerance", controller.atSetpoint());

        double translationVal = MathUtil.clamp(controller.calculate(s_Swerve.gyro.getRoll(), 0.0), -1,
        1);

        s_Swerve.drive(new Translation2d(-translationVal, 0.0), 0.0, false, false);

        System.out.println(s_Swerve.gyro.getRoll());
        // if(Math.abs(currentAngle) > 17){
        //     s_Swerve.drive(new Translation2d(drivePower,0), 0, false, false);
        // }else if(Math.abs(currentAngle) < 11) {
        //     s_Swerve.drive(new Translation2d(drivePower * .5,0), 0, false, false);
        // }
    }

    @Override
    public void end(boolean interuptted){
       
            s_Swerve.setX();
             //End command, sets X mode on wheels
    }

    @Override
    public boolean isFinished(){
        return controller.atSetpoint(); //End when angle is less than one
    }
}