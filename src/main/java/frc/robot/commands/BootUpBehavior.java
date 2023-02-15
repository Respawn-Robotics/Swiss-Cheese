package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;
import frc.robot.Constants;
public class BootUpBehavior extends CommandBase {

    public BootUpBehavior(Swerve swerve) {
    addRequirements(swerve);
    }

    public void AutoNeutralMode(){
    //Constants.Swerve.driveNeutralMode = NeutralMode.Brake;
    //Constants.Swerve.angleNeutralMode = NeutralMode.Coast;
    }

    public void TeleopNeutralMode(){
    //Constants.Swerve.driveNeutralMode = NeutralMode.Brake;
    //Constants.Swerve.angleNeutralMode = NeutralMode.Coast;
    }

    public void disabledNeutralMode(){
    //Constants.Swerve.driveNeutralMode = NeutralMode.Coast;
    //Constants.Swerve.angleNeutralMode = NeutralMode.Coast;
    }
}
