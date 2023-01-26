package frc.robot.commands;
import frc.robot.subsystems.Swerve;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutonStop extends CommandBase {    
    private Swerve s_Swerve;    

    public AutonStop(Swerve s_Swerve) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);
    }

    @Override
    public void execute() {
        s_Swerve.drive(new Translation2d(0,0), 0, false, false);
    }
}