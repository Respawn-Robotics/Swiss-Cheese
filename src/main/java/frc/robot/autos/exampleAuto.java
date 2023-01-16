package frc.robot.autos;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

public class exampleAuto extends SequentialCommandGroup {
    public exampleAuto(Swerve s_Swerve){

        final ProfiledPIDController thetaController =
        new ProfiledPIDController(Constants.AutoConstants.kPThetaController, 0, 0,
            Constants.AutoConstants.kThetaControllerConstraints);
        
        PathPlannerTrajectory exampleTrajectory = PathPlanner.loadPath("LimelightTest", new PathConstraints(4, 3.0));

        addCommands(
            new InstantCommand(() -> s_Swerve.resetOdometry(exampleTrajectory.getInitialHolonomicPose())),

            new PPSwerveControllerCommand(exampleTrajectory,
                s_Swerve::getPose, Constants.Swerve.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                new PIDController(0, 0, 0),
                s_Swerve::setModuleStates, s_Swerve)
        );
    }

    
}