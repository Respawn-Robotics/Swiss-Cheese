package frc.robot.autos;

import frc.robot.Constants;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.Swerve;

import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

public class exampleAuto extends SequentialCommandGroup {
    public exampleAuto(Swerve s_Swerve, LimelightSubsystem limelightSubsystem){
        PathPlannerTrajectory exampleTrajectory = PathPlanner.loadPath("LimelightTest", new PathConstraints(4, 3.0));
        PIDController theta = new PIDController(Constants.AutoConstants.kPThetaController, 0, 0);
        theta.enableContinuousInput(-Math.PI, Math.PI);

        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put("limelighton", limelightSubsystem.setLedMode(1));
        eventMap.put("limelightoff", limelightSubsystem.setLedMode(0));

        PPSwerveControllerCommand path = new PPSwerveControllerCommand(exampleTrajectory,
        s_Swerve::getPose, Constants.Swerve.swerveKinematics,
        new PIDController(Constants.AutoConstants.kPXController, 0, 0),
        new PIDController(Constants.AutoConstants.kPYController, 0, 0),
        theta,
        s_Swerve::setModuleStates, 
        false,
        s_Swerve);

        addCommands(
            new InstantCommand(() -> s_Swerve.resetOdometry(exampleTrajectory.getInitialHolonomicPose())),
            new FollowPathWithEvents(path, exampleTrajectory.getMarkers(), eventMap)
        );
    }

    
}