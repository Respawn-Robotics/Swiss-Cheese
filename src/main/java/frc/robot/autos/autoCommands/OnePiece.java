package frc.robot.autos.autoCommands;

import frc.robot.Constants;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CollectionSubsystem;
import frc.robot.subsystems.Swerve;

import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class OnePiece extends SequentialCommandGroup {
    public OnePiece(Swerve s_Swerve, ArmSubsystem armSubsystem, WristSubsystem wristSubsystem, CollectionSubsystem collectionSubsystem, Vision limelightSubsystem){
        PathPlannerTrajectory exampleTrajectory = PathPlanner.loadPath("testPaths2", new PathConstraints(Constants.AutoConstants.kMaxSpeedMetersPerSecond, Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared));
        PIDController theta = new PIDController(Constants.AutoConstants.kPThetaController, 0, Constants.AutoConstants.kDThetaController);
        theta.enableContinuousInput(-Math.PI, Math.PI);

        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put("score", armSubsystem.setPosition(60000).andThen(wristSubsystem.setPosition(100000)).andThen(new WaitCommand(.5)).andThen(collectionSubsystem.ejectCone()));
        eventMap.put("home", collectionSubsystem.stopMotor().andThen(armSubsystem.setPosition(0)).andThen(wristSubsystem.setPosition(0)));

        PPSwerveControllerCommand path = new PPSwerveControllerCommand(exampleTrajectory,
        s_Swerve::getPose,
        Constants.Swerve.swerveKinematics,
        new PIDController(Constants.AutoConstants.kPXController, 0, Constants.AutoConstants.kDXController),
        new PIDController(Constants.AutoConstants.kPYController, 0, Constants.AutoConstants.kDYController),
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