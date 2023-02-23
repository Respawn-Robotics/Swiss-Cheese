package frc.robot.autos;

import frc.robot.Constants;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CollectionSubsystem;
import frc.robot.subsystems.Swerve;

import java.util.ArrayList;
import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class pathGroup extends SequentialCommandGroup {
    public pathGroup(Swerve s_Swerve, ArmSubsystem arm, WristSubsystem wrist, CollectionSubsystem collection){
        PIDController theta = new PIDController(Constants.AutoConstants.kPThetaController, 0, Constants.AutoConstants.kDThetaController);
        theta.enableContinuousInput(-Math.PI, Math.PI);
ArrayList<PathPlannerTrajectory> pathGroup = (ArrayList<PathPlannerTrajectory>) PathPlanner.loadPathGroup("testPaths2", new PathConstraints(Constants.AutoConstants.kMaxSpeedMetersPerSecond, Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared));

// This is just an example event map. It would be better to have a constant, global event map
// in your code that will be used by all path following commands.
HashMap<String, Command> eventMap = new HashMap<>();
eventMap.put("pickup", arm.pickUpOnGround().andThen(wrist.setPosition(85000)).alongWith(collection.collectCone()));
eventMap.put("home", arm.goToHome().andThen(wrist.setPosition(0)).alongWith(collection.stopMotor()));


SwerveAutoBuilder auto = new SwerveAutoBuilder(s_Swerve::getPose, s_Swerve::resetOdometry, Constants.Swerve.swerveKinematics, new PIDConstants(Constants.AutoConstants.kPXController, 0, Constants.AutoConstants.kDXController), new PIDConstants(Constants.AutoConstants.kPThetaController, 0, Constants.AutoConstants.kDThetaController), s_Swerve::setModuleStates, eventMap,true,s_Swerve);


// Create the AutoBuilder. This only needs to be created once when robot code starts, not every time you want to create an auto command. A good place to put this is in RobotContainer along with your subsystems.

Command fullAuto = auto.fullAuto(pathGroup);

    }
}