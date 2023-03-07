package frc.robot.autos.autoCommands;

import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.JointsSetPosition;
import frc.robot.commands.operator.OperatorCommands;
import frc.robot.drivers.BeamBreak;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CollectionSubsystem;
import frc.robot.subsystems.Swerve;

import java.nio.file.Path;
import java.util.ArrayList;
import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.StopEvent;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class D3OneCone extends SequentialCommandGroup {
    public D3OneCone(Swerve s_Swerve,ArmSubsystem armSubsystem,WristSubsystem wristSubsystem, CollectionSubsystem collectionSubsystem, Vision limelightSubsystem){

// This will load the file "FullAuto.path" and generate it with a max velocity of 4 m/s and a max acceleration of 3 m/s^2
// for every path in the group
ArrayList<PathPlannerTrajectory> pathGroup = (ArrayList<PathPlannerTrajectory>) PathPlanner.loadPathGroup("D3OneCone",
new PathConstraints(1,1),
new PathConstraints(2, 2),
new PathConstraints(4, 3)
);

// This is just an example event map. It would be better to have a constant, global event map
// in your code that will be used by all path following commands.
HashMap<String, Command> eventMap = new HashMap<>();
eventMap.put("ResetSensors", wristSubsystem.setVoltage(-.1f).andThen(new WaitCommand(.8).andThen(wristSubsystem.setVoltage(0).andThen(wristSubsystem.resetPos()))));
eventMap.put("ArmGoUp", armSubsystem.setPosition(Constants.ArmConstants.SCORE_IN_HIGH_CONE).andThen(wristSubsystem.setPosition(Constants.WristConstants.SCORE_IN_HIGH_CONE)));
eventMap.put("RejectRun", collectionSubsystem.ejectCone());
eventMap.put("ArmGoHome", armSubsystem.setPosition(0).andThen(wristSubsystem.setPosition(0).andThen(collectionSubsystem.stopMotor())));
// Create the AutoBuilder. This only needs to be created once when robot code starts, not every time you want to create an auto command. A good place to put this is in RobotContainer along with your subsystems.
SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
    s_Swerve::getPose, // Pose2d supplier
    s_Swerve::resetOdometry, // Pose2d consumer, used to reset odometry at the beginning of auto
    Constants.Swerve.swerveKinematics, // SwerveDriveKinematics
    new PIDConstants(Constants.AutoConstants.kPXController, 0.0, Constants.AutoConstants.kDXController), // PID constants to correct for translation error (used to create the X and Y PID controllers)
    new PIDConstants(Constants.AutoConstants.kPThetaController, 0.0, Constants.AutoConstants.kDThetaController), // PID constants to correct for rotation error (used to create the rotation controller)
    s_Swerve::setModuleStates, // Module states consumer used to output to the drive subsystem
    eventMap,
    false, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
    s_Swerve // The drive subsystem. Used to properly set the requirements of path following commands
);

    autoBuilder.fullAuto(pathGroup).schedule();
    }
}