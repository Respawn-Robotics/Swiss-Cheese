package frc.robot;

import java.io.IOException;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.JointMovementType;
import frc.robot.Constants.WristConstants;
import frc.robot.Constants.JointMovementType;
import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.commands.manual.ManualArmDown;
import frc.robot.commands.manual.ManualArmUp;
import frc.robot.commands.manual.ManualWristDown;
import frc.robot.commands.manual.ManualWristUp;
import frc.robot.disabled.Disable;
import frc.robot.drivers.BeamBreak;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    /* Controllers */
    private final Joystick driver = new Joystick(0);
    private final Joystick operator = new Joystick(1);

    /* Subsystem */
    private final CollectionSubsystem collectionSubsystem = new CollectionSubsystem();
    private final ArmSubsystem armSubsystem = new ArmSubsystem();
    private final WristSubsystem wristSubsystem = new WristSubsystem();
    private final Swerve s_Swerve = new Swerve();
    private final Vision vision = new Vision();

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons */
    private final JoystickButton d_Y = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton d_A = new JoystickButton(driver, XboxController.Button.kA.value);
    private final JoystickButton d_X = new JoystickButton(driver, XboxController.Button.kX.value);
    private final JoystickButton d_B = new JoystickButton(driver, XboxController.Button.kB.value);
    private final JoystickButton d_leftBumper = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    private final JoystickButton d_rightBumper = new JoystickButton(driver, XboxController.Button.kRightBumper.value);
    private final POVButton d_povRight = new POVButton(driver, 90);
    private final POVButton d_povDown = new POVButton(driver, 180);
    private final POVButton d_povLeft = new POVButton(driver, 270);

    /* Operator Buttons */
    private final JoystickButton o_rightBumper = new JoystickButton(operator, XboxController.Button.kRightBumper.value);
    private final JoystickButton o_rightStick = new JoystickButton(operator, XboxController.Button.kRightStick.value);
    private final JoystickButton o_leftBumper = new JoystickButton(operator, XboxController.Button.kLeftBumper.value);
    private final JoystickButton o_leftStick = new JoystickButton(operator, XboxController.Button.kLeftStick.value);
    private final JoystickButton o_start = new JoystickButton(operator, XboxController.Button.kStart.value);
    private final JoystickButton o_back = new JoystickButton(operator, XboxController.Button.kBack.value);
    private final JoystickButton o_A = new JoystickButton(operator, XboxController.Button.kA.value);
    private final JoystickButton o_B = new JoystickButton(operator, XboxController.Button.kB.value);
    private final JoystickButton o_X = new JoystickButton(operator, XboxController.Button.kX.value);
    private final JoystickButton o_Y = new JoystickButton(operator, XboxController.Button.kY.value);
    private final POVButton o_povUp = new POVButton(operator, 0);
    private final POVButton o_povRight = new POVButton(operator, 90);
    private final POVButton o_povDown = new POVButton(operator, 180);
    private final POVButton o_povLeft = new POVButton(operator, 270);

    /* Sensors */
    private final BeamBreak coneBeamBreak = new BeamBreak(1);
    private final BeamBreak cubeBeamBreak = new BeamBreak(2);

    /* Trajectories */
    Trajectory Swervy, Test, straight, tpoint, GameAuto;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        s_Swerve.setDefaultCommand(
                new TeleopSwerve(
                        s_Swerve,
                        () -> -driver.getRawAxis(translationAxis) * .75,
                        () -> -driver.getRawAxis(strafeAxis) * .75,
                        () -> -driver.getRawAxis(rotationAxis) * .75,
                        () -> d_leftBumper.getAsBoolean()));

        try {
            Test = TrajectoryUtil.fromPathweaverJson(
                    Filesystem.getDeployDirectory().toPath().resolve(
                            "pathplanner/generatedJSON/TEST.wpilib.json"));
            Swervy = TrajectoryUtil.fromPathweaverJson(
                    Filesystem.getDeployDirectory().toPath().resolve(
                            "pathplanner/generatedJSON/curvy swervy.wpilib.json"));
            straight = TrajectoryUtil.fromPathweaverJson(
                    Filesystem.getDeployDirectory().toPath().resolve(
                            "pathplanner/generatedJSON/straight.wpilib.json"));
            tpoint = TrajectoryUtil.fromPathweaverJson(
                    Filesystem.getDeployDirectory().toPath().resolve(
                            "pathplanner/generatedJSON/testPaths1.wpilib.json"));
            GameAuto = TrajectoryUtil.fromPathweaverJson(Filesystem.getDeployDirectory().toPath().resolve(
                    "pathplanner/generatedJSON/GameAuto.wpilib.json"));

        } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory", ex.getStackTrace());
        }

        // Configure the button bindings
        configureButtonBindings();
    }

    private void configureButtonBindings() {

        /* Driver Buttons */

        // Set slow drive mode
        d_rightBumper.onTrue(new InstantCommand(() -> s_Swerve.setSlow(true)));
        d_rightBumper.onFalse(new InstantCommand(() -> s_Swerve.setSlow(false)));

        // Gyro Offsets
        d_Y.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
        d_povRight.onTrue(new InstantCommand(() -> s_Swerve.rightGyro()));
        d_povDown.onTrue(new InstantCommand(() -> s_Swerve.downGyro()));
        d_povLeft.onTrue(new InstantCommand(() -> s_Swerve.leftGyro()));

        // Toggle Limelight pipeline
        d_X.onTrue(vision.togglePipeline());

        // Follow retroreflective tape
        d_A.whileTrue(new FollowTape(s_Swerve));

        // Home arm
        d_B.onTrue(new JointsSetPosition(ArmConstants.HOME, 
                                         WristConstants.HOME, 
                                         JointMovementType.WRIST_FIRST, 
                                         0.4, 
                                         armSubsystem, wristSubsystem));

        /* Operator Controls */

        // Acquire off ground        
        o_A.onTrue(new JointsSetPosition(ArmConstants.ACQUIRE_FROM_FLOOR, 
                                         WristConstants.ACQUIRE_FROM_FLOOR, 
                                         JointMovementType.WRIST_FIRST, 
                                         0.4, 
                                         armSubsystem, wristSubsystem));
        // SiS Position
        o_B.onTrue(new JointsSetPosition(ArmConstants.ACQUIRE_FROM_SIS, 
                                         WristConstants.ACUQIRE_FROM_SIS, 
                                         JointMovementType.WRIST_FIRST, 
                                         0.4, 
                                         armSubsystem, wristSubsystem));

        // Acqure from DoS
        o_Y.onTrue(new JointsSetPosition(ArmConstants.ACQUIRE_FROM_DOS, 
                                         WristConstants.ACQUIRE_FROM_DOS, 
                                         JointMovementType.WRIST_FIRST, 
                                         0.4, 
                                         armSubsystem, wristSubsystem));
        // Reset Sensors
        o_X.onTrue(armSubsystem.resetSensor().andThen(wristSubsystem.resetSensor()));

        // Score in high cone
        o_povLeft.onTrue(new JointsSetPosition(ArmConstants.SCORE_IN_HIGH_CONE, 
                                         WristConstants.SCORE_IN_HIGH_CONE, 
                                         JointMovementType.WRIST_FIRST, 
                                         0.4, 
                                         armSubsystem, wristSubsystem));

        // Score in high cube
        o_povRight.onTrue(new JointsSetPosition(ArmConstants.SCORE_IN_HIGH_CUBE, 
                                         WristConstants.SCORE_IN_HIGH_CUBE, 
                                         JointMovementType.WRIST_FIRST, 
                                         0.4, 
                                         armSubsystem, wristSubsystem));

        // Score in mid cone + cube
        o_povDown.onTrue(new JointsSetPosition(ArmConstants.SCORE_IN_MID, 
                                         WristConstants.SCORE_IN_MID,
                                         JointMovementType.WRIST_FIRST, 
                                         0.4, 
                                         armSubsystem, wristSubsystem));

        // Manual Arm and Wrist
        o_leftStick.whileTrue(new ManualWristUp(wristSubsystem));
        //o_rightStick.whileTrue(new ManualWristDown(wristSubsystem));
        o_start.whileTrue(new ManualArmUp(armSubsystem));
        o_back.whileTrue(new ManualArmDown(armSubsystem));

        // Collection Controls
        o_rightBumper.onTrue(collectionSubsystem.collectCube());
        o_leftBumper.onTrue(collectionSubsystem.collectCone());
        o_povDown.onTrue(collectionSubsystem.stopMotor());
    }

    /**
     * Use this to pass the autonomous command to the main k Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return new exampleAuto(s_Swerve, vision);
        // return new PPauto(s_Swerve, GameAuto);
    }

    /**
     * Use this to pass the disabled command to the main k Robot} class.
     * 
     * @return the command to run in autonomous
     */
    public Command getDisableCommand() {
        return new Disable(s_Swerve);
    }
}