package frc.robot;

import java.io.IOException;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    /* Controllers */
    private final Joystick driver = new Joystick(0);
    private final Joystick operator = new Joystick(1);
    private final CollectionSubsystem collection = new CollectionSubsystem();

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    private final JoystickButton followTarget = new JoystickButton(driver, XboxController.Button.kA.value);
    private final JoystickButton togglePipeline = new JoystickButton(driver, XboxController.Button.kX.value);
    private final JoystickButton centerRobot = new JoystickButton(driver, XboxController.Button.kY.value);

    private final JoystickButton motorOn = new JoystickButton(operator, XboxController.Button.kA.value);
    private final JoystickButton motorOff = new JoystickButton(operator, XboxController.Button.kB.value);
    private final JoystickButton reject = new JoystickButton(operator, XboxController.Button.kX.value);
    private final JoystickButton turnOnCompressor = new JoystickButton(driver, XboxController.Button.kRightBumper.value);
    private final JoystickButton turnOffCompressor = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    private final JoystickButton solenoidForward = new JoystickButton(driver, XboxController.Button.kY.value);

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    LimelightSubsystem LimelightSubsystem = new LimelightSubsystem();

    /* Trajectories */
    Trajectory Swervy, Test, straight, tpoint, GameAuto;
    

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -driver.getRawAxis(translationAxis) * .75, 
                () -> -driver.getRawAxis(strafeAxis) * .75, 
                () -> -driver.getRawAxis(rotationAxis) * .75, 
                () -> robotCentric.getAsBoolean()
            )
        );   

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

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
        togglePipeline.onTrue(LimelightSubsystem.togglePipeline());
        followTarget.whileTrue(new FollowTape(s_Swerve));
        //centerRobot.whileTrue(new FixOrientation(s_Swerve));
        

        motorOn.onTrue(collection.runMotor());
        motorOff.onTrue(collection.disableMotor());
        reject.onTrue(collection.reject());
        
    }

    /**
     * Use this to pass the autonomous command to the main k Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return new exampleAuto(s_Swerve, LimelightSubsystem);
        //return new PPauto(s_Swerve, GameAuto);
    }
}