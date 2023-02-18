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
import frc.robot.disabled.Disable;
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

    /* Subsystem */
    private final CollectionSubsystem collectionSubsystem = new CollectionSubsystem();
    private final ArmSubsystem armSubsystem               = new ArmSubsystem(operator);
    private final WristSubsystem wristSubsystem           = new WristSubsystem(operator);
    private final Swerve s_Swerve = new Swerve();
    private final Vision vision = new Vision();

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons */
    private final JoystickButton zeroGyro             = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton robotCentric         = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    private final JoystickButton followTarget         = new JoystickButton(driver, XboxController.Button.kA.value);
    private final JoystickButton togglePipeline       = new JoystickButton(driver, XboxController.Button.kX.value);
    private final JoystickButton lockRobot            = new JoystickButton(driver, XboxController.Button.kB.value);

    /* Operator Buttons */
    private final JoystickButton collectionRunMotor   = new JoystickButton(operator, XboxController.Button.kRightBumper.value);
    private final JoystickButton collectionStopMotor  = new JoystickButton(operator, XboxController.Button.kLeftStick.value);
    private final JoystickButton collectionEjectMotor = new JoystickButton(operator, XboxController.Button.kLeftBumper.value);
    // private final JoystickButton armResetSensor       = new JoystickButton(operator, XboxController.Button.kA.value);
    // private final JoystickButton wristResetSensor     = new JoystickButton(operator, XboxController.Button.kB.value);
    private final JoystickButton armGoHome            = new JoystickButton(operator, XboxController.Button.kA.value);
    private final JoystickButton armSetPosition       = new JoystickButton(operator, XboxController.Button.kB.value);
    private final JoystickButton wristGoHome          = new JoystickButton(operator, XboxController.Button.kX.value);
    private final JoystickButton wristSetPosition     = new JoystickButton(operator, XboxController.Button.kY.value);

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
        togglePipeline.onTrue(vision.togglePipeline());
        followTarget.whileTrue(new FollowTape(s_Swerve));
        lockRobot.onTrue(new InstantCommand(() -> s_Swerve.setX()));

        wristSetPosition.onTrue(wristSubsystem.setPosition());   
        wristGoHome.onTrue(wristSubsystem.slowlyGoDown());

        armSetPosition.onTrue(armSubsystem.setPosition());
        armGoHome.onTrue(armSubsystem.goToHome());
        
        
        collectionStopMotor.onTrue(armSubsystem.stop());
        collectionRunMotor.onTrue(collectionSubsystem.collectCube());
        collectionEjectMotor.onTrue(collectionSubsystem.collectCone());
    }

    /**
     * Use this to pass the autonomous command to the main k Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return new exampleAuto(s_Swerve, vision);
        //return new PPauto(s_Swerve, GameAuto);
    }

    /**
     * Use this to pass the disabled command to the main k Robot} class.
     * 
     *  @return the command to run in autonomous
     */
    public Command getDisableCommand(){
        return new Disable(s_Swerve);
    }
}