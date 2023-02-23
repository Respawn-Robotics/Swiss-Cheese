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
    private final Swerve s_Swerve                         = new Swerve();
    private final Vision vision                           = new Vision();

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons */
    private final JoystickButton zeroGyro                = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton robotCentric            = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    private final JoystickButton slowDrive               = new JoystickButton(driver, XboxController.Button.kRightBumper.value);
    private final JoystickButton followTarget            = new JoystickButton(driver, XboxController.Button.kA.value);
    private final JoystickButton togglePipeline          = new JoystickButton(driver, XboxController.Button.kX.value);
    private final JoystickButton lockRobot               = new JoystickButton(driver, XboxController.Button.kB.value);
    private final POVButton rightGyro                    = new POVButton(driver, 90);
    private final POVButton downGyro                     = new POVButton(driver, 180);
    private final POVButton leftGyro                     = new POVButton(driver, 270);



    /* Operator Buttons */
    private final JoystickButton rightBumper             = new JoystickButton(operator, XboxController.Button.kRightBumper.value);
    private final JoystickButton rightStick              = new JoystickButton(operator, XboxController.Button.kRightStick.value);
    private final JoystickButton leftBumper              = new JoystickButton(operator, XboxController.Button.kLeftBumper.value);
    private final JoystickButton leftStick               = new JoystickButton(operator, XboxController.Button.kLeftStick.value);
    private final JoystickButton start                   = new JoystickButton(operator, XboxController.Button.kStart.value);
    private final JoystickButton back                    = new JoystickButton(operator, XboxController.Button.kBack.value);
    private final JoystickButton A                       = new JoystickButton(operator, XboxController.Button.kA.value);
    private final JoystickButton B                       = new JoystickButton(operator, XboxController.Button.kB.value);
    private final JoystickButton X                       = new JoystickButton(operator, XboxController.Button.kX.value);
    private final JoystickButton Y                       = new JoystickButton(operator, XboxController.Button.kY.value);
    private final POVButton      povUp                   = new POVButton(operator, 0);
    private final POVButton      povTopRight             = new POVButton(operator, 45);
    private final POVButton      povRight                = new POVButton(operator, 90);
    private final POVButton      povBottomRight          = new POVButton(operator, 135);
    private final POVButton      povDown                 = new POVButton(operator, 180);
    private final POVButton      povBottomLeft           = new POVButton(operator, 225);
    private final POVButton      povLeft                 = new POVButton(operator, 270);
    private final POVButton      povTopLeft              = new POVButton(operator, 315);
    
    /* Trajectories */
    Trajectory Swervy, Test, straight, tpoint, GameAuto;
    
    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -driver.getRawAxis(translationAxis) *.75,
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

    private void configureButtonBindings() {

        /* Driver Buttons */
        slowDrive.onTrue(new InstantCommand(() -> s_Swerve.setSlow(true)));
        slowDrive.onFalse(new InstantCommand(() -> s_Swerve.setSlow(false)));

        // Gyro Offsets
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
        rightGyro.onTrue(new InstantCommand(() -> s_Swerve.rightGyro()));
        downGyro.onTrue(new InstantCommand(() -> s_Swerve.downGyro()));
        leftGyro.onTrue(new InstantCommand(() -> s_Swerve.leftGyro()));


        togglePipeline.onTrue(vision.togglePipeline());
        followTarget.whileTrue(new FollowTape(s_Swerve));
        lockRobot.onTrue(armSubsystem.setPosition(0).andThen(wristSubsystem.setPosition(0)));

        /* Operator Controls */

        // Acquire off ground
        A.onTrue(armSubsystem.setPosition(15000).andThen(wristSubsystem.setPosition(85000).andThen(new WaitCommand(.5).andThen(armSubsystem.setPosition(15000)))));

        // SS Position
        B.onTrue(armSubsystem.setPosition(25000).andThen(wristSubsystem.setPosition(47000)));

        // Acqure from DoS
        Y.onTrue(armSubsystem.setPosition(60000).andThen(wristSubsystem.setPosition(129000)));

        // Reset Sensors
        X.onTrue(armSubsystem.resetSensor().andThen(wristSubsystem.resetSensor()));

        // Score in high cone
        povLeft.onTrue(armSubsystem.setPosition(60000).andThen(wristSubsystem.setPosition(130000)));
        // Score in high cube
        povRight.onTrue(armSubsystem.setPosition(60000).andThen(wristSubsystem.setPosition(130000)));
        // Score in mid cone + cube
        povDown.onTrue(armSubsystem.setPosition(60000).andThen(wristSubsystem.setPosition(152000)));

        // Manual Arm and Wrist
        leftStick.whileTrue(new ManualWristUp(wristSubsystem));
        rightStick.whileTrue(new ManualWristDown(wristSubsystem));
        start.whileTrue(new ManualArmUp(armSubsystem));
        back.whileTrue(new ManualArmDown(armSubsystem));

        // Collection Controls
        rightStick.onTrue(collectionSubsystem.stopMotor());
        rightBumper.onTrue(collectionSubsystem.collectCube());
        leftBumper.onTrue(collectionSubsystem.collectCone());
        //povDown.onTrue(collectionSubsystem.shootCube());

    }

    /**
     * Use this to pass the autonomous command to the main k Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return new pathGroup(s_Swerve, armSubsystem,wristSubsystem,collectionSubsystem);
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