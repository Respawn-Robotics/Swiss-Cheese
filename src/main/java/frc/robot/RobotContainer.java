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
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
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
    private final JoystickButton zeroGyro                = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton robotCentric            = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    private final JoystickButton slowDrive            = new JoystickButton(driver, XboxController.Button.kRightBumper.value);
    private final JoystickButton followTarget            = new JoystickButton(driver, XboxController.Button.kA.value);
    private final JoystickButton togglePipeline          = new JoystickButton(driver, XboxController.Button.kX.value);
    private final JoystickButton lockRobot               = new JoystickButton(driver, XboxController.Button.kB.value);
    private final POVButton rightGyro                       = new POVButton(driver, 90);
    private final POVButton downGyro                      = new POVButton(driver, 180);
    private final POVButton leftGyro                     = new POVButton(driver, 270);


    /* Operator Buttons */
    private final JoystickButton collectionRunMotor      = new JoystickButton(operator, XboxController.Button.kRightBumper.value);
    private final JoystickButton collectionStopMotor     = new JoystickButton(operator, XboxController.Button.kRightStick.value);
    private final JoystickButton collectionEjectMotor    = new JoystickButton(operator, XboxController.Button.kLeftBumper.value);
    private final JoystickButton armUp              = new JoystickButton(operator, XboxController.Button.kLeftStick.value);
    private final JoystickButton armDown              = new JoystickButton(operator, XboxController.Button.kStart.value);
    private final JoystickButton wristUp              = new JoystickButton(operator, XboxController.Button.kBack.value);
    private final JoystickButton wristDown              = new JoystickButton(operator, XboxController.Button.kRightStick.value);
    private final JoystickButton A                       = new JoystickButton(operator, XboxController.Button.kA.value);
    private final JoystickButton B                       = new JoystickButton(operator, XboxController.Button.kB.value);
    private final JoystickButton X                       = new JoystickButton(operator, XboxController.Button.kX.value);
    private final JoystickButton Y                       = new JoystickButton(operator, XboxController.Button.kY.value);
    private final POVButton povUp                    = new POVButton(operator, 0);
    private final POVButton povDown                    = new POVButton(operator, 180);
    private final POVButton povLeft                     = new POVButton(operator, 90);
    private final POVButton povRight                    = new POVButton(operator, 270);

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
                () -> robotCentric.getAsBoolean(),
                false
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
        slowDrive.whileTrue(new InstantCommand(()-> s_Swerve.setDefaultCommand(
            new TeleopSwerve(
            s_Swerve, 
            () -> -driver.getRawAxis(translationAxis) *.75,
            () -> -driver.getRawAxis(strafeAxis) * .75,
            () -> -driver.getRawAxis(rotationAxis) * .75,
            () -> robotCentric.getAsBoolean(),
            true
        ))));
        
        // Gyro Offsets
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
        rightGyro.onTrue(new InstantCommand(() -> s_Swerve.rightGyro()));
        downGyro.onTrue(new InstantCommand(() -> s_Swerve.downGyro()));
        leftGyro.onTrue(new InstantCommand(() -> s_Swerve.leftGyro()));


        togglePipeline.onTrue(vision.togglePipeline());
        followTarget.whileTrue(new FollowTape(s_Swerve));
        lockRobot.onTrue(new InstantCommand(() -> s_Swerve.setX()));

        /* Operator Controls */

        // Pick up off ground
        A.onTrue(armSubsystem.setPosition(12000).andThen(wristSubsystem.setPosition(90000).andThen(new WaitCommand(.5).andThen(armSubsystem.setPosition(14000)))));

        // Home Position
        B.onTrue(armSubsystem.setPosition(0).andThen(wristSubsystem.setPosition(0)));

        // Score in mid
        X.onTrue(armSubsystem.setPosition(60000).andThen(wristSubsystem.setPosition(152000)));

        // Score in high cone
        Y.onTrue(armSubsystem.setPosition(60000).andThen(wristSubsystem.setPosition(130000)));
        povRight.onTrue(armSubsystem.setPosition(60000).andThen(wristSubsystem.setPosition(130000)));

        // Acqure from DS
        povUp.onTrue(armSubsystem.setPosition(60000).andThen(wristSubsystem.setPosition(140000)));

        wristUp.whileTrue(new ManualWristUp(wristSubsystem));
        wristDown.whileTrue(new ManualWristDown(wristSubsystem));
        armDown.whileTrue(new ManualArmUp(armSubsystem));
        armUp.whileTrue(new ManualArmDown(armSubsystem));

        povDown.onTrue(armSubsystem.resetSensor().andThen(wristSubsystem.resetSensor()));
        collectionStopMotor.onTrue(collectionSubsystem.stopMotor());
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