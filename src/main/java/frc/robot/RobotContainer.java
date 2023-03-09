package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.autos.autoCommands.*;
import frc.robot.commands.*;
import frc.robot.commands.manual.ManualArmDown;
import frc.robot.commands.manual.ManualArmUp;
import frc.robot.commands.manual.ManualWristDown;
import frc.robot.commands.manual.ManualWristUp;
import frc.robot.commands.operator.OperatorCommands;
import frc.robot.disabled.Disable;
import frc.robot.drivers.BeamBreak;
import frc.robot.shuffleboard.ShuffleboardConfig;
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

    /* Config */
    ShuffleboardConfig shuffleboardConfig = new ShuffleboardConfig();

    /* Controllers */
    private final Joystick driver = new Joystick(0);
    private final Joystick operator = new Joystick(1);

    /* Subsystem */
    public final CollectionSubsystem collectionSubsystem = new CollectionSubsystem();
    public final ArmSubsystem armSubsystem = new ArmSubsystem();
    public final WristSubsystem wristSubsystem = new WristSubsystem();
    private final Swerve s_Swerve = new Swerve();
    private final Vision vision = new Vision();
    private final OperatorCommands operatorCommands = new OperatorCommands(armSubsystem, wristSubsystem, collectionSubsystem);
    private final Superstructure superstructure = new Superstructure(armSubsystem, wristSubsystem, collectionSubsystem, operator, operatorCommands);
    private final FixOrientation level = new FixOrientation(s_Swerve);

    public static BeamBreak cubeBeamBreak = new BeamBreak(2);
    public static BeamBreak coneBeamBreak = new BeamBreak(1);

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;
    //private SlewRateLimiter limit = Constants.rateLimit;


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

    /* Trajectories */
    
    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        s_Swerve.setDefaultCommand(
                new TeleopSwerve(
                        s_Swerve,
                        () -> -driver.getRawAxis(translationAxis),/*limit.calculate(*/
                        () -> -driver.getRawAxis(strafeAxis),/*limit.calculate(*/
                        () -> -driver.getRawAxis(rotationAxis),
                        () -> d_leftBumper.getAsBoolean()));
        
        // Configure the button 
        configureButtonBindings();
    }

    private void configureButtonBindings() {

        /* Driver Buttons */

        // Set slow drive mode
        d_rightBumper.onTrue(new InstantCommand(() -> s_Swerve.setSlow(true)));
        d_rightBumper.onFalse(new InstantCommand(() -> s_Swerve.setSlow(false)));

        // Gyro Offsets
        d_Y.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
        d_povRight.onTrue(new InstantCommand(() -> s_Swerve.leftGyro()));
        d_povDown.onTrue(new InstantCommand(() -> s_Swerve.downGyro()));
        d_povLeft.onTrue(new InstantCommand(() -> s_Swerve.rightGyro()));

        // Toggle Limelight pipeline
        d_X.onTrue(vision.togglePipeline());

        // Follow retroreflective tape/level
        d_A.onTrue(level);//new InstantCommand(()-> s_Swerve.setX()));

        // Home arm
        d_B.onTrue(operatorCommands.goToHome().andThen(collectionSubsystem.stopMotor()));

        /* Operator Controls */
        
        // Acquire cone ground
        o_A.and(o_leftStick.negate())
            .onTrue(operatorCommands.acquireConeFromFloor());
        
        // Acquire cube ground
        o_A.and(o_leftStick)
            .onTrue(operatorCommands.acquireCubeFromFloor());
        
        o_X.onTrue(armSubsystem.resetSensor().andThen(wristSubsystem.resetPos()));

        // Acquire cone DoS
        o_Y.and(o_leftStick.negate())
            .onTrue(operatorCommands.acquireConeFromDoS());

        // Acquire cube DoS
        o_Y.and(o_leftStick)
            .onTrue(operatorCommands.acquireCubeFromDoS());
        
        // Score high cone
        o_povUp.and(o_rightStick.negate())
            .onTrue(operatorCommands.scoreInHighCone());

        // Score high cube
        o_povUp.and(o_rightStick)
            .onTrue(operatorCommands.scoreInHighCube());

        // Score mid cone
        o_povLeft.and(o_rightStick.negate())
            .onTrue(operatorCommands.scoreInMidCone());

        // Score mid cube
        o_povLeft.and(o_rightStick)
            .onTrue(operatorCommands.scoreInMidCube());
        
        // Score low cone
        o_povDown.and(o_rightStick.negate())
            .onTrue(operatorCommands.scoreInLowCone());
        
        // Score low cube
        o_povDown.and(o_rightStick)
            .onTrue(operatorCommands.scoreInLowCube());
        
        o_povRight.onTrue(collectionSubsystem.holdPosition());

        // Manual Arm and Wrist
        // o_leftStick.whileTrue(new ManualArmUp(armSubsystem));
        // o_rightStick.whileTrue(new ManualArmDown(armSubsystem));
        o_start.whileTrue(new ManualArmUp(armSubsystem));
        o_back.whileTrue(new ManualArmDown(armSubsystem));
        o_rightBumper.whileTrue(new ManualWristUp(wristSubsystem));
        o_leftBumper.whileTrue(new ManualWristDown(wristSubsystem));
        


        // Collection Controls
        //o_rightBumper.onTrue(collectionSubsystem.collectCube());
        //o_leftBumper.onTrue(collectionSubsystem.alterCube());
        o_povDown.onTrue(collectionSubsystem.stopMotor());
    }

    /**
     * Use this to pass the autonomous command to the main k Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        //return new D1OnePieceDrive(s_Swerve,armSubsystem,wristSubsystem,collectionSubsystem, vision);
        return new D2CubeEngage(s_Swerve,armSubsystem,wristSubsystem,collectionSubsystem, vision,level);
        //return new D3OnePieceDrive(s_Swerve,armSubsystem,wristSubsystem,collectionSubsystem, vision);
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