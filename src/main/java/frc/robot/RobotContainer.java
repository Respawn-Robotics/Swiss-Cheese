package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.autos.*;
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
    ShuffleboardConfig shuffleboardConfig;

    /* Controllers */
    private final Joystick driver = new Joystick(0);
    private final Joystick operator = new Joystick(1);

    /* Subsystem */
    public final CollectionSubsystem collectionSubsystem = new CollectionSubsystem();
    public final ArmSubsystem armSubsystem = new ArmSubsystem();
    public final WristSubsystem wristSubsystem = new WristSubsystem();
    private final Swerve s_Swerve = new Swerve();
    private final Vision vision = new Vision();
    private final FixOrientation level = new FixOrientation(s_Swerve);
    private final CANDleSubsystem candle = new CANDleSubsystem();
    private final OperatorCommands operatorCommands = new OperatorCommands(armSubsystem, wristSubsystem, collectionSubsystem, candle);
    private final Superstructure superstructure = new Superstructure(armSubsystem, wristSubsystem, collectionSubsystem, operator, driver, operatorCommands, candle);

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
    private final JoystickButton d_rightStick = new JoystickButton(driver, XboxController.Button.kRightStick.value);
    private final POVButton      d_povRight = new POVButton(driver, 90);
    private final POVButton      d_povDown = new POVButton(driver, 180);
    private final POVButton      d_povLeft = new POVButton(driver, 270);
    private final POVButton      d_povUp = new POVButton(driver, 0);
    private final JoystickButton d_start = new JoystickButton(driver, XboxController.Button.kStart.value);
    private final JoystickButton d_back = new JoystickButton(driver, XboxController.Button.kBack.value);
    

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
        
        shuffleboardConfig = new ShuffleboardConfig();
        candle.setBlue();
        // Configure the button 
        configureButtonBindings();
    }

    private void configureButtonBindings() {

        /* Driver Buttons */

        // Set slow drive mode
        d_rightBumper.onTrue(new InstantCommand(() -> s_Swerve.setSlow(true)));
        d_rightBumper.onFalse(new InstantCommand(() -> s_Swerve.setSlow(false)));

        // Stop collection motor
        d_povUp.onTrue(collectionSubsystem.shootCube());

        d_rightStick.onTrue(collectionSubsystem.stopMotor());

        // Gyro Offsets
        d_povRight.onTrue(new InstantCommand(() -> s_Swerve.leftGyro()));
        d_povDown.onTrue(new InstantCommand(() -> s_Swerve.downGyro()));
        d_povLeft.onTrue(new InstantCommand(() -> s_Swerve.rightGyro()));

        // Lock Modules
        d_A.whileTrue(new FollowTape(s_Swerve));

        // Home arm
        d_B.onTrue(operatorCommands.goToHome());

        // Lock Robot
        d_X.onTrue(level);//new InstantCommand(()-> s_Swerve.setX())

        // Gyro Offset
        d_Y.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));

        // Set 0
        d_start.whileTrue(new SetHeading(s_Swerve, 0));

        // Set 180
        d_back.whileTrue(new SetHeading(s_Swerve, 180));

        /* Operator Controls */
        
        // Acquire cone ground
        o_leftStick
            .onTrue(operatorCommands.acquireConeFromFloor());
        
        // Acquire cube ground
        o_A.and(o_leftStick.negate())
            .onTrue(operatorCommands.acquireCubeFromFloor().alongWith(candle.setRed()));
        
        o_X.onTrue(armSubsystem.resetSensor().andThen(wristSubsystem.resetPos()));
        
        o_B.onTrue(operatorCommands.acquireCubeFromDoS());

        // Acquire cone DoS
        o_Y.and(o_leftStick.negate())
            .onTrue(operatorCommands.acquireConeFromDoS().alongWith(candle.setRed()));

        // Acquire cube DoS
        o_Y.and(o_leftStick)
            .onTrue(operatorCommands.acquireCubeFromDoS().alongWith(candle.setRed()));
        
        // Score high cone
        o_povUp.and(o_rightStick.negate())
            .onTrue(operatorCommands.scoreInHighCone().alongWith(candle.setYellow()));

        // Score high cube
        o_povUp.and(o_rightStick)
            .onTrue(operatorCommands.scoreInHighCone().alongWith(candle.setPurple()));

        // Score mid cone
        o_povLeft.and(o_rightStick.negate())
            .onTrue(operatorCommands.scoreInMidCone().alongWith(candle.setYellow()));

        // Score mid cube
        o_povLeft.and(o_rightStick)
            .onTrue(operatorCommands.scoreInMidCube().alongWith(candle.setPurple()));
        
        // Score low cone
        o_povDown.and(o_rightStick.negate())
            .onTrue(operatorCommands.scoreInLowCone().alongWith(candle.setYellow()));
        
        // Score low cube
        o_povDown.and(o_rightStick)
            .onTrue(operatorCommands.scoreInLowCube().alongWith(candle.setPurple()));
        
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
        o_povDown.onTrue(operatorCommands.scoreInLowCube());
    }

    /**
     * Use this to pass the autonomous command to the main k Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        //return new D1OnePieceDrive(s_Swerve,armSubsystem,wristSubsystem,collectionSubsystem, vision);
        SendableChooser<String> val = (SendableChooser)SmartDashboard.getData("Auton Chooser");
        switch (val.getSelected()) {
            case "D1ConeCubeHighE":
                return new D1ConeCubeE(s_Swerve, armSubsystem, wristSubsystem, collectionSubsystem,level);
            case "D1ConeCubeHighPC":
                return new D1ConeCubePC(s_Swerve, armSubsystem, wristSubsystem, collectionSubsystem);
            case "D1OneCone":
                return new D1OneCone(s_Swerve, armSubsystem, wristSubsystem, collectionSubsystem);
            case "D2ConeCubeE":
                return new D2ConeCubeE(s_Swerve, armSubsystem, wristSubsystem, collectionSubsystem, level); 
            case "D2ConeCube":
                return new D2ConeCube(s_Swerve, armSubsystem, wristSubsystem, collectionSubsystem); 
            case "D2ConeE":
                return new D2ConeE(s_Swerve, armSubsystem, wristSubsystem, collectionSubsystem,level);
            case "D2Cone":
                return new D2Cone(s_Swerve, armSubsystem, wristSubsystem, collectionSubsystem);
            case "D2CubeE":
                return new D2CubeE(s_Swerve, armSubsystem, wristSubsystem, collectionSubsystem,level);
            case "D2Cube":
                return new D2CubeStay(s_Swerve, armSubsystem, wristSubsystem, collectionSubsystem);
            case"D3ConeCube":
                return new D3ConeCube(s_Swerve, armSubsystem, wristSubsystem, collectionSubsystem);
            case "D3OneCone":
                return new D3OneCone(s_Swerve, armSubsystem, wristSubsystem, collectionSubsystem); 
            default:
                return null;
        }
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