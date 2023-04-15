package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.Constants.SuperstructureConstants.ROBOT_STATE;
import frc.robot.commands.operator.OperatorCommands;

public class Superstructure extends SubsystemBase {

    private ArmSubsystem armSubsystem;
    private WristSubsystem wristSubsystem;
    private CollectionSubsystem collectionSubsystem;
    private Joystick operator;
    private Joystick driver;
    private OperatorCommands operatorCommands;
    public static ROBOT_STATE currentRobotState;
    private CANDleSubsystem candle;

    public Superstructure(ArmSubsystem armSubsystem, WristSubsystem wristSubsystem, CollectionSubsystem collectionSubsystem, Joystick operator, Joystick driver, OperatorCommands operatorCommands, CANDleSubsystem candle) {
        this.armSubsystem = armSubsystem;
        this.wristSubsystem = wristSubsystem;
        this.collectionSubsystem = collectionSubsystem;
        this.operator = operator;
        this.driver = driver;
        this.operatorCommands = operatorCommands;
        this.candle = candle;
    }
    @Override
    public void periodic() {
        RobotContainer.coneBeamBreak.update();
        RobotContainer.cubeBeamBreak.update();

        SmartDashboard.putNumber("Collection Motor", collectionSubsystem.getMotor().getStatorCurrent());

        if(collectionSubsystem.getMotor().getStatorCurrent() > 30 && (operator.getRawButtonPressed(1) || operator.getRawButtonPressed(2))) {
            new PrintCommand("CONE TRIPPED").schedule();
            new WaitCommand(0)
                .andThen(collectionSubsystem.stopMotor())
                .andThen(collectionSubsystem.holdPosition())
                .schedule();
            
            armSubsystem.setPosition(armSubsystem.getMasterMotor().getSelectedSensorPosition() + 3000)
                .andThen(new WaitCommand(0.2)
                .alongWith(candle.setGreen())
                .andThen(wristSubsystem.setPosition(0)
                .andThen(new WaitCommand(.4)
                .andThen(armSubsystem.setPosition(0)
                .andThen(new WaitCommand(1)
                .andThen(candle.setBlue()))))))
                .schedule();
            // new InstantCommand(() -> driver.setRumble(RumbleType.kBothRumble, 1))
            //     .andThen(new WaitCommand(0.2)
            //     .andThen(new InstantCommand(() -> driver.setRumble(RumbleType.kBothRumble, 0))));
        }

        if(collectionSubsystem.getMotor().getStatorCurrent() > 50 && (operator.getRawButtonPressed(4))) {
            candle.setGreen()
                .andThen(new WaitCommand(1))
                .andThen(candle.setBlue())
                .schedule();
        }

        if(RobotContainer.cubeBeamBreak.wasCleared() || RobotContainer.coneBeamBreak.wasCleared()) {
            // new WaitCommand(0.5)
            //     .andThen(operatorCommands.goToHome())
            //     .schedule();
            new PrintCommand("CLEARED").schedule();
        }

        if(driver.getRawAxis(2) >= 0.2) {
            collectionSubsystem.puffCube().schedule();
        } else if (driver.getRawAxis(3) >= 0.2){
            collectionSubsystem.ejectCone().schedule();
        }
    }
}
