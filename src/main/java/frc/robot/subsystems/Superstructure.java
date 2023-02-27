package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.JointMovementType;
import frc.robot.Constants.WristConstants;
import frc.robot.commands.JointsSetPosition;
import frc.robot.drivers.BeamBreak;

public class Superstructure extends SubsystemBase {

    private ArmSubsystem armSubsystem;
    private WristSubsystem wristSubsystem;
    private CollectionSubsystem collectionSubsystem;
    private Joystick operator;

    public Superstructure(ArmSubsystem armSubsystem, WristSubsystem wristSubsystem, CollectionSubsystem collectionSubsystem, Joystick operator) {
        this.armSubsystem = armSubsystem;
        this.wristSubsystem = wristSubsystem;
        this.collectionSubsystem = collectionSubsystem;
        this.operator = operator;
    }
    @Override
    public void periodic() {
        RobotContainer.coneBeamBreak.update();
        RobotContainer.cubeBeamBreak.update();
        
        // SmartDashboard.putBoolean("Cube Get", cubeBeamBreak.get());
        // SmartDashboard.putBoolean("Cube Tripped", cubeBeamBreak.wasTripped());

        if(RobotContainer.cubeBeamBreak.wasTripped() || RobotContainer.coneBeamBreak.wasTripped()) {
            new PrintCommand("TRIPPED").schedule();
            new WaitCommand(0.5)
                .andThen(new JointsSetPosition(ArmConstants.HOME, WristConstants.HOME, 1, 0.4, armSubsystem, wristSubsystem))
                .schedule();
            new WaitCommand(0.45)
                .andThen(collectionSubsystem.holdPosition())
                .schedule();
        }
        if(RobotContainer.cubeBeamBreak.wasCleared() || RobotContainer.coneBeamBreak.wasCleared()) {
            new WaitCommand(0.5)
                .andThen(new JointsSetPosition(ArmConstants.HOME, WristConstants.HOME, 1, 0.4, armSubsystem, wristSubsystem))
                .schedule();
            new WaitCommand(0.45)
                .andThen(collectionSubsystem.stopMotor())
                .schedule();
        }

        if(operator.getRawAxis(2) >= 0.2) {
            collectionSubsystem.ejectCone().schedule();
        } else if (operator.getRawAxis(3) >= 0.2){
            collectionSubsystem.shootCube().schedule();
        }
    }
}
