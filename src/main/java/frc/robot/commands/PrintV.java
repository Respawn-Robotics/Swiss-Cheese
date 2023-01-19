package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LimelightSubsystem;

public class PrintV extends CommandBase {
  // The subsystem the command runs on
  private final LimelightSubsystem limelightSubsystem;

  public PrintV(LimelightSubsystem subsystem) {
    limelightSubsystem = subsystem;
    addRequirements(limelightSubsystem);
  }

  @Override
  public void initialize() {
   
  }

  @Override
  public void execute(){
    if(limelightSubsystem.isTargetValid()){
        //System.out.println("Tape Seen");
    }
}

  @Override
  public boolean isFinished() {
    return true;
  }
}