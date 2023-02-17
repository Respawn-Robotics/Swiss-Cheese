package frc.robot.disabled;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;

public class Disable extends CommandBase {
  TalonFX FL = new TalonFX(1);
  TalonFX FR = new TalonFX(3);
  TalonFX BL = new TalonFX(5);
  TalonFX BR = new TalonFX(7);
  
  public Disable(Swerve s_Swerve) {
    addRequirements(s_Swerve);
  }

  @Override
  public void execute() {
    FL.setNeutralMode(NeutralMode.Coast);
    FR.setNeutralMode(NeutralMode.Coast);
    BL.setNeutralMode(NeutralMode.Coast);
    BR.setNeutralMode(NeutralMode.Coast);
  }
}