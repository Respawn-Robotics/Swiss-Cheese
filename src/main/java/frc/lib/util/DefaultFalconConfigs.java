package frc.lib.util;

import com.ctre.phoenix.motorcontrol.can.TalonFX;

public class DefaultFalconConfigs {
public void SetDefaultStatusFrames(TalonFX motor){
    motor.setStatusFramePeriod(1, 10);
    motor.setStatusFramePeriod(2, 10);
    motor.setStatusFramePeriod(3, 255);
    motor.setStatusFramePeriod(4, 255);
    motor.setStatusFramePeriod(8, 255);
    motor.setStatusFramePeriod(10, 10);
    motor.setStatusFramePeriod(12, 10);
    motor.setStatusFramePeriod(13, 10);
    motor.setStatusFramePeriod(14, 20);
    motor.setStatusFramePeriod(21, 10);

}

public void SetFollowerStatusFrames(TalonFX motor){
    motor.setStatusFramePeriod(1, 255);
    motor.setStatusFramePeriod(2, 255);
    motor.setStatusFramePeriod(3, 255);
    motor.setStatusFramePeriod(4, 255);
    motor.setStatusFramePeriod(8, 255);
    motor.setStatusFramePeriod(10, 10);
    motor.setStatusFramePeriod(12, 10);
    motor.setStatusFramePeriod(13, 10);
    motor.setStatusFramePeriod(14, 20);
    motor.setStatusFramePeriod(21, 10);

}
}
