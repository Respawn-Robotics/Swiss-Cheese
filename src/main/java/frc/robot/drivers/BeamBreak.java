package frc.robot.drivers;

import edu.wpi.first.wpilibj.DigitalInput;

public class BeamBreak {

    private boolean lastStatus;

    private boolean tripped;
    private boolean cleared;

    private final DigitalInput bb;

    public BeamBreak(int channel) {
        bb = new DigitalInput(channel);
    }

    public void update() {
        boolean value = get();
        tripped = value && !lastStatus;
        cleared = !value && lastStatus;
        lastStatus = value;
    }

    public boolean get() {
        return !bb.get();
    }

    public boolean wasTripped() {
        return tripped;
    }

    public boolean wasCleared() {
        return cleared;
    }
}
