package org.firstinspires.ftc.teamcode.util.Priority;

public abstract class PriorityDevice {
    protected final double basePriority;
    protected final String name;
    protected double lastUpdateTime, callLengthMillis;
    boolean isUpdated = false;

    public PriorityDevice(double basePriority, String name) {
        this.basePriority = basePriority;
        this.name = name;
        lastUpdateTime = System.nanoTime();
    }


    protected abstract double getPriority(double timeRemaining);

    protected abstract void update();

    public void resetUpdateBoolean() {
        isUpdated = false;
    }
}
