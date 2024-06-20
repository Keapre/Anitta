package org.firstinspires.ftc.teamcode.util;


import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.AnalogInput;

public class CRServoEncoder implements CRServo {
    public CRServo servo;
    public AnalogInput delegateEncoder;

    public double relativePos = 0;
    public double absolutePos = 0;
    public double cachedPose = 0;

    public CRServoEncoder(CRServo servo,AnalogInput delegateEncoder) {
        this.servo = servo;
        this.delegateEncoder = delegateEncoder;
        setAbsolutePos(getRelativePos());
        cachedPose  = getRelativePos();

    }

    public double getVoltage() {
        return delegateEncoder.getVoltage();
    }

    public double getAbsolutePos() {
        return absolutePos;
    }

    public void update() {
        double currentRelative = getRelativePos();
        double power = getPower();
        if(power > 0) {
            //absolute creste
            if(currentRelative > cachedPose) {
                absolutePos+=(currentRelative - cachedPose);
            }else {
                //inseamna ca a fct full circle
                absolutePos+=(currentRelative + (360-cachedPose));
            }
        }else {
             if(currentRelative<cachedPose) {
                 absolutePos-=(cachedPose-currentRelative);
             }else {
                 absolutePos-=(cachedPose + (360 - relativePos));
             }
        }
        cachedPose = currentRelative;

    }
    public double getRelativePos() {
        return delegateEncoder.getVoltage() /3.3 * 360;
    }

    public void setAbsolutePos(double _absolute) {
        absolutePos = _absolute;
    }
    @Override
    public ServoController getController() {
        return null;
    }

    @Override
    public int getPortNumber() {
        return 0;
    }

    @Override
    public void setDirection(Direction direction) {

    }

    @Override
    public Direction getDirection() {
        return null;
    }

    @Override
    public void setPower(double power) {

    }

    @Override
    public double getPower() {
        return 0;
    }

    @Override
    public Manufacturer getManufacturer() {
        return null;
    }

    @Override
    public String getDeviceName() {
        return null;
    }

    @Override
    public String getConnectionInfo() {
        return null;
    }

    @Override
    public int getVersion() {
        return 0;
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {

    }

    @Override
    public void close() {

    }
}
