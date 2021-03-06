package frc.robot.Sensors;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;

public class NavxGyro {
    public AHRS ahrs;

    public NavxGyro(edu.wpi.first.wpilibj.I2C.Port kmxp) {
        ahrs = new AHRS(kmxp);
    }

    public void reset() {
        ahrs.reset();
    }

    public double getYawRadians() {
        return Math.toRadians(ahrs.getYaw() + 180);
    }

    public Rotation2d getHeading() {
        return new Rotation2d(-ahrs.getAngle());
    }

}
