package frc.robot.subsystems.Swerve;

import java.lang.Character.Subset;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Sensors.NavxGyro;

public class SwerveSubsytem extends SubsystemBase {
    private double wheel_size_in_meters = Units.inchesToMeters(3.5);
    private SwerveModule galit = new SwerveModule(0, 1, wheel_size_in_meters);
    private SwerveModule idan = new SwerveModule(0, 1, wheel_size_in_meters);
    private SwerveModule amalia = new SwerveModule(0, 1, wheel_size_in_meters);
    private SwerveModule iris = new SwerveModule(0, 1, wheel_size_in_meters);
    private Translation2d galit_location = new Translation2d(0, 0); // I ++ FL
    private Translation2d idan_location = new Translation2d(0, 0); // II -+ BL
    private Translation2d amalia_location = new Translation2d(0, 0);// III -- BR
    private Translation2d iris_location = new Translation2d(0, 0); // IV +- FR
    private SwerveDriveKinematics swerve_kinematics = new SwerveDriveKinematics(galit_location, iris_location,
            idan_location, amalia_location);
    private ChassisSpeeds swerve_speeds;
    private NavxGyro gyro = new NavxGyro(Port.kMXP);
    private SwerveDriveOdometry swerve_odemetry = new SwerveDriveOdometry(swerve_kinematics, gyro.getHeading(),
            new Pose2d(0, 0, new Rotation2d())); // update start position
    private Pose2d current_pose;

    public SwerveSubsytem() {

    }

    public void set_modules(double x_velocity, double y_velocity, double rotation_velocity) {
        this.swerve_speeds = new ChassisSpeeds(x_velocity, y_velocity, rotation_velocity);
        SwerveModuleState[] target_states = this.swerve_kinematics.toSwerveModuleStates(this.swerve_speeds);
        galit.set_module_state(target_states[0]);
        iris.set_module_state(target_states[1]);
        idan.set_module_state(target_states[2]);
        amalia.set_module_state(target_states[3]);
    }

    @Override
    public void periodic() {
        this.current_pose = swerve_odemetry.update(gyro.getHeading(), galit.module_state, iris.module_state,
                idan.module_state, amalia.module_state);
    }

    public void set_modules_field_orianted(double x_velocity, double y_velocity, double rotation_velocity) {
        // maybe needed to add angle offset
        this.swerve_speeds = ChassisSpeeds.fromFieldRelativeSpeeds(x_velocity, y_velocity, rotation_velocity,
                gyro.getHeading());
        SwerveModuleState[] target_states = this.swerve_kinematics.toSwerveModuleStates(this.swerve_speeds);
        galit.set_module_state(target_states[0]);
        iris.set_module_state(target_states[1]);
        idan.set_module_state(target_states[2]);
        amalia.set_module_state(target_states[3]);
    }

}
