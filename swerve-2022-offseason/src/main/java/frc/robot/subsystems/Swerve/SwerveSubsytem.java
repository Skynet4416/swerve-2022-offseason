package frc.robot.subsystems.Swerve;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Sensors.NavxGyro;

public class SwerveSubsytem extends SubsystemBase {
    private double wheel_size_in_meters = Units.inchesToMeters(3.5);
    private SwerveModule galit = new SwerveModule(0, 1, wheel_size_in_meters, "Galit");
    private SwerveModule idan = new SwerveModule(2, 3, wheel_size_in_meters, "Idan");
    private SwerveModule amalia = new SwerveModule(4, 5, wheel_size_in_meters, "Amalia");
    private SwerveModule iris = new SwerveModule(6, 7, wheel_size_in_meters, "Iris");
    private Translation2d galit_location = new Translation2d(1, 1); // I ++ FL
    private Translation2d idan_location = new Translation2d(-1, 1); // II -+ BL
    private Translation2d amalia_location = new Translation2d(-1, -1);// III -- BR
    private Translation2d iris_location = new Translation2d(1, -1); // IV +- FR
    private SwerveDriveKinematics swerve_kinematics = new SwerveDriveKinematics(galit_location, iris_location,
            idan_location, amalia_location);
    private ChassisSpeeds swerve_speeds;
    private Pose2d current_pose;
    private SwerveDriveOdometry swerve_odemetry;
    private final Field2d m_field = new Field2d();
    private NavxGyro gyro;
    int dev = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
    SimDouble angle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(dev, "Yaw"));

    public SwerveSubsytem(NavxGyro gyro) {
        this.gyro = gyro;
        swerve_odemetry = new SwerveDriveOdometry(swerve_kinematics, gyro.getHeading(),
                new Pose2d(0, 0, new Rotation2d())); // update start position
        SmartDashboard.putData("Field", this.m_field);
        set_pid();
        SmartDashboard.putNumber("Navx", gyro.getHeading().getDegrees());
    }

    public void set_pid() {
        set_pid_for_modulee("Galit");
        set_pid_for_modulee("Idan");
        set_pid_for_modulee("Amalia");
        set_pid_for_modulee("Iris");
    }

    public void set_pid_for_modulee(String name) {
        double[][] pid = get_contstants_by_name(name);
        SmartDashboard.putNumber(name + " top kd", pid[0][0]);
        SmartDashboard.putNumber(name + " top ki", pid[0][1]);
        SmartDashboard.putNumber(name + " top kp", pid[0][2]);
        SmartDashboard.putNumber(name + " top kff", pid[0][3]);

        SmartDashboard.putNumber(name + " buttom kd", pid[1][0]);
        SmartDashboard.putNumber(name + " buttom ki", pid[1][1]);
        SmartDashboard.putNumber(name + " buttom kp", pid[1][2]);
        SmartDashboard.putNumber(name + " buttom kff", pid[1][3]);

        SmartDashboard.putNumber(name + " wheel angle kd", pid[2][0]);
        SmartDashboard.putNumber(name + " wheel angle ki", pid[2][1]);
        SmartDashboard.putNumber(name + " wheel angle kp", pid[2][2]);
        SmartDashboard.putNumber(name + " wheel angle kff", pid[2][3]);

        SmartDashboard.putNumber(name + " wheel velocity kd", pid[3][0]);
        SmartDashboard.putNumber(name + " wheel velocity ki", pid[3][1]);
        SmartDashboard.putNumber(name + " wheel velocity kp", pid[3][2]);
        SmartDashboard.putNumber(name + " wheel velocity kff", pid[3][3]);

    }

    public double[][] get_contstants_by_name(String name) {
        return Constants.Swerve.modules.get_contstants_by_name(name);
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
        this.current_pose = swerve_odemetry.update(Rotation2d.fromDegrees(angle.get()), galit.module_state,
                iris.module_state,
                idan.module_state, amalia.module_state);
        SmartDashboard.putNumber("Gyro", angle.get());
        this.m_field.setRobotPose(this.current_pose);
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

    public void set_pos(Pose2d pos) {
        this.swerve_odemetry.resetPosition(pos, gyro.getHeading());
    }
}
