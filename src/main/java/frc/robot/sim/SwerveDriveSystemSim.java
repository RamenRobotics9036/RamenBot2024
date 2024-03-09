package frc.robot.sim;

import static simulationlib.simulation.swerve.SwerveSimConstants.Swerve.kModuleTranslations;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.SwerveDriveSystemAbstract;
import frc.robot.util.AppliedController;
import java.util.function.Supplier;
import simulationlib.shuffle.MultiType;
import simulationlib.shuffle.PrefixedConcurrentMap;
import simulationlib.shuffle.PrefixedConcurrentMap.Client;
import simulationlib.simulation.swerve.SwerveDrive;
import simulationlib.simulation.swerve.SwerveSimConstants.Swerve;

/**
 * Subclass of TankDriveSystem that is used for simulation. Note that this code isn't run if
 * the robot is not running in simulation mode.
 */
public class SwerveDriveSystemSim extends SwerveDriveSystemAbstract {
    private SwerveDrive m_simSwerveDrive;
    private final Field2d m_field2d = new Field2d();
    private final Pose2d[] m_robotPose = {
            new Pose2d()
    };
    private final Pose2d[] m_swerveModulePoses = {
            new Pose2d(), // Front Left
            new Pose2d(), // Front Right
            new Pose2d(), // Back Left
            new Pose2d() // Back Right
    };

    /**
     * Factory method to create a SwerveDriveSystemSim or SwerveDriveSystem object.
     */
    public static SwerveDriveSystemAbstract createSwerveDriveSystemInstance(
            AppliedController controller) {
        SwerveDriveSystemAbstract result;

        if (RobotBase.isSimulation()) {
            result = new SwerveDriveSystemSim(controller);
        }
        else {
            throw new UnsupportedOperationException("Not yet implemented");
            // result = new SwerveDriveSystem(controller);
        }

        return result;
    }

    /**
     * Constructor.
     */
    public SwerveDriveSystemSim(AppliedController controller) {
        m_simSwerveDrive = new SwerveDrive();

        // $TODO - This can go away later. For now, expose pose for Shuffleboard
        Client<Supplier<MultiType>> shuffleClient = PrefixedConcurrentMap
                .createShuffleboardClientForSubsystem("SwerveSystem");
        shuffleClient.addItem("RobotPose", () -> MultiType.of(m_robotPose[0]));
    }

    private void updateRobotPoses() {
        m_robotPose[0] = m_simSwerveDrive.getPoseMeters();
        m_field2d.setRobotPose(m_robotPose[0]);

        for (int i = 0; i < kModuleTranslations.length; i++) {
            Translation2d updatedPositions = kModuleTranslations[i]
                    .rotateBy(m_simSwerveDrive.getPoseMeters().getRotation())
                    .plus(m_simSwerveDrive.getPoseMeters().getTranslation());
            m_swerveModulePoses[i] = new Pose2d(updatedPositions,
                    m_simSwerveDrive.getSwerveModule(i)
                            .getHeadingRotation2d().plus(m_simSwerveDrive.getHeadingRotation2d()));
        }

        m_field2d.getObject("Swerve Modules").setPoses(m_swerveModulePoses);
    }

    @Override
    public void periodic() {
        super.periodic();

        m_simSwerveDrive.periodic();
    }

    @Override
    public void simulationPeriodic() {
        super.simulationPeriodic();

        m_simSwerveDrive.simulationPeriodic();

        updateRobotPoses();
        SmartDashboard.putData("Field2d", m_field2d);
    }

    @Override
    public void drive(double xspeed, double yspeed, double rot, boolean fieldRelative) {
        // super.drive(xspeed, yspeed, rot, fieldRelative);

        System.out.println(
                "$IDO: drive: " + xspeed + ", " + yspeed + ", " + rot + ", " + fieldRelative);

        m_simSwerveDrive.drive(xspeed, yspeed, rot, fieldRelative, true);
    }

    @Override
    public double getxPosition() {
        if (m_simSwerveDrive != null) {
            return m_simSwerveDrive.getPoseMeters().getX();
        }
        else {
            return 0;
        }
    }

    @Override
    public double getyPosition() {
        if (m_simSwerveDrive != null) {
            return m_simSwerveDrive.getPoseMeters().getY();
        }
        else {
            return 0;
        }
    }

    @Override
    public boolean resetGyroFieldRelative() {
        // super.resetGyroFieldRelative();

        if (m_simSwerveDrive != null) {
            // $TODO
            return true;
        }
        else {
            return true;
        }
    }

    @Override
    public boolean resetGyroFieldRelativeAuto() {
        // super.resetGyroFieldRelativeAuto();

        if (m_simSwerveDrive != null) {
            // $TODO
            return true;
        }
        else {
            return true;
        }
    }

    @Override
    public double getAnglePosition() {
        // Read this off of the simulated Pigeon Gyro
        if (m_simSwerveDrive != null) {
            return m_simSwerveDrive.getHeadingDegrees();
        }
        else {
            return 0;
        }
    }

    @Override
    public Rotation2d getRotation2d() {
        if (m_simSwerveDrive != null) {
            return m_simSwerveDrive.getHeadingRotation2d();
        }
        else {
            return new Rotation2d();
        }
    }

    @Override
    public double getFrontLeftTurnEncoder() {
        throw new UnsupportedOperationException("Not yet implemented");
    }

    @Override
    public double getFrontRightTurnEncoder() {
        throw new UnsupportedOperationException("Not yet implemented");
    }

    @Override
    public double getBackLeftTurnEncoder() {
        throw new UnsupportedOperationException("Not yet implemented");
    }

    @Override
    public double getBackRightTurnEncoder() {
        throw new UnsupportedOperationException("Not yet implemented");
    }

    @Override
    public double getFrontLeftDriveEncoder() {
        if (m_simSwerveDrive != null) {
            return m_simSwerveDrive.getSwerveModule(0).getDriveMeters();
        }
        else {
            return 0;
        }
    }

    @Override
    public double getFrontRightDriveEncoder() {
        throw new UnsupportedOperationException("Not yet implemented");
    }

    @Override
    public double getBackLeftDriveEncoder() {
        throw new UnsupportedOperationException("Not yet implemented");
    }

    @Override
    public double getBackRightDriveEncoder() {
        throw new UnsupportedOperationException("Not yet implemented");
    }

    @Override
    public double getFrontLeftDriveVelocity() {
        if (m_simSwerveDrive != null) {
            return m_simSwerveDrive.getSwerveModule(0).getDriveMetersPerSecond();
        }
        else {
            return 0;
        }
    }

    @Override
    public double getFrontRightDriveVelocity() {
        if (m_simSwerveDrive != null) {
            return m_simSwerveDrive.getSwerveModule(1).getDriveMetersPerSecond();
        }
        else {
            return 0;
        }
    }

    @Override
    public double getBackLeftDriveVelocity() {
        if (m_simSwerveDrive != null) {
            return m_simSwerveDrive.getSwerveModule(2).getDriveMetersPerSecond();
        }
        else {
            return 0;
        }
    }

    @Override
    public double getBackRightDriveVelocity() {
        if (m_simSwerveDrive != null) {
            return m_simSwerveDrive.getSwerveModule(3).getDriveMetersPerSecond();
        }
        else {
            return 0;
        }
    }

    @Override
    public void stopSystem() {
        // super.stopSystem();

        if (m_simSwerveDrive != null) {
            ChassisSpeeds chassisSpeeds = new ChassisSpeeds();
            m_simSwerveDrive.driveFromChassisSpeeds(chassisSpeeds, true);
        }
    }

    //
    // These are primarily used for autonomous PathPlanner
    //

    @Override
    public ChassisSpeeds getSpeeds() {
        if (m_simSwerveDrive != null) {
            return m_simSwerveDrive.getChassisSpeeds();
        }
        else {
            return new ChassisSpeeds();
        }
    }

    @Override
    public double getDriveBaseRadius() {
        return Swerve.kModuleTranslations[0].getNorm();
    }

    @Override
    public void resetPose(Pose2d pose) {
        // super.resetPose(pose);

        // $TODO
        // if (m_simSwerveDrive != null) {
        m_simSwerveDrive.resetPose(pose);
        // }
    }

    @Override
    public void driveFromChassisSpeeds(ChassisSpeeds chassisSpeeds) {
        // super.driveFromChassisSpeeds(chassisSpeeds);

        System.out.println("$IDO: driveFromChassisSpeeds");

        if (m_simSwerveDrive != null) {
            m_simSwerveDrive.driveFromChassisSpeeds(chassisSpeeds, true);
        }
    }

    @Override
    public Pose2d getPoseMeters() {
        if (m_simSwerveDrive != null) {
            return m_simSwerveDrive.getPoseMeters();
        }
        else {
            return new Pose2d();
        }
    }

    @Override
    public void toTeleop() {
    }

    @Override
    public void toAuto() {
    }

    @Override
    public void setFieldRelative(boolean fieldRelative) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setFieldRelative'");
    }

    @Override
    public void drive(double xspeed, double yspeed, double rot) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'drive'");
    }

    @Override
    public boolean resetGyroFieldRelativeRedTop() {
        return true;
    }

    @Override
    public boolean resetGyroFieldRelativeRedBottom() {
        return true;
    }

    @Override
    public boolean resetGyroFieldRelativeBlueTop() {
        return true;
    }

    @Override
    public boolean resetGyroFieldRelativeAutoRed() {
        return true;
    }

    @Override
    public boolean resetGyroFieldRelativeBlueBottom() {
        return true;
    }

    @Override
    public void setStatus(boolean[] status) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setStatus'");
    }
}
