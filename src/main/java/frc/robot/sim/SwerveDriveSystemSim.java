package frc.robot.sim;

import static simulationlib.simulation.swerve.SwerveSimConstants.Swerve.kModuleTranslations;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.SwerveDriveSystem;
import frc.robot.util.AppliedController;
import java.util.function.Supplier;
import simulationlib.shuffle.MultiType;
import simulationlib.shuffle.PrefixedConcurrentMap;
import simulationlib.shuffle.PrefixedConcurrentMap.Client;
import simulationlib.simulation.swerve.SwerveDrive;

/**
 * Subclass of TankDriveSystem that is used for simulation. Note that this code isn't run if
 * the robot is not running in simulation mode.
 */
public class SwerveDriveSystemSim extends SwerveDriveSystem {
    private SwerveDrive m_simSwerveDrive;
    private final Field2d m_field2d = new Field2d();
    private final Pose2d[] m_robotPose = {
            new Pose2d()
    };
    private final Pose2d[] m_swerveModulePoses = {
            new Pose2d(), new Pose2d(), new Pose2d(), new Pose2d()
    };

    /**
     * Factory method to create a SwerveDriveSystemSim or SwerveDriveSystem object.
     */
    public static SwerveDriveSystem createTankDriveSystemInstance(AppliedController controller) {
        SwerveDriveSystem result;

        if (RobotBase.isSimulation()) {
            result = new SwerveDriveSystemSim(controller);
        }
        else {
            result = new SwerveDriveSystem(controller);
        }

        return result;
    }

    /**
     * Constructor.
     */
    public SwerveDriveSystemSim(AppliedController controller) {
        // FIRST, we call superclass
        super(controller);

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
        super.drive(xspeed, yspeed, rot, fieldRelative);

        m_simSwerveDrive.drive(xspeed, yspeed, rot, fieldRelative, false);
    }

    @Override
    public double getxPosition() {
        // $TODO
        return 0;
    }

    @Override
    public double getyPosition() {
        // $TODO
        return 0;
    }

    @Override
    public boolean resetGyroFieldRelative() {
        // $TODO
        return true;
    }

    @Override
    public boolean resetGyroFieldRelativeAuto() {
        // $TODO
        return true;
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
        // $TODO
        return new Rotation2d();
    }

    @Override
    public double getFrontLeftTurnEncoder() {
        throw new UnsupportedOperationException("Not yet implemented");
    }

    @Override
    public double getBackLeftTurnEncoder() {
        throw new UnsupportedOperationException("Not yet implemented");
    }

    @Override
    public double getFrontRightTurnEncoder() {
        throw new UnsupportedOperationException("Not yet implemented");
    }

    @Override
    public double getBackRightTurnEncoder() {
        throw new UnsupportedOperationException("Not yet implemented");
    }

    @Override
    public double getFrontLeftDriveEncoder() {
        // $TODO
        return 0;
    }

    @Override
    public double getBackLeftDriveEncoder() {
        throw new UnsupportedOperationException("Not yet implemented");
    }

    @Override
    public double getFrontRightDriveEncoder() {
        throw new UnsupportedOperationException("Not yet implemented");
    }

    @Override
    public double getBackRightDriveEncoder() {
        throw new UnsupportedOperationException("Not yet implemented");
    }

    @Override
    public double getFrontLeftDriveVelocity() {
        // $TODO
        return 0;
    }

    @Override
    public double getBackLeftDriveVelocity() {
        // $TODO
        return 0;
    }

    @Override
    public double getFrontRightDriveVelocity() {
        // $TODO
        return 0;
    }

    @Override
    public double getBackRightDriveVelocity() {
        // $TODO
        return 0;
    }
}
