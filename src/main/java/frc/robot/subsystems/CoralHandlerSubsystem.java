package frc.robot.subsystems;

import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.Rev2mDistanceSensor.Unit;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import org.dyn4j.geometry.Polygon;
import org.dyn4j.geometry.Rectangle;
import org.dyn4j.geometry.Vector2;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;

public class CoralHandlerSubsystem extends SubsystemBase {
    // two neo 550s, 9:1 gearing on both, going in opposite directions.
    // looking on from the top, the right one going clockwise is intake, and
    // counterclockwise is
    // outtake.
    // vice versa for the other
    // TODO: move all of the hardcoded values to Constants

    public final SparkMax m_left;
    public final SparkMax m_right;
    private final Rev2mDistanceSensor m_sensor;

    public final IntakeSimulation m_intakeSim;

    public enum Mode {
        kInactive,
        kIntake,
        kOuttake,
    }

    private Mode m_mode = Mode.kInactive;
    private boolean m_hasCoral = false;

    public CoralHandlerSubsystem(AbstractDriveTrainSimulation driveSim) {
        m_left = new SparkMax(9, MotorType.kBrushless);
        m_right = new SparkMax(10, MotorType.kBrushless);
        if (Robot.getInstance().isReal()) {
            m_sensor = new Rev2mDistanceSensor(Rev2mDistanceSensor.Port.kOnboard);
        } else {
            m_sensor = null;
        }
        m_intakeSim = new IntakeSimulation(
                "Coral",
                driveSim,
                new Rectangle(.762, 1.007),
                1);
    }

    public Mode getMode() {
        return m_mode;
    }

    public boolean hasCoral() {
        return m_hasCoral;
    }

    public void setMode(Mode mode) {
        m_mode = mode;
        switch (mode) {
            case kInactive:
                m_left.set(0);
                m_right.set(0);
                break;
            case kIntake:
                m_left.set(1);
                m_right.set(-1);
                break;
            case kOuttake:
                m_left.set(-1);
                m_right.set(1);
                break;
        }
    }

    @Override
    public void periodic() {
        if (Robot.getInstance().isReal()) {
            m_hasCoral = m_sensor.getRange(Unit.kInches) < 5;
        } else {
            if (m_mode == Mode.kIntake) {
                m_intakeSim.startIntake();
            } else {
                m_intakeSim.stopIntake();
            }
            m_hasCoral = m_intakeSim.getGamePiecesAmount() != 0;
        }
    }

    public void intake() {
        setMode(Mode.kIntake);
    }

    public void outtake() {
        setMode(Mode.kOuttake);
    }
}
