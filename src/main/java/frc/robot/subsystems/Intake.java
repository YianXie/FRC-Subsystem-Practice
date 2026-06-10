package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SubsystemConstants.IntakeConstants;

public class Intake extends SubsystemBase {
    /* Motors and sensors */
    private final TalonFX m_motor = new TalonFX(IntakeConstants.kIntakeMotorCanId);
    private final DigitalInput m_beamBreak = new DigitalInput(IntakeConstants.kIntakeBeamBreakDio);

    /* Simulation only */
    private final DCMotorSim m_motorSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60Foc(1), 0.001, 10.0), DCMotor.getKrakenX60Foc(1));

    /* Constructor */
    public Intake() {
        var cfg = new TalonFXConfiguration();
        cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        m_motor.getConfigurator().apply(cfg);
    }

    /* Actions */
    public void runAt(double speed) {
        m_motor.set(speed);
    }

    public void stop() {
        m_motor.set(0);
    }

    public boolean hasPiece() {
        return !m_beamBreak.get();
    }

    /** The Intake Command */
    public Command runIntakeCommand() {
        return runEnd(() -> runAt(0.7), this::stop);
    }

    /** Run intake until beam stops */
    public Command intakeUntilDetected() {
        return run(() -> runAt(0.7))
                .until(this::hasPiece)
                .finallyDo(this::stop)
                .withName("IntakeUntilDetected");
    }

    /** Eject for a fixed time, then stop. */
    public Command ejectCommand() {
        return runEnd(() -> runAt(-0.5), this::stop)
                .withTimeout(0.75)
                .withName("Eject");
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Intake/HasPiece", hasPiece());
        SmartDashboard.putNumber("Intake/MotorOutput", m_motor.get());
    }

    @Override
    public void simulationPeriodic() {
        var simState = m_motor.getSimState();
        simState.setSupplyVoltage(RobotController.getBatteryVoltage());

        m_motorSim.setInputVoltage(simState.getMotorVoltageMeasure().in(Volts));
        m_motorSim.update(0.020);

        simState.setRawRotorPosition(m_motorSim.getAngularPositionRotations());
        simState.setRotorVelocity(
                Units.radiansToRotations(m_motorSim.getAngularVelocityRadPerSec()));
    }
}
