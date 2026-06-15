package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Voltage;
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

    /* Control requests */
    private final DutyCycleOut m_dutyCycleOutRequest = new DutyCycleOut(0.0);
    private final VoltageOut m_voltageOutRequest = new VoltageOut(0.0);
    private final NeutralOut m_neutralOutRequest = new NeutralOut();

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
    public void setDutyCycleOut(double output) {
        if (Math.abs(output) > 1.0)
            return;
        m_motor.setControl(m_dutyCycleOutRequest.withOutput(output));
    }

    public void setVoltageOut(Voltage voltage) {
        m_motor.setControl(m_voltageOutRequest.withOutput(voltage.in(Volts)));
    }

    public void stop() {
        m_motor.setControl(m_neutralOutRequest);
    }

    public boolean hasPiece() {
        return !m_beamBreak.get();
    }

    /** The Intake Command */
    public Command runIntakeCommand() {
        return runEnd(() -> setVoltageOut(Volts.of(8.0)), this::stop).withName("IntakeCommand");
    }

    /** Run intake until beam stops */
    public Command intakeUntilDetected() {
        return run(() -> setVoltageOut(Volts.of(8.0)))
                .until(this::hasPiece)
                .finallyDo(this::stop)
                .withName("IntakeUntilDetected");
    }

    /** Eject for a fixed time, then stop. */
    public Command ejectCommand() {
        return runEnd(() -> setVoltageOut(Volts.of(-6.0)), this::stop)
                .withTimeout(0.75)
                .withName("Eject");
    }

    @Override
    public void periodic() {
        // State
        SmartDashboard.putBoolean("Intake/HasPiece", hasPiece());
        SmartDashboard.putBoolean("Intake/BeamBreakRaw", m_beamBreak.get());

        // Motor electrical
        SmartDashboard.putNumber("Intake/MotorOutput", m_motor.get());
        SmartDashboard.putNumber("Intake/SupplyVoltage", m_motor.getSupplyVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Intake/MotorVoltage", m_motor.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Intake/SupplyCurrent", m_motor.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Intake/StatorCurrent", m_motor.getStatorCurrent().getValueAsDouble());

        // Motor mechanical
        SmartDashboard.putNumber("Intake/VelocityRPS", m_motor.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Intake/PositionRot", m_motor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Intake/TempCelsius", m_motor.getDeviceTemp().getValueAsDouble());

        // Command
        SmartDashboard.putString("Intake/ActiveCommand",
                getCurrentCommand() != null ? getCurrentCommand().getName() : "None");
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
