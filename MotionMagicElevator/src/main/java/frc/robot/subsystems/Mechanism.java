package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MechanismConstants;

public class Mechanism extends SubsystemBase {

    private enum ControlMode {
        kOpenLoop, kMotionMagic
    }

    //
    // State
    //
    private ControlMode m_controlMode = ControlMode.kOpenLoop;
    private double m_demand;
    private boolean m_homed;

    private final MotionMagicExpoTorqueCurrentFOC m_motionMagic = new MotionMagicExpoTorqueCurrentFOC(0);

    //
    // Hardware
    //
    private final TalonFX m_motor;
    private final TalonFX m_motorFollower;
    private final DigitalInput m_retractLimit;

    public Mechanism() {
        //
        // Base Motor configuration: Brake Mode, Current limits, etc...
        //
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 40;
        config.CurrentLimits.StatorCurrentLimitEnable = false;
        config.CurrentLimits.StatorCurrentLimit = 125;

        //
        // Setup Follower motor
        //
        m_motorFollower = new TalonFX(MechanismConstants.kMotorFollowerID);
        m_motorFollower.getConfigurator().apply(config);
        m_motorFollower.optimizeBusUtilization();
        m_motorFollower.setControl(new Follower(MechanismConstants.kMotorID, true));

        //
        // Apply add extra leader configuration on top of the base config
        // - Any control related settings for PID, Motion Magic, Remote Sensors, etc.. 
        // - Soft limits should only be applied to the leader motor, if they are applied to the follower
        //   it may stop moving if their built in encoders are not in sync.
        //
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 80;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 5;

        // PID Slot 0
        config.Slot0.GravityType = GravityTypeValue.Elevator_Static;
        config.Slot0.kS = 0; // TODO: Add 0.25 V output to overcome static friction
        config.Slot0.kV = 0; // TODO: A velocity target of 1 rps results in 0.12 V output
        config.Slot0.kA = 0; // TODO: An acceleration of 1 rps/s requires 0.01 V output
        config.Slot0.kP = 0; // TODO: A position error of 2.5 rotations results in 12 V output
        config.Slot0.kI = 0; // TODO: no output for integrated error
        config.Slot0.kD = 0; // TODO: A velocity error of 1 rps results in 0.1 V output

        // Motion Magic
        config.MotionMagic.MotionMagicCruiseVelocity = 0; // Unlimited cruise velocity
        config.MotionMagic.MotionMagicExpo_kV = 0.0; // kV is around 0.12 V/rps
        config.MotionMagic.MotionMagicExpo_kA = 0.0; // Use a slower kA of 0.1 V/(rps/s)

        m_motor = new TalonFX(MechanismConstants.kMotorID);
        m_motor.getConfigurator().apply(config);

        // We want to read position data from the leader motor
        m_motor.getPosition().setUpdateFrequency(50);

        // These 3 are needed for the follower motor to work
        m_motor.getDutyCycle().setUpdateFrequency(50);
        m_motor.getMotorVoltage().setUpdateFrequency(50);
        m_motor.getTorqueCurrent().setUpdateFrequency(50);
        m_motor.optimizeBusUtilization();    

        //
        // Limit Switch
        //
        m_retractLimit = new DigitalInput(MechanismConstants.kRetractLimitSwitchChannel);

        //
        // Mechanism state
        //
        m_homed = false;
    }

    //
    //
    //

    public boolean isAtRetractLimit() {
        return m_retractLimit.get();
    }

    public double getHeight() {
        return m_motor.getPosition().getValueAsDouble() * MechanismConstants.kRotationToInches;
    }

    public void stop() {
        setOutputVoltage(0);
    }

    public void setOutputVoltage(double OutputVoltage) {
        m_controlMode = ControlMode.kOpenLoop;
        m_demand = OutputVoltage;
    }

    public void setMotionMagic(double heightInches) {
        m_controlMode = ControlMode.kMotionMagic;
        m_demand = heightInches;
    }

    //
    // Commands
    //
    public Command openLoopCommand(DoubleSupplier OutputVoltageSupplier) {
        return Commands.runEnd(
            () -> this.setOutputVoltage(OutputVoltageSupplier.getAsDouble()), this::stop, this);
    }

    public Command openLoopCommand(double OutputVoltage) {
        return openLoopCommand(() -> OutputVoltage);
    }

    public Command motionMagicCommand(DoubleSupplier heightInches) {
        return Commands.runEnd(
            () -> this.setMotionMagic(heightInches.getAsDouble()), this::stop, this);
    }

    public Command motionMagicCommand(double heightInches) {
        return motionMagicCommand(() -> heightInches);
    }

    @Override
    public void periodic() {
        SmartDashboard.putString("Mechanism control mode", m_controlMode.toString());
        SmartDashboard.putBoolean("Mechanism is at retrat limit", isAtRetractLimit());
        SmartDashboard.putNumber("Mechanism height", getHeight());

        // Elevator is not homed but is at the retract limit, so home the elevator!
        if (!m_homed && isAtRetractLimit()) {
            m_motor.setPosition(0);
            m_homed = true;
        }

        if (!m_homed) {
            // Elevator is not homed
            
            // TODO make sure other mechanisms on elevator (Arm & Wrist) don't move if the elveator isn't homed (or are able to move in a safe direction like toward starting config) 
            if (m_controlMode == ControlMode.kOpenLoop && m_demand < 0) {
                // We can only allow it to move down in open loop (driver pressing down button).
                m_motor.setVoltage(m_demand);
            } else {
                // Otherwise don't let the elevator move down
                m_motor.stopMotor();
            }
        }

        switch (m_controlMode) {

            case kOpenLoop:
                // Do openloop stuff here
                m_motor.setVoltage(m_demand);    
                break;
            case kMotionMagic:
                m_motionMagic.Position = m_demand/MechanismConstants.kRotationToInches;
                m_motor.setControl(m_motionMagic);
                break;
            default:
                m_motor.stopMotor();
        }      
    }
}
