package frc.robot.subsystems.superstructure

import com.ctre.phoenix.motorcontrol.FeedbackDevice
import frc.robot.Ports
import org.ghrobotics.lib.commands.FalconSubsystem
import org.ghrobotics.lib.mathematics.units.Meter
import org.ghrobotics.lib.mathematics.units.inches
import org.ghrobotics.lib.mathematics.units.nativeunit.NativeUnitLengthModel
import org.ghrobotics.lib.mathematics.units.nativeunit.nativeUnits
import org.ghrobotics.lib.motors.FalconMotor
import org.ghrobotics.lib.motors.ctre.FalconSRX
import org.ghrobotics.lib.motors.ctre.falconSRX
import org.ghrobotics.lib.wrappers.FalconDoubleSolenoid
import org.ghrobotics.lib.wrappers.FalconSolenoid

object Elevator : FalconSubsystem() {

    private val shifter = FalconDoubleSolenoid(2, 3, 8)
    val low = true
    val master: FalconSRX<Meter> = falconSRX(5, NativeUnitLengthModel(4096.nativeUnits, 0.75.inches)) {
        //low gear = slow high gear = fast
        //TODO PUd pid now u noob
        talonSRX.config_kP(0, p)
        talonSRX.config_kD(0, d)
        talonSRX.config_kP(1, p)
        talonSRX.config_kD(1, d)

        talonSRX.selectProfileSlot(0, 0)
    }

    override fun lateInit() {

        master.talonSRX.configFactoryDefault()
        master.feedbackSensor = FeedbackDevice.CTRE_MagEncoder_Relative
        with(master.talonSRX) {

        }
    }

    fun setLowGear() {
        shifter.state = FalconSolenoid.State.Forward
        master.talonSRX.selectProfileSlot(0, 0)
    }

    fun setHighGear() {
        shifter.state = FalconSolenoid.State.Reverse
        master.talonSRX.selectProfileSlot(1, 0)
    }


}
