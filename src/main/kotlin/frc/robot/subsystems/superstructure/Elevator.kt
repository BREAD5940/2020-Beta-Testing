package frc.robot.subsystems.superstructure

import com.ctre.phoenix.motorcontrol.FeedbackDevice
import com.ctre.phoenix.motorcontrol.InvertType
import frc.robot.Ports
import org.ghrobotics.lib.commands.FalconSubsystem
import org.ghrobotics.lib.mathematics.units.Meter
import org.ghrobotics.lib.mathematics.units.derived.volt
import org.ghrobotics.lib.mathematics.units.inches
import org.ghrobotics.lib.mathematics.units.nativeunit.DefaultNativeUnitModel
import org.ghrobotics.lib.mathematics.units.nativeunit.NativeUnitLengthModel
import org.ghrobotics.lib.mathematics.units.nativeunit.nativeUnits
import org.ghrobotics.lib.motors.FalconMotor
import org.ghrobotics.lib.motors.ctre.FalconSRX
import org.ghrobotics.lib.motors.ctre.falconSRX
import org.ghrobotics.lib.wrappers.FalconDoubleSolenoid
import org.ghrobotics.lib.wrappers.FalconSolenoid

object Elevator : FalconSubsystem() {

    private val shifter = FalconDoubleSolenoid(2, 3, 8)
    val lowGear = true

    val master: FalconSRX<Meter> = falconSRX(21, NativeUnitLengthModel(4096.nativeUnits, 0.75.inches)) {
        //low gear = slow high gear = fast
        //TODO PUd pid now u noob
        talonSRX.config_kP(0, 0.1)
        talonSRX.config_kD(0, 0.1)
        talonSRX.config_kP(1, 0.1)
        talonSRX.config_kD(1, 0.1)
        //TODO TUNE IT NOW, u lazy
        if (lowGear) {
            talonSRX.selectProfileSlot(0, 0)
        } else {
            talonSRX.selectProfileSlot(1, 0)
        }

    }

    val Follower1 = FalconSRX(22, DefaultNativeUnitModel)
    val Follower2 = FalconSRX(23, DefaultNativeUnitModel)
    val Follower3 = FalconSRX(24, DefaultNativeUnitModel)

    override fun lateInit() {


        //when being small brain then look here
        Follower1.follow(master)
        Follower1.talonSRX.setInverted(InvertType.OpposeMaster)
        Follower2.follow(master)
        Follower3
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
