package frc.robot.subsystems.superstructure
import com.ctre.phoenix.CANifier
import org.apache.commons.math3.geometry.euclidean.threed.Rotation
import org.apache.commons.math3.geometry.spherical.oned.SubLimitAngle
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.commands.FalconSubsystem
import org.ghrobotics.lib.mathematics.units.Meter
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.derived.Radian
import org.ghrobotics.lib.mathematics.units.derived.degree
import org.ghrobotics.lib.mathematics.units.derived.degrees
import org.ghrobotics.lib.mathematics.units.derived.radians
import org.ghrobotics.lib.mathematics.units.inches
import org.ghrobotics.lib.mathematics.units.nativeunit.NativeUnitLengthModel
import org.ghrobotics.lib.mathematics.units.nativeunit.NativeUnitRotationModel
import org.ghrobotics.lib.mathematics.units.nativeunit.nativeUnits
import org.ghrobotics.lib.motors.ctre.FalconSRX
import org.ghrobotics.lib.motors.ctre.falconSRX
import org.ghrobotics.lib.mathematics.units.nativeunit.toNativeUnitPosition
import java.beans.Encoder


object Wrist : FalconSubsystem(){

val wristMotor: FalconSRX<Radian> = falconSRX(33, NativeUnitRotationModel(4096.nativeUnits * 8)) {
    //low gear = slow high gear = fast
    //TODO PUd pid now u noob
    //Copied from elevator, so if bad then this is why
    talonSRX.config_kP(0, 0.1)
    talonSRX.config_kD(0, 0.1)
}
//radian to degree is ~180(3.14)

class wristPreset(val WristRadian : SIUnit <Radian>){
   // val wantedWristRadian = 0
 fun initalize() {
    wristMotor.setPosition(WristRadian)
}
    fun isFinished(){

       if(wristMotor.encoder.position -WristRadian < 0.09.radians){
           return isFinished()
       }
       else{
           return
       }
    }




}


}
