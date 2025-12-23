@file:Suppress("NOTHING_TO_INLINE", "unused")

package com.frcteam3636.swervebase.utils.math

import edu.wpi.first.units.Units.*
import edu.wpi.first.units.VoltageUnit
import edu.wpi.first.units.measure.*
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.Ultrasonic

fun Distance.toAngular(radius: Distance): Angle = Radians.of(this.inMeters() / radius.inMeters())
fun Angle.toLinear(radius: Distance): Distance = Meters.of(this.inRadians() * radius.inMeters())

inline val Ultrasonic.range: Distance get() = rangeMM.millimeters

/** Returns true if the period has elapsed. If it has, advances the clock by the period */
inline fun Timer.advanceIfElapsed(time: Time) = advanceIfElapsed(time.inSeconds())
inline fun Timer.hasElapsed(time: Time) = hasElapsed(time.inSeconds())

// Number -> Measure

inline val Number.meters: Distance get() = Meters.of(this.toDouble())
inline val Number.centimeters: Distance get() = Centimeters.of(this.toDouble())
inline val Number.millimeters: Distance get() = Millimeters.of(this.toDouble())
inline val Number.inches: Distance get() = Inches.of(this.toDouble())
inline val Number.feet: Distance get() = Feet.of(this.toDouble())

inline val Number.seconds: Time get() = Seconds.of(this.toDouble())
inline val Number.milliseconds: Time get() = Milliseconds.of(this.toDouble())
inline val Number.microseconds: Time get() = Microseconds.of(this.toDouble())
inline val Number.minutes: Time get() = Minutes.of(this.toDouble())

inline val Number.radians: Angle get() = Radians.of(this.toDouble())
inline val Number.rotations: Angle get() = Rotations.of(this.toDouble())
inline val Number.degrees: Angle get() = Degrees.of(this.toDouble())

inline val Number.metersPerSecond: LinearVelocity get() = MetersPerSecond.of(this.toDouble())
inline val Number.feetPerSecond: LinearVelocity get() = FeetPerSecond.of(this.toDouble())
inline val Number.inchesPerSecond: LinearVelocity get() = InchesPerSecond.of(this.toDouble())

inline val Number.rotationsPerSecond: AngularVelocity get() = RotationsPerSecond.of(this.toDouble())
inline val Number.rotationsPerSecondPerSecond: AngularAcceleration get() = RotationsPerSecondPerSecond.of(this.toDouble())
inline val Number.rpm: AngularVelocity get() = RPM.of(this.toDouble())
inline val Number.radiansPerSecond: AngularVelocity get() = RadiansPerSecond.of(this.toDouble())
inline val Number.degreesPerSecond: AngularVelocity get() = DegreesPerSecond.of(this.toDouble())

inline val Number.hertz: Frequency get() = Hertz.of(this.toDouble())
inline val Number.millihertz: Frequency get() = Millihertz.of(this.toDouble())

inline val Number.metersPerSecondPerSecond: LinearAcceleration get() = MetersPerSecondPerSecond.of(this.toDouble())
inline val Number.feetPerSecondPerSecond: LinearAcceleration get() = FeetPerSecondPerSecond.of(this.toDouble())
inline val Number.Gs: LinearAcceleration get() = edu.wpi.first.units.Units.Gs.of(this.toDouble())

inline val Number.kilograms: Mass get() = Kilograms.of(this.toDouble())
inline val Number.grams: Mass get() = Grams.of(this.toDouble())
inline val Number.pounds: Mass get() = Pounds.of(this.toDouble())
inline val Number.ounces: Mass get() = Ounces.of(this.toDouble())

inline val Number.newtons: Force get() = Newtons.of(this.toDouble())
inline val Number.ouncesForce: Force get() = OuncesForce.of(this.toDouble())
inline val Number.poundsForce: Force get() = PoundsForce.of(this.toDouble())

inline val Number.newtonMeters: Torque get() = NewtonMeters.of(this.toDouble())
inline val Number.poundFeet: Torque get() = PoundFeet.of(this.toDouble())
inline val Number.ounceInches: Torque get() = OunceInches.of(this.toDouble())

inline val Number.kilogramMetersPerSecond: LinearMomentum get() = KilogramMetersPerSecond.of(this.toDouble())
inline val Number.kilogramMetersSquaredPerSecond: AngularMomentum get() = KilogramMetersSquaredPerSecond.of(this.toDouble())
inline val Number.kilogramSquareMeters: MomentOfInertia get() = KilogramSquareMeters.of(this.toDouble())

inline val Number.volts: Voltage get() = Volts.of(this.toDouble())
inline val Number.millivolts: Voltage get() = Millivolts.of(this.toDouble())

inline val Number.amps: Current get() = Amps.of(this.toDouble())
inline val Number.milliamps: Current get() = Milliamps.of(this.toDouble())

inline val Number.ohms: Resistance get() = Ohms.of(this.toDouble())
inline val Number.kiloOhms: Resistance get() = KiloOhms.of(this.toDouble())
inline val Number.milliOhms: Resistance get() = MilliOhms.of(this.toDouble())

inline val Number.joules: Energy get() = Joules.of(this.toDouble())
inline val Number.millijoules: Energy get() = Millijoules.of(this.toDouble())
inline val Number.kilojoules: Energy get() = Kilojoules.of(this.toDouble())

inline val Number.watts: Power get() = Watts.of(this.toDouble())
inline val Number.milliwatts: Power get() = Milliwatts.of(this.toDouble())
inline val Number.horsepower: Power get() = Horsepower.of(this.toDouble())

inline val Number.kelvin: Temperature get() = Kelvin.of(this.toDouble())
inline val Number.celsius: Temperature get() = Celsius.of(this.toDouble())
inline val Number.fahrenheit: Temperature get() = Fahrenheit.of(this.toDouble())

inline val Number.voltsPerSecond: Velocity<VoltageUnit> get() = Volts.per(Second).of(this.toDouble())

// Measure -> Number

inline fun Distance.inMeters() = `in`(Meters)
inline fun Distance.inCentimeters() = `in`(Centimeters)
inline fun Distance.inMillimeters() = `in`(Millimeter)
inline fun Distance.inInches() = `in`(Inches)
inline fun Distance.inFeet() = `in`(Feet)

inline fun Time.inSeconds() = `in`(Seconds)
inline fun Time.inMilliseconds() = `in`(Milliseconds)
inline fun Time.inMicroseconds() = `in`(Microseconds)
inline fun Time.inMinutes() = `in`(Minutes)

inline fun Angle.inRadians() = `in`(Radians)
inline fun Angle.inRotations() = `in`(Rotations)
inline fun Angle.inDegrees() = `in`(Degrees)

inline fun LinearVelocity.inMetersPerSecond() = `in`(MetersPerSecond)
inline fun LinearVelocity.inFeetPerSecond() = `in`(FeetPerSecond)
inline fun LinearVelocity.inInchesPerSecond() = `in`(InchesPerSecond)
fun LinearVelocity.toAngular(radius: Distance) = RadiansPerSecond.of(this.inMetersPerSecond() / radius.inMeters())!!

inline fun AngularVelocity.inRotationsPerSecond() = `in`(RotationsPerSecond)
inline fun AngularVelocity.inRPM() = `in`(RPM)
inline fun AngularVelocity.inRadiansPerSecond() = `in`(RadiansPerSecond)
inline fun AngularVelocity.inDegreesPerSecond() = `in`(DegreesPerSecond)
fun AngularVelocity.toLinear(radius: Distance) = MetersPerSecond.of(this.inRadiansPerSecond() * radius.inMeters())!!

inline fun Frequency.inHertz() = `in`(Hertz)
inline fun Frequency.inMillihertz() = `in`(Millihertz)

inline fun LinearAcceleration.inMetersPerSecondPerSecond() = `in`(MetersPerSecondPerSecond)
inline fun LinearAcceleration.inFeetPerSecondPerSecond() = `in`(FeetPerSecondPerSecond)
inline fun LinearAcceleration.inGs() = `in`(Gs)

inline fun AngularAcceleration.inRotationsPerSecondPerSecond() = `in`(RotationsPerSecondPerSecond)

inline fun Mass.inKilograms() = `in`(Kilograms)
inline fun Mass.inGrams() = `in`(Grams)
inline fun Mass.inPounds() = `in`(Pounds)
inline fun Mass.inOunces() = `in`(Ounces)

inline fun Force.inNewtons() = `in`(Newtons)
inline fun Force.inOuncesForce() = `in`(OuncesForce)
inline fun Force.inPoundsForce() = `in`(PoundsForce)

inline fun Torque.inNewtonMeters() = `in`(NewtonMeters)
inline fun Torque.inPoundFeet() = `in`(PoundFeet)
inline fun Torque.inOunceInches() = `in`(OunceInches)

inline fun LinearMomentum.inKilogramMetersPerSecond() = `in`(KilogramMetersPerSecond)
inline fun AngularMomentum.inKilogramMetersSquaredPerSecond() = `in`(KilogramMetersSquaredPerSecond)
inline fun MomentOfInertia.inKilogramSquareMeters() = `in`(KilogramSquareMeters)

inline fun Voltage.inVolts() = `in`(Volts)
inline fun Voltage.inMillivolts() = `in`(Millivolts)

inline fun Current.inAmps() = `in`(Amps)
inline fun Current.inMilliamps() = `in`(Milliamps)

inline fun Resistance.inOhms() = `in`(Ohms)
inline fun Resistance.inKiloOhms() = `in`(KiloOhms)
inline fun Resistance.inMilliOhms() = `in`(MilliOhms)

inline fun Energy.inJoules() = `in`(Joules)
inline fun Energy.inMillijoules() = `in`(Millijoules)
inline fun Energy.inKilojoules() = `in`(Kilojoules)

inline fun Power.inWatts() = `in`(Watts)
inline fun Power.inMilliwatts() = `in`(Milliwatts)
inline fun Power.inHorsepower() = `in`(Horsepower)

inline fun Temperature.inKelvin() = `in`(Kelvin)
inline fun Temperature.inCelsius() = `in`(Celsius)
inline fun Temperature.inFahrenheit() = `in`(Fahrenheit)

inline fun Velocity<VoltageUnit>.inVoltsPerSecond() = `in`(Volts.per(Second))
