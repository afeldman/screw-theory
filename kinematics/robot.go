package kinematics

import (
	"fmt"
	"math"
)

// Robot: Forward and inverse kinematics using screw theory
type Robot struct {
	Name        string
	Twists      []Twist
	M           DualQuaternion // Home configuration
	NumJoints   int
	LinkLengths []float64
}

// ExponentialMap: Convert screw axis and angle to SE(3) transformation
func (r *Robot) ExponentialMap(twist Twist, theta float64) DualQuaternion {
	halfAngle := theta / 2.0
	sinHalf := math.Sin(halfAngle)
	cosHalf := math.Cos(halfAngle)

	omegaNorm := math.Sqrt(twist.Omega[0]*twist.Omega[0] +
		twist.Omega[1]*twist.Omega[1] +
		twist.Omega[2]*twist.Omega[2])

	var qRotation Quaternion
	if omegaNorm < 1e-6 {
		qRotation = Quaternion{W: cosHalf}
	} else {
		unitOmega := [3]float64{
			twist.Omega[0] / omegaNorm,
			twist.Omega[1] / omegaNorm,
			twist.Omega[2] / omegaNorm,
		}
		qRotation = Quaternion{
			W: cosHalf,
			X: unitOmega[0] * sinHalf,
			Y: unitOmega[1] * sinHalf,
			Z: unitOmega[2] * sinHalf,
		}
	}

	result := DualQuaternion{Real: qRotation}

	if omegaNorm > 1e-6 {
		cross := [3]float64{
			twist.Omega[1]*twist.V[2] - twist.Omega[2]*twist.V[1],
			twist.Omega[2]*twist.V[0] - twist.Omega[0]*twist.V[2],
			twist.Omega[0]*twist.V[1] - twist.Omega[1]*twist.V[0],
		}
		c := 1 - cosHalf*2
		factor := c / theta
		result.Dual = Quaternion{
			W: 0,
			X: (twist.V[0] * sinHalf * 2) + (cross[0] * factor),
			Y: (twist.V[1] * sinHalf * 2) + (cross[1] * factor),
			Z: (twist.V[2] * sinHalf * 2) + (cross[2] * factor),
		}
	}

	return result
}

// ForwardKinematics: Compute end-effector pose from joint angles
func (r *Robot) ForwardKinematics(jointAnglesDeg []interface{}) *XYZWPR {
	if len(jointAnglesDeg) != r.NumJoints {
		return nil
	}

	angles := make([]float64, r.NumJoints)
	for i, val := range jointAnglesDeg {
		switch v := val.(type) {
		case float64:
			angles[i] = DegToRad(v)
		case int:
			angles[i] = DegToRad(float64(v))
		}
	}

	T := IdentityDualQuaternion()
	for i := 0; i < r.NumJoints; i++ {
		expMap := r.ExponentialMap(r.Twists[i], angles[i])
		T = T.Multiply(expMap)
	}
	T = T.Multiply(r.M)

	pos := T.GetTranslation()
	quat := T.GetRotation()
	euler := quat.ToEuler()

	return &XYZWPR{
		X: pos[0],
		Y: pos[1],
		Z: pos[2],
		W: RadToDeg(euler[2]),
		P: RadToDeg(euler[1]),
		R: RadToDeg(euler[0]),
	}
}

// InverseKinematics: Solve joint angles using Jacobian gradient descent
func (r *Robot) InverseKinematics(target *XYZWPR, seedDeg [6]float64) ([6]float64, error) {
	const (
		maxIterations = 100
		tolerance     = 0.5
		alpha         = 0.03
	)

	angles := make([]float64, 6)
	for i := range seedDeg {
		angles[i] = DegToRad(seedDeg[i])
	}

	for iter := 0; iter < maxIterations; iter++ {
		angleIfaces := make([]interface{}, len(angles))
		for i := range angles {
			angleIfaces[i] = angles[i]
		}
		pose := r.ForwardKinematics(angleIfaces)
		if pose == nil {
			continue
		}

		dx := target.X - pose.X
		dy := target.Y - pose.Y
		dz := target.Z - pose.Z
		error := math.Sqrt(dx*dx + dy*dy + dz*dz)

		if error < tolerance {
			result := [6]float64{}
			for i := range angles {
				result[i] = RadToDeg(angles[i])
			}
			return result, nil
		}

		const delta = 0.001
		for j := 0; j < 6; j++ {
			angles[j] += delta
			angleIfaces := make([]interface{}, len(angles))
			for i := range angles {
				angleIfaces[i] = angles[i]
			}
			posPerturbed := r.ForwardKinematics(angleIfaces)
			angles[j] -= delta

			if posPerturbed != nil {
				dx1 := target.X - posPerturbed.X
				dy1 := target.Y - posPerturbed.Y
				dz1 := target.Z - posPerturbed.Z
				errorPerturbed := math.Sqrt(dx1*dx1 + dy1*dy1 + dz1*dz1)
				grad := (errorPerturbed - error) / delta
				angles[j] -= alpha * grad
			}
		}
	}

	result := [6]float64{}
	for i := range angles {
		result[i] = RadToDeg(angles[i])
	}
	return result, fmt.Errorf("IK did not converge")
}

// CheckJointLimits: Validate joint angles
func (r *Robot) CheckJointLimits(angles []float64, limits JointLimits) bool {
	if len(angles) != 6 {
		return false
	}
	return angles[0] >= DegToRad(limits.J1Min) && angles[0] <= DegToRad(limits.J1Max) &&
		angles[1] >= DegToRad(limits.J2Min) && angles[1] <= DegToRad(limits.J2Max) &&
		angles[2] >= DegToRad(limits.J3Min) && angles[2] <= DegToRad(limits.J3Max) &&
		angles[3] >= DegToRad(limits.J4Min) && angles[3] <= DegToRad(limits.J4Max) &&
		angles[4] >= DegToRad(limits.J5Min) && angles[4] <= DegToRad(limits.J5Max) &&
		angles[5] >= DegToRad(limits.J6Min) && angles[5] <= DegToRad(limits.J6Max)
}

// GetDefaultJointLimits: FANUC R-30iB joint limits
func (r *Robot) GetDefaultJointLimits() JointLimits {
	return JointLimits{
		J1Min: -180, J1Max: 180,
		J2Min: -125, J2Max: 125,
		J3Min: -180, J3Max: 65,
		J4Min: -180, J4Max: 180,
		J5Min: -120, J5Max: 120,
		J6Min: -360, J6Max: 360,
	}
}

// Getter: Return robot name
func (r *Robot) GetName() string {
	return r.Name
}
