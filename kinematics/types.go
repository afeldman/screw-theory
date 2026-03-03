package kinematics

import (
	"math"
)

// XYZWPR: Cartesian pose (position + rotation in Euler angles)
type XYZWPR struct {
	X float64 `json:"x"` // mm
	Y float64 `json:"y"` // mm
	Z float64 `json:"z"` // mm
	W float64 `json:"w"` // degrees (y rotation around Z)
	P float64 `json:"p"` // degrees (pitch rotation around Y)
	R float64 `json:"r"` // degrees (roll rotation around X)
}

// Vector3: 3D vector
type Vector3 [3]float64

// Quaternion: Unit quaternion for 3D rotations (q = w + xi + yj + zk)
type Quaternion struct {
	W, X, Y, Z float64
}

// DualQuaternion: SE(3) transformation (rotation + translation)
// Represented as q_real + ε*q_dual
type DualQuaternion struct {
	Real Quaternion // Rotation part
	Dual Quaternion // Translation part
}

// Twist: Screw-theoretic joint representation
// ω = angular velocity (revolute axis), v = linear velocity
type Twist struct {
	Omega [3]float64 // Angular velocity axis
	V     [3]float64 // Linear velocity
}

// JointLimits: Per-joint constraints
type JointLimits struct {
	J1Min, J1Max float64
	J2Min, J2Max float64
	J3Min, J3Max float64
	J4Min, J4Max float64
	J5Min, J5Max float64
	J6Min, J6Max float64
}

// MotionProfile: Trajectory velocity/acceleration constraints
type MotionProfile struct {
	MaxVelocity       float64 // mm/s
	MaxAccel          float64 // mm/s²
	TimeStep          float64 // seconds
	InterpolationType string  // "linear", "spline", "joint"
}

// IdentityQuaternion: Return identity (no rotation)
func IdentityQuaternion() Quaternion {
	return Quaternion{1, 0, 0, 0}
}

// Normalize: Normalize quaternion to unit length
func (q *Quaternion) Normalize() {
	norm := math.Sqrt(q.W*q.W + q.X*q.X + q.Y*q.Y + q.Z*q.Z)
	if norm > 0 {
		q.W /= norm
		q.X /= norm
		q.Y /= norm
		q.Z /= norm
	}
}

// Multiply: Quaternion multiplication (rotation composition)
func (q1 Quaternion) Multiply(q2 Quaternion) Quaternion {
	return Quaternion{
		W: q1.W*q2.W - q1.X*q2.X - q1.Y*q2.Y - q1.Z*q2.Z,
		X: q1.W*q2.X + q1.X*q2.W + q1.Y*q2.Z - q1.Z*q2.Y,
		Y: q1.W*q2.Y - q1.X*q2.Z + q1.Y*q2.W + q1.Z*q2.X,
		Z: q1.W*q2.Z + q1.X*q2.Y - q1.Y*q2.X + q1.Z*q2.W,
	}
}

// Conjugate: Return quaternion conjugate (q* = w - xi - yj - zk)
func (q Quaternion) Conjugate() Quaternion {
	return Quaternion{q.W, -q.X, -q.Y, -q.Z}
}

// RotateVector: Rotate 3D vector using quaternion (q*v*q^-1)
func (q Quaternion) RotateVector(v Vector3) Vector3 {
	vq := Quaternion{0, v[0], v[1], v[2]}
	result := q.Multiply(vq).Multiply(q.Conjugate())
	return Vector3{result.X, result.Y, result.Z}
}

// ToEuler: Convert quaternion to Euler angles (roll, pitch, yaw radians)
func (q Quaternion) ToEuler() Vector3 {
	var euler Vector3
	euler[0] = math.Atan2(2*(q.W*q.X+q.Y*q.Z), 1-2*(q.X*q.X+q.Y*q.Y))
	sinp := 2 * (q.W*q.Y - q.Z*q.X)
	if sinp >= 1 {
		euler[1] = math.Pi / 2
	} else if sinp <= -1 {
		euler[1] = -math.Pi / 2
	} else {
		euler[1] = math.Asin(sinp)
	}
	euler[2] = math.Atan2(2*(q.W*q.Z+q.X*q.Y), 1-2*(q.Y*q.Y+q.Z*q.Z))
	return euler
}

// FromEuler: Create quaternion from Euler angles (radians)
func FromEuler(roll, pitch, yaw float64) Quaternion {
	cr := math.Cos(roll * 0.5)
	sr := math.Sin(roll * 0.5)
	cp := math.Cos(pitch * 0.5)
	sp := math.Sin(pitch * 0.5)
	cy := math.Cos(yaw * 0.5)
	sy := math.Sin(yaw * 0.5)
	return Quaternion{
		W: cr*cp*cy + sr*sp*sy,
		X: sr*cp*cy - cr*sp*sy,
		Y: cr*sp*cy + sr*cp*sy,
		Z: cr*cp*sy - sr*sp*cy,
	}
}

// IdentityDualQuaternion: Return identity (no transformation)
func IdentityDualQuaternion() DualQuaternion {
	return DualQuaternion{
		Real: IdentityQuaternion(),
		Dual: Quaternion{0, 0, 0, 0},
	}
}

// Multiply: Dual quaternion multiplication
func (dq1 DualQuaternion) Multiply(dq2 DualQuaternion) DualQuaternion {
	return DualQuaternion{
		Real: dq1.Real.Multiply(dq2.Real),
		Dual: dq1.Real.Multiply(dq2.Dual).Add(dq1.Dual.Multiply(dq2.Real)),
	}
}

// Add: Add two quaternions
func (q1 Quaternion) Add(q2 Quaternion) Quaternion {
	return Quaternion{q1.W + q2.W, q1.X + q2.X, q1.Y + q2.Y, q1.Z + q2.Z}
}

// GetTranslation: Extract position from dual quaternion
func (dq DualQuaternion) GetTranslation() Vector3 {
	q_conj := dq.Real.Conjugate()
	result := dq.Dual.Multiply(q_conj)
	return Vector3{2 * result.X, 2 * result.Y, 2 * result.Z}
}

// GetRotation: Extract rotation quaternion
func (dq DualQuaternion) GetRotation() Quaternion {
	return dq.Real
}

// DegToRad: Convert degrees to radians
func DegToRad(deg float64) float64 {
	return deg * math.Pi / 180.0
}

// RadToDeg: Convert radians to degrees
func RadToDeg(rad float64) float64 {
	return rad * 180.0 / math.Pi
}

// Distance: Euclidean distance between two vectors
func Distance(v1, v2 Vector3) float64 {
	dx := v1[0] - v2[0]
	dy := v1[1] - v2[1]
	dz := v1[2] - v2[2]
	return math.Sqrt(dx*dx + dy*dy + dz*dz)
}

// Clamp: Clamp value between min and max
func Clamp(val, min, max float64) float64 {
	if val < min {
		return min
	}
	if val > max {
		return max
	}
	return val
}
