package kinematics

import (
	"math"
)

// XYZWPR represents a Cartesian end-effector pose (position + Euler angle orientation).
//
// Position coordinates are in millimeters. Orientation angles are in degrees,
// following the standard convention: W (yaw around Z), P (pitch around Y), R (roll around X).
//
// Example:
//
//	pose := &XYZWPR{
//	    X: 500.0,   // 500mm in X direction
//	    Y: 100.0,   // 100mm in Y direction
//	    Z: 600.0,   // 600mm in Z direction
//	    W: 0.0,     // 0° yaw
//	    P: 0.0,     // 0° pitch
//	    R: 90.0,    // 90° roll
//	}
type XYZWPR struct {
	X float64 `json:"x"` // Position in mm
	Y float64 `json:"y"` // Position in mm
	Z float64 `json:"z"` // Position in mm
	W float64 `json:"w"` // Rotation in degrees (yaw around Z)
	P float64 `json:"p"` // Rotation in degrees (pitch around Y)
	R float64 `json:"r"` // Rotation in degrees (roll around X)
}

// Vector3 represents a 3D vector as an array of 3 floats.
//
// Used for positions, velocity vectors, and axis directions.
//
// Example:
//
//	v := Vector3{100.0, 200.0, 300.0}
//	x := v[0]  // 100.0
//	y := v[1]  // 200.0
//	z := v[2]  // 300.0
type Vector3 [3]float64

// Quaternion represents a unit quaternion for 3D rotations without singularities.
// Format: q = w + xi + yj + zk, where |q| = 1.
//
// Quaternions avoid gimbal lock and provide smooth interpolation for rotations.
// Always ensure quaternions are normalized to unit magnitude.
//
// Example:
//
//	// Identity rotation (no rotation)
//	q := Quaternion{W: 1, X: 0, Y: 0, Z: 0}
//
//	// 90-degree rotation around Z axis
//	q := Quaternion{W: 0.707, X: 0, Y: 0, Z: 0.707}
//
// Common operations:
//
//	q1.Multiply(q2)       // Compose two rotations
//	q.RotateVector(v)     // Apply rotation to vector
//	q.ToEuler()           // Convert to Euler angles
//	q.Conjugate()         // Get inverse rotation
type Quaternion struct {
	W, X, Y, Z float64
}

// DualQuaternion represents a rigid body transformation in SE(3).
// Combines rotation (Real part) and translation (Dual part) without singularities.
//
// The real part is a unit quaternion representing rotation.
// The dual part encodes translation using dual number algebra:
// Translation vector = 2 * Dual * Real.Conjugate()
//
// Example:
//
//	// No transformation (identity)
//	dq := DualQuaternion{
//	    Real: Quaternion{W: 1},
//	    Dual: Quaternion{W: 0},
//	}
//
//	// 500mm translation in X direction
//	dq := DualQuaternion{
//	    Real: Quaternion{W: 1},
//	    Dual: Quaternion{W: 0, X: 250.0},  // 500/2 in dual X coordinate
//	}
type DualQuaternion struct {
	Real Quaternion // Rotation component
	Dual Quaternion // Translation component
}

// Twist represents a screw axis - the instantaneous axis of motion for a joint.
// Screw theory uses twists instead of DH parameters for cleaner kinematics.
//
// Omega: Angular velocity vector (axis direction for revolute joints, zero for prismatic)
// V: Linear velocity vector (encoded offset for revolute joints, direction for prismatic)
//
// Example - Revolute joint rotating around Z-axis at origin:
//
//	twist := Twist{
//	    Omega: [3]float64{0, 0, 1},  // Rotation axis along Z
//	    V:     [3]float64{0, 0, 0},  // No linear offset
//	}
//
// Example - Revolute joint rotating around Y-axis, 300mm offset in Z:
//
//	twist := Twist{
//	    Omega: [3]float64{0, 1, 0},      // Rotation axis along Y
//	    V:     [3]float64{0, 0, 300},    // 300mm offset in Z
//	}
//
// Example - Prismatic joint sliding along X-axis:
//
//	twist := Twist{
//	    Omega: [3]float64{0, 0, 0},      // No rotation
//	    V:     [3]float64{1, 0, 0},      // Slide along X
//	}
type Twist struct {
	Omega [3]float64 // Angular velocity (rotation axis)
	V     [3]float64 // Linear velocity (translation or offset)
}

// JointLimits represents per-joint angle constraints in degrees.
//
// Use these limits to validate computed joint angles, ensuring they are
// physically achievable by the real robot.
//
// Example - Standard 6-DOF robot limits:
//
//	limits := JointLimits{
//	    J1Min: -180, J1Max: 180,    // Base can rotate 360°
//	    J2Min: -125, J2Max: 125,    // Shoulder has ±125° range
//	    J3Min: -180, J3Max:  65,    // Elbow has asymmetric range
//	    J4Min: -180, J4Max: 180,    // Wrist rotation
//	    J5Min: -120, J5Max: 120,    // Wrist bend
//	    J6Min: -360, J6Max: 360,    // Wrist pan (multi-turn)
//	}
type JointLimits struct {
	J1Min, J1Max float64 // Joint 1 angle limits (degrees)
	J2Min, J2Max float64 // Joint 2 angle limits (degrees)
	J3Min, J3Max float64 // Joint 3 angle limits (degrees)
	J4Min, J4Max float64 // Joint 4 angle limits (degrees)
	J5Min, J5Max float64 // Joint 5 angle limits (degrees)
	J6Min, J6Max float64 // Joint 6 angle limits (degrees)
}

// MotionProfile defines velocity and acceleration constraints for trajectory execution.
//
// Used for validating and planning robot motions to respect physical limitations.
// All times are in seconds, velocities in mm/s, acceleration in mm/s².
//
// Example:
//
//	profile := MotionProfile{
//	    MaxVelocity:       1000.0,  // TCP velocity up to 1000 mm/s
//	    MaxAccel:          5000.0,  // TCP acceleration up to 5000 mm/s²
//	    TimeStep:          0.01,    // Control update rate: 10ms
//	    InterpolationType: "spline", // Use spline interpolation
//	}
type MotionProfile struct {
	MaxVelocity       float64 // Maximum TCP velocity (mm/s)
	MaxAccel          float64 // Maximum TCP acceleration (mm/s²)
	TimeStep          float64 // Control time step (seconds)
	InterpolationType string  // "linear", "spline", "joint", etc.
}

// IdentityQuaternion: Return identity (no rotation)
// IdentityQuaternion returns a unit quaternion representing no rotation: q = (1, 0, 0, 0).
//
// This is the neutral element for quaternion multiplication. When composed with any other
// quaternion (or 3D vector), it leaves the rotation unchanged.
//
// Usage: Initializing quaternion chains or resetting rotations to identity.
//
// Returns:
//
//   - Quaternion: Unit quaternion {W: 1, X: 0, Y: 0, Z: 0}
//
// Mathematics:
//
// For quaternion multiplication q ⊗ q_id = q:
//
//	q ⊗ (1,0,0,0) = q
//
// Example:
//
//	q_id := IdentityQuaternion()
//	fmt.Printf("Identity: %+v\n", q_id)  // Identity: {W:1 X:0 Y:0 Z:0}
//	
//	// Composing rotations starting from identity
//	rotation := IdentityQuaternion()
//	rotation = rotation.Multiply(FromEuler(0, 0, 45*Pi/180))
//	rotation = rotation.Multiply(FromEuler(0, 90*Pi/180, 0))
//
// See also: Multiply, Conjugate, FromEuler
//
func IdentityQuaternion() Quaternion {
	return Quaternion{1, 0, 0, 0}
}

// Normalize ensures the quaternion has unit length (magnitude = 1).
//
// Quaternions used for rotations must be normalized (unit quaternions).
// Repeated multiplications can introduce floating-point drift, causing the
// magnitude to deviate from 1. This method rescales the quaternion to restore
// unit norm without changing the represented rotation.
//
// The normalization divides all components by the quaternion's magnitude:
//
//	magnitude = √(W² + X² + Y² + Z²)
//	q_norm = q / magnitude
//
// Notes:
//
//   - Modifies the quaternion in-place (receiver semantics).
//   - Safe to call even if the quaternion is already normalized.
//   - Does nothing if magnitude is zero (prevents division by zero).
//   - Should be called periodically during repeated multiplications to prevent drift.
//
// Performance: ~15 CPU cycles (sqrt + 4 divisions)
//
// Example:
//
//	q := FromEuler(0, 0, 0)
//	// After many multiplications, magnitude may drift
//	for i := 0; i < 1000; i++ {
//		q = q.Multiply(someRotation)
//	}
//	
//	// Restore unit norm
//	q.Normalize()
//	
//	// Now safe to use in critical calculations
//	v := q.RotateVector(vector)
//
// See also: Multiply, RotateVector
//
func (q *Quaternion) Normalize() {
	norm := math.Sqrt(q.W*q.W + q.X*q.X + q.Y*q.Y + q.Z*q.Z)
	if norm > 0 {
		q.W /= norm
		q.X /= norm
		q.Y /= norm
		q.Z /= norm
	}
}

// Multiply performs quaternion multiplication, composing two rotations into one.
//
// Given two rotations represented by unit quaternions q1 and q2, this method
// computes their composition: a single quaternion representing the combined rotation.
//
// The rotation order is: first apply q1, then apply q2. This is NOT commutative
// (q1 ⊗ q2 ≠ q2 ⊗ q1 in general), reflecting the non-commutative nature of 3D rotations.
//
// Mathematical formula:
//
//	q_result = q1 ⊗ q2
//
// Where the components are computed as:
//
//	W = w1*w2 - x1*x2 - y1*y2 - z1*z2
//	X = w1*x2 + x1*w2 + y1*z2 - z1*y2
//	Y = w1*y2 - x1*z2 + y1*w2 + z1*x2
//	Z = w1*z2 + x1*y2 - y1*x2 + z1*w2
//
// This is the Hamiltonian product formula used in computer graphics and robotics.
//
// Parameters:
//
//   - q2: The second quaternion to compose with (not modified)
//
// Returns:
//
//   - Quaternion: A new quaternion representing the combined rotation q1 ⊗ q2
//
// Performance: ~20 CPU cycles on modern processors
//
// Examples:
//
// Composing sequential rotations:
//
//	// Create individual rotations
//	q_roll := FromEuler(45*Pi/180, 0, 0)      // 45° around X-axis
//	q_pitch := FromEuler(0, 30*Pi/180, 0)     // 30° around Y-axis
//	
//	// Compose them: first roll, then pitch
//	q_combined := q_roll.Multiply(q_pitch)
//	
//	v := Vector3{1, 0, 0}
//	v_rotated := q_combined.RotateVector(v)
//
// Chaining multiple rotations:
//
//	q := IdentityQuaternion()
//	q = q.Multiply(FromEuler(10*Pi/180, 0, 0))
//	q = q.Multiply(FromEuler(0, 20*Pi/180, 0))
//	q = q.Multiply(FromEuler(0, 0, 30*Pi/180))
//
// Using in product-of-exponentials formula:
//
//	T := IdentityDualQuaternion()
//	for i := 0; i < 6; i++ {
//		exp_i := robot.ExponentialMap(twists[i], angles[i])
//		T = T.Multiply(exp_i)  // Compose transformations
//	}
//
// Notes:
//
//   - Order matters: q1.Multiply(q2) ≠ q2.Multiply(q1)
//   - For proper kinematics composition, use in PoE formula
//   - Repeated multiplications may accumulate floating-point error; call Normalize() periodically
//
// See also: Conjugate, RotateVector, Normalize
//
func (q1 Quaternion) Multiply(q2 Quaternion) Quaternion {
	return Quaternion{
		W: q1.W*q2.W - q1.X*q2.X - q1.Y*q2.Y - q1.Z*q2.Z,
		X: q1.W*q2.X + q1.X*q2.W + q1.Y*q2.Z - q1.Z*q2.Y,
		Y: q1.W*q2.Y - q1.X*q2.Z + q1.Y*q2.W + q1.Z*q2.X,
		Z: q1.W*q2.Z + q1.X*q2.Y - q1.Y*q2.X + q1.Z*q2.W,
	}
}

// Conjugate returns the complex conjugate of the quaternion: q* = w - xi - yj - zk.
//
// For a unit quaternion (used in rotations), the conjugate represents the inverse rotation.
// Multiplying a quaternion by its conjugate always yields the identity quaternion.
//
// Mathematical property (for unit quaternions):
//
//	q ⊗ q* = (1, 0, 0, 0) = identity
//	q* ≈ q^(-1)  (only true for unit quaternions)
//
// The conjugate operation in quaternion space is equivalent to:
//   - Negating the imaginary part (angle axis)
//   - Keeping the scalar part
//
// Returns:
//
//   - Quaternion: A new quaternion with same W but negated X, Y, Z
//
// Performance: O(1), merely negation operations
//
// Examples:
//
// Reversing a rotation:
//
//	q := FromEuler(45*Pi/180, 0, 0)  // 45° rotation around X
//	q_inv := q.Conjugate()             // Creates reverse rotation
//	
//	// Composing rotation and its inverse yields identity
//	q_identity := q.Multiply(q_inv)
//	// q_identity ≈ IdentityQuaternion()
//
// Rotating a vector and then rotating back:
//
//	q := FromEuler(45*Pi/180, 30*Pi/180, 0)
//	v := Vector3{1, 2, 3}
//	
//	// Apply rotation
//	v_rotated := q.RotateVector(v)
//	
//	// Reverse rotation
//	v_back := q.Conjugate().RotateVector(v_rotated)
//	// v_back ≈ v
//
// See also: Multiply, RotateVector
//
func (q Quaternion) Conjugate() Quaternion {
	return Quaternion{q.W, -q.X, -q.Y, -q.Z}
}

// RotateVector applies the quaternion rotation to a 3D vector.
//
// Uses the sandwich (conjugate) formula from group theory:
//
//	v' = q ⊗ v ⊗ q*
//
// Where v is treated as a quaternion with w=0 (pure imaginary): v_q = (0, vx, vy, vz).
//
// This formula ensures that:
//   - Magnitude is preserved: ||v'|| = ||v||
//   - Orthogonal vectors remain orthogonal after rotation
//   - The rotation axis and angle are determined by q
//
// Parameters:
//
//   - v: A 3D vector [x, y, z] to be rotated
//
// Returns:
//
//   - Vector3: A new vector representing the rotated position
//
// Performance: ~100 CPU cycles (3 quaternion multiplications + extraction)
//
// Examples:
//
// Rotating a single vector:
//
//	q := FromEuler(0, 45*Pi/180, 0)  // 45° pitch (around Y-axis)
//	v := Vector3{1, 0, 0}
//	
//	v_rotated := q.RotateVector(v)
//	// v_rotated ≈ [0.707, 0, 0.707]
//
// Rotating multiple vectors with the same rotation:
//
//	q := FromEuler(30*Pi/180, 45*Pi/180, 60*Pi/180)
//	
//	vectors := []Vector3{
//		{1, 0, 0},
//		{0, 1, 0},
//		{0, 0, 1},
//	}
//	
//	for i, v := range vectors {
//		v_rot := q.RotateVector(v)
//		fmt.Printf("v[%d] rotated: [%.3f, %.3f, %.3f]\n", i, v_rot[0], v_rot[1], v_rot[2])
//	}
//
// Rotating TCP in forward kinematics (along with translation from dual quaternion):
//
//	dq := exponent_map  // SE(3) transformation
//	tcp_position := dq.GetTranslation()  // Translation part
//	tcp_rotation := dq.GetRotation()      // Rotation part
//	
//	// To apply transformation: translate and rotate offset vector
//	local_offset := Vector3{0, 0, 100}  // 100 mm reach
//	world_offset := tcp_rotation.RotateVector(local_offset)
//	tcp_world := Vector3{
//		tcp_position[0] + world_offset[0],
//		tcp_position[1] + world_offset[1],
//		tcp_position[2] + world_offset[2],
//	}
//
// Notes:
//
//   - Quaternion must be normalized (unit quaternion) for proper rotation
//   - The input vector is not modified
//   - This is the standard method to apply quaternion rotations to 3D points
//   - Used extensively in SE(3) transformation composition
//
// See also: Conjugate, Multiply, FromEuler
//
func (q Quaternion) RotateVector(v Vector3) Vector3 {
	vq := Quaternion{0, v[0], v[1], v[2]}
	result := q.Multiply(vq).Multiply(q.Conjugate())
	return Vector3{result.X, result.Y, result.Z}
}

// ToEuler converts a unit quaternion to Euler angles in radians.
//
// Returns angles in the ZYX convention (intrinsic rotations, also called Tait-Bryan angles):
//   - Roll (φ): Rotation around X-axis (pitch forward/backward)
//   - Pitch (θ): Rotation around Y-axis (roll left/right)
//   - Yaw (ψ): Rotation around Z-axis (heading/compass direction)
//
// The conversion formulas use atan2 for all-quadrant arctangent and asin with clamping
// to handle edge cases and singularities (gimbal lock at ±90° pitch).
//
// Returns:
//
//   - Vector3: [roll, pitch, yaw] in radians where:
//     [0] = roll (≈ rotation around X-axis)
//     [1] = pitch (≈ rotation around Y-axis, range [-π/2, π/2])
//     [2] = yaw (≈ rotation around Z-axis)
//
// Singularities (Gimbal Lock):
//
// At pitch = ±π/2, the roll and yaw axes become aligned (singularity).
// The function returns yaw = 0 and places the entire rotation in roll.
// This is unavoidable with Euler angles; consider keeping as quaternions for stability.
//
// Performance: ~25 CPU cycles (atan2 calls are expensive)
//
// Examples:
//
// Converting quaternion back to Euler angles:
//
//	q := FromEuler(45*Pi/180, 30*Pi/180, 60*Pi/180)
//	angles := q.ToEuler()
//	
//	fmt.Printf("Roll: %.2f°\n", RadToDeg(angles[0]))    // ~45°
//	fmt.Printf("Pitch: %.2f°\n", RadToDeg(angles[1]))   // ~30°
//	fmt.Printf("Yaw: %.2f°\n", RadToDeg(angles[2]))     // ~60°
//
// Round-trip conversion (quaternion → Euler → quaternion):
//
//	q1 := FromEuler(0.5, 0.3, 1.2)  // Original quaternion
//	angles := q1.ToEuler()           // Convert to Euler
//	q2 := FromEuler(angles[0], angles[1], angles[2])  // Convert back
//	
//	// q1 and q2 should represent the same rotation (small floating-point error)
//
// Converting XYZWPR pose orientation to check consistency:
//
//	pose := &XYZWPR{X: 100, Y: 200, Z: 300, W: 45, P: 30, R: 60}  // W/P/R are in degrees
//	q := FromEuler(DegToRad(pose.R), DegToRad(pose.P), DegToRad(pose.W))
//	convertedAngles := q.ToEuler()
//	
//	fmt.Printf("Original W/P/R: %.2f° / %.2f° / %.2f°\n", pose.W, pose.P, pose.R)
//	fmt.Printf("Via quaternion:  %.2f° / %.2f° / %.2f°\n",
//		RadToDeg(convertedAngles[2]), RadToDeg(convertedAngles[1]), RadToDeg(convertedAngles[0]))
//
// Notes:
//
//   - The input quaternion should be normalized for accurate results
//   - Euler angles are not gimbal-lock free; use quaternions for stability in animation
//   - The ZYX convention is standard in aerospace; robotics may use different conventions
//   - For display to users (XYZWPR), convert degrees: roll=R, pitch=P, yaw=W
//
// See also: FromEuler, RadToDeg
//
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

// FromEuler creates a unit quaternion from Euler angles in the ZYX convention.
//
// This is the inverse of ToEuler. Given three rotation angles (in radians), constructs
// the equivalent unit quaternion representing the combined rotation.
//
// The ZYX convention applies rotations in this order:
// 1. Rotate around Z-axis (yaw) by angle ψ
// 2. Rotate around Y-axis (pitch) by angle θ
// 3. Rotate around X-axis (roll) by angle φ
//
// Parameters:
//
//   - roll: Rotation around X-axis in radians (φ)
//   - pitch: Rotation around Y-axis in radians (θ)
//   - yaw: Rotation around Z-axis in radians (ψ)
//
// Returns:
//
//   - Quaternion: Unit quaternion representing the combined rotation
//
// Mathematical computation:
//
// Uses half-angle formulas for numerical stability:
//
//	cr = cos(roll/2),  sr = sin(roll/2)
//	cp = cos(pitch/2), sp = sin(pitch/2)
//	cy = cos(yaw/2),   sy = sin(yaw/2)
//	
//	q = [
//	  w = cr*cp*cy + sr*sp*sy,
//	  x = sr*cp*cy - cr*sp*sy,
//	  y = cr*sp*cy + sr*cp*sy,
//	  z = cr*cp*sy - sr*sp*cy
//	]
//
// Performance: ~40 CPU cycles (8 trig functions via precomputation)
//
// Examples:
//
// Creating a rotation from Euler angles:
//
//	// 45° around X, 30° around Y, 60° around Z
//	q := FromEuler(45*Pi/180, 30*Pi/180, 60*Pi/180)
//	fmt.Printf("Quaternion: %+v\n", q)
//
// Converting XYZWPR pose to quaternion-based representation:
//
//	pose := &XYZWPR{X: 1000, Y: 500, Z: 800, W: 45, P: 30, R: 60}
//	
//	// Note: In robot poses, order is often Roll, Pitch, Yaw (not Yaw, Pitch, Roll)
//	// Adjust conversion based on your robot's convention
//	q := FromEuler(
//		DegToRad(pose.R),  // Roll from R component
//		DegToRad(pose.P),  // Pitch from P component
//		DegToRad(pose.W),  // Yaw from W component
//	)
//
// Creating intermediate orientations via interpolation (SLERP):
//
//	q1 := FromEuler(0, 0, 0)
//	qf := FromEuler(0, Pi/4, 0)
//	
//	// Linear interpolation in quaternion space (not quite SLERP, but simple)
//	for t := 0.0; t <= 1.0; t += 0.1 {
//		// In production, use proper SLERP instead
//		q := Quaternion{
//			W: (1-t)*q1.W + t*qf.W,
//			X: (1-t)*q1.X + t*qf.X,
//			Y: (1-t)*q1.Y + t*qf.Y,
//			Z: (1-t)*q1.Z + t*qf.Z,
//		}
//		q.Normalize()  // Restore unit quaternion
//		fmt.Printf("t=%.1f: %+v\n", t, q.ToEuler())
//	}
//
// Notes:
//
//   - The input angles are in radians; use DegToRad() to convert from degrees
//   - The resulting quaternion is always normalized (unit quaternion)
//   - Use this to initialize rotations for forward/inverse kinematics
//   - The ZYX convention is standard in aerospace; verify your robot's convention before use
//
// See also: ToEuler, DegToRad, IdentityQuaternion
//
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

// IdentityDualQuaternion returns the identity transformation (no translation, no rotation).
//
// Represents SE(3) identity:
//   - Real part: Identity quaternion (no rotation)
//   - Dual part: Zero quaternion (no translation)
//
// Used as the starting point for composition chains in the Product of Exponentials formula.
//
// Returns:
//
//   - DualQuaternion: Identity transformation T = I (4×4 homogeneous matrix equivalent)
//
// Usage: Initialize transformation chains before composing joint exponentials.
//
// Example:
//
//	T := IdentityDualQuaternion()
//	for i := 0; i < 6; i++ {
//		exp_i := robot.ExponentialMap(robot.Twists[i], theta[i])
//		T = T.Multiply(exp_i)  // Compose transformations
//	}
//	T = T.Multiply(robot.M)  // Apply home configuration
//
// See also: Multiply, ExponentialMap
//
func IdentityDualQuaternion() DualQuaternion {
	return DualQuaternion{
		Real: IdentityQuaternion(),
		Dual: Quaternion{0, 0, 0, 0},
	}
}

// Multiply composes two SE(3) transformations represented as dual quaternions.
//
// Dual quaternion multiplication corresponds to matrix multiplication of 4×4 homogeneous
// transformation matrices: T_result = T1 * T2 in SE(3).
//
// Used in the Product of Exponentials formula to chain joint transformations:
//
//	T_ee = exp(ξ_1 θ_1) * exp(ξ_2 θ_2) * ... * exp(ξ_6 θ_6) * M
//
// Where each term is a dual quaternion representing one joint's exponential map.
//
// Mathematical formula:
//
//	dq_result = dq1 ⊗ dq2
//	
//	Real: dq1.Real ⊗ dq2.Real
//	Dual: (dq1.Real ⊗ dq2.Dual) + (dq1.Dual ⊗ dq2.Real)
//
// Parameters:
//
//   - dq2: The second dual quaternion to compose (not modified)
//
// Returns:
//
//   - DualQuaternion: The composed transformation representing T_result = T1 * T2
//
// Performance: ~35 CPU cycles (3 quaternion multiplications + 1 addition)
//
// Examples:
//
// Computing forward kinematics using PoE:
//
//	r := NewR30iBRobot()
//	angles := [6]float64{0, -45, 45, 0, 0, 0}  // degrees
//	
//	T := IdentityDualQuaternion()
//	for i := 0; i < 6; i++ {
//		theta := DegToRad(angles[i])
//		exp_i := r.ExponentialMap(r.Twists[i], theta)
//		T = T.Multiply(exp_i)  // Chain transformations
//	}
//	T = T.Multiply(r.M)  // Apply home configuration
//	
//	// Extract TCP pose from T
//	pos := T.GetTranslation()
//	rot := T.GetRotation()
//	fmt.Printf("TCP at: [%.1f, %.1f, %.1f] mm\n", pos[0], pos[1], pos[2])
//
// Chaining transformations step-by-step:
//
//	T1 := robot.ExponentialMap(twist1, angle1)
//	T2 := robot.ExponentialMap(twist2, angle2)
//	T3 := robot.ExponentialMap(twist3, angle3)
//	
//	// Sequential composition
//	T_total := T1.Multiply(T2).Multiply(T3)
//	
//	// Or build up incrementally
//	T := IdentityDualQuaternion()
//	T = T.Multiply(T1)
//	T = T.Multiply(T2)
//	T = T.Multiply(T3)
//
// Notes:
//
//   - Order matters: dq1.Multiply(dq2) ≠ dq2.Multiply(dq1)
//   - This is the standard SE(3) composition operation
//   - Corresponds to homogeneous matrix multiplication
//   - Used in all forward kinematics computations
//   - Both dual quaternions should represent valid SE(3) transformations
//
// See also: ExponentialMap, GetTranslation, GetRotation
//
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

// GetTranslation extracts the position vector (translation) from a dual quaternion.
//
// A dual quaternion encodes both rotation (real part) and translation (dual part).
// This method extracts the translation component as a 3D vector.
//
// The extraction formula is:
//
//	t = 2 * dq.Dual ⊗ dq.Real*
//
// Where dq.Real* is the conjugate of the rotation part.
//
// Returns:
//
//   - Vector3: The position [x, y, z] in the world frame (typically in millimeters for robots)
//
// Performance: ~20 CPU cycles (1 conjugate + 1 multiply + vector extraction)
//
// Examples:
//
// Extracting TCP position from forward kinematics:
//
//	r := NewR30iBRobot()
//	angles := [6]float64{0, -45, 45, 0, 0, 0}  // degrees
//	
//	T := IdentityDualQuaternion()
//	for i := 0; i < 6; i++ {
//		theta := DegToRad(angles[i])
//		exp_i := r.ExponentialMap(r.Twists[i], theta)
//		T = T.Multiply(exp_i)
//	}
//	T = T.Multiply(r.M)
//	
//	tcpPos := T.GetTranslation()
//	fmt.Printf("TCP at X=%.1f Y=%.1f Z=%.1f mm\n", tcpPos[0], tcpPos[1], tcpPos[2])
//
// Computing position error for inverse kinematics:
//
//	currentPose := T.GetTranslation()
//	targetPose := &XYZWPR{X: 500, Y: 300, Z: 800}
//	
//	error := Distance(currentPose,
//		Vector3{targetPose.X, targetPose.Y, targetPose.Z})
//	fmt.Printf("Position error: %.2f mm\n", error)
//
// Separating rotation and translation for analysis:
//
//	T := /* computed transformation */
//	pos := T.GetTranslation()
//	rot := T.GetRotation()
//	
//	// Use position for Cartesian workspace analysis
//	// Use rotation for orientation constraints
//	workspaceRadius := Distance(Vector3{0, 0, 0}, pos)
//	fmt.Printf("Distance from base: %.1f mm\n", workspaceRadius)
//
// Notes:
//
//   - The returned vector is a new Vector3 (not a reference)
//   - Units depend on the robot: typically millimeters (mm) for industrial robots
//   - Always called after computing transformations in ForwardKinematics
//   - Works with any SE(3) dual quaternion representing rigid transformations
//
// See also: GetRotation, IdentityDualQuaternion
//
func (dq DualQuaternion) GetTranslation() Vector3 {
	q_conj := dq.Real.Conjugate()
	result := dq.Dual.Multiply(q_conj)
	return Vector3{2 * result.X, 2 * result.Y, 2 * result.Z}
}

// GetRotation extracts the orientation (rotation quaternion) from a dual quaternion.
//
// A dual quaternion encodes both rotation (real part) and translation (dual part).
// This method returns just the rotation component as a unit quaternion.
//
// Returns:
//
//   - Quaternion: The rotation as a unit quaternion (no translation information)
//
// Performance: O(1), mere field access
//
// Examples:
//
// Extracting TCP orientation from forward kinematics:
//
//	r := NewR30iBRobot()
//	angles := [6]float64{0, -45, 45, 0, 0, 0}  // degrees
//	
//	T := IdentityDualQuaternion()
//	for i := 0; i < 6; i++ {
//		theta := DegToRad(angles[i])
//		exp_i := r.ExponentialMap(r.Twists[i], theta)
//		T = T.Multiply(exp_i)
//	}
//	T = T.Multiply(r.M)
//	
//	tcpRot := T.GetRotation()
//	eulerAngles := tcpRot.ToEuler()
//	fmt.Printf("TCP orientation: Roll=%.1f° Pitch=%.1f° Yaw=%.1f°\n",
//		RadToDeg(eulerAngles[0]), RadToDeg(eulerAngles[1]), RadToDeg(eulerAngles[2]))
//
// Rotating a vector in TCP frame to world frame:
//
//	T := /* computed frame transformation */
//	tcpRot := T.GetRotation()
//	
//	// A vector in TCP frame
//	tcpVector := Vector3{100, 0, 0}  // Point 100mm in X direction from TCP
//	
//	// Same vector in world frame
//	worldVector := tcpRot.RotateVector(tcpVector)
//	fmt.Printf("World vector: [%.1f, %.1f, %.1f]\n", worldVector[0], worldVector[1], worldVector[2])
//
// Checking angular difference between two poses:
//
//	T1 := /* first cartesian pose */
//	T2 := /* second cartesian pose */
//	
//	rot1 := T1.GetRotation()
//	rot2 := T2.GetRotation()
//	
//	// Relative rotation from T1 to T2
//	relRot := rot1.Conjugate().Multiply(rot2)
//	
//	// Angular distance
//	angularDistance := 2 * math.Acos(relRot.W)  // W component is cos(θ/2)
//	fmt.Printf("Angular difference: %.2f°\n", RadToDeg(angularDistance))
//
// Notes:
//
//   - This returns only the rotation; use GetTranslation() for position
//   - The returned quaternion is the real part of the dual quaternion
//   - Typically normalized, but may drift under repeated multiplications
//   - For complete pose, always call both GetTranslation() and GetRotation()
//
// See also: GetTranslation, RotateVector, ToEuler
//
func (dq DualQuaternion) GetRotation() Quaternion {
	return dq.Real
}

// DegToRad converts an angle from degrees to radians.
//
// Formula: radians = degrees × π/180
//
// Roboticists typically work in degrees (more intuitive), but trigonometric functions
// in Go expect radians. Use this before passing angles to Euler/quaternion conversions.
//
// Parameters:
//
//   - deg: Angle in degrees (can be any value, including negative and > 360)
//
// Returns:
//
//   - float64: Equivalent angle in radians
//
// Performance: ~3 CPU cycles (multiply + divide, precomputed constant)
//
// Examples:
//
// Converting joint angles to radians:
//
//	angles := [6]float64{0, -45, 45, 0, 0, 0}  // degrees
//	anglesRad := make([]float64, 6)
//	for i, a := range angles {
//		anglesRad[i] = DegToRad(a)
//	}
//
// Converting XYZWPR pose to quaternion for kinematics:
//
//	pose := &XYZWPR{X: 500, Y: 300, Z: 800, W: 45, P: 30, R: 60}
//	q := FromEuler(
//		DegToRad(pose.R),
//		DegToRad(pose.P),
//		DegToRad(pose.W),
//	)
//
// Notes:
//
// - Inverse of RadToDeg
// - The conversion is always accurate (no rounding domain issues)
// - Common angles: 90° = π/2 ≈ 1.5708, 180° = π ≈ 3.1416, 360° = 2π ≈ 6.2832
//
// See also: RadToDeg, FromEuler
//
func DegToRad(deg float64) float64 {
	return deg * math.Pi / 180.0
}

// RadToDeg converts an angle from radians to degrees.
//
// Formula: degrees = radians × 180/π
//
// Used after quaternion operations (which produce radians) to display angles in
// human-re calculates the Euclidean distance between two 3D points.
//
// Formula: d = √((x₂-x₁)² + (y₂-y₁)² + (z₂-z₁)²)
//
// Used extensively in collision detection, workspace analysis, and IK convergence checking.
//
// Parameters:
//
//   - v1: First 3D point [x, y, z]
//   - v2: Second 3D point [x, y, z]
//
// Returns:
//
//   - float64: Euclidean distance between the two points (non-negative)
//
// Performance: ~25 CPU cycles (3 subtractions + 3 multiplications + 1 sqrt + accumulation)
//
// Examples:
//
// Checking IK convergence (position error):
//
//	currentPos := computedPose.GetTranslation()
//	targetPos := Vector3{targetX, targetY, targetZ}
//	
//	error := Distance(currentPos, targetPos)
//	if error < 0.5 {  // Converged to 0.5 mm
//		fmt.Println("IK Solution found!")
//	}
//
// Workspace reachability analysis:
//
//	basePos := Vector3{0, 0, 0}
//	pointOfInterest := Vector3{500, 400, 600}
//	
//	reach := Distance(basePos, pointOfInterest)
//	r30ibMaxReach := 1700.0  // mm
//	
//	if reach <= r30ibMaxReach {
//		fmt.Println("Point is reachable")
//	} else {
//		fmt.Println("Point exceeds robot reach")
//	}
//
// Collision detection with sphere obstacles:
//
//	tcpPos := T.GetTranslation()
//	obstacleCenter := Vector3{800, 500, 1200}
//	obstacleRadius := 100.0  // mm
//	
//	distToObstacle := Distance(tcpPos, obstacleCenter)
//	safetyMargin := 50.0  // mm
//	
//	if distToObstacle < (obstacleRadius + safetyMargin) {
//		fmt.Println("Collision warning!")
//	}
//
// Measuring TCP path length in trajectory:
//
//	totalLength := 0.0
//	for i := 1; i < len(trajectory); i++ {
//		segment := Distance(trajectory[i-1].GetTranslation(), trajectory[i].GetTranslation())
//		totalLength += segment
//	}
//	fmt.P restricts a value to stay within a specified range [min, max].
//
// If the value is below min, returns min.
// If the value is above max, returns max.
// Otherwise, returns the value unchanged.
//
// Formula:
//
//	clamp(val, min, max) = {
//	    min,  if val < min
//	    val,  if min ≤ val ≤ max
//	    max,  if val > max
//	}
//
// Used for saturating/limiting values to safe ranges, especially for:
//   - Joint angle limits
//   - Velocity/acceleration constraints
//   - Numerical algorithm convergence bounds
//
// Parameters:
//
//   - val: The value to clamp
//   - min: Minimum allowed value (lower bound, inclusive)
//   - max: Maximum allowed value (upper bound, inclusive)
//
// Returns:
//
//   - float64: The clamped value within [min, max]
//
// Precondition: Expects min ≤ max; behavior undefined if min > max
//
// Performance: O(1), simple comparisons (no expensive operations)
//
// Examples:
//
// Limiting joint angles to safe ranges:
//
//	j1_min, j1_max := -180.0, 180.0
//	computed_j1 := -210.0  // Violates limit
//	
//	safe_j1 := DegToRad(Clamp(RadToDeg(computed_j1), j1_min, j1_max))
//	// Result: -180° (clamped)
//
// Constraining motion within workspace:
//
//	// Y coordinate should stay in valid workspace
//	y_min, y_max := -500.0, 500.0  // mm
//	
//	proposed_y := 550.0  // Exceeds workspace
//	safe_y := Clamp(proposed_y, y_min, y_max)
//	// Result: 500.0 mm
//
// Velocity profiling with acceleration limits:
//
//	const maxAccel = 50.0  // mm/s²
//	dt := 0.01  // 10 ms control cycle
//	
//	desired_vel := 200.0  // mm/s
//	current_vel := 100.0  // mm/s
//	
//	max_vel_change := maxAccel * dt
//	new_vel := Clamp(
//		desired_vel,
//		current_vel - max_vel_change,
//		current_vel + max_vel_change,
//	)
//	// Result: 100.5 mm/s (limited acceleration)
//
// Numerical algorithm bounds:
//
//	// Prevent gradient descent step from becoming too large
//	alpha := 0.03  // Base learning rate
//	gradient := /* computed gradient */
//	
//	// Limit step size even if gradient is large
//	max_step := 0.1
//	step := Clamp(alpha * gradient, -max_step, max_step)
//	theta -= step
//
// Notes:
//
//   - Essential for safety in robotic systems
//   - Always check/clamp user inputs before sending to robot hardware
//   - Use in all numerical algorithms that might drift out of bounds
//   - Symmetric clamp: use Clamp(val, -limit, limit) for ±bounds
//
// See also: CheckJointLimits
//\n", totalLength)
//
// Notes:
//
// - Always non-negative
// - Distance(v1, v2) = Distance(v2, v1) (symmetric)
// - For 3D robotics, units are typically millimeters (mm)
// - Used as convergence criterion in numerical IK solvers
//
// See also: Clamp, Vector3
//ot teaching and debugging.
//
// Parameters:
//
//   - rad: Angle in radians (can be any value, including negative and > 2π)
//
// Returns:
//
//   - float64: Equivalent angle in degrees
//
// Performance: ~3 CPU cycles (multiply + divide, precomputed constant)
//
// Examples:
//
// Converting computed radians results back to degrees for display:
//
//	euler := q.ToEuler()
//	fmt.Printf("Orientation: Roll=%.1f° Pitch=%.1f° Yaw=%.1f°\n",
//		RadToDeg(euler[0]), RadToDeg(euler[1]), RadToDeg(euler[2]))
//
// Creating joint angle output from inverse kinematics:
//
//	anglesRad := [6]float64{ /* IK results in radians */ }
//	fmt.Println("Joint angles (degrees):")
//	for i, rad := range anglesRad {
//		fmt.Printf("  J%d: %.2f°\n", i+1, RadToDeg(rad))
//	}
//
// Formatting XYZWPR pose for display:
//
//	T := /* transformation from FK */
//	rot := T.GetRotation()
//	eulerRad := rot.ToEuler()
//	
//	pose := XYZWPR{
//		X: T.GetTranslation()[0],
//		Y: T.GetTranslation()[1],
//		Z: T.GetTranslation()[2],
//		R: RadToDeg(eulerRad[0]),  // Roll to degrees
//		P: RadToDeg(eulerRad[1]),  // Pitch to degrees
//		W: RadToDeg(eulerRad[2]),  // Yaw to degrees
//	}
//
// Notes:
//
// - Inverse of DegToRad
// - Always use for displaying angles to users
// - Common angles: π ≈ 1.5708 rad = 180°, π/2 ≈ 0.7854 rad = 90°, 2π ≈ 6.2832 rad = 360°
//
// See also: DegToRad, ToEuler
//
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
