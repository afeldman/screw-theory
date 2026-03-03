package kinematics

// NewR30iBRobot: Create FANUC R-30iB robot with screw theory kinematics
func NewR30iBRobot() *Robot {
	twists := []Twist{
		{Omega: [3]float64{0, 0, 1}, V: [3]float64{0, 0, 0}},
		{Omega: [3]float64{0, 1, 0}, V: [3]float64{0, 0, 330}},
		{Omega: [3]float64{0, 1, 0}, V: [3]float64{600, 0, 330}},
		{Omega: [3]float64{0, 1, 0}, V: [3]float64{1075, 0, 330}},
		{Omega: [3]float64{0, 0, 1}, V: [3]float64{1075, 0, 800}},
		{Omega: [3]float64{0, 1, 0}, V: [3]float64{1075, 0, 800}},
	}

	M := DualQuaternion{
		Real: Quaternion{W: 1},
		Dual: Quaternion{
			W: 0,
			X: 1075.0 / 2.0,
			Z: 900.0 / 2.0,
		},
	}

	return &Robot{
		name:        "FANUC R-30iB",
		twists:      twists,
		M:           M,
		numJoints:   6,
		linkLengths: []float64{330, 600, 475, 470, 0, 100},
	}
}
