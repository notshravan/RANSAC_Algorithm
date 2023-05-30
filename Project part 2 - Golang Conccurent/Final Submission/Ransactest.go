package Final_Submission

import (
	"bufio"
	"fmt"
	"log"
	"math"
	"math/rand"
	"os"
	"strconv"
	"strings"
	"sync"
	"time"

)

type Point3D struct {
	X float64
	Y float64
	Z float64
}

type Plane3D struct {
	A float64
	B float64
	C float64
	D float64
}

type Plane3DwSupport struct {
	Plane3D
	SupportSize int
}

// ReadXYZ reads an XYZ file and returns a slice of Point3D
func ReadXYZ(filename string) []Point3D {
	file, err := os.Open(filename)
	if err != nil {
		fmt.Println(err)
		os.Exit(1)
	}
	defer file.Close()

	var points []Point3D
	scanner := bufio.NewScanner(file)
	for scanner.Scan() {
		line := scanner.Text()
		fields := strings.Fields(line)
		if len(fields) != 3 {
			continue
		}
		x, err := strconv.ParseFloat(fields[0], 64)
		if err != nil {
			continue
		}
		y, err := strconv.ParseFloat(fields[1], 64)
		if err != nil {
			continue
		}
		z, err := strconv.ParseFloat(fields[2], 64)
		if err != nil {
			continue
		}
		points = append(points, Point3D{x, y, z})
	}

	if err := scanner.Err(); err != nil {
		fmt.Println(err)
		os.Exit(1)
	}

	return points
}

// Saves a slice of Point3D into an XYZ file.
func saveXYZ(filename string, points []Point3D) {
	f, err := os.Create(filename)
	if err != nil {
		log.Fatal(err)
	}
	defer f.Close()
	fmt.Fprintf(f, "x\ty\tz\n")
	for _, pt := range points {
		fmt.Fprintf(f, "%v\t%v\t%v\n", pt.X, pt.Y, pt.Z)
	}
}

// Calculates the distance between a point and a plane.
func (plane *Plane3D) GetDistance(p1 *Point3D) float64 {
	// Compute the signed distance using the plane equation
	Numerator := (plane.A*p1.X + plane.B*p1.Y + plane.C*p1.Z + plane.D)

	// Return the absolute value of the signed distance as the unsigned distance
	return math.Abs(Numerator) / math.Sqrt(plane.A*plane.A+plane.B*plane.B+plane.C*plane.C)
}

// Computes the plane defined by a slice of 3 points.
func GetPlane(points []Point3D) Plane3D {
	// Calculate centroid of points
	var centroid Point3D
	for _, p := range points {
		centroid.X += p.X //average of the coordinates of all the points (X).
		centroid.Y += p.Y //average of the coordinates of all the points (Y).
		centroid.Z += p.Z //average of the coordinates of all the points (Z).
	}
	centroid.X /= float64(len(points))
	centroid.Y /= float64(len(points))
	centroid.Z /= float64(len(points))

	// Calculate Difference Point vectors from centroid to two other points
	v1 := Point3D{points[0].X - centroid.X, points[0].Y - centroid.Y, points[0].Z - centroid.Z}
	v2 := Point3D{points[1].X - centroid.X, points[1].Y - centroid.Y, points[1].Z - centroid.Z}

	// Compute normal vector of plane as the cross product of v1 and v2
	normal := Point3D{
		(v1.Y * v2.Z) - (v1.Z * v2.Y),
		(v1.Z * v2.X) - (v1.X * v2.Z),
		(v1.X * v2.Y) - (v1.Y * v2.X),
	}

	// Compute D value of plane equation using the centroid and normal vector
	d := -((normal.X * centroid.X) + (normal.Y * centroid.Y) + (normal.Z * centroid.Z))

	// Construct plane and return
	return Plane3D{normal.X, normal.Y, normal.Z, d}
}

//func (p1 *Point3D) CalculateDifferenceVector(p2 *Point3D) Point3D {
//	return Point3D{
//		p2.X - p1.X,
//		p2.Y - p1.Y,
//		p2.Z - p1.Z,
//	}
//}

// computes the number of required RANSAC iterations
func GetNumberOfIterations(confidence float64, percentageOfPointsOnPlane float64) int {
	return (int)((math.Log(1 - confidence)) / (math.Log(1 - math.Pow(percentageOfPointsOnPlane, 3))))

}

// computes the support of a plane in a slice of points
func GetSupport(plane Plane3D, points *[]Point3D, eps float64) Plane3DwSupport {
	var support int
	for _, p := range *points {
		if plane.GetDistance(&p) < eps {
			support++
		}
	}
	return Plane3DwSupport{plane, support}
}

// extracts the points that supports the given plane
// and returns them as a slice of points
func GetSupportingPoints(plane Plane3D, points *[]Point3D, eps float64) []Point3D {
	var supportingPoints []Point3D
	for _, pt := range *points {
		distance := plane.GetDistance(&pt)
		if distance < eps {
			supportingPoints = append(supportingPoints, pt)
		}
	}
	return supportingPoints
}

// creates a new slice of points in which all points
// belonging to the plane have been removed
func RemovePlane(plane Plane3D, points *[]Point3D, eps float64) []Point3D {
	var remainingPoints []Point3D
	for _, pt := range *points {
		distance := plane.GetDistance(&pt)
		if math.Abs(distance) > eps {
			remainingPoints = append(remainingPoints, pt)
		}
	}
	return remainingPoints
}

func main() {
	rand.Seed(time.Now().UnixNano())

	filename := os.Args[1]
	fmt.Printf("Processing file: %s\n", filename)

	points := ReadXYZ(filename)
	confidence, err := strconv.ParseFloat(os.Args[2], 64)
	if err != nil {
		fmt.Printf("Error parsing confidence value: %v\n", err)
		os.Exit(1)
	}

	minPointsPercent, err := strconv.ParseFloat(os.Args[3], 64)
	if err != nil {
		fmt.Printf("Error parsing minimum points percentage value: %v\n", err)
		os.Exit(1)
	}

	distanceThreshold, err := strconv.ParseFloat(os.Args[4], 64)
	if err != nil {
		fmt.Printf("Error parsing distance threshold value: %v\n", err)
		os.Exit(1)
	}

	startTime := time.Now()

	for i := 0; i < 3; i++ {
		fmt.Printf("Estimating plane #%d\n", i+1)

		stopChan := make(chan bool)

		numIterations := GetNumberOfIterations(confidence, minPointsPercent)

		var bestPlaneSupport Plane3DwSupport = Plane3DwSupport{Plane3D{0, 0, 0, 0}, 0}
		wg := &sync.WaitGroup{}
		wg.Add(4)

		planeStream := PlaneEstimator(wg, stopChan,
			takeN(wg, stopChan,
				tripletPointGen(wg, stopChan,
					randomPointsGenerator(wg, stopChan, points), numIterations)))laneEstimator()

		fanOut := 16
		wg.Add(fanOut)
		supportingPointsStream := make([]<-chan planeWithSupport, fanOut)
		for j := 0; j < fanOut; j++ {
			supportingPointsStream[j] = supportingPointsFinder(wg, stopChan, planeStream, points, distanceThreshold)
		}

		wg.Add(1)
		dominantPlane := dominantPlaneIdentifier(wg, stopChan, fanIn(wg, stopChan, supportingPointsStream), &bestPlaneSupport)

		close(stopChan)
		wg.Wait()

		fmt.Printf("Best plane support: %v\nBest support count: %d\n", bestPlaneSupport.plane, bestPlaneSupport.supportSize)

		saveFilename := fmt.Sprintf("%s_p%d.xyz", strings.TrimSuffix(filename, ".xyz"), i+1)
		savePointCloud(saveFilename, getSupportingPoints(bestPlaneSupport.plane, points, distanceThreshold))

		points = removePlane(bestPlaneSupport.plane, points, distanceThreshold)

		fmt.Printf("Finished processing plane #%d\n", i+1)
	}

	fmt.Println("\nFinished processing all planes")
	fmt.Printf("Total runtime: %v\n", time.Since(startTime))
}
