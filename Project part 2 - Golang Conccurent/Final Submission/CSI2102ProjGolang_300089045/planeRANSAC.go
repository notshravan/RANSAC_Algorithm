package main

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
func GetSupportingPoints(plane Plane3D, points []Point3D, eps float64) []Point3D {
	var supportingPoints []Point3D
	for _, pt := range points {
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

		planeStream := PlaneEstimator(TakeN(wg, stopChan,
			TripletPointGen(GenRandomPoints(points, RandomPoints, stopChan, wg), stopChan, wg), numIterations), stopChan, wg)

		var fanOut = 16
		wg.Add(fanOut)
		supportingPointsStream := make([]<-chan Plane3DwSupport, fanOut)
		for j := 0; j < fanOut; j++ {
			supportingPointsStream[j] = SupportingPointsFinder(wg, stopChan, planeStream, &points, distanceThreshold)
		}

		wg.Add(1)
		DominantPlaneIdentifier(wg, stopChan, FanIn(wg, stopChan, supportingPointsStream), &bestPlaneSupport)

		close(stopChan)
		wg.Wait()

		fmt.Printf("Best plane support: %v\nBest support count: %d\n", bestPlaneSupport.Plane3D, bestPlaneSupport.SupportSize)

		saveFilename := fmt.Sprintf("%s_p%d.xyz", strings.TrimSuffix(filename, ".xyz"), i+1)
		saveXYZ(saveFilename, GetSupportingPoints(bestPlaneSupport.Plane3D, points, distanceThreshold))

		points = RemovePlane(bestPlaneSupport.Plane3D, &points, distanceThreshold)

		fmt.Printf("Finished processing plane #%d\n", i+1)
	}

	fmt.Println("\nFinished processing all planes")
	fmt.Printf("Total runtime: %v\n", time.Since(startTime))
}

// Returns a random point in the point splice
func RandomPoints(points []Point3D) Point3D {
	rand_Index := rand.Intn(len(points))
	return points[rand_Index]
}

// Generates randoms points from the provided slice of points.
// Outputs channel transmits instances of Point3D
func GenRandomPoints(points []Point3D, fct func([]Point3D) Point3D, stop <-chan bool, wg *sync.WaitGroup) <-chan Point3D {
	pointStream := make(chan Point3D)
	go func() {
		defer func() {
			close(pointStream)
			wg.Done()
		}()
		for {
			select {
			case <-stop:
				return
			case pointStream <- fct(points):
			}
		}
	}()
	return pointStream
}

// Reads Point3D instances from input channel and accumulate 3 points
// Output channel transmits arrays of Point3D
func TripletPointGen(inputPointStream <-chan Point3D, stop <-chan bool, wg *sync.WaitGroup) <-chan []Point3D {
	outputPointStream := make(chan []Point3D)
	go func() {
		defer func() {
			close(outputPointStream)
			wg.Done()
		}()
		for {
			select {
			case <-stop:
				return
			default:
				pointSlice := []Point3D{<-inputPointStream}
				for i := 0; i < 2; i++ {
					select {
					case <-stop:
						return
					case p := <-inputPointStream:
						pointSlice = append(pointSlice, p)
					}
				}
				outputPointStream <- pointSlice
			}
		}
	}()
	return outputPointStream
}

// Reads arrays of Point3D and resends them.
// Automatically stops the pipeline after received N arrays
func TakeN(wg *sync.WaitGroup, stop <-chan bool, inputPointStream <-chan []Point3D, n int) <-chan []Point3D {
	outputChannel := make(chan []Point3D)
	go func() {
		defer func() {
			close(outputChannel)
			wg.Done()
		}()
		for i := 0; i < n; i++ {
			select {
			case <-stop:
				return
			case outputChannel <- <-inputPointStream:
			}
		}
	}()
	return outputChannel
}

// Reads arrays of three Point3D and compute the plane defined by these points.
// Output channel transmits Plane3D instances.
func PlaneEstimator(inputPointStream <-chan []Point3D, stop <-chan bool, wg *sync.WaitGroup) <-chan Plane3D {
	outputPlaneStream := make(chan Plane3D)
	go func() {
		defer func() {
			close(outputPlaneStream)
			wg.Done()
		}()
		for {
			select {
			case <-stop:
				return
			case pointSlice := <-inputPointStream:
				if len(pointSlice) == 3 {
					plane := GetPlane(pointSlice)
					outputPlaneStream <- plane
				}
			}
		}
	}()
	return outputPlaneStream
}

// Counts the number of points in the provided slice Point3D(input point cloud) that support the receied Plane3D
// Output channel transmits the Plane3D and the number of supporting points in a Point3DwSupport instances.
func SupportingPointsFinder(wg *sync.WaitGroup, stop <-chan bool, inputPlaneStream <-chan Plane3D, pointCloud *[]Point3D, eps float64) <-chan Plane3DwSupport {
	outputPlaneStream := make(chan Plane3DwSupport)

	go func() {
		defer func() {
			close(outputPlaneStream)
			wg.Done()
		}()
		for i := range inputPlaneStream {
			select {
			case <-stop:
				return
			case outputPlaneStream <- GetSupport(i, pointCloud, eps):
			}
		}

	}()

	return outputPlaneStream
}

// Multiplexes the results received from multiple channels into one output channel
func FanIn(wg *sync.WaitGroup, stop <-chan bool, channels []<-chan Plane3DwSupport) <-chan Plane3DwSupport {
	var multiplexGroup sync.WaitGroup
	outputPlaneSupportStream := make(chan Plane3DwSupport)

	reader := func(ch <-chan Plane3DwSupport) {
		defer func() { multiplexGroup.Done() }()
		for i := range ch {
			select {
			case <-stop:
				return
			case outputPlaneSupportStream <- i:
			}
		}
	}

	multiplexGroup.Add(len(channels))
	for _, ch := range channels {
		go reader(ch)
	}

	go func() {
		defer func() {
			close(outputPlaneSupportStream)
			wg.Done()
		}()
		multiplexGroup.Wait()
	}()

	return outputPlaneSupportStream
}

// Receives Plane3DwSupport instances and keeps in memory the plane with the best support
func DominantPlaneIdentifier(wg *sync.WaitGroup, stop <-chan bool, inputPlaneSupportStream <-chan Plane3DwSupport, bestSupport *Plane3DwSupport) {

	for i := range inputPlaneSupportStream {
		if bestSupport.SupportSize < i.SupportSize {
			*bestSupport = i
		}
	}
}
