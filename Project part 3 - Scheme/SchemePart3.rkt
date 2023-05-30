#lang racket



;Overview of all the functions
;readXYZ: Reads the point cloud data from a file.
;ransac-number-of-iterations: Calculates the number of iterations needed for RANSAC based on confidence and percentage.
;plane: Computes the plane equation from 3 points.
;support: Calculates the support for a plane given the point cloud and a distance threshold.
;dominant-plane: Finds the dominant plane using RANSAC with the provided number of iterations and distance threshold.
;plane-ransac: The main function that calls the other functions to perform the RANSAC algorithm.


;; Plane equation from 3 points
(define (plane P1 P2 P3)
  (let ((v12 (vector-subtract P2 P1))
        (v13 (vector-subtract P3 P1)))
    (let ((normal (cross-product v12 v13)))
      (let ((a (vector-ref normal 0))
            (b (vector-ref normal 1))
            (c (vector-ref normal 2)))
        (let ((d (- (+ (* a (vector-ref P1 0))
                       (* b (vector-ref P1 1))
                       (* c (vector-ref P1 2))))))
          (list a b c d)
        )
      )
    )
  )
)

;; Vector subtraction Similar to Vector-Sum given in lecture 19 notes.
(define (vector-subtract v1 v2)
  (map (lambda (x y) (- x y)) v1 v2)
)

;; Cross product
(define (cross-product v1 v2)
  (list (- (* (vector-ref v1 1) (vector-ref v2 2)) (* (vector-ref v1 2) (vector-ref v2 1)))
        (- (* (vector-ref v1 2) (vector-ref v2 0)) (* (vector-ref v1 0) (vector-ref v2 2)))
        (- (* (vector-ref v1 0) (vector-ref v2 1)) (* (vector-ref v1 1) (vector-ref v2 0)))
  )
)


;; Support for a given plane
;; Calculate support for a plane
(define (support plane-params points eps)
  (define (point-plane-distance plane-params point)
    (let ((a (car plane-params))
          (b (cadr plane-params))
          (c (caddr plane-params))
          (d (cadddr plane-params))
          (x (car point))
          (y (cadr point))
          (z (caddr point)))
      (abs (/ (+ (* a x) (* b y) (* c z) d) (sqrt (+ (expt a 2) (expt b 2) (expt c 2))))
    )
  )
  (let loop ((points points) (support-count 0)) ; count the points supporting the plane.
    (if (null? points)
        (cons support-count plane-params)
        (let ((distance (point-plane-distance plane-params (car points))))
          (if (< distance eps)
              (loop (cdr points) (+ support-count 1))
              (loop (cdr points) support-count)
          )
        )
    )
  )
)


;; Dominant plane computation
(define (dominant-plane Ps k)
  (if (= k 0)
      (cons -1 '()) ; Return an invalid plane if k is 0
      (let ((P1 (list-ref Ps (random (length Ps))))
            (P2 (list-ref Ps (random (length Ps))))
            (P3 (list-ref Ps (random (length Ps)))))
        (let ((plane (plane P1 P2 P3))
              (sup (support plane Ps)))
          (max-cons sup (dominant-plane Ps (- k 1))))
      )
  )
)

;; Utility function to get the maximum of two cons pairs
(define (max-cons a b)
  (if (> (car a) (car b))
      a
      b
))

;; RANSAC number of iterations computation
(define (ransac-number-of-iterations confidence percentage)
  (let ((p3 (expt percentage 3)))
    (inexact->exact (ceiling (/ (log (- 1 confidence)) (log (- 1 p3))))) ;inexact->exact
  )
)


;; Main function
(define (planeRANSAC filename confidence percentage eps)
  (let ((Ps (readXYZ filename)))
    (let ((k (ransac-number-of-iterations confidence percentage)))
      (dominant-plane Ps k)
    )
  )
)


;;Function to read directly from file
(define (readXYZ fileIn)  
 (let ((sL (map (lambda s (string-split (car s)))  
 (cdr (file->lines fileIn)))))  (map (lambda (L) 
 (map (lambda (s) 
 (if (eqv? (string->number s) #f) 
 s 
(string->number s))) L)) sL)))

  

