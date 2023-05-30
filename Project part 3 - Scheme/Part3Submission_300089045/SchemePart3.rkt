#lang racket

;Student Name: Shravan Vyas
;Student Number: 300089045
;Csi 2120 Part 3 - Scheme Implementation

;; Read point cloud data from a file
;; Input: fileIn - a file containing point cloud data
;; Output: A list of points, where each point is a list of three numbers (x, y, z)

(define (readXYZ fileIn)  
 (let ((sL (map (lambda s (string-split (car s)))  
 (cdr (file->lines fileIn)))))  (map (lambda (L) 
 (map (lambda (s) 
 (if (eqv? (string->number s) #f) 
 s 
(string->number s))) L)) sL))) 

;; RANSAC number of iterations computation
;; Input: confidence - desired probability of finding the dominant plane (e.g. 0.99)
;;        percentage - percentage of points supporting the dominant plane (e.g.Between 0.6 - 0.8)
;; Output: The number of iterations required to achieve the given confidence

(define (ransac-number-of-iterations confidence percentage)
  (let ((log-denom (log (- 1 (expt percentage 3)))))
    (ceiling (/ (log (- 1 confidence)) log-denom))
  )
)

;; Calculate Plane equation from 3 points
;; Input: P1, P2, P3 - three points (lists of three numbers) in 3D space
;; Output: A list of four numbers (a, b, c, d) representing the plane equation ax + by + cz + d = 0

(define (plane P1 P2 P3)
  (let* ((v1 (- (cadr P2) (cadr P1))) 
         (v2 (- (caddr P2) (caddr P1)))
         (v3 (- (car P2) (car P1)))
         (v4 (- (cadr P3) (cadr P1)))
         (v5 (- (caddr P3) (caddr P1)))
         (v6 (- (car P3) (car P1)))
         (a (- (* v2 v5) (* v4 v6)))
         (b (- (* v6 v1) (* v3 v5)))
         (c (- (* v4 v1) (* v3 v2)))
         (d (- (* (car P1) a) (* (cadr P1) b) (* (caddr P1) c))))
    (list a b c d)
))

;;Calculate Support for the Plane
;; Input: plane-params - a list of four numbers (a, b, c, d) representing the plane equation ax + by + cz + d = 0
;;        points - a list of points, where each point is a list of three numbers (x, y, z)
;;        eps - threshold distance for a point to be considered in support of the plane
;; Output: A pair (support-count . plane-params), where support-count is the number of points in support of the plane

(define (support plane-params points eps)
  (define (point-plane-distance plane-params point)
    (let ((a (car plane-params))
          (b (cadr plane-params))
          (c (caddr plane-params))
          (d (cadddr plane-params))
          (x (car point))
          (y (cadr point))
          (z (caddr point)))
      (abs (/ (+ (* a x) (* b y) (* c z) d) (sqrt (+ (expt a 2) (expt b 2) (expt c 2)))));does the plane equation calculation ax + by + cz = d, absolute value
    )
  )
  (let loop ((points points) (support-count 0));Iteratively compare each points to find all the supports, returns support-count
    (if (null? points)
        (cons support-count plane-params)
        (let ((distance (point-plane-distance plane-params (car points))))
          (if (< distance eps)
              (loop (cdr points) (+ support-count 1))
              (loop (cdr points) support-count))
        )
    )
  )
)


;; Main RANSAC function
;; Input: filename - the name of the file containing point cloud data
;;        confidence - desired probability of finding the dominant plane (e.g. Ideally 0.99)
;;        percentage - percentage of points supporting the dominant plane (e.g. between 0.6 - 0.8)
;;        eps - threshold distance for a point to be considered in support of the plane
;; Output: The dominant plane and its support count as a pair (plane-params/ the plane equation in terms of (a,b,c,d) . support-count)

(define (plane-ransac filename confidence percentage eps)
  (let ((Ps (readXYZ filename))
        (iterations (ransac-number-of-iterations confidence percentage)))
    (dominant-plane Ps iterations eps)
  )
)

;; Find the dominant plane using RANSAC
(define (dominant-plane Ps iterations eps)
  (define (iterative-ransac Ps best-plane best-support k); recursively find the best support
    (if (zero? k)
        (cons best-plane best-support)
        (let* ((P1 (list-ref Ps (random (length Ps)))) ;select 3 random poins from Ps
               (P2 (list-ref Ps (random (length Ps))))
               (P3 (list-ref Ps (random (length Ps))))
               (plane-params (plane P1 P2 P3))
               (support-result (support plane-params Ps eps))) ;finds the support 
          (if (> (car support-result) best-support)
              (iterative-ransac Ps plane-params (car support-result) (- k 1))
              (iterative-ransac Ps best-plane best-support (- k 1))
          )
        )
    )
  ) ;(display iterations) ;Uncomment this to find number of iterations
  (iterative-ransac Ps '() 0 iterations);initially calling the Ransac algoritm (best plane is empty '()), support is 0, and num iterations calculated
)
