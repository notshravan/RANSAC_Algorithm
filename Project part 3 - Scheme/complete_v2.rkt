#lang racket
;; Read point cloud data from a file
(define (readXYZ fileIn)  
 (let ((sL (map (lambda s (string-split (car s)))  
 (cdr (file->lines fileIn)))))  (map (lambda (L) 
 (map (lambda (s) 
 (if (eqv? (string->number s) #f) 
 s 
(string->number s))) L)) sL))) 

;; RANSAC number of iterations computation
(define (ransac-number-of-iterations confidence percentage)
  (let ((log-denom (log (- 1 (expt percentage 3)))))
    (ceiling (/ (log (- 1 confidence)) log-denom))
  )
)

;; Plane equation from 3 points
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

;; Calculate support for a plane
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
      (abs (/ (+ (* a x) (* b y) (* c z) d) (sqrt (+ (expt a 2) (expt b 2) (expt c 2)))))
    )
  )
  (let loop ((points points) (support-count 0))
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
  ) ;(display iterations)
  (iterative-ransac Ps '() 0 iterations);initially calling the Ransac algoritm (best plane is empty '()), support is 0, and num iterations calculated
)