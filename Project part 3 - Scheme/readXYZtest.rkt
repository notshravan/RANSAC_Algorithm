#lang racket
(define (readXYZ fileIn)  
 (let ((sL (map (lambda s (string-split (car s)))  
 (cdr (file->lines fileIn)))))

(map (lambda (L) 
       (map (lambda (s) 
              (if (eqv? (string->number s) #f) 
                  s 
                  (string->number s))) L)
        ) sL)
   )
  )

(readXYZ "Point_Cloud_1_No_Road_Reduced.xyz") 