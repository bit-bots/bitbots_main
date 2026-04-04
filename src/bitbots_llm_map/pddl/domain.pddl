; Domain-Header

(define (domain bitbots_normal_behavior) 
    (:requirements :strips :typing :durative-actions :fluents :adl)
)

(:types robot ball goal position)

(:predicates
    (at ?r - robot ?p - position)
    ()
)
