; Domain-Header

(define (domain bitbots_normal_behavior) 
    (:requirements :strips :typing :durative-actions :fluents :adl)

    (:types robot ball goal position)

    ; world conditions
    (:predicates 
        (at ?r - robot ?p - position)

        (ball_at ?p - position)

        (has_ball ?r - robot)

        (free_kick)

        (positioned_for_kick ?r - robot)

        (clear_path ?from - position ?to - position)

        (ball_in_goal)
    )

  (:durative-action walk_to
    :parameters (?r - robot ?p - position)
    :duration (= ?duration 4)                    ; apprx. 4 sec
    :condition (and (at start (free_kick))
                    (at start (not (at ?r ?p))))
    :effect (and (at start (not (at ?r ?p)))
                 (at end (at ?r ?p)))
  )

  (:durative-action position_for_free_kick
    :parameters (?r - robot)
    :duration (= ?duration 2)
    :condition (and (at start (free_kick))
                    (at start (has_ball ?r))
                    (at start (not (positioned_for_kick ?r))))
    :effect (and (at end (positioned_for_kick ?r)))
  )

  (:durative-action kick_towards_goal
    :parameters (?r - robot ?target - position)
    :duration (= ?duration 1)
    :condition (and (at start (positioned_for_kick ?r))
                    (at start (has_ball ?r))
                    (at start (clear_path (ball_at ?target) ?target)))
    :effect (and (at start (not (has_ball ?r)))
                 (at end (ball_in_goal))
                 (at end (not (free_kick))))
  )

  (:durative-action wait_seconds
    :parameters (?r - robot ?seconds - number)
    :duration (= ?duration ?seconds)
    :condition ()
    :effect ()
  )
)
