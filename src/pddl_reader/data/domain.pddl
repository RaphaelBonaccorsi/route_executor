(define (domain drone_waypoints)
  (:requirements :strips)

  ;; Types
  (:types 
    drone waypoint)

  ;; Predicates (State variables)
  (:predicates 
    (drone_at ?d - drone ?wp - waypoint)  ;; The drone is at a specific waypoint
    (connected ?wp1 - waypoint ?wp2 - waypoint)  ;; Two waypoints are connected
  )

  ;; Actions
  (:action move
    :parameters (?d - drone ?from - waypoint ?to - waypoint)
    :precondition (and (drone_at ?d ?from) (connected ?from ?to))
    :effect (and (not (drone_at ?d ?from)) (drone_at ?d ?to))
  )
)
