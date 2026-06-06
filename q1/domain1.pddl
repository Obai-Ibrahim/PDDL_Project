;; ============================================================
;; D5-V1 Q1 -- Agricultural Robotics: Crop Monitoring
;; Classical PDDL (no time). ENHSP-compatible.
;;
;; Simple structure:
;;   - move (cost +1), inspect, report  -- all instantaneous
;;   - report does NOT require the robot to be at the plot
;;     (only that the plot was inspected)
;;   - cost increases ONLY on move
;;   - no base node; robot starts at a junction
;;
;; Crop state is represented explicitly as a symbolic condition.
;; In Q1 the condition is static (classical PDDL has no time);
;; in Q2 it becomes a numeric level driven by a process.
;; ============================================================

(define (domain crop-monitoring-q1)

  (:requirements :typing :negative-preconditions :fluents :equality)

  (:types
    node           - object
    plot junction  - node
    robot          - object
    condition      - object   ; good / normal / critical
  )

  (:predicates
    (robot-at        ?r - robot ?n - node)
    (connected       ?n1 - node ?n2 - node)
    (crop-condition  ?p - plot ?c - condition)
    (inspected       ?p - plot)
    (reported        ?p - plot)
  )

  (:functions
    (total-cost)
  )
  ;; ----------------------------------------------------------
  ;; move: travel along an edge. Cost +1 (the ONLY cost source).
  ;; ----------------------------------------------------------
  (:action move
    :parameters (?r - robot ?from - node ?to - node)
    :precondition (and
      (robot-at ?r ?from)
      (connected ?from ?to)
    )
    :effect (and
      (not (robot-at ?r ?from))
      (robot-at ?r ?to)
      (increase (total-cost) 1)
    )
  )
  ;; ----------------------------------------------------------
  ;; inspect: observe the crop. Robot must be at the plot.
  ;; Passive: does not change the crop condition.
  ;; ----------------------------------------------------------
  (:action inspect
    :parameters (?r - robot ?p - plot)
    :precondition (and
      (robot-at ?r ?p)
      (not (inspected ?p))
    )
    :effect (and
      (inspected ?p)
    )
  )
  ;; ----------------------------------------------------------
  ;; report: log an inspection. Decoupled from robot position --
  ;; only requires that the plot was inspected. No cost.
  ;; ----------------------------------------------------------
  (:action report
    :parameters (?r - robot ?p - plot)
    :precondition (and
      (inspected ?p)
      (not (reported ?p))
    )
    :effect (and
      (reported ?p)
    )
  )

)
