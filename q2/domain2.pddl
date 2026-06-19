;; ============================================================
;; D5-V1 Q2 -- Agricultural Robotics: Crop Monitoring (PDDL+)
;;
;; PDDL+ domain for ENHSP.
;; Time is modelled via processes and events only (as ENHSP don't support durative actions).
;; Key design:
;;   - condition-level degrades continuously via process
;;   - failure event fires when condition-level < 20 AND
;;     plot has NOT been reported 
;;   - move takes 1 second via move-timer process
;;   - report takes 0.2 seconds via report-timer process
;;   - goal: all plots reported, with no failures
;;
;; Crop state layers:
;;   (crop-condition ?p ?c) -- explicit symbolic state (good/normal/critical)
;;   (condition-level ?p)   -- numeric level, driven by degradation process
;;   (inspected ?p)         -- set by inspect action, stops failure eligibility
;;   (reported ?p)          -- set by report-complete event
;; ============================================================

(define (domain crop-monitoring-q2)

  (:requirements :typing :fluents
                 :time :continuous-effects)

  (:types
    node           - object
    plot junction  - node
    robot          - object
    condition      - object
  )

  (:predicates
    (robot-at        ?r - robot ?n - node)
    (connected       ?n1 - node ?n2 - node)
    (crop-condition  ?p - plot ?c - condition)
    (inspected       ?p - plot)
    (reported        ?p - plot)
    (failed          ?p - plot)

    ;; movement state
    (is-moving       ?r - robot)
    (move-dest       ?r - robot ?n - node)
    (move-src        ?r - robot ?n - node)

    ;; reporting state
    (is-reporting    ?r - robot)
    (report-target   ?r - robot ?p - plot)
  )

  (:functions
    (condition-level  ?p - plot)     ; continuous health level
    (degrade-rate     ?p - plot)     ; degradation rate per time unit
    (move-timer       ?r - robot)    ; counts up while moving
    (report-timer     ?r - robot)    ; counts up while reporting
    (edge-length      ?n1 - node ?n2 - node)
  )

  ;; ==========================================================
  ;; PROCESSES
  ;; ==========================================================
  ;; ----------------------------------------------------------
  ;; Crop degradation: condition-level decreases continuously
  ;; After inspection it is ok for the level to keep dropping
  ;; (failure event only fires on unreported plots).
  ;; ----------------------------------------------------------
  (:process degrade
    :parameters (?p - plot)
    :precondition (and
      (> (condition-level ?p) 0)
    )
    :effect (and
      (decrease (condition-level ?p) (* #t (degrade-rate ?p)))
    )
  )

  ;; ----------------------------------------------------------
  ;; Move timer: counts up while robot is in motion.
  ;; ----------------------------------------------------------
  (:process moving
    :parameters (?r - robot)
    :precondition (and
      (is-moving ?r)
    )
    :effect (and
      (increase (move-timer ?r) (* #t 1))
    )
  )

  ;; ----------------------------------------------------------
  ;; Report timer: counts up while robot is reporting.
  ;; ----------------------------------------------------------
  (:process reporting
    :parameters (?r - robot)
    :precondition (and
      (is-reporting ?r)
    )
    :effect (and
      (increase (report-timer ?r) (* #t 1))
    )
  )

  ;; ==========================================================
  ;; EVENTS
  ;; ==========================================================
  ;; ----------------------------------------------------------
  ;; Crop failure: fires when condition drops below -20 AND
  ;; the plot has NOT been reported.
  ;; Once (failed ?p) is set it is permanent -- the crop
  ;; cannot be reported, making the goal unreachable.
  ;; ----------------------------------------------------------
  (:event crop-fails
    :parameters (?p - plot)
    :precondition (and
      (<= (condition-level ?p) 20)
      (not (reported ?p))
      (not (failed ?p))
    )
    :effect (and
      (failed ?p)
    )
  )

  ;; ----------------------------------------------------------
  ;; Move complete: fires when move-timer reaches 1 second.
  ;; Teleports robot to destination, resets timer.
  ;; ----------------------------------------------------------
  (:event move-complete
    :parameters (?r - robot ?src - node ?dst - node)
    :precondition (and
      (is-moving ?r)
      (>= (move-timer ?r) 1)
      (move-src ?r ?src)
      (move-dest ?r ?dst)
    )
    :effect (and
      (not (is-moving ?r))
      (not (robot-at ?r ?src))
      (robot-at ?r ?dst)
      (not (move-src ?r ?src))
      (not (move-dest ?r ?dst))
      (assign (move-timer ?r) 0)
    )
  )

  ;; ----------------------------------------------------------
  ;; Report complete: fires when report-timer reaches 0.2 sec.
  ;; Sets reported, resets timer.
  ;; ----------------------------------------------------------
  (:event report-complete
    :parameters (?r - robot ?p - plot)
    :precondition (and
      (is-reporting ?r)
      (>= (report-timer ?r) 0.2)
      (report-target ?r ?p)
    )
    :effect (and
      (not (is-reporting ?r))
      (reported ?p)
      (not (report-target ?r ?p))
      (assign (report-timer ?r) 0)
    )
  )

  ;; ==========================================================
  ;; ACTIONS (all instantaneous)
  ;; ==========================================================

  ;; ----------------------------------------------------------
  ;; start-move: begins travel along an edge.
  ;; Robot must be idle (not moving, not reporting).
  ;; ----------------------------------------------------------
  (:action start-move
    :parameters (?r - robot ?from - node ?to - node)
    :precondition (and
      (robot-at ?r ?from)
      (connected ?from ?to)
      (not (is-moving ?r))
      (not (is-reporting ?r))
    )
    :effect (and
      (is-moving ?r)
      (move-src ?r ?from)
      (move-dest ?r ?to)
    )
  )

  ;; ----------------------------------------------------------
  ;; inspect: instantaneous observation of crop condition.
  ;; Robot must be present and plot must not have failed.
  ;; Sets (inspected ?p) -- stops failure eligibility.
  ;; ----------------------------------------------------------
  (:action inspect
    :parameters (?r - robot ?p - plot)
    :precondition (and
      (robot-at ?r ?p)
      (not (inspected ?p))
      (not (is-moving ?r))
      (not (is-reporting ?r))
    )
    :effect (and
      (inspected ?p)
    )
  )

  ;; ----------------------------------------------------------
  ;; start-report: begins the 0.2-second reporting process.
  ;; Plot must be inspected and not yet reported.
  ;; ----------------------------------------------------------
  (:action start-report
    :parameters (?r - robot ?p - plot)
    :precondition (and
      (inspected ?p)
      (not (failed ?p))
      (not (reported ?p))
      (not (is-moving ?r))
      (not (is-reporting ?r))
    )
    :effect (and
      (is-reporting ?r)
      (report-target ?r ?p)
    )
  )

)
