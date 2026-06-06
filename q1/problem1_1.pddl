;; ============================================================
;; D5-V1 Q1 -- Problem 1: STABLE CROPS (no cost optimization)
;;
;; All plots healthy. Goal: inspect and report every plot.
;; No metric -- the planner just finds ANY valid plan.
;; Robot starts at junction O2 (no base node).
;;
;; Layout:
;;        x1 -- x2 -- x3 -- x4
;;        |     |     |     |
;;        O1 -- O2 -- O3 -- O4
;;        |     |     |     |
;;        x5 -- x6 -- x7 -- x8
;; ============================================================

(define (problem crop-mon-q1-p1)
  (:domain crop-monitoring-q1)

  (:objects
    r1                              - robot
    x1 x2 x3 x4 x5 x6 x7 x8        - plot
    O1 O2 O3 O4                     - junction
    good normal critical            - condition
  )

  (:init
    ;; --- robot starts at junction O2 ---
    (robot-at r1 O2)

    ;; ============================================================
    ;; TOPOLOGY
    ;; ============================================================

    ;; --- HORIZONTAL: top row ---
    (connected x1 x2) (connected x2 x1)
    (connected x2 x3) (connected x3 x2)
    (connected x3 x4) (connected x4 x3)

    ;; --- HORIZONTAL: bottom row ---
    (connected x5 x6) (connected x6 x5)
    (connected x6 x7) (connected x7 x6)
    (connected x7 x8) (connected x8 x7)

    ;; --- VERTICAL: through junctions ---
    (connected x1 O1) (connected O1 x1)
    (connected O1 x5) (connected x5 O1)
    (connected x2 O2) (connected O2 x2)
    (connected O2 x6) (connected x6 O2)
    (connected x3 O3) (connected O3 x3)
    (connected O3 x7) (connected x7 O3)
    (connected x4 O4) (connected O4 x4)
    (connected O4 x8) (connected x8 O4)

    ;; --- HORIZONTAL: junction row ---
    (connected O1 O2) (connected O2 O1)
    (connected O2 O3) (connected O3 O2)
    (connected O3 O4) (connected O4 O3)

    ;; ============================================================
    ;; CROP STATE (all healthy / good)
    ;; ============================================================
    (crop-condition x1 good)
    (crop-condition x2 good)
    (crop-condition x3 good)
    (crop-condition x4 good)
    (crop-condition x5 good)
    (crop-condition x6 good)
    (crop-condition x7 good)
    (crop-condition x8 good)

    ;; --- cost starts at 0 (tracked but not optimized here) ---
    (= (total-cost) 0)
  )

  (:goal (and
    (reported x1) (reported x2) (reported x3) (reported x4)
    (reported x5) (reported x6) (reported x7) (reported x8)
  ))

  ;; NO metric -- any valid plan is accepted.
)
