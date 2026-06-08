;; ============================================================
;; D5-V1 Q2 -- Problem 1: TIMING IRRELEVANT
;;
;; All plots start at condition-level 90 with slow degradation
;; rate 2/sec. At this rate a plot takes 55 seconds to reach
;; -20. The robot can inspect all plots in ~20 seconds easily.
;; Any visiting order works -- timing does not affect feasibility.
;;
;; Layout:
;;        x1 -- x2 -- x3 -- x4
;;        |     |     |     |
;;        O1    O2    O3    O4
;;        |     |     |     |
;;        x5 -- x6 -- x7 -- x8
;;        |
;;       base
;; ============================================================

(define (problem crop-mon-q2-p1)
  (:domain crop-monitoring-q2)

  (:objects
    r1                                      - robot
    x1 x2 x3 x4 x5 x6 x7 x8            - plot
    O1 O2 O3 O4                             - junction
    good normal critical                    - condition
  )

  (:init

    ;; --- robot starts at base, idle ---
    (robot-at r1 O2)

    ;; --- move and report timers start at 0 ---
    (= (move-timer r1)   0)
    (= (report-timer r1) 0)

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
    (connected O1 O2) (connected O2 O1)
    (connected O2 O3) (connected O3 O2)
    (connected O3 O4) (connected O4 O3)

    ;; --- BASE ---

    ;; ============================================================
    ;; EDGE LENGTHS (all = 1)
    ;; ============================================================
    (= (edge-length x1 x2) 1) (= (edge-length x2 x1) 1)
    (= (edge-length x2 x3) 1) (= (edge-length x3 x2) 1)
    (= (edge-length x3 x4) 1) (= (edge-length x4 x3) 1)
    (= (edge-length x5 x6) 1) (= (edge-length x6 x5) 1)
    (= (edge-length x6 x7) 1) (= (edge-length x7 x6) 1)
    (= (edge-length x7 x8) 1) (= (edge-length x8 x7) 1)
    (= (edge-length x1 O1) 1) (= (edge-length O1 x1) 1)
    (= (edge-length O1 x5) 1) (= (edge-length x5 O1) 1)
    (= (edge-length x2 O2) 1) (= (edge-length O2 x2) 1)
    (= (edge-length O2 x6) 1) (= (edge-length x6 O2) 1)
    (= (edge-length x3 O3) 1) (= (edge-length O3 x3) 1)
    (= (edge-length O3 x7) 1) (= (edge-length x7 O3) 1)
    (= (edge-length x4 O4) 1) (= (edge-length O4 x4) 1)
    (= (edge-length O4 x8) 1) (= (edge-length x8 O4) 1)

    (= (edge-length O1 O2) 1) (= (edge-length O2 O1) 1)
    (= (edge-length O2 O3) 1) (= (edge-length O3 O2) 1)
    (= (edge-length O3 O4) 1) (= (edge-length O4 O3) 1)

    ;; ============================================================
    ;; CROP STATE + DEGRADATION
    ;; All plots start healthy (90) with slow rate (2/sec).
    ;; Time to failure = (90 - (-20)) / 2 = 55 seconds.
    ;; Robot can inspect all 8 plots in ~20 seconds -- safe.
    ;; ============================================================
    (crop-condition x1 good) (= (condition-level x1) 90) (= (degrade-rate x1) 2)
    (crop-condition x2 good) (= (condition-level x2) 90) (= (degrade-rate x2) 2)
    (crop-condition x3 good) (= (condition-level x3) 90) (= (degrade-rate x3) 2)
    (crop-condition x4 good) (= (condition-level x4) 90) (= (degrade-rate x4) 2)
    (crop-condition x5 good) (= (condition-level x5) 90) (= (degrade-rate x5) 2)
    (crop-condition x6 good) (= (condition-level x6) 90) (= (degrade-rate x6) 2)
    (crop-condition x7 good) (= (condition-level x7) 90) (= (degrade-rate x7) 2)
    (crop-condition x8 good) (= (condition-level x8) 90) (= (degrade-rate x8) 2)
  )

  (:goal (and
    (reported x1) (reported x2) (reported x3) (reported x4)
    (reported x5) (reported x6) (reported x7) (reported x8)
  ))

 ;;(:metric minimize (total-time))
)
