;; ============================================================
;; D5-V1 Q2 -- Problem 2: TIMING CRITICAL
;;
;; One plot has a critical condition label with slow degradation,
;; and one has fast degradation:
;;   x3: starts at 95, rate 2/sec  → condition labeled critical, not timing-urgent
;;   x8: starts at 70, rate 10/sec → fails in (70+20)/10 = 9.0s
;;
;; The robot starts at O2. Reaching x8 takes at least 3 moves
;; (O2→O3→O4→x8 = 3 moves = 3 seconds via move-timer).
;;
;; A NAIVE route that ignores x8's degradation rate will reach
;; it too late -- the robot MUST prioritize x8 by finding the
;; fastest path to it first.
;;
;; Other plots (x1, x2, x4, x5, x6, x7) start at 90 with
;; slow rate 2/sec -- safe.
;;
;; Layout:
;;        x1 -- x2 -- x3  -- x4
;;        |     |     |      |
;;        O1 -- O2 -- O3  -- O4
;;        |     |     |      |
;;        x5 -- x6 -- x7 -- x8*
;;        |
;;       base       (* = time-critical)
;; ============================================================

(define (problem crop-mon-q2-p2)
  (:domain crop-monitoring-q2)

  (:objects
    r1                                      - robot
    x1 x2 x3 x4 x5 x6 x7 x8            - plot
    O1 O2 O3 O4                             - junction
    good normal critical                    - condition
  )

  (:init

    ;; --- robot starts at O2, idle ---
    (robot-at r1 O2)
    (= (move-timer r1)   0)
    (= (report-timer r1) 0)

    ;; ============================================================
    ;; TOPOLOGY (identical to Problem 1)
    ;; ============================================================

    (connected x1 x2) (connected x2 x1)
    (connected x2 x3) (connected x3 x2)
    (connected x3 x4) (connected x4 x3)
    (connected x5 x6) (connected x6 x5)
    (connected x6 x7) (connected x7 x6)
    (connected x7 x8) (connected x8 x7)
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
    ;; ============================================================

    ;; --- SAFE plots: start high, degrade slowly ---
    (crop-condition x1 normal)   (= (condition-level x1) 90) (= (degrade-rate x1) 5)
    (crop-condition x2 good)   (= (condition-level x2) 90) (= (degrade-rate x2) 2)
    (crop-condition x4 normal)     (= (condition-level x4) 90) (= (degrade-rate x4) 4)
    (crop-condition x5 normal)   (= (condition-level x5) 90) (= (degrade-rate x5) 4)
    (crop-condition x6 good)     (= (condition-level x6) 90) (= (degrade-rate x6) 2)
    (crop-condition x7 normal)   (= (condition-level x7) 90) (= (degrade-rate x7) 4)
    (crop-condition x3 critical) (= (condition-level x3) 95) (= (degrade-rate x3) 10)
    (crop-condition x8 critical) (= (condition-level x8) 70) (= (degrade-rate x8) 10)
  )

  (:goal (and
    (reported x1) (reported x2) (reported x3) (reported x4)
    (reported x5) (reported x6) (reported x7) (reported x8)
  ))

 ;; (:metric minimize (total-time))
)

