#!/usr/bin/env roseus

(require "package://jsk_2016_01_baxter_apc/euslisp/main-stow.l")
(apc-stow-init)

(defun shuffle ()
  (send *baxter* :fold-pose-back :rarm)
  (send *baxter* :fold-pose-back :larm)
  (setq avs (list (send *baxter* :angle-vector)))
  (send *baxter* :rarm :inverse-kinematics
        (make-coords :pos #f(750 -300 -250)
                     :rpy (list 0 0 pi/2))
        :use-gripper t)
  (send *baxter* :larm :inverse-kinematics
        (make-coords :pos #f(750 300 -250)
                     :rpy (list 0 0 -pi/2))
        :use-gripper t)
  (pushback (send *baxter* :angle-vector) avs)
  (send *baxter* :rarm :inverse-kinematics
        (make-coords :pos #f(750 -200 -250)
                     :rpy (list 0 0 pi/2))
        :use-gripper t)
  (send *baxter* :larm :inverse-kinematics
        (make-coords :pos #f(750 200 -250)
                     :rpy (list 0 0 -pi/2))
        :use-gripper t)
  (pushback (send *baxter* :angle-vector) avs)
  (send *ri* :angle-vector-sequence avs)
  (send *ri* :wait-interpolation)
  (dotimes (i 10)
    (setq direction (+ (* 2 (mod i 2)) -1))
    (send *baxter* :rarm :move-end-pos (float-vector 0 (* direction 200) 0) :world :rotation-axis :z)
    (send *baxter* :larm :move-end-pos (float-vector 0 (* direction 200) 0) :world :rotation-axis :z)
    (send *ri* :angle-vector (send *baxter* :angle-vector))
    (send *ri* :wait-interpolation))
  (send *ri* :angle-vector-sequence (reverse avs)))
