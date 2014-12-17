#!/usr/bin/env roseus

(load "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2w-interface.l")

(hrp2w-init)
(objects *hrp2w*)

(ros::roseus-add-msgs "jsk_pcl_ros")
(ros::roseus "catch_tray")

(setq *rtorso-co* (send (car (send *hrp2w* :torso)) :copy-worldcoords))
(setq *ltorso-co* (send (car (send *hrp2w* :torso)) :copy-worldcoords))
;; トルソリンクの一番最初の世界座標系での座標をcopy

(defun my-reset nil
  (send *hrp2w* :reset-manip-pose)
  (send *ri* :angle-vector (send *hrp2w* :angle-vector) 5000)   
  (send *ri* :wait-interpolation)
  )

;;掴む前のポーズ.
(defun pre-pose nil
  (send *hrp2w* :angle-vector #f(0.0 30.0 0.0 40.0 -56.5395 -60.0223 49.4609 -112.253 -68.1949 -32.7942 69.9981 80.0 -45.0344 55.8813 -34.5312 -112.616 64.4653 34.6465 73.182 -80.0))
  (send *ri* :angle-vector (send *hrp2w* :angle-vector) 5000)
  (send *ri* :wait-interpolation)
  )

(defun recog-cb (msg)
  (print "start callback function")
  (setq *boxes* (send msg :boxes))
  (setq cnt 0)
  (setq *rmax* 0)
  (setq *rmax-vol* 0)
  (setq *lmax* 0)
  (setq *lmax-vol* 0)
  (dolist (box *boxes*) 
    (setq *vol* (* (send box :dimensions :x)
		   (send box :dimensions :y)
		   (send box :dimensions :z)))
    (if (> (send box :pose :position :x) 0) 
	(if (> *vol* *rmax-vol*)
	    (progn
	      (setq *rmax* cnt)
	      (setq *rmax-vol* *vol*)
	      )
	  )
      )
    (if (< (send box :pose :position :x) 0)
	(if (> *vol* *lmax-vol*)
	    (progn
	      (setq *lmax* cnt)
	      (setq *lmax-vol* *vol*)
	      )
	  )
      )
    (setq cnt (+ cnt 1))
    )
  (setq *r-obj* (format nil "/camera_remote/depth_registered/cluster_decomposer1output~0,2D" *rmax*))
  (setq *l-obj* (format nil "/camera_remote/depth_registered/cluster_decomposer1output~0,2D" *lmax*))
  (print "finished callback function")
  )
(ros::subscribe "/camera_remote/depth_registered/cluster_decomposer1/boxes" jsk_pcl_ros::BoundingBoxArray #'recog-cb)

(defun recog nil
  (setq *boxes* nil)
  (ros::spin-once)
  
  (setq *r-tfl* (instance ros::transform-listener :init))
  (setq *l-tfl* (instance ros::transform-listener :init))
  
  (when *boxes*
    (send *r-tfl* :wait-for-transform
	  "CHEST_LINK0" *r-obj* (ros::time 0) 3)
    (setq *r-target-co*
	  (send *r-tfl* :lookup-transform
		"CHEST_LINK0" *r-obj* (ros::time 0)))
    (send *l-tfl* :wait-for-transform
	  "CHEST_LINK0" *l-obj* (ros::time 0) 3)
    (setq *l-target-co*
	  (send *l-tfl* :lookup-transform
		"CHEST_LINK0" *l-obj* (ros::time 0)))    
    (setq *r-target* (send *rtorso-co* :transform *r-target-co*))
    (setq *l-target* (send *ltorso-co* :transform *l-target-co*))
    ;; トルソから計算し直すことでeus世界の世界座標系に変換
    (setq *rtray_pos* (send *hrp2w* :rarm :end-coords :copy-worldcoords))
    (setq *ltray_pos* (send *hrp2w* :larm :end-coords :copy-worldcoords))
    ;; 両手の手先の角度を動かしたくないので，一度今の角度を保持するため座標ごとコピー
    (send *rtray_pos* :locate (send *r-target* :pos) :world)
    (send *ltray_pos* :locate (send *l-target* :pos) :world)
    ;; コピーした右手座標の位置だけを認識したものにすげ替える
    (print "finish recognition")
    )
  (objects (list *hrp2w* *rtray_pos* *ltray_pos*))
  )

;;アームをトレイまで移動させる
(defun go-tray-pos nil
  ;;right arm
  (setq *rarm-end* (send *hrp2w* :rarm :end-coords :copy-worldcoords))
  (send *rarm-end* :locate (send *rtray_pos* :pos) :world)
  (send *rarm-end* :translate (float-vector -60 0 0))
  (send *hrp2w* :rarm :inverse-kinematics *rarm-end* :rotation-axis t :torso t)
  ;;left arm
  (setq *larm-end* (send *hrp2w* :larm :end-coords :copy-worldcoords))
  (send *larm-end* :locate (send *ltray_pos* :pos) :world)
  (send *larm-end* :translate (float-vector -60 0 0))
  (send *hrp2w* :larm :inverse-kinematics *larm-end* :rotation-axis t :torso t)
  (send *ri* :angle-vector (send *hrp2w* :angle-vector) 5000)   
  (send *ri* :wait-interpolation)
  )

(defun my-grasp nil
  (send *ri* :start-impedance :larm)
  (hrp2w-start-grasp :rarm :angle -8)
  (hrp2w-start-grasp :larm :angle  8)
  )

;;掴んだ後トレイを持ち上げる.
(defun lift-up-tray nil
  (setq *rarm-end2* (send *hrp2w* :rarm :end-coords :copy-worldcoords))
  (setq *larm-end2* (send *hrp2w* :larm :end-coords :copy-worldcoords))
  (send *rarm-end2* :translate (float-vector -250 0 -100))
  (send *larm-end2* :translate (float-vector -250 0 -100))
  (send *hrp2w* :torso :waist-y :joint-angle 0)
  (send *hrp2w* :rarm :inverse-kinematics *rarm-end2* :rotation-axis t :torso t)
  (send *hrp2w* :larm :inverse-kinematics *larm-end2* :rotation-axis t :torso t)
  (send *ri* :angle-vector (send *hrp2w* :angle-vector) 5000)   
  (send *ri* :wait-interpolation)
  )

(defun put-down nil
  (setq *rarm-end3* (send *hrp2w* :rarm :end-coords :copy-worldcoords))
  (setq *larm-end3* (send *hrp2w* :larm :end-coords :copy-worldcoords))
  (send *rarm-end3* :translate (float-vector 250 0 100))
  (send *larm-end3* :translate (float-vector 250 0 100))
  (send *hrp2w* :torso :waist-y :joint-angle 0)
  (send *hrp2w* :rarm :inverse-kinematics *rarm-end3* :rotation-axis t 	:torso t)
  (send *hrp2w* :larm :inverse-kinematics *larm-end3* :rotation-axis t :torso t)
  (send *ri* :angle-vector (send *hrp2w* :angle-vector) 5000)
  (send *ri* :wait-interpolation)
  )

(defun my-release nil
  (hrp2w-stop-grasp :rarm)
  (hrp2w-stop-grasp :larm)
  (send *ri* :stop-impedance :larm)
  )

(defun all_motion nil
  (my-reset)
  (pre-pose)
  (recog)
  (go-tray-pos)
  (my-grasp)
  (lift-up-tray)
  (put-down)
  (my-release)
  (pre-pose)
  (my-reset)
)
