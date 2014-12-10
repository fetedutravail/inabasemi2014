#!/usr/bin/env roseus

(load "package://hrpsys_ros_bridge_tutorials/euslisp/hrp2w-interface.l")
;;(hrp2w-init)
;;vmaxも動かすか
(hrp2w-init :sim t)
(objects *hrp2w*)

(ros::roseus-add-msgs "jsk_pcl_ros")
(ros::roseus "catch_tray")

(setq *torso-co* (send (car (send *hrp2w* :torso)) :copy-worldcoords))

;; トルソリンクの一番最初の世界座標系での座標をコピー

;;当て決めで手を動かす.
(defun catch_tray nil
  (send *hrp2w* :angle-vector #f(0.0 30.0 0.0 40.0 -49.3179 -14.353 19.4521 -62.9683 -22.2948 0.53918 53.9622 60.0 -49.3179 14.353 -19.4521 -62.9683 22.2948 -0.53919 53.9622 -60.0))
  ;;(send *ri* :angle-vector (send *hrp2w* :angle-vector) 5000)
  )
;;シミュレーション用(catch_trayの位置にtray_posをおく)
(defun sim_pos nil
  (catch_tray)
  (setq *rtray_pos* (send *hrp2w* :rarm :end-coords :copy-worldcoords))  
  (setq *ltray_pos* (send *hrp2w* :larm :end-coords :copy-worldcoords))
  (send *hrp2w* :reset-manip-pose)
  )

(defun my-reset nil
  (send *hrp2w* :reset-manip-pose)
  (send *ri* :angle-vector (send *hrp2w* :angle-vector) 5000)   
  (send *ri* :wait-interpolation)
  )

;;台車がtrayに近づく関数
(defun approach nil
  (setq *boxes* nil)
  (ros::spin-once)
  (setq *r2-tfl* (instance ros::transform-listener :init))
  (setq *l2-tfl* (instance ros::transform-listener :init))
  
  (when *boxes
    (send *r2-tfl* :wait-for-transform
	  "CHEST_LINK0" *r-obj* (ros::time 0) 3)
    (setq *r2-target-co*
	  (send *tfl* :lookup-transform
		"CHEST_LINK0" *r-obj* (ros::time 0)))
    (send *l2-tfl* :wait-for-transform
	  "CHEST_LINK0" *l-obj* (ros::time 0) 3)
    (setq *l2-target-co*
	  (send *tfl* :lookup-transform
		"CHEST_LINK0" *l-obj* (ros::time 0)))
    (setq *r2-tray-pos* (send *torso-co* :transform *target-co*))
    (setq *l2-tray-pos* (send *torso-co* :transform *target-co*))
    (setq *ctray-pos* (midpoint 0.5 (send *r2-tray-pos* :pos) (send *l2-tray-pos* :pos)))
    (send *ctray-pos* (v+ *ctray-pos* (-700 0 0)))
    (objects (list *hrp2w* *ctray-pos*))
    ;;(send *vmax* :go-pos-relative-unsafe (send *ctray-pos* :pos :x) (send *ctray-pos* :pos :y) 0 10000)
    ;;(send *vmax* :wait-interpolation)
    )
  (objects (list *hrp2w* *r2-tray-pos* *l2-tray-pos*))
  )

;;掴む前のポーズ.
(defun pre_pose nil
  (send *hrp2w* :angle-vector #f(0.0 15.0 0.0 40.0 -56.5395 -60.0223 49.4609 -112.253 -68.1949 -32.7942 69.9981 80.0 -45.0344 55.8813 -34.5312 -112.616 64.4653 34.6465 73.182 -80.0))
  (send *ri* :angle-vector (send *hrp2w* :angle-vector) 5000)
  (send *ri* :wait-interpolation)
  )

(defun detect-cb (msg)
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
  (print "finished callback function"
  )
(ros::subscribe "/camera_remote/depth_registered/cluster_decomposer1/boxes" jsk_pcl_ros::BoundingBoxArray #'detect-cb)

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
    (setq *r-target* (send *torso-co* :transform *r-target-co*))
    (setq *l-target* (send *torso-co* :transform *l-target-co*))
    ;; トルソから計算し直すことでeus世界の世界座標系に変換
    (setq *rtray_pos* (send *hrp2w* :rarm :end-coords :copy-worldcoords))
    (setq *ltray_pos* (send *hrp2w* :larm :end-coords :copy-worldcoords))
    ;; 両手の手先の角度を動かしたくないので，一度今の角度を保持するため座標ごとコピー
    (send *rtray_pos* :locate (send *r-target* :pos) :world)
    (send *ltray_pos* :locate (send *r-target* :pos) :world)
    ;; コピーした右手座標の位置だけを認識したものにすげ替える
    (print "finish recognition")
    )
  (objects (list *hrp2w* *rtray_pos* *ltray_pos*))
  )

;;経由点のつもりで作ったらちょうどいい位置に移動したので移動させるときこちらを使う
(defun go-tray-pos nil
  ;;右手の経由
  (setq *rarm-end* (send *hrp2w* :rarm :end-coords :copy-worldcoords))
  (send *rarm-end* :locate (send *rtray_pos* :pos) :world)
  (send *rarm-end* :translate (float-vector -50 0 50))
  (send *hrp2w* :rarm :inverse-kinematics *rarm-end* :rotation-axis t :torso t)
  ;;左手の経由
  (setq *larm-end* (send *hrp2w* :larm :end-coords :copy-worldcoords))
  (send *larm-end* :locate (send *ltray_pos* :pos) :world)
  (send *larm-end* :translate (float-vector -50 0 50))
  (send *hrp2w* :larm :inverse-kinematics *larm-end* :rotation-axis t :torso t)
  (send *ri* :angle-vector (send *hrp2w* :angle-vector) 5000)   
  (send *ri* :wait-interpolation)
  )

(defun grasp nil
  (send *ri* :start-impedance :larm)
  (hrp2w-start-grasp :rarm -8)
  (hrp2w-start-grasp :larm 8)
  )

;;掴んだ後トレイを持ち上げる.
(defun up-tray nil
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

(defun release nil
  (hrp2w-stop-grasp :rarm)
  (hrp2w-stop-grasp :larm)
  (send *ri* :stop-impedance :larm)
  )

