;; Auto-generated. Do not edit!


(when (boundp 'planner::GantryPath)
  (if (not (find-package "PLANNER"))
    (make-package "PLANNER"))
  (shadow 'GantryPath (find-package "PLANNER")))
(unless (find-package "PLANNER::GANTRYPATH")
  (make-package "PLANNER::GANTRYPATH"))

(in-package "ROS")
;;//! \htmlinclude GantryPath.msg.html


(defclass planner::GantryPath
  :super ros::object
  :slots (_path ))

(defmethod planner::GantryPath
  (:init
   (&key
    ((:path __path) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _path __path)
   self)
  (:path
   (&optional __path)
   (if __path (setq _path __path)) _path)
  (:serialization-length
   ()
   (+
    ;; float64[] _path
    (* 8    (length _path)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64[] _path
     (write-long (length _path) s)
     (dotimes (i (length _path))
       (sys::poke (elt _path i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64[] _path
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _path (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _path i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;;
   self)
  )

(setf (get planner::GantryPath :md5sum-) "48ba10a81da6976bce0a9617ff2a025d")
(setf (get planner::GantryPath :datatype-) "planner/GantryPath")
(setf (get planner::GantryPath :definition-)
      "float64[] path
")



(provide :planner/GantryPath "48ba10a81da6976bce0a9617ff2a025d")


