;; Auto-generated. Do not edit!


(when (boundp 'object_detection::BoundingBoxes)
  (if (not (find-package "OBJECT_DETECTION"))
    (make-package "OBJECT_DETECTION"))
  (shadow 'BoundingBoxes (find-package "OBJECT_DETECTION")))
(unless (find-package "OBJECT_DETECTION::BOUNDINGBOXES")
  (make-package "OBJECT_DETECTION::BOUNDINGBOXES"))

(in-package "ROS")
;;//! \htmlinclude BoundingBoxes.msg.html


(defclass object_detection::BoundingBoxes
  :super ros::object
  :slots (_BoundingBoxes ))

(defmethod object_detection::BoundingBoxes
  (:init
   (&key
    ((:BoundingBoxes __BoundingBoxes) ())
    )
   (send-super :init)
   (setq _BoundingBoxes __BoundingBoxes)
   self)
  (:BoundingBoxes
   (&rest __BoundingBoxes)
   (if (keywordp (car __BoundingBoxes))
       (send* _BoundingBoxes __BoundingBoxes)
     (progn
       (if __BoundingBoxes (setq _BoundingBoxes (car __BoundingBoxes)))
       _BoundingBoxes)))
  (:serialization-length
   ()
   (+
    ;; object_detection/BoundingBox[] _BoundingBoxes
    (apply #'+ (send-all _BoundingBoxes :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; object_detection/BoundingBox[] _BoundingBoxes
     (write-long (length _BoundingBoxes) s)
     (dolist (elem _BoundingBoxes)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; object_detection/BoundingBox[] _BoundingBoxes
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _BoundingBoxes (let (r) (dotimes (i n) (push (instance object_detection::BoundingBox :init) r)) r))
     (dolist (elem- _BoundingBoxes)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get object_detection::BoundingBoxes :md5sum-) "a68dab7e3456d8b1363c13112d36861c")
(setf (get object_detection::BoundingBoxes :datatype-) "object_detection/BoundingBoxes")
(setf (get object_detection::BoundingBoxes :definition-)
      "BoundingBox[] BoundingBoxes

================================================================================
MSG: object_detection/BoundingBox
float32 center_x
float32 center_y
float32 width
float32 height
float32 confidence
float32 class_id
")



(provide :object_detection/BoundingBoxes "a68dab7e3456d8b1363c13112d36861c")


