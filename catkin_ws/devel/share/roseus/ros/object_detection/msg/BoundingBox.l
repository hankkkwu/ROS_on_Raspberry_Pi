;; Auto-generated. Do not edit!


(when (boundp 'object_detection::BoundingBox)
  (if (not (find-package "OBJECT_DETECTION"))
    (make-package "OBJECT_DETECTION"))
  (shadow 'BoundingBox (find-package "OBJECT_DETECTION")))
(unless (find-package "OBJECT_DETECTION::BOUNDINGBOX")
  (make-package "OBJECT_DETECTION::BOUNDINGBOX"))

(in-package "ROS")
;;//! \htmlinclude BoundingBox.msg.html


(defclass object_detection::BoundingBox
  :super ros::object
  :slots (_center_x _center_y _width _height _confidence _class_id ))

(defmethod object_detection::BoundingBox
  (:init
   (&key
    ((:center_x __center_x) 0.0)
    ((:center_y __center_y) 0.0)
    ((:width __width) 0.0)
    ((:height __height) 0.0)
    ((:confidence __confidence) 0.0)
    ((:class_id __class_id) 0.0)
    )
   (send-super :init)
   (setq _center_x (float __center_x))
   (setq _center_y (float __center_y))
   (setq _width (float __width))
   (setq _height (float __height))
   (setq _confidence (float __confidence))
   (setq _class_id (float __class_id))
   self)
  (:center_x
   (&optional __center_x)
   (if __center_x (setq _center_x __center_x)) _center_x)
  (:center_y
   (&optional __center_y)
   (if __center_y (setq _center_y __center_y)) _center_y)
  (:width
   (&optional __width)
   (if __width (setq _width __width)) _width)
  (:height
   (&optional __height)
   (if __height (setq _height __height)) _height)
  (:confidence
   (&optional __confidence)
   (if __confidence (setq _confidence __confidence)) _confidence)
  (:class_id
   (&optional __class_id)
   (if __class_id (setq _class_id __class_id)) _class_id)
  (:serialization-length
   ()
   (+
    ;; float32 _center_x
    4
    ;; float32 _center_y
    4
    ;; float32 _width
    4
    ;; float32 _height
    4
    ;; float32 _confidence
    4
    ;; float32 _class_id
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _center_x
       (sys::poke _center_x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _center_y
       (sys::poke _center_y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _width
       (sys::poke _width (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _height
       (sys::poke _height (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _confidence
       (sys::poke _confidence (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _class_id
       (sys::poke _class_id (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _center_x
     (setq _center_x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _center_y
     (setq _center_y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _width
     (setq _width (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _height
     (setq _height (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _confidence
     (setq _confidence (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _class_id
     (setq _class_id (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get object_detection::BoundingBox :md5sum-) "18ce2c7c2f15e548586297b928e28f2f")
(setf (get object_detection::BoundingBox :datatype-) "object_detection/BoundingBox")
(setf (get object_detection::BoundingBox :definition-)
      "float32 center_x
float32 center_y
float32 width
float32 height
float32 confidence
float32 class_id
")



(provide :object_detection/BoundingBox "18ce2c7c2f15e548586297b928e28f2f")


