;; Auto-generated. Do not edit!


(when (boundp 'gps_bagger::callResponse)
  (if (not (find-package "GPS_BAGGER"))
    (make-package "GPS_BAGGER"))
  (shadow 'callResponse (find-package "GPS_BAGGER")))
(unless (find-package "GPS_BAGGER::CALLRESPONSE")
  (make-package "GPS_BAGGER::CALLRESPONSE"))
(unless (find-package "GPS_BAGGER::CALLRESPONSEREQUEST")
  (make-package "GPS_BAGGER::CALLRESPONSEREQUEST"))
(unless (find-package "GPS_BAGGER::CALLRESPONSERESPONSE")
  (make-package "GPS_BAGGER::CALLRESPONSERESPONSE"))

(in-package "ROS")





(defclass gps_bagger::callResponseRequest
  :super ros::object
  :slots (_command ))

(defmethod gps_bagger::callResponseRequest
  (:init
   (&key
    ((:command __command) "")
    )
   (send-super :init)
   (setq _command (string __command))
   self)
  (:command
   (&optional __command)
   (if __command (setq _command __command)) _command)
  (:serialization-length
   ()
   (+
    ;; string _command
    4 (length _command)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _command
       (write-long (length _command) s) (princ _command s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _command
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _command (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass gps_bagger::callResponseResponse
  :super ros::object
  :slots (_response ))

(defmethod gps_bagger::callResponseResponse
  (:init
   (&key
    ((:response __response) "")
    )
   (send-super :init)
   (setq _response (string __response))
   self)
  (:response
   (&optional __response)
   (if __response (setq _response __response)) _response)
  (:serialization-length
   ()
   (+
    ;; string _response
    4 (length _response)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _response
       (write-long (length _response) s) (princ _response s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _response
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _response (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass gps_bagger::callResponse
  :super ros::object
  :slots ())

(setf (get gps_bagger::callResponse :md5sum-) "22c7c465d64c7e74c6ae22029c7ca150")
(setf (get gps_bagger::callResponse :datatype-) "gps_bagger/callResponse")
(setf (get gps_bagger::callResponse :request) gps_bagger::callResponseRequest)
(setf (get gps_bagger::callResponse :response) gps_bagger::callResponseResponse)

(defmethod gps_bagger::callResponseRequest
  (:response () (instance gps_bagger::callResponseResponse :init)))

(setf (get gps_bagger::callResponseRequest :md5sum-) "22c7c465d64c7e74c6ae22029c7ca150")
(setf (get gps_bagger::callResponseRequest :datatype-) "gps_bagger/callResponseRequest")
(setf (get gps_bagger::callResponseRequest :definition-)
      "# Request
string command
---
string response

")

(setf (get gps_bagger::callResponseResponse :md5sum-) "22c7c465d64c7e74c6ae22029c7ca150")
(setf (get gps_bagger::callResponseResponse :datatype-) "gps_bagger/callResponseResponse")
(setf (get gps_bagger::callResponseResponse :definition-)
      "# Request
string command
---
string response

")



(provide :gps_bagger/callResponse "22c7c465d64c7e74c6ae22029c7ca150")


