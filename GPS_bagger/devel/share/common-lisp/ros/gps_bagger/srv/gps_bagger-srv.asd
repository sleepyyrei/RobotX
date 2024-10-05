
(cl:in-package :asdf)

(defsystem "gps_bagger-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :sensor_msgs-msg
)
  :components ((:file "_package")
    (:file "WaypointService" :depends-on ("_package_WaypointService"))
    (:file "_package_WaypointService" :depends-on ("_package"))
    (:file "callResponse" :depends-on ("_package_callResponse"))
    (:file "_package_callResponse" :depends-on ("_package"))
  ))