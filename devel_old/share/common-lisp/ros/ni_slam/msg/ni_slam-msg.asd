
(cl:in-package :asdf)

(defsystem "ni_slam-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "MapInfo" :depends-on ("_package_MapInfo"))
    (:file "_package_MapInfo" :depends-on ("_package"))
  ))