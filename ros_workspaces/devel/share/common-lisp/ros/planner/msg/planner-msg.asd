
(cl:in-package :asdf)

(defsystem "planner-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "GantryPath" :depends-on ("_package_GantryPath"))
    (:file "_package_GantryPath" :depends-on ("_package"))
  ))